/*
 * Copyright (c) 2018 Tomasz Król <tomicode@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>

#define NRF24_MAX_BUFFER 32

#define NRF24_REG_CONFIG 0x00
#define NRF24_REG_EN_AA 0x01
#define NRF24_REG_EN_RXADDR 0x02
#define NRF24_REG_SETUP_AW 0x03
#define NRF24_REG_SETUP_RETR 0x04
#define NRF24_REG_RF_CH 0x05
#define NRF24_REG_RF_SETUP 0x06
#define NRF24_REG_STATUS 0x07
#define NRF24_REG_OBSERVE_TX 0x08
#define NRF24_REG_RPD 0x09
#define NRF24_REG_RX_ADDR_P0 0x0A
#define NRF24_REG_RX_ADDR_P1 0x0B
#define NRF24_REG_RX_ADDR_P2 0x0C
#define NRF24_REG_RX_ADDR_P3 0x0D
#define NRF24_REG_RX_ADDR_P4 0x0E
#define NRF24_REG_RX_ADDR_P5 0x0F
#define NRF24_REG_TX_ADDR 0x10
#define NRF24_REG_RX_PW_P0 0x11
#define NRF24_REG_FIFO_STATUS 0x17

struct nrf24_radio {
    dev_t devt;
    uint8_t in_use;

    uint8_t status;
    uint8_t config;

    struct spi_device *spi;

    struct list_head device;

    struct gpio_desc *irq_gpiod;
    struct gpio_desc *ce_gpiod;
    struct gpio_desc *led_gpiod;

    uint8_t *mem_buf;
};

static DEFINE_MUTEX(nrf24_module_lock);
static LIST_HEAD(nrf24_devices);

static struct class *nrf24_class;

static int nrf24_major_num;

static unsigned int nrf24_minor_count;

static ssize_t nrf24_read(struct file *file, char __user *buf,
        size_t count, loff_t *offset)
{
    return 0;
}

static ssize_t nrf24_write(struct file *file, const char __user *buf,
        size_t count, loff_t *offset)
{
    return 0;
}

static long nrf24_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    return 0;
}

static int nrf24_open(struct inode *inode, struct file *file)
{
    struct nrf24_radio *rdev;
    int status = -ENXIO;

    mutex_lock(&nrf24_module_lock);

    list_for_each_entry(rdev, nrf24_devices, device) {
        if (rdev->devt == inode->i_rdev) {
            status = 0;
            break;
        }
    }

    if (status == 0 && rdev->in_use)
        status = -EBUSY;

    if (status < 0) {
        mutex_unlock(&nrf24_module_lock);
        return status;
    }

    rdev->mem_buf = kmalloc(sizeof(uint8_t) * NRF24_MAX_BUFFER, GFP_KERNEL);
    if (!rdev->mem_buf) {
        mutex_unlock(&nrf24_module_lock);
        return -ENOMEM;
    }

    rdev->in_use++;
    file->private_data = rdev;

    nonseekable_open(inode, file);
    mutex_unlock(&nrf24_module_lock);

    return 0;
}

static int nrf24_release(struct inode *inode, struct file *file)
{
    struct nrf24_radio *rdev;

    mutex_lock(&nrf24_module_lock);

    rdev = file->private_data;
    if (!rdev->in_use) {
        mutex_unlock(&nrf24_module_lock);
        printk(KERN_WANRNING "nrf24: Release of a unbound file node.");
        return -EINVAL;
    }

    if (rdev->mem_buf) {
        kfree(rdev->mem_buf);
        rdev->mem_buf = NULL;
    }
    rdev->in_use--;

    mutex_unlock(&nrf24_module_lock);
    return 0;
}

static const struct file_operations nrf24_fops = {
    .owner          = THIS_MODULE,
    .read           = nrf24_read,
    .write          = nrf24_write,
    .open           = nrf24_open,
    .release        = nrf24_release,
    .unlocked_ioctl = nrf24_ioctl,
};

static int nrf24_probe(struct spi_device *spi)
{
    struct nrf24_radio *rdev;

    printk(KERN_DEBUG "Request nrf24_radio creation. Device probe.\n");
    rdev = kmalloc(sizeof(*rdev), GFP_KERNEL);
    if (!rdev)
        return -ENOMEM;

    LIST_HEAD_INIT(&rdev->device);
    rdev->spi = spi;
    rdev->devt = MKDEV(nrf24_major_num, nrf24_minor_count);

    mutex_lock(&nrf24_module_lock);

    /* nRF24 gpio */
    rdev->led_gpiod = gpiod_get_optional(&spi->dev, "led", GPIOD_OUT_LOW);
    rdev->irq_gpiod = gpiod_get(&spi->dev, "interrupt", GPIOD_IN);
    rdev->ce_gpiod  = gpiod_get(&spi->dev, "ce", GPIOD_OUT_LOW);

    if (IS_ERR(rdev->ce_gpiod) || IS_ERR(rdev->irq_gpiod)) {
        if (rdev->led_gpiod)
            gpiod_put(rdev->led_gpiod);

        if (!IS_ERR(rdev->ce_gpiod))
            gpiod_put(rdev->ce_gpiod);

        if (!IS_ERR(rdev->irq_gpiod))
            gpiod_put(rdev->irq_gpiod);

        mutex_unlock(&nrf24_module_lock);
        kfree(rdev);
        return -EINVAL;
    }

    if (rdev->spi) {
        struct device *dev;
        ssize_t status;

        dev = device_create(nrf24_class, &spi->dev, rdev->devt, rdev,
                "radio-%d", nrf24_minor_count++);

        if (PTR_ERR_OR_ZERO(dev)) {
            if (rdev->led_gpiod)
                gpiod_put(rdev->led_gpiod);
            gpiod_put(rdev->ce_gpiod);
            gpiod_put(rdev->irq_gpiod);

            mutex_unlock(&nrf24_module_lock);
            kfree(rdev);
            return -ENODEV;
        }

        status = spi_w8r8(spi, NRF24_REG_STATUS);
        if (status < 0)
            printk(KERN_WARNING "Unable to query STATUS register.\n");
        rdev->status = (uint8_t)status;

        status = spi_w8r8(spi, NRF24_REG_CONFIG);
        if (status < 0)
            printk(KERN_WARNING "Unable to query CONFIG register.\n");
        rdev->config = (uint8_t)status;
    }

    if (rdev->status != 0x07)
        printk(KERN_WARNING "Status register unexpected value, got: %02x.\n", rdev->status);

    if ((rdev->config & 0x02) == 0) {
        uint16_t config = 0x20;

        // Set PWR_UP bit in config register
        config = config | (rdev->config << 8);

        if (spi_write(spi, &config, sizeof(uint16_t)) == 0)
            rdev->config = (config >> 8);
    }

    printk(KERN_INFO "spi radio device: config: %02x, status: %02x\n",
            rdev->config, rdev->status);

    spi_set_drvdata(spi, rdev);
    mutex_unlock(&nrf24_module_lock);

    return 0;
}

static int nrf24_remove(struct spi_device *spi)
{
    struct nrf24_radio *rdev = spi_get_drvdata(spi);

    printk(KERN_DEBUG "Destroy nrf24_radio structure. Device remove.\n");

    if (rdev->config & 0x02) {
        rdev->config &= ~0x02;
        if (spi_write(spi, &rdev->config, sizeof(rdev->config)) < 0)
            printk(KERN_WARNING "Unable to shutdown radio module!\n");
    }

    if (rdev->led_gpiod)
        gpiod_put(rdev->led_gpiod);
    gpiod_put(rdev->irq_gpiod);
    gpiod_put(rdev->ce_gpiod);

    rdev->spi = NULL;

    device_destroy(nrf24_class, rdev->devt);
    kfree(rdev);

    return 0;
}

static void nrf24_shutdown(struct spi_device *spi)
{
    printk(KERN_DEBUG "Shutdown device called.\n");
}

static const struct of_device_id nrf24_of_ids[] = {
    { .compatible = "nordicsemi,nrf24l01p" },
    {},
};
MODULE_DEVICE_TABLE(of, nrf24_of_ids);

static struct spi_driver nrf24_spi_driver = {
    .driver = {
        .name = "nrf",
        .of_match_table = nrf24_of_ids,
    },
    .probe = nrf24_probe,
    .remove = nrf24_remove,
    .shutdown = nrf24_shutdown,
};

static int __init nrf24_init(void)
{
    int status;

    printk(KERN_INFO "Initialize nRF24 module.\n");

    status = register_chrdev(0, "nrf24", &nrf24_fops);
    if (status < 0)
        return status;

    nrf24_major_num = status;

    nrf24_class = class_create(THIS_MODULE, "nrf24radio");
    if (IS_ERR(nrf24_class)) {
        printk(KERN_WARNING "Unable to register nRF24 radio class.\n");
        unregister_chrdev(nrf24_major_num, "nrf24");
        return -EINVAL;
    }

    status = spi_register_driver(&nrf24_spi_driver);
    if (status < 0) {
        printk(KERN_WARNING "Unable to register nRF24 SPI Driver.\n");
        class_destroy(nrf24_class);
        unregister_chrdev(nrf24_major_num, "nrf24");
    }

    return status;
}

static void __exit nrf24_exit(void)
{
    printk(KERN_INFO "Unloading module.. Bye.\n");

    spi_unregister_driver(&nrf24_spi_driver);
    class_destroy(nrf24_class);
    unregister_chrdev(nrf24_major_num, "nrf24");
}

module_init(nrf24_init);
module_exit(nrf24_exit);

MODULE_DESCRIPTION("nRF24L01 radio module driver");
MODULE_AUTHOR("Tomasz Król <tomicode@gmail.com>");
MODULE_LICENSE("GPL");
