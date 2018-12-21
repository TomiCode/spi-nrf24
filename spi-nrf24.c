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
#include <linux/delay.h>
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
#define NRF24_REG_RX_PW_P1 0x12
#define NRF24_REG_FIFO_STATUS 0x17

struct nrf24_radio {
    dev_t devt;
    uint8_t in_use;

    uint8_t config;
    uint8_t status;
    uint8_t fifo_status;

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

static void nrf24_register_dump(struct spi_device *spi)
{
    uint8_t status, config;
    uint8_t addr[5] = {0};
    uint8_t s_ch1, s_ch2;

    config = spi_w8r8(spi, NRF24_REG_CONFIG);
    status = spi_w8r8(spi, NRF24_REG_STATUS);
    printk(KERN_DEBUG "nRF24 registers\nCONFIG: %02x, STATUS: %02x\n",
        config, status);

    status = spi_w8r8(spi, NRF24_REG_RF_CH);
    config = spi_w8r8(spi, NRF24_REG_RF_SETUP);
    printk(KERN_DEBUG "RF_SETUP: %02x, RF_CH: %02x\n", config, status);

    s_ch1 = spi_w8r8(spi, NRF24_REG_RX_PW_P0);
    s_ch2 = spi_w8r8(spi, NRF24_REG_RX_PW_P1);
    printk(KERN_DEBUG "RX_PW_P0: %02x, RX_PW_P1: %02x\n", s_ch1, s_ch2);

    config = NRF24_REG_RX_ADDR_P0;
    spi_write_then_read(spi, &config, 1, addr, 5);
    printk(KERN_DEBUG "RX_ADDR_P0: %02x %02x %02x %02x %02x", addr[0],
        addr[1], addr[2], addr[3], addr[4]);

    config = NRF24_REG_RX_ADDR_P1;
    spi_write_then_read(spi, &config, 1, addr, 5);
    printk(KERN_DEBUG "RX_ADDR_P1: %02x %02x %02x %02x %02x", addr[0],
        addr[1], addr[2], addr[3], addr[4]);

    config = NRF24_REG_TX_ADDR;
    spi_write_then_read(spi, &config, 1, addr, 5);
    printk(KERN_DEBUG "TX_ADDR: %02x %02x %02x %02x %02x", addr[0],
        addr[1], addr[2], addr[3], addr[4]);
}

static int nrf24_write_reg(struct spi_device *spi, uint8_t addr, uint8_t val)
{
    uint16_t spi_buf;
    struct spi_transfer t = {
        .tx_buf = &spi_buf,
        .len    = sizeof(uint16_t),
    };

    spi_buf = 0x20 | (addr & 0x1f) | (val << 8);
    return spi_sync_transfer(spi, &t, 1);
}

static int nrf24_read_reg(struct spi_device *spi, uint8_t addr, uint8_t *val)
{
    struct spi_transfer t[2] = {
        { .tx_buf = &addr,  .len = sizeof(uint8_t) },
        { .rx_buf = val,    .len = sizeof(uint8_t) },
    };

    return spi_sync_transfer(spi, t, 2);
}

static int nrf24_write_then_read_reg(struct spi_device *spi, uint8_t addr, uint8_t *val)
{
    uint16_t spi_buf;
    int ret;

    struct spi_transfer t[2] = {
        { .tx_buf = &spi_buf,   .len = sizeof(uint16_t) },
        { .rx_buf = val,        .len = sizeof(uint8_t) },
    };

    spi_buf = 0x20 | (addr & 0x1f) | (*val << 8);
    ret = spi_sync_transfer(spi, t, 1);
    if (ret < 0)
        return ret;

    t[0].tx_buf = &addr;
    t[0].len    = sizeof(uint8_t);
    return spi_sync_transfer(spi, t, 2);
}

static inline int nrf24_query_status(struct nrf24_radio *rdev)
{
    int ret;

    ret = nrf24_read_reg(rdev->spi, NRF24_REG_STATUS, &rdev->status);
    if (ret < 0)
        return ret;

    return nrf24_read_reg(rdev->spi, NRF24_REG_FIFO_STATUS, &rdev->fifo_status);
}


static int nrf24_cmd(struct spi_device *spi, uint8_t cmd)
{
    struct spi_transfer t = {
        .tx_buf = &cmd,
        .len    = sizeof(uint8_t),
    };

    return spi_sync_transfer(spi, &t, 1);
}

static int nrf24_disable_rx_mode(struct nrf24_radio *rdev)
{
    if (rdev->status & 0x01) {
        int ret;

        rdev->status &= ~0x01;
        ret = nrf24_write_then_read_reg(rdev->spi, NRF24_REG_CONFIG, &rdev->status);
        if (ret < 0)
            return ret;
    }

    gpiod_set_value(rdev->ce_gpiod, 0);
    return 0;
}

static int nrf24_rx_mode(struct nrf24_radio *rdev)
{
    if ((rdev->status & 0x01) == 0) {
        int ret;

        rdev->status |= 0x01;
        ret = nrf24_write_then_read_reg(rdev->spi, NRF24_REG_CONFIG, &rdev->status);
        if (ret < 0)
            return ret;
    }

    gpiod_set_value(rdev->ce_gpiod, 1);
    udelay(130);

    return 0;
}

static ssize_t nrf24_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    struct nrf24_radio *rdev = file->private_data;
    size_t data_len;
    int status;

    printk(KERN_DEBUG "Read from nrf24 device, count: %d, offset: %lld.\n",
            count, *offset);

    nrf24_register_dump(rdev->spi);

    if (*offset)
        return -EINVAL;

    if ((rdev->config & 0x01) == 0) {
        printk(KERN_WARNING "nrf24: device not in rx mode.\n");
        return -EIO;
    }

    status = spi_w8r8(rdev->spi, NRF24_REG_FIFO_STATUS);
    if (status < 0) {
        printk(KERN_WARNING "nrf24: unable to query FIFO_STATUS register.\n");
        return status;
    }

    printk(KERN_DEBUG "nrf24: FIFO_STATUS: %02x\n", status);
    if (status & 0x01)
        return -ENODATA;

    status = 0x61;
    if (!spi_write_then_read(rdev->spi, &status, sizeof(uint8_t),
                rdev->mem_buf, NRF24_MAX_BUFFER)) {
        printk(KERN_WARNING "nrf24: spi io operation failed.\n");
        return -EIO;
    }

    data_len = min_t(size_t, count, NRF24_MAX_BUFFER);
    copy_to_user(buf, rdev->mem_buf, data_len);

    return 0;
}

static ssize_t nrf24_write(struct file *file, const char __user *buf, size_t count,
        loff_t *offset)
{
    struct nrf24_radio *rdev = file->private_data;
    int rx_mode = 0;
    int ret;

    printk(KERN_DEBUG "Write to nrf24 device, count: %d, offset: %lld.\n",
            count, *offset);

    if (count > NRF24_MAX_BUFFER || *offset)
        return -EINVAL;

    if (rdev->config & 0x01) {
        rx_mode = 1;
        ret = nrf24_disable_rx_mode(rdev);
        if (ret < 0)
            return ret;
    }

    ret = nrf24_query_status(rdev);
    if (ret < 0)
        return ret;

    if (rdev->fifo_status & 0x20 || rdev->config & 0x01)
        return -ENOSPC;

    *(rdev->mem_buf) = 0xA0;
    copy_from_user(rdev->mem_buf + 1, buf, count);

    ret = spi_write(rdev->spi, rdev->mem_buf, count + 1);
    if (ret < 0)
        return ret;

    gpiod_set_value(rdev->ce_gpiod, 1);
    udelay(12);
    gpiod_set_value(rdev->ce_gpiod, 0);

    if (rx_mode) {
        ret = nrf24_rx_mode(rdev);
        if (ret < 0) {
            printk(KERN_WARNING "nrf24: unable to return to rx mode.\n");
            return ret;
        }
    }

    nrf24_register_dump(rdev->spi);
    return count;
}

static long nrf24_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct nrf24_radio *rdev = file->private_data;
    (void)rdev;
    printk(KERN_DEBUG "nrf24: ioctl command: %u, arg: %02lx.\n", cmd, arg);

    return 0;
}

/*
 * Reset configuration register within the module,
 * check rx/tx fifo status and (if neccessary) clear the fifos.
 * Enable module (PWR_UP) and set to PRIM_RX mode.
 */
static int nrf24_device_setup(struct nrf24_radio *rdev)
{
    int ret;

    rdev->config = 0x08;
    ret = nrf24_write_then_read_reg(rdev->spi, NRF24_REG_CONFIG, &rdev->config);
    if (ret < 0)
        return ret;

    ret = nrf24_query_status(rdev);
    if (ret < 0)
        return ret;

    if (rdev->status & 0x10) {
        if (rdev->fifo_status != 0x11) {
            nrf24_cmd(rdev->spi, 0xe1);
            nrf24_cmd(rdev->spi, 0xe2);
        }

        nrf24_write_reg(rdev->spi, NRF24_REG_STATUS, rdev->status & ~0x10);
    }

    ret = nrf24_query_status(rdev);
    if (ret < 0)
        return ret;

    // MASK_TX_DS | MASK_MAX_RT | PWR_UP
    rdev->config |= 0x32;
    return nrf24_write_then_read_reg(rdev->spi, NRF24_REG_CONFIG, &rdev->config);
}

static int nrf24_open(struct inode *inode, struct file *file)
{
    struct nrf24_radio *rdev;
    int ret = -ENXIO;

    printk(KERN_DEBUG "nrf24: device open.\n");
    mutex_lock(&nrf24_module_lock);

    list_for_each_entry(rdev, &nrf24_devices, device) {
        if (rdev->devt == inode->i_rdev) {
            ret = 0;
            break;
        }
    }

    if (ret == 0 && rdev->in_use)
        ret = -EBUSY;

    if (ret < 0) {
        mutex_unlock(&nrf24_module_lock);
        return ret;
    }

    rdev->mem_buf = kmalloc(sizeof(uint8_t) * (NRF24_MAX_BUFFER + 1), GFP_KERNEL);
    if (!rdev->mem_buf) {
        mutex_unlock(&nrf24_module_lock);
        return -ENOMEM;
    }

    ret = nrf24_rx_mode(rdev);
    if (ret < 0) {
        kfree(rdev->mem_buf);
        mutex_unlock(&nrf24_module_lock);
        return ret;
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

    printk(KERN_DEBUG "Release a nrf24 device.\n");
    mutex_lock(&nrf24_module_lock);

    rdev = file->private_data;
    if (!rdev->in_use) {
        mutex_unlock(&nrf24_module_lock);
        printk(KERN_WARNING "nrf24: Release of a unbound file node.");
        return -EINVAL;
    }

    if (rdev->mem_buf) {
        kfree(rdev->mem_buf);
        rdev->mem_buf = NULL;
    }
    if (rdev->led_gpiod)
        gpiod_set_value(rdev->led_gpiod, 0);
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

static inline void nrf24_gpio_unregister(nrf24_radio *rdev)
{
    if (rdev->led_gpiod) {
        gpiod_set_value(rdev->led_gpiod, 0);
        gpiod_put(rdev->led_gpiod);
    }
    gpiod_set_value(rdev->ce_gpiod, 0);
    gpiod_put(rdev->ce_gpiod);
    gpiod_put(rdev->irq_gpiod);
}

static int nrf24_probe(struct spi_device *spi)
{
    struct nrf24_radio *rdev;
    int ret;

    printk(KERN_DEBUG "Request nrf24_radio creation. Device probe.\n");
    mutex_lock(&nrf24_module_lock);

    rdev = kzalloc(sizeof(*rdev), GFP_KERNEL);
    if (!rdev)
        return -ENOMEM;

    INIT_LIST_HEAD(&rdev->device);
    rdev->spi = spi;
    rdev->devt = MKDEV(nrf24_major_num, nrf24_minor_count);

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

        kfree(rdev);
        mutex_unlock(&nrf24_module_lock);
        return -EINVAL;
    }

    if (IS_ERR(device_create(nrf24_class, &spi->dev, rdev->devt, rdev,
                    "radio-%d", nrf24_minor_count++))) {
        nrf24_gpio_unregister(rdev);
        kfree(rdev);
        mutex_unlock(&nrf24_module_lock);
        return -ENODEV;
    }

    ret = nrf24_device_setup(rdev);
    if (ret < 0) {
        nrf24_gpio_unregister(rdev);
        kfree(rdev);
        mutex_unlock(&nrf24_module_lock);
        return ret;
    }

    list_add(&rdev->device, &nrf24_devices);
    spi_set_drvdata(spi, rdev);
    mutex_unlock(&nrf24_module_lock);

    return 0;
}

static int nrf24_remove(struct spi_device *spi)
{
    struct nrf24_radio *rdev = spi_get_drvdata(spi);

    printk(KERN_WARNING "Destroy nrf24_radio structure. Device remove.\n");

    if (rdev->led_gpiod)
        gpiod_put(rdev->led_gpiod);
    gpiod_put(rdev->irq_gpiod);
    gpiod_put(rdev->ce_gpiod);

    rdev->spi = NULL;

    list_del(&rdev->device);
    device_destroy(nrf24_class, rdev->devt);
    kfree(rdev);

    return 0;
}

static void nrf24_shutdown(struct spi_device *spi)
{
    printk(KERN_WARNING "Shutdown device called.\n");
}

static const struct of_device_id nrf24_of_ids[] = {
    { .compatible = "nordicsemi,nrf24l01p" },
    { },
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
