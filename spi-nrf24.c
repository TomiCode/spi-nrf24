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
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>


/*
 * Module internal register addresses.
 */
#define NRF24_CONFIG        0x00
#define NRF24_EN_AA         0x01
#define NRF24_EN_RXADDR     0x02
#define NRF24_SETUP_AW      0x03
#define NRF24_SETUP_RETR    0x04
#define NRF24_RF_CH         0x05
#define NRF24_RF_SETUP      0x06
#define NRF24_STATUS        0x07
#define NRF24_RX_ADDR_P0    0x0A
#define NRF24_RX_ADDR_P1    0x0B
#define NRF24_RX_ADDR_P2    0x0C
#define NRF24_RX_ADDR_P3    0x0D
#define NRF24_RX_ADDR_P4    0x0E
#define NRF24_RX_ADDR_P5    0x0F
#define NRF24_TX_ADDR       0x10
#define NRF24_RX_PW_P0      0x11
#define NRF24_RX_PW_P1      0x12
#define NRF24_FIFO_STATUS   0x17
#define NRF24_DYNPD         0x1C
#define NRF24_FEATURE       0x1D

/*
 * Config register bits.
 */
#define NRF24_PRIM_RX       BIT(0)
#define NRF24_PWR_UP        BIT(1)
#define NRF24_CRCO          BIT(2)
#define NRF24_EN_CRC        BIT(3)
#define NRF24_MASK_MAX_RT   BIT(4)
#define NRF24_MASK_TX_DS    BIT(5)
#define NRF24_MASK_RX_DR    BIT(6)

/*
 * Status register important bits.
 */
#define NRF24_MAX_RT    BIT(4)
#define NRF24_TX_DS     BIT(5)
#define NRF24_RX_DR     BIT(6)

/*
 * SPI Commands.
 */
#define NRF24_R_RX_PAYLOAD  0x61
#define NRF24_W_TX_PAYLOAD  0xA0
#define NRF24_FLUSH_TX      0xE1
#define NRF24_FLUSH_RX      0xE2

/*
 * Device driver constraints.
 */
#define NRF24_BUFFER_SIZE 32U
#define NRF24_MAX_NODES 8

/*
 * Device ioctl commands.
 */
#define NRF24_IOC_SET_ADDR 0x01
#define NRF24_IOC_SET_RFCH 0x02

/*
 * Device driver internal rx/tx buffer with the related command.
 */
struct nrf24_dev_buf {
    uint8_t cmd;
    uint8_t msg[NRF24_BUFFER_SIZE];
} __attribute__((packed));

/*
 * Device required gpio descriptors.
 */
struct nrf24_dev_gpios {
    struct gpio_desc *led;
    struct gpio_desc *irq;
    struct gpio_desc *ce;
};

struct nrf24_dev {
    uint8_t attrs;
    uint8_t node_id;
    spinlock_t bus_lock;
    unsigned int irq;

    struct spi_device *spi;
    struct list_head device;

    struct nrf24_dev_gpios *gpio;
    struct nrf24_dev_buf *buf;
};

static DEFINE_MUTEX(nrf24_module_lock);
static DECLARE_WAIT_QUEUE_HEAD(nrf24_event_queue);

static DECLARE_BITMAP(nrf24_nodes, NRF24_MAX_NODES);
static LIST_HEAD(nrf24_devices);

static struct class *nrf24_class;
static int nrf24_major;

static void nrf24_register_dump(struct spi_device *spi)
{
    uint8_t status, config;
    uint8_t addr[5] = {0};
    uint8_t s_ch1, s_ch2;

    config = spi_w8r8(spi, NRF24_CONFIG);
    status = spi_w8r8(spi, NRF24_STATUS);
    printk(KERN_DEBUG "nRF24 registers\nCONFIG: %02x, STATUS: %02x\n",
        config, status);

    status = spi_w8r8(spi, NRF24_RF_CH);
    config = spi_w8r8(spi, NRF24_RF_SETUP);
    printk(KERN_DEBUG "RF_SETUP: %02x, RF_CH: %02x\n", config, status);

    s_ch1 = spi_w8r8(spi, NRF24_RX_PW_P0);
    s_ch2 = spi_w8r8(spi, NRF24_RX_PW_P1);
    printk(KERN_DEBUG "RX_PW_P0: %02x, RX_PW_P1: %02x\n", s_ch1, s_ch2);

    config = NRF24_RX_ADDR_P0;
    spi_write_then_read(spi, &config, 1, addr, 5);
    printk(KERN_DEBUG "RX_ADDR_P0: %02x %02x %02x %02x %02x", addr[0],
        addr[1], addr[2], addr[3], addr[4]);

    config = NRF24_RX_ADDR_P1;
    spi_write_then_read(spi, &config, 1, addr, 5);
    printk(KERN_DEBUG "RX_ADDR_P1: %02x %02x %02x %02x %02x", addr[0],
        addr[1], addr[2], addr[3], addr[4]);

    config = NRF24_TX_ADDR;
    spi_write_then_read(spi, &config, 1, addr, 5);
    printk(KERN_DEBUG "TX_ADDR: %02x %02x %02x %02x %02x", addr[0],
        addr[1], addr[2], addr[3], addr[4]);
}

static int nrf24_write_reg(struct nrf24_dev *rdev, uint8_t addr, uint8_t val)
{
    uint16_t spi_buf;
    struct spi_transfer t = {
        .tx_buf = &spi_buf,
        .len    = sizeof(uint16_t),
    };

    spi_buf = 0x20 | (addr & 0x1f) | (val << 8);
    return spi_sync_transfer(rdev->spi, &t, 1);
}

static int nrf24_read_reg(struct nrf24_dev *rdev, uint8_t addr, uint8_t *val)
{
    struct spi_transfer t[2] = {
        { .tx_buf = &addr,  .len = sizeof(uint8_t) },
        { .rx_buf = val,    .len = sizeof(uint8_t) },
    };

    return spi_sync_transfer(rdev->spi, t, 2);
}

static int nrf24_write_then_read_reg(struct nrf24_dev *rdev, uint8_t addr, uint8_t *val)
{
    uint16_t spi_buf;
    int ret;

    struct spi_transfer t[2] = {
        { .tx_buf = &spi_buf,   .len = sizeof(uint16_t) },
        { .rx_buf = val,        .len = sizeof(uint8_t) },
    };

    spi_buf = 0x20 | (addr & 0x1f) | (*val << 8);
    ret = spi_sync_transfer(rdev->spi, t, 1);
    if (ret < 0)
        return ret;

    t[0].tx_buf = &addr;
    t[0].len    = sizeof(uint8_t);
    return spi_sync_transfer(rdev->spi, t, 2);
}

static int nrf24_test_bit_reg(struct nrf24_dev *rdev, uint8_t addr, uint8_t bits, uint8_t *reg)
{
    uint8_t r_value;
    int ret;

    struct spi_transfer t[2] = {
        { .tx_buf = &addr,      .len = sizeof(uint8_t) },
        { .rx_buf = &r_value,   .len = sizeof(uint8_t) },
    };

    ret = spi_sync_transfer(rdev->spi, t, 2);
    if (ret < 0)
        return 0;

    if (reg)
        *reg = r_value;

    if (r_value & bits)
        return 1;

    return 0;
}

static int nrf24_cmd(struct nrf24_dev *rdev, uint8_t cmd)
{
    struct spi_transfer t = {
        .tx_buf = &cmd,
        .len    = sizeof(uint8_t),
    };

    return spi_sync_transfer(rdev->spi, &t, 1);
}

static int nrf24_flush(struct file *file, fl_owner_t id)
{
    struct nrf24_dev *rdev = file->private_data;
    int ret;

    ret = nrf24_cmd(rdev, NRF24_FLUSH_TX);
    if (ret < 0)
        return ret;

    return nrf24_cmd(rdev, NRF24_FLUSH_RX);
}

static ssize_t nrf24_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    struct nrf24_dev *rdev = file->private_data;
    uint8_t r_config, r_status, r_fifo;
    int ret;

    printk(KERN_DEBUG "nrf24: read data from device %d, offset: %lld.\n", count, *offset);
    if (*offset)
        return -EINVAL;

    ret = nrf24_read_reg(rdev, NRF24_CONFIG, &r_config);
    if (ret < 0)
        return ret;

    if (r_config & NRF24_PRIM_RX) {
        printk(KERN_WARNING "nrf24: device already in rx mode.\n");
        return -EIO;
    }

    ret = nrf24_write_reg(rdev, NRF24_CONFIG, r_config | NRF24_PRIM_RX);
    if (ret < 0)
        return ret;

    ret = nrf24_read_reg(rdev, NRF24_FIFO_STATUS, &r_fifo);
    if (ret < 0)
        return ret;

    if ((r_fifo & 0x01) == 0)
        nrf24_cmd(rdev, NRF24_FLUSH_RX);

    gpiod_set_value(rdev->gpio->ce, 1);
    udelay(130);

    ret = nrf24_read_reg(rdev, NRF24_STATUS, &r_status);
    if (ret < 0)
        return ret;

    if ((r_status & NRF24_RX_DR) == 0) {
        ret = wait_event_interruptible(nrf24_event_queue,
                nrf24_test_bit_reg(rdev, NRF24_STATUS, NRF24_RX_DR, &r_status));

        if (r_status & NRF24_RX_DR)
            nrf24_write_reg(rdev, NRF24_STATUS, NRF24_RX_DR);
    }

    if (ret == -ERESTARTSYS) {
        dev_info(&rdev->spi->dev, "read interrupted.\n");
        r_status = 0;
    }

    gpiod_set_value(rdev->gpio->ce, 0);
    nrf24_write_reg(rdev, NRF24_CONFIG, r_config);

    if (r_status & NRF24_RX_DR) {
        struct spi_transfer t[2] = {
            { .tx_buf = &rdev->buf->cmd, .len = sizeof(uint8_t) },
            { .rx_buf = rdev->buf->msg, .len = sizeof(rdev->buf->msg) }
        };

        memset(rdev->buf, 0, sizeof(*rdev->buf));

        rdev->buf->cmd = NRF24_R_RX_PAYLOAD;
        ret = spi_sync_transfer(rdev->spi, t, 2);

        printk(KERN_INFO "sync_transfer: %d, %02x:%02x\n", ret,
            rdev->buf->msg[0], rdev->buf->msg[1]);

        nrf24_cmd(rdev, NRF24_FLUSH_RX);
    }
    else
        return -ENODATA;

    if (r_status)
        copy_to_user(buf, rdev->buf->msg, min(NRF24_BUFFER_SIZE, count));

    return min(NRF24_BUFFER_SIZE, count);
}

static ssize_t nrf24_write(struct file *file, const char __user *buf, size_t count,
        loff_t *offset)
{
    struct nrf24_dev *rdev = file->private_data;
    uint8_t r_config, r_status;
    int restore_rx = 0;
    int ret;

    nrf24_register_dump(rdev->spi);

    printk(KERN_DEBUG "Write to nrf24 device, count: %d, offset: %lld.\n",
            count, *offset);

    if (count > NRF24_BUFFER_SIZE || *offset)
        return -EINVAL;

    ret = nrf24_read_reg(rdev, NRF24_CONFIG, &r_config);
    if (!ret && r_config & NRF24_PRIM_RX) {
        gpiod_set_value(rdev->gpio->ce, 0);
        ret = nrf24_write_reg(rdev, NRF24_CONFIG, r_config & ~NRF24_PRIM_RX);

        if (!ret)
            restore_rx = 1;
    }

    if (ret < 0)
        return ret;

    ret = nrf24_read_reg(rdev, NRF24_STATUS, &r_status);
    if (ret < 0)
        return ret;

    if (r_status & 0x01)
        return -ENOSPC;

    memset(rdev->buf, 0, sizeof(*rdev->buf));
    rdev->buf->cmd = NRF24_W_TX_PAYLOAD;
    copy_from_user(rdev->buf->msg, buf, count);

    ret = spi_write(rdev->spi, rdev->buf, sizeof(*rdev->buf));
    if (ret < 0)
        return ret;

    gpiod_set_value(rdev->gpio->ce, 1);
    udelay(11);
    gpiod_set_value(rdev->gpio->ce, 0);

    wait_event_interruptible(nrf24_event_queue,
            nrf24_test_bit_reg(rdev, NRF24_STATUS, NRF24_MAX_RT | NRF24_TX_DS, &r_status));

    nrf24_write_reg(rdev, NRF24_STATUS, 0x70);

    if (r_status & NRF24_MAX_RT)
        nrf24_cmd(rdev, NRF24_FLUSH_TX);

    if (restore_rx) {
        nrf24_read_reg(rdev, NRF24_CONFIG, &r_config);
        nrf24_write_reg(rdev, NRF24_CONFIG, r_config | NRF24_PRIM_RX);
        gpiod_set_value(rdev->gpio->ce, 1);
    }

    if (r_status & NRF24_MAX_RT)
        return -EIO;

    return count;
}

static long nrf24_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct nrf24_dev *rdev = file->private_data;
    u8 r_reg;
    u8 ce_state;
    int ret;

    printk(KERN_DEBUG "nrf24: ioctl command: %u, arg: %02lx.\n", cmd, arg);

    ret = gpiod_get_value(rdev->gpio->ce);
    if (ret < 0)
        return ret;

    ce_state = ret;

    switch(cmd) {
    case NRF24_IOC_SET_ADDR:
        printk(KERN_INFO "set device tx address.\n");
        ret = copy_from_user(rdev->buf->msg, (void __user *)arg, sizeof(uint8_t) * 5);
        if (ret < 0) {
            printk(KERN_INFO "Unable to copy user data.\n");
            break;
        }

        printk(KERN_INFO "address %02x %02x\n", rdev->buf->msg[3],
                rdev->buf->msg[4]);

        rdev->buf->cmd = NRF24_TX_ADDR | 0x20;
        ret = spi_write(rdev->spi, rdev->buf, sizeof(uint8_t) * 6);
        if (ret == 0) {
            rdev->buf->cmd = NRF24_RX_ADDR_P0 | 0x20;
            ret = spi_write(rdev->spi, rdev->buf, sizeof(uint8_t) * 6);
        }
        break;
    case NRF24_IOC_SET_RFCH:
        ret = get_user(r_reg, (__u8 __user *)arg);
        if (ret == 0)
            ret = nrf24_write_reg(rdev, NRF24_RF_CH, r_reg);

        break;
    }

    if (ce_state) {
        gpiod_set_value(rdev->gpio->ce, 1);
        udelay(130);
    }

    return 0;
}

static irqreturn_t nrf24_irq_handler(int irq, void *dev_id)
{
    printk(KERN_INFO "nrf24: interrupt occurred.\n");
    wake_up_interruptible(&nrf24_event_queue);

    return IRQ_HANDLED;
}

static int nrf24_open(struct inode *inode, struct file *file)
{
    struct nrf24_dev *rdev;
    uint8_t r_config;
    int ret = -ENXIO;

    printk(KERN_DEBUG "nrf24: device open.\n");
    mutex_lock(&nrf24_module_lock);

    list_for_each_entry(rdev, &nrf24_devices, device) {
        if (rdev->node_id == iminor(inode)) {
            ret = 0;
            break;
        }
    }

    if (ret == 0 && rdev->attrs & 0x01)
        ret = -EBUSY;

    if (ret < 0) {
        mutex_unlock(&nrf24_module_lock);
        return ret;
    }

    rdev->buf = kzalloc(sizeof(*rdev->buf), GFP_KERNEL);
    if (!rdev->buf) {
        mutex_unlock(&nrf24_module_lock);
        return -ENOMEM;
    }

    ret = nrf24_read_reg(rdev, NRF24_CONFIG, &r_config);
    if (!ret)
        ret = nrf24_write_reg(rdev, NRF24_CONFIG, r_config | NRF24_PWR_UP);

    if (!ret)
        schedule_timeout_interruptible(msecs_to_jiffies(3));

    if (ret < 0) {
        kfree(rdev->buf);
        mutex_unlock(&nrf24_module_lock);
        return ret;
    }

    rdev->attrs |= 0x01;
    file->private_data = rdev;

    nonseekable_open(inode, file);
    mutex_unlock(&nrf24_module_lock);

    return 0;
}

static int nrf24_release(struct inode *inode, struct file *file)
{
    struct nrf24_dev *rdev;

    printk(KERN_DEBUG "Release a nrf24 device.\n");
    mutex_lock(&nrf24_module_lock);

    rdev = file->private_data;
    if ((rdev->attrs & 0x01) == 0) {
        mutex_unlock(&nrf24_module_lock);
        printk(KERN_WARNING "nrf24: release a unbound device node.");
        return -EINVAL;
    }

    if (rdev->buf) {
        kfree(rdev->buf);
        rdev->buf = NULL;
    }

    rdev->attrs &= ~0x01;
    mutex_unlock(&nrf24_module_lock);
    return 0;
}

static const struct file_operations nrf24_fops = {
    .owner          = THIS_MODULE,
    .read           = nrf24_read,
    .write          = nrf24_write,
    .flush          = nrf24_flush,
    .open           = nrf24_open,
    .release        = nrf24_release,
    .unlocked_ioctl = nrf24_ioctl,
};

static int nrf24_spi_device_reset(struct nrf24_dev *rdev)
{
    int ret;

    gpiod_set_value(rdev->gpio->ce, 0);
    if (rdev->gpio->led)
        gpiod_set_value(rdev->gpio->led, 0);

    ret = nrf24_write_reg(rdev, NRF24_CONFIG, 0x08);
    if (ret < 0)
        return ret;

    ret = nrf24_write_reg(rdev, NRF24_SETUP_RETR, 0x5f);
    if (ret < 0)
        return ret;

    ret = nrf24_write_reg(rdev, NRF24_RF_SETUP, 0x0e);
    if (ret < 0)
        return ret;

    ret = nrf24_write_reg(rdev, NRF24_FEATURE, 0x00);
    if (ret < 0)
        return ret;

    ret = nrf24_write_reg(rdev, NRF24_DYNPD, 0x00);
    if (ret < 0)
        return ret;

    ret = nrf24_write_reg(rdev, NRF24_STATUS, 0x70);
    if (ret < 0)
        return ret;

    ret = nrf24_write_reg(rdev, NRF24_RF_CH, 0x34);
    if (ret < 0)
        return ret;

    ret = nrf24_write_reg(rdev, NRF24_RX_PW_P0, 0x20);
    if (ret < 0)
        return ret;

    ret = nrf24_write_reg(rdev, NRF24_RX_PW_P1, 0x20);
    ret = nrf24_cmd(rdev, NRF24_FLUSH_TX);
    if (ret < 0)
        return ret;

    return nrf24_cmd(rdev, NRF24_FLUSH_RX);
}

static int nrf24_gpio_create(struct nrf24_dev *rdev)
{
    struct nrf24_dev_gpios *gpios;

    gpios = kzalloc(sizeof(*rdev->gpio), GFP_KERNEL);
    if (!gpios)
        return -ENOMEM;

    gpios->irq = gpiod_get(&rdev->spi->dev, "interrupt", GPIOD_IN);
    if (IS_ERR(gpios->irq)) {
        kfree(gpios);
        return -EIO;
    }

    gpios->ce = gpiod_get(&rdev->spi->dev, "ce", GPIOD_OUT_LOW);
    if (IS_ERR(gpios->ce)) {
        gpiod_put(gpios->irq);
        kfree(gpios);
        return -EIO;
    }

    gpios->led = gpiod_get_optional(&rdev->spi->dev, "led", GPIOD_OUT_LOW);
    rdev->gpio = gpios;

    return 0;
}

static void nrf24_gpio_destroy(struct nrf24_dev *rdev)
{
    struct nrf24_dev_gpios *gpios = rdev->gpio;

    if (gpios->led)
        gpiod_put(gpios->led);

    gpiod_put(gpios->ce);
    gpiod_put(gpios->irq);

    kfree(gpios);
    rdev->gpio = NULL;
}

static int nrf24_probe(struct spi_device *spi)
{
    struct nrf24_dev *rdev;
    int ret, node_id;

    dev_info(&spi->dev, "device probe.\n");
    rdev = kzalloc(sizeof(*rdev), GFP_KERNEL);
    if (!rdev)
        return -ENOMEM;

    INIT_LIST_HEAD(&rdev->device);
    spin_lock_init(&rdev->bus_lock);

    rdev->spi = spi;

    mutex_lock(&nrf24_module_lock);
    node_id = find_first_zero_bit(nrf24_nodes, NRF24_MAX_NODES);
    if (node_id < NRF24_MAX_NODES) {
        struct device *dev;

        dev = device_create(nrf24_class, &spi->dev, MKDEV(nrf24_major, rdev->node_id),
                rdev, "radio-%d", rdev->node_id);

        ret = PTR_ERR_OR_ZERO(dev);
    }
    else {
        dev_warn(&spi->dev, "no empty node available!\n");
        ret = -ENODEV;
    }

    if (ret == 0) {
        set_bit(node_id, nrf24_nodes);
        list_add(&rdev->device, &nrf24_devices);
    }
    mutex_unlock(&nrf24_module_lock);

    if (!ret)
        ret = nrf24_gpio_create(rdev);

    if (ret < 0) {
        kfree(rdev);
        return ret;
    }

    ret = gpiod_to_irq(rdev->gpio->irq);
    printk(KERN_INFO "gpiod_to_irq result: %x\n", ret);
    if (ret < 0) {
        printk(KERN_WARNING "nrf24: unable to get irq number. (%x)\n", ret);
    }
    else {
        rdev->irq = ret;
        ret = request_irq(rdev->irq, nrf24_irq_handler,
                IRQF_TRIGGER_FALLING, "nrf24_irq", rdev);

        if (ret < 0) {
            printk(KERN_WARNING "nrf24: unable to request irq number. (%x)\n", ret);
            rdev->irq = 0;
        }
    }

    spi_set_drvdata(spi, rdev);
    return nrf24_spi_device_reset(rdev);
}

static int nrf24_remove(struct spi_device *spi)
{
    struct nrf24_dev *rdev;
    int ret;

    dev_warn(&spi->dev, "remove device.\n");
    rdev = spi_get_drvdata(spi);

    ret = nrf24_spi_device_reset(rdev);
    if (ret < 0)
        return ret;

    spin_lock_irq(&rdev->bus_lock);
    rdev->spi = NULL;
    nrf24_gpio_destroy(rdev);
    spin_unlock_irq(&rdev->bus_lock);

    mutex_lock(&nrf24_module_lock);
    device_destroy(nrf24_class, MKDEV(nrf24_major, rdev->node_id));

    if (rdev->irq > 0)
        free_irq(rdev->irq, rdev);

    clear_bit(rdev->node_id, nrf24_nodes);
    list_del(&rdev->device);

    kfree(rdev);
    mutex_unlock(&nrf24_module_lock);

    return 0;
}

static void nrf24_shutdown(struct spi_device *spi)
{
    dev_warn(&spi->dev, "shutdown mode called.\n");
}

static const struct of_device_id nrf24_of_ids[] = {
    { .compatible = "nordicsemi,nrf24l01p" },
    { },
};
MODULE_DEVICE_TABLE(of, nrf24_of_ids);

static struct spi_driver nrf24_spi_driver = {
    .driver = {
        .name = "nrf24",
        .of_match_table = nrf24_of_ids,
    },
    .probe = nrf24_probe,
    .remove = nrf24_remove,
    .shutdown = nrf24_shutdown,
};

static int __init nrf24_init(void)
{
    int ret;

    printk(KERN_INFO "Initialize nRF24 module.\n");

    ret = register_chrdev(0, nrf24_spi_driver.driver.name, &nrf24_fops);
    if (ret < 0)
        return ret;

    nrf24_major = ret;
    nrf24_class = class_create(THIS_MODULE, nrf24_spi_driver.driver.name);
    if (IS_ERR(nrf24_class)) {
        printk(KERN_WARNING "nrf24: unable to register device class.\n");
        unregister_chrdev(nrf24_major, nrf24_spi_driver.driver.name);
        return -EINVAL;
    }

    ret = spi_register_driver(&nrf24_spi_driver);
    if (ret < 0) {
        printk(KERN_WARNING "Unable to register nRF24 SPI Driver.\n");
        class_destroy(nrf24_class);
        unregister_chrdev(nrf24_major, nrf24_spi_driver.driver.name);
    }
    return ret;
}

static void __exit nrf24_exit(void)
{
    printk(KERN_INFO "Unloading module.. Bye.\n");
    spi_unregister_driver(&nrf24_spi_driver);
    class_destroy(nrf24_class);
    unregister_chrdev(nrf24_major, nrf24_spi_driver.driver.name);
}

module_init(nrf24_init);
module_exit(nrf24_exit);

MODULE_DESCRIPTION("nRF24L01 radio module driver");
MODULE_AUTHOR("Tomasz Król <tomicode@gmail.com>");
MODULE_LICENSE("GPL");
