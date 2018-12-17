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

#include <linux/spi/spi.h>

struct nrf24_radio {
  dev_t devt;
  struct spi_device *spi;

  struct gpio_desc *irq_gpiod;
  struct gpio_desc *ce_gpiod;
  struct gpio_desc *led_gpiod;
};

static const struct file_operations nrf24_fops = {
  .owner = THIS_MODULE,
};

static struct class *nrf24_class;

static int nrf24_major_num;

static int nrf24_probe(struct spi_device *spi)
{
  struct nrf24_radio *nrf24dev;
  struct device *dev;

  nrf24dev = kmalloc(sizeof(*nrf24dev), GFP_KERNEL);
  if (!nrf24dev)
    return -ENOMEM;

  nrf24dev->spi = spi;
  nrf24dev->devt = MKDEV(nrf24_major_num, 0);

  
  dev = device_create(nrf24_class, &spi->dev, nrf24dev->devt,
      nrf24dev, "radio-%d", spi->chip_select);

  if (PTR_ERR_OR_ZERO(dev) != 0) {
    kfree(nrf24dev);
    return -ENODEV;
  }

  spi_set_drvdata(spi, nrf24dev);
  return 0;
}

static int nrf24_remove(struct spi_device *spi)
{
  struct nrf24_radio *nrf24dev;

  printk(KERN_INFO "nrf24_remove called.\n");

  nrf24dev = spi_get_drvdata(spi);
  nrf24dev->spi = NULL;

  device_destroy(nrf24_class, nrf24dev->devt);
  kfree(nrf24dev);

  return 0;
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
