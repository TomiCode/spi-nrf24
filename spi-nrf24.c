/*
 * Copyright (c) 2018 Tomasz Król <tomicode@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <linux/spi/spi.h>

struct nrf24_radio {
  struct spi_device *spi;
};

static const struct file_operations nrf24_fops = {
  .owner = THIS_MODULE,
};

static struct class *nrf24_class;

static int nrf24_major_num;

static int nrf24_probe(struct spi_device *spi)
{
  printk(KERN_INFO "nrf24_probe called\n");
  return 0;
}

static int nrf24_remove(struct spi_device *spi)
{
  printk(KERN_INFO "nrf24_remove called.\n");
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
  printk(KERN_INFO "Hello world from nrf24 driver.\n");

  int status = register_chrdev(0, "nrf24", &nrf24_fops);
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
MODULE_LICENSE("MIT");
