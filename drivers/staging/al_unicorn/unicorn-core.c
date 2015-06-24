/*
  unicorn.c - V4L2 driver for FPGA UNICORN

  Copyright (c) 2010 - 2011 Aldebaran robotics
  joseph pinkasfeld joseph.pinkasfeld@gmail.com
  Ludovic SMAL <lsmal@aldebaran-robotics.com>
  Samuel MARTIN <s.martin49@gmail.com>

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include <linux/module.h>							/* Needed by all modules */
#include <linux/kernel.h>							/* Needed for KERN_INFO */
#include <linux/init.h>								/* Needed for the macros */

#define __UNICORN_CORE_C

#include "unicorn.h"
#include "unicorn-i2c.h"


#define DRIVER_AUTHOR 					"Joseph Pinkasfeld <joseph.pinkasfeld@gmail.com>;Ludovic SMAL <lsmal@aldebaran-robotics.com>"
#define DRIVER_DESC   					"Aldebaran Robotics Unicorn dual channel video acquisition"

static unsigned int unicorn_devcount = 0;
static struct unicorn_dev unicorn;

module_param(unicorn_debug, int, 0644);
MODULE_PARM_DESC(unicorn_debug, "enable debug messages [Unicorn core]");

module_param(unicorn_version, int, 0444);
MODULE_PARM_DESC(unicorn_version, "show fpga version major|minor|revision|bugfix");


/* IRQ handler */
static irqreturn_t unicorn_irq_handler(int irq, void *dev_id)
{
  struct unicorn_dev *dev;


  dev = (struct unicorn_dev *)dev_id;
  dev->pending = dev->interrupts_controller->irq.pending;
  dprintk(1, dev->name, "%s() 0x%X\n", __func__,dev->pending);


  if (dev->pending)
  {


  if ((dev->pending & IT_ABH32_ERROR)||
    (dev->pending & IT_ABH32_FIFO_RX_ERROR)||
    (dev->pending & IT_ABH32_FIFO_TX_ERROR))
  {
    printk(KERN_ERR "UNICORN %s() error IRQ \n", __func__);
  }
  if (dev->pending & IT_I2C_WRITE_FIFO_ERROR)
  {
    printk(KERN_ERR "UNICORN %s() IT_I2C_WRITE_FIFO_ERROR \n", __func__);
  }
  if (dev->pending & IT_I2C_READ_FIFO_ERROR)
  {
    printk(KERN_ERR "UNICORN %s() IT_I2C_READ_FIFO_ERROR \n", __func__);
  }
  if (dev->pending & IT_I2C_TRANSFER_END)
  {
    dev->i2c_eof = 1;
     //dprintk(1, "UNICORN %s() IT_I2C_TRANSFER_END \n", __func__);
  }
  if (dev->pending & IT_I2C_ACCES_ERROR)
  {
      dev->i2c_error = 1;
      dprintk(1, dev->name, "UNICORN %s() IT_I2C_ACCES_ERROR \n", __func__);
  }
 /* if (dev->interrupts_controller->irq.pending)
  {
    printk(KERN_ERR "%s() irq still pending 0x%X unknown state \n", __func__,dev->pending);
  }*/

  dev->interrupt_queue_flag = 1;
  return IRQ_HANDLED;
  }
  else
  {
  return IRQ_NONE;
  }
}


/**
 * Unmap the BAR regions that had been mapped earlier using map_bars()
 */
static void unmap_bars(struct unicorn_dev *dev)
{
  if (dev->fpga_regs) {
    /* unmap BAR */
    pci_iounmap(dev->fpga_regs, dev->pci);
    dev->fpga_regs = NULL;
  }
}

/**
 * Map the device memory regions into kernel virtual address space after
 * verifying their sizes respect the minimum sizes needed, given by the
 * bar_min_len[] array.
 */
static int map_bars(struct unicorn_dev *dev)
{
  int ret;
  unsigned long bar_start = pci_resource_start(dev->pci, 0);
  unsigned long bar_end = pci_resource_end(dev->pci, 0);
  unsigned long bar_length = bar_end - bar_start + 1;
  dev->fpga_regs = NULL;

  /* map the device memory or IO region into kernel virtual
         * address space */
  dev->fpga_regs = pci_iomap(dev->pci, 0, bar_length);
  if (!dev->fpga_regs) {
    printk(KERN_DEBUG "Could not map BAR #%d.\n", 0);
    ret = -1;
    goto fail;
  }

  ret = 0;
  goto success;
  fail:
    /* unmap any BARs that we did map */
    unmap_bars(dev);
  success:
    return ret;
}

static int unicorn_dev_setup(void)
{
  int ret = 0;
  mutex_init(&unicorn.mutex);
  mutex_init(&unicorn.i2c_mutex);
  sprintf(unicorn.name, DRV_NAME);

  unicorn.nr = ++unicorn_devcount;

  ret = pci_enable_device(unicorn.pci);
  if (ret)
  {
    printk(KERN_INFO "Cannot enable PCI device\n");
    return -1;
  }
  pci_set_master(unicorn.pci);

  ret = pci_request_regions(unicorn.pci, DRV_NAME);
  if(ret)
  {
    printk(KERN_ERR "Cannot request pci region\n");
    goto fail_disable;
  }
  ret = map_bars(&unicorn);

  unicorn.pcie_dma = (struct abh32_pcie_dma *)(unicorn.fpga_regs + AHB32_PCIE_DMA);
  unicorn.global_register = (struct abh32_global_registers *)(unicorn.fpga_regs + AHB32_GLOBAL_REGISTERS);
  unicorn.interrupts_controller = (struct abh32_interrupts_controller *)(unicorn.fpga_regs + AHB32_INTERRUPTS_CTRL);
  unicorn.i2c_master = (struct abh32_i2c_master *)(unicorn.fpga_regs + AHB32_I2C_MASTER);
  unicorn.spi_flash = (struct abh32_spi_flash *)(unicorn.fpga_regs + AHB32_SPI_FLASH);

  printk(KERN_INFO "unicorn : bugfix:%d revision:%d major:%d minor:%d\n"
         ,unicorn.global_register->general.version.bugfix
         ,unicorn.global_register->general.version.revision
         ,unicorn.global_register->general.version.major
         ,unicorn.global_register->general.version.minor);

  /*
  if(unicorn.global_register->general.version.major < VERSION_MAJOR_MIN )
  {
    printk(KERN_ERR "unicorn : wrong major version (min=%d) \n",VERSION_MAJOR_MIN);
    goto fail_unmap;
  }
  else if(unicorn.global_register->general.version.major == VERSION_MAJOR_NO_VERSION )
  {
    printk(KERN_ERR "unicorn : wrong major version (min=%d) \n",VERSION_MAJOR_MIN);
    goto fail_unmap;

  }
  else if(unicorn.global_register->general.version.major == VERSION_MAJOR_MIN )
  {
    if(unicorn.global_register->general.version.minor < VERSION_MINOR_MIN )
    {
      printk(KERN_ERR "unicorn : wrong minor version (min=%d)\n",VERSION_MINOR_MIN);
      goto fail_unmap;
    }
  }
*/

  unicorn_version = ((unicorn.global_register->general.version.major & 0xFF) << 24)   |
                    ((unicorn.global_register->general.version.minor & 0xFF) << 16)   |
                    ((unicorn.global_register->general.version.revision & 0xFF) << 8) |
                    (unicorn.global_register->general.version.bugfix & 0xFF);

  return 0;
 /* fail_unmap :
    unmap_bars(&unicorn);
    pci_release_regions(unicorn.pci);*/
  fail_disable :
    pci_disable_device(unicorn.pci);
  return -1;
}

static int unicorn_interrupt_init(void)
{
  int ret = 0;

  unicorn.interrupts_controller->irq.ctrl = 0;

  /* Interrupts : */
  if (pci_enable_msi(unicorn.pci))
  {
    printk(KERN_INFO "unicorn : failed to activate MSI interrupts !\n");
    return -1;
  }

  ret = request_irq(unicorn.pci->irq, unicorn_irq_handler, 0, DRV_NAME, &unicorn);
  if (ret)
  {
    printk(KERN_INFO "unicorn : can't get IRQ %d, err = %d\n", unicorn.pci->irq, ret);
    return -1;
  }

  return 0;
}

static int unicorn_pci_probe(struct pci_dev *pci_dev,
                  const struct pci_device_id *pci_id)
{
  int err=0;

  /* PCIe identification : */
  if (pci_id->vendor != PEGD_VENDOR_ID)
  {
    printk(KERN_INFO "unicorn_probe, vendor != PEGD_VENDOR_ID !\n");
    return -1;
  }

  unicorn.pci = pci_dev;


  err =  unicorn_dev_setup();
  if(err < 0)
  {
    goto fail_unregister_device;
  }
  err = unicorn_interrupt_init();
  if(err < 0)
  {
    goto fail_disable_device;
  }

  err = unicorn_init_i2c(&unicorn);
  if(err < 0)
  {
    goto fail_irq_disable;
  }


  return 0;

  unicorn_i2c_remove(&unicorn);
fail_irq_disable :
  free_irq(unicorn.pci->irq, &unicorn);

if (unicorn.pci->msi_enabled)
  {
    pci_disable_msi(pci_dev);
  }

fail_disable_device :
  if(unicorn.fpga_regs)
  {
    unmap_bars(&unicorn);
  }
  pci_disable_device(unicorn.pci);
  pci_release_regions(unicorn.pci);

fail_unregister_device :


  return err;
}

static void unicorn_pci_remove(struct pci_dev *pci_dev)
{




  unicorn_i2c_remove(&unicorn);


  free_irq(pci_dev->irq, &unicorn);

  if (pci_dev->msi_enabled)
  {
    pci_disable_msi(pci_dev);
  }

  if(unicorn.fpga_regs)
  {
    unmap_bars(&unicorn);
  }

  pci_disable_device(unicorn.pci);

  pci_release_regions(unicorn.pci);

}


static struct pci_device_id unicorn_pci_tbl[] = {
  {
   .vendor = PCI_VENDOR_ID_XILINX,
   .device = PEGD_DEVICE_ID,
   .subvendor = PEGD_SUBVENDOR_ID,
   .subdevice = PEGD_SUBDEVICE_ID,
   },
  {
   /* --- end of list --- */
   }
};

MODULE_DEVICE_TABLE(pci, unicorn_pci_tbl);

struct pci_driver __refdata unicorn_pci_driver =
{
   name:      PEGD_DEVICE_NAME,
   id_table:  unicorn_pci_tbl,
   probe:     unicorn_pci_probe,
   remove:    unicorn_pci_remove
};



static int __init unicorn_init(void)
{
  int err = 0;


/* Clear unicorn structure : */
  memset(&unicorn, 0, sizeof(unicorn));

/* PCI bus initialisation : */
  if (pci_register_driver(&unicorn_pci_driver))
  {
    printk(KERN_INFO "error : pci_register_driver(&unicorn_pci_driver) != 0 !\n");
    return -1;
  }

#ifdef CONFIG_AL_UNICORN_SYSFS_FLASH
  unicorn.class_unicorn=class_create(THIS_MODULE,"unicorn");
  err = unicorn_spi_flash_init(unicorn.class_unicorn,&unicorn);
#endif

  return err;
}

static void __exit unicorn_exit(void)
{
  pci_unregister_driver(&unicorn_pci_driver);

#ifdef CONFIG_AL_UNICORN_SYSFS_FLASH
  class_destroy(unicorn.class_unicorn);
#endif
  printk(KERN_INFO "UNICORN module unregistered\n");
}

module_init(unicorn_init);
module_exit(unicorn_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_SUPPORTED_DEVICE(PEGD_DEVICE_NAME);

/* unicorn.c end */
