/*
 * drivers/misc/bug_dock.c
 *
 * Dock driver for OMAP3 BUGBASE
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <plat/hardware.h>
#include <plat/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include "../arch/arm/mach-omap2/mux.h"
#include <linux/bmi.h>

#define USB_RESET        193
#define DOCK_PRESENCE    113

extern void McBSP_DR_DX_enable(int);
extern int test_mcbsp (int);
static DEFINE_MUTEX(count_lock);
int usb_count;

struct dock_device {
  struct platform_device  *pdev;
  struct work_struct       irq_handler_work;
};

static struct platform_device *bug_dock_dev;

/**
 *    sysfs interface
 */

static ssize_t usb_enable_show(struct device *dev, 
				    struct device_attribute *attr, char *buf)
{
  int len = 0;

  if (usb_count > 0)
    len += sprintf(buf+len, "1\n");
  else
    len += sprintf(buf+len, "0\n");    

  return len;
}
	
static ssize_t usb_enable_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t len)
{
    if (strchr(buf, '1') != NULL){
      increment_usb_dep();
    }
    else if (strchr(buf, '0') != NULL){
      mutex_lock(&count_lock);
      usb_count = 1;                         //this will turn the hub off
      mutex_unlock(&count_lock);
      decrement_usb_dep();
    }
    return len;
}

static DEVICE_ATTR(usb_enable, 0664, usb_enable_show, usb_enable_store);

static ssize_t store_mcbsp_test (struct device * dev, struct device_attribute * attr,
		const char * buf, size_t count)
{ 
	int val=0;
	int ret =0;
	mdelay (10);
	sscanf (buf, "%d", &val);
	switch (val)
	{
		case 1:
			ret = test_mcbsp(3);
			if (ret < 0){
				printk("McBSP test failed\n");
				return -EIO;
			}
			break;
		case 3:
			ret = test_mcbsp(3);
			if (ret < 0){
				printk("McBSP test failed\n");
				return -EIO;
			}
			break;
		case 4:
			ret = test_mcbsp(4);
			if (ret < 0){
				printk("McBSP test failed\n");
				return -EIO;
			}
			break;
		default:
			break;
	}
	return count;
}
static DEVICE_ATTR(mcbsp_test, S_IWUSR, NULL, &store_mcbsp_test);

static ssize_t store_mcbsp_en (struct device * dev, struct device_attribute * attr,
		const char * buf, size_t count)
{
	int val=0;

	sscanf (buf, "%d", &val);

	if(val == 1)
		McBSP_DR_DX_enable(1);
	if (val == 0)
		McBSP_DR_DX_enable(0);

	return count;
}
static DEVICE_ATTR(mcbsp_en, S_IWUSR, NULL, &store_mcbsp_en);


/**
 *    work queuing
 */

static inline struct dock_device *irq_handler_work_to_dock_device(struct work_struct *work)
{
  return container_of(work, struct dock_device, irq_handler_work);
}

static void irq_handler_work(struct work_struct *work)
{
  bool value = gpio_get_value(DOCK_PRESENCE);
  
  if (value) {
    printk(KERN_INFO "bug_dock: Dock Removed\n");
    decrement_usb_dep();
  }
  else {
    printk(KERN_INFO "bug_dock: Dock Inserted\n");
    increment_usb_dep();
  }
}

/**
 *    interrupt handler
 */

static irqreturn_t dock_irq_handler(int irq, void *dev_id)
{
  struct dock_device *bug_dock = dev_id;
  schedule_work(&bug_dock->irq_handler_work);
  return IRQ_HANDLED;
}

/**
 *    exported functions
 */

void increment_usb_dep(void) {
  bool value = gpio_get_value(USB_RESET);

  mutex_lock(&count_lock);
  if (!value) {
    gpio_set_value (USB_RESET, 1);
  }
  usb_count ++;
  mutex_unlock(&count_lock);
}
EXPORT_SYMBOL(increment_usb_dep);

void decrement_usb_dep(void){
  mutex_lock(&count_lock);
  usb_count --;
  if (usb_count == 0) {
    gpio_set_value (USB_RESET, 0);
  }
  mutex_unlock(&count_lock);
}
EXPORT_SYMBOL(decrement_usb_dep);

/**
 *    probe/remove
 */

static int bug_dock_probe(struct platform_device *pdev)
{
  int err;
  bool value;
  struct dock_device *bug_dock;

  mutex_lock(&count_lock);

  bug_dock = kzalloc(sizeof(struct dock_device), GFP_KERNEL);
  bug_dock->pdev = pdev;

  // init usb_count
  usb_count = 0;

  // init work struct
  INIT_WORK(&bug_dock->irq_handler_work, irq_handler_work);

  // request control of USB reset
  err =  gpio_request(USB_RESET, "usb_reset");
  if (err)
	  printk("*******gpio request failed\n");
  err |= gpio_direction_output(USB_RESET, 1);
  
  omap_mux_init_gpio(DOCK_PRESENCE, OMAP_PIN_INPUT_PULLUP);
  // request dock presence pin
  err =  gpio_request(DOCK_PRESENCE, "dock_presence");
  err |= gpio_direction_input(DOCK_PRESENCE);
  omap_set_gpio_debounce(DOCK_PRESENCE, 1);
  omap_set_gpio_debounce_time(DOCK_PRESENCE, 0xff);

//  omap_cfg_reg(AH19_34XX_GPIO113);

  // request dock presence irq
  err = request_irq(gpio_to_irq(DOCK_PRESENCE), dock_irq_handler,
	      IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "bug-dock", bug_dock); 

  platform_set_drvdata(pdev, bug_dock);

  err = sysfs_create_file(&pdev->dev.kobj, &dev_attr_usb_enable.attr);
  if (err < 0)
    printk(KERN_ERR "Error creating SYSFS entries...\n");

  err = sysfs_create_file(&pdev->dev.kobj, &dev_attr_mcbsp_test.attr);
  if (err < 0)
    printk(KERN_ERR "Error creating SYSFS entries for McBSP tests...\n");
  err = sysfs_create_file(&pdev->dev.kobj, &dev_attr_mcbsp_en.attr);
  if (err < 0)
    printk(KERN_ERR "Error creating SYSFS entries for McBSP EN...\n");


  // check for dock
  value = gpio_get_value(DOCK_PRESENCE);
  if (!value) {
    gpio_set_value (USB_RESET, 1);
    usb_count ++;
  }
  else {
    gpio_set_value (USB_RESET, 0);
  }

  if (err < 0) {
    printk(KERN_ERR "bug_dock: error during probe...\n");
    return -EINVAL;
  }

  mutex_unlock(&count_lock);

  printk(KERN_INFO "bug_dock: bug_dock_probe...\n");
  return 0;
}

static int bug_dock_remove(struct platform_device *pdev)
{
  struct dock_device *bug_dock;
  bug_dock = platform_get_drvdata(pdev);

  free_irq(gpio_to_irq(DOCK_PRESENCE), bug_dock);
  gpio_free(DOCK_PRESENCE);
  gpio_free(USB_RESET);

  printk(KERN_INFO "bug_dock: bug_dock_remove...\n");
  return 0;
}

static int bug_dock_suspend(struct platform_device *pdev, pm_message_t state)
{
       return 0;
}

static int bug_dock_resume(struct platform_device *pdev)
{
       return 0;
}

static struct platform_driver bug_dock_drv = {
       .probe          = bug_dock_probe,
       .remove         = bug_dock_remove,
       .suspend        = bug_dock_suspend,
       .resume         = bug_dock_resume,
       .driver         = {
                               .name   = "bug-dock",
                       },
};

static int __init bug_dock_init(void)
{
  int ret; 

  bug_dock_dev = platform_device_alloc("bug-dock", -1);
  ret = platform_device_add(bug_dock_dev);

  ret = platform_driver_register(&bug_dock_drv);
  return ret;
}

static void __exit bug_dock_exit(void)
{
  platform_driver_unregister(&bug_dock_drv);
  platform_device_unregister(bug_dock_dev);
}

module_init(bug_dock_init);
module_exit(bug_dock_exit);

MODULE_AUTHOR("Dave R");
MODULE_DESCRIPTION("OMAP3 Bug Dock Driver");
MODULE_LICENSE("GPL");
