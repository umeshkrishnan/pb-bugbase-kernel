/*
 * BQ27x00 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <plat/gpio.h>
#include <asm/unaligned.h>

#define DRIVER_VERSION			"1.0.0"

#define BQ27x00_REG_TEMP		0x06
#define BQ27x00_REG_VOLT		0x08
#define BQ27x00_REG_FLAGS		0x0A
#define BQ27x00_REG_AI			0x14
#define BQ27x00_REG_TTF			0x18
#define BQ27x00_REG_TTE			0x16
#define BQ27x00_REG_RSOC		0x2c /* Relative State-of-Charge */

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);
struct bq27x00_device_info;
struct bq27x00_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct bq27x00_device_info *di);
};

struct bq27x00_device_info {
	struct device 		*dev;
	int			id;
	int			voltage_uV;
	int			current_uA;
	int			temp_C;
	int			charge_rsoc;
	struct bq27x00_access_methods	*bus;
	struct power_supply	bat;

	struct i2c_client	*client;
};

static enum power_supply_property bq27x00_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY, /* in percents! */
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,

};

/*
 * Common code for BQ27x00 devices
 */

static int bq27x00_read(u8 reg, int *rt_value, int b_single,
			struct bq27x00_device_info *di)
{
	int ret;

	ret = di->bus->read(reg, rt_value, b_single, di);
	*rt_value = be16_to_cpu(*rt_value);

	return ret;
}

/*
 * Return the battery temperature in Celsius degrees
 * Or < 0 if something fails.
 */
static int bq27x00_battery_temperature(struct bq27x00_device_info *di)
{
	int ret;
	int temp = 0;

	ret = bq27x00_read(BQ27x00_REG_TEMP, &temp, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading temperature\n");
		return ret;
	}

	return ((temp / 10 ) - 273);
}

static int bq27x00_battery_stat(struct bq27x00_device_info *di)
{
	int ret;
	int flags = 0;

	ret = bq27x00_read(BQ27x00_REG_FLAGS, &flags, 0, di);

	if ((flags & (1 << 9)) == 1)
		return POWER_SUPPLY_STATUS_FULL;

	if (gpio_get_value(164))
		return POWER_SUPPLY_STATUS_DISCHARGING;
	else
		return POWER_SUPPLY_STATUS_CHARGING;
}

static int bq27x00_battery_present(struct bq27x00_device_info *di)
{
	int ret;
	int flags = 0;

	ret = bq27x00_read(BQ27x00_REG_FLAGS, &flags, 0, di);
	if ((flags & (1 << 3)) == 1)
		return 1;
	else
		return 0;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27x00_battery_voltage(struct bq27x00_device_info *di)
{
	int ret;
	int volt = 0;

	ret = bq27x00_read(BQ27x00_REG_VOLT, &volt, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading voltage\n");
		return ret;
	}

	return volt;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static signed int bq27x00_battery_current(struct bq27x00_device_info *di)
{
	int ret;
	int curr = 0;
	int flags = 0;

	ret = bq27x00_read(BQ27x00_REG_AI, &curr, 0, di);
//	printk("current = %d\n", curr);
	if (ret) {
		dev_err(di->dev, "error reading current\n");
		return 0;
	}

	ret = bq27x00_read(BQ27x00_REG_FLAGS, &flags, 0, di);
//	printk("Flags = %x\n", flags);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return 0;
	}
	if ((flags & (1 << 0)) != 0) {
//		dev_dbg(di->dev, "negative current!\n");
//		printk("negative current!\n");
		return -((~curr +1) & 0x00ff);
	}

	return curr;
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27x00_battery_rsoc(struct bq27x00_device_info *di)
{
	int rsoc;

	rsoc = i2c_smbus_read_word_data(di->client, BQ27x00_REG_RSOC);
	return (rsoc & 0xffff);
}

static int bq27x00_battery_tte(struct bq27x00_device_info *di)
{
	int ret;
	unsigned int tte = 0;

	ret = bq27x00_read(BQ27x00_REG_TTE, &tte, 1, di);
//	printk("TTE = %x\n", tte);
	if (ret) {
		dev_err(di->dev, "error reading Time To Empty\n");
		return ret;
	}

//	return rsoc >> 8;
	return tte;
}

static int bq27x00_battery_ttf(struct bq27x00_device_info *di)
{
	int ret;
	unsigned int ttf = 0;

	ret = bq27x00_read(BQ27x00_REG_TTF, &ttf, 1, di);
//	printk("TTF = %x\n", ttf);
	if (ret) {
		dev_err(di->dev, "error reading Time To Empty\n");
		return ret;
	}

//	return rsoc >> 8;
	return ttf;
}


#define to_bq27x00_device_info(x) container_of((x), \
				struct bq27x00_device_info, bat);

int bq27500_usb_power(int level)
{
	        gpio_set_value(111, level);
		        return 0;
}
EXPORT_SYMBOL(bq27500_usb_power);

int bq27500_usb_susp(int enable)
{
	        gpio_set_value(96, enable);
		        return 0;
}
EXPORT_SYMBOL(bq27500_usb_susp);

static ssize_t show_bq27500_usb_power(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	char *sval = buf;

	sval += sprintf(sval, "%d\n",
			gpio_get_value(111)? 1 : 0);

	sval += 1;
	*sval = 0;

	return sval - buf + 1;
}

static ssize_t set_bq27500_usb_power(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned int res;
	res = simple_strtoul(buf, NULL, 10);
	if (res)
		bq27500_usb_power(1);
	else
		bq27500_usb_power(0);
	return count;
}

static ssize_t show_bq27500_usb_susp(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	char *sval = buf;

	sval += sprintf(sval, "%d\n",
			gpio_get_value(96)? 1 : 0);

	sval += 1;
	*sval = 0;

	return sval - buf + 1;
}
static ssize_t set_bq27500_usb_susp(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned int res;
	res = simple_strtoul(buf, NULL, 10);
	if (res)
		bq27500_usb_susp(1);
	else
		bq27500_usb_susp(0);
	return count;
}

static ssize_t show_bq27500_ac_present(struct device *dev, struct device_attribute *attr,
		char *buf, size_t count)
{
	char *sval = buf;
	sval += sprintf(sval, "%s\n",
			gpio_get_value(43)? "not present" : "present");

	sval += 1;
	*sval = 0;

	return sval - buf + 1;
}

static ssize_t show_bq27500_sw_status(struct device *dev, struct device_attribute *attr,
		char *buf, size_t count)
{
	char *sval = buf;
	sval += sprintf(sval, "%d\n",
			gpio_get_value(107)? 1 : 0);
	if (gpio_get_value(107))
		printk("sw status high for base\n");
	else
		printk("sw status low for base\n");

	sval += 1;
	*sval = 0;

	return sval - buf + 1;
}
static DEVICE_ATTR(usb_power, 0664, &show_bq27500_usb_power,
		                &set_bq27500_usb_power);
static DEVICE_ATTR(usb_susp, 0664, &show_bq27500_usb_susp,
		                &set_bq27500_usb_susp);
static DEVICE_ATTR(ac_present, S_IRUGO, &show_bq27500_ac_present, NULL);
static DEVICE_ATTR(sw_status, S_IRUGO, &show_bq27500_sw_status, NULL);

static int bq27x00_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq27x00_battery_stat(di);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = (gpio_get_value(64)) ? POWER_SUPPLY_HEALTH_LOW : POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq27x00_battery_present(di);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		val->intval = bq27x00_battery_voltage(di);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = bq27x00_battery_current(di);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bq27x00_battery_rsoc(di);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq27x00_battery_temperature(di);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		val->intval = bq27x00_battery_tte(di);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		val->intval = bq27x00_battery_ttf(di);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void bq27x00_powersupply_init(struct bq27x00_device_info *di)
{
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27x00_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27x00_battery_props);
	di->bat.get_property = bq27x00_battery_get_property;
	di->bat.external_power_changed = NULL;
}

/*
 * BQ27200 specific code
 */

static int bq27200_read(u8 reg, int *rt_value, int b_single,
			struct bq27x00_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_be16(data);
			else
				*rt_value = data[0];

			return 0;
		}
	}
	return err;
}

static int bq27200_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct bq27x00_device_info *di;
	struct bq27x00_access_methods *bus;
	int num;
	int retval = 0;
	int value = 0;
	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	name = kasprintf(GFP_KERNEL, "bq27200-%d", num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	di->id = num;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access method "
					"data\n");
		retval = -ENOMEM;
		goto batt_failed_3;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->bat.name = name;
	bus->read = &bq27200_read;
	di->bus = bus;
	di->client = client;

	bq27x00_powersupply_init(di);

	/* Read the HW Version */
	value = i2c_smbus_write_word_data(di->client, 0, 0x0003);
	if (value < 0)
		goto batt_failed_3;

	value = i2c_smbus_read_word_data(di->client, 0);
	if (value < 0)
		goto batt_failed_3;
	printk ("HW Version: %#x\n\r", value);

	/* Read the CHEM ID */
	value = i2c_smbus_write_word_data(di->client, 0, 0x0008);
	if (value < 0)
		goto batt_failed_3;

	value = i2c_smbus_read_word_data(di->client, 0);
	if (value < 0)
		goto batt_failed_3;
	printk ("CHEM ID: %#x\n\r", value);

	/* Read the Firmware Version */
	value = i2c_smbus_write_word_data(di->client, 0, 0x0002);
	if (value < 0)
		goto batt_failed_3;

	value = i2c_smbus_read_word_data(di->client, 0);
	if (value < 0)
		goto batt_failed_3;
	printk ("Firmware Version: %#x\n\r", value);

	/* Issue RESET command */
	if (i2c_smbus_write_word_data(di->client, 0, 0x0041) < 0)
		goto batt_failed_3;

	mdelay (200);

	/* Perform IT_ENABLE */
	if (i2c_smbus_write_word_data(di->client, 0, 0x0021) < 0)
		goto batt_failed_3;

	mdelay (200);

	retval = power_supply_register(&client->dev, &di->bat);
	if (retval) {
		dev_err(&client->dev, "failed to register battery\n");
		goto batt_failed_4;
	}
	if (device_create_file(&client->dev, &dev_attr_usb_susp) < 0)
		printk(KERN_ERR "ERROR:usb suspend sysfs entry\n");
	if (device_create_file(&client->dev, &dev_attr_usb_power) < 0)
		printk(KERN_ERR "ERROR:usb power sysfs entry\n");
	if (device_create_file(&client->dev, &dev_attr_ac_present) < 0)
		printk(KERN_ERR "ERROR:ac present sysfs entry\n");
	if (device_create_file(&client->dev, &dev_attr_sw_status) < 0)
		printk(KERN_ERR "ERROR:sw status sysfs entry\n");

	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	return 0;

batt_failed_4:
	kfree(bus);
batt_failed_3:
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27200_battery_remove(struct i2c_client *client)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);

	power_supply_unregister(&di->bat);

	kfree(di->bat.name);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);
	gpio_free(43);
	gpio_free(164);
	gpio_free(107);
	gpio_free(64);
	gpio_free(96);
	gpio_free(111);

	return 0;
}

/*
 * Module stuff
 */

static const struct i2c_device_id bq27200_id[] = {
	{ "bq27200", 0 },
	{},
};

static struct i2c_driver bq27200_battery_driver = {
	.driver = {
		.name = "bq27200-battery",
	},
	.probe = bq27200_battery_probe,
	.remove = bq27200_battery_remove,
	.id_table = bq27200_id,
};

static int __init bq27x00_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq27200_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27200 driver\n");

	return ret;
}
module_init(bq27x00_battery_init);

static void __exit bq27x00_battery_exit(void)
{
	i2c_del_driver(&bq27200_battery_driver);
}
module_exit(bq27x00_battery_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27x00 battery monitor driver");
MODULE_LICENSE("GPL");
