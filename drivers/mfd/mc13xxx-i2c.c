/*
 * Copyright 2009-2010 Creative Product Design
 * Marc Reilly marc@cpdesign.com.au
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/mfd/mc13xxx.h>
#include <linux/i2c.h>

static int mc13xxx_i2c_reg_read(struct mc13xxx *mc13xxx, unsigned int offset,
		u32 *val)
{
	int ret;
	unsigned char buf[3] = {0, 0, 0};

	ret = i2c_smbus_read_i2c_block_data(mc13xxx->i2cclient,
			offset, 3, buf);
	*val = buf[0] << 16 | buf[1] << 8 | buf[2];

	return ret == 3 ? 0 : ret;
}

static int mc13xxx_i2c_reg_write(struct mc13xxx *mc13xxx, unsigned int offset,
		u32 val)
{
	int ret;
	unsigned char buf[3];

	buf[0] = (val >> 16) & 0xff;
	buf[1] = (val >> 8) & 0xff;
	buf[2] = val & 0xff;

	ret = i2c_smbus_write_i2c_block_data(mc13xxx->i2cclient,
			offset, 3, buf);

	return ret;
}

static int mc13xxx_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct mc13xxx *mc13xxx;
	struct mc13xxx_platform_data *pdata = dev_get_platdata(&client->dev);
	int ret;

	mc13xxx = kzalloc(sizeof(*mc13xxx), GFP_KERNEL);
	if (!mc13xxx)
		return -ENOMEM;

	dev_set_drvdata(&client->dev, mc13xxx);
	mc13xxx->dev = &client->dev;
	mc13xxx->i2cclient = client;
	mc13xxx->read_dev = mc13xxx_i2c_reg_read;
	mc13xxx->write_dev = mc13xxx_i2c_reg_write;

	ret = mc13xxx_common_init(mc13xxx, pdata, client->irq);

	if (ret == 0 && (id->driver_data != mc13xxx->ictype))
		dev_warn(mc13xxx->dev,
				"device id doesn't match auto detection!\n");

	return ret;
}

static int __devexit mc13xxx_i2c_remove(struct i2c_client *client)
{
	struct mc13xxx *mc13xxx = dev_get_drvdata(&client->dev);

	free_irq(client->irq, mc13xxx);

	mfd_remove_devices(&client->dev);

	kfree(mc13xxx);

	return 0;
}

static const struct i2c_device_id mc13xxx_i2c_idtable[] = {
	{"mc13892", MC13XXX_ID_MC13892},
	{ }
};

static struct i2c_driver mc13xxx_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "mc13xxx-i2c"
	},
	.id_table = mc13xxx_i2c_idtable,
	.probe = mc13xxx_i2c_probe,
	.remove = __devexit_p(mc13xxx_i2c_remove),
};

static int __init mc13xxx_i2c_init(void)
{
	return i2c_add_driver(&mc13xxx_i2c_driver);
}
subsys_initcall(mc13xxx_i2c_init);

static void __exit mc13xxx_i2c_exit(void)
{
	i2c_del_driver(&mc13xxx_i2c_driver);
}
module_exit(mc13xxx_i2c_exit);

MODULE_DESCRIPTION("i2c driver for Freescale MC13XXX PMIC");
MODULE_AUTHOR("Marc Reilly <marc@cpdesign.com.au");
MODULE_LICENSE("GPL v2");
