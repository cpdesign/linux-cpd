/*
 * Backlight driver based on isl22346 digital pot
 *
 * Copyright 2011 Creative Product Design
 *
 * Started from code:
 * Copyright 2009-2010 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include <video/isl22346_bl.h>

#define ISL22346_WR	0
#define ISL22346_IVR	0
#define ISL22346_ACR	8

#define ISL22346_MAX_BRIGHTNESS 127

struct isl22346_bl {
	struct i2c_client *client;
	struct backlight_device *bl;
	int current_brightness;
	int inverted;
	enum isl22346_bl_enable_types enable_type;
	unsigned int enable_gpio;
};

static int isl22346_read(struct i2c_client *client, int reg, uint8_t *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading at 0x%02x\n", reg);
		return ret;
	}

	*val = (uint8_t)ret;
	return 0;
}

static int isl22346_write(struct i2c_client *client, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(client, reg, val);
}

static void isl22346_bl_enable(struct isl22346_bl *data, int enable)
{
	if (data->enable_type != ISL22346_BL_ENABLE_NONE) {
		int val;
		if (data->enable_type == ISL22346_BL_ENABLE_GPIO_LOW)
			val = !enable;
		else
			val = !!enable;

		gpio_direction_output(data->enable_gpio, val);
	}
}

static int isl22346_bl_set(struct backlight_device *bl, int brightness)
{
	struct isl22346_bl *data = bl_get_data(bl);
	struct i2c_client *client = data->client;
	int ret = 0;
	int setbrightness;

	if (data->inverted) {
		setbrightness = ISL22346_MAX_BRIGHTNESS - brightness;
	} else {
		setbrightness = brightness;
	}

	if (data->current_brightness != brightness) {
		ret = isl22346_write(client, ISL22346_WR, setbrightness);

		isl22346_bl_enable(data, brightness);
	}

	if (!ret)
		data->current_brightness = brightness;

	return ret;
}

static int isl22346_bl_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;

	return isl22346_bl_set(bl, brightness);
}

static int isl22346_bl_get_brightness(struct backlight_device *bl)
{
	struct isl22346_bl *data = bl_get_data(bl);

	return data->current_brightness;
}

static const struct backlight_ops isl22346_bl_ops = {
	.update_status	= isl22346_bl_update_status,
	.get_brightness	= isl22346_bl_get_brightness,
};

static int isl22346_bl_setup(struct backlight_device *bl)
{
	struct isl22346_bl *data = bl_get_data(bl);
	struct i2c_client *client = data->client;
	int ret = 0;
	uint8_t reg_val;

	ret = isl22346_read(client, ISL22346_WR, &reg_val);

	if (!ret) {
		if (data->inverted)
			data->current_brightness =
				ISL22346_MAX_BRIGHTNESS - reg_val;
		else
			data->current_brightness = reg_val;

		isl22346_bl_enable(data, data->current_brightness);
	} else {
		dev_err(&client->dev, "Couldn't read backlight register\n");
	}

	return ret;
}

static int __devinit isl22346_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct backlight_device *bl;
	struct isl22346_bl *data;
	struct isl22346_bl_platform_data *pdata;
	struct backlight_properties props;
	uint8_t reg_val;
	int ret;

	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		return -EIO;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	pdata = dev_get_platdata(&client->dev);
	if (pdata) {
		dev_info(&client->dev, "Inverted hw values: %d\n",
				pdata->inverted);
		data->inverted = pdata->inverted;

		data->enable_type = pdata->enable_type;
		data->enable_gpio = pdata->enable_gpio;

		if (data->enable_type != ISL22346_BL_ENABLE_NONE) {
			ret = gpio_request(data->enable_gpio, "ISL22346_BL_EN");
			if (ret)
				dev_err(&client->dev,
						"Request error for gpio enable"
						"for isl22346 backlight\n");
		}
	}

	ret = isl22346_read(client, ISL22346_ACR, &reg_val);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to read from IC\n");
		goto err_free;
	}

	props.max_brightness = ISL22346_MAX_BRIGHTNESS;
	props.brightness = 0;
	props.type = BACKLIGHT_RAW;

	data->client = client;
	data->current_brightness = 0;
	i2c_set_clientdata(client, data);

	bl = backlight_device_register(dev_driver_string(&client->dev),
			&client->dev, data, &isl22346_bl_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&client->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_free;
	}

	bl->props.max_brightness =
		bl->props.brightness = ISL22346_MAX_BRIGHTNESS;

	data->bl = bl;

	ret = isl22346_bl_setup(bl);
	if (ret) {
		ret = -EIO;
		goto err_reg;
	}

	backlight_update_status(bl);

	return 0;

err_reg:
	backlight_device_unregister(bl);
err_free:
	kfree(data);

	return ret;
}

static int __devexit isl22346_remove(struct i2c_client *client)
{
	struct isl22346_bl *data = i2c_get_clientdata(client);

	backlight_device_unregister(data->bl);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
static int isl22346_i2c_suspend(struct i2c_client *client, pm_message_t message)
{
	return 0;
}

static int isl22346_i2c_resume(struct i2c_client *client)
{
	return 0;
}
#else
#define isl22346_i2c_suspend NULL
#define isl22346_i2c_resume NULL
#endif

static const struct i2c_device_id isl22346_id[] = {
	{ "isl22346", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, isl22346_id);

static struct i2c_driver isl22346_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.probe    = isl22346_probe,
	.remove   = __devexit_p(isl22346_remove),
	.suspend = isl22346_i2c_suspend,
	.resume  = isl22346_i2c_resume,
	.id_table = isl22346_id,
};

static int __init isl22346_init(void)
{
	return i2c_add_driver(&isl22346_driver);
}
module_init(isl22346_init);

static void __exit isl22346_exit(void)
{
	i2c_del_driver(&isl22346_driver);
}
module_exit(isl22346_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Creative Product Design");
MODULE_DESCRIPTION("ISL22346 Backlight driver");
MODULE_ALIAS("i2c:isl22346-backlight");
