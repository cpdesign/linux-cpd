/*
 * Backlight driver based on isl22316 digital pot
 *
 * Copyright 2011 Creative Product Design
 *
 * Started from code:
 * Copyright 2009-2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
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

#define ISL22316_WR	0
#define ISL22316_IVR	0
#define ISL22316_ACR	2

#define ISL22316_MAX_BRIGHTNESS 127

struct isl22316_bl {
	struct i2c_client *client;
	struct backlight_device *bl;
	int current_brightness;
};

static int isl22316_read(struct i2c_client *client, int reg, uint8_t *val)
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

static int isl22316_write(struct i2c_client *client, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(client, reg, val);
}

static int isl22316_bl_set(struct backlight_device *bl, int brightness)
{
	struct isl22316_bl *data = bl_get_data(bl);
	struct i2c_client *client = data->client;
	int ret = 0;

	if (data->current_brightness != brightness)
		ret |= isl22316_write(client, ISL22316_WR, brightness);

	if (!ret)
		data->current_brightness = brightness;

	return ret;
}

static int isl22316_bl_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;
	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	return isl22316_bl_set(bl, brightness);
}

static int isl22316_bl_get_brightness(struct backlight_device *bl)
{
	struct isl22316_bl *data = bl_get_data(bl);

	return data->current_brightness;
}

static const struct backlight_ops isl22316_bl_ops = {
	.update_status	= isl22316_bl_update_status,
	.get_brightness	= isl22316_bl_get_brightness,
};

static int isl22316_bl_setup(struct backlight_device *bl)
{
	struct isl22316_bl *data = bl_get_data(bl);
	struct i2c_client *client = data->client;
	int ret = 0;
	uint8_t reg_val;

	ret = isl22316_read(client, ISL22316_WR, &reg_val);

	if (!ret)
		data->current_brightness = reg_val & 0x7f;

	return ret;
}

static int __devinit isl22316_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct backlight_device *bl;
	struct isl22316_bl *data;
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

	ret = isl22316_read(client, ISL22316_ACR, &reg_val);
	if (ret < 0)
		goto err_free;

	data->client = client;
	data->current_brightness = 0;
	i2c_set_clientdata(client, data);

	bl = backlight_device_register(dev_driver_string(&client->dev),
			&client->dev, data, &isl22316_bl_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&client->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_free;
	}

	bl->props.max_brightness = ISL22316_MAX_BRIGHTNESS;
	bl->props.brightness = ISL22316_MAX_BRIGHTNESS;

	data->bl = bl;

	ret = isl22316_bl_setup(bl);
	if (ret) {
		ret = -EIO;
		goto err_reg;
	}

	bl->props.brightness = data->current_brightness;

	backlight_update_status(bl);

	return 0;

err_reg:
	backlight_device_unregister(bl);
err_free:
	kfree(data);

	return ret;
}

static int __devexit isl22316_remove(struct i2c_client *client)
{
	struct isl22316_bl *data = i2c_get_clientdata(client);

	backlight_device_unregister(data->bl);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
static int isl22316_i2c_suspend(struct i2c_client *client, pm_message_t message)
{
	return 0;
}

static int isl22316_i2c_resume(struct i2c_client *client)
{
	return 0;
}
#else
#define isl22316_i2c_suspend NULL
#define isl22316_i2c_resume NULL
#endif

static const struct i2c_device_id isl22316_id[] = {
	{ "isl22316", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, isl22316_id);

static struct i2c_driver isl22316_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.probe    = isl22316_probe,
	.remove   = __devexit_p(isl22316_remove),
	.suspend = isl22316_i2c_suspend,
	.resume  = isl22316_i2c_resume,
	.id_table = isl22316_id,
};

static int __init isl22316_init(void)
{
	return i2c_add_driver(&isl22316_driver);
}
module_init(isl22316_init);

static void __exit isl22316_exit(void)
{
	i2c_del_driver(&isl22316_driver);
}
module_exit(isl22316_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Creative Product Design");
MODULE_DESCRIPTION("ISL22316 Backlight driver");
MODULE_ALIAS("i2c:isl22316-backlight");
