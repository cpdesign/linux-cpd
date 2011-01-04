/*
 * Copyright 2009-2010 Pengutronix
 * Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>
 *
 * loosely based on an earlier driver that has
 * Copyright 2009 Pengutronix, Sascha Hauer <s.hauer@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/mfd/core.h>
#include <linux/mfd/mc13xxx.h>

#define MC13XXX_REGOFFSET_SHIFT 25
static int mc13xxx_spi_reg_read(struct mc13xxx *mc13xxx,
				unsigned int offset, u32 *val)
{
	struct spi_transfer t;
	struct spi_message m;
	int ret;

	*val = offset << MC13XXX_REGOFFSET_SHIFT;

	memset(&t, 0, sizeof(t));

	t.tx_buf = val;
	t.rx_buf = val;
	t.len = sizeof(u32);

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	ret = spi_sync(mc13xxx->spidev, &m);

	/* error in message.status implies error return from spi_sync */
	BUG_ON(!ret && m.status);

	if (ret)
		return ret;

	*val &= 0xffffff;

	return 0;
}

static int mc13xxx_spi_reg_write(struct mc13xxx *mc13xxx, unsigned int offset,
		u32 val)
{
	u32 buf;
	struct spi_transfer t;
	struct spi_message m;
	int ret;

	buf = 1 << 31 | offset << MC13XXX_REGOFFSET_SHIFT | val;

	memset(&t, 0, sizeof(t));

	t.tx_buf = &buf;
	t.rx_buf = &buf;
	t.len = sizeof(u32);

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	ret = spi_sync(mc13xxx->spidev, &m);

	BUG_ON(!ret && m.status);

	if (ret)
		return ret;

	return 0;
}

static int mc13xxx_spi_probe(struct spi_device *spi)
{
	struct mc13xxx *mc13xxx;
	struct mc13xxx_platform_data *pdata = dev_get_platdata(&spi->dev);
	int ret;

	mc13xxx = kzalloc(sizeof(*mc13xxx), GFP_KERNEL);
	if (!mc13xxx)
		return -ENOMEM;

	dev_set_drvdata(&spi->dev, mc13xxx);
	spi->mode = SPI_MODE_0 | SPI_CS_HIGH;
	spi->bits_per_word = 32;
	spi_setup(spi);

	mc13xxx->dev = &spi->dev;
	mc13xxx->spidev = spi;
	mc13xxx->read_dev = mc13xxx_spi_reg_read;
	mc13xxx->write_dev = mc13xxx_spi_reg_write;

	ret = mc13xxx_common_init(mc13xxx, pdata, spi->irq);

	if (ret) {
		dev_set_drvdata(&spi->dev, NULL);
	} else {
		const struct spi_device_id *devid =
			spi_get_device_id(mc13xxx->spidev);
		if (!devid || devid->driver_data != mc13xxx->ictype)
			dev_warn(mc13xxx->dev,
				"device id doesn't match auto detection!\n");
	}

	return ret;
}

static int __devexit mc13xxx_spi_remove(struct spi_device *spi)
{
	struct mc13xxx *mc13xxx = dev_get_drvdata(&spi->dev);

	free_irq(mc13xxx->spidev->irq, mc13xxx);

	mfd_remove_devices(&spi->dev);

	kfree(mc13xxx);

	return 0;
}

static const struct spi_device_id mc13xxx_device_id[] = {
	{
		.name = "mc13783",
		.driver_data = MC13XXX_ID_MC13783,
	}, {
		.name = "mc13892",
		.driver_data = MC13XXX_ID_MC13892,
	}, {
		/* sentinel */
	}
};

static struct spi_driver mc13xxx_spi_driver = {
	.id_table = mc13xxx_device_id,
	.driver = {
		.name = "mc13xxx",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = mc13xxx_spi_probe,
	.remove = __devexit_p(mc13xxx_spi_remove),
};

static int __init mc13xxx_spi_init(void)
{
	return spi_register_driver(&mc13xxx_spi_driver);
}
subsys_initcall(mc13xxx_spi_init);

static void __exit mc13xxx_spi_exit(void)
{
	spi_unregister_driver(&mc13xxx_spi_driver);
}
module_exit(mc13xxx_spi_exit);

MODULE_DESCRIPTION("Core driver for Freescale MC13XXX PMIC");
MODULE_AUTHOR("Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>");
MODULE_LICENSE("GPL v2");
