/*
 * Driver for the Freescale Semiconductor MC13XXX touchscreen.
 *
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2009 Sascha Hauer, Pengutronix
 * Copyright 2011 Marc Reilly, Creative Product Design
 *
 * Initial development of this code was funded by
 * Phytec Messtechnik GmbH, http://www.phytec.de/
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/platform_device.h>
#include <linux/mfd/mc13xxx.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>

#define MC13XXX_TS_NAME	"mc13xxx-ts"

#define DEFAULT_SAMPLE_TOLERANCE 300

static unsigned int sample_tolerance = DEFAULT_SAMPLE_TOLERANCE;
module_param(sample_tolerance, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(sample_tolerance,
		"If the minimal and maximal value read out for one axis "
		"differ by this value (default: "
		__stringify(DEFAULT_SAMPLE_TOLERANCE) ") or more, the reading "
		"is supposed to be wrong and is discarded.  Set to 0 to "
		"disable this check.");

struct mc13xxx_ts_priv {
	struct input_dev *idev;
	struct mc13xxx *mc13xxx;

	struct delayed_work work;
	struct workqueue_struct *workq;

	unsigned int sample[4];

	bool (*convert_and_check_samples)(struct mc13xxx_ts_priv *priv,
			int *x, int *y, int *pressure);
};

static irqreturn_t mc13xxx_ts_handler(int irq, void *data)
{
	struct mc13xxx_ts_priv *priv = data;

	mc13xxx_irq_ack(priv->mc13xxx, irq);

	/*
	 * Kick off reading coordinates. Note that if work happens already
	 * be queued for future execution (it rearms itself) it will not
	 * be rescheduled for immediate execution here. However the rearm
	 * delay is HZ / 50 which is acceptable.
	 */
	queue_delayed_work(priv->workq, &priv->work, 0);

	return IRQ_HANDLED;
}

#define sort3(a0, a1, a2) ({						\
		if (a0 > a1)						\
			swap(a0, a1);					\
		if (a1 > a2)						\
			swap(a1, a2);					\
		if (a0 > a1)						\
			swap(a0, a1);					\
		})

static bool mc13783_convert_and_check_samples(struct mc13xxx_ts_priv *priv,
		int *x, int *y, int *pressure)
{
	int x0, x1, x2, y0, y1, y2;
	int cr0, cr1;

	/*
	 * the values are 10-bit wide only, but the two least significant
	 * bits are for future 12 bit use and reading yields 0
	 */
	x0 = priv->sample[0] & 0xfff;
	x1 = priv->sample[1] & 0xfff;
	x2 = priv->sample[2] & 0xfff;
	y0 = priv->sample[3] & 0xfff;
	y1 = (priv->sample[0] >> 12) & 0xfff;
	y2 = (priv->sample[1] >> 12) & 0xfff;
	cr0 = (priv->sample[2] >> 12) & 0xfff;
	cr1 = (priv->sample[3] >> 12) & 0xfff;

	dev_dbg(&priv->idev->dev,
		"x: (% 4d,% 4d,% 4d) y: (% 4d, % 4d,% 4d) cr: (% 4d, % 4d)\n",
		x0, x1, x2, y0, y1, y2, cr0, cr1);

	sort3(x0, x1, x2);
	sort3(y0, y1, y2);

	if (sample_tolerance && ((x2 - x0 >= sample_tolerance)
				|| (y2 - y0 >= sample_tolerance)))
		return false;

	/* report the median coordinate and average pressure */
	*x = x1;
	*y = y1;
	cr0 = (cr0 + cr1) / 2;
	*pressure = cr0 ? 0x1000 - cr0 : cr0;

	return true;
}

/*
 * Any ADC readings for contact pressure are only valid below if they are
 * below TOUCHED_ADC_THRESH. We don't use the full scale of the ADC because
 * the touchscreen doesn't always read 0xfff when the ts is not touched.
 */
#define TOUCHED_ADC_THRESH 2048
static bool mc13892_convert_and_check_samples(struct mc13xxx_ts_priv *priv,
		int *x, int *y, int *pressure)
{
	int x0, x1, y0, y1;
	int cr0, cr1;
	bool both_cr_invalid;

	/*
	 * the values are 10-bit wide only, but the two least significant
	 * bits are for future 12 bit use and reading yields 0
	 */
	x0 = priv->sample[0] & 0xfff;
	x1 = priv->sample[1] & 0xfff;
	y0 = priv->sample[3] & 0xfff;
	y1 = (priv->sample[0] >> 12) & 0xfff;
	cr0 = (priv->sample[2] >> 12) & 0xfff;
	cr1 = (priv->sample[3] >> 12) & 0xfff;

	dev_dbg(&priv->idev->dev,
		"x: (%4d,%4d) y: (%4d, %4d) cr: (%4d, %4d)\n",
		x0, x1, y0, y1, cr0, cr1);

	/*
	 * if both cr values are above threshold then the pen up has occurred,
	 * but the values can often be quite far apart, so don't filter them
	 * out based on sample tolerance in this case.
	 */
	both_cr_invalid = (cr0 > TOUCHED_ADC_THRESH)
			&& (cr1 > TOUCHED_ADC_THRESH);

	if (sample_tolerance &&
			(((abs(x1 - x0) >= sample_tolerance)
			|| (abs(y1 - y0) >= sample_tolerance))
			|| (!both_cr_invalid
				&& (abs(cr0 - cr1) >= sample_tolerance))))
		return false;

	*x = (x0 + x1) / 2;
	*y = (y0 + y1) / 2;
	cr0 = (cr0 + cr1) / 2;

	/* turn contact resistance into pressure. */
	if (both_cr_invalid || (cr0 >= TOUCHED_ADC_THRESH))
		*pressure = 0;
	else
		*pressure = TOUCHED_ADC_THRESH - cr0;

	return true;
}

static void mc13xxx_ts_report_sample(struct mc13xxx_ts_priv *priv)
{
	struct input_dev *idev = priv->idev;
	int x, y, pressure;

	if (priv->convert_and_check_samples(priv, &x, &y, &pressure)) {
		if (pressure) {
			input_report_abs(idev, ABS_X, x);
			input_report_abs(idev, ABS_Y, y);

			dev_dbg(&idev->dev, "report (%d, %d, %d)\n",
					x, y, pressure);
			queue_delayed_work(priv->workq, &priv->work, HZ / 50);
		} else
			dev_dbg(&idev->dev, "report release\n");

		input_report_abs(idev, ABS_PRESSURE, pressure);
		input_report_key(idev, BTN_TOUCH, pressure ? 1 : 0);
		input_sync(idev);
	} else
		dev_dbg(&idev->dev, "discard event\n");
}

static void mc13xxx_ts_work(struct work_struct *work)
{
	struct mc13xxx_ts_priv *priv =
		container_of(work, struct mc13xxx_ts_priv, work.work);
	unsigned int mode = MC13XXX_ADC_MODE_TS;
	unsigned int channel = 12;

	if (mc13xxx_adc_do_conversion(priv->mc13xxx,
				mode, channel, priv->sample) == 0)
		mc13xxx_ts_report_sample(priv);
}

static int mc13xxx_ts_open(struct input_dev *dev)
{
	struct mc13xxx_ts_priv *priv = input_get_drvdata(dev);
	int ret;

	mc13xxx_lock(priv->mc13xxx);

	mc13xxx_irq_ack(priv->mc13xxx, MC13XXX_IRQ_TS);

	ret = mc13xxx_irq_request(priv->mc13xxx, MC13XXX_IRQ_TS,
		mc13xxx_ts_handler, MC13XXX_TS_NAME, priv);
	if (ret)
		goto out;

	ret = mc13xxx_reg_rmw(priv->mc13xxx, MC13XXX_ADC0,
			MC13XXX_ADC0_TSMOD_MASK, MC13XXX_ADC0_TSMOD0);
	if (ret)
		mc13xxx_irq_free(priv->mc13xxx, MC13XXX_IRQ_TS, priv);
out:
	mc13xxx_unlock(priv->mc13xxx);
	return ret;
}

static void mc13xxx_ts_close(struct input_dev *dev)
{
	struct mc13xxx_ts_priv *priv = input_get_drvdata(dev);

	mc13xxx_lock(priv->mc13xxx);
	mc13xxx_reg_rmw(priv->mc13xxx, MC13XXX_ADC0,
			MC13XXX_ADC0_TSMOD_MASK, 0);
	mc13xxx_irq_free(priv->mc13xxx, MC13XXX_IRQ_TS, priv);
	mc13xxx_unlock(priv->mc13xxx);

	cancel_delayed_work_sync(&priv->work);
}

#define DRIVERID_MC13783	0
#define DRIVERID_MC13892	1

static int __init mc13xxx_ts_probe(struct platform_device *pdev)
{
	struct mc13xxx_ts_priv *priv;
	struct input_dev *idev;
	const struct platform_device_id *devid;
	int ret = -ENOMEM;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	idev = input_allocate_device();
	if (!priv || !idev)
		goto err_free_mem;

	INIT_DELAYED_WORK(&priv->work, mc13xxx_ts_work);
	priv->mc13xxx = dev_get_drvdata(pdev->dev.parent);
	priv->idev = idev;

	/*
	 * We need separate workqueue because mc13xxx_adc_do_conversion
	 * uses keventd and thus would deadlock.
	 */
	priv->workq = create_singlethread_workqueue("mc13xxx_ts");
	if (!priv->workq)
		goto err_free_mem;

	idev->name = MC13XXX_TS_NAME;
	idev->dev.parent = &pdev->dev;

	idev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	idev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(idev, ABS_X, 0, 0xfff, 0, 0);
	input_set_abs_params(idev, ABS_Y, 0, 0xfff, 0, 0);

	idev->open = mc13xxx_ts_open;
	idev->close = mc13xxx_ts_close;

	devid = platform_get_device_id(pdev);

	if (devid->driver_data == DRIVERID_MC13783) {
		input_set_abs_params(idev, ABS_PRESSURE, 0, 0xfff, 0, 0);
		priv->convert_and_check_samples =
				mc13783_convert_and_check_samples;
	} else if (devid->driver_data == DRIVERID_MC13892) {
		input_set_abs_params(idev, ABS_PRESSURE, 0,
				TOUCHED_ADC_THRESH - 1, 0, 0);
		priv->convert_and_check_samples =
				mc13892_convert_and_check_samples;
	} else {
		ret = -ENODEV;
		goto err_destroy_wq;
	}

	input_set_drvdata(idev, priv);

	ret = input_register_device(priv->idev);
	if (ret) {
		dev_err(&pdev->dev,
			"register input device failed with %d\n", ret);
		goto err_destroy_wq;
	}

	platform_set_drvdata(pdev, priv);
	return 0;

err_destroy_wq:
	destroy_workqueue(priv->workq);
err_free_mem:
	input_free_device(idev);
	kfree(priv);
	return ret;
}

static int __devexit mc13xxx_ts_remove(struct platform_device *pdev)
{
	struct mc13xxx_ts_priv *priv = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	destroy_workqueue(priv->workq);
	input_unregister_device(priv->idev);
	kfree(priv);

	return 0;
}
const struct platform_device_id mc13xxx_ts_idtable[] = {
	{
		.name = "mc13783-ts",
		.driver_data = DRIVERID_MC13783,
	}, {
		.name = "mc13892-ts",
		.driver_data = DRIVERID_MC13892,
	},
	{ }
};

static struct platform_driver mc13xxx_ts_driver = {
	.id_table	= mc13xxx_ts_idtable,
	.remove		= __devexit_p(mc13xxx_ts_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= MC13XXX_TS_NAME,
	},
};

static int __init mc13xxx_ts_init(void)
{
	return platform_driver_probe(&mc13xxx_ts_driver, &mc13xxx_ts_probe);
}
module_init(mc13xxx_ts_init);

static void __exit mc13xxx_ts_exit(void)
{
	platform_driver_unregister(&mc13xxx_ts_driver);
}
module_exit(mc13xxx_ts_exit);

MODULE_DESCRIPTION("MC13xxx input touchscreen driver");
MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" MC13XXX_TS_NAME);
