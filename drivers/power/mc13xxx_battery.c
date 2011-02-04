/*
 *
 */

#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/math64.h>
#include <linux/delay.h>

#include <linux/mfd/mc13xxx.h>

#define MC13XXX_IRQSENS0		2
#define MC13XXX_IRQSENS0_CHGDETS	(1 << 7)

#define MC13XXX_ACC0		9
#define MC13XXX_ACC0_STARTCC		(1 << 0)
#define MC13XXX_ACC0_RSTCC		(1 << 1)
#define MC13XXX_ACC0_CCDITHER		(1 << 2)
#define MC13XXX_ACC0_CCCALDB		(1 << 3)
#define MC13XXX_ACC0_CCCALA		(1 << 4)
#define MC13XXX_ACC0_CCFAULT		(1 << 7)
#define MC13XXX_ACC0_CCOUT_SHIFT	8

#define MC13XXX_ACC1		10
#define MC13XXX_ACC1_ONEC_MASK		0x7fff

#define MC13XXX_CHARGER0	48
#define MC13XXX_CHARGER0_VCHRG_SHIFT	0
#define MC13XXX_CHARGER0_VCHRG_MASK	0x7
#define MC13XXX_CHARGER0_ICHRG_SHIFT	3
#define MC13XXX_CHARGER0_ICHRG_MASK	0xf
#define MC13XXX_CHARGER0_TREN		(1 << 7)
#define MC13XXX_CHARGER0_ACLPB		(1 << 8)
#define MC13XXX_CHARGER0_THCHKB		(1 << 9)
#define MC13XXX_CHARGER0_FETOVRD	(1 << 10)
#define MC13XXX_CHARGER0_FETCTRL	(1 << 11)
#define MC13XXX_CHARGER0_RVRSMODE	(1 << 13)
#define MC13XXX_CHARGER0_PLIM_SHIFT	15
#define MC13XXX_CHARGER0_PLIM_MASK	0x3
#define MC13XXX_CHARGER0_CHRGRESTART	(1 << 20)
#define MC13XXX_CHARGER0_CHGAUTOB	(1 << 21)
#define MC13XXX_CHARGER0_CYCLB		(1 << 22)
#define MC13XXX_CHARGER0_CHGAUTOVIB	(1 << 23)

struct mc13xxx_battery {
	struct mc13xxx *mc13xxx;

	struct power_supply battery;
	struct power_supply charger;

	struct delayed_work work;
	struct workqueue_struct *workq;

	int cc_cal;

	bool charger_online;
};


static irqreturn_t mc13xxx_handler_low_battery(int irq, void *data)
{
	struct mc13xxx *mc13xxx = data;

	mc13xxx_irq_ack(mc13xxx, irq);

	return IRQ_HANDLED;
}

static irqreturn_t mc13xxx_chgdet_handler(int irq, void *data)
{
	struct mc13xxx_battery *batt = data;

	mc13xxx_irq_ack(batt->mc13xxx, irq);


	pr_info("%s %d\n", __func__, irq);

	queue_delayed_work(batt->workq, &batt->work, HZ / 50);

	return IRQ_HANDLED;
}

static int signedFrom10bit(unsigned int val)
{
	if (val & 0x200)
		return (0x200 - (val & 0x1ff)) * -1;

	return val & 0x1ff;
}

static int mc13xxx_do_cc_start(struct mc13xxx_battery *batt)
{
	int ret;
	u32 acc0, acc1;

	acc0 = MC13XXX_ACC0_STARTCC | MC13XXX_ACC0_CCCALA |
		MC13XXX_ACC0_CCDITHER;
	ret = mc13xxx_reg_write(batt->mc13xxx, MC13XXX_ACC0, acc0);
	acc1 = 2621;
	ret = mc13xxx_reg_write(batt->mc13xxx, MC13XXX_ACC1, acc1);

	pr_info("starting CC\n");

	return ret;
}

static int mc13xxx_do_cc_stop(struct mc13xxx_battery *batt)
{
	int ret;

	ret = mc13xxx_reg_rmw(batt->mc13xxx, MC13XXX_ACC0,
			MC13XXX_ACC0_STARTCC, 0);

	pr_info("stopping CC\n");

	return ret;
}

static int mc13xxx_do_restart_charging(struct mc13xxx_battery *batt)
{
	int ret;
	u32 val;

	pr_info("%s\n", __func__);

	val = (0x3 & MC13XXX_CHARGER0_VCHRG_MASK) << MC13XXX_CHARGER0_VCHRG_SHIFT;
	val |= (0xf & MC13XXX_CHARGER0_ICHRG_MASK) << MC13XXX_CHARGER0_ICHRG_SHIFT;
	val |= (0x3 & MC13XXX_CHARGER0_PLIM_MASK) << MC13XXX_CHARGER0_PLIM_SHIFT;
	
	val |= MC13XXX_CHARGER0_TREN;
	val |= MC13XXX_CHARGER0_CHRGRESTART;
	val |= MC13XXX_CHARGER0_CHGAUTOVIB;

	ret = mc13xxx_reg_write(batt->mc13xxx, MC13XXX_CHARGER0, val);

	return ret;
}


/*		MSB	LSB
 *	0	chan4	chan0
 *	1	chan5	chan1
 *	2	chan6	chan2
 *	3	chan7	chan3
 */
static int mc13xxx_battery_read_battv(struct mc13xxx *mc13xxx, int *retval)
{
	unsigned int samples[4];
	int ret;

	ret = mc13xxx_adc_do_conversion(mc13xxx, MC13XXX_ADC_MODE_MULT_CHAN,
			0, samples);

	*retval = (samples[0] >> 2) & 0x3ff;

	/* [0, 1023] --> [0,4.8V]
	 * 4687.5 uV per ADC count --> 9375 /2
	 */
	*retval = *retval * 9375 / 2;

	return ret;
}

static int mc13xxx_battery_power(struct mc13xxx *mc13xxx, int *retval)
{
	unsigned int samples[4];
	int ret;
	s64 battv, battc;
	s64 temp;

	ret = mc13xxx_adc_do_conversion(mc13xxx, MC13XXX_ADC_MODE_MULT_CHAN,
			1, samples);

	battv = ((samples[0] >> 2) & 0x3ff); //* 9375 / 2;
	battc = signedFrom10bit((samples[1] >> 2) & 0x3ff);// * 5865;

	pr_info("battv %lld, battc %lld \n", battv, battc);

	battv *= 9375 / 2;
	battc *= 5865;

	pr_info("battv %lld, battc %lld \n", battv, battc);

	temp = div64_s64(battv * battc, 1000000ll);

	*retval = temp;

	return ret;
}

static int mc13xxx_charger_read_vchrg(struct mc13xxx *mc13xxx, int *retval)
{
	unsigned int samples[4];
	int ret;

	ret = mc13xxx_adc_do_conversion(mc13xxx, MC13XXX_ADC_MODE_MULT_CHAN,
			3, samples);

	*retval = (samples[3] >> 2) & 0x3ff;

	pr_info("vchrg %d\n", *retval);

	/* [0, 1023] --> [0, 24V] when CHRGDIVRAW is NOT set
	 * 23,437.5 uV per ADC reading --> 46875 / 2
	 */
	*retval = *retval * 46875 / 2;

	return ret;
}

static int mc13xxx_charger_read_cchrg(struct mc13xxx *mc13xxx, int *retval)
{
	unsigned int samples[4];
	int ret;

	ret = mc13xxx_adc_do_conversion(mc13xxx, MC13XXX_ADC_MODE_MULT_CHAN,
			4, samples);

	*retval = (samples[0] >> 14) & 0x3ff;

	/* [-512, 511] --> [ -300mV, 300mV ] --> [-3A, 3A]
	 * 5.865 mA per ADC count
	 */
	*retval = signedFrom10bit(*retval) * 5865;

	return ret;
}

static int mc13xxx_battery_read_battc(struct mc13xxx *mc13xxx, int *retval)
{
	unsigned int samples[4];
	int ret;

	ret = mc13xxx_adc_do_conversion(mc13xxx, MC13XXX_ADC_MODE_MULT_CHAN,
			1, samples);

	*retval = signedFrom10bit((samples[1] >> 2) & 0x3ff) * 5865;

	return ret;
}

static int mc13xxx_battery_update(struct mc13xxx_battery *batt)
{
	int ret;
	u32 val;
	bool now_online;

	mc13xxx_lock(batt->mc13xxx);
	ret = mc13xxx_reg_read(batt->mc13xxx, MC13XXX_IRQSENS0, &val);

	if (ret)
		goto out;

	now_online = !!(val & MC13XXX_IRQSENS0_CHGDET);
	if (now_online != batt->charger_online) {
		batt->charger_online = now_online;

		dev_info(batt->charger.dev, "charger is %s\n",
				now_online ? "online" : "offline");

		pr_info("cancel \n");
		cancel_delayed_work(&batt->work);
	
		if (now_online) {
			mc13xxx_do_cc_start(batt);
			mc13xxx_do_restart_charging(batt);
		} else {
			mc13xxx_do_cc_stop(batt);
		}

		pr_info("queue work\n");
		queue_delayed_work(batt->workq, &batt->work, HZ * 5);
		pr_info("askdjfasldf\n");
	}

out:
	mc13xxx_unlock(batt->mc13xxx);
	return ret;
}


static void mc13xxx_battery_work(struct work_struct *work)
{
	struct mc13xxx_battery *batt = container_of(work, struct mc13xxx_battery, work.work);

	mc13xxx_battery_update(batt);

	queue_delayed_work(batt->workq, &batt->work, HZ * 5);
}

static int mc13xxx_cc_calibrate(struct mc13xxx_battery *batt)
{
	int ret;
	u32 acc0;
	u32 val = 0;

	mc13xxx_lock(batt->mc13xxx);
	acc0 = MC13XXX_ACC0_STARTCC | MC13XXX_ACC0_RSTCC |
		MC13XXX_ACC0_CCCALA | MC13XXX_ACC0_CCDITHER;
	ret = mc13xxx_reg_write(batt->mc13xxx, MC13XXX_ACC0, acc0);
	if (ret)
		goto out;

	msleep(100);

	ret = mc13xxx_reg_write(batt->mc13xxx, MC13XXX_ACC1, 2621);
	if (ret)
		goto out;

	ret = mc13xxx_reg_read(batt->mc13xxx, MC13XXX_ACC0, &val);
	if (ret)
		goto out;

	pr_info("val reg is %u\n", val);
	batt->cc_cal = (int)((s16)(val >> MC13XXX_ACC0_CCOUT_SHIFT));

	pr_info("cal value is %d\n", batt->cc_cal);

	acc0 = MC13XXX_ACC0_RSTCC;
	ret = mc13xxx_reg_write(batt->mc13xxx, MC13XXX_ACC0, acc0);

out:
	mc13xxx_unlock(batt->mc13xxx);
	return ret;
}


static int mc13xxx_cc_get_coulombs(struct mc13xxx_battery *batt, int *coulombs)
{
	int ret;
	u32 val;

	mc13xxx_lock(batt->mc13xxx);

	ret = mc13xxx_reg_read(batt->mc13xxx, MC13XXX_ACC0, &val);
	if (ret)
		goto out;

	*coulombs = (int)((s16)(val >> MC13XXX_ACC0_CCOUT_SHIFT));

out:
	mc13xxx_unlock(batt->mc13xxx);
	return ret;
}


static enum power_supply_property mc13xxx_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

#define to_mc13xxx_charger(x) container_of((x), struct mc13xxx_battery, \
		charger);

static int mc13xxx_charger_get_property(struct power_supply *ps,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;
	struct mc13xxx_battery *batt = to_mc13xxx_charger(ps);

	pr_info("%s %d from (%pf)\n", __func__, psp, __builtin_return_address(0));

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = batt->charger_online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = mc13xxx_charger_read_vchrg(batt->mc13xxx,
				&val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = mc13xxx_charger_read_cchrg(batt->mc13xxx,
				&val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 15000000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = 4100000;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}




#define to_mc13xxx_battery(x) container_of((x), struct mc13xxx_battery, \
		battery);

static int mc13xxx_battery_get_property(struct power_supply *ps,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;
	struct mc13xxx_battery *batt = to_mc13xxx_battery(ps);

	pr_info("%s %d from (%pf)\n", __func__, psp, __builtin_return_address(0));

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = mc13xxx_battery_read_battv(batt->mc13xxx,
				&val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = mc13xxx_battery_read_battc(batt->mc13xxx,
				&val->intval);
		break;
	case POWER_SUPPLY_PROP_POWER_NOW:
		ret = mc13xxx_battery_power(batt->mc13xxx,
				&val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 3800000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = 3400000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = mc13xxx_cc_get_coulombs(batt, &val->intval);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}


static enum power_supply_property mc13xxx_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
};


static int __devinit mc13xxx_battery_probe(struct platform_device *pdev)
{
	struct mc13xxx_battery *batt;
	int ret;

	batt = kzalloc(sizeof(*batt), GFP_KERNEL);
	if (batt == NULL)
		return -ENOMEM;

	batt->mc13xxx = dev_get_drvdata(pdev->dev.parent);
	platform_set_drvdata(pdev, batt);

	INIT_DELAYED_WORK(&batt->work, mc13xxx_battery_work);

	batt->workq = create_singlethread_workqueue("mc13xxx_battery");
	if (!batt->workq) {
		ret = -ESRCH;
		goto workq_err;
	}

	batt->mc13xxx->adcflags |= MC13XXX_ADC_MEASURE_CHARGER;

	batt->battery.name = "mc13xxx_battery";
	batt->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	batt->battery.properties = mc13xxx_battery_props;
	batt->battery.num_properties = ARRAY_SIZE(mc13xxx_battery_props);
	batt->battery.get_property = mc13xxx_battery_get_property;

	ret = power_supply_register(&pdev->dev, &batt->battery);
	if (ret)
		goto battery_err;
	
	batt->mc13xxx->adcflags |= MC13XXX_ADC_MEASURE_BATT;

	batt->charger.name = "mc13xxx_charger";
	batt->charger.type = POWER_SUPPLY_TYPE_MAINS;
	batt->charger.properties = mc13xxx_charger_props;
	batt->charger.num_properties = ARRAY_SIZE(mc13xxx_charger_props);
	batt->charger.get_property = mc13xxx_charger_get_property;

	ret = power_supply_register(&pdev->dev, &batt->charger);
	if (ret)
		goto charger_err;


	mc13xxx_cc_calibrate(batt);

	mc13xxx_lock(batt->mc13xxx);
	mc13xxx_irq_ack(batt->mc13xxx, MC13XXX_IRQ_CHGDET);
	mc13xxx_irq_request(batt->mc13xxx, MC13XXX_IRQ_CHGDET,
			mc13xxx_chgdet_handler, "mc13xxx_charge", batt);
	mc13xxx_unlock(batt->mc13xxx);

	queue_delayed_work(batt->workq, &batt->work, HZ * 5);

	return 0;

charger_err:
	power_supply_unregister(&batt->battery);
battery_err:
	destroy_workqueue(batt->workq);
workq_err:
	kfree(batt);

	return ret;
}

static int __devexit mc13xxx_battery_remove(struct platform_device *pdev)
{
	struct mc13xxx_battery *batt = platform_get_drvdata(pdev);

	mc13xxx_lock(batt->mc13xxx);
	mc13xxx_irq_free(batt->mc13xxx, MC13XXX_IRQ_CHGDET, batt);
	mc13xxx_unlock(batt->mc13xxx);

	cancel_delayed_work_sync(&batt->work);
	destroy_workqueue(batt->workq);

	power_supply_unregister(&batt->charger);
	power_supply_unregister(&batt->battery);

	platform_set_drvdata(pdev, NULL);
	kfree(batt);

	return 0;
}

static struct platform_driver mc13xxx_battery_driver = {
	.driver = {
		.name = "mc13xxx-battery",
		.owner = THIS_MODULE,
	},
	.remove = __exit_p(mc13xxx_battery_remove),
};

static int __init mc13xxx_battery_init(void)
{
	pr_info("mc13xxx battery driver\n");
	return platform_driver_probe(&mc13xxx_battery_driver,
			mc13xxx_battery_probe);
}
subsys_initcall(mc13xxx_battery_init);

static void __exit mc13xxx_battery_exit(void)
{
	platform_driver_unregister(&mc13xxx_battery_driver);
}
module_exit(mc13xxx_battery_exit);


MODULE_AUTHOR("Marc Reilly");
MODULE_DESCRIPTION("mc13xxx Battery Charger Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mc13xxx_battery");
