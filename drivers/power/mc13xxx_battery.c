/*
 *
 */

//#define DEBUG

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
#define MC13XXX_IRQSENS0_CHGDET		(1 << 6)
#define MC13XXX_IRQSENS0_CCCV		(1 << 10)
#define MC13XXX_IRQSENS0_CHGCURR	(1 << 11)
#define MC13XXX_IRQSENS0_LOBATL		(1 << 13)
#define MC13XXX_IRQSENS0_LOBATH		(1 << 14)

#define MC13XXX_IRQSENS1		5
#define MC13XXX_IRQSENS1_BATTDETB	(1 << 22)

#define MC13XXX_PUMSENS		6

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

#define MC13XXX_POWER0		13
#define MC13XXX_POWER0_BPSNS_SHIFT	16
#define MC13XXX_POWER0_BPSNS_MASK	(0x3 << MC13XXX_POWER0_BPSNS_SHIFT)
#define MC13XXX_POWER0_BATTDETEN	(1 << 19)
#define MC13XXX_POWER0_VCOIN_SHIFT	20
#define MC13XXX_POWER0_VCOIN_MASK	(0x7 << MC13XXX_POWER0_VCOIN_SHIFT)
#define MC13XXX_POWER0_COINCHEN		(1 << 23)

#define MC13XXX_CHARGER0	48
#define MC13XXX_CHARGER0_VCHRG_SHIFT	0
#define MC13XXX_CHARGER0_VCHRG_MASK	(0x7 << MC13XXX_CHARGER0_VCHRG_SHIFT)
#define MC13XXX_CHARGER0_ICHRG_SHIFT	3
#define MC13XXX_CHARGER0_ICHRG_MASK	(0xf << MC13XXX_CHARGER0_ICHRG_SHIFT)
#define MC13XXX_CHARGER0_TREN		(1 << 7)
#define MC13XXX_CHARGER0_ACLPB		(1 << 8)
#define MC13XXX_CHARGER0_THCHKB		(1 << 9)
#define MC13XXX_CHARGER0_FETOVRD	(1 << 10)
#define MC13XXX_CHARGER0_FETCTRL	(1 << 11)
#define MC13XXX_CHARGER0_RVRSMODE	(1 << 13)
#define MC13XXX_CHARGER0_PLIM_SHIFT	15
#define MC13XXX_CHARGER0_PLIM_MASK	(0x3 << MC13XXX_CHARGER0_PLIM_SHIFT)
#define MC13XXX_CHARGER0_PLIMDIS	(1 << 17)
#define MC13XXX_CHARGER0_CHRGLENEN	(1 << 18)
#define MC13XXX_CHARGER0_CHGTMRRST	(1 << 19)
#define MC13XXX_CHARGER0_CHRGRESTART	(1 << 20)
#define MC13XXX_CHARGER0_CHGAUTOB	(1 << 21)
#define MC13XXX_CHARGER0_CYCLB		(1 << 22)
#define MC13XXX_CHARGER0_CHGAUTOVIB	(1 << 23)

#define MC13XXX_BATTERY_CC_CAL_MS	100
#define MC13XXX_BATTERY_CC_ONEC_VAL	(2621 * 2)
#define MC13XXX_BATTERY_CHARGE_FULL	10000000

struct mc13xxx_battery {
	struct mc13xxx *mc13xxx;

	struct power_supply battery;
	struct power_supply charger;
	struct power_supply coincell;

	struct delayed_work work;
	struct workqueue_struct *workq;

	int cc_cal;
	struct timespec cc_start_time;
	int cc_read_val;
	bool cc_was_full;

	int battv;
	int battc;	/* positive is from battery */

	int chrgv;
	int chrgc;	/* positive is into battery */

	int bpv;
	int coincellv;

	/* last ms count a sample was obtained*/
	unsigned int sample_ms;

	bool charger_online;
	struct timespec charge_start_time;

	bool battery_online;

	bool cccv;
	bool chgcurr;
	int batt_status;
	int batt_capacity_level;
	int full_count;
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

static irqreturn_t mc13xxx_chgfault_handler(int irq, void *data)
{
	struct mc13xxx_battery *batt = data;

	mc13xxx_irq_ack(batt->mc13xxx, irq);

	return IRQ_HANDLED;
}

/*
 * When charge current drops below threshold
 */
static irqreturn_t mc13xxx_chgcurr_handler(int irq, void *data)
{
	struct mc13xxx_battery *batt = data;

	mc13xxx_irq_ack(batt->mc13xxx, irq);

	pr_info("%s %d\n", __func__, irq);

	return IRQ_HANDLED;
}

/*
 *
 */
static irqreturn_t mc13xxx_cccv_handler(int irq, void *data)
{
	struct mc13xxx_battery *batt = data;

	mc13xxx_irq_ack(batt->mc13xxx, irq);

	pr_info("%s %d\n", __func__, irq);

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------------ */

static int signedFrom10bit(unsigned int val)
{
	if (val & 0x200)
		return (0x200 - (val & 0x1ff)) * -1;

	return val & 0x1ff;
}

/*
 * NOTE: all these mc13xxx_do functions require the lock to
 * be already held
 */
static int mc13xxx_do_cc_start(struct mc13xxx_battery *batt, int calibrate)
{
	int ret;
	u32 acc0, acc1;

	acc0 = MC13XXX_ACC0_STARTCC | MC13XXX_ACC0_CCDITHER | MC13XXX_ACC0_RSTCC;

	if (calibrate) {
		acc0 |= MC13XXX_ACC0_CCCALA;
		acc1 = 1;
	} else
		acc1 = MC13XXX_BATTERY_CC_ONEC_VAL;

	ret = mc13xxx_reg_write(batt->mc13xxx, MC13XXX_ACC1, acc1);
	ret = mc13xxx_reg_write(batt->mc13xxx, MC13XXX_ACC0, acc0);

	getrawmonotonic(&batt->cc_start_time);

	return ret;
}

static int mc13xxx_do_cc_stop(struct mc13xxx_battery *batt)
{
	int ret;

	ret = mc13xxx_reg_rmw(batt->mc13xxx, MC13XXX_ACC0,
			MC13XXX_ACC0_CCCALA | MC13XXX_ACC0_STARTCC, 0);

	pr_info("stopping CC\n");

	return ret;
}

static int mc13xxx_do_cc_set_full(struct mc13xxx_battery *batt)
{
	batt->cc_was_full = true;

	return mc13xxx_do_cc_start(batt, 0);
}

static int mc13xxx_do_restart_charging(struct mc13xxx_battery *batt)
{
	int ret;
	u32 val;

	pr_info("%s\n", __func__);

	val = (0x3 << MC13XXX_CHARGER0_VCHRG_SHIFT) & MC13XXX_CHARGER0_VCHRG_MASK;
	val |= (0xe << MC13XXX_CHARGER0_ICHRG_SHIFT) & MC13XXX_CHARGER0_ICHRG_MASK;
	val |= (0x3 << MC13XXX_CHARGER0_PLIM_SHIFT) & MC13XXX_CHARGER0_PLIM_MASK;

	val |= MC13XXX_CHARGER0_TREN;
	val |= MC13XXX_CHARGER0_CHRGRESTART;
	val |= MC13XXX_CHARGER0_CHGAUTOVIB;
//	val |= MC13XXX_CHARGER0_CHGTMRRST;
	val |= MC13XXX_CHARGER0_THCHKB;
	val |= MC13XXX_CHARGER0_CYCLB;

	ret = mc13xxx_reg_write(batt->mc13xxx, MC13XXX_CHARGER0, val);

	getrawmonotonic(&batt->charge_start_time);

	return ret;
}

static int mc13xxx_do_battery_init(struct mc13xxx_battery *batt)
{
	int ret;
	u32 val = 0;
	u32 mask = 0;

	/* Battery detect enable */
	val |= MC13XXX_POWER0_BATTDETEN;
	mask |= MC13XXX_POWER0_BATTDETEN;
	/* BPSNS = b11 --> LOBATL = 3.1, LOBATH = 3.4 */
	val |= 0x3 << MC13XXX_POWER0_BPSNS_SHIFT;
	mask |= MC13XXX_POWER0_BPSNS_MASK;

	ret = mc13xxx_reg_rmw(batt->mc13xxx, MC13XXX_POWER0, mask, val);

	return ret;
}

static int mc13xxx_do_coincell_init(struct mc13xxx_battery *batt)
{
	int ret;
	u32 mask = 0;
	u32 val = 0;

	/* VCOIN 0b111 = 3.30V */
	val |= (0x7 << MC13XXX_POWER0_VCOIN_SHIFT) & MC13XXX_POWER0_VCOIN_MASK;
	mask |= MC13XXX_POWER0_VCOIN_MASK;

	val |= MC13XXX_POWER0_COINCHEN;
	mask |= MC13XXX_POWER0_COINCHEN;

	ret = mc13xxx_reg_rmw(batt->mc13xxx, MC13XXX_POWER0, mask, val);

	return 0;
}

/*
 *	sample	MSB	  LSB
 *	0	chan4	chan0
 *	1	chan5	chan1
 *	2	chan6	chan2
 *	3	chan7	chan3
 */
static int extract_sample(unsigned int *samples, int channel, int is_signed)
{
	unsigned int tmp;
	int shift = channel < 4 ? 2 : 14;

	tmp = (samples[channel % 4] >> shift) & 0x3ff;

	if (is_signed)
		return signedFrom10bit(tmp);

	return tmp;
}

static int mc13xxx_read_batt_single(struct mc13xxx_battery* batt,
		int *battv_ret, int *battc_ret)
{
	unsigned int samples[4];
	int ret, i;

	int battv[4], battv_avg = 0;
	int battc[4], battc_avg = 0;

	ret = mc13xxx_adc_do_conversion(batt->mc13xxx,
			MC13XXX_ADC_MODE_SINGLE_CHAN, 1, samples);

	if (ret == -EBUSY) {
		msleep(20);
		ret = mc13xxx_adc_do_conversion(batt->mc13xxx,
				MC13XXX_ADC_MODE_SINGLE_CHAN, 1, samples);
	}

	for (i = 0; i < 4; ++i) {
		battv[i] = extract_sample(samples, (i * 2), 0);
		battc[i] = extract_sample(samples, (i * 2) + 1, 1);

		battv_avg += battv[i];
		battc_avg += battc[i];
	}

	battv_avg = (battv_avg / 4) * 9375 / 2;
	battc_avg = (battc_avg / 4) * 5865;

	*battv_ret = battv_avg;
	*battc_ret = battc_avg;

	return ret;
}

static int mc13xxx_update_readings(struct mc13xxx_battery *batt,
		unsigned int since)
{
	unsigned int samples[4];
	int ret;

	unsigned int ms = jiffies_to_msecs(jiffies);
	if (ms - batt->sample_ms < since)
		return 0;

	ret = mc13xxx_adc_do_conversion(batt->mc13xxx,
			MC13XXX_ADC_MODE_MULT_CHAN, 0, samples);

	if (ret == -EBUSY) {
		msleep(20);
		ret = mc13xxx_adc_do_conversion(batt->mc13xxx,
				MC13XXX_ADC_MODE_MULT_CHAN, 0, samples);
	}

	if (ret)
		return ret;

	batt->battv = extract_sample(samples, 0, 0) * 9375 / 2;
	batt->battc = extract_sample(samples, 1, 1) * 5865;

	/* [0, 1023] --> [0, 24V] when CHRGDIVRAW is NOT set
	 * 23,437.5 uV per ADC reading --> 46875 / 2
	 * [0, 1023] --> [0, 12V] when CHRGDIVRAW is set
	 */
	batt->chrgv = extract_sample(samples, 3, 0) * 46875 / 4;
	
	/* [-512, 511] --> [ -300mV, 300mV ] --> [-3A, 3A]
	 * 5.865 mA per ADC count
	 */
	batt->chrgc = extract_sample(samples, 4, 1) * 5865;

	batt->bpv = extract_sample(samples, 2, 0) * 9375 / 2;

	/* [0, 1023] --> [0, 3.6V]
	 * 3515.625 uV per ADC reading --> 3516
	 */
	batt->coincellv = extract_sample(samples, 6, 0) * 3516;

	batt->sample_ms = ms;

	return 0;
}

static int mc13xxx_battery_power(struct mc13xxx_battery *batt, int *retval)
{
	s64 temp;

	temp = div64_s64((s64)(batt->battv) * (s64)(batt->battc), 1000000ll);

	*retval = (int)temp;
	return 0;
}

/*
 *
 */
static int mc13xxx_do_cc_calibrate(struct mc13xxx_battery *batt)
{
	int ret;
	u32 val = 0;

	pr_info("%s, begin", __func__);
	ret = mc13xxx_do_cc_start(batt, 1);
	if (ret)
		return ret;

	msleep(MC13XXX_BATTERY_CC_CAL_MS);

	ret = mc13xxx_reg_read(batt->mc13xxx, MC13XXX_ACC0, &val);
	if (ret)
		return ret;

	batt->cc_cal = (int)((s16)((u16)(val >> MC13XXX_ACC0_CCOUT_SHIFT)));
	pr_info("%s, cc_cal: %d in: %d ms\n", __func__,
			batt->cc_cal, MC13XXX_BATTERY_CC_CAL_MS);

	return ret;
}

static int mc13xxx_do_cc_read(struct mc13xxx_battery *batt, int *coulombs)
{
	int ret;
	u32 val;

	ret = mc13xxx_reg_read(batt->mc13xxx, MC13XXX_ACC0, &val);
	if (ret)
		return ret;

	if (val & MC13XXX_ACC0_CCFAULT) {
		/* either counter roll over, BP has dropped below UVDET, battery removed
		 * In any case we need to clear the fault
		 */
		val &= ~MC13XXX_ACC0_CCFAULT;
		ret = mc13xxx_reg_write(batt->mc13xxx, MC13XXX_ACC0, val);
	}

	*coulombs = (int)((s16)((u16)(val >> MC13XXX_ACC0_CCOUT_SHIFT)));

	return ret;
}

static void mc13xxx_cc_adjust(struct mc13xxx_battery *batt, int coulombs, int *adjusted)
{
	struct timespec timenow;
	struct timespec diff;
	unsigned int diffms;
	int adjust = 0;

	getrawmonotonic(&timenow);

	if (timespec_compare(&timenow, &batt->cc_start_time) > 0) {
		diff = timespec_sub(timenow, batt->cc_start_time);
		diffms = (diff.tv_sec * 1000) + (diff.tv_nsec / 1000000);

		adjust = (batt->cc_cal * diffms) / MC13XXX_BATTERY_CC_CAL_MS;
		adjust /= MC13XXX_BATTERY_CC_ONEC_VAL;
	}

	*adjusted = coulombs + adjust;
}

static int mc13xxx_do_cc_update(struct mc13xxx_battery *batt)
{
	int ret;
	int coulombs = 0;
	int adjusted = 0;

	ret = mc13xxx_do_cc_read(batt, &coulombs);
	if (ret)
		return ret;

	if (abs(coulombs) > 30000) {
		/* reset the CC before we have an overflow */
		mc13xxx_do_cc_start(batt, 0);
	}

//	mc13xxx_cc_adjust(batt, coulombs, &adjusted);

//	pr_info("%s, count: %d, adjusted %d\n", __func__, coulombs, adjusted);
	adjusted = coulombs;
	batt->cc_read_val = adjusted;

	return ret;
}

static int mc13xxx_battery_update(struct mc13xxx_battery *batt)
{
	int ret;
	u32 sens0, sens1;
	u32 chrg_fault;
	bool now_charger_online;
	bool now_battery_online;
	struct timespec timenow;

	int battv = 0, battc = 0;

	/* get battery current and voltage before getting the lock */
	ret = mc13xxx_update_readings(batt, 0);
	if (ret)
		return ret;

	ret = mc13xxx_read_batt_single(batt, &battv, &battc);
	if (ret)
		return ret;

	mc13xxx_lock(batt->mc13xxx);
	ret = mc13xxx_reg_read(batt->mc13xxx, MC13XXX_IRQSENS0, &sens0);
	if (ret)
		goto out;

	ret = mc13xxx_reg_read(batt->mc13xxx, MC13XXX_IRQSENS1, &sens1);
	if (ret)
		goto out;

	now_battery_online = !(sens1 & MC13XXX_IRQSENS1_BATTDETB);
	now_charger_online = !!(sens0 & MC13XXX_IRQSENS0_CHGDET);

	if (now_battery_online != batt->battery_online) {
		batt->battery_online = now_battery_online;

		dev_info(batt->battery.dev, "battery is %s\n",
				now_battery_online ? "online" : "offline");

		/* can't know whether battery is full or not */
		batt->cc_was_full = false;

		if (now_battery_online) {
			mc13xxx_do_cc_start(batt, 0);
		} else {
			mc13xxx_do_cc_stop(batt);
		}
	}

	if (now_charger_online != batt->charger_online) {
		batt->charger_online = now_charger_online;

		dev_info(batt->charger.dev, "charger is %s\n",
				now_charger_online ? "online" : "offline");

		if (batt->charger_online) {
			/* note we don't restart charger state - PMIC does it
			 * automatically
			 */
			getrawmonotonic(&batt->charge_start_time);
		}
	}

	batt->cccv = !!(sens0 & MC13XXX_IRQSENS0_CCCV);
	batt->chgcurr = !!(sens0 & MC13XXX_IRQSENS0_CHGCURR);

//	pr_info("charger_online: %d cccv: %d chgcurr: %d\n",
//			batt->charger_online, batt->cccv, batt->chgcurr);

	if (batt->charger_online) {
		struct timespec diff;
		//struct timespec onehour = {60 * 60, 0};
		struct timespec twohourplus = {(2 * 60 * 60) + 60, 0};
	
		/*
		pr_info("sens0: 0x%06x. 10:%d 9:%d 8:%d\n", sens0,
				!!(sens0 & (1 << 10)),
				!!(sens0 & (1 << 9)),
				!!(sens0 & (1 << 8)));
		*/

		chrg_fault = (sens0 >> 8) & 0x3;
		/* DEBUG, swapped these bits around tyrying to figure out FAULTS*/
		if (chrg_fault == 0x02) {
			dev_info(batt->charger.dev, "Charge time out.\n");
			mc13xxx_do_restart_charging(batt);
		}
		/* TODO: found chip errata here where power limiting bit never gets set */
		if (chrg_fault == 0x01) {
			dev_info(batt->charger.dev, "Power limiting.\n");
		}

		/* */
		getrawmonotonic(&timenow);
		diff = timespec_sub(timenow, batt->charge_start_time);
		if (timespec_compare(&diff, &twohourplus) > 0) {
			dev_info(batt->charger.dev, "Two hour plus elapsed.. restarting\n");
			mc13xxx_reg_rmw(batt->mc13xxx, MC13XXX_CHARGER0,
					MC13XXX_CHARGER0_CHGTMRRST, MC13XXX_CHARGER0_CHGTMRRST);
			
		}

		batt->batt_status = POWER_SUPPLY_STATUS_CHARGING;

		if (batt->cccv && abs(battc) < 100000) {
			batt->full_count++;
			if (batt->full_count > 8) {
				batt->batt_status = POWER_SUPPLY_STATUS_FULL;
				// pr_info("battery full, battc: %d\n", battc);
				mc13xxx_do_cc_set_full(batt);
			}
		} else {
			if (batt->full_count > 4)
				batt->full_count--;
			else
				batt->full_count = 0;
		}

	} else {
		batt->batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		batt->full_count = 0;
	}

	if (batt->battery_online) {
		int cl;
		mc13xxx_do_cc_update(batt);

		/* LOBATL sense bit set when below threshold ... */
		if (sens0 & MC13XXX_IRQSENS0_LOBATL)
			cl = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		/* LOBATH sense bit set when above threshold */
		else if (!(sens0 & MC13XXX_IRQSENS0_LOBATH))
			cl = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (batt->full_count > 8)
				cl = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else {
			if (batt->cc_was_full && batt->cc_read_val > -3000)
				cl = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
			else
				cl = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		}
		batt->batt_capacity_level = cl;
	} else {
		batt->batt_capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
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

static int mc13xxx_battery_get_charge(struct mc13xxx_battery *batt, int *charge)
{
	int coulombs;

	if (!batt->battery_online) {
		*charge = 0;
		return 0;
	}

	coulombs = batt->cc_read_val * 2;

	if (batt->cc_was_full) {
		/* coulombs to uAh (1mAh = 10/36 C)*/
		*charge = 1000 * (coulombs * 10) / 36;

		if (*charge > 0)
			*charge = MC13XXX_BATTERY_CHARGE_FULL;
		else if (abs(*charge) > MC13XXX_BATTERY_CHARGE_FULL)
			*charge = 0;
		else
			*charge = MC13XXX_BATTERY_CHARGE_FULL + *charge;
	} else
		*charge = 0;

	return 0;
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

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = batt->charger_online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = batt->chrgv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = batt->chrgc;
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

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = batt->batt_status;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = batt->battery_online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = batt->battv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = batt->battc;
		break;
	case POWER_SUPPLY_PROP_POWER_NOW:
		ret = mc13xxx_battery_power(batt, &val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 4200000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = 3400000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = mc13xxx_battery_get_charge(batt, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = batt->cc_read_val;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval =batt->batt_capacity_level;
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
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
};

/* ------------------------------------------------------------------------ */
static int mc13xxx_coincell_get_property(struct power_supply *ps,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;
	struct mc13xxx_battery *batt =
		container_of(ps, struct mc13xxx_battery, coincell);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
	{
		u32 power0;

		mc13xxx_lock(batt->mc13xxx);
		ret = mc13xxx_reg_read(batt->mc13xxx,
				MC13XXX_POWER0, &power0);
		val->intval = !!(power0 & MC13XXX_POWER0_COINCHEN);
		mc13xxx_unlock(batt->mc13xxx);
		break;
	}
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = batt->coincellv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 3300000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = 3000000;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static enum power_supply_property mc13xxx_coincell_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
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

	batt->battery.name = "mc13xxx_battery";
	batt->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	batt->battery.properties = mc13xxx_battery_props;
	batt->battery.num_properties = ARRAY_SIZE(mc13xxx_battery_props);
	batt->battery.get_property = mc13xxx_battery_get_property;

	ret = power_supply_register(&pdev->dev, &batt->battery);
	if (ret)
		goto battery_err;

	batt->charger.name = "mc13xxx_charger";
	batt->charger.type = POWER_SUPPLY_TYPE_MAINS;
	batt->charger.properties = mc13xxx_charger_props;
	batt->charger.num_properties = ARRAY_SIZE(mc13xxx_charger_props);
	batt->charger.get_property = mc13xxx_charger_get_property;

	ret = power_supply_register(&pdev->dev, &batt->charger);
	if (ret)
		goto charger_err;
/*
	batt->coincell.name = "mc13xxx_coincell";
	batt->coincell.type = POWER_SUPPLY_TYPE_BATTERY;
	batt->coincell.properties = mc13xxx_coincell_props;
	batt->coincell.num_properties = ARRAY_SIZE(mc13xxx_coincell_props);
	batt->coincell.get_property = mc13xxx_coincell_get_property;

	ret = power_supply_register(&pdev->dev, &batt->coincell);
	if (ret)
		goto coincell_err;
*/
	mc13xxx_lock(batt->mc13xxx);
	{
	batt->mc13xxx->adcflags |= MC13XXX_ADC_MEASURE_BATT;
	batt->mc13xxx->adcflags |= MC13XXX_ADC_MEASURE_CHARGER;
//	batt->mc13xxx->adcflags |= MC13XXX_ADC_MEASURE_LICELL;

	mc13xxx_do_cc_calibrate(batt);
	mc13xxx_do_battery_init(batt);
//	mc13xxx_do_coincell_init(batt);

	mc13xxx_irq_ack(batt->mc13xxx, MC13XXX_IRQ_CHGDET);
	mc13xxx_irq_request(batt->mc13xxx, MC13XXX_IRQ_CHGDET,
			mc13xxx_chgdet_handler, "mc13xxx_chgdet", batt);

	mc13xxx_irq_ack(batt->mc13xxx, MC13XXX_IRQ_CHGFAULT);
	mc13xxx_irq_request(batt->mc13xxx, MC13XXX_IRQ_CHGFAULT,
			mc13xxx_chgfault_handler, "mc13xxx_faults", batt);

	mc13xxx_irq_ack(batt->mc13xxx, MC13XXX_IRQ_CHGCURR);
	mc13xxx_irq_request(batt->mc13xxx, MC13XXX_IRQ_CHGCURR,
			mc13xxx_chgcurr_handler, "mc13xxx_curr", batt);

	mc13xxx_irq_ack(batt->mc13xxx, MC13XXX_IRQ_CCCV);
	mc13xxx_irq_request(batt->mc13xxx, MC13XXX_IRQ_CCCV,
			mc13xxx_cccv_handler, "mc13xxx_curr", batt);
	}
	mc13xxx_unlock(batt->mc13xxx);

	queue_delayed_work(batt->workq, &batt->work, HZ * 5);

	return 0;

coincell_err:
	power_supply_unregister(&batt->charger);
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

	batt->mc13xxx->adcflags &= ~(MC13XXX_ADC_MEASURE_LICELL |
			MC13XXX_ADC_MEASURE_CHARGER |
			MC13XXX_ADC_MEASURE_BATT);

	mc13xxx_irq_free(batt->mc13xxx, MC13XXX_IRQ_CHGDET, batt);
	mc13xxx_irq_free(batt->mc13xxx, MC13XXX_IRQ_CHGFAULT, batt);
	mc13xxx_irq_free(batt->mc13xxx, MC13XXX_IRQ_CHGCURR, batt);
	mc13xxx_irq_free(batt->mc13xxx, MC13XXX_IRQ_CCCV, batt);
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
