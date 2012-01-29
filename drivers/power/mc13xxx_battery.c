/*
 * Copyright 2009-2010 Creative Product Design
 * Marc Reilly marc@cpdesign.com.au
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */
/*
 * This driver provides basic functionality for the battery and charger
 * functions of the mc13892. It uses a simple polling approach which checks
 * the battery and charge status every few seconds.
 *
 * The battery coulomb counter (CC) is supported. Some CC information is kept
 * in the MEMORYA (MEMA) register of the device so that the battery level is
 * available across reset/power cycles.
 */
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

#define MC13XXX_MEMORYA		18

#define MC13XXX_CHARGER0	48
#define MC13XXX_CHARGER0_VCHRG_SHIFT	0
#define MC13XXX_CHARGER0_VCHRG_MASK	(0x7 << MC13XXX_CHARGER0_VCHRG_SHIFT)
#define MC13XXX_CHARGER0_ICHRG_SHIFT	3
#define MC13XXX_CHARGER0_ICHRG_MASK	(0xf << MC13XXX_CHARGER0_ICHRG_SHIFT)
#define MC13XXX_CHARGER0_TREN		(1 << 7)
#define MC13XXX_CHARGER0_THCHKB		(1 << 9)
#define MC13XXX_CHARGER0_PLIM_SHIFT	15
#define MC13XXX_CHARGER0_PLIM_MASK	(0x3 << MC13XXX_CHARGER0_PLIM_SHIFT)
#define MC13XXX_CHARGER0_PLIMDIS	(1 << 17)
#define MC13XXX_CHARGER0_CHGTMRRST	(1 << 19)
#define MC13XXX_CHARGER0_CHRGRESTART	(1 << 20)
#define MC13XXX_CHARGER0_CYCLB		(1 << 22)
#define MC13XXX_CHARGER0_CHGAUTOVIB	(1 << 23)

#define MC13XXX_BATTERY_CC_CAL_MS	100
#define MC13XXX_BATTERY_CC_ONEC_VAL	2621

/* when set, contents of the CC are valid */
#define MC13XXX_BATTERY_MEM_WAS_FULL	(1 << 0)

struct mc13xxx_battery {
	struct mc13xxx *mc13xxx;
	struct mc13892_battery_platform_data *mc13892_pdata;

	struct power_supply battery;
	struct power_supply charger;

	struct delayed_work work;
	struct workqueue_struct *workq;

	/* last read coulomb counter value*/
	int cc_read_val;
	/* set after battery reaches end of charge, means CC count valid */
	bool cc_was_full;
	/* CC count acheived over calibration time */
	int cc_cal;

	/* battery voltage, uV */
	int battv;
	/* battery current, uA, positive is FROM battery */
	int battc;
	/* charger voltage, uV */
	int chrgv;
	/* charger current, positive is INTO battery & system */
	int chrgc;
	int bpv;

	/* last ms count a sample was obtained*/
	unsigned int sample_ms;

	bool charger_online;
	struct timespec charge_start_time;

	bool battery_online;

	/* pmic charge state - follows bits of the same name*/
	/* true when in constant voltage mode */
	bool cccv;
	/* charge current detected */
	bool chgcurr;

	/* hysteresis counter for detecting end of charge */
	int full_count;

	int batt_status;
	int batt_capacity_level;
};

static int signedFrom10bit(unsigned int val)
{
	if (val & 0x200)
		return (0x200 - (val & 0x1ff)) * -1;

	return val & 0x1ff;
}

/*
 * Helper to extract a channel sample from the array.
 * The array is stored as:
 *
 *	index	|MSB ....LSB|
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

/*
 * NOTE: all these mc13xxx_do functions require the lock to
 * be already held.
 */
static int mc13xxx_do_cc_start(struct mc13xxx_battery *batt, bool calibrate)
{
	int ret;
	u32 acc0, acc1;
	int cc_mult = batt->mc13892_pdata->cc_onec_multiplier;

	acc0 = MC13XXX_ACC0_STARTCC | MC13XXX_ACC0_CCDITHER;

	if (calibrate) {
		acc0 |= MC13XXX_ACC0_CCCALA;
		acc1 = 1;
	} else {
		acc1 = MC13XXX_BATTERY_CC_ONEC_VAL * cc_mult;
	}

	ret = mc13xxx_reg_write(batt->mc13xxx, MC13XXX_ACC1, acc1);
	ret = mc13xxx_reg_write(batt->mc13xxx, MC13XXX_ACC0, acc0);

	return ret;
}

static int mc13xxx_do_cc_stop(struct mc13xxx_battery *batt)
{
	int ret;

	ret = mc13xxx_reg_rmw(batt->mc13xxx, MC13XXX_ACC0,
			MC13XXX_ACC0_CCCALA | MC13XXX_ACC0_STARTCC, 0);

	return ret;
}

static int mc13xxx_do_cc_reset(struct mc13xxx_battery *batt)
{
	int ret;

	ret = mc13xxx_reg_rmw(batt->mc13xxx, MC13XXX_ACC0,
			MC13XXX_ACC0_RSTCC, MC13XXX_ACC0_RSTCC);

	return ret;
}

static int mc13xxx_do_cc_set_full(struct mc13xxx_battery *batt, bool is_full)
{
	int ret;

	ret = mc13xxx_reg_rmw(batt->mc13xxx, MC13XXX_MEMORYA,
			MC13XXX_BATTERY_MEM_WAS_FULL,
			is_full ? MC13XXX_BATTERY_MEM_WAS_FULL : 0);

	batt->cc_was_full = is_full;

	return mc13xxx_do_cc_reset(batt);
}

static int mc13xxx_do_restart_charging(struct mc13xxx_battery *batt)
{
	int ret;
	struct mc13892_battery_platform_data *pdata = batt->mc13892_pdata;
	u32 val = 0;

	dev_dbg(batt->charger.dev, "Restarting charging\n");

	/* only set vchrg and ichrg if not in standalone */
	if (pdata->ichrg != MC13892_BATTERY_ICHRG_STANDALONE) {
		val |= (pdata->vchrg << MC13XXX_CHARGER0_VCHRG_SHIFT)
			& MC13XXX_CHARGER0_VCHRG_MASK;
		val |= (pdata->ichrg << MC13XXX_CHARGER0_ICHRG_SHIFT)
			& MC13XXX_CHARGER0_ICHRG_MASK;
		val |= MC13XXX_CHARGER0_CHGAUTOVIB;
	}

	val |= (pdata->plim << MC13XXX_CHARGER0_PLIM_SHIFT)
		& MC13XXX_CHARGER0_PLIM_MASK;

	val |= MC13XXX_CHARGER0_TREN;
	val |= MC13XXX_CHARGER0_CHRGRESTART;
	val |= MC13XXX_CHARGER0_THCHKB;

	ret = mc13xxx_reg_write(batt->mc13xxx, MC13XXX_CHARGER0, val);

	getrawmonotonic(&batt->charge_start_time);

	return ret;
}

static int mc13xxx_do_set_ichrg(struct mc13xxx_battery *batt,
		int ichrg)
{
	int ret;
	u32 mask, val = 0;

	mask = MC13XXX_CHARGER0_ICHRG_MASK | MC13XXX_CHARGER0_CHGAUTOVIB;

	if (ichrg != MC13892_BATTERY_ICHRG_STANDALONE) {
		ichrg <<= MC13XXX_CHARGER0_ICHRG_SHIFT;
		val = (ichrg & MC13XXX_CHARGER0_ICHRG_MASK)
			| MC13XXX_CHARGER0_CHGAUTOVIB;
	}

	ret = mc13xxx_reg_rmw(batt->mc13xxx, MC13XXX_CHARGER0, mask, val);

	return ret;
}

/*
 * Ignores the min and max values and returns average of the rest
 * num must be greater than 2
 */
static int average_samples(int *samples, int num)
{
	int i;
	int vmin, vmax;
	int total;

	BUG_ON(num <= 2);

	total = samples[0];
	vmin = samples[0];
	vmax = vmin;

	for (i = 1; i < num; ++i) {
		int v = samples[i];
		if (v > vmax)
			vmax = v;
		if (v < vmin)
			vmin = v;

		total += v;
	}

	total -= vmin + vmax;

	return total / (num - 2);
}

static int mc13xxx_read_raw_chrg_single(struct mc13xxx_battery* batt,
		int *ichrg_ret)
{
	unsigned int samples[4];
	int ret, i;
	int chrg[8];
	u32 adc0mask = MC13XXX_ADC0_CHRGICON | MC13XXX_ADC0_CHRGRAWDIV;
	u32 adc0val = adc0mask;

	ret = mc13xxx_adc_do_conversion_ex(batt->mc13xxx,
			MC13XXX_ADC_MODE_SINGLE_CHAN, 4, samples,
			adc0mask, adc0val,
			0, 0);

	if (ret == -EBUSY) {
		msleep(10);
		ret = mc13xxx_adc_do_conversion_ex(batt->mc13xxx,
				MC13XXX_ADC_MODE_SINGLE_CHAN, 4, samples,
				adc0mask, adc0val,
				0, 0);
	}

	for (i = 0; i < 8; ++i)
		chrg[i] = extract_sample(samples, i, true);

	*ichrg_ret = average_samples(samples, 8);

	return ret;
}

static int mc13xxx_read_batt_single(struct mc13xxx_battery* batt,
		int *battv_ret, int *battc_ret)
{
	unsigned int samples[4];
	int ret, i;
	u32 adc0mask = MC13XXX_ADC0_BATTICON | MC13XXX_ADC0_CHRGRAWDIV;
	u32 adc0val = adc0mask;

	int battv[4];
	int battc[4];

	ret = mc13xxx_adc_do_conversion_ex(batt->mc13xxx,
			MC13XXX_ADC_MODE_SINGLE_CHAN, 1, samples,
			adc0mask, adc0val,
			0, 0);

	if (ret == -EBUSY) {
		msleep(10);
		ret = mc13xxx_adc_do_conversion_ex(batt->mc13xxx,
				MC13XXX_ADC_MODE_SINGLE_CHAN, 1, samples,
				adc0mask, adc0val,
				0, 0);
	}

	for (i = 0; i < 4; ++i) {
		battv[i] = extract_sample(samples, (i * 2), 0);
		battc[i] = extract_sample(samples, (i * 2) + 1, 1);
	}

	*battv_ret = average_samples(battv, 4) * 9375 / 2;
	*battc_ret = average_samples(battc, 4) * 5865;

	return ret;
}

static int mc13xxx_update_readings(struct mc13xxx_battery *batt,
		unsigned int since)
{
	unsigned int samples[4];
	int ret;
	u32 adc0mask = MC13XXX_ADC0_CHRGICON | MC13XXX_ADC0_BATTICON
				| MC13XXX_ADC0_CHRGRAWDIV;
	u32 adc0val = adc0mask;

	unsigned int ms = jiffies_to_msecs(jiffies);
	if (ms - batt->sample_ms < since)
		return 0;

	ret = mc13xxx_adc_do_conversion_ex(batt->mc13xxx,
			MC13XXX_ADC_MODE_MULT_CHAN, 0, samples,
			adc0mask, adc0val, 0, 0);

	if (ret == -EBUSY) {
		msleep(20);
		ret = mc13xxx_adc_do_conversion_ex(batt->mc13xxx,
				MC13XXX_ADC_MODE_MULT_CHAN, 0, samples,
				adc0mask, adc0val, 0, 0);
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

static int count_from_reg(u32 reg)
{
	return (int)((s16)((u16)(reg >> MC13XXX_ACC0_CCOUT_SHIFT)));
}

/*
 * The CC is calibrated by disconnecting the sense resistor from the CC
 * inputs and setting the accumulator count to 1 so that any error causes
 * the count to change relatively rapidly.
 */
static int mc13xxx_do_cc_calibrate(struct mc13xxx_battery *batt)
{
	int ret;
	u32 startval = 0;
	u32 val = 0;

	ret = mc13xxx_reg_read(batt->mc13xxx, MC13XXX_ACC0, &startval);
	if (ret) {
		dev_warn(batt->battery.dev,
				"Couldn't read counter start value\n");
		return ret;
	}

	ret = mc13xxx_do_cc_start(batt, 1);
	if (ret) {
		dev_warn(batt->battery.dev,
				"Couldn't start CC calibration\n");
		return ret;
	}

	mc13xxx_unlock(batt->mc13xxx);
	msleep(MC13XXX_BATTERY_CC_CAL_MS);
	mc13xxx_lock(batt->mc13xxx);

	ret = mc13xxx_reg_read(batt->mc13xxx, MC13XXX_ACC0, &val);
	if (!ret) {
		int start, finish;
		start = count_from_reg(startval);
		finish = count_from_reg(val);

		batt->cc_cal = finish - start;

		dev_dbg(batt->battery.dev, "%s: cc_cal: %d in: %d ms\n",
				__func__, batt->cc_cal,
				MC13XXX_BATTERY_CC_CAL_MS);
	} else {
		dev_warn(batt->battery.dev,
				"Couldn't read end calibration value\n");
		batt->cc_cal = 0;
	}

	mc13xxx_do_cc_stop(batt);

	return ret;
}

static int mc13xxx_do_cc_read(struct mc13xxx_battery *batt,
		int *coulombs)
{
	int ret;
	u32 val;

	ret = mc13xxx_reg_read(batt->mc13xxx, MC13XXX_ACC0, &val);
	if (ret)
		return ret;

	if (val & MC13XXX_ACC0_CCFAULT) {
		/*
		 * either counter roll over, BP has dropped below UVDET,
		 * battery removed. In any case we need to clear the fault
		 */
		val &= ~MC13XXX_ACC0_CCFAULT;
		ret = mc13xxx_reg_write(batt->mc13xxx, MC13XXX_ACC0, val);
	}

	*coulombs = count_from_reg(val);

	return ret;
}

/*
 * The CC counter is read only, the only way we can change it is to reset it
 * to 0. To simplying things and avoid handling rollover keep the coulomb
 * counter range valid by using an appropriate CC_ONEC value.
 */
static int mc13xxx_do_cc_update(struct mc13xxx_battery *batt)
{
	int ret;
	int coulombs = 0;

	ret = mc13xxx_do_cc_read(batt, &coulombs);
	if (ret)
		return ret;

	if (abs(coulombs) > 30000) {
		/* reset the CC before we have an overflow */
		mc13xxx_do_cc_reset(batt);
	}

	batt->cc_read_val = coulombs;

	return ret;
}

static void mc13xxx_do_wait_ms(struct mc13xxx *mc13xxx, unsigned int ms)
{
	mc13xxx_unlock(mc13xxx);
	msleep(ms);
	mc13xxx_lock(mc13xxx);
}

/*
 * mc13892 errata: #8938 - Failure of charger removal detection
 *
 * check if the charger is really online by forcing it to 0mA.
 * We must wait for the debounce period, errata doc recommends
 * 1000 ms.
 */
static bool mc13892_do_confirm_charger_online(struct mc13xxx_battery *batt,
		u32 *sens0)
{
	int raw_chrgc_avg;
	int ret;
	bool now_charger_online = true;

	// find out if charger curr is low enough to warrant checking
	mc13xxx_unlock(batt->mc13xxx);
	ret = mc13xxx_read_raw_chrg_single(batt, &raw_chrgc_avg);
	mc13xxx_lock(batt->mc13xxx);

	if ((ret == 0) && (raw_chrgc_avg <= 3)) {
		mc13xxx_do_set_ichrg(batt, MC13892_BATTERY_ICHRG_0mA);

		mc13xxx_do_wait_ms(batt->mc13xxx, 1000);

		mc13xxx_do_set_ichrg(batt,
				MC13892_BATTERY_ICHRG_1200mA);

		mc13xxx_do_wait_ms(batt->mc13xxx, 1000);

		ret = mc13xxx_reg_read(batt->mc13xxx,
				MC13XXX_IRQSENS0, sens0);
	}

	if (ret == 0)
		now_charger_online = !!(*sens0 & MC13XXX_IRQSENS0_CHGDET);

	return now_charger_online;
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

	now_charger_online = !!(sens0 & MC13XXX_IRQSENS0_CHGDET);

	if (now_charger_online)
		now_charger_online =
			mc13892_do_confirm_charger_online(batt, &sens0);


	now_battery_online = !(sens1 & MC13XXX_IRQSENS1_BATTDETB);
	if (now_battery_online != batt->battery_online) {
		batt->battery_online = now_battery_online;

		dev_dbg(batt->battery.dev, "battery is %s\n",
				now_battery_online ? "online" : "offline");

		/* can't know whether battery is full or not */
		mc13xxx_do_cc_set_full(batt, false);

		if (now_battery_online) {
			mc13xxx_do_cc_calibrate(batt);
			mc13xxx_do_cc_start(batt, 0);
		} else {
			mc13xxx_do_cc_stop(batt);
		}
	}

	if (now_charger_online != batt->charger_online) {
		batt->charger_online = now_charger_online;

		dev_dbg(batt->charger.dev, "charger is %s\n",
				now_charger_online ? "online" : "offline");

		if (batt->charger_online) {
			/*
			 * force restart of the charge timer to avoid the pmic
			 * internally timing out.
			 * note we don't restart charger state - PMIC does it
			 * automatically
			 */
			getrawmonotonic(&batt->charge_start_time);
			mc13xxx_reg_rmw(batt->mc13xxx, MC13XXX_CHARGER0,
					MC13XXX_CHARGER0_CHGTMRRST,
					MC13XXX_CHARGER0_CHGTMRRST);
		}
	}

	batt->cccv = !!(sens0 & MC13XXX_IRQSENS0_CCCV);
	batt->chgcurr = !!(sens0 & MC13XXX_IRQSENS0_CHGCURR);

	if (batt->charger_online) {
		bool is_timeout;
		bool is_powerlimit = false;
		struct timespec diff;
		struct timespec chrgtimeout = {(120 * 60), 0};

		/* Check if the charger will be timing out */
		getrawmonotonic(&timenow);
		diff = timespec_sub(timenow, batt->charge_start_time);
		is_timeout = (timespec_compare(&diff, &chrgtimeout) > 0)
				? true : false;

		/*
		 * workaround for power limit sense bit showing up as timeout:
		 * always restart the charge timer if the charger is online.
		 * This way the charger won't time out.
		 */
		getrawmonotonic(&batt->charge_start_time);
		mc13xxx_reg_rmw(batt->mc13xxx, MC13XXX_CHARGER0,
				MC13XXX_CHARGER0_CHGTMRRST,
				MC13XXX_CHARGER0_CHGTMRRST);

		chrg_fault = (sens0 >> 8) & 0x3;
		if (chrg_fault == 0x03) {
			dev_warn(batt->charger.dev,
					"PMIC IC die over-temperature\n");
		} else if (chrg_fault == 0x02) {
			/*
			 * Found chip errata where power limiting bit never
			 * gets set, and this bit is set instead.
			 * (applies up to at least v3.2)
			 *
			 * Work around is to monitor the charge time and
			 * manually restart
			 * charging.
			 */
			dev_info(batt->charger.dev,
					"Charge time out or power limiting\n");

			if (!is_timeout)
				is_powerlimit = true;

		} else if (chrg_fault == 0x01) {
			is_powerlimit = true;
		}

		if (is_timeout) {
			mc13xxx_do_restart_charging(batt);
		}

		batt->batt_status = POWER_SUPPLY_STATUS_CHARGING;

		/*
		 * Battery is full when the current into battery drops below
		 * threshold.
		 * Don't let the battery become full if power limiting is
		 * active, as the charge current oscillates as result of the
		 * power limiting.
		 */
		if (!is_powerlimit && batt->cccv
				&& (battv > 4100000) && (abs(battc) < 150000)) {

			batt->full_count++;
			if (batt->full_count > 8) {
				batt->full_count = 8;
				batt->batt_status = POWER_SUPPLY_STATUS_FULL;
				mc13xxx_do_cc_set_full(batt, true);
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

	if (!batt->battery_online) {
		batt->batt_capacity_level =
			POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
	} else {
		int cl;
		mc13xxx_do_cc_update(batt);

		/* LOBATL sense bit set when below threshold ... */
		if (sens0 & MC13XXX_IRQSENS0_LOBATL)
			dev_warn(batt->battery.dev,
					"Battery is extremely low\n");

		/* LOBATH sense bit set when above threshold */
		if (!(sens0 & MC13XXX_IRQSENS0_LOBATH)) {
			cl = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		} else if (batt->full_count >= 8) {
			cl = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		} else {
			if (!batt->cc_was_full)
				cl = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
			else if (batt->cc_read_val > -3000)
				cl = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
			else if (batt->cc_read_val > -15000)
				cl = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
			else
				cl = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		}
		batt->batt_capacity_level = cl;
	}

	power_supply_changed(&batt->battery);
	power_supply_changed(&batt->charger);

 out:
	mc13xxx_unlock(batt->mc13xxx);
	return ret;
}

static void mc13xxx_battery_work(struct work_struct *work)
{
	struct mc13xxx_battery *batt =
		container_of(work, struct mc13xxx_battery, work.work);

	mc13xxx_battery_update(batt);

	queue_delayed_work(batt->workq, &batt->work, HZ * 5);
}

static int mc13xxx_battery_get_charge(struct mc13xxx_battery *batt,
		int *charge)
{
	int coulombs;
	struct mc13892_battery_platform_data *pdata = batt->mc13892_pdata;
	int capacity_uAh = pdata->battery_nominal_capacity_uAh;
	int cc_mult = pdata->cc_onec_multiplier;

	if (!batt->battery_online) {
		*charge = 0;
		return 0;
	}

	coulombs = batt->cc_read_val * cc_mult;

	if (batt->cc_was_full) {
		/* coulombs to uAh (1mAh = 10/36 C)*/
		*charge = 1000 * (coulombs * 10) / 36;

		if (*charge > 0)
			*charge = capacity_uAh;
		else if (abs(*charge) > capacity_uAh)
			/* don't let the charge go negative */
			*charge = 1;
		else
			*charge = capacity_uAh + *charge;
	} else {
		/* we can't know the battery state without a reference */
		*charge = 0;
	}

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
		val->intval = 4200000;
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
		val->intval = 3500000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = mc13xxx_battery_get_charge(batt, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = batt->cc_read_val;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = batt->batt_capacity_level;
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

static int mc13xxx_do_battery_init(struct mc13xxx_battery *batt)
{
	int ret;
	u32 sens1;
	u32 val = 0;
	u32 mask = 0;

	/* Battery detect enable */
	val |= MC13XXX_POWER0_BATTDETEN;
	mask |= MC13XXX_POWER0_BATTDETEN;
	/* BPSNS = b11 --> LOBATL = 3.1, LOBATH = 3.4 */
	val |= 0x3 << MC13XXX_POWER0_BPSNS_SHIFT;
	mask |= MC13XXX_POWER0_BPSNS_MASK;

	ret = mc13xxx_reg_rmw(batt->mc13xxx, MC13XXX_POWER0, mask, val);

	/* restore CC vars */
	ret = mc13xxx_reg_read(batt->mc13xxx, MC13XXX_MEMORYA, &val);
	if (!ret) {
		dev_info(batt->battery.dev,
				"Detected battery charge is valid\n");
		batt->cc_was_full = val & MC13XXX_BATTERY_MEM_WAS_FULL;
	}

	ret = mc13xxx_reg_read(batt->mc13xxx, MC13XXX_IRQSENS1, &sens1);
	if (!ret)
		batt->battery_online = !(sens1 & MC13XXX_IRQSENS1_BATTDETB);

	if (batt->battery_online) {
		if (!batt->cc_was_full) {
			mc13xxx_do_cc_calibrate(batt);
		}
		/* ensure CC is running */
		mc13xxx_do_cc_start(batt, 0);
	}

	return ret;
}

static int __devinit mc13xxx_battery_probe(struct platform_device *pdev)
{
	struct mc13xxx_battery *batt;
	int ret;

	batt = kzalloc(sizeof(*batt), GFP_KERNEL);
	if (batt == NULL)
		return -ENOMEM;

	batt->mc13xxx = dev_get_drvdata(pdev->dev.parent);
	platform_set_drvdata(pdev, batt);

	batt->mc13892_pdata = dev_get_platdata(&pdev->dev);

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

	mc13xxx_lock(batt->mc13xxx);
	mc13xxx_do_battery_init(batt);
	mc13xxx_unlock(batt->mc13xxx);

	queue_delayed_work(batt->workq, &batt->work, HZ * 3);

	dev_info(pdev->dev.parent, "MC13xxx Battery and Charger driver\n");

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

	cancel_delayed_work_sync(&batt->work);
	destroy_workqueue(batt->workq);

	power_supply_unregister(&batt->charger);
	power_supply_unregister(&batt->battery);

	platform_set_drvdata(pdev, NULL);
	kfree(batt);

	return 0;
}

const struct platform_device_id mc13xxx_battery_idtable[] = {
	{
		.name = "mc13892-battery",
	},
	{ }
};

static struct platform_driver mc13xxx_battery_driver = {
	.id_table	= mc13xxx_battery_idtable,
	.driver = {
		.name = "mc13xxx-battery",
		.owner = THIS_MODULE,
	},
	.remove = __exit_p(mc13xxx_battery_remove),
};

static int __init mc13xxx_battery_init(void)
{
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
