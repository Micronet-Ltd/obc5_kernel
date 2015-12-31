/* drivers/i2c/chips/rpr521_driver.c - ROHM RPR521 Linux kernel driver
 *
 * Copyright (C) 2012-2015
 * Written by Andy Mi <andy-mi@rohm.com.cn>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*
 *  This is Linux kernel modules for ambient light + proximity sensor
 *  Revision History
 *  2012-7-19:	Ver. 1.0	New release together with a porting guide.
 *  2012-8-14:	Ver. 1.1	Added calibration and set thresholds methods.
 *		Besides,thresholds are automatically changed
 *		if a ps int is triggered to avoid constant interrupts.
 *  2014-1-09:	Ver. 1.2	Modified some functions for rpr521
 */
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/ioctl.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/unistd.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#include <linux/input/rpr0521c_driver.h>
/*************** Global Data ******************/
/* parameter for als calculation */
#define COEFFICIENT               (4)
const unsigned long data0_coefficient[COEFFICIENT] = {839, 935, 666, 405};
const unsigned long data1_coefficient[COEFFICIENT] = {0, 520,  278,  144};
const unsigned long judge_coefficient[COEFFICIENT] = { 18,  110,  195, 281};

#define _AUTO_THRESHOLD_CHANGE_
/* #define _INIT_CALIB_ */

/* at phone calling and psensor is not disabled, disable lsensor */
#define DISABLE_LSENSOR_WHEN_PSENSOR_ENABLED

/* mode control table */
#define MODE_CTL_FACTOR (16)
static const struct MCTL_TABLE {
		short ALS;
		short PS;
} mode_table[MODE_CTL_FACTOR] = {
		{  0,   0},   /*  0 */
		{  0,  10},   /*  1 */
		{  0,  40},   /*  2 */
		{  0, 100},   /*  3 */
		{  0, 400},   /*  4 */
		{100,  50},   /*  5 */
		{100, 100},   /*  6 */
		{100, 400},   /*  7 */
		{400,   0},   /*  8 */
		{400, 100},   /*  9 */
		{400,   0},   /* 10 */
		{400, 400},   /* 11 */
		{  50,  50},   /* 12 */
		{  0,   0},   /* 13 */
		{  0,   0},   /* 14 */
		{  0,   0}    /* 15 */
};

/* gain table */
#define GAIN_FACTOR (16)
static const struct GAIN_TABLE {
		/*char DATA0;
		char DATA1;*/
		unsigned char DATA0; /* grace modify in 2014.5.7 */
		unsigned char DATA1; /* grace modify in 2014.5.7 */
} gain_table[GAIN_FACTOR] = {
		{  1,   1},   /*  0 */
		{  0,   0},   /*  1 */
		{  0,   0},   /*  2 */
		{  0,   0},   /*  3 */
		{  2,   1},   /*  4 */
		{  2,   2},   /*  5 */
		{  0,   0},   /*  6 */
		{  0,   0},   /*  7 */
		{  0,   0},   /*  8 */
		{  0,   0},   /*  9 */
		{ 64,  64},   /* 10 */
		{  0,   0},   /* 11 */
		{  0,   0},   /* 12 */
		{  0,   0},   /* 13 */
		{128,  64},   /* 14 */
		{128, 128}    /* 15 */
};

/*
 * Global data
 */
struct ALS_PS_DATA *sensor_info;
/* static int sus_res = 0; */
static struct sensors_classdev sensors_light_cdev = {
	.name = "light",
	.vendor = "rohm",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "30000",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 1000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_calibrate = NULL,
	.sensors_write_cal_params = NULL,
	.params = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "proximity",
	.vendor = "rohm",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 1000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.flags = 1,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_calibrate = NULL,
	.sensors_write_cal_params = NULL,
	.params = NULL,
};



static int rpr521_init_client_power_on(struct i2c_client *client);

static int rpr521_set_als_poll_delay(struct i2c_client *client,
	unsigned int val);
static int rpr521_enable_ps_sensor(struct i2c_client *client, int val);
static int rpr521_enable_als_sensor(struct i2c_client *client, int val);
/*************** Functions ******************/
/******************************************************************************
 * NAME     : rpr521_set_enable
 * FUNCTION : set measurement time according to enable
 * REMARKS  : this function will overwrite the work mode.
 *				if it is called improperly,
 *			    you may shutdown some part unexpectedly.
 *				please check als_ps->enable first.
 *			    I assume it is run in normal mode.
 *				If you want low noise mode,
 *				the code should be modified.
 *****************************************************************************/
static int rpr521_set_enable(struct i2c_client *client, int enable)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;

	if (enable > 0xFb) {

		pr_debug("%s: invalid measurement time setting.\n", __func__);

		return -EINVAL;
	} else {

		mutex_lock(&als_ps->update_lock);
		ret = i2c_smbus_write_byte_data(client,
			REG_MODECONTROL, enable);
		mutex_unlock(&als_ps->update_lock);

		als_ps->enable = enable;
		als_ps->als_time = mode_table[(enable & 0xF)].ALS;
		als_ps->ps_time = mode_table[(enable & 0xF)].PS;

		return ret;
	}
}

static int rpr521_set_ps_threshold_low(struct i2c_client *client, int threshold)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	unsigned short write_data;

		/* check whether the parameter is valid */
	if (threshold > REG_PSTL_MAX) {
		pr_debug("%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	/*if(threshold > als_ps->ps_th_h)
	{
		pr_debug("%s: higher than threshold high.\n", __func__);
		return -EINVAL;
	}*/

		/* write register to rpr521 via i2c */
	write_data = CONVERT_TO_BE(threshold);
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_i2c_block_data(client, REG_PSTL,
		sizeof(write_data), (unsigned char *)&write_data);
	mutex_unlock(&als_ps->update_lock);
	if (ret < 0) {
		pr_debug("%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->ps_th_l = threshold;

	return 0;
}

static int rpr521_set_ps_threshold_high(struct i2c_client *client,
	int threshold)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	unsigned short write_data;

		/* check whether the parameter is valid */
	if (threshold > REG_PSTH_MAX) {
		pr_debug("%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	/*if(threshold < als_ps->ps_th_l)
	{
		pr_debug("%s: lower than threshold low.\n", __func__);
		return -EINVAL;
	}*/

		/* write register to rpr521 via i2c */
	write_data = CONVERT_TO_BE(threshold);
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_i2c_block_data(client, REG_PSTH,
		sizeof(write_data), (unsigned char *)&write_data);
	mutex_unlock(&als_ps->update_lock);
	if (ret < 0) {
		pr_debug("%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->ps_th_h = threshold;

	return 0;
}

static int rpr521_calibrate(struct i2c_client *client)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int average;
	unsigned int i, tmp, ps_th_h, ps_th_l;

	average = 0;

	/* rpr521_set_enable(client, 0x41);	PS 10ms */
	rpr521_set_enable(client, PS_EN|PS_DOUBLE_PULSE|PS10MS);


	for (i = 0; i < 20; i++) {
		msleep(20);
		tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
		if (tmp < 0)
			pr_debug("%s: i2c read ps data fail.\n", __func__);

		average += tmp & 0xFFF;
	}
	average /= 20;
	ps_th_h = average + THRES_TOLERANCE + THRES_DEFAULT_DIFF;
	ps_th_l = average + THRES_TOLERANCE;

	if (ps_th_h < 0) {
		pr_debug("%s: high threshold is less than 0.\n", __func__);
		goto err_exit;
	}
	if (ps_th_h > REG_PSTH_MAX)
		goto err_exit;

	if (ps_th_l < 0) {
		pr_debug("%s: low threshold is less than 0.\n", __func__);
		goto err_exit;
	}
	if (ps_th_l > REG_PSTL_MAX)
		goto err_exit;

	if (!(rpr521_set_ps_threshold_high(client, ps_th_h)))
		als_ps->ps_th_h_back = ps_th_h;
	else
		goto err_exit;
	if (!(rpr521_set_ps_threshold_low(client, ps_th_l)))
		als_ps->ps_th_l_back = ps_th_l;
	else
		goto err_exit;

	/* rpr521_set_enable(client, 0); disable ps */
		rpr521_set_enable(client, PS_ALS_SET_MODE_CONTROL);

	return 0;

err_exit:
	rpr521_set_enable(client, PS_ALS_SET_MODE_CONTROL);

	return -EINVAL;

}

#if _FUNCTION_USED_
static int rpr521_set_persist(struct i2c_client *client, int persist)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;

		/* check whether the parameter is valid */
	if (persist > PERSISTENCE_MAX) {
		pr_debug("%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}

		/* write register to rpr521 via i2c */
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_byte_data(client, REG_PERSISTENCE, persist);
	mutex_unlock(&als_ps->update_lock);
	if (ret < 0) {
		pr_debug("%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->persistence = persist;

	return 0;
}

static int rpr521_set_control(struct i2c_client *client, int control)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	unsigned char gain, led_current;

	if (control > REG_ALSPSCTL_MAX) {
		pr_debug("%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}

	gain = (control & 0x3C) >> 2;
	led_current = control & 0x03;

	if (!((gain == ALSGAIN_X1X1) || (gain == ALSGAIN_X1X2) ||
		(gain == ALSGAIN_X2X2) || (gain == ALSGAIN_X64X64)
		|| (gain == ALSGAIN_X128X64) || (gain == ALSGAIN_X128X128))) {
		pr_debug("%s: invalid gain setting.\n", __func__);

		return -EINVAL;
	}

	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_byte_data(client, REG_ALSPSCONTROL, control);
	mutex_unlock(&als_ps->update_lock);
	if (ret < 0) {
		pr_debug("%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->control = control;
	als_ps->gain0 = gain_table[gain].DATA0;
	als_ps->gain1 = gain_table[gain].DATA1;
	als_ps->ledcurrent = led_current;

	return ret;
}
#endif

/******************************************************************************
 * NAME       : long_long_divider
 * FUNCTION   : calc divider of unsigned long long int or unsgined long
 * REMARKS    :
 *****************************************************************************/
static int long_long_divider(long long data, unsigned long base_divier,
	unsigned long *answer, unsigned long long *overplus)
{
	unsigned long long divier;
	unsigned long      unit_sft;

	if ((data < 0) || (base_divier == 0)) {
		*answer   = 0;
		*overplus = 0;
		return CALC_ERROR;
	}

	divier = base_divier;
	if (data > MASK_LONG) {
		unit_sft = 0;
		while ((data > divier) && (divier > 0)) {
				unit_sft++;
				divier = divier << 1;
		}
		while ((data > base_divier) && (unit_sft > 0)) {
			if (data > divier) {
				*answer += 1 << unit_sft;
				data    -= divier;
			}
			unit_sft--;
			divier = divier >> 1;
		}
		*overplus = data;
	} else {

		*answer = (unsigned long)(data & MASK_LONG) / base_divier;
		/* calculate over plus and shift 16bit */
		*overplus = (unsigned long long)(data -
			(*answer * base_divier));

	}

	return 0;
}

/******************************************************************************
 * NAME       : calc_rohm_als_data
 * FUNCTION   : calculate illuminance data for rpr521
 * REMARKS    : final_data is 1000 times, which is defined
 *				as CUT_UNIT, of the actual lux value
 *****************************************************************************/
static int calc_rohm_als_data(unsigned short data0, unsigned short data1,
	unsigned char gain0, unsigned char gain1, unsigned short time)
{
#define DECIMAL_BIT      (15)
#define JUDGE_FIXED_COEF (100)
#define MAX_OUTRANGE    (11357)
#define MAXRANGE_NMODE   (0xFFFF)
#define MAXSET_CASE      (4)

	int                result;
	int                final_data;
	struct CALC_DATA   calc_data;
	struct CALC_ANS    calc_ans;
	unsigned long      calc_judge;
	unsigned char      set_case;
	unsigned long      div_answer;
	unsigned long long div_overplus;
	unsigned long long overplus;
	unsigned long      max_range;

	/* set the value of measured als data */
	calc_data.als_data0  = data0;
	calc_data.als_data1  = data1;
	calc_data.gain_data0 = gain0;

	/* set max range */
	if (calc_data.gain_data0 == 0) {
		/* issue error value when gain is 0 */
		return CALC_ERROR;
	} else{
		max_range = MAX_OUTRANGE / calc_data.gain_data0;
	}

	/* calculate data */
	if (calc_data.als_data0 == MAXRANGE_NMODE) {
		calc_ans.positive = max_range;
		calc_ans.decimal  = 0;
	} else{
		/* get the value which is measured from power table */
		calc_data.als_time = time;
		if (calc_data.als_time == 0) {
			/* issue error value when time is 0 */
			return CALC_ERROR;
		}

	calc_judge = calc_data.als_data1 * JUDGE_FIXED_COEF;
	if (calc_judge < (data0 * judge_coefficient[0]))
		set_case = 0;
	else if (calc_judge < (data0*judge_coefficient[1]))
		set_case = 1;
	else if (calc_judge < (data0*judge_coefficient[2]))
		set_case = 2;
	else if (calc_judge < (data0*judge_coefficient[3]))
		set_case = 3;
	else
		set_case = MAXSET_CASE;

	calc_ans.positive = 0;
	if (set_case >= MAXSET_CASE) {
		calc_ans.decimal = 0;
	} else{
		calc_data.gain_data1 = gain1;
		if (calc_data.gain_data1 == 0) {
			/* issue error value when gain is 0 */
			return CALC_ERROR;
		}
		calc_data.data0 = (unsigned long long)
			(data0_coefficient[set_case]
			* calc_data.als_data0) * calc_data.gain_data1;
		calc_data.data1 = (unsigned long long)
			(data1_coefficient[set_case]
			* calc_data.als_data1) * calc_data.gain_data0;
		if (calc_data.data0 < calc_data.data1)
			return CALC_ERROR;
		else
			calc_data.data = (calc_data.data0 -
				calc_data.data1);
		/* 24 bit at max (128 * 128 * 100 * 10) */
		calc_data.dev_unit = calc_data.gain_data0 *
			calc_data.gain_data1 * calc_data.als_time * 10;
		if (calc_data.dev_unit == 0) {
			/* issue error value when dev_unit is 0 */
			return CALC_ERROR;
		}

		/* calculate a positive number */
		div_answer   = 0;
		div_overplus = 0;

		result = long_long_divider(calc_data.data,
			calc_data.dev_unit, &div_answer,
			&div_overplus);
		if (result == CALC_ERROR)
			return result;
		calc_ans.positive = div_answer;
		/* calculate a decimal number */
		calc_ans.decimal = 0;
		overplus         = div_overplus;
		if (calc_ans.positive < max_range) {
			if (overplus != 0) {
				overplus     = overplus << DECIMAL_BIT;
				div_answer   = 0;
				div_overplus = 0;

				result = long_long_divider(overplus,
					calc_data.dev_unit,
					&div_answer, &div_overplus);
				if (result == CALC_ERROR)
					return result;
				calc_ans.decimal = div_answer;
			}
		}

		else{
			calc_ans.positive = max_range;
		}
	}
	}

	final_data = calc_ans.positive * CUT_UNIT +
		((calc_ans.decimal * CUT_UNIT) >> DECIMAL_BIT);

	return final_data;

#undef DECIMAL_BIT
#undef JUDGE_FIXED_COEF
#undef MAX_OUTRANGE
#undef MAXRANGE_NMODE
#undef MAXSET_CASE
}


/******************************************************************************
 * NAME       : calculate_ps_data
 * FUNCTION   : calculate proximity data for rpr521
 * REMARKS    : 12 bit output
 *****************************************************************************/
static int calc_rohm_ps_data(unsigned short ps_reg_data)
{
		return ps_reg_data & 0xFFF;
}

static unsigned int rpr521_als_data_to_level(unsigned int als_data)
{
	return als_data;
}

static void rpr521_reschedule_work(struct ALS_PS_DATA *als_ps,
						unsigned long delay)
{
	unsigned long flags;

	spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags);

/*
 * If work is already scheduled then
 * subsequent schedules will not
 * change the scheduled time that's
 * why we have to cancel it first.
 */
	cancel_delayed_work(&als_ps->dwork);
	schedule_delayed_work(&als_ps->dwork, delay);

	spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);
}

static int rpr521_set_als_poll_delay(struct i2c_client *client,
		unsigned int val)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long flags;

	if (val < PS_ALS_SET_MIN_DELAY_TIME * 1000)
		val = PS_ALS_SET_MIN_DELAY_TIME * 1000;

	als_ps->als_poll_delay = val / 1000;	/* convert us => ms */

	if (als_ps->als_enable_flag == 1) {
		spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags);
		cancel_delayed_work(&als_ps->als_dwork);
		schedule_delayed_work(&als_ps->als_dwork,
		msecs_to_jiffies(als_ps->als_poll_delay));
		spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);
	}

	return 0;
}

/* ALS polling routine */
static void rpr521_als_polling_work_handler(struct work_struct *work)
{
	struct ALS_PS_DATA *als_ps = container_of(work,
		struct ALS_PS_DATA, als_dwork.work);
	struct i2c_client *client = als_ps->client;
	ktime_t timestamp;
	int tmp = 0;

	timestamp = ktime_get();
	tmp = i2c_smbus_read_word_data(client, REG_ALSDATA0);
	if (tmp < 0)
		pr_debug("%s: i2c read data0 fail.\n", __func__);
	als_ps->als_data0_raw = (unsigned short)tmp;
	tmp = i2c_smbus_read_word_data(client, REG_ALSDATA1);
	if (tmp < 0)
		pr_debug("%s: i2c read data1 fail.\n", __func__);

	als_ps->als_data1_raw = (unsigned short)tmp;

/* Theorically it is not necesssary to do so,
 * but I just want to avoid any potential error.  -- Andy 2012.6.6
 */
	tmp = i2c_smbus_read_byte_data(client, REG_ALSPSCONTROL);
	if (tmp < 0)
		pr_debug("%s: i2c read gain fail.\n", __func__);

	tmp = (tmp & 0x3C) >> 2;
	als_ps->gain0 = gain_table[tmp].DATA0;
	als_ps->gain1 = gain_table[tmp].DATA1;

	tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
	pr_debug("REG_MODECONTROL=%x\n", tmp);

	if (tmp < 0)
		pr_debug("%s: i2c read time fail.\n", __func__);

	tmp = tmp & 0xF;
	als_ps->als_time = mode_table[tmp].ALS;

	als_ps->als_data = calc_rohm_als_data(als_ps->als_data0_raw,
		als_ps->als_data1_raw, als_ps->gain0,
		als_ps->gain1, als_ps->als_time);
	if (als_ps->als_data == 0)
		als_ps->als_data++;

	als_ps->als_level = rpr521_als_data_to_level(als_ps->als_data);

	pr_debug("rpr521 als report: data0 = %d, data1 = %d\n",
		als_ps->als_data0_raw, als_ps->als_data1_raw);
	pr_debug("rpr521 als report: gain0 = %d, gain1 = %d.\n",
		als_ps->gain0, als_ps->gain1);
	pr_debug("rpr521 als report: time = %d, lux = %d, level = %d.\n",
		als_ps->als_time, als_ps->als_data, als_ps->als_level);

	tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
	pr_debug("REG_PSDATA=%x\n", tmp);

	if (als_ps->als_data != CALC_ERROR)
		input_report_abs(als_ps->input_dev_als,
			ABS_MISC, als_ps->als_level);
	input_event(als_ps->input_dev_als, EV_SYN, SYN_TIME_SEC,
			ktime_to_timespec(timestamp).tv_sec);
	input_event(als_ps->input_dev_als, EV_SYN, SYN_TIME_NSEC,
			ktime_to_timespec(timestamp).tv_nsec);
	input_sync(als_ps->input_dev_als);
	schedule_delayed_work(&als_ps->als_dwork,
		msecs_to_jiffies(als_ps->als_poll_delay));

}


/* PS interrupt routine */
static void rpr521_ps_int_work_handler(struct work_struct *work)
{
	struct ALS_PS_DATA *als_ps = container_of((struct delayed_work *)work,
		struct ALS_PS_DATA, dwork);
	struct i2c_client *client = als_ps->client;
	ktime_t timestamp;
	int tmp;

	timestamp = ktime_get();
	pr_debug("rpr521_ps_int_work_handler\n");

	tmp =  i2c_smbus_read_byte_data(client, REG_INTERRUPT);
	if (tmp < 0) {
		pr_debug("%s: i2c read interrupt status fail.\n", __func__);
		goto err_exit;
	}
	if (tmp & PS_INT_MASK) {
		tmp = i2c_smbus_read_byte_data(client, REG_ALSPSCONTROL);
		if (tmp < 0) {
			pr_debug("%s:read ir current fail.\n", __func__);

			goto err_exit;
		}
		als_ps->ledcurrent = tmp & 0x3;

		tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
		if (tmp < 0) {
			pr_debug("%s: i2c read ps data fail.\n", __func__);

			goto err_exit;
		}
		als_ps->ps_data_raw = (unsigned short)tmp;
		als_ps->ps_data = calc_rohm_ps_data(als_ps->ps_data_raw);

		if (als_ps->ps_data > als_ps->ps_th_h) {
			tmp = i2c_smbus_read_byte_data(client, REG_PERSISTENCE);
			if (tmp < 0) {
				pr_debug("%s:read current fail.\n", __func__);
				goto err_exit;
			}
			if ((tmp>>6) == INFRARED_LOW) {
				als_ps->ps_direction = 0;
#ifdef _AUTO_THRESHOLD_CHANGE_
			rpr521_set_ps_threshold_high(client, REG_PSTH_MAX);
			rpr521_set_ps_threshold_low(client,
				als_ps->ps_th_l_back);
#endif
			} else{
				goto err_exit;
			}
		} else if (als_ps->ps_data < als_ps->ps_th_l) {
			als_ps->ps_direction = 1;
#ifdef _AUTO_THRESHOLD_CHANGE_
		rpr521_set_ps_threshold_high(client, als_ps->ps_th_h_back);
		rpr521_set_ps_threshold_low(client, 0);
#endif
		}

		tmp = i2c_smbus_read_byte_data(client, REG_PERSISTENCE);
		if (tmp < 0) {
			pr_debug("%s: i2c read persistence fail.\n", __func__);
			goto err_exit;
		}

		pr_debug("PS:raw_data=%d, data=%d, direction=%d, 0x43=%d.\n",
			als_ps->ps_data_raw, als_ps->ps_data,
			als_ps->ps_direction, tmp);

		pr_debug("ps_high = %d, ps_low = %d, ps_high_back = %d, ps_low_back = %d.\n",
			i2c_smbus_read_word_data(client, REG_PSTH),
			i2c_smbus_read_word_data(client, REG_PSTL),
			als_ps->ps_th_h_back, als_ps->ps_th_l_back);


		input_report_abs(als_ps->input_dev_ps,
			ABS_DISTANCE, als_ps->ps_direction);
		input_event(als_ps->input_dev_ps, EV_SYN, SYN_TIME_SEC,
				ktime_to_timespec(timestamp).tv_sec);
		input_event(als_ps->input_dev_ps, EV_SYN, SYN_TIME_NSEC,
				ktime_to_timespec(timestamp).tv_nsec);
		input_sync(als_ps->input_dev_ps);
	} else{
		pr_debug("%s: unknown interrupt source.\n", __func__);
	}
	enable_irq(client->irq);

	return;

err_exit:
	enable_irq(client->irq);
}

/* assume this is ISR */
static irqreturn_t rpr521_interrupt(int vec, void *info)
{
	struct i2c_client *client = (struct i2c_client *)info;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);

	disable_irq_nosync(client->irq);

	pr_debug("%s\n", __func__);
	rpr521_reschedule_work(als_ps, 0);

	return IRQ_HANDLED;
}

static int rpr521_enable_als_sensor(struct i2c_client *client, int val)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	struct rpr521_platform_data *pdata = als_ps->platform_data;

	unsigned long flags;
	int tmp;

	pr_debug("%s: val=%d\n", __func__, val);

	if ((val != 0) && (val != 1)) {
		pr_err("%s: invalid value (val = %d)\n", __func__, val);
		return -EINVAL;
	}
	/* reinit device after resume */
	if (val == 1) {
		if ((als_ps->als_enable_flag == 0)
			&& (als_ps->ps_enable_flag == 0)) {

			/* power on reinit the device */
			if (pdata->power_on)
				pdata->power_on(true);

			rpr521_init_client_power_on(client);
		}
		tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
		pr_debug("%s: tmp=%x\n", __func__, tmp);
		tmp = tmp | 0x80;

		if (als_ps->als_enable_flag == 0)
			rpr521_set_enable(client, tmp);
		/*}*/

		if (als_ps->als_enable_flag == 0)
			als_ps->als_enable_flag = 1;
		spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags);
				cancel_delayed_work(&als_ps->als_dwork);
		schedule_delayed_work(&als_ps->als_dwork,
			msecs_to_jiffies(als_ps->als_poll_delay));
		spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);
	} else{
			als_ps->als_enable_flag = 0;
			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			tmp = tmp & (~0x80);
			rpr521_set_enable(client, tmp);
/*
		if (als_ps->enable_als_sensor == 1) {
			als_ps->enable_als_sensor = 0;
			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			tmp = tmp & (~0x80);
			rpr521_set_enable(client, tmp);
		}
*/
		spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags);
		cancel_delayed_work(&als_ps->als_dwork);
		spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);
	}

	/* Vote off  regulators if both light and prox sensor are off */
	if ((als_ps->als_enable_flag == 0) &&
		(als_ps->ps_enable_flag == 0) &&
		(pdata->power_on))
		pdata->power_on(false);

	return 0;
}

/*************** SysFS Support ******************/
static ssize_t rpr521_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "%d\n", als_ps->ps_enable_flag);
}

static ssize_t rpr521_store_enable_ps_sensor(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	/* unsigned long val = simple_strtoul(buf, NULL, 10); */
	unsigned long val;
	int tmp, error;

	error = kstrtoul(buf, 10, &val);
	if (error)
		return error;

	pr_info("rpr521 enable PS sensor -> %ld\n", val);

	if ((val != 0) && (val != 1)) {
		pr_debug("%s:store unvalid value=%ld\n", __func__, val);
		return count;
	}

	if (val == 1) {
		if (als_ps->ps_enable_flag == 0) {
			als_ps->ps_enable_flag = 1;

			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			pr_debug("%s: tmp=%x\n", __func__, tmp);
			tmp = tmp | 0x40;
			rpr521_set_enable(client, tmp);
		}
	} else{
		if (als_ps->ps_enable_flag == 1) {
			als_ps->ps_enable_flag = 0;

			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			tmp = tmp & (~0x40);
			rpr521_set_enable(client, tmp);
		}
	}

	return count;
}

static ssize_t rpr521_show_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "%d\n", als_ps->als_enable_flag);
}

static ssize_t rpr521_store_enable_als_sensor(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	/*unsigned long val = simple_strtoul(buf, NULL, 10);*/
	unsigned long flags;
	unsigned long val;
	int tmp, error;

	error = kstrtoul(buf, 10, &val);
	if (error)
		return error;

	if ((val != 0) && (val != 1)) {
		pr_debug("%s: enable als sensor=%ld\n", __func__, val);
		return count;
	}

	if (val == 1) {
		/* turn on light  sensor */
		if (als_ps->als_enable_flag == 0) {
			als_ps->als_enable_flag = 1;

			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			pr_debug("%s: tmp=%x\n", __func__, tmp);
			tmp = tmp | 0x80;
			rpr521_set_enable(client, tmp);
		}

		spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags);
			 cancel_delayed_work(&als_ps->als_dwork);
		schedule_delayed_work(&als_ps->als_dwork,
			msecs_to_jiffies(als_ps->als_poll_delay));
		spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);
	} else{
		if (als_ps->als_enable_flag == 1) {
			als_ps->als_enable_flag = 0;

			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			tmp = tmp & (~0x80);
			rpr521_set_enable(client, tmp);
		}

		spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags);
		cancel_delayed_work(&als_ps->als_dwork);
		spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);
	}

	return count;
}

static int rpr521_als_set_enable(struct sensors_classdev *sensors_cdev,
			unsigned int enable)
{
	struct ALS_PS_DATA *als_ps = container_of(sensors_cdev,
			struct ALS_PS_DATA, als_cdev);

	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	return rpr521_enable_als_sensor(als_ps->client, enable);
}
static int rpr521_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct ALS_PS_DATA *als_ps = container_of(sensors_cdev,
			struct ALS_PS_DATA, ps_cdev);

	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	return rpr521_enable_ps_sensor(als_ps->client, enable);
}
/******************************************************************************/

static ssize_t rpr521_show_als_poll_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "%d\n", als_ps->als_poll_delay*1000);
}

static ssize_t rpr521_store_als_poll_delay(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	/*unsigned long val = simple_strtoul(buf, NULL, 10);*/
	unsigned long flags;
	unsigned long val;
	int error;

	error = kstrtoul(buf, 10, &val);
	if (error)
		return error;

	if (val < PS_ALS_SET_MIN_DELAY_TIME * 1000)
		val = PS_ALS_SET_MIN_DELAY_TIME * 1000;

	als_ps->als_poll_delay = val / 1000;

	if (als_ps->als_enable_flag == 1) {
		spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags);
		cancel_delayed_work(&als_ps->als_dwork);
		schedule_delayed_work(&als_ps->als_dwork,
		msecs_to_jiffies(als_ps->als_poll_delay));
		spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);
	}

	return count;
}

static int rpr521_als_poll_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct ALS_PS_DATA *als_ps = container_of(sensors_cdev,
			struct ALS_PS_DATA, als_cdev);
	rpr521_set_als_poll_delay(als_ps->client, delay_msec);
	return 0;
}

static int rpr521_cdev_ps_flush(struct sensors_classdev *sensors_cdev)
{
	struct ALS_PS_DATA *als_ps = container_of(sensors_cdev,
			struct ALS_PS_DATA, ps_cdev);

	input_event(als_ps->input_dev_ps, EV_SYN, SYN_CONFIG,
			als_ps->flush_count++);
	input_sync(als_ps->input_dev_ps);

	return 0;
}

static int rpr521_cdev_als_flush(struct sensors_classdev *sensors_cdev)
{
	struct ALS_PS_DATA *als_ps = container_of(sensors_cdev,
			struct ALS_PS_DATA, als_cdev);

	input_event(als_ps->input_dev_als, EV_SYN, SYN_CONFIG,
		als_ps->flush_count++);
	input_sync(als_ps->input_dev_als);

	return 0;
}
static int rpr521_enable_ps_sensor(struct i2c_client *client, int val)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	struct rpr521_platform_data *pdata = als_ps->platform_data;
	int tmp;

	pr_debug("rpr521 enable PS sensor -> %d\n", val);

	if ((val != 0) && (val != 1)) {
		pr_debug("%s:store unvalid value=%d\n", __func__, val);
		return -EINVAL;
	}

	if (val == 1) {
			if ((als_ps->als_enable_flag == 0) &&
				(als_ps->ps_enable_flag == 0)) {
				/**** Power on and initalize the device ****/
				if (pdata->power_on)
					pdata->power_on(true);
			}

			rpr521_init_client_power_on(client);

			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			pr_debug("%s: tmp=%x\n", __func__, tmp);
			tmp = tmp | 0x40;

			if (als_ps->ps_enable_flag == 0) {

				als_ps->ps_enable_flag = 1;
				rpr521_set_enable(client, tmp);
			}
	} else{
			als_ps->ps_enable_flag = 0;
			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			tmp = tmp & (~0x40);
			rpr521_set_enable(client, tmp);

	}

	/* when enable auto change backlight,
	* als_enable should be reset when ps
	* disabled after use
	*/
	if ((val == 0) && (als_ps->als_enable_flag == 1)) {
		tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
		pr_debug("%s: tmp=%x\n", __func__, tmp);
		tmp = tmp | 0x80;
		rpr521_set_enable(client, tmp);
	}

	/* Vote off  regulators if both light and prox sensor are off */
	if ((als_ps->als_enable_flag == 0) &&
		(als_ps->ps_enable_flag == 0) &&
		(pdata->power_on))
		pdata->power_on(false);

	return 0;
}

static ssize_t rpr521_show_als_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int tmp;
	int tmp1;
	tmp = i2c_smbus_read_word_data(client, REG_ALSDATA0);
	tmp1 = i2c_smbus_read_word_data(client, REG_ALSDATA1);

	return snprintf(buf, PAGE_SIZE, "%d %d\n", tmp, tmp1);
}

static ssize_t rpr521_show_ps_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int tmp = 0;
	tmp = i2c_smbus_read_word_data(client, REG_PSDATA);

	return snprintf(buf, PAGE_SIZE, "%d\n", tmp);
}

static ssize_t rpr521_show_ps_thres_high(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ps_data = 0, ps_high = 0, ps_low = 0;

	ps_data = i2c_smbus_read_word_data(client, REG_PSDATA);
	if (ps_data < 0) {
			pr_debug("%s: i2c read led current fail.\n", __func__);
			return -EPERM;
	}

	ps_high = i2c_smbus_read_word_data(client, REG_PSTH);
	if (ps_high < 0) {
			pr_debug("%s: i2c read led current fail.\n", __func__);
			return -EPERM;
	}

	ps_low = i2c_smbus_read_word_data(client, REG_PSTL);
	if (ps_low < 0) {
			pr_debug("%s: i2c read led current fail.\n", __func__);
			return -EPERM;
	}

	return snprintf(buf, PAGE_SIZE, "%d %d %d\n", ps_data, ps_high, ps_low);
}

static ssize_t rpr521_store_ps_thres_high(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	/*unsigned long val = simple_strtoul(buf, NULL, 10);*/
	unsigned long val;
	int	error;

	error = kstrtoul(buf, 10, &val);
	if (error)
		return error;

	if (!(rpr521_set_ps_threshold_high(client, val)))
		als_ps->ps_th_h_back = als_ps->ps_th_h;

	return count;
}

static ssize_t rpr521_show_ps_thres_low(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "%d\n", als_ps->ps_th_l);
}

static ssize_t rpr521_store_ps_thres_low(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	/*unsigned long val = simple_strtoul(buf, NULL, 10);*/
	unsigned long val;
	int error;

	error = kstrtoul(buf, 10, &val);
	if (error)
		return error;

	if (!(rpr521_set_ps_threshold_low(client, val)))
		als_ps->ps_th_l_back = als_ps->ps_th_l;

	return count;
}

static ssize_t rpr521_show_ps_calib(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "%d\t%d\n",
		als_ps->ps_th_h, als_ps->ps_th_l);
}

static ssize_t rpr521_store_ps_calib(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
#define SET_LOW_THRES	1
#define SET_HIGH_THRES	2
#define SET_BOTH_THRES	3

	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	/*unsigned long val = simple_strtoul(buf, NULL, 10);*/
	unsigned int i, tmp, ps_th_h, ps_th_l;
	int average;	/* This should be signed to avoid error. */
	unsigned long val;
	int error;

	error = kstrtoul(buf, 10, &val);
	if (error)
		return error;

	switch (val) {
	case SET_LOW_THRES:
		average = 0;
		ps_th_h = als_ps->ps_th_h_back;
		ps_th_l = als_ps->ps_th_l_back;
		for (i = 0; i < 20; i++) {
			tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
			if (tmp < 0) {
				pr_debug("%s:read ps fail.\n", __func__);
				return -EPERM;
			}
			average += tmp & 0xFFF;
		}
		average /= 20;
		ps_th_l = average + THRES_TOLERANCE;
		if (ps_th_l > REG_PSTL_MAX)
			return -EPERM;
		if (ps_th_l < 0)
			return -EPERM;
		if (ps_th_h < ps_th_l + THRES_DIFF) {
			ps_th_h = ps_th_l + THRES_DIFF;
			if (ps_th_h > REG_PSTH_MAX)
				return -EPERM;
			if (!rpr521_set_ps_threshold_high(client, ps_th_h))
				als_ps->ps_th_h_back = ps_th_h;
			else
				return -EPERM;
		}
		if (!rpr521_set_ps_threshold_low(client, ps_th_l))
			als_ps->ps_th_l_back = ps_th_l;
		else
			return -EPERM;
		break;

	case SET_HIGH_THRES:
		average = 0;
		ps_th_h = als_ps->ps_th_h_back;
		ps_th_l = als_ps->ps_th_l_back;
		for (i = 0; i < 20; i++) {
			tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
			if (tmp < 0) {
				pr_debug("%s:read ps data fail.\n", __func__);
				return -EPERM;
			}
			average += tmp & 0xFFF;
		}
		average /= 20;
		ps_th_h = average - THRES_TOLERANCE;
		if (ps_th_h > REG_PSTH_MAX)
			return -EPERM;

		if (ps_th_h < 0)
			return -EPERM;

		if (ps_th_l > ps_th_h - THRES_DIFF) {
			ps_th_l = ps_th_h - THRES_DIFF;
			if (ps_th_l < 0)
				return -EPERM;

			if (!rpr521_set_ps_threshold_low(client, ps_th_l))
				als_ps->ps_th_l_back = ps_th_l;
			else
				return -EPERM;
		}
		if (!rpr521_set_ps_threshold_high(client, ps_th_h))
			als_ps->ps_th_h_back = ps_th_h;
		else
			return -EPERM;
		break;

	case SET_BOTH_THRES:
		rpr521_calibrate(client);
		break;

	default:
		return -EINVAL;
	}

	return count;

#undef SET_BOTH_THRES
#undef SET_HIGH_THRES
#undef SET_LOW_THRES
}

static ssize_t rpr521_show_type(struct device *dev,
				struct device_attribute *attr,
				char *buf){
		struct i2c_client *client = to_i2c_client(dev);
		struct ALS_PS_DATA *data = i2c_get_clientdata(client);

		return snprintf(buf, PAGE_SIZE, "%d\n", data->type);
}

static DEVICE_ATTR(als_poll_delay,  0660,
						rpr521_show_als_poll_delay,
						rpr521_store_als_poll_delay);

static DEVICE_ATTR(enable_als_sensor,  0660 ,
					rpr521_show_enable_als_sensor,
					rpr521_store_enable_als_sensor);

static DEVICE_ATTR(enable_ps_sensor,  0660 ,
					 rpr521_show_enable_ps_sensor,
					 rpr521_store_enable_ps_sensor);

static DEVICE_ATTR(ps_thres_high,  0660 ,
					rpr521_show_ps_thres_high,
					rpr521_store_ps_thres_high);

static DEVICE_ATTR(ps_thres_low,  0660 ,
					 rpr521_show_ps_thres_low,
					 rpr521_store_ps_thres_low);

static DEVICE_ATTR(ps_calib,  0660 ,
					 rpr521_show_ps_calib,
					 rpr521_store_ps_calib);
static DEVICE_ATTR(als_data, S_IRUGO, rpr521_show_als_data, NULL);
static DEVICE_ATTR(ps_data, S_IRUGO, rpr521_show_ps_data, NULL);
static DEVICE_ATTR(type, S_IRUGO, rpr521_show_type, NULL);

static struct attribute *rpr521_attributes[] = {
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_enable_als_sensor.attr,
	&dev_attr_als_poll_delay.attr,
	&dev_attr_ps_thres_high.attr,
	&dev_attr_ps_thres_low.attr,
	&dev_attr_ps_calib.attr,
	&dev_attr_als_data.attr,
	&dev_attr_ps_data.attr,
	&dev_attr_type.attr,
	NULL
};

static const struct attribute_group rpr521_attr_group = {
	.attrs = rpr521_attributes,
};

/*************** Initialze Functions ******************/
static int rpr521_init_client(struct i2c_client *client)
{
		struct init_func_write_data {
				unsigned char mode_ctl;
				unsigned char psals_ctl;
				unsigned char persist;
				unsigned char reserved0;
				unsigned char reserved1;
				unsigned char reserved2;
				unsigned char reserved3;
				unsigned char reserved4;
				unsigned char reserved5;
				unsigned char intr;
				unsigned char psth_hl;
				unsigned char psth_hh;
				unsigned char psth_ll;
				unsigned char psth_lh;
				unsigned char alsth_hl;
				unsigned char alsth_hh;
				unsigned char alsth_ll;
				unsigned char alsth_lh;
		} write_data;

	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
		int result;
		unsigned char gain;

		unsigned char mode_ctl    = PS_ALS_SET_MODE_CONTROL;
		unsigned char psals_ctl   = PS_ALS_SET_ALSPS_CONTROL;
		unsigned char persist     = PS_ALS_SET_INTR_PERSIST;
		unsigned char intr        = PS_ALS_SET_INTR;
		unsigned short psth_upper  = als_ps->ps_th_h;
		unsigned short psth_low    = als_ps->ps_th_l;
		unsigned short alsth_upper = PS_ALS_SET_ALS_TH;
		unsigned short alsth_low   = PS_ALS_SET_ALS_TL;

		/* execute software reset */
		result =  i2c_smbus_write_byte_data(client,
			REG_SYSTEMCONTROL, 0xC0);
		if (result != 0)
				return result;

		write_data.mode_ctl  = mode_ctl;
		write_data.psals_ctl = psals_ctl;
		write_data.persist   = persist;
		write_data.reserved0 = 0;
		write_data.reserved1 = 0;
		write_data.reserved2 = 0;
		write_data.reserved3 = 0;
		write_data.reserved4 = 0;
		write_data.reserved5 = 0;
		write_data.intr      = intr;
		write_data.psth_hl   = CONVERT_TO_BE(psth_upper) & MASK_CHAR;
		write_data.psth_hh   = CONVERT_TO_BE(psth_upper) >> 8;
		write_data.psth_ll   = CONVERT_TO_BE(psth_low) & MASK_CHAR;
		write_data.psth_lh   = CONVERT_TO_BE(psth_low) >> 8;
		write_data.alsth_hl  = CONVERT_TO_BE(alsth_upper) & MASK_CHAR;
		write_data.alsth_hh  = CONVERT_TO_BE(alsth_upper) >> 8;
		write_data.alsth_ll  = CONVERT_TO_BE(alsth_low) & MASK_CHAR;
		write_data.alsth_lh  = CONVERT_TO_BE(alsth_low) >> 8;
		result = i2c_smbus_write_i2c_block_data(client, REG_MODECONTROL,
			sizeof(write_data), (unsigned char *)&write_data);

	if (result < 0) {
		pr_debug("%s: i2c write fail.\n", __func__);
		return result;
	}

	gain = (psals_ctl & 0x3C) >> 2;

	als_ps->enable = mode_ctl;
	als_ps->als_time = mode_table[(mode_ctl & 0xF)].ALS;
	als_ps->ps_time = mode_table[(mode_ctl & 0xF)].PS;
	als_ps->persistence = persist;
	als_ps->als_th_l = alsth_low;
	als_ps->als_th_h = alsth_upper;
	als_ps->control = psals_ctl;
	als_ps->gain0 = gain_table[gain].DATA0;
	als_ps->gain1 = gain_table[gain].DATA1;
	als_ps->ledcurrent = psals_ctl & 0x03;

	als_ps->als_enable_flag = 0;
	als_ps->ps_enable_flag = 0;
#ifdef _INIT_CALIB_
	rpr521_calibrate(client);
#else
	als_ps->ps_th_h_back = als_ps->ps_th_h;
	als_ps->ps_th_l_back = als_ps->ps_th_l;
#endif

		return result;
}

static int rpr521_init_client_power_on(struct i2c_client *client)
{
		struct init_func_write_data {
				unsigned char mode_ctl;
				unsigned char psals_ctl;
				unsigned char persist;
				unsigned char reserved0;
				unsigned char reserved1;
				unsigned char reserved2;
				unsigned char reserved3;
				unsigned char reserved4;
				unsigned char reserved5;
				unsigned char intr;
				unsigned char psth_hl;
				unsigned char psth_hh;
				unsigned char psth_ll;
				unsigned char psth_lh;
				unsigned char alsth_hl;
				unsigned char alsth_hh;
				unsigned char alsth_ll;
				unsigned char alsth_lh;
		} write_data;

	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
		int result;
		unsigned char gain;

		unsigned char mode_ctl    = PS_ALS_SET_MODE_CONTROL;
		unsigned char psals_ctl   = PS_ALS_SET_ALSPS_CONTROL;
		unsigned char persist     = PS_ALS_SET_INTR_PERSIST;
		unsigned char intr        = PS_ALS_SET_INTR;
/*		unsigned short psth_upper  = als_ps->ps_th_h;
		unsigned short psth_low    = als_ps->ps_th_l;*/
		unsigned short alsth_upper = PS_ALS_SET_ALS_TH;
		unsigned short alsth_low   = PS_ALS_SET_ALS_TL;

		/* execute software reset */
		result =  i2c_smbus_write_byte_data(client,
			REG_SYSTEMCONTROL, 0xC0);
		if (result != 0)
				return result;

		write_data.mode_ctl  = mode_ctl;
		write_data.psals_ctl = psals_ctl;
		write_data.persist   = persist;
		write_data.reserved0 = 0;
		write_data.reserved1 = 0;
		write_data.reserved2 = 0;
		write_data.reserved3 = 0;
		write_data.reserved4 = 0;
		write_data.reserved5 = 0;
		write_data.intr      = intr;
/*		write_data.psth_hl   = CONVERT_TO_BE(psth_upper) & MASK_CHAR;
		write_data.psth_hh   = CONVERT_TO_BE(psth_upper) >> 8;
		write_data.psth_ll   = CONVERT_TO_BE(psth_low) & MASK_CHAR;
		write_data.psth_lh   = CONVERT_TO_BE(psth_low) >> 8;*/
		write_data.alsth_hl  = CONVERT_TO_BE(alsth_upper) & MASK_CHAR;
		write_data.alsth_hh  = CONVERT_TO_BE(alsth_upper) >> 8;
		write_data.alsth_ll  = CONVERT_TO_BE(alsth_low) & MASK_CHAR;
		write_data.alsth_lh  = CONVERT_TO_BE(alsth_low) >> 8;
		result = i2c_smbus_write_i2c_block_data(client, REG_MODECONTROL,
			sizeof(write_data), (unsigned char *)&write_data);

	if (result < 0) {
		pr_debug("%s: i2c write fail.\n", __func__);
		return result;
	}

	gain = (psals_ctl & 0x3C) >> 2;

	als_ps->enable = mode_ctl;
	als_ps->als_time = mode_table[(mode_ctl & 0xF)].ALS;
	als_ps->ps_time = mode_table[(mode_ctl & 0xF)].PS;
	als_ps->persistence = persist;
	als_ps->als_th_l = alsth_low;
	als_ps->als_th_h = alsth_upper;
	als_ps->control = psals_ctl;
	als_ps->gain0 = gain_table[gain].DATA0;
	als_ps->gain1 = gain_table[gain].DATA1;
	als_ps->ledcurrent = psals_ctl & 0x03;

	/* als_ps->als_enable_flag = 0; */
	/* als_ps->enable_ps_sensor = 0; */
#ifdef _INIT_CALIB_
	rpr521_calibrate(client);
#else
	/*als_ps->ps_th_h_back = als_ps->ps_th_h;
	als_ps->ps_th_l_back = als_ps->ps_th_l;*/
#endif

	return result;
}
/*****************regulator configuration start**************/
static int sensor_regulator_configure(struct ALS_PS_DATA *data, bool on)
{
	int rc;

	if (!on) {

		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				RPR0521C_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				RPR0521C_VIO_MAX_UV);

		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				RPR0521C_VDD_MIN_UV, RPR0521C_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			dev_err(&data->client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				RPR0521C_VIO_MIN_UV, RPR0521C_VIO_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;
reg_vio_put:
	regulator_put(data->vio);

reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, RPR0521C_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

static int sensor_regulator_power_on(struct ALS_PS_DATA *data, bool on)
{
	int rc = 0;

	if (!on) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			rc = regulator_enable(data->vdd);
			dev_err(&data->client->dev,
					"Regulator vio re-enabled rc=%d\n", rc);
			/*
			 * Successfully re-enable regulator.
			 * Enter poweron delay and returns error.
			 */
			if (!rc) {
				rc = -EBUSY;
				goto enable_delay;
			}
		}
		return rc;
	} else {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			regulator_disable(data->vdd);
			return rc;
		}
	}

enable_delay:
	msleep(130);
	dev_dbg(&data->client->dev,
		"Sensor regulator power on =%d\n", on);
	return rc;
}

static int sensor_platform_hw_power_on(bool on)
{
	struct ALS_PS_DATA *data;
	int err = 0;

	/* sensor_info is global pointer to struct ltr553_data */
	if (sensor_info == NULL)
		return -ENODEV;

	data = sensor_info;

	if (data->power_on != on) {
		if (!IS_ERR_OR_NULL(data->pinctrl)) {
			if (on)
				err = pinctrl_select_state(data->pinctrl,
					data->pin_default);
			else
				err = pinctrl_select_state(data->pinctrl,
					data->pin_sleep);
			if (err)
				dev_err(&data->client->dev,
					"Can't select pinctrl state\n");
		}

		err = sensor_regulator_power_on(data, on);
		if (err)
			dev_err(&data->client->dev,
					"Can't configure regulator!\n");
		else
			data->power_on = on;
	}

	return err;
}

static int sensor_platform_hw_init(void)
{
	struct i2c_client *client;
	struct ALS_PS_DATA *data;
	int error;

	if (sensor_info == NULL)
		return -ENODEV;

	data = sensor_info;
	client = data->client;

	error = sensor_regulator_configure(data, true);
	if (error < 0) {
		dev_err(&client->dev, "unable to configure regulator\n");
		return error;
	}

	return 0;
}

static void sensor_platform_hw_exit(void)
{
	struct ALS_PS_DATA *data = sensor_info;

	if (data == NULL)
		return;

	sensor_regulator_configure(data, false);
}
/******************regulator ends***********************/
static int rpr521_pinctrl_init(struct ALS_PS_DATA *data)
{
	struct i2c_client *client = data->client;

	data->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		dev_err(&client->dev, "Failed to get pinctrl\n");
		return PTR_ERR(data->pinctrl);
	}

	data->pin_default =
		pinctrl_lookup_state(data->pinctrl, "default");
	if (IS_ERR_OR_NULL(data->pin_default)) {
		dev_err(&client->dev, "Failed to look up default state\n");
		return PTR_ERR(data->pin_default);
	}

	data->pin_sleep =
		pinctrl_lookup_state(data->pinctrl, "sleep");
	if (IS_ERR_OR_NULL(data->pin_sleep)) {
		dev_err(&client->dev, "Failed to look up sleep state\n");
		return PTR_ERR(data->pin_sleep);
	}

	return 0;
}

static int sensor_parse_dt(struct device *dev,
		struct rpr521_platform_data *rpr_pdata)
{
	struct device_node *np = dev->of_node;
	struct i2c_client *client;
	struct ALS_PS_DATA *data = sensor_info;

	unsigned int tmp;
	int rc = 0;
	client = data->client;

	/* set functions of platform data */
	rpr_pdata->init = sensor_platform_hw_init;
	rpr_pdata->exit = sensor_platform_hw_exit;
	rpr_pdata->power_on = sensor_platform_hw_power_on;

	/* irq gpio no. is data-> irq_gpio */
	data->irq_gpio = of_get_named_gpio_flags(np, "rpr0521c,irq-gpio",
				0, &data->irq_gpio_flags);
	if (data->irq_gpio < 0)
		return data->irq_gpio;

	if (gpio_is_valid(data->irq_gpio)) {
		rc = gpio_request(data->irq_gpio, "rpr_irq_gpio");
		if (rc)
			dev_err(&client->dev, "irq gpio request failed");

		rc = gpio_direction_input(data->irq_gpio);
		if (rc) {
			dev_err(&client->dev,
				"set_direction for irq gpio failed\n");
		}
	}

	/* ps tuning data*/
	rc = of_property_read_u32(np, "rpr0521c,prox_th_min", &tmp);
	if (rc) {
		dev_err(dev, "Unable to read prox_th_min\n");
		return rc;
	}

	data->ps_th_l = tmp;

	rc = of_property_read_u32(np, "rpr0521c,prox_th_max", &tmp);
	 if (rc) {
		dev_err(dev, "Unable to read prox_th_max\n");
		return rc;
	}

	data->ps_th_h = tmp;

	return 0;
}

static int rpr521_ps_set_calibrate_data(struct ALS_PS_DATA *als_ps)
{
	int average;
	unsigned int ps_th_h, ps_th_l;
	
	average = als_ps->ps_cal_params[2];
	
	/* rpr521_set_enable(client, 0x41); PS 10ms */
	rpr521_set_enable(als_ps->client, PS_EN|PS_DOUBLE_PULSE|PS10MS);


	ps_th_h = average + THRES_TOLERANCE + THRES_DEFAULT_DIFF;
	ps_th_l = average + THRES_TOLERANCE;

	if (ps_th_h < 0) {
		pr_debug("%s: high threshold is less than 0.\n", __func__);
		goto err_exit;
	}
	if (ps_th_h > REG_PSTH_MAX)
		goto err_exit;
	
	if (ps_th_l < 0) {
		pr_debug("%s: low threshold is less than 0.\n", __func__);
		goto err_exit;
	}
	if (ps_th_l > REG_PSTL_MAX)
		goto err_exit;

	if (!(rpr521_set_ps_threshold_high(als_ps->client, ps_th_h)))
		als_ps->ps_th_h_back = ps_th_h;
	else
		goto err_exit;
	if (!(rpr521_set_ps_threshold_low(als_ps->client, ps_th_l)))
		als_ps->ps_th_l_back = ps_th_l;
	else
		goto err_exit;

	/* rpr521_set_enable(client, 0); disable ps */
	rpr521_set_enable(als_ps->client, PS_ALS_SET_MODE_CONTROL);

	return 0;
	
err_exit:
	rpr521_set_enable(als_ps->client, PS_ALS_SET_MODE_CONTROL);

	return -EINVAL;

}


static int rpr521_ps_calibrate(struct sensors_classdev *sensors_cdev,
		int axis, int apply_now)
{
	struct ALS_PS_DATA *als_ps = container_of(sensors_cdev,
			struct ALS_PS_DATA, ps_cdev);
	int average;
	unsigned int i, tmp, ps_th_h, ps_th_l;
	int temp[3] = { 0 };

	average = 0;
	/* rpr521_set_enable(client, 0x41);	PS 10ms */
	rpr521_set_enable(als_ps->client, PS_EN|PS_DOUBLE_PULSE|PS10MS);

	for (i = 0; i < 20; i++) {
		msleep(20);
		tmp = i2c_smbus_read_word_data(als_ps->client, REG_PSDATA);
		if (tmp < 0)
			pr_debug("%s: i2c read ps data fail.\n", __func__);

		average += tmp & 0xFFF;
	}
	average /= 20;
	ps_th_h = average + THRES_TOLERANCE + THRES_DEFAULT_DIFF;
	ps_th_l = average + THRES_TOLERANCE;
	if (axis == AXIS_THRESHOLD_H)
		temp[0] = average;
	else if (axis == AXIS_THRESHOLD_L)
		temp[1] = average;
	else if (axis == AXIS_BIAS)
		temp[2] = average;
	if (apply_now) {
		als_ps->ps_cal_params[0] = temp[0];
		als_ps->ps_cal_params[1] = temp[1];
		als_ps->ps_cal_params[2] = temp[2];
		rpr521_ps_set_calibrate_data(als_ps);
	}
	memset(als_ps->calibrate_buf, 0 , sizeof(als_ps->calibrate_buf));
	snprintf(als_ps->calibrate_buf, sizeof(als_ps->calibrate_buf),
			"%d,%d,%d", temp[0], temp[1], temp[2]);
	sensors_cdev->params = als_ps->calibrate_buf;

	/* rpr521_set_enable(client, 0); disable ps */
	rpr521_set_enable(als_ps->client, PS_ALS_SET_MODE_CONTROL);

	return 0;
}

static int rpr521_ps_write_calibrate(struct sensors_classdev *sensors_cdev,
		struct cal_result_t *cal_result)
{
	struct ALS_PS_DATA *data = container_of(sensors_cdev,
			struct ALS_PS_DATA, ps_cdev);
	data->ps_cal_params[0] = cal_result->threshold_h;
	data->ps_cal_params[1] = cal_result->threshold_l;
	data->ps_cal_params[2] = cal_result->bias;
	rpr521_ps_set_calibrate_data(data);
	return 0;
}

static int rpr521_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
#define ROHM_PSALS_ALSMAX (65535)
#define ROHM_PSALS_PSMAX  (4095)

	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct ALS_PS_DATA *als_ps;
	struct rpr521_platform_data *pdata;

	int err = 0;
	int dev_id, ret;
	pr_info("%s probe started.\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	als_ps = kzalloc(sizeof(struct ALS_PS_DATA), GFP_KERNEL);
	if (!als_ps) {
		dev_err(&als_ps->client->dev,
			"%s: Mem Alloc Fail...\n", __func__);
		goto exit;
	}

	/* Set initial defaults */
	als_ps->als_enable_flag = 0;
	als_ps->ps_enable_flag = 0;

	/* Global pointer for this device */
	sensor_info = als_ps;
	als_ps->client = client;

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct rpr521_platform_data), GFP_KERNEL);

		if (!pdata) {
				dev_err(&client->dev,
				"failed to allocate memory for platform data\n");
				return -ENOMEM;
		}
		als_ps->platform_data = pdata;

		client->dev.platform_data = pdata;

		ret = sensor_parse_dt(&client->dev, pdata);
		if (ret) {
				dev_err(&client->dev,
		"Unable to parse platfrom data err=%d\n", ret);
				return ret;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			dev_err(&client->dev, "No platform data\n");
			return -ENODEV;
		}
	}

	/* initialize pinctrl */
	err = rpr521_pinctrl_init(als_ps);
	if (err) {
		dev_err(&client->dev, "Can't initialize pinctrl\n");
			goto exit_pinctrl;
	}
	err = pinctrl_select_state(als_ps->pinctrl, als_ps->pin_default);
	if (err) {
		dev_err(&client->dev,
			"Can't select pinctrl default state\n");
		goto exit_pinctrl;
	}

	/* h/w initialization */
	if (pdata->init)
		ret = pdata->init();

	if (pdata->power_on)
		ret = pdata->power_on(true);

	i2c_set_clientdata(client, als_ps);

	pr_debug("enable = %x\n", als_ps->enable);

	dev_id = i2c_smbus_read_byte_data(client, REG_MANUFACT_ID);
	if (dev_id != 0xE0) {
			kfree(als_ps);
			return -EPERM;
	}
	als_ps->type = 1;
	pr_debug("%s: id(0x%x), this is rpr521!\n", __func__, dev_id);

	mutex_init(&als_ps->update_lock);

	INIT_DELAYED_WORK(&als_ps->dwork, rpr521_ps_int_work_handler);
	INIT_DELAYED_WORK(&als_ps->als_dwork, rpr521_als_polling_work_handler);

	pr_info("%s interrupt is hooked\n", __func__);

	/* Initialize the RPR400 chip */
	err = rpr521_init_client(client);
	if (err)
		goto exit_kfree;


	als_ps->als_poll_delay = PS_ALS_SET_MIN_DELAY_TIME;

	/* Register to Input Device */
	als_ps->input_dev_als = input_allocate_device();
	if (!als_ps->input_dev_als) {
		err = -ENOMEM;
		pr_debug("%s: Failed to allocate input device als\n", __func__);
		goto exit_kfree;
	}

	als_ps->input_dev_ps = input_allocate_device();
	if (!als_ps->input_dev_ps) {
		err = -ENOMEM;
		pr_debug("%s: Failed to allocate input device ps\n", __func__);
		goto exit_free_dev_als;
	}

	input_set_drvdata(als_ps->input_dev_ps, als_ps);
	input_set_drvdata(als_ps->input_dev_als, als_ps);

	set_bit(EV_ABS, als_ps->input_dev_als->evbit);
	set_bit(EV_ABS, als_ps->input_dev_ps->evbit);

	input_set_abs_params(als_ps->input_dev_als,
		ABS_MISC, 0, ROHM_PSALS_ALSMAX, 0, 0);
	input_set_abs_params(als_ps->input_dev_ps,
		ABS_DISTANCE, 0, ROHM_PSALS_PSMAX, 0, 0);

	als_ps->input_dev_als->name = "light";
	als_ps->input_dev_ps->name = "proximity";
	als_ps->input_dev_als->id.bustype = BUS_I2C;
	als_ps->input_dev_als->dev.parent = &als_ps->client->dev;
	als_ps->input_dev_ps->id.bustype = BUS_I2C;
	als_ps->input_dev_ps->dev.parent = &als_ps->client->dev;

	err = input_register_device(als_ps->input_dev_als);
	if (err) {
		err = -ENOMEM;
		pr_debug("%s:register input device als fail: %s\n", __func__,
					 als_ps->input_dev_als->name);
		goto exit_free_dev_ps;
	}

	err = input_register_device(als_ps->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		pr_debug("%s:register input device ps fail: %s\n", __func__,
					 als_ps->input_dev_ps->name);
		goto exit_unregister_dev_als;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &rpr521_attr_group);
	if (err) {
		pr_debug("%s sysfs_create_groupX\n", __func__);
		goto exit_unregister_dev_ps;
	}

	pr_debug("%s support ver. %s enabled\n", __func__, DRIVER_VERSION);
	if (request_irq(client->irq, rpr521_interrupt, IRQF_TRIGGER_FALLING,
		RPR521_DRV_NAME, (void *)client)) {
		pr_debug("%s Could not allocate rpr521_INT !\n", __func__);

		goto exit_remove_sysfs_group;
	}

	irq_set_irq_wake(client->irq, 1);

	/* Register to sensors class */
	als_ps->als_cdev = sensors_light_cdev;
	als_ps->als_cdev.sensors_enable = rpr521_als_set_enable;
	als_ps->als_cdev.sensors_poll_delay = rpr521_als_poll_delay;
	als_ps->als_cdev.sensors_flush = rpr521_cdev_als_flush;

	als_ps->ps_cdev = sensors_proximity_cdev;
	als_ps->ps_cdev.sensors_enable = rpr521_ps_set_enable;
	als_ps->ps_cdev.sensors_poll_delay = NULL,
	als_ps->ps_cdev.sensors_flush = rpr521_cdev_ps_flush;
	als_ps->ps_cdev.sensors_calibrate = rpr521_ps_calibrate;
	als_ps->ps_cdev.sensors_write_cal_params = rpr521_ps_write_calibrate;
	memset(&als_ps->ps_cdev.cal_result, 0 , sizeof(als_ps->ps_cdev.cal_result));

	ret = sensors_classdev_register(&client->dev, &als_ps->als_cdev);
	if (ret) {
		pr_err("%s: Unable to register to sensors class: %d\n",
				__func__, ret);
		goto exit_free_irq;
	}
	ret = sensors_classdev_register(&client->dev, &als_ps->ps_cdev);
	if (ret) {
		pr_err("%s: Unable to register to sensors class: %d\n",
						 __func__, ret);
		goto exit_create_class_sysfs;
	}

	pr_info("%s: INT No. %d", __func__, client->irq);
	return 0;

exit_create_class_sysfs:
	sensors_classdev_unregister(&als_ps->als_cdev);
exit_free_irq:
	free_irq(client->irq, client);
exit_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &rpr521_attr_group);
exit_unregister_dev_ps:
	input_unregister_device(als_ps->input_dev_ps);
exit_unregister_dev_als:
	input_unregister_device(als_ps->input_dev_als);
exit_free_dev_ps:
	input_free_device(als_ps->input_dev_ps);
exit_free_dev_als:
	input_free_device(als_ps->input_dev_als);
exit_kfree:
	if (pdata->power_on)
		pdata->power_on(false);
	if (pdata->exit)
		pdata->exit();
exit_pinctrl:
	kfree(als_ps);
	sensor_info = NULL;
exit:
	return err;

#undef ROHM_PSALS_ALSMAX
#undef ROHM_PSALS_PSMAX
}

static int rpr521_remove(struct i2c_client *client)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);

	input_unregister_device(als_ps->input_dev_als);
	input_unregister_device(als_ps->input_dev_ps);

	input_free_device(als_ps->input_dev_als);
	input_free_device(als_ps->input_dev_ps);

	free_irq(client->irq, client);

	sysfs_remove_group(&client->dev.kobj, &rpr521_attr_group);

	/* Power down the device */
	rpr521_set_enable(client, 0);

	kfree(als_ps);

	return 0;
}
/*
static int rpr521_suspend(struct i2c_client *client, pm_message_t mesg)
{
	pr_info("%s\n", __func__);
	disable_irq(client->irq);

	return rpr521_set_enable(client, PS_ALS_SET_MODE_CONTROL);
}

static int rpr521_resume(struct i2c_client *client)
{
	pr_info("%s\n", __func__);

	enable_irq(client->irq);

	return rpr521_set_enable(client, PS_ALS_SET_MODE_CONTROL|PS_EN);
}
*/

static int rpr521_suspend(struct device *dev)
{
	struct ALS_PS_DATA *als_ps;
	als_ps = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);
	if (als_ps->ps_enable_flag == 0)
		disable_irq(als_ps->client->irq);
	else
		enable_irq_wake(als_ps->client->irq);

	#ifdef DISABLE_LSENSOR_WHEN_PSENSOR_ENABLED
	/*
	* Save sensor state and disable them,
	* this is to ensure internal state flags are set correctly.
	*/
	als_ps->als_enable_state = als_ps->als_enable_flag;

	if (als_ps->als_enable_state) {
		if (rpr521_enable_als_sensor(als_ps->client, 0))
			dev_err(&als_ps->client->dev,
				"Disable light sensor fail!\n");
	}
	#else
	#endif

	return 0;
}

static int rpr521_resume(struct device *dev)
{
	struct ALS_PS_DATA *als_ps;
	als_ps = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);
	if (als_ps->ps_enable_flag == 0)
		enable_irq(als_ps->client->irq);

	#ifdef DISABLE_LSENSOR_WHEN_PSENSOR_ENABLED
	/* Resume L sensor state as P sensor does not disable */
	if (als_ps->als_enable_state) {
		if (rpr521_enable_als_sensor(als_ps->client, 1))
			dev_err(&als_ps->client->dev,
				"enable light sensor fail!\n");
	}
	#else
	#endif

	return 0;
}

MODULE_DEVICE_TABLE(i2c, rpr521_id);

static const struct i2c_device_id rpr521_id[] = {
	{ "rpr521", 0 },
	{ }
};
#ifdef CONFIG_OF
static struct of_device_id rpr_match_table[] = {
					{ .compatible = "rohm,rpr0521c",},
					{ },
				};
#else
#define rpr_match_table NULL
#endif

static const struct dev_pm_ops rpr521_pm_ops = {
	.suspend	= rpr521_suspend,
	.resume		= rpr521_resume,
};

static struct i2c_driver rpr521_driver = {
	.driver = {
		.name	= RPR521_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = rpr_match_table,
		.pm = &rpr521_pm_ops,
	},
	.probe	= rpr521_probe,
	.remove	= rpr521_remove,
	.id_table = rpr521_id,
};

static int __init rpr521_init(void)
{
	return i2c_add_driver(&rpr521_driver);
}

static void __exit rpr521_exit(void)
{
	i2c_del_driver(&rpr521_driver);
}

MODULE_AUTHOR("Andy Mi @ ROHM");
MODULE_DESCRIPTION("rpr521 ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(rpr521_init);
module_exit(rpr521_exit);
