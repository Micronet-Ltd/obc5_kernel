/* drivers/i2c/chips/rpr0400_driver.c - ROHM RPR0400 Linux kernel driver
 *
 * Copyright (C) 2012 
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
 *  2012-8-14:	Ver. 1.1	Added calibration and set thresholds methods. Besides, the thresholds are automatically changed if a ps int is triggered to avoid constant interrupts.
 */
#include <linux/debugfs.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/i2c/rpr0400_driver.h>

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */

#include <linux/sensors.h>

#define PS_ALS_SET_MODE_CONTROL   (NORMAL_MODE)
#define PS_ALS_SET_ALSPS_CONTROL  (LEDCURRENT_100MA | ALSGAIN_X2X2)	//Set high gain value to acquire high accuracy
#define PS_ALS_SET_INTR_PERSIST   (1)
#define PS_ALS_SET_INTR           (PS_THH_BOTH_OUTSIDE| POLA_ACTIVEL | OUTPUT_LATCH | MODE_PROXIMITY)
#define PS_ALS_SET_PS_TH          (150)	//Customer should change the threshold value according to their mechanical design and measured data
#define PS_ALS_SET_PS_TL          (100)	//Changed from (0x000)
#define PS_ALS_SET_ALS_TH         (2000) 	//Compare with ALS_DATA0. ALS_Data equals 0.192*ALS_DATA0 roughly. Usually not used.
#define PS_ALS_SET_ALS_TL         (0x0000)	//Usually not used.
#define PS_ALS_SET_MIN_DELAY_TIME (100)	//Andy Mi: Changed from 125 to 100. I have no idea why it is 125 previously. 


/*************** Global Data ******************/
/* parameter for als calculation */
#define COEFFICIENT               (4)
const unsigned long data0_coefficient[COEFFICIENT] = {192, 141, 127, 117};
const unsigned long data1_coefficient[COEFFICIENT] = {316, 108,  86,  74};
//const unsigned long judge_coefficient[COEFFICIENT] = { 29,  65,  85, 199};
const unsigned long judge_coefficient[COEFFICIENT] = { 29,  65,  85, 158}; //grace modified in 2013.3.6

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
    {100,   0},   /*  5 */
    {100, 100},   /*  6 */
    {100, 400},   /*  7 */
    {400,   0},   /*  8 */
    {400, 100},   /*  9 */
    {400,   0},   /* 10 */
    {400, 400},   /* 11 */
    {  0,   0},   /* 12 */
    {  0,   0},   /* 13 */
    {  0,   0},   /* 14 */
    {  0,   0}    /* 15 */
};

/* gain table */
#define GAIN_FACTOR (16)
static const struct GAIN_TABLE {
    char DATA0;
    char DATA1;
} gain_table[GAIN_FACTOR] = {
    {  1,   1},   /*  0 */
    {  2,   1},   /*  1 */
    {  0,   0},   /*  2 */
    {  0,   0},   /*  3 */
    {  0,   0},   /*  4 */
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

/* add by shengweiguang begin */
struct ALS_PS_DATA {
	struct i2c_client *client;
	struct mutex update_lock;
	struct delayed_work	dwork;	/* for PS interrupt */
	struct delayed_work    als_dwork; /* for ALS polling */
	struct input_dev *input_dev_als;
	struct input_dev *input_dev_ps;

	unsigned int enable;	//used to indicate working mode
	unsigned int als_time;	//als measurement time
	unsigned int ps_time;	//ps measurement time
	unsigned int ps_th_l;	//ps threshold low
	unsigned int ps_th_h;	//ps threshold high
	unsigned int ps_th_l_back; //ps threshold low backup
	unsigned int ps_th_h_back; //ps threshold high backup
	unsigned int als_th_l;	//als threshold low, not used in the program
	unsigned int als_th_h;	//als threshold high, not used in the program
	unsigned int persistence;	//persistence
	unsigned int control;	//als_ps_control

	/* register value */
	unsigned short als_data0_raw;	//register value of data0
	unsigned short als_data1_raw;	//register value of data1
	unsigned short ps_data_raw;	//register value of ps

	/* control flag from HAL */
	unsigned int enable_ps_sensor;
	unsigned int enable_als_sensor;

	/* PS parameters */

	unsigned int ps_direction;		/* 0 = near-to-far; 1 = far-to-near */
	unsigned int ps_data;			/* to store PS data */
	float ps_distance;
	unsigned int ledcurrent;	//led current

	/* ALS parameters */
	unsigned int als_data;			/* to store ALS data */
	unsigned int als_level;
	unsigned int gain0;	//als data0 gain
	unsigned int gain1;	//als data1 gain
	unsigned int als_poll_delay;	// the unit is ms I think. needed for als polling

	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;
};




static struct sensors_classdev sensors_light_cdev = {
	.name = "rpr0410-light",
	.vendor = "rohm",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "60000",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "rpr0410-proximity",
	.vendor = "rohm",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_get_crosstalk =NULL,
};



/* add by shengweiguang end */

/*************** Functions ******************/
/******************************************************************************
 * NAME       : rpr400_set_enable
 * FUNCTION   : set measurement time according to enable
 * REMARKS    : this function will overwrite the work mode. if it is called improperly, 
 *			   you may shutdown some part unexpectedly. please check als_ps->enable first.
 *			   I assume it is run in normal mode. If you want low noise mode, the code should be modified.
 *****************************************************************************/
static int rpr400_set_enable(struct i2c_client *client, int enable)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;

	if(enable > 0x0B)
	{
		printk(KERN_ERR "%s: invalid measurement time setting.\n", __func__);
		return -EINVAL;
	}
	else
	{
		mutex_lock(&als_ps->update_lock);
		ret = i2c_smbus_write_byte_data(client, REG_MODECONTROL, enable);
		mutex_unlock(&als_ps->update_lock);

		als_ps->enable = enable;
		als_ps->als_time = mode_table[(enable & 0xF)].ALS;
		als_ps->ps_time = mode_table[(enable & 0xF)].PS;

		return ret;
	}
}

static int rpr400_set_ps_threshold_low(struct i2c_client *client, int threshold)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	unsigned short write_data;

    /* check whether the parameter is valid */
	if(threshold > REG_PSTL_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	if(threshold > als_ps->ps_th_h)
	{
		printk(KERN_ERR "%s: higher than threshold high.\n", __func__);
		return -EINVAL;
	}
	
    /* write register to RPR400 via i2c */
	write_data = CONVERT_TO_BE(threshold);
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_i2c_block_data(client, REG_PSTL, sizeof(write_data), (unsigned char *)&write_data);
	mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->ps_th_l = threshold;	//Update the value after successful i2c write to avoid difference. 
		
	return 0;
}

static int rpr400_set_ps_threshold_high(struct i2c_client *client, int threshold)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	unsigned short write_data;

    /* check whether the parameter is valid */
	if(threshold > REG_PSTH_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	if(threshold < als_ps->ps_th_l)
	{
		printk(KERN_ERR "%s: lower than threshold low.\n", __func__);
		return -EINVAL;
	}
	
    /* write register to RPR400 via i2c */
	write_data = CONVERT_TO_BE(threshold);
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_i2c_block_data(client, REG_PSTH, sizeof(write_data), (unsigned char *)&write_data);
	mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->ps_th_h = threshold;	//Update the value after successful i2c write to avoid difference. 
		
	return 0;
}

static int rpr400_calibrate(struct i2c_client *client)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int average;
 	unsigned int i, tmp, ps_th_h, ps_th_l;	

	average = 0;

	rpr400_set_enable(client, 0x01);	//PS 10ms

	for(i = 0; i < 20; i ++)
	{
		tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
		if(tmp < 0)
		{
			printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
			goto err_exit;
		}
		average += tmp & 0xFFF;	// 12 bit data
	}
	average /= 20;

//	ps_th_h = average + PS_ALS_SET_PS_TH;
//	ps_th_l = average + PS_ALS_SET_PS_TL;
	ps_th_h = average + THRES_TOLERANCE + THRES_DEFAULT_DIFF;
	ps_th_l = average + THRES_TOLERANCE;

	if(ps_th_h < 0)
	{
		printk(KERN_ERR "%s: high threshold is less than 0.\n", __func__);
		goto err_exit;
	}
	if(ps_th_h > REG_PSTH_MAX)
	{
		printk(KERN_ERR "%s: high threshold is greater than maximum allowed value.\n", __func__);
		goto err_exit;
	}
	if(ps_th_l < 0)
	{
		printk(KERN_ERR "%s: low threshold is less than 0.\n", __func__);
		goto err_exit;
	}
	if(ps_th_l > REG_PSTL_MAX)
	{
		printk(KERN_ERR "%s: low threshold is greater than maximum allowed value.\n", __func__);
		goto err_exit;
	}

	if(!(rpr400_set_ps_threshold_high(client, ps_th_h)))
		als_ps->ps_th_h_back = ps_th_h;
	else 
		goto err_exit;
	if(!(rpr400_set_ps_threshold_low(client, ps_th_l)))
		als_ps->ps_th_l_back = ps_th_l;
	else
		goto err_exit;

	rpr400_set_enable(client, 0);	//disable ps
	return 0;		

err_exit:
	rpr400_set_enable(client, 0);	//disable ps
	return -1;
	
}

#if _FUNCTION_USED_	//masked because they are claimed but not used, which may cause error when compilling if the warning level is high enough. These functions provides some methods.
static int rpr400_set_persist(struct i2c_client *client, int persist)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	
    /* check whether the parameter is valid */
	if(persist > PERSISTENCE_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	
    /* write register to RPR400 via i2c */
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_byte_data(client, REG_PERSISTENCE, persist);
	mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->persistence = persist;	//Update the value after successful i2c write to avoid difference. 

	return 0;
}

static int rpr400_set_control(struct i2c_client *client, int control)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	unsigned char gain, led_current;
	
	if(control > REG_ALSPSCTL_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	
	gain = (control & 0x3C) >> 2;	//gain setting values
	led_current = control & 0x03;		//led current setting value

	if(!((gain == ALSGAIN_X1X1) || (gain == ALSGAIN_X1X2) || (gain == ALSGAIN_X2X2) || (gain == ALSGAIN_X64X64)
		|| (gain == ALSGAIN_X128X64) || (gain == ALSGAIN_X128X128)))
	{
		printk(KERN_ERR "%s: invalid gain setting. \n", __func__);
		return -EINVAL;
	}
	
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_byte_data(client, REG_ALSPSCONTROL, control);
	mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->control = control;
	als_ps->gain0 = gain_table[gain].DATA0;
	als_ps->gain1 = gain_table[gain].DATA1;
	als_ps->ledcurrent = led_current;

	return ret;
}
#endif


#if 1
/******************************************************************************
 * NAME       : long_long_divider
 * FUNCTION   : calc divider of unsigned long long int or unsgined long
 * REMARKS    :
 *****************************************************************************/
static void long_long_divider(unsigned long long data, unsigned long base_divier, unsigned long *answer, unsigned long long *overplus)
{
    volatile unsigned long long divier;
    volatile unsigned long      unit_sft;

    if ((long long)data < 0)	// . If data MSB is 1, it may go to endless loop. 
    	{
    	data /= 2;	//0xFFFFFFFFFFFFFFFF / 2 = 0x7FFFFFFFFFFFFFFF
	base_divier /= 2;
    	}
    divier = base_divier;
    if (data > MASK_LONG) {
        unit_sft = 0;
        while (data > divier) {
            unit_sft++;
            divier = divier << 1;
        }
        while (data > base_divier) {
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
        *overplus = (unsigned long long)(data - (*answer * base_divier));
    }
}

/******************************************************************************
 * NAME       : calc_rohm_als_data
 * FUNCTION   : calculate illuminance data for RPR400
 * REMARKS    : final_data is 1000 times, which is defined as CUT_UNIT, of the actual lux value
 *****************************************************************************/
static int calc_rohm_als_data(unsigned short data0, unsigned short data1, unsigned char gain0, unsigned char gain1, unsigned char time)
{
#define DECIMAL_BIT      (15)
#define JUDGE_FIXED_COEF (100)
#define MAX_OUTRANGE     (11357)
#define MAXRANGE_NMODE   (0xFFFF)
#define MAXSET_CASE      (4)

	int                final_data;
	CALC_DATA          calc_data;
	CALC_ANS           calc_ans;
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
	if (calc_data.gain_data0 == 0) 
	{
		/* issue error value when gain is 0 */
		return (CALC_ERROR);
	}
	else
	{
		max_range = MAX_OUTRANGE / calc_data.gain_data0;
	}
	
	/* calculate data */
	if (calc_data.als_data0 == MAXRANGE_NMODE) 
	{
		calc_ans.positive = max_range;
		calc_ans.decimal  = 0;
	} 
	else 
	{
		/* get the value which is measured from power table */
		calc_data.als_time = time;
		if (calc_data.als_time == 0) 
		{
			/* issue error value when time is 0 */
			return (CALC_ERROR);
		}

		calc_judge = calc_data.als_data1 * JUDGE_FIXED_COEF;
		if (calc_judge < (calc_data.als_data0 * judge_coefficient[0])) 
		{
			set_case = 0;
		} 
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[1]))
		{
			set_case = 1;
		} 
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[2])) 
		{
			set_case = 2;
		}
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[3])) 
		{
			 set_case = 3;
		} 
		else
		{
			set_case = MAXSET_CASE;
		}
		calc_ans.positive = 0;
		if (set_case >= MAXSET_CASE) 
		{
			calc_ans.decimal = 0;	//which means that lux output is 0
		}
		else
		{
			calc_data.gain_data1 = gain1;
			if (calc_data.gain_data1 == 0) 
			{
				/* issue error value when gain is 0 */
				return (CALC_ERROR);
			}
			calc_data.data0      = (unsigned long long )(data0_coefficient[set_case] * calc_data.als_data0) * calc_data.gain_data1;
			calc_data.data1      = (unsigned long long )(data1_coefficient[set_case] * calc_data.als_data1) * calc_data.gain_data0;
			if(calc_data.data0 < calc_data.data1)	//In this case, data will be less than 0. As data is unsigned long long, it will become extremely big.
			{
				return (CALC_ERROR);
			}
			else
			{
				calc_data.data       = (calc_data.data0 - calc_data.data1);
			}
			calc_data.dev_unit   = calc_data.gain_data0 * calc_data.gain_data1 * calc_data.als_time * 10;	//24 bit at max (128 * 128 * 100 * 10)
			if (calc_data.dev_unit == 0) 
			{
				/* issue error value when dev_unit is 0 */
				return (CALC_ERROR);
			}

			/* calculate a positive number */
			div_answer   = 0;
			div_overplus = 0;
			long_long_divider(calc_data.data, calc_data.dev_unit, &div_answer, &div_overplus);
			calc_ans.positive = div_answer;
			/* calculate a decimal number */
			calc_ans.decimal = 0;
			overplus         = div_overplus;
			if (calc_ans.positive < max_range)
			{
				if (overplus != 0)
				{
					overplus     = overplus << DECIMAL_BIT;
					div_answer   = 0;
					div_overplus = 0;
					long_long_divider(overplus, calc_data.dev_unit, &div_answer, &div_overplus);
					calc_ans.decimal = div_answer;
				}
			}

			else
			{
				calc_ans.positive = max_range;
			}
		}
	}
	
	final_data = calc_ans.positive * CUT_UNIT + ((calc_ans.decimal * CUT_UNIT) >> DECIMAL_BIT);
					
	return (final_data);

#undef DECIMAL_BIT
#undef JUDGE_FIXED_COEF
#undef MAX_OUTRANGE
#undef MAXRANGE_NMODE
#undef MAXSET_CASE
}

#else


/******************************************************************************
 * NAME       : long_long_divider
 * FUNCTION   : calc divider of unsigned long long int or unsgined long
 * REMARKS    :
 *****************************************************************************/
static int long_long_divider(long long data, unsigned long base_divier, unsigned long *answer, unsigned long long *overplus)
{
    volatile long long divier;
    volatile long      unit_sft;

    if ((data < 0) || (base_divier == 0)) {
        *answer   = 0;
        *overplus = 0;
        return (CALC_ERROR);
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
        *overplus = (unsigned long long)(data - (*answer * base_divier));
    }

    return (0);
}


/******************************************************************************
 * NAME       : calculate_als_data
 * FUNCTION   : calculate illuminance data for RPR400
 * REMARKS    : final_data format
 *            :+-------------------------+------------------------+
 *            :| positive values : 17bit | decimal values : 15bit |
 *            :+-------------------------+------------------------+
 *****************************************************************************/
static int calc_rohm_als_data(unsigned short data0, unsigned short data1, unsigned char gain0, unsigned char gain1, unsigned char time)
{
#define DECIMAL_BIT      (15)
#define JUDGE_FIXED_COEF (100)
#define MAX_OUTRANGE     (11357)
#define MAXRANGE_NMODE   (0xFFFF)
#define MAXRANGE_50MODE  (0x7FFF)
#define MAXSET_CASE      (4)

    int                result;
    int                final_data;
    CALC_DATA          calc_data;
    CALC_ANS           calc_ans;
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
        return (CALC_ERROR);
    } else {
        max_range = MAX_OUTRANGE / calc_data.gain_data0;
    }
    /* calculate data */
    if (calc_data.als_data0 == MAXRANGE_NMODE) {
        calc_ans.positive = max_range;
        calc_ans.decimal  = 0;
    } else {
            /* get the value which is measured from power table */
            calc_data.als_time = time;

            calc_judge = calc_data.als_data1 * JUDGE_FIXED_COEF;
            if (calc_judge < (calc_data.als_data0 * judge_coefficient[0])) {
                set_case = 0;
            } else if (calc_judge < (calc_data.als_data0 * judge_coefficient[1])) {
                set_case = 1;
            } else if (calc_judge < (calc_data.als_data0 * judge_coefficient[2])) {
                set_case = 2;
            } else if (calc_judge < (calc_data.als_data0 * judge_coefficient[3])) {
                set_case = 3;
            } else {
                set_case = MAXSET_CASE;
            }
            calc_ans.positive = 0;
            if (set_case >= MAXSET_CASE) {
                calc_ans.decimal = 0;
            } else {
                calc_data.gain_data1 = gain1;
                calc_data.data0      = (long long )(data0_coefficient[set_case] * calc_data.als_data0) * calc_data.gain_data1;
                calc_data.data1      = (long long )(data1_coefficient[set_case] * calc_data.als_data1) * calc_data.gain_data0;
                calc_data.data       = (calc_data.data0 - calc_data.data1);
                calc_data.dev_unit   = calc_data.gain_data0 * calc_data.gain_data1 * calc_data.als_time * 10;
                /* calculate a positive number */
                div_answer   = 0;
                div_overplus = 0;
                result = long_long_divider(calc_data.data, calc_data.dev_unit, &div_answer, &div_overplus);
                if (result == CALC_ERROR) {
                    return (result);
                }
                calc_ans.positive = div_answer;
                /* calculate a decimal number */
                calc_ans.decimal = 0;
                overplus         = div_overplus;
                if (calc_ans.positive < max_range) {
                    if (overplus != 0) {
                        overplus     = overplus << DECIMAL_BIT;
                        div_answer   = 0;
                        div_overplus = 0;
                        result = long_long_divider(overplus, calc_data.dev_unit, &div_answer, &div_overplus);
                        if (result == CALC_ERROR) {
                            return (result);
                        }
                        calc_ans.decimal = div_answer;
                    }
                } else {
                    calc_ans.positive = max_range;
                }
            }
    }
    //final_data = (int)(calc_ans.positive << DECIMAL_BIT) + calc_ans.decimal;
    final_data = calc_ans.positive * CUT_UNIT + ((calc_ans.decimal * CUT_UNIT) >> DECIMAL_BIT) ;

    return (final_data);
	
#undef DECIMAL_BIT
#undef JUDGE_FIXED_COEF
#undef MAX_OUTRANGE
#undef MAXRANGE_NMODE
#undef MAXRANGE_50MODE
#undef MAXSET_CASE
}

#endif
/******************************************************************************
 * NAME       : calculate_ps_data
 * FUNCTION   : calculate proximity data for RPR400
 * REMARKS    : 12 bit output
 *****************************************************************************/
static int calc_rohm_ps_data(unsigned short ps_reg_data)
{
    return (ps_reg_data & 0xFFF);
}

static unsigned int rpr400_als_data_to_level(unsigned int als_data)
{
#if 0
#define ALS_LEVEL_NUM 15
	int als_level[ALS_LEVEL_NUM]  = { 0, 50, 100, 150,  200,  250,  300, 350, 400,  450,  550, 650, 750, 900, 1100};
	int als_value[ALS_LEVEL_NUM]  = { 0, 50, 100, 150,  200,  250,  300, 350, 400,  450,  550, 650, 750, 900, 1100};
    	unsigned char idx;

	for(idx = 0; idx < ALS_LEVEL_NUM; idx ++)
	{
		if(als_data < als_value[idx])
		{
			break;
		}
	}
	if(idx >= ALS_LEVEL_NUM)
	{
		printk(KERN_ERR "RPR400 als data to level: exceed range.\n");
		idx = ALS_LEVEL_NUM - 1;
	}
	
	return als_level[idx];
#undef ALS_LEVEL_NUM
#else
	return als_data;
#endif
}

static void rpr400_reschedule_work(struct ALS_PS_DATA *als_ps,
					  unsigned long delay)
{
	unsigned long flags;

	spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags);

	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	//__cancel_delayed_work(&als_ps->dwork);
	cancel_delayed_work(&als_ps->dwork); // modify by shengweiguang for schedule bug
	schedule_delayed_work(&als_ps->dwork, delay);

	spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);
}

/* ALS polling routine */
static void rpr400_als_polling_work_handler(struct work_struct *work)
{
	struct ALS_PS_DATA *als_ps = container_of(work, struct ALS_PS_DATA, als_dwork.work);
	struct i2c_client *client=als_ps->client;
	int tmp = 0;
	
	schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// restart timer
	
	tmp = i2c_smbus_read_word_data(client, REG_ALSDATA0);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read data0 fail. \n", __func__);
		//return tmp;
	}
	als_ps->als_data0_raw = (unsigned short)tmp;
	tmp = i2c_smbus_read_word_data(client, REG_ALSDATA1);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read data1 fail. \n", __func__);
		//return tmp;
	}
	als_ps->als_data1_raw = (unsigned short)tmp;

// Theorically it is not necesssary to do so, but I just want to avoid any potential error.  -- Andy 2012.6.6
	tmp = i2c_smbus_read_byte_data(client, REG_ALSPSCONTROL);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read gain fail. \n", __func__);
		//return tmp;
	}
	tmp = (tmp & 0x3C) >> 2;
	als_ps->gain0 = gain_table[tmp].DATA0;
	als_ps->gain1 = gain_table[tmp].DATA1;	
	
	tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read time fail. \n", __func__);
		//return tmp;
	}
	tmp = tmp & 0xF;
	als_ps->als_time = mode_table[tmp].ALS;	
	
	als_ps->als_data = calc_rohm_als_data(als_ps->als_data0_raw, als_ps->als_data1_raw, als_ps->gain0, als_ps->gain1, als_ps->als_time);	
	if(als_ps->als_data == 0)
		als_ps->als_data ++;

	als_ps->als_level = rpr400_als_data_to_level(als_ps->als_data);

	printk(KERN_INFO "RPR400 als report: data0 = %d, data1 = %d, gain0 = %d, gain1 = %d, time = %d, lux = %d, level = %d.\n", als_ps->als_data0_raw, 
		als_ps->als_data1_raw, als_ps->gain0, als_ps->gain1, als_ps->als_time, als_ps->als_data, als_ps->als_level);
	if(als_ps->als_data != CALC_ERROR)
	{
		input_report_abs(als_ps->input_dev_als, ABS_MISC, als_ps->als_level); // report als data. maybe necessary to convert to lux level
		input_sync(als_ps->input_dev_als);	
	}	
}


/* PS interrupt routine */
static void rpr400_ps_int_work_handler(struct work_struct *work)
{
	struct ALS_PS_DATA *als_ps = container_of((struct delayed_work *)work, struct ALS_PS_DATA, dwork);
	struct i2c_client *client=als_ps->client;
	int tmp;

	tmp =  i2c_smbus_read_byte_data(client, REG_INTERRUPT);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read interrupt status fail. \n", __func__);
		return;
	}
	if(tmp & PS_INT_MASK)	//Interrupt is caused by PS
	{
		tmp = i2c_smbus_read_byte_data(client, REG_ALSPSCONTROL);
		if(tmp < 0)
		{
			printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
			return;
		}
		als_ps->ledcurrent = tmp & 0x3;
		
		tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
		if(tmp < 0)
		{
			printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
			return;
		}
		als_ps->ps_data_raw = (unsigned short)tmp;

		als_ps->ps_data = calc_rohm_ps_data(als_ps->ps_data_raw);

		if(als_ps->ps_data > als_ps->ps_th_h)
		{		
			als_ps->ps_direction = 0;
#ifdef _AUTO_THRESHOLD_CHANGE_
			rpr400_set_ps_threshold_high(client, REG_PSTH_MAX);
			rpr400_set_ps_threshold_low(client, als_ps->ps_th_l_back);
#endif
		}
		else if(als_ps->ps_data < als_ps->ps_th_l)
		{
			als_ps->ps_direction = 1;
#ifdef _AUTO_THRESHOLD_CHANGE_
			rpr400_set_ps_threshold_high(client, als_ps->ps_th_h_back);
			rpr400_set_ps_threshold_low(client, 0);
#endif
		}
/* delete by shengweiguang
		printk(KERN_INFO "RPR400 ps report: raw_data = %d, data = %d, direction = %d. \n", 
				als_ps->ps_data_raw, als_ps->ps_data, als_ps->ps_direction);
*/
		input_report_abs(als_ps->input_dev_ps, ABS_DISTANCE, als_ps->ps_direction); 
		input_sync(als_ps->input_dev_ps);	
	}
	else
	{
		printk(KERN_ERR "%s: unknown interrupt source.\n", __func__);
	}
	
	enable_irq(client->irq);
}

/* assume this is ISR */
static irqreturn_t rpr400_interrupt(int vec, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	disable_irq_nosync(client->irq);
	//printk("%s\n", __func__);
	rpr400_reschedule_work(als_ps, 0);

	return IRQ_HANDLED;
}

/*************** SysFS Support ******************/
static ssize_t rpr400_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", als_ps->enable_ps_sensor);
}

static ssize_t rpr400_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
// 	unsigned long flags;

	printk(KERN_INFO "RPR400 enable PS sensor -> %ld \n", val);
		
	if ((val != 0) && (val != 1)) 
	{
		printk("%s:store unvalid value=%ld\n", __func__, val);
		return count;
	}
	
	if(val == 1) 
	{
		//turn on p sensor
		wake_lock(&ps_lock);
		if (als_ps->enable_ps_sensor == 0) 
		{
			als_ps->enable_ps_sensor = 1;
		
			if (als_ps->enable_als_sensor == 0) 
			{
				rpr400_set_enable(client, PS100MS);	//PS on and ALS off
			}
			else
			{
				rpr400_set_enable(client, BOTH100MS);	//PS on and ALS on
			}
		}
	} 
	else 
	{
		if(als_ps->enable_ps_sensor == 1)
		{
			als_ps->enable_ps_sensor = 0;
			if (als_ps->enable_als_sensor == 0) 
			{
				rpr400_set_enable(client, BOTH_STANDBY);	//PS off and ALS off
			}
			else 
			{
				rpr400_set_enable(client, ALS100MS);	//PS off and ALS on
			}
			wake_unlock(&ps_lock);
		}
	}
		
	return count;
}

/* add by shengweiguang begin */
static int rpr0410_enable_als_sensor(struct i2c_client *client, int val)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
 	unsigned long flags;
	
	printk(KERN_INFO "RPR400 enable ALS sensor -> %d\n", val);

	if ((val != 0) && (val != 1))
	{
		pr_err("%s: invalid value (val = %d)\n", __func__, val);
		return -EINVAL;
	}
	
	if(val == 1)
	{
		//turn on light  sensor
		if (als_ps->enable_als_sensor==0)
		{
			als_ps->enable_als_sensor = 1;
			
			if(als_ps->enable_ps_sensor == 1)
			{
				rpr400_set_enable(client, BOTH100MS);
			}
			else
			{
				rpr400_set_enable(client, ALS100MS);
			}
		}
		
		spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
		//__cancel_delayed_work(&als_ps->als_dwork);
		cancel_delayed_work(&als_ps->als_dwork);
		schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// 125ms
		spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	}
	else
	{
		if(als_ps->enable_als_sensor == 1)
		{
			als_ps->enable_als_sensor = 0;

			if(als_ps->enable_ps_sensor == 1)
			{
				rpr400_set_enable(client, PS100MS);
			}
			else
			{
				rpr400_set_enable(client, BOTH_STANDBY);
			}
		}
		
		spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
		//__cancel_delayed_work(&als_ps->als_dwork);
		cancel_delayed_work(&als_ps->als_dwork);
		spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	}
	
	return 0;
}

static int rpr0410_set_als_poll_delay(struct i2c_client *client,
		unsigned int val)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);

 	unsigned long flags;
	
	if (val < PS_ALS_SET_MIN_DELAY_TIME * 1000000)
		val = PS_ALS_SET_MIN_DELAY_TIME * 1000000;	
	
	als_ps->als_poll_delay = val /1000000;	// convert us => ms
	
	if (als_ps->enable_als_sensor == 1)//grace modified in 2013.10.09
	{
	
	/* we need this polling timer routine for sunlight canellation */
	spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
		
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	//__cancel_delayed_work(&als_ps->als_dwork);
	cancel_delayed_work(&als_ps->als_dwork);
	schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// 125ms
			
	spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	
	}
	return 0;
}

static int rpr0410_enable_ps_sensor(struct i2c_client *client,
		 int val)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);

	printk(KERN_INFO "RPR400 enable PS sensor -> %d \n", val);
		
	if ((val != 0) && (val != 1)) 
	{
		pr_err("%s: invalid value (val = %d)\n", __func__, val);
		return -EINVAL;
	}
	
	if(val == 1) 
	{
		//turn on p sensor
		wake_lock(&ps_lock);
		if (als_ps->enable_ps_sensor == 0) 
		{
			als_ps->enable_ps_sensor = 1;
		
			if (als_ps->enable_als_sensor == 0) 
			{
				rpr400_set_enable(client, PS100MS);	//PS on and ALS off
			}
			else
			{
				rpr400_set_enable(client, BOTH100MS);	//PS on and ALS on
			}
		}
	} 
	else 
	{
		if(als_ps->enable_ps_sensor == 1)
		{
			als_ps->enable_ps_sensor = 0;
			if (als_ps->enable_als_sensor == 0) 
			{
				rpr400_set_enable(client, BOTH_STANDBY);	//PS off and ALS off
			}
			else 
			{
				rpr400_set_enable(client, ALS100MS);	//PS off and ALS on
			}
			wake_unlock(&ps_lock);
		}
	}
		
	return 0;
}

/* add by shengweiguang end */

static ssize_t rpr400_show_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", als_ps->enable_als_sensor);
}

static ssize_t rpr400_store_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
 	unsigned long flags;
	
	printk(KERN_INFO "RPR400 enable ALS sensor -> %ld\n", val);

	if ((val != 0) && (val != 1))
	{
		printk("%s: enable als sensor=%ld\n", __func__, val);
		return count;
	}
	
	if(val == 1)
	{
		//turn on light  sensor
		if (als_ps->enable_als_sensor==0)
		{
			als_ps->enable_als_sensor = 1;
			
			if(als_ps->enable_ps_sensor == 1)
			{
				rpr400_set_enable(client, BOTH100MS);
			}
			else
			{
				rpr400_set_enable(client, ALS100MS);
			}
		}
		
		spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
		//__cancel_delayed_work(&als_ps->als_dwork);
		cancel_delayed_work(&als_ps->als_dwork);
		schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// 125ms
		spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	}
	else
	{
		if(als_ps->enable_als_sensor == 1)
		{
			als_ps->enable_als_sensor = 0;

			if(als_ps->enable_ps_sensor == 1)
			{
				rpr400_set_enable(client, PS100MS);
			}
			else
			{
				rpr400_set_enable(client, BOTH_STANDBY);
			}
		}
		
		spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
		//__cancel_delayed_work(&als_ps->als_dwork);
		cancel_delayed_work(&als_ps->als_dwork);
		spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	}
	
	return count;
}

static ssize_t rpr400_show_als_poll_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", als_ps->als_poll_delay*1000);	// return in micro-second
}

static ssize_t rpr400_store_als_poll_delay(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
//	int ret;
//	int poll_delay = 0;
 	unsigned long flags;
	
	if (val < PS_ALS_SET_MIN_DELAY_TIME * 1000)
		val = PS_ALS_SET_MIN_DELAY_TIME * 1000;	
	
	als_ps->als_poll_delay = val /1000;	// convert us => ms
	
	if (als_ps->enable_als_sensor == 1)//grace modified in 2013.10.09
	{
	
	/* we need this polling timer routine for sunlight canellation */
	spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
		
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	//__cancel_delayed_work(&als_ps->als_dwork);
	cancel_delayed_work(&als_ps->als_dwork);
	schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// 125ms
			
	spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	
	}
	return count;
}

//grace add in 2013.10.09 begin
static ssize_t rpr400_show_als_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int tmp;
	int tmp1;
	tmp = i2c_smbus_read_word_data(client, REG_ALSDATA0);
	tmp1 = i2c_smbus_read_word_data(client, REG_ALSDATA1);
	tmp = tmp << 8;
	tmp =tmp | tmp1;
	return sprintf(buf, "%d\n", tmp);
}

static ssize_t rpr400_show_ps_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int tmp = 0;
	tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
	return sprintf(buf, "%d\n", tmp);
}
//grace add in 2013.10.09 end

static ssize_t rpr400_show_ps_thres_high(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	//struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	//grace test in 2013.10.09 begin
	int ps_data = 0, ps_high = 0, ps_low = 0;
	ps_data = i2c_smbus_read_word_data(client, REG_PSDATA);
	if(ps_data < 0)
	{
			printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
			return -1;
	}
	
	ps_high = i2c_smbus_read_word_data(client, REG_PSTH);
	if(ps_high < 0)
	{
			printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
			return -1;
	}
	
	ps_low = i2c_smbus_read_word_data(client, REG_PSTL);
	if(ps_low < 0)
	{
			printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
			return -1;
	}
	
	//return sprintf(buf, "%d\n", als_ps->ps_th_h);	
	return sprintf(buf, "%d %d %d\n", ps_data, ps_high, ps_low);
	//grace test in 2013.10.09 end
}

static ssize_t rpr400_store_ps_thres_high(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	if(!(rpr400_set_ps_threshold_high(client, val)))
		als_ps->ps_th_h_back = als_ps->ps_th_h;
	
	return count;
}

static ssize_t rpr400_show_ps_thres_low(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", als_ps->ps_th_l);	
}

static ssize_t rpr400_store_ps_thres_low(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	if(!(rpr400_set_ps_threshold_low(client, val)))
		als_ps->ps_th_l_back = als_ps->ps_th_l;
	
	return count;
}

static ssize_t rpr400_show_ps_calib(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\t%d\n", als_ps->ps_th_h, als_ps->ps_th_l);	
}

static ssize_t rpr400_store_ps_calib(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
#define SET_LOW_THRES	1
#define SET_HIGH_THRES	2
#define SET_BOTH_THRES	3

	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
 	unsigned int i, tmp, ps_th_h, ps_th_l;
	int average;	//This should be signed to avoid error.
	
	switch(val)
	{
		case SET_LOW_THRES:		
		//Take 20 average for noise. use noise + THRES_TOLERANCE as low threshold.
		//If high threshold is lower than the new low threshold + THRES_DIFF, make the high one equal low + THRES_DIFF
		//Please make sure that there is NO object above the sensor, otherwise it may cause the high threshold too high to trigger which make the LCD NEVER shutdown.
		//If the noise is too large, larger than 4065, it will return -1. If so, the mechanical design MUST be redo. It is quite unlikely. 
			average = 0;
			ps_th_h = als_ps->ps_th_h_back;
			ps_th_l = als_ps->ps_th_l_back;
			for(i = 0; i < 20; i ++)
			{
				tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
				if(tmp < 0)
				{
					printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
					return -1;
				}
				average += tmp & 0xFFF;	// 12 bit data
			}
			average /= 20;		//This is the average noise
			ps_th_l = average + THRES_TOLERANCE;	
			if(ps_th_l > REG_PSTL_MAX)
			{
				printk(KERN_ERR "%d in %s: low threshold is too high. \n", __LINE__, __func__);
				return -1;
			}
			if(ps_th_l < 0)
			{
				printk(KERN_ERR "%d in %s: low threshold is too low. \n", __LINE__, __func__);
				return -1;
			}
			if(ps_th_h < ps_th_l + THRES_DIFF)
			{
				ps_th_h = ps_th_l + THRES_DIFF;	//It will not be minus or an error should have occured earlier. 
				if(ps_th_h > REG_PSTH_MAX)
				{
					printk(KERN_ERR "%d in %s: high threshold is too high. \n", __LINE__, __func__);
					return -1;
				}
				if(!rpr400_set_ps_threshold_high(client, ps_th_h))
					als_ps->ps_th_h_back = ps_th_h;
				else
					return -1;
			}
			if(!rpr400_set_ps_threshold_low(client, ps_th_l))
				als_ps->ps_th_l_back = ps_th_l;
			else
				return -1;
			break;
		
		case SET_HIGH_THRES:	
		//Take 20 average for signal. use signal -THRES_TOLERANCE as high threshold. 
		//If low threshold is higher than the new high one - THRES_DIFF, make the low one equal high - THRES_DIFF
		//Please make sure that there IS AN object above the sensor, otherwise it will be a disaster. The high threshold will be too low which will cause the panel ALWAYS shutdown
		//Customer can use their own standard to set the test scenario. For example, a 18% grey card @ 2cm, or another example, a 90% white card 4cm, and etc. 
		//If the signal is too weak, less than 30, it will return -1. If so, the mechanical design MUST be redo. It shall not happen very frequently. 
			average = 0;
			ps_th_h = als_ps->ps_th_h_back;
			ps_th_l = als_ps->ps_th_l_back;
			for(i = 0; i < 20; i ++)
			{
				tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
				if(tmp < 0)
				{
					printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
					return -1;
				}
				average += tmp & 0xFFF;	// 12 bit data
			}
			average /= 20;		//This is the average signal
			ps_th_h = average - THRES_TOLERANCE;
			if(ps_th_h > REG_PSTH_MAX)
			{
				printk(KERN_ERR "%d in %s: high threshold is too high. \n", __LINE__, __func__);
				return -1;
			}
			if(ps_th_h < 0)
			{
				printk(KERN_ERR "%d in %s: high threshold is too low. \n", __LINE__, __func__);
				return -1;
			}
			if(ps_th_l > ps_th_h - THRES_DIFF)
			{
				ps_th_l = ps_th_h - THRES_DIFF;	//Given that REG_PSTH_MAX = REG_PSTL+MAX, it will not be greater than REG_PSTL_MAX or an error should have occured earlier.
				if(ps_th_l < 0)
				{
					printk(KERN_ERR "%d in %s: low threshold is too low. \n", __LINE__, __func__);
					return -1;
				}
				if(!rpr400_set_ps_threshold_low(client, ps_th_l))
					als_ps->ps_th_l_back = ps_th_l;
				else
					return -1;
			}
			if(!rpr400_set_ps_threshold_high(client, ps_th_h))
				als_ps->ps_th_h_back = ps_th_h;
			else
				return -1;
			break;
		
		case SET_BOTH_THRES:	//Take 20 average for noise. use noise + PS_ALS_SET_PS_TL as low threshold, noise + PS_ALS_SET_PS_TH as high threshold
			rpr400_calibrate(client);
			break;

		default:
			return -EINVAL;	//NOT supported!
	}
			
	return count;

#undef SET_BOTH_THRES
#undef SET_HIGH_THRES
#undef SET_LOW_THRES
}
/* add by shengweiguang begin */
static int rpr0410_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct ALS_PS_DATA *data = container_of(sensors_cdev,
			struct ALS_PS_DATA, als_cdev);

	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	return rpr0410_enable_als_sensor(data->client, enable);
}

static int  rpr0410_als_poll_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct ALS_PS_DATA *data = container_of(sensors_cdev,
			struct ALS_PS_DATA, als_cdev);
	rpr0410_set_als_poll_delay(data->client, delay_msec);
	return 0;
}

static int rpr0410_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
		struct ALS_PS_DATA *data = container_of(sensors_cdev,
			struct ALS_PS_DATA, ps_cdev);

	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	return rpr0410_enable_ps_sensor(data->client, enable);
}

static ssize_t rpr0410_get_ps_crosstalk(struct sensors_classdev *sensors_cdev,
		unsigned int prox_data)
{
	return 0;
}
/* add by shengweiguang end */

static DEVICE_ATTR(als_poll_delay,  S_IRUGO | S_IWUGO,
				    rpr400_show_als_poll_delay, rpr400_store_als_poll_delay);

static DEVICE_ATTR(enable_als_sensor,  S_IRUGO | S_IWUGO ,
				  rpr400_show_enable_als_sensor, rpr400_store_enable_als_sensor);

static DEVICE_ATTR(enable_ps_sensor,  S_IRUGO | S_IWUGO ,
				   rpr400_show_enable_ps_sensor, rpr400_store_enable_ps_sensor);

static DEVICE_ATTR(ps_thres_high,  S_IRUGO | S_IWUGO ,
				  rpr400_show_ps_thres_high, rpr400_store_ps_thres_high);

static DEVICE_ATTR(ps_thres_low,  S_IRUGO | S_IWUGO ,
				   rpr400_show_ps_thres_low, rpr400_store_ps_thres_low);

static DEVICE_ATTR(ps_calib,  S_IRUGO | S_IWUGO ,
				   rpr400_show_ps_calib, rpr400_store_ps_calib);
static DEVICE_ATTR(als_data, S_IRUGO | S_IWUGO, rpr400_show_als_data, NULL); //grace add in 2013.10.09
static DEVICE_ATTR(ps_data, S_IRUGO | S_IWUGO, rpr400_show_ps_data, NULL);   //grace add in 2013.10.09

static struct attribute *rpr400_attributes[] = {
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_enable_als_sensor.attr,
	&dev_attr_als_poll_delay.attr,
	&dev_attr_ps_thres_high.attr,
	&dev_attr_ps_thres_low.attr,
	&dev_attr_ps_calib.attr,
	&dev_attr_als_data.attr,  //grace add in 2013.10.09
	&dev_attr_ps_data.attr,   //grace add in 2013.10.09
	NULL
};

static const struct attribute_group rpr400_attr_group = {
	.attrs = rpr400_attributes,
};

/* add by shengweiguang begin */
static struct attribute *rpr0410_als_attributes[] = {
	&dev_attr_enable_als_sensor.attr,
	&dev_attr_als_poll_delay.attr,
	&dev_attr_als_data.attr,  //grace add in 2013.10.09
	NULL
};

static const struct attribute_group rpr0410_als_attr_group = {
	.attrs = rpr0410_als_attributes,
};

static struct attribute *rpr0410_ps_attributes[] = {
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_ps_thres_high.attr,
	&dev_attr_ps_thres_low.attr,
	&dev_attr_ps_calib.attr,
	&dev_attr_ps_data.attr,   //grace add in 2013.10.09
	NULL
};

static const struct attribute_group rpr0410_ps_attr_group = {
	.attrs = rpr0410_ps_attributes,
};

/* add by shengweiguang end */

/*************** Initialze Functions ******************/
static int rpr400_init_client(struct i2c_client *client)
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
    int result;
    unsigned char gain;

    unsigned char mode_ctl    = PS_ALS_SET_MODE_CONTROL;
    unsigned char psals_ctl   = PS_ALS_SET_ALSPS_CONTROL;
    unsigned char persist     = PS_ALS_SET_INTR_PERSIST;
    unsigned char intr        = PS_ALS_SET_INTR;
    unsigned short psth_upper  = PS_ALS_SET_PS_TH;
    unsigned short psth_low    = PS_ALS_SET_PS_TL;
    unsigned short alsth_upper = PS_ALS_SET_ALS_TH;
    unsigned short alsth_low   = PS_ALS_SET_ALS_TL;

    struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
    /* execute software reset */
    result =  i2c_smbus_write_byte_data(client, REG_SYSTEMCONTROL, 0xC0);	//soft-reset
    if (result != 0) {
        return (result);
    }

    /* not check parameters are psth_upper, psth_low, alsth_upper, alsth_low */
    /* check the PS orerating mode */
    if ((NORMAL_MODE != mode_ctl) && (LOW_NOISE_MODE != mode_ctl)) {
	printk(KERN_ERR "%s: invalid parameter.\n", __func__);
        return (-EINVAL);
    }
    /* check the parameter of ps and als control */
    if (psals_ctl > REG_ALSPSCTL_MAX) {
	printk(KERN_ERR "%s: invalid parameter.\n", __func__);
        return (-EINVAL);
    }
    /* check the parameter of ps interrupt persistence */
    if (persist > PERSISTENCE_MAX) {
	printk(KERN_ERR "%s: invalid parameter.\n", __func__);
        return (-EINVAL);
    }
    /* check the parameter of interrupt */
    if (intr > REG_INTERRUPT_MAX) {
	printk(KERN_ERR "%s: invalid parameter.\n", __func__);
        return (-EINVAL);
    }
    /* check the parameter of proximity sensor threshold high */
    if (psth_upper > REG_PSTH_MAX) {
	printk(KERN_ERR "%s: invalid parameter.\n", __func__);
        return (-EINVAL);
    }
    /* check the parameter of proximity sensor threshold low */
    if (psth_low > REG_PSTL_MAX) {
	printk(KERN_ERR "%s: invalid parameter.\n", __func__);
        return (-EINVAL);
    }
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
    result               = i2c_smbus_write_i2c_block_data(client, REG_MODECONTROL, sizeof(write_data), (unsigned char *)&write_data);

	if(result < 0)
	{
		printk(KERN_ERR "%s: i2c write fail. \n", __func__);
		return result;
	}

	gain = (psals_ctl & 0x3C) >> 2;	//gain setting values
	
	als_ps->enable = mode_ctl;
	als_ps->als_time = mode_table[(mode_ctl & 0xF)].ALS;
	als_ps->ps_time = mode_table[(mode_ctl & 0xF)].PS;
	als_ps->persistence = persist;
	als_ps->ps_th_l = psth_low;
	als_ps->ps_th_h = psth_upper;
	als_ps->als_th_l = alsth_low;
	als_ps->als_th_h = alsth_upper;
	als_ps->control = psals_ctl;
	als_ps->gain0 = gain_table[gain].DATA0;
	als_ps->gain1 = gain_table[gain].DATA1;
	als_ps->ledcurrent = psals_ctl & 0x03;
		
	als_ps->enable_als_sensor = 0; //grace add in 2013.10.09
	als_ps->enable_ps_sensor = 0;  //grace add in 2013.10.09

#ifdef _INIT_CALIB_
	rpr400_calibrate(client);
#else
	als_ps->ps_th_h_back = als_ps->ps_th_h;
	als_ps->ps_th_l_back = als_ps->ps_th_l;
#endif

    return (result);
}

/*********** I2C init/probing/exit functions ****************/

static struct i2c_driver rpr400_driver;

static int rpr400_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
#define ROHM_PSALS_ALSMAX (65535)
#define ROHM_PSALS_PSMAX  (4095)

	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct ALS_PS_DATA *data;
	//unsigned long flags;
	
	int err = 0;
	printk("%s started.\n",__func__);

	wake_lock_init(&ps_lock,WAKE_LOCK_SUSPEND,"ps wakelock");

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	data = kzalloc(sizeof(struct ALS_PS_DATA), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}
	data->client = client;
	i2c_set_clientdata(client, data);

//	printk("enable = %x\n", data->enable);

	mutex_init(&data->update_lock);

	INIT_DELAYED_WORK(&data->dwork, rpr400_ps_int_work_handler);
	INIT_DELAYED_WORK(&data->als_dwork, rpr400_als_polling_work_handler); 
	
//	printk("%s interrupt is hooked\n", __func__);

	/* Initialize the RPR400 chip */
	err = rpr400_init_client(client);
	if (err)
		goto exit_kfree;


	data->als_poll_delay = PS_ALS_SET_MIN_DELAY_TIME;	
//	err = rpr400_set_enable(client, BOTH100MS);
	/* Arrange als work after 125ms */
/*	
	data->als_poll_delay = PS_ALS_SET_MIN_DELAY_TIME;
	spin_lock_irqsave(&data->update_lock.wait_lock, flags); 
	//__cancel_delayed_work(&data->als_dwork);
	cancel_delayed_work(&data->als_dwork);
	schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));	// 125ms
	spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);
*/

	/* Register to Input Device */
	data->input_dev_als = devm_input_allocate_device(&client->dev);
	if (!data->input_dev_als) {
		err = -ENOMEM;
		printk("%s: Failed to allocate input device als\n", __func__);
		goto exit_free_irq;
	}

	data->input_dev_ps = devm_input_allocate_device(&client->dev);
	if (!data->input_dev_ps) {
		err = -ENOMEM;
		printk("%s: Failed to allocate input device ps\n", __func__);
		goto exit_free_dev_als;
	}
	
	input_set_drvdata(data->input_dev_ps, data); //grace modified in 2013.9.24
	input_set_drvdata(data->input_dev_als, data); //grace modified in 2013.9.24
	
	set_bit(EV_ABS, data->input_dev_als->evbit);
	set_bit(EV_ABS, data->input_dev_ps->evbit);

	input_set_abs_params(data->input_dev_als, ABS_MISC, 0, ROHM_PSALS_ALSMAX, 0, 0);
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, ROHM_PSALS_PSMAX, 0, 0);

	data->input_dev_als->name = "light";
	data->input_dev_ps->name = "proximity";
	//grace modified in 2013.3.22 begin
	data->input_dev_als->id.bustype = BUS_I2C;
  data->input_dev_als->dev.parent =&data->client->dev;
  data->input_dev_ps->id.bustype = BUS_I2C;
  data->input_dev_ps->dev.parent =&data->client->dev;
  //grace modified in 2013.3.22 begin


	err = input_register_device(data->input_dev_als);
	if (err) {
		err = -ENOMEM;
		printk("%s: Unable to register input device als: %s\n", __func__, 
		       data->input_dev_als->name);
		goto exit_free_dev_ps;
	}

/* add by shengweiguang begin */
    err = sysfs_create_group(&(data->input_dev_als->dev.kobj),
                 &rpr0410_als_attr_group);
	if (err)
	{
		printk("%s sysfs_create_group als failed\n", __func__);
		goto exit_unregister_dev_ps;
	}
/* add by shengweiguang end */

	err = input_register_device(data->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		printk("%s: Unable to register input device ps: %s\n", __func__, 
		       data->input_dev_ps->name);
		goto exit_unregister_dev_als;
	}

/* add by shengweiguang begin */
    err = sysfs_create_group(&(data->input_dev_ps->dev.kobj),
                 &rpr0410_ps_attr_group);
	if (err)
	{
		printk("%s sysfs_create_group ps failed\n", __func__);
		goto exit_unregister_dev_ps;
	}
/* add by shengweiguang end */
	
	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &rpr400_attr_group);
	if (err)
	{
		printk("%s sysfs_create_groupX\n", __func__);
		goto exit_unregister_dev_ps;
	}

	printk("%s support ver. %s enabled\n", __func__, DRIVER_VERSION);
	if (request_irq(client->irq, rpr400_interrupt, IRQF_DISABLED|IRQ_TYPE_EDGE_FALLING,
		RPR400_DRV_NAME, (void *)client)) {
		printk("%s Could not allocate RPR400_INT !\n", __func__);
	
		goto exit_kfree;
	}

/* add by shengweiguang begin */
	data->als_cdev = sensors_light_cdev;
	data->als_cdev.sensors_enable = rpr0410_als_set_enable;
	data->als_cdev.sensors_poll_delay = rpr0410_als_poll_delay;
	data->ps_cdev = sensors_proximity_cdev;
	data->ps_cdev.sensors_enable = rpr0410_ps_set_enable;
	data->ps_cdev.sensors_get_crosstalk = rpr0410_get_ps_crosstalk;
	data->ps_cdev.sensors_poll_delay = NULL;
	//err = sensors_classdev_register(&client->dev, &data->als_cdev);
	err = sensors_classdev_register(&data->input_dev_als->dev, &data->als_cdev);
	if (err) {
		pr_err("%s: Unable to register to sensors class: %d\n",
				__func__, err);
		goto exit_unregister_dev_ps;
	}

	//err = sensors_classdev_register(&client->dev, &data->ps_cdev);
	err = sensors_classdev_register(&data->input_dev_ps->dev, &data->ps_cdev);
	if (err) {
		pr_err("%s: Unable to register to sensors class: %d\n",
			       __func__, err);
		goto exit_unregister_dev_ps;
	}
/* add by shengweiguang end */
	
	printk(KERN_INFO "%s: INT No. %d", __func__, client->irq);
	return 0;


exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);	
exit_unregister_dev_als:
	printk("%s exit_unregister_dev_als:\n", __func__);
	input_unregister_device(data->input_dev_als);
exit_free_dev_ps:
	input_free_device(data->input_dev_ps);
exit_free_dev_als:
	input_free_device(data->input_dev_als);
exit_free_irq:
	free_irq(client->irq, client);	
exit_kfree:
	kfree(data);
exit:
	return err;

#undef ROHM_PSALS_ALSMAX
#undef ROHM_PSALS_PSMAX
}

static int rpr400_remove(struct i2c_client *client)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	input_unregister_device(als_ps->input_dev_als);
	input_unregister_device(als_ps->input_dev_ps);
	
	input_free_device(als_ps->input_dev_als);
	input_free_device(als_ps->input_dev_ps);

	free_irq(client->irq, client);

	sysfs_remove_group(&client->dev.kobj, &rpr400_attr_group);

	/* Power down the device */
	rpr400_set_enable(client, 0);

	kfree(als_ps);

	return 0;
}

static int rpr400_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk("%s\n", __func__);

	disable_irq(client->irq);
	//wake_unlock(&ps_lock);
	return rpr400_set_enable(client, 0);
}

static int rpr400_resume(struct i2c_client *client)
{
	printk("%s \n", __func__);
	//wake_lock(&ps_lock);
	enable_irq(client->irq);
	return rpr400_set_enable(client, PS100MS);
}


MODULE_DEVICE_TABLE(i2c, rpr400_id);

static const struct i2c_device_id rpr400_id[] = {
	{ "rpr400", 0 },
	{ }
};
/* add by shengweiguang begin */
static struct of_device_id rpr0410_match_table[] = {
	{ .compatible = "rohm,rpr0410", },
	{ },
};
/* add by shengweiguang end */
static struct i2c_driver rpr400_driver = {
	.driver = {
		.name	= RPR400_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table= rpr0410_match_table,
	},
	.suspend = rpr400_suspend,
	.resume	= rpr400_resume,
	.probe	= rpr400_probe,
	.remove	= rpr400_remove,
	.id_table = rpr400_id,
};

static int __init rpr400_init(void)
{
	return i2c_add_driver(&rpr400_driver);
}

static void __exit rpr400_exit(void)
{
	i2c_del_driver(&rpr400_driver);
}

MODULE_AUTHOR("Andy Mi @ ROHM");
MODULE_DESCRIPTION("RPR400 ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(rpr400_init);
module_exit(rpr400_exit);
