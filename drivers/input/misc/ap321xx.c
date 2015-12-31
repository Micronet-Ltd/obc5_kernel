/*
 * This file is part of the AP3216C sensor driver.
 * AP3216C is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: YC Hou <yc.hou@liteonsemi.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: ap3216c_v1.13_20121203_MSM7227.c
 *
 * Summary:
 *	AP3216C sensor dirver.
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 06/28/11 YC		 Original Creation (Test version:1.0)
 * 06/28/11 YC       Change dev name to dyna for demo purpose (ver 1.5).
 * 08/29/11 YC       Add engineer mode. Change version to 1.6.
 * 09/26/11 YC       Add calibration compensation function and add not power up 
 *                   prompt. Change version to 1.7.
 * 02/02/12 YC       1. Modify irq function to seperate two interrupt routine. 
 *					 2. Fix the index of reg array error in em write. 
 * 02/22/12 YC       3. Merge AP3216C and AP3216C into the same driver. (ver 1.8)
 * 03/01/12 YC       Add AP3212C into the driver. (ver 1.8)
 * 08/09/12 YC       1. Seperate 1 input dev into 2 input dev. Both ALS and PS can
 *					    use their own dev.
 *					 2. Add timer for polling mode.
 *					 3. Add early_suspend/late_resume function.
 * 09/03/12 YC       Modify the edit errors. Set ver to v1.10.
 * 09/04/12 YC       Fix the PS data print out error in work function. 
 *					 Modify to add interrupt and polling control by polling flag. Set ver to v1.11.
 * 11/01/12 YC       Fix the polling flag setting error.
 * 12/03/12 YC       Update dynamic range of AP3216C. Change to v1.13.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/irq.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef CONFIG_GET_HARDWARE_INFO
#include <mach/hardware_info.h>
#endif
//added  by yanfei for sensor list 2014-06-23 
#include <linux/sensors.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */
#include <linux/regulator/consumer.h>

#define AP3216C_DRV_NAME	"ap321xx"
#define DRIVER_VERSION		"1.13"

#define AP3216C_NUM_CACHABLE_REGS	26

#define AP3216C_ALS_CALIBRATION  0x19		//add by zg
#define AP3216C_RAN_COMMAND	0x10
#define AP3216C_RAN_MASK		0x30
#define AP3216C_RAN_SHIFT	(4)

#define AP3216C_MODE_COMMAND	0x00
#define AP3216C_MODE_SHIFT	(0)
#define AP3216C_MODE_MASK	0x07

#define	AP3216C_ADC_LSB		0x0c
#define	AP3216C_ADC_MSB		0x0d

#define	AP3216C_PX_LSB		0x0e
#define	AP3216C_PX_MSB		0x0f
#define	AP3216C_PX_LSB_MASK	0x0f
#define	AP3216C_PX_MSB_MASK	0x3f

#define AP3216C_OBJ_COMMAND	0x0f
#define AP3216C_OBJ_MASK		0x80
#define AP3216C_OBJ_SHIFT	(7)

#define AP3216C_INT_COMMAND	0x01
#define AP3216C_INT_SHIFT	(0)
#define AP3216C_INT_MASK		0x03
#define AP3216C_INT_PMASK		0x02
#define AP3216C_INT_AMASK		0x01

#define AP3216C_ALS_LTHL			0x1a
#define AP3216C_ALS_LTHL_SHIFT	(0)
#define AP3216C_ALS_LTHL_MASK	0xff

#define AP3216C_ALS_LTHH			0x1b
#define AP3216C_ALS_LTHH_SHIFT	(0)
#define AP3216C_ALS_LTHH_MASK	0xff

#define AP3216C_ALS_HTHL			0x1c
#define AP3216C_ALS_HTHL_SHIFT	(0)
#define AP3216C_ALS_HTHL_MASK	0xff

#define AP3216C_ALS_HTHH			0x1d
#define AP3216C_ALS_HTHH_SHIFT	(0)
#define AP3216C_ALS_HTHH_MASK	0xff

//added by litao to select zone mode of proximity interrupt trigger (qw601) 2013-12-27 
#define AP3216C_PS_INTERRUPT_MODE	0x22	//default value is 0x01

#define AP3216C_PX_LTHL			0x2a
#define AP3216C_PX_LTHL_SHIFT	(0)
#define AP3216C_PX_LTHL_MASK		0x03

#define AP3216C_PX_LTHH			0x2b
#define AP3216C_PX_LTHH_SHIFT	(0)
#define AP3216C_PX_LTHH_MASK		0xff

#define AP3216C_PX_HTHL			0x2c
#define AP3216C_PX_HTHL_SHIFT	(0)
#define AP3216C_PX_HTHL_MASK		0x03

#define AP3216C_PX_HTHH			0x2d
#define AP3216C_PX_HTHH_SHIFT	(0)
#define AP3216C_PX_HTHH_MASK		0xff

#define AP3216C_PX_CONFIGURE	0x20
#define AP3216C_PX_LED_CONTROL 0x21

/* add by shengweiguang begin */
#define AP3426_INT_CONTROL	0x02

#define AP3426_PX_LTHL_MASK 0xff
#define AP3426_PX_LTHH_MASK 0x03

#define AP3426_PX_HTHL_MASK 0xff
#define AP3426_PX_HTHH_MASK 0x03

#define AP3426_PX_MSB_MASK 0x03
#define AP3426_PX_LSB_MASK 0xff
/* add by shengweiguang end */

#define ALS_ACTIVE    0x01
#define PS_ACTIVE    0x02
#define POWER_DOWN    0x00
#define ALS_SET_MIN_DELAY_TIME (100)
#define ALS_SET_DEFAULT_DELAY_TIME (200)

//added by litao to select zone mode of proximity interrupt trigger (qw601) 2013-12-27 begin
#define PS_INTERRUPT_ZONE_MODE
#define PS_INTERRUPT_HIGH_THRESHLOD		50
#define PS_INTERRUPT_LOW_THRESHLOD		30
//added by litao to select zone mode of proximity interrupt trigger (qw601) 2013-12-27 end

//added by litao for ap321xx (qw601) 2014-01-14 begin
#define PS_THRES_HIGH_MAX 0xfff
#define PS_THRES_LOW_MAX	0xfff
//added by litao for ap321xx (qw601) 2014-01-14 end
//added by yanfei  for ap321xx pinctrl (ql1001) 2014-06-27
static struct ap3216c_data *pdev_data = NULL;
//added by chenchen for pocket mode 20140925
bool proximity_open_flag;

//#define LSC_DBG
#ifdef LSC_DBG
#define LDBG(s,args...)	{printk("LDBG: func [%s], line [%d], ",__func__,__LINE__); printk(s,## args);}
#else
#define LDBG(s,args...) {}
#endif
/*add by shengweiguang begin */
enum ap_chip_type{
	AP321XX,
	AP3426,
};

static enum ap_chip_type qrt_chip_type;
/* add by shengweiguang end */
 
struct ap3216c_data {
	struct i2c_client *client;
	struct mutex lock;
	u8 reg_cache[AP3216C_NUM_CACHABLE_REGS];
	int irq;
	int gpio_int;
//	struct early_suspend early_suspend;
//added by yanfei  for ap321xx pinctrl (ql1001) 2014-06-27 begin
	int (*power_on)(bool);
 	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;
//added by yanfei  for ap321xx pinctrl (ql1001) 2014-06-27 end
//added by yanfei  for ap321xx (ql1001) 2014-06-23 begin
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;
//added by yanfei  for ap321xx (ql1001) 2014-06-23 end	
	struct input_dev *light_input_dev;
	struct delayed_work work_light;
	int  light_poll_delay;
	bool als_enable;

	struct input_dev *proximity_input_dev;
	struct delayed_work  work_proximity;
	int proximity_poll_delay;
	bool ps_enable;
	struct wake_lock prx_wake_lock;

//added by litao for ap321xx (qw601) 2014-01-13 begin
	unsigned int ps_thres_high;
	unsigned int ps_thres_low;
	unsigned int ps_high;
	unsigned int ps_low;	
//add by yanfei for ap3216c calibration fuction 2014-08-06
	unsigned int  crosstalk;
//added by litao for ap321xx (qw601) 2014-01-13 end
	
	struct regulator *vdd;
	struct regulator *vio;
//delete by yanfei  for ap321xx pinctrl (ql1001) 2014-06-27
//	bool power_on;
};
//added by yanfei  for ap321xx (ql1001) 2014-06-23 begin
static struct sensors_classdev sensors_light_cdev = {
	.name = "ap321xx-light",
	.vendor = "liteon",
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
	.name = "ap321xx-proximity",
	.vendor = "liteon",
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
//add by yanfei for ap3216 calibration fuction 2014-08-06
//added by yanfei  for ap321xx (ql1001) 2014-06-23 end
// AP3216C register
static u8 ap3216c_reg[AP3216C_NUM_CACHABLE_REGS] = 
	{0x00,0x01,0x02,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,
	 0x10,0x19,0x1a,0x1b,0x1c,0x1d,
	 0x20,0x21,0x22,0x23,0x24,0x28,0x29,0x2a,0x2b,0x2c,0x2d};

// AP3216C range
static int ap3216c_range[4] = {20661,5162,1291,323};

int cali = 100;



#define ADD_TO_IDX(addr,idx)	{														\
									int i;												\
									for(i = 0; i < AP3216C_NUM_CACHABLE_REGS; i++)						\
									{													\
										if (addr == ap3216c_reg[i])						\
										{												\
											idx = i;									\
											break;										\
										}												\
									}													\
								}

/*
 * register access helpers
 */

static int __ap3216c_read_reg(struct i2c_client *client,
			       u32 reg, u8 mask, u8 shift)
{
	struct ap3216c_data *data = i2c_get_clientdata(client);
	u8 idx = 0xff;

	ADD_TO_IDX(reg,idx)
	return (data->reg_cache[idx] & mask) >> shift;
}

static int __ap3216c_write_reg(struct i2c_client *client,
				u32 reg, u8 mask, u8 shift, u8 val)
{
	struct ap3216c_data *data = i2c_get_clientdata(client);
	int ret = 0;
	u8 tmp;
	u8 idx = 0xff;

	ADD_TO_IDX(reg,idx)
	if (idx >= AP3216C_NUM_CACHABLE_REGS)
		return -EINVAL;

	mutex_lock(&data->lock);

	tmp = data->reg_cache[idx];
	tmp &= ~mask;
	tmp |= val << shift;

	ret = i2c_smbus_write_byte_data(client, reg, tmp);
	if (!ret)
		data->reg_cache[idx] = tmp;

	mutex_unlock(&data->lock);
	return ret;
}

/*
 * internally used functions
 */

/* range */
static int ap3216c_get_range(struct i2c_client *client)
{
	u8 idx = __ap3216c_read_reg(client, AP3216C_RAN_COMMAND,
		AP3216C_RAN_MASK, AP3216C_RAN_SHIFT); 
	return ap3216c_range[idx];
}

static int ap3216c_set_range(struct i2c_client *client, int range)
{
	return __ap3216c_write_reg(client, AP3216C_RAN_COMMAND,
		AP3216C_RAN_MASK, AP3216C_RAN_SHIFT, range);;
}


/* mode */
static int ap3216c_get_mode(struct i2c_client *client)
{
	int ret;

	ret = __ap3216c_read_reg(client, AP3216C_MODE_COMMAND,
			AP3216C_MODE_MASK, AP3216C_MODE_SHIFT);

	return ret;
}

/* sw reset */
static int ap3216c_sw_reset(struct i2c_client *client)
{
	int ret;

		/* SW reset */
			ret=__ap3216c_write_reg(client, AP3216C_MODE_COMMAND,
				  AP3216C_MODE_MASK, AP3216C_MODE_SHIFT, 4);
	return ret;
}

/* ALS low threshold */
static int ap3216c_get_althres(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __ap3216c_read_reg(client, AP3216C_ALS_LTHL,
				AP3216C_ALS_LTHL_MASK, AP3216C_ALS_LTHL_SHIFT);
	msb = __ap3216c_read_reg(client, AP3216C_ALS_LTHH,
				AP3216C_ALS_LTHH_MASK, AP3216C_ALS_LTHH_SHIFT);
	return ((msb << 8) | lsb);
}

static int ap3216c_set_althres(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	
	msb = val >> 8;
	lsb = val & AP3216C_ALS_LTHL_MASK;

	err = __ap3216c_write_reg(client, AP3216C_ALS_LTHL,
		AP3216C_ALS_LTHL_MASK, AP3216C_ALS_LTHL_SHIFT, lsb);
	if (err)
		return err;

	err = __ap3216c_write_reg(client, AP3216C_ALS_LTHH,
		AP3216C_ALS_LTHH_MASK, AP3216C_ALS_LTHH_SHIFT, msb);

	return err;
}

/* ALS high threshold */
static int ap3216c_get_ahthres(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __ap3216c_read_reg(client, AP3216C_ALS_HTHL,
				AP3216C_ALS_HTHL_MASK, AP3216C_ALS_HTHL_SHIFT);
	msb = __ap3216c_read_reg(client, AP3216C_ALS_HTHH,
				AP3216C_ALS_HTHH_MASK, AP3216C_ALS_HTHH_SHIFT);
	return ((msb << 8) | lsb);
}

static int ap3216c_set_ahthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	
	msb = val >> 8;
	lsb = val & AP3216C_ALS_HTHL_MASK;
	
	err = __ap3216c_write_reg(client, AP3216C_ALS_HTHL,
		AP3216C_ALS_HTHL_MASK, AP3216C_ALS_HTHL_SHIFT, lsb);
	if (err)
		return err;

	err = __ap3216c_write_reg(client, AP3216C_ALS_HTHH,
		AP3216C_ALS_HTHH_MASK, AP3216C_ALS_HTHH_SHIFT, msb);

	return err;
}

/* PX low threshold */
static int ap3216c_get_plthres(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __ap3216c_read_reg(client, AP3216C_PX_LTHL,
				AP3216C_PX_LTHL_MASK, AP3216C_PX_LTHL_SHIFT);
	msb = __ap3216c_read_reg(client, AP3216C_PX_LTHH,
				AP3216C_PX_LTHH_MASK, AP3216C_PX_LTHH_SHIFT);
	return ((msb << 2) | lsb);
}

/* add by shengweiguang begin */
static int ap3426_get_plthres(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __ap3216c_read_reg(client, AP3216C_PX_LTHL,
				AP3426_PX_LTHL_MASK, AP3216C_PX_LTHL_SHIFT);
	msb = __ap3216c_read_reg(client, AP3216C_PX_LTHH,
				AP3426_PX_LTHH_MASK, AP3216C_PX_LTHH_SHIFT);
	return ((msb << 8) | lsb);
}
/* add by shengweiguang end */

static int ap3216c_set_plthres(struct i2c_client *client, int val)
{
//added by litao for ap321xx (qw601) 2014-01-14
	struct ap3216c_data *data = i2c_get_clientdata(client);
	int lsb, msb, err;

//added by litao for ap321xx (qw601) 2014-01-14 begin
	if(val > PS_THRES_LOW_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}

	if(val > data->ps_thres_high)
	{
		printk(KERN_ERR "%s: higher than threshold high.\n", __func__);
		return -EINVAL;
	}
//added by litao for ap321xx (qw601) 2014-01-14	end

	msb = val >> 2;
	lsb = val & AP3216C_PX_LTHL_MASK;
	
	err = __ap3216c_write_reg(client, AP3216C_PX_LTHL,
		AP3216C_PX_LTHL_MASK, AP3216C_PX_LTHL_SHIFT, lsb);
	if (err)
		return err;

	err = __ap3216c_write_reg(client, AP3216C_PX_LTHH,
		AP3216C_PX_LTHH_MASK, AP3216C_PX_LTHH_SHIFT, msb);

	return err;
}

/* add by shengweiguang begin */
static int ap3426_set_plthres(struct i2c_client *client, int val)
{
//added by litao for ap321xx (qw601) 2014-01-14
	struct ap3216c_data *data = i2c_get_clientdata(client);
	int lsb, msb, err;

//added by litao for ap321xx (qw601) 2014-01-14 begin
	if(val > PS_THRES_LOW_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}

	if(val > data->ps_thres_high)
	{
		printk(KERN_ERR "%s: higher than threshold high.\n", __func__);
		return -EINVAL;
	}
//added by litao for ap321xx (qw601) 2014-01-14	end

	msb = val >> 8;
	lsb = val & AP3426_PX_LTHL_MASK;
	
	err = __ap3216c_write_reg(client, AP3216C_PX_LTHL,
		AP3426_PX_LTHL_MASK, AP3216C_PX_LTHL_SHIFT, lsb);
	if (err)
		return err;

	err = __ap3216c_write_reg(client, AP3216C_PX_LTHH,
		AP3426_PX_LTHH_MASK, AP3216C_PX_LTHH_SHIFT, msb);

	return err;
}
/* add by shengweiguag end */

/* PX high threshold */
static int ap3216c_get_phthres(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __ap3216c_read_reg(client, AP3216C_PX_HTHL,
				AP3216C_PX_HTHL_MASK, AP3216C_PX_HTHL_SHIFT);
	msb = __ap3216c_read_reg(client, AP3216C_PX_HTHH,
				AP3216C_PX_HTHH_MASK, AP3216C_PX_HTHH_SHIFT);
	return ((msb << 2) | lsb);
}

/* add by shengweiguang begin */
static int ap3426_get_phthres(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __ap3216c_read_reg(client, AP3216C_PX_HTHL,
				AP3426_PX_HTHL_MASK, AP3216C_PX_HTHL_SHIFT);
	msb = __ap3216c_read_reg(client, AP3216C_PX_HTHH,
				AP3426_PX_HTHH_MASK, AP3216C_PX_HTHH_SHIFT);
	return ((msb << 8) | lsb);
}
/* add by shengweiguang end */

static int ap3216c_set_phthres(struct i2c_client *client, int val)
{
//added by litao for ap321xx (qw601) 2014-01-14
	struct ap3216c_data *data = i2c_get_clientdata(client);
	int lsb, msb, err;

//added by litao for ap321xx (qw601) 2014-01-14 begin
	if(val > PS_THRES_HIGH_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}

	if(val < data->ps_thres_low)
	{
		printk(KERN_ERR "%s: lower than threshold low.\n", __func__);
		return -EINVAL;
	}
//added by litao for ap321xx (qw601) 2014-01-14	end
	
	msb = val >> 2;
	lsb = val & AP3216C_ALS_HTHL_MASK;
	
	err = __ap3216c_write_reg(client, AP3216C_PX_HTHL,
		AP3216C_PX_HTHL_MASK, AP3216C_PX_HTHL_SHIFT, lsb);
	if (err)
		return err;

	err = __ap3216c_write_reg(client, AP3216C_PX_HTHH,
		AP3216C_PX_HTHH_MASK, AP3216C_PX_HTHH_SHIFT, msb);

	return err;
}

/* add by shengweiguang begin */
static int ap3426_set_phthres(struct i2c_client *client, int val)
{
//added by litao for ap321xx (qw601) 2014-01-14
	struct ap3216c_data *data = i2c_get_clientdata(client);
	int lsb, msb, err;

//added by litao for ap321xx (qw601) 2014-01-14 begin
	if(val > PS_THRES_HIGH_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}

	if(val < data->ps_thres_low)
	{
		printk(KERN_ERR "%s: lower than threshold low.\n", __func__);
		return -EINVAL;
	}
//added by litao for ap321xx (qw601) 2014-01-14	end
	
	msb = val >> 8;
	lsb = val & AP3426_PX_HTHL_MASK;
	
	err = __ap3216c_write_reg(client, AP3216C_PX_HTHL,
		AP3426_PX_HTHL_MASK, AP3216C_PX_HTHL_SHIFT, lsb);
	if (err)
		return err;

	err = __ap3216c_write_reg(client, AP3216C_PX_HTHH,
		AP3426_PX_HTHH_MASK, AP3216C_PX_HTHH_SHIFT, msb);

	return err;
}

/* add by shengweiguang end */

static int ap3216c_get_adc_value(struct i2c_client *client, int lock)
{
	struct ap3216c_data *data = i2c_get_clientdata(client);
	unsigned int lsb, msb, range;
	unsigned long tmp;

	if (!lock)	mutex_lock(&data->lock);
	lsb = i2c_smbus_read_byte_data(client, AP3216C_ADC_LSB);

	if (lsb < 0) {
		if (!lock)	mutex_unlock(&data->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, AP3216C_ADC_MSB);
	if (!lock)	mutex_unlock(&data->lock);

	if (msb < 0)
		return msb;

	range = ap3216c_get_range(client);

	tmp = (((msb << 8) | lsb) * range) >> 16;
	tmp *= cali;

	return (tmp / 100);
}

static int ap3216c_get_object(struct i2c_client *client, int lock)
{
	struct ap3216c_data *data = i2c_get_clientdata(client);
	int val;

	if (!lock)	mutex_lock(&data->lock);
	val = i2c_smbus_read_byte_data(client, AP3216C_OBJ_COMMAND);
	val &= AP3216C_OBJ_MASK;

	if (!lock)	mutex_unlock(&data->lock);

	return val >> AP3216C_OBJ_SHIFT;
}
static int ap3216c_get_intstat(struct i2c_client *client)
{
	int val;
	
	val = i2c_smbus_read_byte_data(client, AP3216C_INT_COMMAND);
	val &= AP3216C_INT_MASK;

	return val >> AP3216C_INT_SHIFT;
}
static int ap3216c_get_px_value(struct i2c_client *client, int lock)
{
	struct ap3216c_data *data = i2c_get_clientdata(client);
	u8 lsb, msb;

	if (!lock) mutex_lock(&data->lock);
	
	lsb = i2c_smbus_read_byte_data(client, AP3216C_PX_LSB);
	msb = i2c_smbus_read_byte_data(client, AP3216C_PX_MSB);

	if (!lock)	mutex_unlock(&data->lock);

	return (u32)(((msb & AP3216C_PX_MSB_MASK) << 4) | (lsb & AP3216C_PX_LSB_MASK));
}

/* add by shengweiguang begin */
static int ap3426_get_px_value(struct i2c_client *client, int lock)
{
	struct ap3216c_data *data = i2c_get_clientdata(client);
	u8 lsb, msb;
	if (!lock) mutex_lock(&data->lock);
	
	lsb = i2c_smbus_read_byte_data(client, AP3216C_PX_LSB);
	msb = i2c_smbus_read_byte_data(client, AP3216C_PX_MSB);

	if (!lock)	mutex_unlock(&data->lock);

	return (u32)(((msb & AP3426_PX_MSB_MASK) << 8) | (lsb & AP3426_PX_LSB_MASK));
}
/* add by shengweiguang end */

/*
 * sysfs layer
 */

/* range */
static ssize_t ap3216c_show_range(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct ap3216c_data *data = input_get_drvdata(input);
	return sprintf(buf, "%i\n", ap3216c_get_range(data->client));
}

static ssize_t ap3216c_store_range(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct ap3216c_data *data = input_get_drvdata(input);
	unsigned long val;
	int ret;

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 3))
		return -EINVAL;

	ret = ap3216c_set_range(data->client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(range, S_IWUSR | S_IRUGO,
		   ap3216c_show_range, ap3216c_store_range);



/* lux */
/***********************add ps mode*****************************/


/**********************add ps mode******************************/

static ssize_t ap3216c_show_lux(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct ap3216c_data *data = input_get_drvdata(input);

	/* No LUX data if power down */
	if (ap3216c_get_mode(data->client) == 0x00)
		return sprintf((char*) buf, "%s\n", "Please power up first!");

	return sprintf(buf, "%d\n", ap3216c_get_adc_value(data->client,0));
}

static DEVICE_ATTR(lux, S_IRUGO, ap3216c_show_lux, NULL);


/* Px data */
static ssize_t ap3216c_show_pxvalue(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct ap3216c_data *data = input_get_drvdata(input);

	/* No Px data if power down */
	if (ap3216c_get_mode(data->client) == 0x00)
		return -EBUSY;

/* add by shengweiguang begin */
	if (qrt_chip_type == AP3426)
	{
		return sprintf(buf, "%d\n", ap3426_get_px_value(data->client,0));
	}		
/* add by shengweiguang end */

	return sprintf(buf, "%d\n", ap3216c_get_px_value(data->client,0));
}

static DEVICE_ATTR(pxvalue, S_IRUGO, ap3216c_show_pxvalue, NULL);


/* proximity object detect */
static ssize_t ap3216c_show_object(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct ap3216c_data *data = input_get_drvdata(input);
	return sprintf(buf, "%d\n", ap3216c_get_object(data->client,0));
}

static DEVICE_ATTR(object, S_IRUGO, ap3216c_show_object, NULL);


/* ALS low threshold */
static ssize_t ap3216c_show_althres(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct ap3216c_data *data = input_get_drvdata(input);
	return sprintf(buf, "%d\n", ap3216c_get_althres(data->client));
}

static ssize_t ap3216c_store_althres(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct ap3216c_data *data = input_get_drvdata(input);
	unsigned long val;
	int ret;

	if (strict_strtoul(buf, 10, &val) < 0)
		return -EINVAL;

	ret = ap3216c_set_althres(data->client, val);
	if (ret < 0)
		return ret;

	return count;
}
static DEVICE_ATTR(althres, S_IWUSR | S_IRUGO,
		   ap3216c_show_althres, ap3216c_store_althres);


/* ALS high threshold */
static ssize_t ap3216c_show_ahthres(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct ap3216c_data *data = input_get_drvdata(input);
	return sprintf(buf, "%d\n", ap3216c_get_ahthres(data->client));
}

static ssize_t ap3216c_store_ahthres(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct ap3216c_data *data = input_get_drvdata(input);
	unsigned long val;
	int ret;

	if (strict_strtoul(buf, 10, &val) < 0)
		return -EINVAL;

	ret = ap3216c_set_ahthres(data->client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(ahthres, S_IWUSR | S_IRUGO,
		   ap3216c_show_ahthres, ap3216c_store_ahthres);

/* Px low threshold */
static ssize_t ap3216c_show_plthres(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct ap3216c_data *data = input_get_drvdata(input);

/* add by shengweiguang begin */
	if (qrt_chip_type == AP3426)
		return sprintf(buf, "%d\n", ap3426_get_plthres(data->client));
	else
/* add by shengweiguang end */
		return sprintf(buf, "%d\n", ap3216c_get_plthres(data->client));
}

static ssize_t ap3216c_store_plthres(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct ap3216c_data *data = input_get_drvdata(input);
	unsigned long val;
	int ret;

	if (strict_strtoul(buf, 10, &val) < 0)
		return -EINVAL;

/* add by shengweiguang begin */
	if (qrt_chip_type == AP3426)
			ret = ap3426_set_plthres(data->client, val);
	else
/* add by shengweiguang end */
		ret = ap3216c_set_plthres(data->client, val);
	if (ret < 0)
		return ret;

//added by litao for ap321xx (qw601) 2014-01-14
	data->ps_thres_low = val;

	return count;
}

static DEVICE_ATTR(plthres, S_IWUSR | S_IRUGO,
		   ap3216c_show_plthres, ap3216c_store_plthres);

/* Px high threshold */
static ssize_t ap3216c_show_phthres(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct ap3216c_data *data = input_get_drvdata(input);

/* add by shengweiguang begin */
	if (qrt_chip_type == AP3426)
		return sprintf(buf, "%d\n", ap3426_get_phthres(data->client));
	else
/* add by shengweiguang end */
		return sprintf(buf, "%d\n", ap3216c_get_phthres(data->client));
}

static ssize_t ap3216c_store_phthres(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct ap3216c_data *data = input_get_drvdata(input);
	unsigned long val;
	int ret;

	if (strict_strtoul(buf, 10, &val) < 0)
		return -EINVAL;

/* add by shengweiguang begin */
	if (qrt_chip_type == AP3426)
		ret = ap3426_set_phthres(data->client, val);
	else
/* add by shengweiguang end */
		ret = ap3216c_set_phthres(data->client, val);
	if (ret < 0)
		return ret;

//added by litao for ap321xx (qw601) 2014-01-14
	data->ps_thres_high = val;
	
	return count;
}

static DEVICE_ATTR(phthres, S_IWUSR | S_IRUGO,
		   ap3216c_show_phthres, ap3216c_store_phthres);


static ssize_t ap3216c_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct ap3216c_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", data->ps_enable);
}
//added by yanfei  for ap321xx  (ql1001) 2014-06-23 begin
static int ap3216c_enable_ps_sensor(struct i2c_client *client,
		 int val)
{
       struct ap3216c_data *data = i2c_get_clientdata(client);
	 //modifird by yanfei  for ap321xx log (ql1001) 2014-06-27
	printk(KERN_ERR "ap3216c enable PS sensor -> %d \n", val);
	if ((val != 0) && (val != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, val);
		return -EINVAL;
	}
	if(val == 1) 
	{

		if (data->ps_enable== 0) 
		{
			data->ps_enable= 1;
		
			if (data->als_enable == 0) 
			{
				__ap3216c_write_reg(data->client, AP3216C_MODE_COMMAND,
				  AP3216C_MODE_MASK, AP3216C_MODE_SHIFT, 2);
			}
			else
			{
				__ap3216c_write_reg(data->client, AP3216C_MODE_COMMAND,
				  AP3216C_MODE_MASK, AP3216C_MODE_SHIFT, 3);
			}

//deleted by litao because it's not the correct method of solving the problem (qw601) 2013-12-27
//			schedule_delayed_work(&data->work_proximity, 0);
			enable_irq(data->client->irq);	

//deleted by litao to cancel this irq api (ql1006) 2014-08-29
			enable_irq_wake(data->client->irq);
			
		}
	} 
	else 
	{
		if(data->ps_enable == 1)
		{
			data->ps_enable = 0;
			if (data->als_enable == 0) 
			{
				__ap3216c_write_reg(data->client, AP3216C_MODE_COMMAND,
				  AP3216C_MODE_MASK, AP3216C_MODE_SHIFT, 0);
			}
			else 
			{
				__ap3216c_write_reg(data->client, AP3216C_MODE_COMMAND,
				  AP3216C_MODE_MASK, AP3216C_MODE_SHIFT, 1);
			}
//deleted by litao to cancel this irq api (ql1006) 2014-08-29
			disable_irq_wake(data->client->irq);
			disable_irq(data->client->irq);
		}
	}
		
	return 0;
}
static ssize_t ap3216c_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int err = -1;
	printk(KERN_ERR "ap3216c enable PS sensor -> %ld \n", val);
	if (val != 0 && val != 1) {
		pr_err("%s: invalid value(%ld)\n", __func__, val);
		return -EINVAL;
	}
	err = ap3216c_enable_ps_sensor(client,val);	
	return count;
}
static int ap3216c_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
		struct ap3216c_data *data = container_of(sensors_cdev,
			struct ap3216c_data, ps_cdev);

	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	return ap3216c_enable_ps_sensor(data->client, enable);
}
//added by yanfei  for ap321xx  (ql1001) 2014-06-23 end

static DEVICE_ATTR(enable_ps_sensor,  S_IRUGO | S_IWUGO ,
				   ap3216c_show_enable_ps_sensor, ap3216c_store_enable_ps_sensor);
/***********************************calibration fuction************************************************/
static ssize_t ap3216c_show_ps_calibration(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct ap3216c_data *data = input_get_drvdata(input);
	struct i2c_client *client=data->client;
	int tmp;
	int i;
	int read_count = 0;
	int sum_value = 0;
	int ps_th_limit = 623;
	msleep(200);
	for(i=0;i<10;i++){

/* add by shengweiguang begin */
		if (qrt_chip_type == AP3426)
			tmp = ap3426_get_px_value(client,0);
		else
/* add by shengweiguang end */
			tmp = ap3216c_get_px_value(client,0);
		if(tmp < 0)
		{
			printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
			return -1;	
		}else if((tmp < ps_th_limit)&&(tmp >0)){
			sum_value += tmp;
			read_count ++;
		}
		msleep(100);
	}
//add by yanfei for calibration fail
	if(read_count == 0){
		data->crosstalk =1;
		return sprintf(buf, "%d\n",  data->crosstalk);
	}else{
	        data->crosstalk = sum_value/read_count;
		 printk(KERN_ERR"%s data->crosstalk=%d,sum_value=%d,read_count=%d\n",__func__,data->crosstalk,sum_value,read_count);
/* add by shengweiguang begin */
		if (qrt_chip_type == AP3426)
		 	tmp=ap3426_set_phthres(client, data->ps_high+data->crosstalk);
		else
/* add by shengweiguang end */	
		 	tmp=ap3216c_set_phthres(client, data->ps_high+data->crosstalk);
		 if(tmp<0)
		 	{
		 		printk(KERN_ERR"%s err ap3216c_set_phthres\n ",__func__);
		 	}
		 data->ps_thres_high=data->ps_high+data->crosstalk;
/* add by shengweiguang begin */
		if (qrt_chip_type == AP3426)
			tmp=ap3426_set_plthres(client, data->ps_low+data->crosstalk);
		else
/* add by shengweiguang end */		 
		 	tmp=ap3216c_set_plthres(client, data->ps_low+data->crosstalk);
		 if(tmp<0)
		 	{
		 		printk(KERN_ERR"%s err ap3216c_set_plthres\n ",__func__);
		 	}		 
		 data->ps_thres_low=data->ps_low+data->crosstalk;
	}
	printk(KERN_ERR"%s data->crosstalk=%d\n",__func__,data->crosstalk);
	return sprintf(buf, "%d\n", data->crosstalk);

}

static ssize_t ap3216c_get_ps_crosstalk(struct sensors_classdev *sensors_cdev,
		unsigned int prox_data)
{
	struct ap3216c_data *data = container_of(sensors_cdev,
			struct ap3216c_data, ps_cdev);
	struct i2c_client *client=data->client;
	int ret;
	data->crosstalk= prox_data;
	if(data->crosstalk==0)
		{
			printk(KERN_ERR"%s %d %d %d %d \n",__func__,data->ps_thres_high,data->ps_thres_low,data->ps_high,data->ps_low);
			return 0;
		}

/* add by shengweiguang begin */
	if (qrt_chip_type == AP3426)
		ret=ap3426_set_phthres(client, data->ps_high+data->crosstalk);
	else
/* add by shengweiguang end */
		ret=ap3216c_set_phthres(client, data->ps_high+data->crosstalk);
	if(ret<0)
		{
		 	printk(KERN_ERR"%s err ap3216c_set_phthres\n ",__func__);
		}
	else
		{
			data->ps_thres_high=data->ps_high+data->crosstalk;
		}
		
/* add by shengweiguang begin */
		if (qrt_chip_type == AP3426)
			ret=ap3426_set_plthres(client, data->ps_low+data->crosstalk);
		else
/* add by shengweiguang end */	
		ret=ap3216c_set_plthres(client, data->ps_low+data->crosstalk);
	if(ret<0)
		{
			 printk(KERN_ERR"%s err ap3216c_set_plthres\n ",__func__);
		 }
	else
		{
			data->ps_thres_low=data->ps_low+data->crosstalk;
		}
	return 0;
}
static DEVICE_ATTR(ps_cal,  0664 ,
				   ap3216c_show_ps_calibration, NULL);
//modified by yanfei for calibration 2014/07/28  end

/***********************************calibration fuction************************************************/
/* calibration */
static ssize_t ap3216c_show_calibration_state(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return sprintf(buf, "%d\n", cali);
}

static ssize_t ap3216c_store_calibration_state(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct ap3216c_data *data = input_get_drvdata(input);
	int stdls, lux; 
	char tmp[10];

	/* No LUX data if not operational */
	if (ap3216c_get_mode(data->client) == 0x00)
	{
		printk("Please power up first!");
		return -EINVAL;
	}

	cali = 100;
	sscanf(buf, "%d %s", &stdls, tmp);

	if (!strncmp(tmp, "-setcv", 6))
	{
		cali = stdls;
		return -EBUSY;
	}

	if (stdls < 0)
	{
		printk("Std light source: [%d] < 0 !!!\nCheck again, please.\n\
		Set calibration factor to 100.\n", stdls);
		return -EBUSY;
	}

	lux = ap3216c_get_adc_value(data->client, 0);
	cali = stdls * 100 / lux;

	return -EBUSY;
}

static DEVICE_ATTR(calibration, S_IWUSR | S_IRUGO,
		   ap3216c_show_calibration_state, ap3216c_store_calibration_state);
static ssize_t ap3216c_show_als_poll_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{

	struct input_dev *input = to_input_dev(dev);
	struct ap3216c_data *data = input_get_drvdata(input);
	return sprintf(buf, "%d\n", data->light_poll_delay*1000000);	// return in nano-second
}
//added by yanfei for ap321xx(ql1001) 2014-06-23 begin
static int ap321xx_set_als_poll_delay(struct i2c_client *client,
		unsigned int val)
{
	struct ap3216c_data *data = i2c_get_clientdata(client);

	pr_debug("%s: val=%d\n", __func__, val);

	if (val < ALS_SET_MIN_DELAY_TIME * 1000000)
		val = ALS_SET_MIN_DELAY_TIME * 1000000;	
	
	data->light_poll_delay = val /1000000;	// convert us => ms

	
	/* we need this polling timer routine for sunlight canellation */
	if(data->als_enable == 1){ 
		/*
		 * If work is already scheduled then subsequent schedules will not
		 * change the scheduled time that's why we have to cancel it first.
		 */
		 //modify by yanfei for work 
		cancel_delayed_work(&data->work_light);
		schedule_delayed_work(&data->work_light, msecs_to_jiffies(data->light_poll_delay));
				
	}
	return 0;

}
static ssize_t ap3216c_store_als_poll_delay(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	ap321xx_set_als_poll_delay(client, val);
	return count;
}
static int  ap3216c_als_poll_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct ap3216c_data *data = container_of(sensors_cdev,
			struct ap3216c_data, als_cdev);
	ap321xx_set_als_poll_delay(data->client, delay_msec);
	return 0;
}
//added by yanfei for ap321xx(ql1001) 2014-06-23 end
static DEVICE_ATTR(poll_delay,  S_IRUGO | S_IWUGO,
				    ap3216c_show_als_poll_delay, ap3216c_store_als_poll_delay);
static ssize_t ap3216c_show_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct ap3216c_data *data = input_get_drvdata(input);
	
	return sprintf(buf, "%d\n", data->als_enable);
}
//added by yanfei for ap321xx(ql1001) 2014-06-23 begin
static int ap3216c_enable_als_sensor(struct i2c_client *client, int val)
{
	struct ap3216c_data *data = i2c_get_clientdata(client);
	//modifird by yanfei  for ap321xx log (ql1001) 2014-06-27
	printk(KERN_ERR "ap3216c enable ALS sensor -> %d\n", val);

	if ((val != 0) && (val != 1)) {
		pr_err("%s: invalid value (val = %d)\n", __func__, val);
		return -EINVAL;
	}
	
	if(val == 1)
	{
		//turn on light  sensor
		if (data->als_enable==0)
		{
			data->als_enable = 1;
			
			if(data->ps_enable == 1)
			{
				__ap3216c_write_reg(data->client, AP3216C_MODE_COMMAND,
				  AP3216C_MODE_MASK, AP3216C_MODE_SHIFT, 3);
			}
			else
			{
				__ap3216c_write_reg(data->client, AP3216C_MODE_COMMAND,
				  AP3216C_MODE_MASK, AP3216C_MODE_SHIFT, 1);
			}
		}
		//modify by yanfei for work 20140618
		cancel_delayed_work(&data->work_light);
		schedule_delayed_work(&data->work_light, msecs_to_jiffies(data->light_poll_delay));	
	}
	else
	{
		if(data->als_enable == 1)
		{
			data->als_enable = 0;

			if(data->ps_enable == 1)
			{
				__ap3216c_write_reg(data->client, AP3216C_MODE_COMMAND,
				  AP3216C_MODE_MASK, AP3216C_MODE_SHIFT, 2);
			}
			else
			{
				__ap3216c_write_reg(data->client, AP3216C_MODE_COMMAND,
				  AP3216C_MODE_MASK, AP3216C_MODE_SHIFT, 0);
			}
		}	
		//modify by yanfei for work 20140618
		cancel_delayed_work(&data->work_light);
	}
	return 0;
}
static int ap3216c_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct ap3216c_data *data = container_of(sensors_cdev,
			struct ap3216c_data, als_cdev);

	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	return ap3216c_enable_als_sensor(data->client, enable);
}
static ssize_t ap3216c_store_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int err = -1;
	unsigned long val = simple_strtoul(buf, NULL, 10);

	printk(KERN_ERR "ap3216c enable ALS sensor -> %ld\n", val);

	if ((val != 0) && (val != 1))
	{
		printk("%s: enable als sensor=%ld\n", __func__, val);
		return -EINVAL;
	}
	err = ap3216c_enable_als_sensor(client, val);
	
	return count;
}
//added by yanfei for ap321xx(ql1001) 2014-06-23 end
static DEVICE_ATTR(enable_als_sensor,  S_IRUGO | S_IWUGO ,
				  ap3216c_show_enable_als_sensor, ap3216c_store_enable_als_sensor);
#ifdef LSC_DBG
/* engineer mode */
static ssize_t ap3216c_em_read(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ap3216c_data *data = i2c_get_clientdata(client);
	int i;
	u8 tmp;
	
	for (i = 0; i < AP3216C_NUM_CACHABLE_REGS; i++)
	{
		mutex_lock(&data->lock);
		tmp = i2c_smbus_read_byte_data(data->client, ap3216c_reg[i]);
		mutex_unlock(&data->lock);

		printk("Reg[0x%x] Val[0x%x]\n", ap3216c_reg[i], tmp);
	}

	return 0;
}

static ssize_t ap3216c_em_write(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ap3216c_data *data = i2c_get_clientdata(client);
	u32 addr,val,idx=0;
	int ret = 0;

	sscanf(buf, "%x%x", &addr, &val);

	printk("Write [%x] to Reg[%x]...\n",val,addr);
	mutex_lock(&data->lock);

	ret = i2c_smbus_write_byte_data(data->client, addr, val);
	ADD_TO_IDX(addr,idx)
	if (!ret)
		data->reg_cache[idx] = val;

	mutex_unlock(&data->lock);

	return count;
}
static DEVICE_ATTR(em, S_IWUSR |S_IRUGO,
				   ap3216c_em_read, ap3216c_em_write);
#endif

static struct attribute *ap3216c_als_attributes[] = {
	&dev_attr_range.attr,
	&dev_attr_lux.attr,
	&dev_attr_althres.attr,
	&dev_attr_ahthres.attr,
	&dev_attr_calibration.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_enable_als_sensor.attr,
#ifdef LSC_DBG
	&dev_attr_em.attr,
#endif
	NULL
};

static const struct attribute_group ap3216c_als_attr_group = {
	.attrs = ap3216c_als_attributes,
};

static struct attribute *ap3216c_ps_attributes[] = {
	&dev_attr_object.attr,
	&dev_attr_pxvalue.attr,
	&dev_attr_plthres.attr,
	&dev_attr_phthres.attr,
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_ps_cal.attr,
	
	NULL
};

static const struct attribute_group ap3216c_ps_attr_group = {
	.attrs = ap3216c_ps_attributes,
};

static int ap3216c_init_client(struct i2c_client *client)
{
	struct ap3216c_data *data = i2c_get_clientdata(client);
	int i;
	int ret = 0;

//deleted by litao for ap321xx (qw601) 2014-01-14
//	int Pthval;

       /*fix bug:running test,reboot failed*/
       ap3216c_sw_reset(client);

	/* read all the registers once to fill the cache.
	 * if one of the reads fails, we consider the init failed */
	for (i = 0; i < AP3216C_NUM_CACHABLE_REGS; i++) {
		int v = i2c_smbus_read_byte_data(client, ap3216c_reg[i]);
		if (v < 0)
		{
			pr_info("shengweiguang: AP321XX EEE111111111!!\n");
			return -ENODEV;
		}
		data->reg_cache[i] = v;
	}
/* add by shengweiguang begin */
	if (data->reg_cache[AP3426_INT_CONTROL] == 0x88)
	{
		qrt_chip_type = AP3426;
		pr_err(" AP321XX : chip type: AP3426 ! ! ! \n");
	}
	else
	{
		qrt_chip_type = AP321XX;
		pr_err(" AP321XX : chip type: AP321XX ! ! ! \n");
	}
/* add by shengweiguang end */
pr_info("shengweiguang: AP321XX EEE2222222!!\n");

	ret = i2c_smbus_write_byte_data(client,AP3216C_RAN_COMMAND,0x30);
	if(ret<0)
	{
		printk(KERN_ERR"write AP3216C_RAN_COMMAND reg fail!!!!\n");
	}
pr_info("shengweiguang: AP321XX EEE333333333!!\n");
	ret = i2c_smbus_write_byte_data(client,AP3216C_ALS_CALIBRATION,0x40);
	if(ret<0)
	{
		printk(KERN_ERR"write AP3216C_ALS_CALIBRATION reg fail!!!!\n");
	}
pr_info("shengweiguang: AP321XX EEE44444444!!\n");
       //modified by yanfei  for ap321xx  (ql1001) 2014-06-27 begin
	ret = i2c_smbus_write_byte_data(client,AP3216C_PX_CONFIGURE,0x19);
	if(ret<0)
	{
		printk(KERN_ERR"write AP3216C_PX_CONFIGURE reg fail!!!!\n");
	}
	pr_info("shengweiguang: AP321XX EEE55555555555!!\n");
	ret = i2c_smbus_write_byte_data(client,AP3216C_PX_LED_CONTROL,0x22);
	if(ret<0)
	{
		printk(KERN_ERR"write AP3216C_PX_LED_CONTROL reg fail!!!!\n");
	}
	pr_info("shengweiguang: AP321XX EEE666666666!!\n");
	//modified by yanfei  for ap321xx  (ql1001) 2014-06-27 end
#if 1
       ret = i2c_smbus_write_byte_data(client,0x02,1);
	if(ret<0)
	{
		printk(KERN_ERR"write AP3216C_PX_CONFIGURE reg fail!!!!\n");
	}
#endif
pr_info("shengweiguang: AP321XX EEE777777777777!!\n");
	//modified by yanfei  for ap321xx  (ql1001) threshold  2014-06-27 
	//delete by yanfei  for ap321xx  (ql1001) threshold  2014-06-27 
	 //data->ps_thres_high = 200;
/* add by shengweiguang begin */
	if (qrt_chip_type == AP3426)
		ret = ap3426_set_phthres(data->client, data->ps_thres_high);
	else
/* add by shengweiguang end */	
		ret = ap3216c_set_phthres(data->client, data->ps_thres_high);
	if(ret<0)
	{
		printk(KERN_ERR"write ap3216c_set_phthres  fail!!!!\n");
	}
	//modified by yanfei  for ap321xx  (ql1001) threshold 2014-06-27 
	//delete by yanfei  for ap321xx  (ql1001) threshold  2014-06-27 
       //data->ps_thres_low = 160;
pr_info("shengweiguang: AP321XX EEE888888888!!\n");
/* add by shengweiguang begin */
	if (qrt_chip_type == AP3426)
		ret = ap3426_set_plthres(data->client, data->ps_thres_low);
	else
/* add by shengweiguang end */	
		ret = ap3216c_set_plthres(data->client, data->ps_thres_low);
//modified by litao for ap321xx (qw601) 2014-01-14 end
	if(ret<0)
	{
		printk(KERN_ERR"write ap3216c_set_plthres  fail!!!!\n");
	}
	pr_info("shengweiguang: AP321XX EEE999999999!!\n");
//added by litao to select zone mode of proximity interrupt trigger (qw601) 2013-12-27 begin
#if defined(PS_INTERRUPT_ZONE_MODE)
	ret = i2c_smbus_write_byte_data(client,AP3216C_PS_INTERRUPT_MODE,0x0);
#else
	ret = i2c_smbus_write_byte_data(client,AP3216C_PS_INTERRUPT_MODE,0x01);
#endif
	if(ret<0)
	{
		printk(KERN_ERR"write AP3216C_PS_INTERRUPT_MODE reg fail!!!!\n");
	}
//added by litao to select zone mode of proximity interrupt trigger (qw601) 2013-12-27 end
	__ap3216c_write_reg(data->client, AP3216C_MODE_COMMAND,
				  AP3216C_MODE_MASK, AP3216C_MODE_SHIFT, 0);
	
	return 0;
}

static void ap3216c_work_func_proximity(struct work_struct *work)
{

	struct ap3216c_data *data = container_of(work, struct ap3216c_data, work_proximity.work);
	struct i2c_client *client=data->client;



	int Pval;
	int ret = 0;
       int int_stat;
//add by yanfei for null pointer 20140923 beign 
	if((client==NULL)||(data==NULL))
		{
		  if(client==NULL)
			printk("%s:client NULL pointer in here\n ", __func__);
		  else
			printk("%s:data  NULL pointer in here\n ", __func__);
		  return;
		}
//add by yanfei for null pointer 20140923 end 
	int_stat = ap3216c_get_intstat(client);

	// PX int
	if (int_stat & AP3216C_INT_PMASK)
	{

		printk("%s:AP3216C_INT_PMASK\n ", __func__);

/* add by shengweiguang begin */
		if (qrt_chip_type == AP3426)
			Pval = ap3426_get_px_value(client,0);
		else
/* add by shengweiguang end */
			Pval = ap3216c_get_px_value(client,0);
		
		if(Pval < 0)
		{
			printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
			return;
		}
		if(Pval==1023)
		{
			Pval = 800;
		}

		if(Pval >= data->ps_thres_high)
		{
			input_report_abs(data->proximity_input_dev, ABS_DISTANCE, 0);
			input_sync(data->proximity_input_dev);
//added by chenchen for pocket mode 20140925			
			proximity_open_flag = true;

			printk(KERN_ERR "%s: near_far_state is 0 \n", __func__);

/* add by shengweiguang begin */
			if (qrt_chip_type == AP3426)
				ap3426_set_phthres(client, PS_THRES_HIGH_MAX);
			else
/* add by shengweiguang end */	
				ap3216c_set_phthres(client, PS_THRES_HIGH_MAX);

/* add by shengweiguang begin */
			if (qrt_chip_type == AP3426)
				ap3426_set_plthres(client, data->ps_thres_low);
			else
/* add by shengweiguang end */	
				ap3216c_set_plthres(client, data->ps_thres_low);

		}

		else if(Pval <= data->ps_thres_low)
		{
			input_report_abs(data->proximity_input_dev, ABS_DISTANCE, 1);
			input_sync(data->proximity_input_dev);
//added by chenchen for pocket mode 20140925			
			proximity_open_flag = false;

			printk(KERN_ERR "%s: near_far_state is 1 \n", __func__);

/* add by shengweiguang begin */
			if (qrt_chip_type == AP3426)
				ap3426_set_phthres(client, data->ps_thres_high);
			else
/* add by shengweiguang end */	
				ap3216c_set_phthres(client, data->ps_thres_high);
			
/* add by shengweiguang begin */
			if (qrt_chip_type == AP3426)
				ap3426_set_plthres(client, 0);
			else
/* add by shengweiguang end */				
				ap3216c_set_plthres(client, 0);

		}

		else
		{
			input_report_abs(data->proximity_input_dev, ABS_DISTANCE, 1);
			input_sync(data->proximity_input_dev);

			printk(KERN_ERR "%s: near_far_state is 1 \n", __func__);
		}

	}
	else
	{
		printk(KERN_ERR "%s: unknown interrupt source.\n", __func__); 
	}
	ret = i2c_smbus_write_byte_data(client,AP3216C_INT_COMMAND,0x02);
	if(ret<0)
	{
		printk(KERN_ERR"AP3216C clear interrupt statu flag fail!!!!\n");
	}
	enable_irq(client->irq);
//delete irq wake by yanfei 
	enable_irq_wake(client->irq);
}

static void ap3216c_work_func_light(struct work_struct *work)
{
	struct ap3216c_data *data = container_of(work, struct ap3216c_data, work_light.work);
	int Aval;
	
	mutex_lock(&data->lock);
  
	Aval = ap3216c_get_adc_value(data->client,1);
	//printk("ALS lux value: %u\n", Aval);
	
	mutex_unlock(&data->lock);
	
	input_report_abs(data->light_input_dev, ABS_MISC, Aval);
	input_sync(data->light_input_dev);
	schedule_delayed_work(&data->work_light, msecs_to_jiffies(data->light_poll_delay));	
}


static int ap3216c_input_init(struct ap3216c_data *data)
{
    struct input_dev *input_dev;
    int ret;

    /* allocate proximity input_device */
    input_dev = input_allocate_device();
    if (!input_dev) {
        LDBG("could not allocate input device\n");
        goto err_pxy_all;
    }
    data->proximity_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "proximity";
    input_set_capability(input_dev, EV_ABS, ABS_DISTANCE);
    input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

    LDBG("registering proximity input device\n");
    ret = input_register_device(input_dev);
    if (ret < 0) {
        LDBG("could not register input device\n");
        goto err_pxy_reg;;
    }

    ret = sysfs_create_group(&input_dev->dev.kobj,
                 &ap3216c_ps_attr_group);
    if (ret) {
        LDBG("could not create sysfs group\n");
        goto err_pxy_sys;;
    }

    /* allocate light input_device */
    input_dev = input_allocate_device();
    if (!input_dev) {
        LDBG("could not allocate input device\n");
        goto err_light_all;
    }
    input_set_drvdata(input_dev, data);
    input_dev->name = "light";
    input_set_capability(input_dev, EV_ABS, ABS_MISC);
    input_set_abs_params(input_dev, ABS_MISC, 0, 1, 0, 0);

    LDBG("registering light sensor input device\n");
    ret = input_register_device(input_dev);
    if (ret < 0) {
        LDBG("could not register input device\n");
        goto err_light_reg;
    }
    data->light_input_dev = input_dev;
    ret = sysfs_create_group(&input_dev->dev.kobj, &ap3216c_als_attr_group);
    if (ret) {
        LDBG("could not create sysfs group\n");
        goto err_light_sys;
    }

    return 0;

err_light_sys:
    input_unregister_device(data->light_input_dev);
err_light_reg:
    input_free_device(input_dev);
err_light_all:
    sysfs_remove_group(&data->proximity_input_dev->dev.kobj,
			   &ap3216c_ps_attr_group);
err_pxy_sys:
    input_unregister_device(data->proximity_input_dev);
err_pxy_reg:
    input_free_device(input_dev);
err_pxy_all:

    return (-1);   
}

static void ap3216c_input_fini(struct ap3216c_data *data)
{
    struct input_dev *dev = data->light_input_dev;

    input_unregister_device(dev);
    input_free_device(dev);
    
    dev = data->proximity_input_dev;
    input_unregister_device(dev);
    input_free_device(dev);
}

/*
 * I2C layer
 */

static irqreturn_t ap3216c_irq(int irq, void *data_)
{

	 struct i2c_client *client=(struct i2c_client *)data_;
	struct ap3216c_data *data = i2c_get_clientdata(client);
//add by yanfei for null pointer 20140923 beign 
	if((client==NULL)||(data==NULL))
		{
		  if(client==NULL)
			printk("%s:client NULL pointer in here\n ", __func__);
		  else
			printk("%s:data  NULL pointer in here\n ", __func__);
		  return -EINVAL;
		}
//add by yanfei for null pointer 20140923 end 
	disable_irq_nosync(client->irq);
//delete irq wake by yanfei for ql1006 20140829
    	disable_irq_wake(client->irq);
//modify for avoiding system go to sleep 20141011
    	wake_lock_timeout(&data->prx_wake_lock, 5*HZ);

//add by yanfei for wake lock 20140828
	schedule_delayed_work(&data->work_proximity, 0);


	return IRQ_HANDLED;
}


#define AP3216C_VDD_MIN_UV	2000000
#define AP3216C_VDD_MAX_UV	3300000
#define AP3216C_VIO_MIN_UV 	1750000
#define AP3216C_VIO_MAX_UV	1950000
static int ap3216c_power_on_off(struct ap3216c_data *data, bool on)
{
	int rc = 0;
//added by yanfei  for ap321xx pinctrl (ql1001) 2014-06-27 begin
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
		}
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
			rc = regulator_disable(data->vdd);
		}
	}

	msleep(10);
//added by yanfei  for ap321xx pinctrl (ql1001) 2014-06-27 end

	return rc;
}

static int ap3216c_power_init(struct ap3216c_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0, AP3216C_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0, AP3216C_VIO_MAX_UV);

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
			rc = regulator_set_voltage(data->vdd, AP3216C_VDD_MIN_UV,
						   AP3216C_VDD_MAX_UV);
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
			rc = regulator_set_voltage(data->vio, AP3216C_VIO_MIN_UV,
						   AP3216C_VIO_MAX_UV);
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
		regulator_set_voltage(data->vdd, 0, AP3216C_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

static struct i2c_driver ap3216c_driver;
//added by yanfei  for ap321xx pinctrl (ql1001) 2014-06-27 begin
static int sensor_platform_hw_power_on(bool on)
{
	int err;

	if (pdev_data == NULL)
		return -ENODEV;

	if (!IS_ERR_OR_NULL(pdev_data->pinctrl)) {
		if (on)
			err = pinctrl_select_state(pdev_data->pinctrl,
				pdev_data->pin_default);
		else
			err = pinctrl_select_state(pdev_data->pinctrl,
				pdev_data->pin_sleep);
		if (err)
			dev_err(&pdev_data->client->dev,
				"Can't select pinctrl state\n");
	}
	ap3216c_power_on_off(pdev_data, on);

	return 0;
}
static int ap321xx_pinctrl_init(struct ap3216c_data *data)
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
//added by yanfei  for ap321xx pinctrl (ql1001) 2014-06-27 end
#ifdef 	CONFIG_OF
static int ap3216c_parse_dt(struct device *dev,
				 struct ap3216c_data *data)
{
	struct device_node *np = dev->of_node;
	int rc;
	u32 temp_val;
//added by yanfei  for ap321xx pinctrl (ql1001) 2014-06-27
	data->power_on = sensor_platform_hw_power_on;
	data->gpio_int = of_get_named_gpio(np,"qcom,irq-gpio", 0);
	printk(KERN_ERR "ap3216c irq gpio is %d.\n",data->gpio_int);
//added by yanfei  for ap321xx  (ql1001) 2014-07-01 begin	
	rc = of_property_read_u32(np, "liteon,ps-thdh", &temp_val);
	if (!rc)
		data->ps_thres_high= (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdh\n");
		return rc;
	}

	rc = of_property_read_u32(np, "liteon,ps-thdl", &temp_val);
	if (!rc)
		data->ps_thres_low= (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdl\n");
		return rc;
	}
	rc = of_property_read_u32(np, "liteon,ps-high", &temp_val);
	if (!rc)
		data->ps_high= (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdh\n");
		return rc;
	}

	rc = of_property_read_u32(np, "liteon,ps-low", &temp_val);
	if (!rc)
		data->ps_low= (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdl\n");
		return rc;
	}
//added by yanfei  for ap321xx  (ql1001) 2014-07-01 end	
	return 0;
}
#endif

static struct of_device_id ap321x_match_table[];

//static int __devinit ap3216c_probe(struct i2c_client *client,
static int  ap3216c_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct ap3216c_data *data;
	int err = 0;
	int temp = 0;
//delete log by yanfei 20140828
	pr_info("shengweiguang: AP321XX !!!!!!!!!!!!!!!!!\n");
	printk(KERN_ERR "%s: %d \n", __func__,__LINE__);
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	data = kzalloc(sizeof(struct ap3216c_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
//modified by yanfei  for ap321xx pinctrl (ql1001) 2014-06-27 begin
	pdev_data = data;
	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);
	wake_lock_init(&data->prx_wake_lock, WAKE_LOCK_SUSPEND, "prx_wake_lock");
//modified by yanfei  for ap321xx pinctrl (ql1001) 2014-06-27 end
pr_info("shengweiguang: AP321XX 222222222222222222!!\n");
	err = ap3216c_power_init(data, true);
	if (err < 0) {
		dev_err(&client->dev, "power init failed! err=%d", err);
		goto exit_kfree;
	}
pr_info("shengweiguang: AP321XX 333333333333333!!\n");
	err = ap3216c_power_on_off(data, true);
	if (err < 0) {
		dev_err(&client->dev, "power on failed! err=%d\n", err);
		ap3216c_power_init(data, false);
		goto exit_kfree;
	}
	pr_info("shengweiguang: AP321XX 444444444!!\n");
	/* initialize the AP3216C chip */
	err = ap3216c_init_client(client);
	if (err)
		goto exit_kfree;
pr_info("shengweiguang: AP321XX 55555555555555!!\n");
	err = ap3216c_input_init(data);
	if (err)
		goto exit_kfree;
pr_info("shengweiguang: AP321XX 6666666666666!!\n");
	temp = ap3216c_parse_dt(&client->dev, data);
//added by yanfei  for ap321xx pinctrl (ql1001) 2014-06-27 begin
	/* initialize pinctrl */
	err = ap321xx_pinctrl_init(data);
	if (err) {
		dev_err(&client->dev, "Can't initialize pinctrl\n");
		goto exit_kfree;
		}
	err = pinctrl_select_state(data->pinctrl, data->pin_default);
	if (err) {
		dev_err(&client->dev,
			"Can't select pinctrl default state\n");
		goto exit_kfree;
	}
	if (temp){
		goto exit_kfree;
	}
	else
	{

		if(gpio_is_valid(data->gpio_int))
		{

			err = gpio_request(data->gpio_int,"ap3216c_gpio_int");

			if(err)
			{
				pr_err("%s: unable to request interrupt gpio %d\n",__func__,data->gpio_int);
				goto exit_kfree;
			}

			err = gpio_direction_input(data->gpio_int);
			if (err) {
				pr_err("%s: unable to set direction for gpio %d\n",__func__, data->gpio_int);

				gpio_free(data->gpio_int);
				goto exit_kfree;
			}
			client->irq = gpio_to_irq(data->gpio_int);
			printk(KERN_ERR "ap3216c irq is %d.\n",client->irq);
		}
		else
		{
			err = -EINVAL;
			goto exit_kfree;
		}
	}

	pr_info("shengweiguang: AP321XX 77777777777!!\n");
 	if (data->power_on)
		err = data->power_on(true);
	input_report_abs(data->proximity_input_dev, ABS_DISTANCE, 1);
	input_sync(data->proximity_input_dev);
//added by yanfei for ap3216 20140708 end
//added by yanfei  for ap321xx pinctrl (ql1001) 2014-06-27 end
	err = request_irq(client->irq,ap3216c_irq, IRQF_TRIGGER_FALLING,
			 "ap3216c",(void *)client );


	if (err) {
		dev_err(&client->dev, "ret: %d, could not get IRQ %d\n",err,client->irq);
			goto exit_irq;
	}
pr_info("shengweiguang: AP321XX 8888888888888!!\n");
	disable_irq(client->irq);  
	data->light_poll_delay=ALS_SET_DEFAULT_DELAY_TIME;
	
	INIT_DELAYED_WORK(&data->work_proximity, ap3216c_work_func_proximity);
	INIT_DELAYED_WORK(&data->work_light, ap3216c_work_func_light);  

	dev_info(&client->dev, "Driver version %s enabled\n", DRIVER_VERSION);
		/* Register to sensors class */
//added by yanfei for ap321xx(ql1001) 2014-06-23 begin
	data->als_cdev = sensors_light_cdev;
	data->als_cdev.sensors_enable = ap3216c_als_set_enable;
	data->als_cdev.sensors_poll_delay = ap3216c_als_poll_delay;
	data->ps_cdev = sensors_proximity_cdev;
	data->ps_cdev.sensors_enable = ap3216c_ps_set_enable;
	data->ps_cdev.sensors_get_crosstalk = ap3216c_get_ps_crosstalk;
	data->ps_cdev.sensors_poll_delay = NULL;
	err = sensors_classdev_register(&client->dev, &data->als_cdev);
	if (err) {
		pr_err("%s: Unable to register to sensors class: %d\n",
				__func__, err);
		goto exit_kfree;
	}
pr_info("shengweiguang: AP321XX 999999999999!!\n");
	err = sensors_classdev_register(&client->dev, &data->ps_cdev);
	if (err) {
		pr_err("%s: Unable to register to sensors class: %d\n",
			       __func__, err);
		goto exit_kfree;
	}
//add by yanfei for sensor hardware information 20140711 begin
#ifdef CONFIG_GET_HARDWARE_INFO
	register_hardware_info(ALS_PS, ap321x_match_table[0].compatible);
#endif
//add by yanfei for sensor hardware information 20140711 end
//added by yanfei for ap321xx(ql1001) 2014-06-23 end
	return 0;

exit_irq:
	ap3216c_input_fini(data);

exit_kfree:
	wake_lock_destroy(&data->prx_wake_lock);
	mutex_destroy(&data->lock);
	if (data->power_on)
		data->power_on(false);
	kfree(data);
//added by yanfei  for ap321xx pinctrl (ql1001) 2014-06-27 
	pdev_data = NULL;
	return err;
}
//added by yanfei  for ap321xx  (ql1001) 2014-06-27 begin
//modify by yanfei for suspend and resume 20140717 begin
static int ap3216x_suspend(struct device *dev)
{
//added by yanfei  for ap321xx  (ql1001) 2014-06-27 begin
	struct ap3216c_data *data = dev_get_drvdata(dev);
	if (data->als_enable)
		{
			cancel_delayed_work_sync(&data->work_light);
		}
//added by yanfei  for ap321xx  (ql1001) 2014-06-27 end
	return 0;
}

static int ap3216x_resume(struct device *dev)
{
//added by yanfei  for ap321xx  (ql1001) 2014-06-27 begin
	struct ap3216c_data *data = dev_get_drvdata(dev);
	if (data->als_enable)
		{
			schedule_delayed_work(&data->work_light, msecs_to_jiffies(data->light_poll_delay));
		}
//added by yanfei  for ap321xx  (ql1001) 2014-06-27 end
	return 0;
}
//modify by yanfei for suspend and resume 20140717 begin
//added by yanfei  for ap321xx  (ql1001) 2014-06-27 end
static int  ap3216c_remove(struct i2c_client *client)	
{	
	struct ap3216c_data *data = i2c_get_clientdata(client);
	

	free_irq(client->irq, data);
	sysfs_remove_group(&data->light_input_dev->dev.kobj, &ap3216c_als_attr_group);
	input_unregister_device(data->light_input_dev);
	sysfs_remove_group(&data->proximity_input_dev->dev.kobj, &ap3216c_ps_attr_group);
	input_unregister_device(data->proximity_input_dev);
	mutex_destroy(&data->lock);
	wake_lock_destroy(&data->prx_wake_lock);
	if (data->power_on)
		data->power_on(false);
	kfree(data);
//added by yanfei  for ap321xx pinctrl (ql1001) 2014-06-27 
	pdev_data = NULL;
	return 0;
}
//added by yanfei  for ap321xx  (ql1001) 2014-06-27 begin
#ifdef CONFIG_OF
static struct of_device_id ap321x_match_table[] = {
	{ .compatible = "liteon,ap321xx", },
	{ },
};
#endif /* CONFIG_OF */

static const struct i2c_device_id ap3216c_id[] = {
	{ "ap321x", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ap3216c_id);
//modify by yanfei for suspend and resume 20140717 begin
static const struct dev_pm_ops ap321xx_pm_ops = {
	.suspend	= ap3216x_suspend,
	.resume 	= ap3216x_resume,
};
static struct i2c_driver ap3216c_driver = {
	.driver = {
		.name	= AP3216C_DRV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = ap321x_match_table,
#endif /* CONFIG_OF */
		.pm = &ap321xx_pm_ops,
	},
	.probe	= ap3216c_probe,
	.remove	= ap3216c_remove,
//	.suspend  = ap3216x_suspend,
//	.resume 	= ap3216x_resume,
	//.remove	= __devexit_p(ap3216c_remove),
	.id_table = ap3216c_id,
};
//added by yanfei  for ap321xx  (ql1001) 2014-06-27  end
static int __init ap3216c_init(void)
{
	return i2c_add_driver(&ap3216c_driver);
}

static void __exit ap3216c_exit(void)
{
	i2c_del_driver(&ap3216c_driver);
}

MODULE_AUTHOR("YC Hou, LiteOn-semi corporation.");
MODULE_DESCRIPTION("Test AP3216C, AP3212C and AP3216C driver on mini6410.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

late_initcall(ap3216c_init);
module_exit(ap3216c_exit);
