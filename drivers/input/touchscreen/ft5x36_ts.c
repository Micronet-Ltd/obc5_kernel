/* drivers/input/touchscreen/focaltech_ts.c
 *
 * FocalTech Serils IC TouchScreen driver.
 *  
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
//#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
//#include <mach/irqs.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include "ft5x36_ts.h"
#include "ft5x36_info.h"
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

/* Added by zengguang to add head file. 2013.5.5 start*/
#ifdef CONFIG_GET_HARDWARE_INFO
#include <mach/hardware_info.h>
static char tmp_tp_name[100];
#endif
/*  Added by zengguang to add head file. 2013.5.5 end*/

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FT_SUSPEND_LEVEL 1
#endif
#define dev_info(dev, fmt, arg...) _dev_info(dev, fmt, ##arg)
#ifdef FTS_CTL_IIC
#include "ft5x36_ctl.h"
#endif
#ifdef FTS_SYSFS_DEBUG
#include "ft5x36_ex.h"
#endif

/*added by jiao.shp for tp gesture in 20141113 START*/
#if 0//def pr_debug
#undef pr_debug
#define pr_debug pr_err
#endif

#include <linux/wakelock.h>
struct wake_lock tp_gesture_wakelock;
struct i2c_client *ft5x46_i2c_client;
#define FTS_GESTURE_POINTS 30
bool tp_gesture_enable = false ;
bool tp_gesture_support = false;
bool tp_glove_mode_enable = false;
short pointnum = 0;
int gesture_id = 0;
/*added by jiao.shp for tp gesture in 20141113 END*/

//static struct mutex ft5x36_mutex;/*Added by liumx for tp no effect 2013.10.9 start*/

//added by chenchen for proximity function 20140828 begin
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
#include <linux/input-polldev.h>
#include <linux/wakelock.h>

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "proximity",
	.vendor = "focaltech",
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
};
#endif
//added by chenchen for proximity function 20140828 end

//modified for up last point by chenchen 2014.8.22 begin
static u8 last_touchpoint=0;
static u8 now_touchpoint=0;
//modified for up last point by chenchen 2014.8.22 end
#define FTS_POINT_UP		0x01
#define FTS_POINT_DOWN		0x00
#define FTS_POINT_CONTACT	0x02

//added by chenchen for proximity function 20140828 begin
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
#define PROXIMITY_INPUT_DEV_NAME 	"proximity"
#define SENSOR_PROX_TP_USE_WAKELOCK
static struct i2c_client * i2c_prox_client = NULL;
#ifdef SENSOR_PROX_TP_USE_WAKELOCK
static struct wake_lock sensor_prox_tp_wake_lock;
#endif
static DEFINE_MUTEX(tp_prox_sensor_mutex);

static int tp_prox_sensor_opened;
static char tp_prox_sensor_data = 0; // 0 near 1 far
static char tp_prox_sensor_data_changed = 0;
static char tp_prox_pre_sensor_data = 1;

static int is_suspend = 0;
static int is_need_report_pointer = 1;

static void tp_prox_sensor_enable(int enable);
#endif
//added by chenchen for proximity function 20140828 end

/*
*fts_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
struct kobject *ft5x36_virtual_key_properties_kobj;
static ssize_t
ft5x36_virtual_keys_register(struct kobject *kobj,
                            struct kobj_attribute *attr,
                            char *buf)
{
       return snprintf(buf, 200,
       __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":160:1344:170:40"
       ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)   ":360:1344:170:40"
       ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":570:1344:170:40"
       "\n");
}

static struct kobj_attribute ft5x36_virtual_keys_attr = {
       .attr = {
               .name = "virtualkeys.FT5x36",
               .mode = S_IRUGO,
       },
       .show = &ft5x36_virtual_keys_register,
};

static struct attribute *ft5x36_virtual_key_properties_attrs[] = {
       &ft5x36_virtual_keys_attr.attr,
       NULL,
};

static struct attribute_group ft5x36_virtual_key_properties_attr_group = {
       .attrs = ft5x36_virtual_key_properties_attrs,
};

int fts_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
int fts_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);

	return ret;
}
#if 0
/*release the point*/
static void fts_ts_release(struct fts_ts_data *data)
{
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_sync(data->input_dev);
}
#endif


/*Added by jiao.shp in 20141014 START*/
#if defined(CONFIG_QL1001_P2170) || defined(CONFIG_QL1001_J20) || defined(CONFIG_QL1001_J20TMP)||defined(CONFIG_QL1001_P2170FHD) 
static int zero_point_rejetc_check(struct fts_ts_data *data,u8 *buf)
{
    struct ts_event *event = &data->event;
    s16 fts_pos_x = 0;
    s16 fts_pos_y = 0;
    int i = 0;
    int j = 0;
    
    for(i=0;i<event->touch_point;i++){
        fts_pos_x = (s16) (buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
        8 | (s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i];
        fts_pos_y = (s16) (buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
        8 | (s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i];
        if(fts_pos_x == 0 && fts_pos_y == 0){
            for(j=i;j<event->touch_point;j++){
                event->au16_x[j] = event->au16_x[j+1];
                event->au16_y[j] = event->au16_y[j+1];
                event->au8_touch_event[j] = event->au8_touch_event[j+1];
                event->au8_finger_id[j] = event->au8_finger_id[j+1];
            }
            event->touch_point--;
            last_touchpoint--;
        }
    } 
    
    return 0;
}
#endif
/*Added by jiao.shp in 20141014 END*/

/*added by jiao.shp for tp gesture in 20141113 START*/
static int fts_gesture_id_report(struct fts_ts_data *data)
{

    pr_debug("%s,gesture_id is 0x%x..\n",__func__,gesture_id);
    input_report_key(data->input_dev, gesture_id, 1);
    input_sync(data->input_dev);
    input_report_key(data->input_dev, gesture_id, 0);
    input_sync(data->input_dev);

	return 0;
}

static int fts_read_gesture_id(struct fts_ts_data *data)
{
	unsigned char buf[FTS_GESTURE_POINTS * 2] = { 0 };   
    struct ts_event *event = &data->event;
	int ret = -1;    
	int i = 0;    
	buf[0] = 0xd3; 

    pr_debug("%s..\n",__func__);
    wake_lock_timeout(&tp_gesture_wakelock,HZ*2);
	ret = fts_i2c_Read(data->client, buf, 1, buf, 8);    
	if (ret < 0) 
	{        
		dev_err(&data->client->dev,"%s read touchdata failed.\n", __func__);        
		return ret;    
	} 

	if (0x24 == buf[0])
	{        
		gesture_id = 0x24;
		pr_debug("The Gesture ID is %x \n",gesture_id);
		return 0;   
	}else{
        gesture_id = (int)(buf[0]);
        pr_debug("The Gesture ID is %x \n",gesture_id);
		return 0;  
    }
    
	pointnum = (short)(buf[1]) & 0xff;
	buf[0] = 0xd3;
	ret = fts_i2c_Read(data->client, buf, 1, buf, (pointnum * 4 + 8));    
	if (ret < 0) 
	{        
		dev_err(&data->client->dev,"%s read touchdata failed.\n", __func__);        
		return ret;    
	}
    
	pr_debug("The Gesture ID is 0x%x, Pointnum is 0x%x \n",gesture_id,pointnum);    
	for(i = 0;i < pointnum;i++)
	{
		event->au16_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[1 + (4 * i)])& 0xFF);
		event->au16_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
	 } 
     
	 return 0;
}
/*added by jiao.shp for tp gesture in 20141113 END*/

/*Read touch point information when the interrupt  is asserted.*/
static int fts_read_Touchdata(struct fts_ts_data *data)
{
	struct ts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
//add for error point begin by chenchen 20140827	
	int bufnum=0;
	u8 pointid = FT_MAX_ID;
//added by chenchen for proximity function 20140828 begin
	#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	u8 state;
	u8 proximity_status;
	#endif

	#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	if (tp_prox_sensor_opened == 1)
	{
		i2c_smbus_read_i2c_block_data(data->client, 0xB0, 1, &state);
		//TPD_DEBUG("proxi_5206 0xB0 state value is 1131 0x%02X\n", state);
		if(!(state&0x01))
		{
			tp_prox_sensor_enable(1);
		}
		i2c_smbus_read_i2c_block_data(data->client, 0x01, 1, &proximity_status);
		//TPD_DEBUG("proxi_5206 0x01 value is 1139 0x%02X\n", proximity_status);
		tp_prox_pre_sensor_data = tp_prox_sensor_data;
		if (proximity_status == 0xC0)  //near
		{
			tp_prox_sensor_data = 0;	
		}
		else if(proximity_status == 0xE0)  //far
		{
			tp_prox_sensor_data = 1;
		}
		 //TPD_DEBUG( "%s tp_prox_pre_sensor_data=%d,tp_prox_sensor_data=%d\n", __func__,tp_prox_pre_sensor_data,tp_prox_sensor_data);
	 	if( tp_prox_pre_sensor_data != tp_prox_sensor_data)
	 	{  
		    printk( "%s ensor data changed\n", __func__);
			mutex_lock(&tp_prox_sensor_mutex);
	        tp_prox_sensor_data_changed = 1;
	        mutex_unlock(&tp_prox_sensor_mutex);
			return 1;
	 	}
		if(is_need_report_pointer == 0)
		{
			printk( ":%s:  we don not report pointer when sleep in call\n", __func__);
			return 1;
		}		
	}  
	#endif
//added by chenchen for proximity function 20140828 end

	ret = fts_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);
//add for error point by chenchen 20140827 begin
	 for(i=0;i<5;i++)
       {
           if(buf[i]==0)
           {
		   	bufnum++;
           }
	 }
	 if(bufnum>=5)
	 	return -1;
	 
//add for error point by chenchen 20140827 end
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));

	//event->touch_point = buf[2] & 0x0F;
//modified for up last point by chenchen 2014.8.22	
    now_touchpoint = buf[2] & 0x0F;
	event->touch_point = 0;
	for (i = 0; i < data->pdata->max_touch_point; i++) {
	//for (i = 0; i < event->touch_point; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		else
			event->touch_point++;
//modified for up last point by chenchen 2014.8.22		
		 	last_touchpoint=event->touch_point; 
		event->au16_x[i] =
		    (s16) (buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i];
		event->au16_y[i] =
		    (s16) (buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i];
		event->au8_touch_event[i] =
		    buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
		    (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
	}
	//Added by jiao.shp
	#if defined(CONFIG_QL1001_P2170) || defined(CONFIG_QL1001_J20) || defined(CONFIG_QL1001_J20TMP)||defined(CONFIG_QL1001_P2170FHD) 
    zero_point_rejetc_check(data,buf);
	#endif
	
	event->pressure = FT_PRESS;

	return 0;
}

/*
*report the point information
*/
static void fts_report_value(struct fts_ts_data *data)
{
	struct ts_event *event = &data->event;
	//struct fts_platform_data *pdata = &data->pdata;
	int i = 0;
	//int up_point = 0;
	//int touch_point = 0;
	int fingerdown = 0;

	for (i = 0; i < event->touch_point; i++) {
		if (event->au8_touch_event[i] == 0 || event->au8_touch_event[i] == 2) {
			event->pressure = FT_PRESS;
			fingerdown++;
		} else {
			event->pressure = 0;
		}
		input_mt_slot(data->input_dev, event->au8_finger_id[i]);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,
						!!event->pressure);
		if (event->pressure == FT_PRESS) {
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					event->au16_y[i]);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					event->pressure);
		}
	//printk(KERN_INFO "----------------X=%d.Y=%d\n",event->au16_x[i],event->au16_y[i]);
	}
	input_report_key(data->input_dev, BTN_TOUCH, !!fingerdown);
	input_sync(data->input_dev);

//modified for up last point by chenchen 20140822 begin	
	if((last_touchpoint>0)&&(now_touchpoint==0))    
    {        /* release all touches */ 
              i=0;
         	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) 
	 		{
                   input_mt_slot(data->input_dev, i);
                   input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
        	}
         	last_touchpoint=0;
		input_report_key(data->input_dev, BTN_TOUCH, !!fingerdown);
		input_sync(data->input_dev);
	}
//modified for up last point by chenchen 20140822 end	
}

/*The FocalTech device will signal the host about TRIGGER_FALLING.
*Processed when the interrupt is asserted.
*/

//added by chenchen for proximity function 20140828 begin
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR

static void tp_prox_sensor_enable(int enable)
{
	u8 state;
	int ret = -1;

    if(i2c_prox_client==NULL)
    	return;

	i2c_smbus_read_i2c_block_data(i2c_prox_client, 0xB0, 1, &state);
	printk("[proxi_5206]read: 999 0xb0's value is 0x%02X : %s\n", state, enable?"enable":"disable");
	if (enable){
		state |= 0x01;
	}else{
		state &= 0x00;	
	}
	ret = i2c_smbus_write_i2c_block_data(i2c_prox_client, 0xB0, 1, &state);
	if(ret < 0)
	{
		printk("[proxi_5206]write psensor switch command failed\n");
	}
	return;
}

static int tp_prox_set_enable(struct sensors_classdev *sensors_cdev,unsigned int enable)
{
	disable_irq(i2c_prox_client->irq);
	tp_prox_sensor_enable((int)enable);
	if(enable){
		#ifdef SENSOR_PROX_TP_USE_WAKELOCK
		wake_lock(&sensor_prox_tp_wake_lock);
		#endif
		tp_prox_sensor_opened = 1;
		tp_prox_sensor_data = 1;
		tp_prox_sensor_data_changed = 1;

	}else{
		tp_prox_sensor_opened = 0;
		#ifdef SENSOR_PROX_TP_USE_WAKELOCK
		wake_unlock(&sensor_prox_tp_wake_lock);
		#endif
	}
	enable_irq(i2c_prox_client->irq);
	return 0;
}


static ssize_t tp_prox_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 4, "%d\n", tp_prox_sensor_opened);
}

static ssize_t tp_prox_enable_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	//struct fts_ts_data *prox = i2c_get_clientdata(client);
	struct input_dev *input_dev = to_input_dev(dev);
	unsigned long data;
	int error;
	
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	printk("%s, data=%ld\n",__func__,data);
	mutex_lock(&input_dev->mutex);
	disable_irq(client->irq);
	mutex_lock(&tp_prox_sensor_mutex);
	tp_prox_sensor_enable((int)data);

	if(data){

		#ifdef SENSOR_PROX_TP_USE_WAKELOCK
		wake_lock(&sensor_prox_tp_wake_lock);
		#endif
		tp_prox_sensor_opened = 1;
		tp_prox_sensor_data = 1;
		tp_prox_sensor_data_changed = 1;

	}else{
		tp_prox_sensor_opened = 0;
		#ifdef SENSOR_PROX_TP_USE_WAKELOCK
		wake_unlock(&sensor_prox_tp_wake_lock);
		#endif
	}
	mutex_unlock(&tp_prox_sensor_mutex);
	enable_irq(client->irq);
	mutex_unlock(&input_dev->mutex);

	return count;
}

static DEVICE_ATTR(enable, 0777,
			tp_prox_enable_show, tp_prox_enable_store);

/* Returns currently selected poll interval (in ms) */

static ssize_t tp_prox_get_poll_delay(struct device *dev,

				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_data *ps = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", ps->input_poll_dev->poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t tp_prox_set_poll_delay(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_data *ps = i2c_get_clientdata(client);
	struct input_dev *input_dev = ps->input_prox_dev;
	unsigned int interval;
	int error;

	error = kstrtouint(buf, 10, &interval);
	if (error < 0)
		return error;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);
	disable_irq(client->irq);
	/*
	 * Set current interval to the greater of the minimum interval or
	 * the requested interval
	 */
	ps->input_poll_dev->poll_interval = max((int)interval,(int)ps->input_poll_dev->poll_interval_min);

	enable_irq(client->irq);
	mutex_unlock(&input_dev->mutex);

	return count;
}

static DEVICE_ATTR(poll_delay, 0664,
			tp_prox_get_poll_delay, tp_prox_set_poll_delay);

static struct attribute *tp_prox_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static struct attribute_group tp_prox_attribute_group = {
	.attrs = tp_prox_attributes
};
static void  tp_prox_init_input_device(struct fts_ts_data *prox,
					      struct input_dev *input_dev)
{
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	input_dev->name = PROXIMITY_INPUT_DEV_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &prox->client->dev;
}

static void tp_prox_poll(struct input_polled_dev *dev)
{
	struct fts_ts_data *prox = dev->private;

	if (tp_prox_sensor_data_changed){
		mutex_lock(&tp_prox_sensor_mutex);
		tp_prox_sensor_data_changed = 0;
		mutex_unlock(&tp_prox_sensor_mutex);
		printk("%s poll tp_prox_sensor_data=%d\n",__func__,tp_prox_sensor_data);
		input_report_abs(prox->input_prox_dev, ABS_DISTANCE, tp_prox_sensor_data);
		input_sync(prox->input_prox_dev);
	}
}

static int  tp_prox_setup_polled_device(struct fts_ts_data *ps)
{
	int err;
	struct input_polled_dev *poll_dev;

	poll_dev = input_allocate_polled_device();
	if (!poll_dev) {
		printk("Failed to allocate polled device\n");
		return -ENOMEM;
	}

	ps->input_poll_dev = poll_dev;
	ps->input_prox_dev = poll_dev->input;

	poll_dev->private = ps;
	poll_dev->poll = tp_prox_poll;
	//poll_dev->open = tp_prox_polled_input_open;
	//poll_dev->close = tp_prox_polled_input_close;
	poll_dev->poll_interval = 100;
	poll_dev->poll_interval_min= 0;

	tp_prox_init_input_device(ps, poll_dev->input);
	err = input_register_polled_device(poll_dev);
	printk("%s, err=%d, poll-interval=%d\n",__func__,err,poll_dev->poll_interval);
	if (err) {
		printk("Unable to register polled device, err=%d\n", err);
		input_free_polled_device(poll_dev);
		return -ENOMEM;
	}

	return 0;
}

static void tp_prox_teardown_polled_device(struct fts_ts_data *ps)
{
	input_unregister_polled_device(ps->input_poll_dev);
	input_free_polled_device(ps->input_poll_dev);
}
#endif
//added by chenchen for proximity function 20140828 end

static irqreturn_t fts_ts_interrupt(int irq, void *dev_id)
{
	struct fts_ts_data *fts_ts = dev_id;
	int ret = 0;
    
	disable_irq_nosync(fts_ts->irq);
	/*Modified by jiao.shp for tp gesture in 20141113 START*/
    if(true == tp_gesture_enable){
        ret = fts_read_gesture_id(fts_ts);
        if (ret == 0)
            fts_gesture_id_report(fts_ts);
    }else{
        ret = fts_read_Touchdata(fts_ts);
	    if (ret == 0)
		    fts_report_value(fts_ts);
    }
	/*Modified by jiao.shp for tp gesture in 20141113 START*/

	enable_irq(fts_ts->irq);

	return IRQ_HANDLED;
}

/*added by jiao.shp for tp gesture in 20141113 START*/
static ssize_t ft5x46_tp_gesture_support_show(struct kobject *kobj,
                            struct kobj_attribute *attr,
                            char *buf)
{
	ssize_t num_read_chars = 0;

	num_read_chars = snprintf(buf,PAGE_SIZE, "%d\n", tp_gesture_support);

	return num_read_chars;
}

static ssize_t ft5x46_tp_gesture_support_store(struct kobject *kobj,
                        	struct kobj_attribute *attr,
                        	const char *buf, size_t count)
{
	unsigned long  val = 0;

	val = simple_strtoul(buf, NULL, 10);;
	if(val)
		tp_gesture_support=true;
	else
		tp_gesture_support=false;

 	return count;
}

static ssize_t ft5x46_tp_glove_mode_support_show(struct kobject *kobj,
                            struct kobj_attribute *attr,
                            char *buf)
{
	ssize_t num_read_chars = 0;

	num_read_chars = snprintf(buf,PAGE_SIZE, "%d\n", tp_glove_mode_enable);

	return num_read_chars;
}

static ssize_t ft5x46_tp_glove_mode_support_store(struct kobject *kobj,
                        	struct kobj_attribute *attr,
                        	const char *buf, size_t count)
{
	unsigned long  val = 0;
    u8 auc_i2c_write_buf[2];
    
	val = simple_strtoul(buf, NULL, 10);;
	if(val){                
        auc_i2c_write_buf[0] = 0xc0;
        auc_i2c_write_buf[1] = 0x01;
        fts_i2c_Write(ft5x46_i2c_client, auc_i2c_write_buf, 2);
        tp_glove_mode_enable=true;
    }		
	else{
        auc_i2c_write_buf[0] = 0xc0;
        auc_i2c_write_buf[1] = 0x00;
        fts_i2c_Write(ft5x46_i2c_client, auc_i2c_write_buf, 2);
        tp_glove_mode_enable=false;
    }		

 	return count;
}

static ssize_t ft5x46_tp_gesture_id_show(struct kobject *kobj,
                            struct kobj_attribute *attr,
                            char *buf)
{
	ssize_t num_read_chars = 0;

	num_read_chars = snprintf(buf,PAGE_SIZE, "%d\n", gesture_id);

	return num_read_chars;
}

struct kobject *ft5x46_tp_gesture_properties_kobj;
struct kobject *ft5x46_tp_glove_mode_prop_kobj;

static struct kobj_attribute ft5x46_tp_gesture_attr = {
       .attr = {
               .name = "tp_gesture_enable",
               .mode = S_IRUGO | S_IWUGO,
       },
       .show = &ft5x46_tp_gesture_support_show,
       .store = &ft5x46_tp_gesture_support_store,
};

static struct kobj_attribute ft5x46_tp_gesture_id_attr = {
       .attr = {
               .name = "gesture_id",
               .mode = S_IRUGO,
       },
       .show = &ft5x46_tp_gesture_id_show,
};

static struct kobj_attribute ft5x46_tp_glove_prop_attr = {
       .attr = {
               .name = "glove_mode",
               .mode = S_IRUGO | S_IWUGO,
       },
       .show = &ft5x46_tp_glove_mode_support_show,
       .store = &ft5x46_tp_glove_mode_support_store,
};

static struct attribute *ft5x46_tp_gesture_properties_attrs[] = {
       &ft5x46_tp_gesture_attr.attr,
       &ft5x46_tp_gesture_id_attr.attr,
       NULL,
};

static struct attribute *ft5x46_tp_glove_prop_attrs[] = {
       &ft5x46_tp_glove_prop_attr.attr,
       NULL,
};

static struct attribute_group ft5x46_tp_gesture_properties_attr_group = {
       .attrs = ft5x46_tp_gesture_properties_attrs,
};

static struct attribute_group ft5x46_tp_glove_prop_attr_group = {
       .attrs = ft5x46_tp_glove_prop_attrs,
};

static int tp_gesture_input_dev_init(struct input_dev *input_dev)
{
    int i = 0 ;
    
    for(i=0x20;i< 0x25;i++){
        input_set_capability(input_dev, EV_KEY, i);
    }

    for(i=0x30;i< 0x35;i++){
        input_set_capability(input_dev, EV_KEY, i);
    }

    return 0;
}

#ifdef CONFIG_OF
static int ft5x46_tp_gesture_init(struct device *dev,
			struct fts_platform_data *pdata)
{
    struct device_node *np = dev->of_node;
    int rc;
    
    pdata->tp_gesture_support = of_property_read_bool(np,
                        "focaltech,tp_gesture_support");
	if(pdata->tp_gesture_support){
	      pr_info("%s,tp gesture support...\n",__func__);
	      ft5x46_tp_gesture_properties_kobj =
                       kobject_create_and_add("ft5x46_tp_gesture", NULL);
	      if(ft5x46_tp_gesture_properties_kobj){
    	      rc = sysfs_create_group(ft5x46_tp_gesture_properties_kobj,
                                   &ft5x46_tp_gesture_properties_attr_group);
    	      if(rc)
    		    return rc;
              wake_lock_init(&tp_gesture_wakelock, WAKE_LOCK_SUSPEND,"gesture_wakelock");
              tp_gesture_support = true;
	      }         
	}
    return 0;
}

static int ft5x46_tp_glove_mode_init(struct device *dev,
			                                 struct fts_platform_data *pdata)
{
    struct device_node *np = dev->of_node;
    int rc;
    u8 auc_i2c_write_buf[2];
    struct i2c_client *client;

    pdata->tp_glove_mode_support = of_property_read_bool(np,
                        "focaltech,tp_glove_mode_support");
	if(pdata->tp_glove_mode_support){
	      pr_info("%s,tp glove mode support...\n",__func__);
          client = to_i2c_client(dev);
	      ft5x46_tp_glove_mode_prop_kobj =
                       kobject_create_and_add("ft5x46_tp_glove", NULL);
	      if(ft5x46_tp_glove_mode_prop_kobj){
    	      rc = sysfs_create_group(ft5x46_tp_glove_mode_prop_kobj,
                                   &ft5x46_tp_glove_prop_attr_group);
    	      if(rc)
    		    return rc;
              tp_glove_mode_enable = false;
              auc_i2c_write_buf[0] = 0xc0;
              auc_i2c_write_buf[1] = 0x00;
              fts_i2c_Write(client, auc_i2c_write_buf, 2);
	      }         
	}
    return 0;
}
#else
static int ft5x46_tp_gesture_init(struct device *dev,
			struct fts_platform_data *pdata)
{
	return -ENODEV;
}
static int ft5x46_tp_gesture_init(struct device *dev,
			struct fts_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static int ft5x46_tp_gesture_enable(struct i2c_client *client)
{
	u8 auc_i2c_write_buf[10];    	
	
	pr_debug("%s..\n",__func__);
	if(tp_gesture_enable == true){
	     return 0;
	}
	   
	auc_i2c_write_buf[0] = 0xd0;
	auc_i2c_write_buf[1] = 0x01;
	fts_i2c_Write(client, auc_i2c_write_buf, 2);	//let fw open gestrue function
	
	auc_i2c_write_buf[0] = 0xd1;
	auc_i2c_write_buf[1] = 0x3f;
	fts_i2c_Write(client, auc_i2c_write_buf, 2);
	
	auc_i2c_write_buf[0] = 0xd2;
	auc_i2c_write_buf[1] = 0xff;
	fts_i2c_Write(client, auc_i2c_write_buf, 2);
	
	
	tp_gesture_enable = true;
	enable_irq_wake(client->irq);
	
  return 0;
}

static int ft5x46_tp_gesture_disable(struct i2c_client *client)
{
    
	u8 auc_i2c_write_buf[10];

	if(tp_gesture_enable == false){
	     return 0;
	}	
	auc_i2c_write_buf[0] = 0xd0;
	auc_i2c_write_buf[1] = 0x00;
	fts_i2c_Write(client, auc_i2c_write_buf, 2);	//let fw close gestrue function 
	
	disable_irq_wake(client->irq);
	tp_gesture_enable = false;
	
	return 0;
}

/*added by jiao.shp for tp gesture in 20141113 END*/

#ifdef CONFIG_OF
static int ft5x36_get_dt_coords(struct device *dev, char *name,
				struct fts_platform_data *pdata)
{
	u32 coords[FT_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FT_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "focaltech,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "focaltech,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int ft5x36_parse_dt(struct device *dev,
			struct fts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_BUTTONS];
	bool virthual_key_support;
	bool auto_clb;

	rc = ft5x36_get_dt_coords(dev, "focaltech,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = ft5x36_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (rc)
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np,
						"focaltech,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np,
						"focaltech,no-force-update");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
				0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
				0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	rc = of_property_read_u32(np, "focaltech,family-id", &temp_val);
	if (!rc)
		pdata->family_id = temp_val;
	else{
			dev_err(dev, "Unable to read family id\n");
			return rc;
	}

	rc = of_property_read_u32(np, "focaltech,max_touch_point", &temp_val);
	if (!rc)
		pdata->max_touch_point = temp_val;
	else{
			dev_err(dev, "Unable to read max touch point\n");
		       pdata->max_touch_point = 2;
	}

	auto_clb = of_property_read_bool(np,
                        "focaltech,auto_clb");
	if(auto_clb)
           pdata->auto_clb=true;
	else
           pdata->auto_clb=false;

	prop = of_find_property(np, "focaltech,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"focaltech,button-map", button_map,
			num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

	virthual_key_support = of_property_read_bool(np,
                        "focaltech,virtual_key");
	if(virthual_key_support){
	      printk(KERN_INFO "%s,virtual key support...\n",__func__);
	      ft5x36_virtual_key_properties_kobj =
                       kobject_create_and_add("board_properties", NULL);
	      if(ft5x36_virtual_key_properties_kobj){
	      rc = sysfs_create_group(ft5x36_virtual_key_properties_kobj,
                               &ft5x36_virtual_key_properties_attr_group);
	      if(rc)
		    return rc;
	       }
	}	
	return 0;
}
#else
static int ft5x36_parse_dt(struct device *dev,
			struct fts_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static int ft5x36_power_on(struct fts_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc=regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
		}
	}

	return rc;
}

static int ft5x36_power_init(struct fts_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}
	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}
/*modified for pin used touch start.zengguang 2014.08.22*/
/***********************************************
Name: 		<ft5x06_ts_pinctrl_init>
Author:		<zengguang>
Date:   		<2014.07.17>
Purpose:		<modified for pin used touch>
Declaration:	QRT Telecom Technology Co., LTD
***********************************************/
static int ft5x36_ts_pinctrl_init(struct fts_ts_data *data)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	data->ts_pinctrl = devm_pinctrl_get(&(data->client->dev));
	if (IS_ERR_OR_NULL(data->ts_pinctrl)) {
		dev_dbg(&data->client->dev,
			"Target does not use pinctrl\n");
		retval = PTR_ERR(data->ts_pinctrl);
		data->ts_pinctrl = NULL;
		return retval;
	}

	data->gpio_state_active
		= pinctrl_lookup_state(data->ts_pinctrl,
			"pmx_ts_active");
	if (IS_ERR_OR_NULL(data->gpio_state_active)) {
		dev_dbg(&data->client->dev,
			"Can not get ts default pinstate\n");
		retval = PTR_ERR(data->gpio_state_active);
		data->ts_pinctrl = NULL;
		return retval;
	}

	data->gpio_state_suspend
		= pinctrl_lookup_state(data->ts_pinctrl,
			"pmx_ts_suspend");
	if (IS_ERR_OR_NULL(data->gpio_state_suspend)) {
		dev_err(&data->client->dev,
			"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(data->gpio_state_suspend);
		data->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

/***********************************************
Name: 		<ft5x06_ts_pinctrl_select>
Author:		<zengguang>
Date:   		<2014.07.17>
Purpose:		<Set ctp pin state to handle hardware>
Declaration:	QRT Telecom Technology Co., LTD
***********************************************/
static int ft5x36_ts_pinctrl_select(struct fts_ts_data *data,
						bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? data->gpio_state_active
		: data->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(data->ts_pinctrl, pins_state);
		if (ret) {
			dev_err(&data->client->dev,
				"can not set %s pins\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	} else {
		dev_err(&data->client->dev,
			"not a valid '%s' pinstate\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
	}

	return 0;
}

static int ft5x36_ts_pinctrl_put(struct fts_ts_data *data)
{
	if(NULL != data->ts_pinctrl)
           pinctrl_put(data->ts_pinctrl);
	
	return 0;
}

/*modified for pin used touch start.zengguang 2014.08.22*/
#ifdef CONFIG_PM
static int ft5x36_ts_suspend(struct device *dev)
{
	struct fts_ts_data *ft5x36_ts = dev_get_drvdata(dev);
	char txbuf[2];
	int err;
	int i;


	if (ft5x36_ts->loading_fw) {
		dev_info(dev, "Firmware loading in process...\n");
		return 0;
	}

	if (ft5x36_ts->suspended) {
		dev_info(dev, "Already in suspend state\n");
		return 0;
	}
	pr_info("###%s\n",__func__);
//del by chenchen 20140825 begin
/*modified for pin used touch start.zengguang 2014.08.22*/	
//	if (ft5x36_ts->ts_pinctrl) {
//		err = ft5x36_ts_pinctrl_select(ft5x36_ts, false);
//		if (err < 0)
//			pr_err("Cannot get idle pinctrl state\n");
//	}
/*modified for pin used touch end.zengguang 2014.08.22*/
//del by chenchen 20140825 end
//added by chenchen for proximity function 20140828 begin	
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	if(tp_prox_sensor_opened){
		printk("tp can not sleep in call\n");
		is_need_report_pointer = 0;
		return 0;
	}
#endif
//added by chenchen for proximity function 20140828 end	

    /*Modified by jiao.shp for tp gesture in 20141113 START*/
    if (tp_gesture_support == true){
        ft5x46_tp_gesture_enable(ft5x36_ts->client);
        ft5x36_ts->suspended = true;
        return 0;
    }
    
    disable_irq(ft5x36_ts->client->irq);
    
    	/* release all touches */
		for (i = 0; i < 5; i++) {
			input_mt_slot(ft5x36_ts->input_dev, i);
			input_mt_report_slot_state(ft5x36_ts->input_dev, MT_TOOL_FINGER, 0);
		}
		input_mt_report_pointer_emulation(ft5x36_ts->input_dev, false);
		input_sync(ft5x36_ts->input_dev);
		
    if (gpio_is_valid(ft5x36_ts->pdata->reset_gpio)) {
		txbuf[0] = FTS_REG_PMODE;
		txbuf[1] = FTS_PMODE_HIBERNATE;
		fts_i2c_Write(ft5x36_ts->client, txbuf, sizeof(txbuf));
  }

    /*Modified by jiao.shp for tp gesture in 20141113 END*/

    /*Modified by jiao.shp for tp gesture in 20141113 START*/
  	if (ft5x36_ts->pdata->power_on) {
  		err = ft5x36_ts->pdata->power_on(false);
  		if (err) {
  			dev_err(dev, "power off failed");
  			goto pwr_off_fail;
  		}
  	} else {
  		err = ft5x36_power_on(ft5x36_ts, false);
  		if (err) {
  			dev_err(dev, "power off failed");
  			goto pwr_off_fail;
  		}
  	}
    /*Modified by jiao.shp for tp gesture in 20141113 END*/

	ft5x36_ts->suspended = true;
//added by chenchen for proximity function 20140828 begin
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	is_suspend = 1;
#endif

	return 0;

pwr_off_fail:
	if (gpio_is_valid(ft5x36_ts->pdata->reset_gpio)) {
		gpio_set_value_cansleep(ft5x36_ts->pdata->reset_gpio, 0);
		msleep(FT_RESET_DLY);
		gpio_set_value_cansleep(ft5x36_ts->pdata->reset_gpio, 1);
	}
	enable_irq(ft5x36_ts->client->irq);
	return err;
}

static int ft5x36_ts_resume(struct device *dev)
{
	struct fts_ts_data *ft5x36_ts = dev_get_drvdata(dev);
	int err;
//added by chenchen for proximity function 20140828 begin	
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	is_need_report_pointer = 1;
	if((tp_prox_sensor_opened) && (is_suspend == 0)){
		printk("%s tp no need to wake up in call\n",__func__);
		return 0;
	}
#endif
//added by chenchen for proximity function 20140828 end	

	if (!ft5x36_ts->suspended) {
		dev_info(dev, "Already in awake state\n");
		return 0;
	}

  if(true == tp_gesture_enable){
      ft5x46_tp_gesture_disable(ft5x36_ts->client);
      ft5x36_ts->suspended = false;
      return 0;
  }
    

 if (ft5x36_ts->pdata->power_on) {
		err = ft5x36_ts->pdata->power_on(true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	} else {
		err = ft5x36_power_on(ft5x36_ts, true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	}
    
	if (gpio_is_valid(ft5x36_ts->pdata->reset_gpio)) {
		gpio_set_value_cansleep(ft5x36_ts->pdata->reset_gpio, 0);
		mdelay(FT_RESET_DLY);
		gpio_set_value_cansleep(ft5x36_ts->pdata->reset_gpio, 1);
	}
	mdelay(FT_STARTUP_DLY);

  enable_irq(ft5x36_ts->client->irq);

 if(tp_glove_mode_enable == true){
      u8 auc_i2c_write_buf[2];
      auc_i2c_write_buf[0] = 0xc0;
      auc_i2c_write_buf[1] = 0x01;
      fts_i2c_Write(ft5x36_ts->client, auc_i2c_write_buf, 2);
  }

	ft5x36_ts->suspended = false;
//added by chenchen for proximity function 20140828 begin
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	tp_prox_sensor_enable(tp_prox_sensor_opened);
	is_suspend = 0;
#endif

	return 0;
}

static const struct dev_pm_ops ft5x36_ts_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = ft5x36_ts_suspend,
	.resume = ft5x36_ts_resume,
#endif
};
#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct fts_ts_data *ft5x36_data =
		container_of(self, struct fts_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			ft5x36_data && ft5x36_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			ft5x36_ts_resume(&ft5x36_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			ft5x36_ts_suspend(&ft5x36_data->client->dev);
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ft5x36_ts_early_suspend(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct fts_ts_data,
						   early_suspend);

	ft5x36_ts_suspend(&data->client->dev);
}

static void ft5x36_ts_late_resume(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct fts_ts_data,
						   early_suspend);

	ft5x36_ts_resume(&data->client->dev);
}
#endif
/*Added by liumx for tp no effect 2013.10.9 start*/
/*
static void fts_work_func(struct work_struct *work)
{
	//struct fts_ts_data *fts_ts = dev_id;
	int ret = 0;
	struct fts_ts_data *fts_ts = NULL;
	mutex_lock(&ft5x36_mutex);
	fts_ts = container_of(work, struct fts_ts_data, work);
	ret = fts_read_Touchdata(fts_ts);
	if (ret == 0)
		fts_report_value(fts_ts);
	//spin_lock_irqsave(&fts_ts->irq_lock, flag);
	enable_irq(fts_ts->irq);
	//spin_unlock_irqrestore(&fts_ts->irq_lock, flag);
	mutex_unlock(&ft5x36_mutex);
}*/
/*Added by liumx for tp no effect 2013.10.9 end*/

#ifdef CONFIG_GET_HARDWARE_INFO
//yuquan added begin hardware info
//modify by chenchen for hardware info 20140820 begin
static void ft5x36_hardware_info_reg(u8 family_id,u8 vendod_id,u8 fw_ver)
{
    int i=0;
    if(family_id==IC_FT5x36)
	 strcpy(tmp_tp_name, "FT5x36_");
    else if(family_id==IC_FT6x06)
	 strcpy(tmp_tp_name, "FT6x06_");
    //added by jiao.shp for focaltech FT5526 20141023	begin
    else if(family_id==IC_FT5x46)
	 strcpy(tmp_tp_name, "FT5x46");
    //added by jiao.shp for focaltech FT5526 20141023	end
    else{
	 printk(KERN_ERR "invalid family id = 0x%x\n",family_id);
	 return;
    }
    for(i=0;i<sizeof(ft5x36_name)/sizeof(ft5x36_ven_info);i++){
        if(vendod_id==ft5x36_name[i].vendor_id){
	     strcat(tmp_tp_name,ft5x36_name[i].ven_name);
		 strcat(tmp_tp_name,"Ver0x");
		sprintf(tmp_tp_name+strlen(tmp_tp_name), "%x",fw_ver);
//modify by chenchen for hardware info 20140820 end		
	     break;
        }else{
            continue;
        }
    }
    if(i==sizeof(ft5x36_name)/sizeof(ft5x36_ven_info))
        strcat(tmp_tp_name,"unknow");
    register_hardware_info(CTP, tmp_tp_name);

}
//yuquan added end hardware info
#endif
static int fts_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct fts_platform_data *pdata =
	    (struct fts_platform_data *)client->dev.platform_data;
	struct fts_ts_data *fts_ts;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char ven_id_val;
	unsigned char ven_id_addr;
//added by chenchen for hardware info 20140820 begin
	unsigned char fw_ver_val;
	unsigned char fw_ver_addr;
//added by chenchen for hardware info 20140820 end	
	//char projectcode[32]; 

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct fts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = ft5x36_parse_dt(&client->dev, pdata);
		if (err)
			return err;
    /*added by jiao.shp for tp gesture in 20141113 START*/
    err = ft5x46_tp_gesture_init(&client->dev, pdata);
    if (err)
        return err;
    /*added by jiao.shp for tp gesture in 20141113 END*/
    err = ft5x46_tp_glove_mode_init(&client->dev, pdata);
    if (err)
        return err;
	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "Invalid pdata\n");
		return -EINVAL;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		return err;
	}
	fts_ts = kzalloc(sizeof(struct fts_ts_data), GFP_KERNEL);

	if (!fts_ts) {
		err = -ENOMEM;
		return err;
	}
	
	i2c_set_clientdata(client, fts_ts);
	fts_ts->irq = client->irq;
	fts_ts->client = client;
	fts_ts->pdata = pdata;
	fts_ts->x_max = pdata->x_max - 1;
	fts_ts->y_max = pdata->y_max - 1;
	fts_ts->suspended = false;
	#if 0
	err = request_threaded_irq(client->irq, NULL, fts_ts_interrupt,
				   pdata->irqflags, client->dev.driver->name,
				   fts_ts);
	if (err < 0) {
		dev_err(&client->dev, "fts_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	disable_irq(client->irq);
	#endif
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto free_mem;
	}
	fts_ts->input_dev = input_dev;
	
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev,pdata->max_touch_point,0);
       input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min, pdata->x_max, 0, 0);
       input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min, pdata->y_max, 0, 0);
       input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
       input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
       input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

       input_set_capability(input_dev, EV_KEY, KEY_MENU); 
       input_set_capability(input_dev, EV_KEY, KEY_HOMEPAGE); 
       input_set_capability(input_dev, EV_KEY, KEY_BACK); 
       /*added by jiao.shp for tp gesture in 20141113 START*/
       if(pdata->tp_gesture_support){
           ft5x46_i2c_client = client;
           tp_gesture_input_dev_init(input_dev); 
       }
       /*added by jiao.shp for tp gesture in 20141113 END*/
	input_dev->name = FTS_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.product = 0x0101;	
	input_dev->id.vendor = 0x1010;	
	input_dev->id.version = 0x000a;	
	
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"fts_ts_probe: failed to register input device: %s\n",
			dev_name(&client->dev));
		goto free_inputdev;
	}
		
	if (pdata->power_init) {
		err = pdata->power_init(true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	} else {
		err = ft5x36_power_init(fts_ts, true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	}

	if (pdata->power_on) {
		err = pdata->power_on(true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	} else {
		err = ft5x36_power_on(fts_ts, true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	}

	if (gpio_is_valid(pdata->irq_gpio)) {
		
		err = gpio_request(pdata->irq_gpio, "ft5x36_irq_gpio");
		if (err) {
			dev_err(&client->dev, "irq gpio request failed");
			goto pwr_off;
		}
		err = gpio_direction_input(pdata->irq_gpio);
		if (err) {
			dev_err(&client->dev,
				"set_direction for irq gpio failed\n");
			goto free_irq_gpio;
		}
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		
		err = gpio_request(pdata->reset_gpio, "ft5x36_reset_gpio");
		if (err) {
			dev_err(&client->dev, "reset gpio request failed");
			goto free_irq_gpio;
		}

		err = gpio_direction_output(pdata->reset_gpio, 0);
		if (err) {
			dev_err(&client->dev,
				"set_direction for reset gpio failed\n");
			goto free_reset_gpio;
		}
		msleep(FT_RESET_DLY);
		gpio_set_value_cansleep(fts_ts->pdata->reset_gpio, 1);
	}

/*modified for pin used touch start.zengguang 2014.08.22*/
	err = ft5x36_ts_pinctrl_init(fts_ts);
	if (!err && fts_ts->ts_pinctrl) {
		err = ft5x36_ts_pinctrl_select(fts_ts, true);
		if (err < 0){
			dev_err(&client->dev,
				"fts pinctrl select failed\n");
		}
	}
/*modified for pin used touch end.zengguang 2014.08.22*/
	/*make sure CTP already finish startup process */
	msleep(200);

	/*get some register information */
	ven_id_addr = FTS_REG_VENDOR_ID;
	err = fts_i2c_Read(client, &ven_id_addr, 1, &ven_id_val, 1);
	printk(KERN_INFO"[FTS]FTS_REG_VENDOR_ID =0x%X\n ",ven_id_val);
	if (err < 0) {
		dev_err(&client->dev, "vendor id read failed");
		goto free_reset_gpio;
	}

	/*Added by liumx for tp no effect 2013.10.9 start*/
	/*
	fts_ts->fts_wq = create_workqueue("fts_wq");
	INIT_WORK(&fts_ts->work, fts_work_func);*/
	/*Added by liumx for tp no effect 2013.10.9 end*/
//modify interrupt trigger by chenchen 20140825
	err = request_threaded_irq(client->irq, NULL, fts_ts_interrupt,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->dev.driver->name,
				   fts_ts);
	if (err < 0) {
		dev_err(&client->dev, "fts_probe: request irq failed\n");
		goto free_reset_gpio;
	}
//added by chenchen for proximity function 20140828 begin
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	i2c_prox_client = client;
	#ifdef SENSOR_PROX_TP_USE_WAKELOCK
	wake_lock_init(&sensor_prox_tp_wake_lock, WAKE_LOCK_SUSPEND, "prox_tp");
	#endif

	//err = sysfs_create_group(&client->dev.kobj, &tp_prox_attribute_group);
	tp_prox_setup_polled_device(fts_ts);
	err = sysfs_create_group(&fts_ts->input_poll_dev->input->dev.kobj, &tp_prox_attribute_group);
	if (err) {
		printk( "sysfs create failed: %d\n", err);
	}

	//tp_prox_setup_polled_device(fts_ts);
	fts_ts->ps_cdev = sensors_proximity_cdev;
	fts_ts->ps_cdev.sensors_enable = tp_prox_set_enable;
	fts_ts->ps_cdev.sensors_poll_delay = NULL;
	err = sensors_classdev_register(&client->dev, &fts_ts->ps_cdev);
	if (err) {
		pr_err("%s: Unable to register to sensors class: %d\n",
				__func__, err);
		//goto exit_kfree;
	}
#endif
//added by chenchen for proximity function 20140828 end
	disable_irq(client->irq);

#if defined(CONFIG_FB)
	fts_ts->fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&fts_ts->fb_notif);

	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
			err);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	fts_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						    FT_SUSPEND_LEVEL;
	fts_ts->early_suspend.suspend = ft5x36_ts_early_suspend;
	fts_ts->early_suspend.resume = ft5x36_ts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

#ifdef FTS_SYSFS_DEBUG
	fts_create_sysfs(client);
  #ifdef FTS_APK_DEBUG
	fts_create_apk_debug_channel(client);
  #endif
#endif

//modify by chenchen for hardware info 20140909 begin	
	fw_ver_addr = FTS_REG_FW_VER;
	err = fts_i2c_Read(client, &fw_ver_addr, 1, &fw_ver_val, 1);
	if (err < 0) {
		dev_err(&client->dev, "FW version read failed");
		goto free_reset_gpio;
	}
	printk(KERN_INFO "FT5x36:firmware version in tp mudule = 0x%x\n",fw_ver_val);	
#ifdef CONFIG_GET_HARDWARE_INFO
	ft5x36_hardware_info_reg(pdata->family_id,ven_id_val,fw_ver_val);
#endif	
//modify by chenchen for hardware info 20140909 end

#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(client) < 0)
		dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",
				__func__);
#endif
    
	enable_irq(client->irq);
	printk(KERN_INFO "%s,FT5x36 enable_irq over",__func__);
	return 0;
	
free_reset_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
free_irq_gpio:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
/*modified for pin used touch start.zengguang 2014.08.22*/	
	if (fts_ts->ts_pinctrl) {
		err = ft5x36_ts_pinctrl_select(fts_ts, false);
		if (err < 0)
			pr_err("Cannot get idle pinctrl state\n");
		ft5x36_ts_pinctrl_put(fts_ts);
	}
/*modified for pin used touch end.zengguang 2014.08.22*/
pwr_off:
	if (pdata->power_on)
		pdata->power_on(false);
	else
		ft5x36_power_on(fts_ts, false);
pwr_deinit:
	if (pdata->power_init)
		pdata->power_init(false);
	else
		ft5x36_power_init(fts_ts, false);
unreg_inputdev:
	input_unregister_device(input_dev);
	input_dev = NULL;

free_inputdev:
	input_free_device(input_dev);
	
free_mem:
	kfree(fts_ts);
	return err;
}
static int  fts_ts_remove(struct i2c_client *client)
{
	struct fts_ts_data *fts_ts;
	fts_ts = i2c_get_clientdata(client);
	input_unregister_device(fts_ts->input_dev);
	#ifdef CONFIG_PM
	gpio_free(fts_ts->pdata->reset_gpio);
	#endif
	
//added by chenchen for proximity function 20140828 begin
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
    sysfs_remove_group(&client->dev.kobj, &tp_prox_attribute_group);
    tp_prox_teardown_polled_device(fts_ts);
#ifdef SENSOR_PROX_TP_USE_WAKELOCK
    wake_lock_destroy(&sensor_prox_tp_wake_lock);
#endif
#endif
//added by chenchen for proximity function 20140828 end

/*modified for pin used touch start.zengguang 2014.08.22*/	
	if (fts_ts->ts_pinctrl) {
		ft5x36_ts_pinctrl_put(fts_ts);
	}
/*modified for pin used touch end.zengguang 2014.08.22*/
#ifdef FTS_SYSFS_DEBUG
  #ifdef FTS_APK_DEBUG
	fts_release_apk_debug_channel();
  #endif
	fts_release_sysfs(client);
#endif
	#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
	#endif
	free_irq(client->irq, fts_ts);
	kfree(fts_ts);
	i2c_set_clientdata(client, NULL);
    /*Added by jiao.shp for tp gesture in 20141114*/
    if(fts_ts->pdata->tp_gesture_support)
        wake_lock_destroy(&tp_gesture_wakelock);
	return 0;
}

static const struct i2c_device_id fts_ts_id[] = {
	{FTS_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, fts_ts_id);

#ifdef CONFIG_OF
static struct of_device_id ft5x36_match_table[] = {
	{ .compatible = "focaltech,FT5x36",},
	{ },
};
#else
#define ft5x36_match_table NULL
#endif


static struct i2c_driver fts_ts_driver = {
	.probe 		= fts_ts_probe,
	.remove 	=  fts_ts_remove,
	.id_table 	= fts_ts_id,
	.driver = {
		   .name = FTS_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = ft5x36_match_table,
#ifdef CONFIG_PM
		   .pm = &ft5x36_ts_pm_ops,
#endif
		   },
};

static int __init fts_ts_init(void)
{
	int ret;
	ret = i2c_add_driver(&fts_ts_driver);
	if (ret) {
		printk(KERN_WARNING "Adding FTS driver failed " "(errno = %d)\n", ret);
	} else {
		pr_info("Successfully added driver %s\n",fts_ts_driver.driver.name);
	}
	return ret;
}

static void __exit fts_ts_exit(void)
{
	i2c_del_driver(&fts_ts_driver);
}

module_init(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_AUTHOR("<luowj>");
MODULE_DESCRIPTION("FocalTech TouchScreen driver");
MODULE_LICENSE("GPL");
