/*
  * drivers/misc/mp3309.c    
  * Author : jiao.shp
  */

#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>

#define BL_CTRL_BY_I2C_WRITE
#define QRD_BL_CTRLER_SLAVE_REG0 0x00
#define QRD_BL_CTRLER_SLAVE_REG1 0x01

struct i2c_client *mp3309_i2c_client;

static s32 mp3309_i2c_read( struct i2c_client *client_dev,u8 reg_address)
{
    struct i2c_client *client = NULL;
    unsigned char rxData[2];
    struct i2c_msg msgs[2];
    unsigned short length = 0;
    
    rxData[0]= reg_address;
    client = client_dev;
    length = 1;
    
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;                     
    msgs[0].len = 1;
    msgs[0].buf = rxData;

    msgs[1].addr =client ->addr;
    msgs[1].flags = I2C_M_RD;             
    msgs[1].len = length;
    msgs[1].buf = rxData;

    
    if (i2c_transfer(client->adapter, msgs, 2) < 0) {
        pr_err("mp3309_i2c_read: transfer error\n");
        return -EIO;
    }
    else
    return (s32)rxData[0];
}

void mp3309_test(struct i2c_client *client_dev)
{
    s32 ret;

    ret = mp3309_i2c_read(client_dev,QRD_BL_CTRLER_SLAVE_REG0);
    if(ret < 0){
        pr_err("%s is error at line %d,ret is %d \n",__func__,__LINE__,ret);
        return;
    }else
    pr_info("MP3309 Reg 0x00 is 0x%x\n",ret);

    ret = mp3309_i2c_read(client_dev,QRD_BL_CTRLER_SLAVE_REG1);
    if(ret < 0){
        pr_err("%s is error at line %d,ret is %d \n",__func__,__LINE__,ret);
        return;
    }else
    pr_info("MP3309 Reg 0x01 is 0x%x\n",ret);    
}

static int mp3309_i2c_write( struct i2c_client *client_dev,u8 reg_address,u8 data)
{
    struct i2c_client *client = NULL;
    unsigned char txData[2];
    unsigned short length = 0;
    struct i2c_msg msg;
    
    client = client_dev;
    txData[0] = reg_address;
    txData[1] = data;
    length = sizeof(txData);
    
    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = length;
    msg.buf = txData;
    
    if (i2c_transfer(client->adapter, &msg, 1) < 0) {
        pr_err("mp3309_i2c_write: transfer error\n");
        return -EIO;
    } 
    else
    return 0;
}

#ifdef BL_CTRL_BY_I2C_WRITE
unsigned char change_order(unsigned char input)
{       
	unsigned int t;
	unsigned char result;
	for(t=0;t  <8;t++)       
	{               
		if((input&(1 <<t))!=0)
			result|=(1 <<(7-t));
		else 
			result&=~(1 << (7-t));
	}
	return result;
}  

int bl_level_system_to_mp3309(int level)
{
	int mp3309_level;
	
	if(level>255||level<0)
		level=0;
	mp3309_level=level/8;
	return mp3309_level;
}

int mp3309_bl_ctrl(int level)
{
	unsigned char reg_value;
	unsigned char tmp;
	int ret = 0;
	tmp=change_order((unsigned char)bl_level_system_to_mp3309(level));
	tmp=tmp>>3;
	reg_value = 0x80 | tmp << 2;
	pr_debug("#####into  mp3309_bl_ctrl at %d reg_value=%d\n",__LINE__,reg_value);
	ret = mp3309_i2c_write(mp3309_i2c_client,QRD_BL_CTRLER_SLAVE_REG0,reg_value);
    	if(ret){
		pr_err("mp3309_i2c_init is error at %d\n",__LINE__);
        return ret;
    }

	return 0;	
}
EXPORT_SYMBOL(mp3309_bl_ctrl);
#endif

int mp3309_i2c_init(void)
{
    int ret = 0;

    ret = mp3309_i2c_write(mp3309_i2c_client,QRD_BL_CTRLER_SLAVE_REG0,0x80);
    if(ret){
        pr_err("mp3309_i2c_init is error at %d\n",__LINE__);
        return ret;
    }

    ret = mp3309_i2c_write(mp3309_i2c_client,QRD_BL_CTRLER_SLAVE_REG1,0x28);//0x08
    if(ret){
        pr_err("mp3309_i2c_init is error at %d\n",__LINE__);
        return ret;
    }
    mp3309_test(mp3309_i2c_client);
    printk("%s is successful,ret is %d\n",__func__,ret);
    
	return ret;
}
EXPORT_SYMBOL(mp3309_i2c_init);

static int mp3309_probe(struct i2c_client *client, const struct i2c_device_id *id)
{ 
    pr_info("%s is started,chip address is 0x%x \n",__func__,client->addr);

    if(client == NULL)
        return -ENODEV;
    else
        mp3309_i2c_client = client;
    
    return 0;
}

static int mp3309_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id mp3309_i2c_id[] = {
	{"qcom,BL_MP3309", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, mp3309_i2c_id);

static struct of_device_id mp3309_match_table[] = {
	{ .compatible = "qcom,BL_MP3309", },
	{ },
};

static struct i2c_driver mp3309_driver = {
	.probe      = mp3309_probe,
	.remove     = mp3309_remove,
	.id_table 	= mp3309_i2c_id,
	.driver = {
		.name     = "qcom,BL_MP3309",
		.owner    = THIS_MODULE,
		.of_match_table = mp3309_match_table,
	},
};

static int __init mp3309_init(void)
{
	int ret;
    
    pr_info("%s is started. \n",__func__);
    ret = i2c_add_driver(&mp3309_driver);
    pr_info("%s is end,ret is %d\n",__func__,ret);
    return ret; 
}

static void __exit mp3309_exit(void)
{
    i2c_del_driver(&mp3309_driver);
}

module_init(mp3309_init);
module_exit(mp3309_exit);

MODULE_DESCRIPTION("mp3309 BL controler Driver");
MODULE_LICENSE("GPL");

