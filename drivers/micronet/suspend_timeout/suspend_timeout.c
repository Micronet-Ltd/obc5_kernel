/*
 * Suspend timeout driver
 *
 * Copyright 2016 Micronet Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/
#define pr_fmt(fmt) "%s %s: " fmt, KBUILD_MODNAME, __func__

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <linux/ctype.h>
#include <linux/gpio.h>
#include <linux/notifier.h>

#define MIN_SUSPEND_TM  10000//from Android
#define MAX_SUSPEND_TM  0x7FFFFFFF//from Android

#define SP_TM_BIT       1  
#define IGN_TM_BIT      2
#define IGN_LVL_BIT     3

extern int32_t gpio_in_register_notifier(struct notifier_block *nb);

struct suspend_tm_data {
    struct device           dev;
    struct device           dev_f;
    struct miscdevice       mdev;
    struct miscdevice       dev_ign_tm;
    unsigned long           open;
    uint32_t                to_sp;
    uint32_t                to_db;
    uint32_t                to_ign;
    uint32_t                to_ign_db;
    int32_t                 c_sp;
    int32_t                 c_ign_tm;
    struct device_attribute dev_attr;
    struct device_attribute dev_attr_ito;
    spinlock_t              dev_lock;
    unsigned long           lock_flags;
    wait_queue_head_t       wq_sp;
    struct completion       completion_sp;
    wait_queue_head_t       wq_ign_tm;
    struct completion       completion_ign_tm;
};
///////////////////////
///////////////
static int32_t test_timeout(uint32_t val)
{
    if((-1 != (int32_t)val) && (MIN_SUSPEND_TM > val || MAX_SUSPEND_TM < val) )
    {
        return 0;
    }
    return 1;
}
////
static int32_t ignition_tm_open(struct inode *inode, struct file *file)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, dev_ign_tm);

    pr_notice("\n");
    if(test_and_set_bit(IGN_TM_BIT, &data->open))
	return -EPERM;

    return 0;
}
static int32_t ignition_tm_release(struct inode *inode, struct file *file)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, dev_ign_tm);

    pr_notice("\n");
    complete(&data->completion_ign_tm);

    clear_bit(IGN_TM_BIT, &data->open);

    return 0;
}
static ssize_t ignition_tm_read(struct file * file, char __user * buf, size_t count, loff_t *ppos)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, dev_ign_tm);

    uint8_t output[32];

    pr_info("+\n");
    if (0 == test_bit(IGN_TM_BIT, &data->open)) {//test_bit
        return -EINVAL;
    }
    spin_lock_irqsave(&data->dev_lock, data->lock_flags);
    sprintf(output, "%d\n", data->to_ign);
    data->c_ign_tm  = 0;
    spin_unlock_irqrestore(&data->dev_lock, data->lock_flags);

    if(copy_to_user(buf, output, strlen(output)))
        return -EINVAL;

    pr_info(": %s\n",output);

    return strlen(output);//-'\n'
}
static ssize_t ignition_tm_write(struct file * file, const char __user * buf, size_t count, loff_t * ppos)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, dev_ign_tm);

    uint8_t output[32] = {0};
    uint32_t val;

    pr_info("\n");

    if (0 == test_bit(IGN_TM_BIT, &data->open)) {//test_bit
        return -EINVAL;
    }
    if(count > sizeof(output)) {
        pr_err("count %d too big\n", (int32_t)count);
        return -EINVAL;
    }

    if(copy_from_user(output, buf, min(count, sizeof(output))))
        return -EACCES;

    val = simple_strtol(output, 0, 10);
    pr_info("output %s; val %d\n", output, val);

    if(!isdigit(output[0]))//from db
    {
        pr_err("suspend timeout min = %u ms, max = %u, input %d\n", MIN_SUSPEND_TM, MAX_SUSPEND_TM, val);
        val = 0;
        //set to user 0 //return -EINVAL;
    }
    
    spin_lock_irqsave(&data->dev_lock, data->lock_flags); 
    data->to_ign_db = val;
    data->to_ign = data->to_ign_db;
    spin_unlock_irqrestore(&data->dev_lock, data->lock_flags);
    complete(&data->completion_ign_tm);
    return count;
}

static uint32_t ignition_tm_poll(struct file * file,  poll_table * wait)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, dev_ign_tm);

    uint32_t mask = 0;

    poll_wait(file, &data->wq_ign_tm, wait);
    spin_lock_irqsave(&data->dev_lock, data->lock_flags);
	if(0 != data->c_ign_tm)
        mask |= POLLIN | POLLRDNORM;
    spin_unlock_irqrestore(&data->dev_lock, data->lock_flags);

    return mask;
}
static const struct file_operations ignition_tm_dev_fops = {
	.owner      = THIS_MODULE,
	.llseek     = no_llseek,
	.read       = ignition_tm_read,
	.write      = ignition_tm_write,
	.open       = ignition_tm_open,
	.release    = ignition_tm_release,
	.poll       = ignition_tm_poll,
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int32_t suspend_tm_open(struct inode *inode, struct file *file)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, mdev);

    pr_notice("\n");
    if(test_and_set_bit(SP_TM_BIT, &data->open))
	return -EPERM;

    return 0;
}
static int32_t suspend_tm_release(struct inode *inode, struct file *file)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, mdev);

    complete(&data->completion_sp);

    pr_notice("\n");
    clear_bit(SP_TM_BIT, &data->open);

    return 0;
}
static ssize_t suspend_tm_read(struct file * file, char __user * buf, size_t count, loff_t *ppos)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, mdev);

    uint8_t output[32];

    pr_info("+\n");
    if(0 == test_bit(SP_TM_BIT, &data->open)) {
        return -EINVAL;
    }
    spin_lock_irqsave(&data->dev_lock, data->lock_flags);
    sprintf(output, "%d\n", data->to_sp);
    data->c_sp  = 0;
    spin_unlock_irqrestore(&data->dev_lock, data->lock_flags);

    if(copy_to_user(buf, output, strlen(output)))
        return -EINVAL;

    pr_info(": %s\n",output);

    return strlen(output);
}
static ssize_t suspend_tm_write(struct file * file, const char __user * buf, size_t count, loff_t * ppos)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, mdev);

    uint8_t output[32] = {0};
    uint32_t val;

    pr_info("\n");

    if(0 == test_bit(SP_TM_BIT, &data->open)) {
        return -EINVAL;
    }
    if(count > sizeof(output)) {
        pr_err("count %d too big\n", (int32_t)count);
        return -EINVAL;
    }

    if(copy_from_user(output, buf, min(count, sizeof(output))))
        return -EACCES;

    val = simple_strtol(output, 0, 10);
    pr_info("output %s; val %d\n", output, val);

    if(!isdigit(output[0]))//from db
    {
        pr_err("suspend timeout min = %u ms, max = %u, input %d\n", MIN_SUSPEND_TM, MAX_SUSPEND_TM, val);
        val = 0;
        //set to user 0 //return -EINVAL;
    }
    
    spin_lock_irqsave(&data->dev_lock, data->lock_flags); 
    data->to_db = val;
    data->to_sp = data->to_db;
    spin_unlock_irqrestore(&data->dev_lock, data->lock_flags);
    complete(&data->completion_sp);
    return count;
}

static uint32_t suspend_tm_poll(struct file * file,  poll_table * wait)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, mdev);

    uint32_t mask = 0;

    poll_wait(file, &data->wq_sp, wait);
    spin_lock_irqsave(&data->dev_lock, data->lock_flags);
	if(0 != data->c_sp)
        mask |= POLLIN | POLLRDNORM;
    spin_unlock_irqrestore(&data->dev_lock, data->lock_flags);

    return mask;
}
static const struct file_operations suspend_tm_dev_fops = {
	.owner      = THIS_MODULE,
	.llseek     = no_llseek,
	.read       = suspend_tm_read,
	.write      = suspend_tm_write,
	.open       = suspend_tm_open,
	.release    = suspend_tm_release,
	.poll       = suspend_tm_poll,
};
///////////////////////////////////////////////////////////////////////////

static ssize_t suspend_tm_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int32_t err;
    uint32_t value;
    struct suspend_tm_data* data = container_of(dev, struct suspend_tm_data, dev_f);

//    if(!isdigit(buf[0])) 
//    {
//        if ((buf[0] != '-') || count < 2 || (buf[1] != '1'))//enabled '-1' only 
//        {
//            pr_err("error input %s\n", buf);
//            return -EINVAL;
//        }
//    }
    value = simple_strtol(buf, 0, 10); 
    pr_info("%d (count %u)\n", value, (uint32_t)count);

    //-1(never) or from 15000, 30000, 60000, 120000, 300000, 600000, 1800000
    if(0 == test_timeout(value))
    {
        pr_err("suspend timeout min = %u ms, max = %u, input %d\n", MIN_SUSPEND_TM, MAX_SUSPEND_TM, value);
        return -EINVAL;
    }
    if(-1 == (int32_t)value) 
    {
        value = MAX_SUSPEND_TM;//for PowerManagerService
    }
    if(data->to_db == value)
        return count;

    init_completion(&data->completion_sp);

    spin_lock_irqsave(&data->dev_lock, data->lock_flags); 
    data->to_sp = value;
    data->c_sp  = 1;    
    spin_unlock_irqrestore(&data->dev_lock, data->lock_flags);

    wake_up(&data->wq_sp);
    err = wait_for_completion_interruptible(&data->completion_sp);
    if (0 != err) 
        return err;
    return count; 
}

static ssize_t suspend_tm_show(struct device* dev, struct device_attribute *attr, char *buf) 
{
    struct suspend_tm_data* data = container_of(dev, struct suspend_tm_data, dev_f);
    uint32_t value = 0;

    pr_info("%d\n", (int32_t)data->to_db);

    spin_lock_irqsave(&data->dev_lock, data->lock_flags); 
    value = data->to_db;
    spin_unlock_irqrestore(&data->dev_lock, data->lock_flags);

    if(MAX_SUSPEND_TM <= value)//for PowerManagerService 
    {
        value = -1;
    }
    return sprintf(buf, "%d\n", value);
}
static ssize_t ignition_tm_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int32_t err;
    uint32_t value;
    struct suspend_tm_data* data = container_of(dev, struct suspend_tm_data, dev_f);

    value = simple_strtol(buf, 0, 10); 
    pr_info("%d (count %u)\n", value, (uint32_t)count);

    //-1(never) in menu: 15000, 30000, 60000, 120000, 300000, 600000, 1800000
    if(0 == test_timeout(value))
    {
        pr_err("suspend timeout min = %u ms, max = %u, input %d\n", MIN_SUSPEND_TM, MAX_SUSPEND_TM, value);
        return -EINVAL;
    }
    if(-1 == (int32_t)value) 
    {
        value = MAX_SUSPEND_TM;//for PowerManagerService
    }
    if(data->to_ign_db == value)
        return count;

    init_completion(&data->completion_ign_tm);

    spin_lock_irqsave(&data->dev_lock, data->lock_flags); 
    data->to_ign = value;
    data->c_ign_tm  = 1;    
    spin_unlock_irqrestore(&data->dev_lock, data->lock_flags);

    wake_up(&data->wq_ign_tm);
    err = wait_for_completion_interruptible(&data->completion_ign_tm);
    if (0 != err) 
        return err;

    return count;
}

static ssize_t ignition_tm_show(struct device* dev, struct device_attribute *attr, char *buf) 
{
    struct suspend_tm_data* data = container_of(dev, struct suspend_tm_data, dev_f);
    uint32_t value = 0;

    pr_info("%d\n", (int32_t)data->to_ign_db);

    spin_lock_irqsave(&data->dev_lock, data->lock_flags); 
    value = data->to_ign_db;
    spin_unlock_irqrestore(&data->dev_lock, data->lock_flags);

    if(MAX_SUSPEND_TM <= value)//for PowerManagerService 
    {
        value = -1;
    }
    return sprintf(buf, "%d\n", value);
}

static void suspend_tm_delete(struct suspend_tm_data *data)
{

    device_remove_file(&data->dev_f, &data->dev_attr);
    device_remove_file(&data->dev_f, &data->dev_attr_ito);
    device_unregister(&data->dev_f);

    misc_deregister(&data->mdev);
    misc_deregister(&data->dev_ign_tm);
}

static struct suspend_tm_data *suspend_tm_create(struct platform_device *pdev)
{
    struct suspend_tm_data *data;
    int32_t err;

    pr_notice("\n");

    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if(!data)
        return ERR_PTR(-ENOMEM);

    spin_lock_init(&data->dev_lock);
    init_waitqueue_head(&data->wq_sp);
    init_completion(&data->completion_sp);
    init_waitqueue_head(&data->wq_ign_tm);
    init_completion(&data->completion_ign_tm);
///

    dev_set_name(&data->dev_f, "suspend_timeout");

    err = device_register(&data->dev_f);
    if(err) {
        dev_err(&pdev->dev, "could not register device, err %d\n", err);
        kfree(data);
        return ERR_PTR(err);
    }

    sysfs_attr_init(&data->dev_attr.attr);
    data->dev_attr.attr.name = "suspend_timeout";
    data->dev_attr.attr.mode = S_IRUGO | S_IWUGO;

    data->dev_attr.show = suspend_tm_show;
    data->dev_attr.store = suspend_tm_set;

    err = device_create_file(&data->dev_f, &data->dev_attr);
    if (err) {
        dev_err(&pdev->dev, "could not create sysfs %s file\n", data->dev_attr.attr.name);
        device_unregister(&data->dev_f);
        kfree(data);
        return ERR_PTR(err);
    }

    sysfs_attr_init(&data->dev_attr_ito.attr);
    data->dev_attr_ito.attr.name = "ignition_timeout";
    data->dev_attr_ito.attr.mode = S_IRUGO | S_IWUGO;

    data->dev_attr_ito.show = ignition_tm_show;
    data->dev_attr_ito.store = ignition_tm_set;

    err = device_create_file(&data->dev_f, &data->dev_attr_ito);
    if (err) {
        dev_err(&pdev->dev, "could not create sysfs %s file\n", data->dev_attr_ito.attr.name);
        device_unregister(&data->dev_f);
        device_remove_file(&data->dev_f, &data->dev_attr);
        kfree(data);
        return ERR_PTR(err);
    }

    data->mdev.minor    = MISC_DYNAMIC_MINOR;
    data->mdev.name	= "suspend_timeout";
    data->mdev.fops	= &suspend_tm_dev_fops;

//    pr_notice("%s: register %s\n", __func__, led->mdev.name);

    err = misc_register(&data->mdev);
    if(err) {
        device_remove_file(&data->dev_f, &data->dev_attr);
        device_remove_file(&data->dev_f, &data->dev_attr_ito);
        device_unregister(&data->dev_f);
        kfree(data);
        pr_err("%s: failure to register misc device, err %d\n", __func__, err);
        return ERR_PTR(err);
    }

    data->dev_ign_tm.minor  = MISC_DYNAMIC_MINOR;
    data->dev_ign_tm.name	= "ignition_timeout";
    data->dev_ign_tm.fops	= &ignition_tm_dev_fops;

//    pr_notice("%s: register %s\n", __func__, led->mdev.name);

    err = misc_register(&data->dev_ign_tm);
    if(err) {
        misc_deregister(&data->mdev);
        device_remove_file(&data->dev_f, &data->dev_attr);
        device_remove_file(&data->dev_f, &data->dev_attr_ito);
        device_unregister(&data->dev_f);
        kfree(data);
        pr_err("%s: failure to register misc device, err %d\n", __func__, err);
        return ERR_PTR(err);
    }


    return data;
}

static const struct of_device_id of_suspend_tm_match[] = {
	{ .compatible = "suspend-timeout", },
	{},
};

static int32_t suspend_tm_probe(struct platform_device *pdev)
{
    struct suspend_tm_data *pdat;

    pdat = suspend_tm_create(pdev);
    if (IS_ERR(pdat))
        return PTR_ERR(pdat);

    platform_set_drvdata(pdev, pdat);

    return 0;
}

static int32_t suspend_tm_remove(struct platform_device *pdev)
{
    struct suspend_tm_data *pdat = platform_get_drvdata(pdev);

    if (pdat) {
        suspend_tm_delete(pdat); 
        kfree(pdat);
    }

    platform_set_drvdata(pdev, 0);

    return 0;
}

static struct platform_driver suspend_tm_driver = {
	.probe		= suspend_tm_probe,
	.remove		= suspend_tm_remove,
	.driver		= {
		.name	= "suspend_timeout",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(of_suspend_tm_match),
	},
};

module_platform_driver(suspend_tm_driver);

MODULE_AUTHOR("Igor Lantsman <igor.lantsman@micronet-inc.com>");
MODULE_DESCRIPTION("Suspend and shutdown timeouts");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:suspend_tm");
