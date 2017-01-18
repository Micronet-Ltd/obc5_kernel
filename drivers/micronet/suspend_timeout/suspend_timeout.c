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

#define MIN_SUSPEND_TM  10000//from Android
#define MAX_SUSPEND_TM  0x7FFFFFFF//from Android

//device_create_file
struct suspend_tm_data {
    struct device           dev;
    struct device           dev_f;
    struct miscdevice       mdev;
    struct miscdevice       dev_ign_tm;
    struct miscdevice       dev_ign_value;
    unsigned long           open;
    //unsigned long           open_ign;
    uint32_t                to;
    uint32_t                to_db;
    uint32_t                to_ign;
    uint32_t                to_ign_db;
    uint32_t                ign_val;
    int32_t                 c;
    int32_t                 c_ign;
    struct device_attribute dev_attr;
    struct device_attribute dev_attr_ito;
    struct device_attribute dev_attr_ign;
    spinlock_t              dev_lock;
    unsigned long           lock_flags;
    wait_queue_head_t       wq;
    struct completion       completion;
    wait_queue_head_t       wq_ign;
    struct completion       completion_ign;
};

static int test_timeout(uint32_t val)
{
    if((-1 != (int32_t)val) && (MIN_SUSPEND_TM > val || MAX_SUSPEND_TM < val) )
    {
        return 0;
    }
    return 1;
}
////
static int ignition_tm_open(struct inode *inode, struct file *file)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, dev_ign_tm);

    pr_notice("\n");
    if(test_and_set_bit(2, &data->open))
	return -EPERM;

    return 0;
}
static int ignition_tm_release(struct inode *inode, struct file *file)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, dev_ign_tm);

    pr_notice("\n");
    complete(&data->completion_ign);

    clear_bit(2, &data->open);

    return 0;
}
static ssize_t ignition_tm_read(struct file * file, char __user * buf, size_t count, loff_t *ppos)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, dev_ign_tm);

    uint8_t output[32];

    pr_info("+\n");
    if (0 == data->open) {//test_bit
        return -EINVAL;
    }
    spin_lock_irqsave(&data->dev_lock, data->lock_flags);
    sprintf(output, "%d\n", data->to_ign);
    data->c_ign  = 0;
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

    if (0 == data->open) {//test_bit
        return -EINVAL;
    }
    if(count > sizeof(output)) {
        pr_err("count %d too big\n", (int)count);
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
    complete(&data->completion_ign);
    return count;
}

static unsigned int ignition_tm_poll(struct file * file,  poll_table * wait)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, dev_ign_tm);

    unsigned int mask = 0;

    poll_wait(file, &data->wq_ign, wait);
    spin_lock_irqsave(&data->dev_lock, data->lock_flags);
	if(0 != data->c_ign)
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
static int suspend_tm_open(struct inode *inode, struct file *file)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, mdev);

    pr_notice("\n");
    if(test_and_set_bit(1, &data->open))
	return -EPERM;

    return 0;
}
static int suspend_tm_release(struct inode *inode, struct file *file)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, mdev);

    complete(&data->completion);

    pr_notice("\n");
    clear_bit(1, &data->open);

    return 0;
}
static ssize_t suspend_tm_read(struct file * file, char __user * buf, size_t count, loff_t *ppos)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, mdev);

    uint8_t output[32];

    pr_info("+\n");
    if (0 == data->open) { //test_bit
        return -EINVAL;
    }
    spin_lock_irqsave(&data->dev_lock, data->lock_flags);
    sprintf(output, "%d\n", data->to);
    data->c  = 0;
    spin_unlock_irqrestore(&data->dev_lock, data->lock_flags);

    if(copy_to_user(buf, output, strlen(output)))
        return -EINVAL;

    pr_info(": %s\n",output);

    return strlen(output);//-'\n'
}
static ssize_t suspend_tm_write(struct file * file, const char __user * buf, size_t count, loff_t * ppos)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, mdev);

    uint8_t output[32] = {0};
    uint32_t val;

    pr_info("\n");

    if (0 == data->open) {
        return -EINVAL;
    }
    if(count > sizeof(output)) {
        pr_err("count %d too big\n", (int)count);
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
    data->to = data->to_db;
    spin_unlock_irqrestore(&data->dev_lock, data->lock_flags);
    complete(&data->completion);
    return count;
}

static unsigned int suspend_tm_poll(struct file * file,  poll_table * wait)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, mdev);

    unsigned int mask = 0;

    poll_wait(file, &data->wq, wait);
    spin_lock_irqsave(&data->dev_lock, data->lock_flags);
	if(0 != data->c)
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
static int ign_open(struct inode *inode, struct file *file)
{
    return 0;
}
static int ign_release(struct inode *inode, struct file *file)
{
    return 0;
}
static ssize_t ign_read(struct file * file, char __user * buf, size_t count, loff_t *ppos)
{
    return 0;
}
static ssize_t ign_write(struct file * file, const char __user * buf, size_t count, loff_t * ppos)
{
    struct miscdevice *miscdev =  file->private_data;
    struct suspend_tm_data *data = container_of(miscdev, struct suspend_tm_data, dev_ign_value);

    uint8_t output[32] = {0};
    uint32_t val;

    pr_notice("\n");

//    if (0 == data->open) {
//        return -EINVAL;
//    }
    if(count > sizeof(output)) {
        pr_err("count %d too big\n", (int)count);
        return -EINVAL;
    }

    if(copy_from_user(output, buf, min(count, sizeof(output))))
        return -EACCES;

    val = simple_strtol(output, 0, 10);
    pr_notice("output %s; val %d\n", output, val);

    data->ign_val = val;
    return count;
}
//static unsigned int ign_poll(struct file * file,  poll_table * wait)
//{
//    return 0;
//}
static const struct file_operations ign_dev_fops = {
	.owner      = THIS_MODULE,
	.llseek     = no_llseek,
	.read       = ign_read,
	.write      = ign_write,
	.open       = ign_open,
	.release    = ign_release,
//	.poll       = ign_poll,
};

static ssize_t suspend_tm_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int err;
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
    pr_info("%d (count %u)\n", value, (unsigned int)count);

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

    init_completion(&data->completion);

    spin_lock_irqsave(&data->dev_lock, data->lock_flags); 
    data->to = value;
    data->c  = 1;    
    spin_unlock_irqrestore(&data->dev_lock, data->lock_flags);

    wake_up(&data->wq);
    err = wait_for_completion_interruptible(&data->completion);
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
    int err;
    uint32_t value;
    struct suspend_tm_data* data = container_of(dev, struct suspend_tm_data, dev_f);

    value = simple_strtol(buf, 0, 10); 
    pr_info("%d (count %u)\n", value, (unsigned int)count);

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
    if(data->to_ign_db == value)
        return count;

    init_completion(&data->completion_ign);

    spin_lock_irqsave(&data->dev_lock, data->lock_flags); 
    data->to_ign = value;
    data->c_ign  = 1;    
    spin_unlock_irqrestore(&data->dev_lock, data->lock_flags);

    wake_up(&data->wq_ign);
    err = wait_for_completion_interruptible(&data->completion_ign);
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
static ssize_t ignition_show(struct device* dev, struct device_attribute *attr, char *buf) 
{
    struct suspend_tm_data* data = container_of(dev, struct suspend_tm_data, dev_f);
    pr_info("%s: %u\n", __func__, data->ign_val);
    return sprintf(buf, "%d\n", (int32_t)data->ign_val);
}

static void suspend_tm_delete(struct suspend_tm_data *data)
{
    device_remove_file(&data->dev_f, &data->dev_attr);
    device_remove_file(&data->dev_f, &data->dev_attr_ito);
    device_remove_file(&data->dev_f, &data->dev_attr_ign);
    device_unregister(&data->dev_f);

    misc_deregister(&data->mdev);
    misc_deregister(&data->dev_ign_tm);
    misc_deregister(&data->dev_ign_value);
}

static struct suspend_tm_data *suspend_tm_create(struct platform_device *pdev)
{
    struct suspend_tm_data *data;
    int err;

    pr_notice("%s\n", __func__);

    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if(!data)
        return ERR_PTR(-ENOMEM);

    spin_lock_init(&data->dev_lock);
    init_waitqueue_head(&data->wq);
    init_completion(&data->completion);
    init_waitqueue_head(&data->wq_ign);
    init_completion(&data->completion_ign);
    //data->dev_f.parent = &pdev->dev;
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
    sysfs_attr_init(&data->dev_attr_ign.attr);
    data->dev_attr_ign.attr.name = "ignition_value";
    data->dev_attr_ign.attr.mode = S_IRUGO;

    data->dev_attr_ign.show = ignition_show;
    data->dev_attr_ign.store = NULL;

    err = device_create_file(&data->dev_f, &data->dev_attr_ign);
    if (err) {
        dev_err(&pdev->dev, "could not create sysfs %s file\n", data->dev_attr_ign.attr.name);
        device_remove_file(&data->dev_f, &data->dev_attr);
        device_remove_file(&data->dev_f, &data->dev_attr_ito);
        device_unregister(&data->dev_f);
        kfree(data);
        return ERR_PTR(err);
    }

    data->mdev.minor = MISC_DYNAMIC_MINOR;
    data->mdev.name	= "suspend_timeout";
    data->mdev.fops	= &suspend_tm_dev_fops;

//    pr_notice("%s: register %s\n", __func__, led->mdev.name);

    err = misc_register(&data->mdev);
    if(err) {
        device_remove_file(&data->dev_f, &data->dev_attr);
        device_remove_file(&data->dev_f, &data->dev_attr_ito);
        device_remove_file(&data->dev_f, &data->dev_attr_ign);
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
        device_remove_file(&data->dev_f, &data->dev_attr_ign);
        device_unregister(&data->dev_f);
        kfree(data);
        pr_err("%s: failure to register misc device, err %d\n", __func__, err);
        return ERR_PTR(err);
    }

    data->dev_ign_value.minor = MISC_DYNAMIC_MINOR;
    data->dev_ign_value.name	= "ignition_value";
    data->dev_ign_value.fops	= &ign_dev_fops;

//    pr_notice("%s: register %s\n", __func__, led->mdev.name);

    err = misc_register(&data->dev_ign_value);
    if(err) {
        misc_deregister(&data->mdev);
        misc_deregister(&data->dev_ign_tm);
        device_remove_file(&data->dev_f, &data->dev_attr);
        device_remove_file(&data->dev_f, &data->dev_attr_ito);
        device_remove_file(&data->dev_f, &data->dev_attr_ign);
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

static int suspend_tm_probe(struct platform_device *pdev)
{
    struct suspend_tm_data *pdat;

    pdat = suspend_tm_create(pdev);
    if (IS_ERR(pdat))
        return PTR_ERR(pdat);

    platform_set_drvdata(pdev, pdat);

    return 0;
}

static int suspend_tm_remove(struct platform_device *pdev)
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
