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
    unsigned long           open;
    uint32_t                to;
    uint32_t                to_db;
    uint32_t                to_tmp;
    int32_t                 c;
    struct device_attribute dev_attr;
    struct device_attribute dev_attr_tmp;
    spinlock_t              dev_lock;
    unsigned long           lock_flags;
    wait_queue_head_t       wq;
    struct completion       completion;
};

static int test_timeout(uint32_t val)
{
    if((-1 != (int32_t)val) && (MIN_SUSPEND_TM > val || MAX_SUSPEND_TM < val) )
    {
        return 0;
    }
    return 1;
}

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
    if (0 == data->open) {
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
static ssize_t suspend_tm_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
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
    wait_for_completion(&data->completion);

    return count;
}

static ssize_t suspend_tm_show(struct device* dev, struct device_attribute *attr, char *buf) 
{
    struct suspend_tm_data* data = container_of(dev, struct suspend_tm_data, dev_f);
    uint32_t value = 0;

    pr_notice("%d\n", (int32_t)data->to);

    spin_lock_irqsave(&data->dev_lock, data->lock_flags); 
    value = data->to_db;
    spin_unlock_irqrestore(&data->dev_lock, data->lock_flags);

    if(MAX_SUSPEND_TM <= value)//for PowerManagerService 
    {
        value = -1;
    }
    return sprintf(buf, "%d\n", value);
}
static ssize_t suspend_tm_tmp_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    uint32_t value;
    struct suspend_tm_data* data = container_of(dev, struct suspend_tm_data, dev_f);

    pr_info("%s (count %u)\n", buf, (unsigned int)count);
    if('\n' == buf[0]) {
        return count;//skip
    }
    value = simple_strtol(buf, 0, 10); 
    data->to_tmp = value;

    return count;
}

static ssize_t suspend_tm_tmp_show(struct device* dev, struct device_attribute *attr, char *buf) 
{
    struct suspend_tm_data* data = container_of(dev, struct suspend_tm_data, dev_f);
    pr_info("%s: %u\n", __func__, data->to_tmp);
    return sprintf(buf, "%d\n", (int32_t)data->to_tmp);
}
static void suspend_tm_delete(struct suspend_tm_data *data)
{
    device_remove_file(&data->dev_f, &data->dev_attr);
    device_remove_file(&data->dev_f, &data->dev_attr_tmp);
    device_unregister(&data->dev_f);

    misc_deregister(&data->mdev);
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
    //data->dev_f.parent = &pdev->dev;
    dev_set_name(&data->dev_f, "suspend_timeout");

    err = device_register(&data->dev_f);
    if(err) {
        dev_err(&pdev->dev, "could not register device, err %d\n", err);
        kfree(data);
        return ERR_PTR(err);
    }

    sysfs_attr_init(&data->dev_attr.attr);
    data->dev_attr.attr.name = "suspend_timeout_value";
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
    sysfs_attr_init(&data->dev_attr_tmp.attr);
    data->dev_attr_tmp.attr.name = "suspend_timeout_tmp";
    data->dev_attr_tmp.attr.mode = S_IRUGO | S_IWUGO;

    data->dev_attr_tmp.show = suspend_tm_tmp_show;
    data->dev_attr_tmp.store = suspend_tm_tmp_set;

    err = device_create_file(&data->dev_f, &data->dev_attr_tmp);
    if (err) {
        dev_err(&pdev->dev, "could not create sysfs %s file\n", data->dev_attr_tmp.attr.name);
        device_unregister(&data->dev_f);
        device_remove_file(&data->dev_f, &data->dev_attr);
        kfree(data);
        return ERR_PTR(err);
    }

    data->mdev.minor = MISC_DYNAMIC_MINOR;
    data->mdev.name	= "suspend_timeout";
    data->mdev.fops	= &suspend_tm_dev_fops;

//    pr_notice("%s: register %s\n", __func__, led->mdev.name);

    err = misc_register(&data->mdev);
    if(err) {
        device_unregister(&data->dev_f);
        device_remove_file(&data->dev_f, &data->dev_attr);
        device_remove_file(&data->dev_f, &data->dev_attr_tmp);
        kfree(data);
        pr_notice("%s: failure to register misc device, err %d\n", __func__, err);
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
MODULE_DESCRIPTION("MCU host interface based LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:suspend_tm");
