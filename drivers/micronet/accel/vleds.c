/*
 * MCU light LEDs driver
 *
 * Copyright 2016 Micronet Inc., vladimir.zatulovsky@micronet-inc.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <asm/uaccess.h>

struct vled_data {
    struct device           dev;
    struct led_classdev     cdev_0;
    struct led_classdev     cdev_1;
    struct miscdevice       mdev;
    unsigned long open;
    unsigned int c, prev_led0, prev_led1;
    struct mutex vled_lock;
};

static int vleds_open(struct inode *inode, struct file *file)
{
    struct miscdevice *miscdev =  file->private_data;
	struct vled_data *vleds = container_of(miscdev, struct vled_data, mdev);

	if(test_and_set_bit(1, &vleds->open))
		return -EPERM;

	return 0;
}

static int vleds_release(struct inode *inode, struct file *file)
{
    struct miscdevice *miscdev =  file->private_data;
    struct vled_data *vleds = container_of(miscdev, struct vled_data, mdev);

	clear_bit(1, &vleds->open);

	return 0;
}

static ssize_t vleds_read(struct file * file, char __user * buf, size_t count, loff_t *ppos)
{
    int i;
    struct miscdevice *miscdev =  file->private_data;
    struct vled_data *vleds = container_of(miscdev, struct vled_data, mdev);

	uint8_t output[16];

    if (0 == vleds->open) {
        return -EINVAL;
    }

    if (count < sizeof(output)) {
		printk("%s: buffer too small\n", __func__);
		return -EINVAL;
	}

    // Vladimir:
    // The MCU LEDs have 8 colores only by static PMGD780SN control or full color by switchable PMGD780SN control
    // but led class operates by brightness and not relates to color. Here is simple conversion brightness to color
    // Using:
    // echo llrrggbb > /sys/class/leds/vled[n]/brightness
    //      | | | |                        |
    //      [-8-bit every field]        led number
    //  ll is brightness
    //
    output[0] = 0;
    output[1] = (uint8_t)((vleds->cdev_0.brightness & 0x7F000000) >> 24);  // brightness
    output[2] = (uint8_t)((vleds->cdev_0.brightness & 0xFF0000) >> 16);    // r
    output[3] = (uint8_t)((vleds->cdev_0.brightness & 0xFF00) >> 8);       // g
    output[4] = (uint8_t)(vleds->cdev_0.brightness & 0xFF);              // b
    output[5] = 1;
    output[6] = (uint8_t)((vleds->cdev_1.brightness & 0x7F000000) >> 24);  // brightness
    output[7] = (uint8_t)((vleds->cdev_1.brightness & 0xFF0000) >> 16);    // r
    output[8] = (uint8_t)((vleds->cdev_1.brightness & 0xFF00) >> 8);       // g
    output[9] = (uint8_t)(vleds->cdev_1.brightness & 0xFF);              // b

    printk("%s: out data\n", __func__);
    for (i = 0; i < sizeof(output); i++) {
        printk("%X ", output[i]);
    }
    printk("\n");

    mutex_lock(&vleds->vled_lock); 
    output[15] = vleds->c;
    vleds->c = -1;
    mutex_unlock(&vleds->vled_lock);

    if(copy_to_user(buf, output, sizeof(output)))
        return -EINVAL;
    *ppos = 0;

//    printk("%s: succeed\n", __func__);

    return 0;
}

static ssize_t vleds_write(struct file * file, const char __user * buf, size_t count, loff_t * ppos)
{
    // Nothing to do, led class return current brightness value

	return 0;
}

static unsigned int vleds_poll(struct file * file,  poll_table * wait)
{
    struct miscdevice *miscdev =  file->private_data;
    struct vled_data *vleds = container_of(miscdev, struct vled_data, mdev);
	unsigned int mask = 0;

    mutex_lock(&vleds->vled_lock);
//	poll_wait(file, &vleds->waitq, wait);
	if(-1 != vleds->c)
		mask |= POLLIN | POLLRDNORM;
    mutex_unlock(&vleds->vled_lock);

	return mask;
}

static const struct file_operations vleds_dev_fops = {
	.owner      = THIS_MODULE,
	.llseek     = no_llseek,
	.read       = vleds_read,
	.write      = vleds_write,
	.open       = vleds_open,
	.release    = vleds_release,
	.poll       = vleds_poll,
};

static void vled_set(struct led_classdev *led_cdev, enum led_brightness value)
{
    unsigned int l, *prev;
    struct vled_data * vleds;

    printk("%s: %X\n", __func__, value);

    if (0 == strncmp(led_cdev->name, "vled0", strlen("vled0"))) {
        vleds = container_of(led_cdev, struct vled_data, cdev_0);
        prev = &vleds->prev_led0;
        l = 0;
    } else /*if (0 == strncmp(led_cdev->name, "vled1", strlen("vled1")))*/ {
        vleds = container_of(led_cdev, struct vled_data, cdev_1);
        prev = &vleds->prev_led1;
        l = 1;
    }

    mutex_lock(&vleds->vled_lock);
    if (*prev != value) {
        *prev = (int)value;
        vleds->c = l;
    }
    mutex_unlock(&vleds->vled_lock);
}

static void vled_delete(struct vled_data *led)
{
    led_classdev_unregister(&led->cdev_0);
    led_classdev_unregister(&led->cdev_1);

    misc_deregister(&led->mdev);
}

#ifdef CONFIG_OF
static struct vled_data *vled_create_of(struct platform_device *pdev)
{
	struct vled_data *led;
	int err;

	led = devm_kzalloc(&pdev->dev, sizeof(*led), GFP_KERNEL);
	if (!led)
		return ERR_PTR(-ENOMEM);

    led->prev_led0 = led->prev_led1 = led->c = -1;
    mutex_init(&led->vled_lock);
    led->cdev_0.name = "vled0";
    led->cdev_0.default_trigger = "none";
    led->cdev_0.brightness_set = vled_set;
    led->cdev_0.brightness = LED_OFF;
    led->cdev_0.max_brightness = 0x7FFFFFFF;
    led->cdev_0.flags |= LED_CORE_SUSPENDRESUME;

    printk("%s: register %s\n", __func__, led->cdev_0.name);

    err = led_classdev_register(pdev->dev.parent, &led->cdev_0);
    if (err < 0) {
        kfree(led);
        return ERR_PTR(err);
    }

    led->cdev_1.name = "vled1";
    led->cdev_1.default_trigger = "none";
    led->cdev_1.brightness_set = vled_set;
    led->cdev_1.brightness = LED_OFF;
    led->cdev_1.max_brightness = 0x7FFFFFFF;
    led->cdev_1.flags |= LED_CORE_SUSPENDRESUME;


    printk("%s: register %s\n", __func__, led->cdev_1.name);

    err = led_classdev_register(pdev->dev.parent, &led->cdev_1);
    if (err < 0) {
        led_classdev_unregister(&led->cdev_1);
        kfree(led);
        return ERR_PTR(err);
    }

    led->mdev.minor = MISC_DYNAMIC_MINOR;
    led->mdev.name	= "vleds";
    led->mdev.fops	= &vleds_dev_fops;

    printk("%s: register %s\n", __func__, led->mdev.name);

    err = misc_register(&led->mdev);
    if(err) {
        led_classdev_unregister(&led->cdev_1);
        led_classdev_unregister(&led->cdev_1);
        kfree(led);
        printk("%s: failure to register misc device \n", __func__);
        return ERR_PTR(err);
    }

	return led;
}

static const struct of_device_id of_vled_match[] = {
	{ .compatible = "virtual-leds", },
	{},
};
#else
static struct vled_data *vled_create_of(struct platform_device *pdev)
{
	return ERR_PTR(-ENODEV);
}
#endif


static int vled_probe(struct platform_device *pdev)
{
	struct vled_data *pdat;

    pdat = vled_create_of(pdev);
    if (IS_ERR(pdat))
        return PTR_ERR(pdat);

	platform_set_drvdata(pdev, pdat);

	return 0;
}

static int vled_remove(struct platform_device *pdev)
{
	struct vled_data *pdat = platform_get_drvdata(pdev);

    if (pdat) {
        vled_delete(pdat); 
        kfree(pdat);
    }

	platform_set_drvdata(pdev, 0);

	return 0;
}

static struct platform_driver vled_driver = {
	.probe		= vled_probe,
	.remove		= vled_remove,
	.driver		= {
		.name	= "vleds",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(of_vled_match),
	},
};

module_platform_driver(vled_driver);

MODULE_AUTHOR("Vladimir Zatulovsky <vladimir.zatulovsky@micronet-inc.com>");
MODULE_DESCRIPTION("MCU host interface based LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:vleds");
