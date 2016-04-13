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
#include <asm/uaccess.h>

struct vled_data {
    struct device           dev;
    struct led_classdev     cdev_r;
    struct led_classdev     cdev_g;
    struct led_classdev     cdev_b;
    struct miscdevice       mdev;
    unsigned long open;
    unsigned int r,g,b;
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
    struct miscdevice *miscdev =  file->private_data;
    struct vled_data *vleds = container_of(miscdev, struct vled_data, mdev);

	uint8_t output[16];

	if(count < sizeof(output)) {
		printk("%s: buffer too small\n", __func__);
		return -EINVAL;
	}

    // Vladimir:
    // The MCU LEDs have 8 colores only but led class operates by brightness and not relates to color
    // Here is simple conversion brightness to 8 color 3:3:2
    output[0] = 0;
    output[1] = vleds->r;
    output[2] = vleds->r & 0xE0;        // r
    output[3] = (vleds->r & 0x1C) << 3; // g
    output[4] = (vleds->r & 0x3) << 6;  // b
    output[5] = 1;
    output[6] = vleds->g;
    output[7] = vleds->g & 0xE0;        // r
    output[8] = (vleds->g & 0x1C) << 3; // g
    output[9] = (vleds->g & 0x3) << 6;  // b
    output[10] = 2;
    output[11] = vleds->b;
    output[12] = vleds->b & 0xE0;        // r
    output[13] = (vleds->b & 0x1C) << 3; // g
    output[14] = (vleds->b & 0x3) << 6;  // b
    output[15] = 0;

    if(copy_to_user(buf, output, sizeof(output)))
        return -EINVAL;
    *ppos = 0;

    printk("%s: succeed\n", __func__);

    return 0;
}

static ssize_t vleds_write(struct file * file, const char __user * buf, size_t count, loff_t * ppos)
{
    // Nothing to do, led class return current brightness value

	return 0;
}

static const struct file_operations vleds_dev_fops = {
	.owner      = THIS_MODULE,
	.llseek     = no_llseek,
	.read       = vleds_read,
	.write      = vleds_write,
	.open       = vleds_open,
	.release    = vleds_release,
//	.poll       = vleds_poll,
};

static void vled_set(struct led_classdev *led_cdev, enum led_brightness value)
{
    struct vled_data * vleds;

    if (0 == strncmp(led_cdev->name, "vled0", strlen("vled0"))) {
        vleds = container_of(led_cdev, struct vled_data, cdev_r);
        vleds->r = (int)value;
    } else if (0 == strncmp(led_cdev->name, "vled1", strlen("vled1"))) {
        vleds = container_of(led_cdev, struct vled_data, cdev_g);
        vleds->g = (int)value;
    } else if (0 == strncmp(led_cdev->name, "vled2", strlen("vled2"))) {
        vleds = container_of(led_cdev, struct vled_data, cdev_b);
        vleds->b = (int)value;
    }
}

static void vled_delete(struct vled_data *led)
{
    led_classdev_unregister(&led->cdev_r);
    led_classdev_unregister(&led->cdev_g);
    led_classdev_unregister(&led->cdev_b);

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

    led->cdev_r.name = "vled0";
    led->cdev_r.default_trigger = "none";
    led->cdev_r.brightness_set = vled_set;
    led->cdev_r.brightness = LED_OFF;
    led->cdev_r.flags |= LED_CORE_SUSPENDRESUME;

    printk("%s: register %s\n", __func__, led->cdev_r.name);

    err = led_classdev_register(pdev->dev.parent, &led->cdev_r);
    if (err < 0) {
        kfree(led);
        return ERR_PTR(err);
    }

    led->cdev_g.name = "vled1";
    led->cdev_g.default_trigger = "none";
    led->cdev_g.brightness_set = vled_set;
    led->cdev_g.brightness = LED_OFF;
    led->cdev_g.flags |= LED_CORE_SUSPENDRESUME;


    printk("%s: register %s\n", __func__, led->cdev_g.name);

    err = led_classdev_register(pdev->dev.parent, &led->cdev_g);
    if (err < 0) {
        led_classdev_unregister(&led->cdev_r);
        kfree(led);
        return ERR_PTR(err);
    }

    led->cdev_b.name = "vled2";
    led->cdev_b.default_trigger = "none";
    led->cdev_b.brightness_set = vled_set;
    led->cdev_b.brightness = LED_OFF;
    led->cdev_b.flags |= LED_CORE_SUSPENDRESUME;

    printk("%s: register %s\n", __func__, led->cdev_b.name);

    err = led_classdev_register(pdev->dev.parent, &led->cdev_b);
    if (err < 0) {
        led_classdev_unregister(&led->cdev_r);
        led_classdev_unregister(&led->cdev_b);
        kfree(led);
        return ERR_PTR(err);
    }

    led->mdev.minor = MISC_DYNAMIC_MINOR;
    led->mdev.name	= "vleds";
    led->mdev.fops	= &vleds_dev_fops;

    printk("%s: register %s\n", __func__, led->mdev.name);

    err = misc_register(&led->mdev);
    if(err) {
        led_classdev_unregister(&led->cdev_r);
        led_classdev_unregister(&led->cdev_g);
        led_classdev_unregister(&led->cdev_b);
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
