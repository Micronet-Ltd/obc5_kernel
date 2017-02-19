/*
 * Copyright (C) 2017 Micronet Inc, All Right Reserved.
 *
 * Athor: vladimir.zatulovsky@micronet-inc.com
 *
 */

#define pr_fmt(fmt) "%s: " fmt, __func__
//#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/switch.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

#define SWITCH_DOCK	(1 << 0)
#define SWITCH_IGN  (1 << 1)

struct dock_switch_device {
	struct  switch_dev sdev;
    struct  work_struct work;
	int     dock_pin;
    int 	ign_pin;
    int     usb_switch_pin;
    int     dock_irq;
    int 	ign_irq;
	int	    dock_active_l;
	int	    ign_active_l;
    unsigned sched_irq;
	int	    state;
};

static void dock_switch_work_func(struct work_struct *work) 
{
	struct dock_switch_device *ds  = container_of(work, struct dock_switch_device, work);
    int val;
	
    val = 0;
    pr_notice("pins[%d, %d]\n", ds->dock_pin, ds->ign_pin);
    pr_notice("irqs[%d, %d]\n", ds->dock_irq, ds->ign_irq);
    pr_notice("sched reason[%u]\n", ds->sched_irq);
    if (gpio_is_valid(ds->dock_pin)) {
		if (ds->dock_active_l == gpio_get_value(ds->dock_pin) ) {
            pr_notice("dock connected\n");
            val |= SWITCH_DOCK;
        } else {
            pr_notice("dock disconnected\n");
        }

        if (ds->sched_irq & SWITCH_DOCK) {
            pr_notice("enable dock monitor\n");
            ds->sched_irq &= ~SWITCH_DOCK;
            if (gpio_is_valid(ds->usb_switch_pin)) {
                pr_notice("switch usb\n");
                gpio_set_value(ds->usb_switch_pin, !!(ds->dock_active_l == gpio_get_value(ds->dock_pin)));
            }
            enable_irq(ds->dock_irq);
        }
	}

    if (gpio_is_valid(ds->ign_pin)) {
        if (ds->ign_active_l == gpio_get_value(ds->ign_pin) ) {
            pr_notice("ignition connected\n");
            val |= SWITCH_IGN;
        } else {
            pr_notice("ignition disconnected\n");
        }

        if (ds->sched_irq & SWITCH_IGN) {
            pr_notice("enable ignition monitor\n");
            ds->sched_irq &= ~SWITCH_IGN;
            enable_irq(ds->ign_irq); 
        }
	}

	if (ds->state != val) {
		ds->state = val;
		switch_set_state(&ds->sdev, val ? 1 : 0);
	}
}

static irqreturn_t dock_switch_irq_handler(int irq, void *arg)
{
	struct dock_switch_device *ds = (struct dock_switch_device *)arg;

    pr_notice("pins[%d]\n", irq);
    disable_irq_nosync(irq);

    if (irq == ds->dock_irq) {
        ds->sched_irq |= SWITCH_DOCK;
    }

    if (irq == ds->ign_irq) {
        ds->sched_irq |= SWITCH_IGN;
    }

    schedule_work(&ds->work);

	return IRQ_HANDLED;
}

static ssize_t dock_switch_print_state(struct switch_dev *sdev, char *buffer)
{
	struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);

	return sprintf(buffer, "%d", ds->state);
}

static int dock_switch_probe(struct platform_device *pdev)
{
	int err;
	struct dock_switch_device *ds;
    struct device *dev = &pdev->dev;
    struct device_node *np;
	
    np = dev->of_node;
    if (!np) {
        pr_err("failure to find device tree\n");
        return -EINVAL;
    }
	
	ds = devm_kzalloc(dev, sizeof(struct dock_switch_device), GFP_KERNEL);
	if (!ds)
		return -ENOMEM;

    do {
        err = of_get_named_gpio_flags(np, "treq5,dock-pin", 0, (enum of_gpio_flags *)&ds->dock_active_l);
        if (!gpio_is_valid(err)) {
            pr_err("ivalid docking pin\n");
            err = -EINVAL;
            break;
        }

        INIT_WORK(&ds->work, dock_switch_work_func);

        ds->dock_pin = err;
        ds->dock_active_l = !ds->dock_active_l;
        pr_notice("dock active level %s\n", (ds->dock_active_l)?"high":"low");
        err = of_get_named_gpio_flags(np, "treq5,ign-pin", 0, (enum of_gpio_flags *)&ds->ign_active_l);
        if (!gpio_is_valid(err)) {
            pr_err("ivalid ignition pin\n");
            err = -EINVAL;
            break;
        }
        ds->ign_pin = err;
        ds->ign_active_l = !ds->ign_active_l;
        pr_notice("ignition active level %s\n", (ds->ign_active_l)?"high":"low");

        if (gpio_is_valid(ds->dock_pin)) {
            err = devm_gpio_request(dev, ds->dock_pin, "dock-state");
            if (err < 0) {
                pr_err("failure to request the gpio[%d]\n", ds->dock_pin);
                break;
            }
            //err = gpio_direction_output(ds->dock_pin, 1);
            err = gpio_direction_input(ds->dock_pin);
            if (err < 0) {
                pr_err("failure to set direction of the gpio[%d]\n", ds->dock_pin);
                break;
            }
            gpio_export(ds->dock_pin, 1);
            if (1) {
                ds->dock_irq = gpio_to_irq(ds->dock_pin); 
                if (ds->dock_irq < 0) {
                    pr_err("failure to request gpio[%d] irq\n", ds->dock_pin);
                } else {
                    err = devm_request_irq(dev, ds->dock_irq, dock_switch_irq_handler,
                                           IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                           pdev->name, ds);
                    if (!err) {
                        disable_irq_nosync(ds->dock_irq);
                    } else {
                        pr_err("failure to request irq[%d] irq -- polling available\n", ds->dock_irq);
                    }
                }
            }

            ds->usb_switch_pin = of_get_named_gpio(np,"treq5,usb-switch-pin", 0);
            if (gpio_is_valid(ds->usb_switch_pin)) {
                err = devm_gpio_request(dev, ds->usb_switch_pin, "usb_switch");	
                if (err) {
                    pr_err("usb switch pin is busy!\n");
                } else {
                    gpio_direction_output(ds->usb_switch_pin, !!(ds->dock_active_l == gpio_get_value(ds->dock_pin)));
                    gpio_export(ds->usb_switch_pin, 0);
                }
            }
        }

        if (gpio_is_valid(ds->ign_pin)) {
            err = devm_gpio_request(dev, ds->ign_pin, "ignition-state");
            if (err < 0) {
                pr_err("failure to request the gpio[%d]\n", ds->ign_pin);
                break;
            }
            err = gpio_direction_input(ds->ign_pin);
//            err = gpio_direction_output(ds->ign_pin, 1);
            if (err < 0) {
                pr_err("failure to set direction of the gpio[%d]\n", ds->ign_pin);
                break;
            }
            gpio_export(ds->ign_pin, 1);
            if (1) {
                ds->ign_irq = gpio_to_irq(ds->ign_pin); 
                if (ds->ign_irq < 0) {
                    pr_err("failure to request gpio[%d] irq\n", ds->ign_pin);
                } else {
                    err = devm_request_irq(dev, ds->ign_irq, dock_switch_irq_handler,
                                           IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                           pdev->name, ds);
                    if (!err) {
                        disable_irq_nosync(ds->ign_irq);
                    } else {
                        pr_err("failure to request irq[%d] irq -- polling available\n", ds->ign_irq);
                    }
                }
            }
        }

        ds->sdev.name = "dock";
        ds->sdev.print_state = dock_switch_print_state;
        err = switch_dev_register(&ds->sdev);
        if (err < 0) {
            pr_err("err_register_switch\n");
            break;
        }

        dev_set_drvdata(dev, ds);
        ds->sched_irq = SWITCH_DOCK | SWITCH_IGN;
        pr_notice("sched reason[%u]\n", ds->sched_irq);
        schedule_work(&ds->work);

        pr_notice("registered\n");

        return 0;
    } while (0);

	if (ds->dock_irq)
        devm_free_irq(&pdev->dev, ds->dock_irq, ds);
    if (ds->ign_irq)
        devm_free_irq(&pdev->dev, ds->dock_irq, ds);
	if (gpio_is_valid(ds->dock_pin))
        devm_gpio_free(&pdev->dev, ds->dock_pin);
    if (gpio_is_valid(ds->ign_pin))
        devm_gpio_free(&pdev->dev, ds->ign_pin);
	devm_kfree(dev, ds);

	pr_err("failure\n");

    return 0;
}

static int dock_switch_remove(struct platform_device *pdev)
{
	struct dock_switch_device *ds = platform_get_drvdata(pdev);

    cancel_work_sync(&ds->work);

    switch_dev_unregister(&ds->sdev);

    device_wakeup_disable(&pdev->dev);
	if (ds->ign_irq) {
        disable_irq_nosync(ds->ign_irq);
		devm_free_irq(&pdev->dev, ds->ign_irq, ds);
    }
    if (ds->dock_irq) {
        disable_irq_nosync(ds->ign_irq);
        devm_free_irq(&pdev->dev, ds->dock_irq, ds);
    }

	if (gpio_is_valid(ds->dock_pin)) 
		devm_gpio_free(&pdev->dev, ds->dock_pin);
	if (gpio_is_valid(ds->ign_pin))
		devm_gpio_free(&pdev->dev, ds->ign_pin);

    dev_set_drvdata(&pdev->dev, 0);

	devm_kfree(&pdev->dev, ds);

    return 0;
}

static int dock_switch_suspend(struct device *dev)
{
	struct dock_switch_device *ds = dev_get_drvdata(dev);

	cancel_work_sync(&ds->work);

    if (device_may_wakeup(dev)) {
        if (ds->ign_irq) {
            enable_irq_wake(ds->ign_irq);
        }
        if (ds->dock_irq) {
            enable_irq_wake(ds->dock_irq);
        }
    }

    return 0;
}


static int dock_switch_resume(struct device *dev)
{
	struct dock_switch_device *ds = dev_get_drvdata(dev);

    if (device_may_wakeup(dev)) {
        if (ds->ign_irq) {
            disable_irq_nosync(ds->ign_irq);
            disable_irq_wake(ds->ign_irq);
        }
        if (ds->dock_irq) {
            disable_irq_nosync(ds->dock_irq);
            disable_irq_wake(ds->dock_irq);
        }
    }

    ds->sched_irq = SWITCH_DOCK | SWITCH_IGN;
	schedule_work(&ds->work);

	return 0;
}

static const struct dev_pm_ops dock_switch_pm_ops = {
	.suspend	= dock_switch_suspend,
	.resume		= dock_switch_resume,
};

static struct of_device_id dock_switch_match[] = {
	{ .compatible = "treq5,dock-switch", },
	{},
};

static struct platform_driver dock_switch_platform_driver = {
	.probe = dock_switch_probe,
	.remove = dock_switch_remove, 
	.driver = {
		.name = "dock",
		.owner = THIS_MODULE, 
        .of_match_table = dock_switch_match,
        .pm = &dock_switch_pm_ops,
	},
};

module_platform_driver(dock_switch_platform_driver);

MODULE_DESCRIPTION("Dock switch Monitor");
MODULE_AUTHOR("Vladimir Zatulovsky <vladimir.zatulovsky@micronet-inc.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:treqr5-dock-switch");

