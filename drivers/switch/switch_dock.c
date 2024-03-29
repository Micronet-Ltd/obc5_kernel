/*
 * Copyright (C) 2017 Micronet Inc, All Right Reserved.
 *
 * Athor: vladimir.zatulovsky@micronet-inc.com
 *
 */

#define pr_fmt(fmt) "%s: " fmt, __func__
#include <linux/kconfig.h>
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
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>

#include <linux/notifier.h>
#include <linux/power_supply.h>

#if defined  (CONFIG_PRODUCT_A3001)
static int32_t gpio_in_register_notifier(struct notifier_block *nb)
{
    return 0;
}
#else
extern int32_t gpio_in_register_notifier(struct notifier_block *nb);
#endif

#define SWITCH_DOCK	(1 << 0)
#define SWITCH_IGN  (1 << 1)
#define SWITCH_EDOCK (1 << 2)
#define SWITCH_ODOCK (1 << 3)

#define DEBOUNCE_INTERIM  200 //500
#define PATERN_INTERIM    100
#define BASIC_PATTERN     0
#define SMART_PATTERN     1000
#define IG_HI_PATTERN     500
#define IG_LOW_PATTERN    100

#define VIRT_GPIO_OFF	0
#define VIRT_GPIO_INIT	1
#define VIRT_GPIO_ON	2

#define FORBID_EXT_SPKR	9
enum e_dock_type {
    e_dock_type_unspecified = -1,
    e_dock_type_basic,
    e_dock_type_smart
};

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
    struct  wake_lock wlock;
	struct  device*	pdev;
    struct 	notifier_block   ignition_notifier;
    int		virt_gpio_offset;
    int     virt_init;
    enum e_dock_type dock_type;
    struct power_supply *usb_psy;
    int 	ampl_enable;
};
/////////////
static DEFINE_MUTEX(ampl_lock);
static void set_aml_enable(struct dock_switch_device *ds, int val)
{
	if (!gpio_is_valid(ds->dock_pin)) {
		return;
	}

	mutex_lock(&ampl_lock);

	ds->ampl_enable = val;
	pr_info("set %d\n", val);

	if(FORBID_EXT_SPKR == val) {
		gpio_direction_input(ds->dock_pin);
	}
	else {//if(0 == val) {
        gpio_direction_output(ds->dock_pin, !val);
	}

	mutex_unlock(&ampl_lock);

}
static ssize_t ampl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
	struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);

	unsigned long val;

	if (!ds || !gpio_is_valid(ds->dock_pin)) {
		return -EINVAL;
	}

	mutex_lock(&ampl_lock);

	if (FORBID_EXT_SPKR != ds->ampl_enable && kstrtol(buf, 10, &val) == 0 && (val == 0 || val == 1)) {
		pr_info("val %lu (%d)\n", val, ds->ampl_enable);
		if(val != ds->ampl_enable) {
			ds->ampl_enable = val;
			gpio_set_value(ds->dock_pin, !val);
		}
	}else {
		mutex_unlock(&ampl_lock);
		return -EINVAL;
	}
	mutex_unlock(&ampl_lock);

	return count;
}
static ssize_t ampl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
	struct dock_switch_device *ds = container_of(sdev, struct dock_switch_device, sdev);

	if (!ds || !gpio_is_valid(ds->dock_pin)) {
		return 0;
	}
	return sprintf(buf, "%d\n", ds->ampl_enable);
}

static DEVICE_ATTR(ampl_enable, S_IRUGO | S_IWUGO, ampl_show, ampl_store);
/////////////
static int wait_for_stable_signal(int pin, int interim)
{
    long long timer;
    int pulses = 0, state = 0;

//    long long start_time = 0, end_time = 0;

    timer = ktime_to_ms(ktime_get()) + interim;

    if (gpio_is_valid(pin)) {
//        pr_notice("start %d pulses %lld\n", gpio_get_value(pin), ktime_to_ms(ktime_get()));
        do {
            if (state != gpio_get_value(pin)) {
                state ^= 1;
//                pr_notice("detcted %d pulses %lld\n", pulses, ktime_to_ms(ktime_get()));
                pulses++;
                ///temp!!!
//                if(1==pulses)
//                	start_time = ktime_to_ms(ktime_get());
//               	end_time = ktime_to_ms(ktime_get());
            }
        } while (ktime_to_ms(ktime_get()) < timer);
    }
    pulses >>= 1;

    pr_notice("detcted %d pulses %lld\n", pulses, ktime_to_ms(ktime_get()));
 //   pr_notice("detcted %d pulses %lld (delta %lld)\n", pulses, ktime_to_ms(ktime_get()), (end_time - start_time));
    return pulses;
}

static inline int pulses2freq(int pulses, int interim)
{
    pr_notice("%d HZ %lld\n", 1000 * pulses / interim, ktime_to_ms(ktime_get()));
    return 1000 * pulses / interim;
}

#define HYST_PATTERN(p1, p2) ((p2) - (((p2) - (p1)) >> 1))
//#define HYST_PATTERN(p1, p2) ((p1) + (((p2) - (p1)) >> 2))
static inline int freq2pattern(int freq)
{
    if (freq < IG_LOW_PATTERN - pulses2freq(2, PATERN_INTERIM)) {//HYST_PATTERN(BASIC_PATTERN, IG_LOW_PATTERN)) {
        return BASIC_PATTERN;
    } else if (freq <= HYST_PATTERN(IG_LOW_PATTERN, IG_HI_PATTERN) && IG_LOW_PATTERN - pulses2freq(2, PATERN_INTERIM) <= freq) {
        return IG_LOW_PATTERN;
    } else if (freq <= HYST_PATTERN(IG_HI_PATTERN, SMART_PATTERN) && HYST_PATTERN(IG_LOW_PATTERN, IG_HI_PATTERN) < freq) {
        return IG_HI_PATTERN;
    }

    return SMART_PATTERN; 
}

static void dock_switch_work_func(struct work_struct *work) 
{
	struct dock_switch_device *ds = container_of(work, struct dock_switch_device, work);
    long long timer = ktime_to_ms(ktime_get());
    int val = 0, act = 0;
    union power_supply_propval prop = {0,};
    struct irq_desc *desc;

    if (e_dock_type_basic != ds->dock_type) {
        val = wait_for_stable_signal(ds->ign_pin, DEBOUNCE_INTERIM + PATERN_INTERIM);
        val = pulses2freq(val, PATERN_INTERIM);
        val = freq2pattern(val);
        pr_notice("pattern[%d, %d] [%lld]%lld\n", val, gpio_get_value(ds->ign_pin), timer, ktime_to_ms(ktime_get()));
       	if (BASIC_PATTERN == val) {
            if (gpio_is_valid(ds->dock_pin)) {
				if (e_dock_type_smart == ds->dock_type && ds->ign_active_l != gpio_get_value(ds->ign_pin)) {
					pr_notice("smart cradle unplagged %lld [dock_pin %d]\n", ktime_to_ms(ktime_get()), gpio_get_value(ds->dock_pin));
					if (ds->usb_psy) {
						prop.intval = 0;
						ds->usb_psy->set_property(ds->usb_psy, POWER_SUPPLY_PROP_CHARGE_ENABLED, &prop);
					}
					ds->dock_type = e_dock_type_unspecified;
					ds->sched_irq |= SWITCH_DOCK;
					act = 1;
				} else if (e_dock_type_smart != ds->dock_type && ds->dock_active_l == gpio_get_value(ds->dock_pin) ){
					pr_notice("basic cradle attempt to be plugged %lld  [dock_pin %d]\n", ktime_to_ms(ktime_get()), gpio_get_value(ds->dock_pin));
					ds->dock_type = e_dock_type_basic;
					act = 1;
				}

				if(act) {//e_dock_type_basic == ds->dock_type || ds->ign_active_l != gpio_get_value(ds->ign_pin)) {
		            act = 0;
					val = 0;
					// pin function is basic dock detect
					pr_notice("enable dock detect function %lld\n", ktime_to_ms(ktime_get()));
					set_aml_enable(ds, FORBID_EXT_SPKR);//
					//gpio_direction_input(ds->dock_pin);
					// switch otg connector
					if (gpio_is_valid(ds->usb_switch_pin)) {
						pr_notice("switch usb connector %lld\n", ktime_to_ms(ktime_get()));
						gpio_set_value(ds->usb_switch_pin, !!(ds->dock_active_l == gpio_get_value(ds->dock_pin)));
					}
				}
				else {//nothing changed
					val = ds->state;
				}
            }
        } else if (e_dock_type_smart == ds->dock_type) {
            if (IG_HI_PATTERN == val) {
                val = (SWITCH_DOCK | SWITCH_IGN | SWITCH_EDOCK);
                pr_notice("ignition plugged %lld\n", ktime_to_ms(ktime_get()));
            } else if (IG_LOW_PATTERN == val) {
                val = SWITCH_DOCK | SWITCH_EDOCK;
                pr_notice("ignition unplugged %lld\n", ktime_to_ms(ktime_get()));
            } else {
                val = ds->state;
            }
        } else if (SMART_PATTERN == val || IG_HI_PATTERN == val || IG_LOW_PATTERN == val) {
            pr_notice("smart cradle attempt to be plugged %lld\n", ktime_to_ms(ktime_get()));
            ds->dock_type = e_dock_type_smart;
            if (ds->usb_psy) {
                prop.intval = 1;
                ds->usb_psy->set_property(ds->usb_psy, POWER_SUPPLY_PROP_CHARGE_ENABLED, &prop);
            }
            if (IG_HI_PATTERN == val) {
                val = SWITCH_DOCK | SWITCH_IGN | SWITCH_EDOCK; 
            } else {
                val = SWITCH_DOCK | SWITCH_EDOCK; 
            }

            // disable dock interrupts while smart cradle
            if (ds->dock_irq) {
            	desc = irq_to_desc(ds->dock_irq);
            	if(0 == desc->depth) {
                	disable_irq_nosync(ds->dock_irq);
                    pr_notice("disable dock irq[%d] %lld\n", ds->dock_irq, ktime_to_ms(ktime_get()));
                }
                ds->sched_irq &= ~SWITCH_DOCK;
            }

            if (gpio_is_valid(ds->dock_pin)) {
                // pin function is smart cradle spkr switch
                //pr_notice("enable spkr switch function %lld\n", ktime_to_ms(ktime_get()));
                //canceled as cause of hw design
                //gpio_direction_output(ds->dock_pin, 1);
				set_aml_enable(ds, 0);//
            }

            // switch otg connector
            if (gpio_is_valid(ds->usb_switch_pin)) {
                pr_notice("switch usb connector %lld\n", ktime_to_ms(ktime_get()));
                gpio_set_value(ds->usb_switch_pin, ds->dock_active_l);
            }
        }
    }

    if (e_dock_type_basic == ds->dock_type) {
        if (gpio_is_valid(ds->dock_pin)) {
            if (ds->dock_active_l == gpio_get_value(ds->dock_pin) ) {
                pr_notice("basic cradle plagged %lld\n", ktime_to_ms(ktime_get()));
                val |= SWITCH_DOCK;
            } else {
                pr_notice("basic cradle unplagged %lld\n", ktime_to_ms(ktime_get()));
                ds->dock_type = e_dock_type_unspecified;
            }
        }

        if (gpio_is_valid(ds->ign_pin)) {
            if (ds->ign_active_l == gpio_get_value(ds->ign_pin) ) {
                val |= SWITCH_IGN;
            }
        }
    }

    // interrupts handled
    if (ds->sched_irq & SWITCH_DOCK) {
//        pr_notice("enable dock monitor irq[%d]\n", ds->dock_irq);
    	desc = irq_to_desc(ds->dock_irq);
    	if(desc->depth > 0) {
    		ds->sched_irq &= ~SWITCH_DOCK;
    		enable_irq(ds->dock_irq);
    	}
    }

    if (ds->sched_irq & SWITCH_IGN) {
//        pr_notice("enable ignition monitor irq[%d]\n", ds->ign_irq);
    	desc = irq_to_desc(ds->ign_irq);
    	if(desc->depth > 0) {
        	ds->sched_irq &= ~SWITCH_IGN;
        	enable_irq(ds->ign_irq);
        }
    }

	if (ds->state != val) {
        if (val & SWITCH_IGN) {
            wake_lock(&ds->wlock);
        } else {
            wake_unlock(&ds->wlock);
        }
		ds->state = val;
//        pr_notice("notify dock state [%d] %lld\n", ds->state, ktime_to_ms(ktime_get()));
		switch_set_state(&ds->sdev, val);
	}
}

static void dock_switch_work_virt_func(struct work_struct *work)
{
	struct dock_switch_device *ds  = container_of(work, struct dock_switch_device, work);
    int val = 0, err;
    

    if (VIRT_GPIO_OFF == ds->virt_init)
    	return;

	if (VIRT_GPIO_INIT == ds->virt_init) {
		if(gpio_is_valid(ds->ign_pin)) {
			err = devm_gpio_request(ds->pdev, ds->ign_pin, "ignition-state");//pdev->name);
			if (err < 0) {
				pr_err("failure to request the gpio[%d]\n", ds->ign_pin);
				return;
			} else {
				ds->virt_init = VIRT_GPIO_ON;
				gpio_direction_input(ds->ign_pin);
				gpio_export(ds->ign_pin, 0);
		        pr_notice("virt gpio is initialized\n");
			}
		} else {
			pr_err("ign gpio [%d] is not valid\n", ds->ign_pin);
			return;
		}
	}

	val |= SWITCH_ODOCK;
    if (ds->ign_active_l == gpio_get_value(ds->ign_pin) ) {
        val |= SWITCH_IGN;
    }

    if (ds->state != val) {
        pr_notice("ignition changed to %d\n", val);
		ds->state = val;
		switch_set_state(&ds->sdev, val);
	}
}

static irqreturn_t dock_switch_irq_handler(int irq, void *arg)
{
	struct dock_switch_device *ds = (struct dock_switch_device *)arg;

//    pr_notice("pins[%d]\n", irq);
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

static int32_t __ref dock_switch_ign_callback(struct notifier_block *nfb, unsigned long reason, void *arg)
{
    struct dock_switch_device *ds = container_of(nfb, struct dock_switch_device, ignition_notifier);
    unsigned long val = 0;

    if (0 == reason) {
        pr_notice("%ld\n", reason);
		ds->virt_init = VIRT_GPIO_INIT;
   		schedule_work(&ds->work);
    } else if (1 == reason) {
        if (arg) {
        	val = *((unsigned long*)arg);
            pr_notice("%ld - %ld\n", reason, val);
        	if (val & (1 << ds->virt_gpio_offset)) {
        		schedule_work(&ds->work);
        	}
        }
    }

    return NOTIFY_OK;
}

static int dock_switch_probe(struct platform_device *pdev)
{
	int err;
	struct dock_switch_device *ds;
    struct device *dev = &pdev->dev;
    struct device_node *np;
    int32_t	proj_num = 1;//TREQ_5
    const char *proj;
    uint32_t arr[2] = {0};
	
    np = dev->of_node;
    if (!np) {
        pr_err("failure to find device tree\n");
        return -EINVAL;
    }
	
	ds = devm_kzalloc(dev, sizeof(struct dock_switch_device), GFP_KERNEL);
	if (!ds)
		return -ENOMEM;

///
    proj = of_get_property(np, "compatible", NULL);
    if (proj && 0 == strncmp("treqr5,dock-switch", proj, 18)) {
        proj_num = 2;
    }
    pr_notice("project A300%d\n", proj_num);
///

    do {
		if (1 == proj_num) {
			err = of_get_named_gpio_flags(np, "treq5,dock-pin", 0, (enum of_gpio_flags *)&ds->dock_active_l);
			if (!gpio_is_valid(err)) {
				pr_err("ivalid docking pin\n");
				err = -EINVAL;
				break;
			}

            ds->usb_psy = power_supply_get_by_name("usb");

            INIT_WORK(&ds->work, dock_switch_work_func);
            wake_lock_init(&ds->wlock, WAKE_LOCK_SUSPEND, "switch_dock_wait_lock");

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

            ds->dock_type = e_dock_type_unspecified;

			if (gpio_is_valid(ds->dock_pin)) {
				err = devm_gpio_request(dev, ds->dock_pin, "dock-state");
				if (err < 0) {
					pr_err("failure to request the gpio[%d]\n", ds->dock_pin);
					break;
				}
				err = gpio_direction_input(ds->dock_pin);
				if (err < 0) {
					pr_err("failure to set direction of the gpio[%d]\n", ds->dock_pin);
					break;
				}
				gpio_export(ds->dock_pin, 1);
				set_aml_enable(ds, FORBID_EXT_SPKR);
				ds->dock_irq = gpio_to_irq(ds->dock_pin);
				if (ds->dock_irq < 0) {
					pr_err("failure to request gpio[%d] irq\n", ds->dock_pin);
				} else {
					err = devm_request_irq(dev, ds->dock_irq, dock_switch_irq_handler,
										   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_DISABLED,
										   pdev->name, ds);
					if (!err) {
						disable_irq_nosync(ds->dock_irq);
					} else {
						pr_err("failure to request irq[%d] irq -- polling available\n", ds->dock_irq);
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
				if (err < 0) {
					pr_err("failure to set direction of the gpio[%d]\n", ds->ign_pin);
					break;
				}
				gpio_export(ds->ign_pin, 0);
				ds->ign_irq = gpio_to_irq(ds->ign_pin);
				if (ds->ign_irq < 0) {
					pr_err("failure to request gpio[%d] irq\n", ds->ign_pin);
				} else {
					err = devm_request_irq(dev, ds->ign_irq, dock_switch_irq_handler,
										   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_DISABLED,
										   pdev->name, ds);
					if (!err) {
						disable_irq_nosync(ds->ign_irq);
					} else {
						pr_err("failure to request irq[%d] irq -- polling available\n", ds->ign_irq);
					}
				}
			}
			if (!(ds->dock_irq < 0 && ds->ign_irq < 0)) {
				device_init_wakeup(dev, 1);
			}
		} else {
            //proj TREQr_5
			err = of_property_read_u32_array(np, "treqr5,virt-gpio", arr, 2);
			if (!err) {
				err                    = arr[0];//base
				ds->virt_gpio_offset   = arr[1];//offset
				err += ds->virt_gpio_offset;
				if (!gpio_is_valid(err)) {
					pr_err("ivalid ignition pin\n");
					err = -EINVAL;
					break;
				}
			} else {
				pr_err("cannot get ign gpio\n");
				err = -EINVAL;
				break;
			}

			pr_notice("ignition detect pin %d\n", err);
			ds->ign_pin = err;
			ds->virt_init = VIRT_GPIO_OFF;

			if(gpio_is_valid(ds->ign_pin)) {
				err = devm_gpio_request(dev, ds->ign_pin, "ignition-state");//pdev->name);
				if (err < 0) {
					pr_err("failure to request the gpio[%d]\n", ds->ign_pin);
				} else {
					err = gpio_direction_input(ds->ign_pin);
					if (err < 0) {
						pr_err("failure to set direction of the gpio[%d]\n", ds->ign_pin);
						break;
					}
					gpio_export(ds->ign_pin, 0);
					ds->virt_init = VIRT_GPIO_ON;
				}
			}

			ds->ign_active_l = 1;

			INIT_WORK(&ds->work, dock_switch_work_virt_func);

			ds->ignition_notifier.notifier_call = dock_switch_ign_callback;
			err = gpio_in_register_notifier(&ds->ignition_notifier);
			if (err) {
				pr_err("failure to register remount notifier [%d]\n", err);
				err = -EINVAL;
				break;
			}
		}
        ds->sdev.name = "dock";
        ds->sdev.print_state = dock_switch_print_state;
        err = switch_dev_register(&ds->sdev);
        if (err < 0) {
            pr_err("err_register_switch\n");
            break;
        }
    	err = device_create_file((&ds->sdev)->dev, &dev_attr_ampl_enable);
    	if (err < 0) {
            pr_err("err0r create amplifier file\n");
            break;
        }

        ds->pdev = dev;
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
        devm_free_irq(&pdev->dev, ds->ign_irq, ds);
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

	device_remove_file((&ds->sdev)->dev, &dev_attr_ampl_enable);
    switch_dev_unregister(&ds->sdev);

    if (device_may_wakeup(&pdev->dev))
        device_wakeup_disable(&pdev->dev);
	if (ds->ign_irq) {
        disable_irq_nosync(ds->ign_irq);
		devm_free_irq(&pdev->dev, ds->ign_irq, ds);
    }
    if (ds->dock_irq) {
        disable_irq_nosync(ds->dock_irq);
        devm_free_irq(&pdev->dev, ds->dock_irq, ds);
    }

	if (gpio_is_valid(ds->dock_pin)) 
		devm_gpio_free(&pdev->dev, ds->dock_pin);
	if (gpio_is_valid(ds->ign_pin))
		devm_gpio_free(&pdev->dev, ds->ign_pin);

    wake_lock_destroy(&ds->wlock);
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
            pr_notice("enable wake source IGN[%d]\n", ds->ign_irq);
            enable_irq_wake(ds->ign_irq);
        }
        if (ds->dock_irq && (e_dock_type_smart != ds->dock_type)) {
            pr_notice("enable wake source DOCK[%d]\n", ds->dock_irq);
            enable_irq_wake(ds->dock_irq);
        }
    }

    return 0;
}


static int dock_switch_resume(struct device *dev)
{
	struct dock_switch_device *ds = dev_get_drvdata(dev);
	struct irq_desc *desc;

    if (device_may_wakeup(dev)) {
        if (ds->ign_irq) {
        	desc = irq_to_desc(ds->ign_irq);
            if(desc->depth == 0) {
            	disable_irq_nosync(ds->ign_irq);
                pr_notice("disable wake source IGN[%d]\n", ds->ign_irq);
            }
            disable_irq_wake(ds->ign_irq);
        }
        if (ds->dock_irq && (e_dock_type_smart != ds->dock_type)) {
        	desc = irq_to_desc(ds->dock_irq);
            if(desc->depth == 0) {
            	disable_irq_nosync(ds->dock_irq);
                pr_notice("disable wake source DOCK[%d]\n", ds->dock_irq);
            }
            disable_irq_wake(ds->dock_irq);
        }
    }

    ds->sched_irq = SWITCH_IGN;
    if (e_dock_type_smart != ds->dock_type) {
        ds->sched_irq |= SWITCH_DOCK;
    }
    pr_notice("sched reason[%u]\n", ds->sched_irq); 
	schedule_work(&ds->work);

	return 0;
}

static const struct dev_pm_ops dock_switch_pm_ops = {
	.suspend	= dock_switch_suspend,
	.resume		= dock_switch_resume,
};

static struct of_device_id dock_switch_match[] = {
	{ .compatible = "treq5,dock-switch", },
	{ .compatible = "treqr5,dock-switch", },
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

