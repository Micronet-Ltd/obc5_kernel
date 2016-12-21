
// by skj
#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <linux/pinctrl/consumer.h>
#include <linux/miscdevice.h>
#include <linux/seq_file.h>

struct watch_dog_pin_info{
	int toggle_pin;
	int port_det_pin;
	int usb_switch_pin;	
	int high_delay;
	int low_delay;
	struct delayed_work	toggle_work;
	int state; // high=1 low=0
    struct miscdevice mdev;
    int rf_kill_pin;
    int rf_state;
    unsigned long open;
};


static ssize_t rfkillpin_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    int val;
    struct miscdevice *md =  file->private_data;
    struct watch_dog_pin_info *wdi = container_of(md, struct watch_dog_pin_info, mdev);

    if (1 == count) {
        val = ('0' == *buf)?0:1;

		if(gpio_is_valid(wdi->rf_kill_pin)){
			gpio_set_value(wdi->rf_kill_pin, val);
            wdi->rf_state = val;
            return count;
		}
	}

	return 0;
}

static ssize_t rfkillpin_read(struct file * file, char __user * buf, size_t count, loff_t *ppos)
{
    struct miscdevice *md =  file->private_data;
    struct watch_dog_pin_info *wdi = container_of(md, struct watch_dog_pin_info, mdev);

    if (0 == wdi->open) {
        return -EINVAL;
    }

    if (1 != count) {
		printk("%s: buffer too small\n", __func__);
		return -EINVAL;
	}

    *buf = '0' + wdi->rf_state;

    if (ppos) {
        *ppos = 0;
    }

    return count;
}

static int rfkillpin_open(struct inode *inode, struct file *file)
{
    struct miscdevice *md =  file->private_data;
	struct watch_dog_pin_info *wdi = container_of(md, struct watch_dog_pin_info, mdev);

	if(test_and_set_bit(1, &wdi->open))
		return -EPERM;

	return 0;
}

static int rfkillpin_release(struct inode *inode, struct file *file)
{
    struct miscdevice *md =  file->private_data;
    struct watch_dog_pin_info *wdi = container_of(md, struct watch_dog_pin_info, mdev);

	clear_bit(1, &wdi->open);

	return 0;
}

static const struct file_operations rfkill_operations = {
    .owner      = THIS_MODULE,
	.read		= rfkillpin_read,
    .write      = rfkillpin_write,
    .open       = rfkillpin_open,
    .release    = rfkillpin_release,
};

static void watchdog_toggle_work(struct work_struct *work)
{
	struct watch_dog_pin_info *inf =
		container_of(work, struct watch_dog_pin_info,
				toggle_work.work);
    unsigned long d;

    inf->state ^= 1;
    d = (inf->state)?inf->high_delay:inf->low_delay;
    gpio_set_value(inf->toggle_pin, inf->state);
    schedule_delayed_work(&inf->toggle_work,msecs_to_jiffies(d));
}

static irqreturn_t port_det_handler(int irq, void *dev_id)
{
	struct watch_dog_pin_info *inf=dev_id;

	if(0==gpio_get_value(inf->port_det_pin))
		gpio_direction_output(inf->usb_switch_pin,0);	
	else
		gpio_direction_output(inf->usb_switch_pin,1);	
	return IRQ_HANDLED;
}

static int watchdog_pin_probe(struct platform_device *op)
{
	struct device *dev = &op->dev;
	struct device_node *np = op->dev.of_node;	
	struct watch_dog_pin_info *inf;
	int rc;
	int irq;

	if(np==NULL){		
		dev_err(dev, "Can not find config!\n");
		return -ENOENT;
	}
	inf = devm_kzalloc(dev, sizeof(*inf), GFP_KERNEL);
	if (!inf) {
		dev_err(dev, "Memory exhausted!\n");
		return -ENOMEM;
	}
	inf->toggle_pin=of_get_named_gpio(np,"ehang,toggle-pin",0);
	if(inf->toggle_pin<0){
		dev_err(dev, "Memory exhausted!\n");
		return -ENOMEM;		
	}
	rc=devm_gpio_request(dev,inf->toggle_pin,"watchdog_pin");
	if(rc<0){
		dev_err(dev, "toggle pin is busy!\n");
		return -ENOMEM;			
	}
	gpio_direction_output(inf->toggle_pin,0);

	inf->port_det_pin=
		of_get_named_gpio(np,"ehang,port-det-pin",0);
	if(gpio_is_valid(inf->port_det_pin)){
		rc=devm_gpio_request(dev,inf->port_det_pin,"port_det");	
		if(rc<0)
			dev_err(dev, "port det pin is busy!\n");		
		gpio_direction_input(inf->port_det_pin);	
	}
	
	inf->usb_switch_pin=
		of_get_named_gpio(np,"ehang,usb-switch-pin",0);
	if(gpio_is_valid(inf->usb_switch_pin) && gpio_is_valid(inf->port_det_pin)){
		rc=devm_gpio_request(dev,inf->usb_switch_pin,"usb_switch");	
		if(rc<0)
			dev_err(dev, "usb switch pin is busy!\n");
		if(0==gpio_get_value(inf->port_det_pin))
			gpio_direction_output(inf->usb_switch_pin,0);	
		else
			gpio_direction_output(inf->usb_switch_pin,1);

		irq= gpio_to_irq(inf->port_det_pin);
		rc = devm_request_threaded_irq(dev, irq, NULL,
				port_det_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"port_det_handler", inf);
		if (rc < 0) {
			dev_err(dev,
				"request_irq for irq=%d  failed rc = %d\n",
				irq, rc);
		}	
	}	

	inf->rf_kill_pin = of_get_named_gpio(np,"ehang,rf-kill-pin", 0);

    if (gpio_is_valid(inf->rf_kill_pin)) {
        rc = devm_gpio_request(dev,inf->rf_kill_pin,"rf-kill-pin");
        if (rc < 0) {
            dev_err(dev, "rf-kill-pin is busy!\n");
            return -ENOMEM;			
        }
        gpio_direction_output(inf->rf_kill_pin, 1);
        inf->rf_state = 1;
    }
	
	rc = of_property_read_u32(np, "ehang,high-delay",
						&inf->high_delay);

	if(rc<0)
		inf->high_delay=1000;

	rc = of_property_read_u32(np, "ehang,low-delay",
						&inf->low_delay);
	if(rc<0)
		inf->low_delay=1000;	

	dev_set_drvdata(dev, inf);

    inf->state = 0;
	
	INIT_DELAYED_WORK(&inf->toggle_work,
					watchdog_toggle_work);

	schedule_delayed_work(&inf->toggle_work,
		msecs_to_jiffies(inf->low_delay));	

    inf->mdev.minor = MISC_DYNAMIC_MINOR;
    inf->mdev.name	= "rf_kill_pin";
    inf->mdev.fops	= &rfkill_operations;
	misc_register(&inf->mdev);
	return 0;
}

static int watchdog_pin_remove(struct platform_device *op)
{
    return 0;
}

static int watchdog_pin_suspend(struct device *dev)
{
    struct watch_dog_pin_info *wdi = dev_get_drvdata(dev);

    if(gpio_is_valid(wdi->rf_kill_pin)){
        gpio_set_value(wdi->rf_kill_pin, 0);
    }

    return 0;
}

static int watchdog_pin_resume(struct device *dev)
{
    struct watch_dog_pin_info *wdi = dev_get_drvdata(dev);

    if(gpio_is_valid(wdi->rf_kill_pin)){
        gpio_set_value(wdi->rf_kill_pin, wdi->rf_state);
    }

	return 0;
}

static const struct dev_pm_ops watchdog_pin_pm_ops =
{
	.suspend	= watchdog_pin_suspend,
	.resume		= watchdog_pin_resume,
};

static struct of_device_id watchdog_pin_match[] = {
	{ .compatible = "ehang,watchdog-pin", },
	{},
};

static struct platform_driver watchdog_pin_driver = {
	.probe		= watchdog_pin_probe,
	.remove		= watchdog_pin_remove,
	.driver		= {
		.name = "watchdog-pin",
		.owner = THIS_MODULE,
		.of_match_table = watchdog_pin_match,
        .pm = &watchdog_pin_pm_ops,
	},
};

module_platform_driver(watchdog_pin_driver);

MODULE_AUTHOR("skj at ehang Inc.");
MODULE_DESCRIPTION("WATCH DOG GPIO TOGGLE driver");
MODULE_LICENSE("GPL");


