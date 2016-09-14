/**
 * power_lost_detect.c - Power Lost Detect Driver
 *
 * Copyright (C) 2013-2016 Micronet Ltd.
 *
 * Written by Vladimir Zatulovsky 
 * <vladimir.zatulovsky@micronet-inc.com> 
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#define pr_fmt(fmt) "%s %s: " fmt, KBUILD_MODNAME, __func__

#include "../../../out/target/product/msm8916_64/obj/KERNEL_OBJ/include/generated/autoconf.h"
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/cpu.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
//#include <linux/cpufreq.h>

#undef DEBUG_POWER_LOST
#define DEBUG_POWER_LOST 1

enum {
	e_pwrl_unspecified = -1,
    e_pwrl_display_off,
    e_pwrl_gpu_off,
    e_pwrl_cores_off,
    e_pwrl_wlan_off,
    e_pwrl_wan_off,
	e_pwrl_device_off,
    e_pwrl_device_suspend
}e_pwrl_state;

struct a8_power_lost_hmon_attr {
    struct device_attribute attr;
    char name[32];
};

struct a8_power_lost_detect_info {
    struct device dev;
    struct device *hmd;
	int pwr_lost_irq;
    int pwr_lost_pin;
    int pwr_lost_pin_level;
	int	pwr_lost_ps;
	int pwr_lost_off_delay;
	int pwr_lost_wan_delay;
    int pwr_lost_wlan_delay;
	long long pwr_lost_timer;
    int sys_ready;
    int remount_complete;
    spinlock_t pwr_lost_lock;
    unsigned long lock_flags;
	struct delayed_work pwr_lost_work;
    struct a8_power_lost_hmon_attr attr_in;
    struct a8_power_lost_hmon_attr attr_inl;
    struct a8_power_lost_hmon_attr attr_wan_d;
    struct a8_power_lost_hmon_attr attr_wlan_d;
    struct a8_power_lost_hmon_attr attr_off_d;
    struct notifier_block a8_power_lost_cpu_notifier;
    struct notifier_block a8_power_lost_remount_notifier;
#if defined (DEBUG_POWER_LOST)
    int dbg_lvl;
    struct a8_power_lost_hmon_attr attr_ps;
#endif
};

static ssize_t a8_power_lost_show_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", "a8-power-lost-detect");
}

static DEVICE_ATTR(name, 0444, a8_power_lost_show_name, 0);

#if defined (DEBUG_POWER_LOST)
static ssize_t a8_power_lost_ps_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct a8_power_lost_detect_info *pwrl = dev_get_drvdata(dev);

    return sprintf(buf, "%d\n", pwrl->dbg_lvl);

}

static ssize_t a8_power_lost_ps_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int val = -1;
    struct a8_power_lost_detect_info *pwrl = dev_get_drvdata(dev);

    if (kstrtos32(buf, 10, &val))
        return -EINVAL;

    spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    pwrl->dbg_lvl = val;
    pwrl->pwr_lost_timer = ktime_to_ms(ktime_get());
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);

    schedule_delayed_work(&pwrl->pwr_lost_work, 0);

    return count;
}
#endif

static ssize_t a8_power_lost_in_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int val = -1;
    struct a8_power_lost_detect_info *pwrl = dev_get_drvdata(dev);

    if (gpio_is_valid(pwrl->pwr_lost_pin)) {
        val = __gpio_get_value(pwrl->pwr_lost_pin);
    }

	return sprintf(buf, "%d\n", val);
}

static ssize_t a8_power_lost_in_l_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct a8_power_lost_detect_info *pwrl = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", pwrl->pwr_lost_pin_level);
}

static ssize_t a8_power_lost_in_l_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int val;
    struct a8_power_lost_detect_info *pwrl = dev_get_drvdata(dev);

    if (kstrtos32(buf, 10, &val))
        return -EINVAL;

    spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    pwrl->pwr_lost_pin_level = val;
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);

    return count;
}

static ssize_t a8_power_lost_offd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct a8_power_lost_detect_info *pwrl = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", pwrl->pwr_lost_off_delay);
}

static ssize_t a8_power_lost_offd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int val;
    struct a8_power_lost_detect_info *pwrl = dev_get_drvdata(dev);

    if (kstrtos32(buf, 10, &val))
        return -EINVAL;

    spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    pwrl->pwr_lost_off_delay = val;
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);

    return count;
}

static ssize_t a8_power_lost_wand_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct a8_power_lost_detect_info *pwrl = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", pwrl->pwr_lost_wan_delay);
}

static ssize_t a8_power_lost_wand_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int val;
    struct a8_power_lost_detect_info *pwrl = dev_get_drvdata(dev);

    if (kstrtos32(buf, 10, &val))
        return -EINVAL;

    spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    pwrl->pwr_lost_wan_delay = val;
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);

    return count;
}

static ssize_t a8_power_lost_wland_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct a8_power_lost_detect_info *pwrl = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", pwrl->pwr_lost_wlan_delay);
}

static ssize_t a8_power_lost_wland_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int val;
    struct a8_power_lost_detect_info *pwrl = dev_get_drvdata(dev);

    if (!pwrl->sys_ready) {
        return -EINVAL;
    }

    if (kstrtos32(buf, 10, &val)) 
        return -EINVAL;

    spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    pwrl->pwr_lost_wlan_delay = val;
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);

    return count;
}

static RAW_NOTIFIER_HEAD(power_lost_chain);
static DEFINE_RAW_SPINLOCK(power_lost_chain_lock);

void power_lost_notify(unsigned long reason, void *arg)
{
	unsigned long flags;

    raw_spin_lock_irqsave(&power_lost_chain_lock, flags);
    raw_notifier_call_chain(&power_lost_chain, reason, 0);
    raw_spin_unlock_irqrestore(&power_lost_chain_lock, flags);
}

int power_lost_register_notifier(struct notifier_block *nb)
{
	unsigned long flags;
    int err;

	raw_spin_lock_irqsave(&power_lost_chain_lock, flags);
	err = raw_notifier_chain_register(&power_lost_chain, nb);
	raw_spin_unlock_irqrestore(&power_lost_chain_lock, flags);

	return err;
}
EXPORT_SYMBOL(power_lost_register_notifier);

#if 0
// example notification using

static int power_lost_notifier(struct notifier_block *nb, unsigned long val, void *priv)
{
	struct <specific> *dev_specific = container_of(nb, struct <specific>, power_lost_notifier);

    // Do something
    //

	return NOTIFY_OK;
}

// In probe
power_lost_register_notifier(<specific> *dev_specific->power_lost_notifier)


extern int dpm_dev_suspend(char *);
extern int dpm_dev_resume(char *);
#endif

// Vladimir
// TODO: implement remount completing wait
//
#if 0
static int remount_ro_done(void)
{
    int f;
    char mount_dev[256];
    char mount_dir[256];
    char mount_type[256];
    char mount_opts[256];
    int mount_freq;
    int mount_passno;
    int match;
    int found_rw_fs = 0;

    f = sys_open("/proc/mounts", O_RDONLY, 0);
    if (! f) {
        pr_notice("failure to open /proc/mounts\n");
        return 1;
    }

    do {
        match = fscanf(f, "%255s %255s %255s %255s %d %d\n",
                       mount_dev, mount_dir, mount_type,
                       mount_opts, &mount_freq, &mount_passno);
        mount_dev[255] = 0;
        mount_dir[255] = 0;
        mount_type[255] = 0;
        mount_opts[255] = 0;
        if ((match == 6) && !strncmp(mount_dev, "/dev/block", 10) && strstr(mount_opts, "rw,")) {
            found_rw_fs = 1;
            break;
        }
    } while (match != EOF);

    fclose(f);

    return !found_rw_fs;
}
#endif

extern int emergency_remount_register_notifier(struct notifier_block *nb);

static void remount_ro(struct a8_power_lost_detect_info *pwrl)
{
    int fd, i = 0;

    fd = emergency_remount_register_notifier(&pwrl->a8_power_lost_remount_notifier);
    if (fd) {
        pr_err("failure to register remount notifier [%d]\n", fd);
        return;
    }

    fd = sys_open("/proc/sysrq-trigger", O_WRONLY, 0); 
    if (fd < 0) {
        pr_notice("failure to open /proc/sysrq-trigger\n");
        return;
    }
    sys_write(fd, "u", 1);
    sys_close(fd);


    while (!pwrl->remount_complete && (i++ < 40)) {
        msleep(100);
    }

    return;
}

static void __ref a8_power_lost_detect_work(struct work_struct *work)
{
    int val, err;
    long long timer;
	struct a8_power_lost_detect_info *pwrl = container_of(work, struct a8_power_lost_detect_info, pwr_lost_work.work);

    spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    timer = ktime_to_ms(ktime_get());
    val = __gpio_get_value(pwrl->pwr_lost_pin);
#if defined (DEBUG_POWER_LOST)
    val |= pwrl->dbg_lvl;
#endif

    if (pwrl->pwr_lost_ps == e_pwrl_unspecified) {
        if (pwrl->pwr_lost_pin_level == val) {
            pwrl->pwr_lost_ps = e_pwrl_display_off;
            pr_notice("power lost %lld\n", timer);
            pr_notice("shutdown display %lld\n", ktime_to_ms(ktime_get()));
            power_lost_notify(1, 0);
            timer = 0;
        } else {
            pr_notice("power restored %lld\n", timer);
            spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);
            for (val = num_possible_cpus() - 1; val > 0; val--) {
                if (cpu_online(val))
                    continue;
#if defined (CONFIG_SMP)
                err = cpu_up(val);
                if (err && err == notifier_to_errno(NOTIFY_BAD))
                    pr_notice("up cpu%d is declined\n", val);
                else if (err)
                    pr_err("failure to up cpu%d. err:%d\n", val, err);
                else
                    pr_notice("up cpu%d\n", val);
#endif
            }
            power_lost_notify(0, 0);
            enable_irq(pwrl->pwr_lost_irq);
            return;
        }
    } else if (pwrl->pwr_lost_ps == e_pwrl_display_off) {
        if (pwrl->pwr_lost_pin_level == val) {
            pr_notice("shutdown gpu %lld\n", ktime_to_ms(ktime_get()));
            pwrl->pwr_lost_ps = e_pwrl_gpu_off;
        } else {
            pwrl->pwr_lost_ps = e_pwrl_unspecified;
        }
        timer = 0;
    } else if (pwrl->pwr_lost_ps == e_pwrl_gpu_off) {
        if (pwrl->pwr_lost_pin_level == val) {
            pr_notice("shutdown cores %lld\n", ktime_to_ms(ktime_get()));
            pwrl->pwr_lost_ps = e_pwrl_cores_off;
            //cpu_hotplug_driver_lock();
            for (val = num_possible_cpus() - 1; val > 0; val--) {
                if (!cpu_online(val))
                    continue;
#if defined (CONFIG_SMP)
                spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);
                err = cpu_down(val);
                spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
                if (err)
                    pr_err("Unable shutdown cpu%d [%d]\n", val, err);
//                else
//                    pr_notice("shutdown cpu%d\n", val);
#endif
            }
            //cpu_hotplug_driver_unlock();
        } else {
            pwrl->pwr_lost_ps = e_pwrl_unspecified;
        }
        timer = 0;
    } else if (pwrl->pwr_lost_ps == e_pwrl_cores_off) {
        if (pwrl->pwr_lost_pin_level == val) {
            val = pwrl->pwr_lost_wan_delay;
            pwrl->pwr_lost_ps = e_pwrl_wan_off;
            pr_notice("wan may be shutdown %lld\n", ktime_to_ms(ktime_get()));

            if (pwrl->pwr_lost_wlan_delay < val) {
                val = pwrl->pwr_lost_wlan_delay;
                pwrl->pwr_lost_ps = e_pwrl_wlan_off;
                pr_notice("wlan may be shutdown %lld\n", ktime_to_ms(ktime_get()));
            }
            if (pwrl->pwr_lost_off_delay < val) {
                val = pwrl->pwr_lost_off_delay;
                pwrl->pwr_lost_ps = e_pwrl_device_off;
                pr_notice("device may be shutdown %lld\n", ktime_to_ms(ktime_get()));
            }
            timer = val;
        } else {
            pwrl->pwr_lost_ps = e_pwrl_unspecified;
            timer = 0;
        }
    } else if (pwrl->pwr_lost_ps == e_pwrl_wan_off) {
        pr_notice("shutdown wan %lld\n", ktime_to_ms(ktime_get()));

        if (pwrl->pwr_lost_wlan_delay > (ktime_to_ms(ktime_get()) - pwrl->pwr_lost_timer)) {
            val = pwrl->pwr_lost_timer + pwrl->pwr_lost_wlan_delay - ktime_to_ms(ktime_get());
            pwrl->pwr_lost_ps = e_pwrl_wlan_off;
            pr_notice("wlan may be shutdown %lld\n", ktime_to_ms(ktime_get()));
        } else {
            pr_notice("device may be shutdown %lld\n", ktime_to_ms(ktime_get()));
            if (pwrl->pwr_lost_off_delay > (ktime_to_ms(ktime_get()) - pwrl->pwr_lost_timer)) {
                val = pwrl->pwr_lost_timer + pwrl->pwr_lost_off_delay - ktime_to_ms(ktime_get());
            } else {
                val = 0;
            }
            pwrl->pwr_lost_ps = e_pwrl_device_off;
        }
        timer = val;
    } else if (pwrl->pwr_lost_ps == e_pwrl_wlan_off) {
        pr_notice("shutdown wlan %lld\n", ktime_to_ms(ktime_get()));

        if (pwrl->pwr_lost_wan_delay > (ktime_to_ms(ktime_get()) - pwrl->pwr_lost_timer)) {
            val = pwrl->pwr_lost_timer + pwrl->pwr_lost_wan_delay - ktime_to_ms(ktime_get());
            pwrl->pwr_lost_ps = e_pwrl_wan_off;
            pr_notice("wan may be shutdown %lld\n", ktime_to_ms(ktime_get()));
        } else {
            pr_notice("device may be shutdown %lld\n", ktime_to_ms(ktime_get()));
            if (pwrl->pwr_lost_off_delay > (ktime_to_ms(ktime_get()) - pwrl->pwr_lost_timer)) {
                val = pwrl->pwr_lost_timer + pwrl->pwr_lost_off_delay - ktime_to_ms(ktime_get());
            } else {
                val = 0;
            }
            pwrl->pwr_lost_ps = e_pwrl_device_off;
        }
        timer = val;
    } else if (pwrl->pwr_lost_ps == e_pwrl_device_off) {
        pwrl->remount_complete = 0;
        spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);
        pr_notice("urgent remount block devices ro %lld\n", ktime_to_ms(ktime_get()));
        //sys_umount("/data", MNT_FORCE | MNT_DETACH);

        sys_sync();
        remount_ro(pwrl);
        pr_notice("urgent shutdown device %lld\n", ktime_to_ms(ktime_get()));
        orderly_poweroff(1);
        return;
    } else {
        pr_notice("Oooops, bug, bug, bug %lld\n", ktime_to_ms(ktime_get()));
    }
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    schedule_delayed_work(&pwrl->pwr_lost_work, (timer)?msecs_to_jiffies(timer): 0);
#if 0
		dpm_dev_suspend("omapdss");//a300-panel");
#endif
}

static irqreturn_t a8_power_lost_detect_handler(int irq, void *irq_data)
{
	struct a8_power_lost_detect_info *pwrl = irq_data;

	disable_irq_nosync(pwrl->pwr_lost_irq);

    pr_notice("power is lost %lld\n", ktime_to_ms(ktime_get()));

    spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    pwrl->pwr_lost_timer = ktime_to_ms(ktime_get());
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);

	schedule_delayed_work(&pwrl->pwr_lost_work, 0);

	return IRQ_HANDLED;
}

static int __ref a8_power_lost_cpu_callback(struct notifier_block *nfb, unsigned long a, void *pcpu)
{
	uint32_t cpu = (uintptr_t)pcpu;
    struct a8_power_lost_detect_info *pwrl = container_of(nfb, struct a8_power_lost_detect_info, a8_power_lost_cpu_notifier);

	if (a == CPU_UP_PREPARE || a == CPU_UP_PREPARE_FROZEN) {
		if (pwrl->pwr_lost_ps != e_pwrl_unspecified) {
			pr_notice("prevent cpu%d up\n", cpu);
			return NOTIFY_BAD;
		}
        pr_notice("voting for cpu%d up\n", cpu);
	}

	return NOTIFY_OK;
}

static int __ref a8_power_lost_remount_callback(struct notifier_block *nfb, unsigned long a, void *arg)
{
    struct a8_power_lost_detect_info *pwrl = container_of(nfb, struct a8_power_lost_detect_info, a8_power_lost_remount_notifier);

    spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    pwrl->remount_complete = (unsigned int)a;
    pr_notice("remounted %d\n", pwrl->remount_complete);
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);

	return NOTIFY_OK;
}

static int a8_power_lost_detect_probe(struct platform_device *pdev)
{
	int err, val;
	struct a8_power_lost_detect_info *pwrl;
    struct device *dev = &pdev->dev;
    struct device_node *np;

    np = dev->of_node; //of_find_compatible_node(0, 0, "a8-hm-power-lost");
    if (!np) {
        pr_err("failure to find device tree\n");
        return -EINVAL;
    }

    pwrl = devm_kzalloc(dev, sizeof(struct a8_power_lost_detect_info), GFP_KERNEL);
    if (!pwrl) {
        pr_err("Add some memory\n");
        return -ENOMEM;
    }

    pwrl->sys_ready = 0;
    pwrl->dbg_lvl = 0;
    pwrl->a8_power_lost_cpu_notifier.notifier_call = a8_power_lost_cpu_callback;
    pwrl->a8_power_lost_remount_notifier.notifier_call = a8_power_lost_remount_callback;

    err = register_cpu_notifier(&pwrl->a8_power_lost_cpu_notifier);
    if (err) {
        pr_err("failure to register cpu notifier [%d]\n", err);
    }

    do {
        spin_lock_init(&pwrl->pwr_lost_lock);

#if defined (DEBUG_POWER_LOST)
        snprintf(pwrl->attr_ps.name, sizeof(pwrl->attr_ps.name) - 1, "ps");
        pwrl->attr_ps.attr.attr.name = pwrl->attr_ps.name;
        pwrl->attr_ps.attr.attr.mode = 0444;
        pwrl->attr_ps.attr.show = a8_power_lost_ps_show;
        pwrl->attr_ps.attr.store = a8_power_lost_ps_store;
        sysfs_attr_init(&pwrl->attr_ps.attr.attr);

#endif
        snprintf(pwrl->attr_in.name, sizeof(pwrl->attr_in.name) - 1, "state");
        pwrl->attr_in.attr.attr.name = pwrl->attr_in.name;
        pwrl->attr_in.attr.attr.mode = 0444;
        pwrl->attr_in.attr.show = a8_power_lost_in_show;
        sysfs_attr_init(&pwrl->attr_in.attr.attr);

        snprintf(pwrl->attr_inl.name, sizeof(pwrl->attr_inl.name) - 1, "level");
        pwrl->attr_inl.attr.attr.name = pwrl->attr_inl.name;
        pwrl->attr_inl.attr.attr.mode = 0666;
        pwrl->attr_inl.attr.show = a8_power_lost_in_l_show;
        pwrl->attr_inl.attr.store = a8_power_lost_in_l_store;
        sysfs_attr_init(&pwrl->attr_inl.attr.attr);

        snprintf(pwrl->attr_off_d.name, sizeof(pwrl->attr_off_d.name) - 1, "off_delay");
        pwrl->attr_off_d.attr.attr.name = pwrl->attr_off_d.name;
        pwrl->attr_off_d.attr.attr.mode = 0444;
        pwrl->attr_off_d.attr.show = a8_power_lost_offd_show;
        pwrl->attr_off_d.attr.store = a8_power_lost_offd_store;
        sysfs_attr_init(&pwrl->attr_off_d.attr.attr);

        snprintf(pwrl->attr_wan_d.name, sizeof(pwrl->attr_wan_d.name) - 1, "wan_off_delay");
        pwrl->attr_wan_d.attr.attr.name = pwrl->attr_wan_d.name;
        pwrl->attr_wan_d.attr.attr.mode = 0444;
        pwrl->attr_wan_d.attr.show = a8_power_lost_wand_show;
        pwrl->attr_wan_d.attr.store = a8_power_lost_wand_store;
        sysfs_attr_init(&pwrl->attr_wan_d.attr.attr);

        snprintf(pwrl->attr_wlan_d.name, sizeof(pwrl->attr_wlan_d.name) - 1, "wlan_off_delay");
        pwrl->attr_wlan_d.attr.attr.name = pwrl->attr_wlan_d.name;
        pwrl->attr_wlan_d.attr.attr.mode = 0444;
        pwrl->attr_wlan_d.attr.show = a8_power_lost_wland_show;
        pwrl->attr_wlan_d.attr.store = a8_power_lost_wland_store;
        sysfs_attr_init(&pwrl->attr_wlan_d.attr.attr);

        val = of_get_named_gpio_flags(np, "qcom,irq-gpio", 0, (enum of_gpio_flags *)&pwrl->pwr_lost_pin_level);
//        val = of_get_named_gpio(np,"a8,pwr-lost-pin",0);
        if (!gpio_is_valid(val)) {
            pr_err("ivalid power lost detect pin\n");
            err = -EINVAL;
            break;
        }
        pwrl->pwr_lost_pin = val;

        err = of_property_read_u32(np, "a8,pwr-lost-level", &val); 
        if (err < 0) {
            val = 1;
        }
//        pwrl->pwr_lost_pin_level = val;

        err = of_property_read_u32(np, "a8,wan-off-delay", &val); 
        if (err < 0) {
            val = -1;
        }
        pwrl->pwr_lost_wan_delay = val;

        err = of_property_read_u32(np, "a8,wlan-off-delay", &val); 
        if (err < 0) {
            val = -1;
        }
        pwrl->pwr_lost_wlan_delay = val;

        err = of_property_read_u32(np, "a8,plat-off-delay", &val); 
        if (err < 0) {
            val = 1000;
        }
        pwrl->pwr_lost_off_delay = val;
        pr_notice("Initial parameters\n");
        pr_notice("pin %d\n", pwrl->pwr_lost_pin);
        pr_notice("level detection %s\n", (pwrl->pwr_lost_pin_level)?"high":"low");
        pr_notice("shutdown after %d\n", pwrl->pwr_lost_off_delay);
        pr_notice("shutdown WAN after %d\n", pwrl->pwr_lost_wan_delay);
        pr_notice("shutdown WLAN after %d\n", pwrl->pwr_lost_wlan_delay);

        pwrl->hmd = hwmon_device_register(dev);
    	if (IS_ERR(pwrl->hmd)) {
    		err = PTR_ERR(pwrl->hmd);
    		break;
    	}

        dev_set_drvdata(pwrl->hmd, pwrl);

        err = device_create_file(pwrl->hmd, &dev_attr_name);
    	if (err) {
            err = -EINVAL;
            break;
        }
#if defined (DEBUG_POWER_LOST)
        err = device_create_file(pwrl->hmd, &pwrl->attr_ps.attr);
#endif
        err = device_create_file(pwrl->hmd, &pwrl->attr_in.attr);
        if (err) {
            err = -EINVAL;
            break;
        }
        err = device_create_file(pwrl->hmd, &pwrl->attr_inl.attr);
        if (err) {
            err = -EINVAL;
            break;
        }
        err = device_create_file(pwrl->hmd, &pwrl->attr_off_d.attr);
        if (err) {
            err = -EINVAL;
            break;
        }
        err = device_create_file(pwrl->hmd, &pwrl->attr_wan_d.attr);
        if (err) {
            err = -EINVAL;
            break;
        }
        err = device_create_file(pwrl->hmd, &pwrl->attr_wlan_d.attr);
    	if (err) {
            err = -EINVAL;
            break;
        }

        dev_set_drvdata(dev, pwrl);

        err = devm_gpio_request(dev, pwrl->pwr_lost_pin, "power-detector-state");
        if (err < 0) {
            pr_err("failure to request the gpio[%d]\n", pwrl->pwr_lost_pin);
            break;
        }
		err = gpio_direction_input(pwrl->pwr_lost_pin);
		if (err < 0) {
            pr_err("failure to set direction of the gpio[%d]\n", pwrl->pwr_lost_pin);
            break;
		}
        gpio_export(pwrl->pwr_lost_pin, 0);

        INIT_DELAYED_WORK(&pwrl->pwr_lost_work, a8_power_lost_detect_work);

		pwrl->pwr_lost_irq = gpio_to_irq(pwrl->pwr_lost_pin);
		if (pwrl->pwr_lost_irq < 0) {
            pr_err("failure to request gpio[%d] irq\n", pwrl->pwr_lost_pin);
		} else {
            err = devm_request_irq(dev, pwrl->pwr_lost_irq, a8_power_lost_detect_handler,
                                   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                   pdev->name, pwrl);
            if (!err) {
                disable_irq_nosync(pwrl->pwr_lost_irq);
                device_init_wakeup(dev, 1);
//                if (device_may_wakeup(dev))
//                    disable_irq_wake(pwrl->pwr_lost_irq);
            } else {
                pr_err("failure to request irq[%d] irq\n", pwrl->pwr_lost_irq);
            }
        }

		pwrl->pwr_lost_timer = -1;
        pwrl->pwr_lost_ps = e_pwrl_unspecified;

        schedule_delayed_work(&pwrl->pwr_lost_work, (pwrl->pwr_lost_off_delay)?msecs_to_jiffies(pwrl->pwr_lost_off_delay): 0);
        pwrl->sys_ready = 1;

		pr_notice("power lost detector registered (%d, %d)\n", pwrl->pwr_lost_pin, pwrl->pwr_lost_irq);

		return 0;
	}while(0);

    unregister_cpu_notifier(&pwrl->a8_power_lost_cpu_notifier);

    if (pwrl->pwr_lost_irq >= 0) {
        disable_irq_nosync(pwrl->pwr_lost_irq); 
//        if (device_may_wakeup(dev))
//            disable_irq_wake(pwrl->pwr_lost_irq);
        device_wakeup_disable(dev);
        devm_free_irq(dev, pwrl->pwr_lost_irq, pwrl);
    }
    if(gpio_is_valid(pwrl->pwr_lost_pin))
        devm_gpio_free(dev, pwrl->pwr_lost_pin);
    dev_set_drvdata(dev, 0);
    dev_set_drvdata(pwrl->hmd, 0);
    if (!IS_ERR(pwrl->hmd)) {
#if defined (DEBUG_POWER_LOST)
        device_remove_file(pwrl->hmd, &pwrl->attr_ps.attr);
#endif
        device_remove_file(pwrl->hmd, &dev_attr_name);
        device_remove_file(pwrl->hmd, &pwrl->attr_in.attr);
        device_remove_file(pwrl->hmd, &pwrl->attr_inl.attr);
        device_remove_file(pwrl->hmd, &pwrl->attr_off_d.attr);
        device_remove_file(pwrl->hmd, &pwrl->attr_wan_d.attr);
        device_remove_file(pwrl->hmd, &pwrl->attr_wlan_d.attr);
        hwmon_device_unregister(pwrl->hmd);
    }
	devm_kfree(dev, pwrl);

	return err;
}

#if defined CONFIG_PM
static int a8_power_lost_detect_suspend(struct device *dev)
{
    struct a8_power_lost_detect_info *pwrl = dev_get_drvdata(dev);

    if (pwrl->pwr_lost_ps != e_pwrl_unspecified) {
        return -EINVAL;
    }
    spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    pwrl->pwr_lost_ps = e_pwrl_device_suspend;
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    if (device_may_wakeup(dev))
        enable_irq_wake(pwrl->pwr_lost_irq);

    pr_notice("\n");
	return 0;
}

static int a8_power_lost_detect_resume(struct device *dev)
{
    struct a8_power_lost_detect_info *pwrl = dev_get_drvdata(dev);

    spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    pwrl->pwr_lost_ps = e_pwrl_unspecified;
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    if (device_may_wakeup(dev))
        disable_irq_wake(pwrl->pwr_lost_irq);

    schedule_delayed_work(&pwrl->pwr_lost_work, 0);

	pr_notice("\n");

	return 0;
}
#else
#define a8_power_lost_detect_suspend 0
#define a8_power_lost_detect_resume 0
#endif

static const struct dev_pm_ops a8_power_lost_detect_pm_ops =
{
	.suspend	= a8_power_lost_detect_suspend,
	.resume		= a8_power_lost_detect_resume,
};

static int a8_power_lost_detect_remove(struct platform_device *pdev)
{
	struct a8_power_lost_detect_info *pwrl = platform_get_drvdata(pdev);

    cancel_delayed_work(&pwrl->pwr_lost_work);
    disable_irq_nosync(pwrl->pwr_lost_irq);
//    if (device_may_wakeup(&pdev->dev))
//        disable_irq_wake(pwrl->pwr_lost_irq);
    device_wakeup_disable(&pdev->dev);
    devm_free_irq(&pdev->dev, pwrl->pwr_lost_irq, pwrl);

    unregister_cpu_notifier(&pwrl->a8_power_lost_cpu_notifier);

    device_remove_file(pwrl->hmd, &dev_attr_name);
    device_remove_file(pwrl->hmd, &pwrl->attr_in.attr);
    device_remove_file(pwrl->hmd, &pwrl->attr_inl.attr);
    device_remove_file(pwrl->hmd, &pwrl->attr_off_d.attr);
    device_remove_file(pwrl->hmd, &pwrl->attr_wan_d.attr);
    device_remove_file(pwrl->hmd, &pwrl->attr_wlan_d.attr);
#if defined (DEBUG_POWER_LOST)
    device_remove_file(pwrl->hmd, &pwrl->attr_ps.attr);
#endif
    dev_set_drvdata(pwrl->hmd, 0);
    hwmon_device_unregister(pwrl->hmd);

	if(gpio_is_valid(pwrl->pwr_lost_pin))
		devm_gpio_free(&pdev->dev, pwrl->pwr_lost_pin);
    dev_set_drvdata(&pdev->dev, 0);

	devm_kfree(&pdev->dev, pwrl);

	return 0;
}

static void a8_power_lost_detect_shutdown(struct platform_device *pdev) {
    a8_power_lost_detect_remove(pdev);
}

static const struct of_device_id of_a8_power_lost_detect_match[] = {
	{ .compatible = "a8-hm-power-lost", },
	{},
};
MODULE_DEVICE_TABLE(of, of_a8_power_lost_detect_match);

static struct platform_driver a8_power_lost_detect_driver = {
	.probe      = a8_power_lost_detect_probe,
    .remove	    = a8_power_lost_detect_remove,
    .shutdown   = a8_power_lost_detect_shutdown,

	.driver = {
		.name = "a8_power_lost_hm",
		.of_match_table = of_match_ptr(of_a8_power_lost_detect_match),
        .pm = &a8_power_lost_detect_pm_ops,
	},
};

static int __init a8_power_lost_detect_init(void)
{
	return platform_driver_register(&a8_power_lost_detect_driver);
}

device_initcall(a8_power_lost_detect_init);

MODULE_DESCRIPTION("Power Lost Detect Hardware Monitor");
MODULE_AUTHOR("Vladimir Zatulovsky <vladimir.zatulovsky@micronet-inc.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:a8-power-lost-detect");

