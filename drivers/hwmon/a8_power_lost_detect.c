#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/reboot.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#undef DEBUG_POWER_LOST
    int remount_complete;
    struct notifier_block a8_power_lost_cpu_notifier;
    struct notifier_block a8_power_lost_remount_notifier;
    int dbg_lvl;
    return sprintf(buf, "%d\n", pwrl->pwr_lost_ps);
    pwrl->pwr_lost_ps = val;
    pwrl->pwr_lost_timer = ktime_to_ms(ktime_get());

    schedule_delayed_work(&pwrl->pwr_lost_work, 0);
    if (!pwrl->sys_ready) {
        return -EINVAL;
    }

int click_register_notifier(struct notifier_block *nb)
	raw_spin_lock_irqsave(&click_lock, flags);
	err = raw_notifier_chain_register(&click_chain, nb);
	raw_spin_unlock_irqrestore(&click_lock, flags);
EXPORT_SYMBOL(click_register_notifier);
#if 0
// example notification using

static int click_notifier(struct notifier_block *nb, unsigned long val, void *priv)
	struct klight_leds_data *kl = container_of(nb, struct klight_leds_data, click_notifier);


power_lost_register_notifier(<specific> *dev_specific->power_lost_notifier)
		register_cpu_notifier(&msm_thermal_cpu_notifier);
// Vladimir
// TODO: implement remount completing wait
//
#if 0
		unsigned long action, void *hcpu)
	uint32_t cpu = (uintptr_t)hcpu;
    char mount_dev[256];
	if (action == CPU_UP_PREPARE || action == CPU_UP_PREPARE_FROZEN) {
    char mount_type[256];
    char mount_opts[256];
    int mount_freq;
    int mount_passno;
    int match;
    int found_rw_fs = 0;
    f = sys_open("/proc/mounts", O_RDONLY, 0);
    if (! f) {
			pr_notice("Prevent cpu%d up\n", cpu);
			return NOTIFY_BAD;
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
    } while (match != EOF);
    fclose(f);
    return !found_rw_fs;
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

static void a8_power_lost_detect_work(struct work_struct *work)
    int val;//, err;
#if defined (DEBUG_POWER_LOST)
    val |= pwrl->dbg_lvl;
#endif
            power_lost_notify(1, 0);
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
    } else if (pwrl->pwr_lost_ps == e_pwrl_wan_off) {
        pr_notice("shutdown wan %lld\n", ktime_to_ms(ktime_get()));

        if (pwrl->pwr_lost_wlan_delay > (ktime_to_ms(ktime_get()) - pwrl->pwr_lost_timer)) {
            val = pwrl->pwr_lost_timer + pwrl->pwr_lost_wlan_delay - ktime_to_ms(ktime_get());
            pwrl->pwr_lost_ps = e_pwrl_wlan_off;
            pr_notice("wlan may be shutdown %lld\n", ktime_to_ms(ktime_get()));
            pr_notice("shutdown cores %lld\n", ktime_to_ms(ktime_get()));
            if (pwrl->pwr_lost_off_delay > (ktime_to_ms(ktime_get()) - pwrl->pwr_lost_timer)) {
            pwrl->pwr_lost_ps = e_pwrl_cores_off;
            timer = 0;
#endif
        timer = val;
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);
			printk("%s: system not ready; urgent shutdown %lld\n", __func__, ktime_to_ms(ktime_get()));
        if (pwrl->pwr_lost_wan_delay > (ktime_to_ms(ktime_get()) - pwrl->pwr_lost_timer)) {
            val = pwrl->pwr_lost_timer + pwrl->pwr_lost_wan_delay - ktime_to_ms(ktime_get());
            pwrl->pwr_lost_ps = e_pwrl_wan_off;
            printk("%s: fs syncked %lld\n", __func__, ktime_to_ms(ktime_get()));
        } else {
            printk("%s: urgent shutdown %lld\n", __func__, ktime_to_ms(ktime_get()));
            machine_power_off();
                val = pwrl->pwr_lost_timer + pwrl->pwr_lost_off_delay - ktime_to_ms(ktime_get());
            } else {
                val = 0;
		if (pld->ps != pwl_unspecified) {
        timer = val;
    } else if (pwrl->pwr_lost_ps == e_pwrl_device_off) {
		pld->pld_work_timer = pld->pld_ignore_timer;
        spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);
			printk("%s: Shutdown fs %lld\n", __func__, ktime_to_ms(ktime_get()));

        remount_ro(pwrl);
				printk("%s: fs in safe mode %lld\n", __func__, ktime_to_ms(ktime_get()));
        orderly_poweroff(1);
			}
    } else {
			printk("%s: urgent shutdown %lld\n", __func__, ktime_to_ms(ktime_get()));
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    schedule_delayed_work(&pwrl->pwr_lost_work, (timer)?msecs_to_jiffies(timer): 0);
	}
	mutex_unlock(&pld->work_lock);
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

    pwrl->sys_ready = 0;
    pwrl->dbg_lvl = 0;
    pwrl->a8_power_lost_cpu_notifier.notifier_call = a8_power_lost_cpu_callback;
    pwrl->a8_power_lost_remount_notifier.notifier_call = a8_power_lost_remount_callback;

    err = register_cpu_notifier(&pwrl->a8_power_lost_cpu_notifier);
    if (err) {
        pr_err("failure to register cpu notifier [%d]\n", err);
                                   IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
        pwrl->sys_ready = 1;

    unregister_cpu_notifier(&pwrl->a8_power_lost_cpu_notifier);

    unregister_cpu_notifier(&pwrl->a8_power_lost_cpu_notifier);
