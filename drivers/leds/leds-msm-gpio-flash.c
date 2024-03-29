
/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/list.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>

// added by yangze for led-flash (ql1700) 2014-08-22
#include <linux/workqueue.h>

#define LED_GPIO_FLASH_DRIVER_NAME	"qcom,leds-gpio-flash"
#define LED_TRIGGER_DEFAULT		"none"

struct led_gpio_flash_data {
	int flash_en;
	int flash_now;
	int brightness;
	struct led_classdev cdev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_default;
	// added by yangze for led-flash (ql1700) 2014-08-22 begin
	struct delayed_work    delayed_work;
	unsigned int delay_ms;
	// added by yangze for led-flash (ql1700) 2014-08-22 end
};


static struct of_device_id led_gpio_flash_of_match[] = {
	{.compatible = LED_GPIO_FLASH_DRIVER_NAME,},
	{},
};

// added by yangze for led-flash (ql1700) 2014-08-22 begin
static void led_gpio_flash_off(struct work_struct *work)
{
	int rc = 0;
	struct led_gpio_flash_data *flash_led = container_of(work, struct led_gpio_flash_data, delayed_work.work);

	rc = gpio_direction_output(flash_led->flash_en, 0);
	if (rc) {
		pr_err("%s: Failed to set gpio %d\n", __func__,
		       flash_led->flash_en);
		goto err;
	}
	rc = gpio_direction_output(flash_led->flash_now, 0);
	if (rc) {
		pr_err("%s: Failed to set gpio %d\n", __func__,
		       flash_led->flash_now);
		goto err;
	}
	flash_led->brightness = 0;
err:
	return;	
}
// added by yangze for led-flash (ql1700) 2014-08-22 end

static void led_gpio_brightness_set(struct led_classdev *led_cdev,
				    enum led_brightness value)
{
	int rc = 0;
	struct led_gpio_flash_data *flash_led =
	    container_of(led_cdev, struct led_gpio_flash_data, cdev);

	int brightness = value;
	int flash_en = 0, flash_now = 0;
//Added by zhaochengliang for change flash control logic (QL1002) SW000000 2014/02/19 begin
	if (brightness > LED_HALF) {
		flash_en = 1;
		flash_now = 1;//jerry add for SGM3785 test
	} else if (brightness > LED_OFF) {
		flash_en = 0;
		flash_now = 1;
	} else {
		flash_en = 0;
		flash_now = 0;
	}
//Added by zhaochengliang for change flash control logic (QL1002) SW000000 2014/02/19 end

// modified by yangze for led-flash (ql1700) 2014-08-22 begin
	if((flash_led->brightness>LED_OFF)&&(flash_led->brightness <= LED_HALF)&&(brightness<=LED_OFF)){
		cancel_delayed_work(&(flash_led->delayed_work));
		schedule_delayed_work(&(flash_led->delayed_work), msecs_to_jiffies(flash_led->delay_ms));
	}else{
		cancel_delayed_work(&(flash_led->delayed_work));
		rc = gpio_direction_output(flash_led->flash_en, flash_en);
		if (rc) {
			pr_err("%s: Failed to set gpio %d\n", __func__,
			       flash_led->flash_en);
			goto err;
		}
		rc = gpio_direction_output(flash_led->flash_now, flash_now);
		if (rc) {
			pr_err("%s: Failed to set gpio %d\n", __func__,
			       flash_led->flash_now);
			goto err;
		}
		flash_led->brightness = brightness;
	}
// modified by yangze for led-flash (ql1700) 2014-08-22 end	
err:
	return;
}

static enum led_brightness led_gpio_brightness_get(struct led_classdev
						   *led_cdev)
{
	struct led_gpio_flash_data *flash_led =
	    container_of(led_cdev, struct led_gpio_flash_data, cdev);
	return flash_led->brightness;
}

int led_gpio_flash_probe(struct platform_device *pdev)
{
	int rc = 0;
	const char *temp_str;
	struct led_gpio_flash_data *flash_led = NULL;
	struct device_node *node = pdev->dev.of_node;
	
/* add by zhanghuan 20141117 for disable regulator 8916_l15 */
//#if defined(CONFIG_QL1001_P2170) || defined(CONFIG_QL1001_J20) || defined(CONFIG_QL1001_J20TMP) || defined(CONFIG_QL1001_P2170FHD)
	struct regulator *reg_8916_l15;

	reg_8916_l15 = regulator_get(NULL,"8916_l15");
	rc = regulator_enable(reg_8916_l15);
	
	if(rc){
		dev_err(&pdev->dev, "%s: Failed enable regulator 8916_l15. rc = %d\n",
			__func__, rc);
		goto error;
	}
//#endif
/* add by zhanghuan 20141117 end */
	
	flash_led = devm_kzalloc(&pdev->dev, sizeof(struct led_gpio_flash_data),
				 GFP_KERNEL);
	if (flash_led == NULL) {
		dev_err(&pdev->dev, "%s:%d Unable to allocate memory\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	flash_led->cdev.default_trigger = LED_TRIGGER_DEFAULT;
	rc = of_property_read_string(node, "linux,default-trigger", &temp_str);
	if (!rc)
		flash_led->cdev.default_trigger = temp_str;

	flash_led->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(flash_led->pinctrl)) {
		pr_err("%s:failed to get pinctrl\n", __func__);
		return PTR_ERR(flash_led->pinctrl);
	}

	flash_led->gpio_state_default = pinctrl_lookup_state(flash_led->pinctrl,
		"ocp8110_default");
	if (IS_ERR(flash_led->gpio_state_default)) {
		pr_err("%s:can not get active pinstate\n", __func__);
		return -EINVAL;
	}

	rc = pinctrl_select_state(flash_led->pinctrl,
		flash_led->gpio_state_default);
	if (rc)
		pr_err("%s:set state failed!\n", __func__);

	flash_led->flash_en = of_get_named_gpio(node, "qcom,flash-en", 0);
	if (flash_led->flash_en < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"flash-en", node->full_name, flash_led->flash_en);
		goto error;
	} else {
		rc = gpio_request(flash_led->flash_en, "FLASH_EN");
		if (rc) {
			dev_err(&pdev->dev,
				"%s: Failed to request gpio %d,rc = %d\n",
				__func__, flash_led->flash_en, rc);

			goto error;
		}
	}

	flash_led->flash_now = of_get_named_gpio(node, "qcom,flash-now", 0);
	if (flash_led->flash_now < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"flash-now", node->full_name, flash_led->flash_now);
		goto error;
	} else {
		rc = gpio_request(flash_led->flash_now, "FLASH_NOW");
		if (rc) {
			dev_err(&pdev->dev,
				"%s: Failed to request gpio %d,rc = %d\n",
				__func__, flash_led->flash_now, rc);
			goto error;
		}
	}

	rc = of_property_read_string(node, "linux,name", &flash_led->cdev.name);
	if (rc) {
		dev_err(&pdev->dev, "%s: Failed to read linux name. rc = %d\n",
			__func__, rc);
		goto error;
	}
	
// added by yangze for led-flash (ql1700) 2014-08-22 begin
	rc = of_property_read_u32(node, "qcom,delay-ms",&flash_led->delay_ms);
	if (rc){
		flash_led->delay_ms = HZ/2;
	}
	//dev_err(&pdev->dev, "%s: qcom,delay-ms = %d\n",__func__,flash_led->delay_ms);
// added by yangze for led-flash (ql1700) 2014-08-22 end	

	platform_set_drvdata(pdev, flash_led);
	flash_led->cdev.max_brightness = LED_FULL;
	flash_led->cdev.brightness_set = led_gpio_brightness_set;
	flash_led->cdev.brightness_get = led_gpio_brightness_get;

	rc = led_classdev_register(&pdev->dev, &flash_led->cdev);
	if (rc) {
		dev_err(&pdev->dev, "%s: Failed to register led dev. rc = %d\n",
			__func__, rc);
		goto error;
	}
	// added by yangze for led-flash (ql1700) 2014-08-22
	INIT_DELAYED_WORK(&(flash_led->delayed_work), led_gpio_flash_off); 
	
	pr_err("%s:probe successfully!\n", __func__);
	return 0;

error:
	devm_kfree(&pdev->dev, flash_led);
	return rc;
}

int led_gpio_flash_remove(struct platform_device *pdev)
{
	struct led_gpio_flash_data *flash_led =
	    (struct led_gpio_flash_data *)platform_get_drvdata(pdev);

	led_classdev_unregister(&flash_led->cdev);
	devm_kfree(&pdev->dev, flash_led);
	return 0;
}

static struct platform_driver led_gpio_flash_driver = {
	.probe = led_gpio_flash_probe,
	.remove = led_gpio_flash_remove,
	.driver = {
		   .name = LED_GPIO_FLASH_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = led_gpio_flash_of_match,
		   }
};

static int __init led_gpio_flash_init(void)
{
	return platform_driver_register(&led_gpio_flash_driver);
}

static void __exit led_gpio_flash_exit(void)
{
	return platform_driver_unregister(&led_gpio_flash_driver);
}

late_initcall(led_gpio_flash_init);
module_exit(led_gpio_flash_exit);

MODULE_DESCRIPTION("QCOM GPIO LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("leds:leds-msm-gpio-flash");
