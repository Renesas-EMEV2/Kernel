/*
 *  File Name       : arch/arm/mach-emxx/light.c
 *  Function        : LED
 *  Release Version : Ver 1.01
 *  Release Date    : 2011/01/21
 *
 * Copyright (C) 2010 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>

#include <linux/gpio.h>
#include <mach/pwm.h>
#include "pwm.h"

#define DEFAULT_BACKLIGHT_BRIGHTNESS 255
#define HW_MAX_BRIGHTNESS 255
#define HW_MIN_BRIGHTNESS 0

struct emxx_pwm_cmpcnt_t pwm_cmpcnt;

/*******************************/
/* LCD BackLight */
static void
emxx_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
#if 0
	int hw_val = (value * HW_MAX_BRIGHTNESS) / 255;

	if (hw_val < HW_MIN_BRIGHTNESS)
		hw_val = HW_MIN_BRIGHTNESS;
	if (hw_val > HW_MAX_BRIGHTNESS)
		hw_val = HW_MAX_BRIGHTNESS;
#endif

	pwm_cmpcnt.trail_edge = (1995*(HW_MAX_BRIGHTNESS-value+HW_MIN_BRIGHTNESS))/HW_MAX_BRIGHTNESS;
	if(pwm_cmpcnt.trail_edge < pwm_cmpcnt.lead_edge)
		pwm_cmpcnt.trail_edge = pwm_cmpcnt.lead_edge;
	//pwm_cmpcnt.trail_edge = pwm_cmpcnt.total_cycle - pwm_cmpcnt.lead_edge;
	pwm_cmpcnt.loop_count = 0;
	emxx_pwm_stop(0);
	emxx_pwm_set_compare_counter(0, &pwm_cmpcnt);
	emxx_pwm_start(0);
}

static struct led_classdev emxx_backlight_led = {
	.name = "lcd-backlight",
	.brightness = DEFAULT_BACKLIGHT_BRIGHTNESS,
	.brightness_set = emxx_brightness_set,
};


/*******************************/
/* LED */
static struct gpio_led emxx_led_list[] = {
	{
		.name = "led1",
		.gpio = GPIO_NULL,
		.retain_state_suspended = 1,
	},
	{
		.name = "led2",
		.gpio = GPIO_NULL,
		.retain_state_suspended = 1,
	},
};
static struct gpio_led_platform_data emxx_leds_data = {
	.num_leds	= ARRAY_SIZE(emxx_led_list),
	.leds		= emxx_led_list,
};
static struct platform_device emxx_leds = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &emxx_leds_data,
	},
};

/*******************************/
static int emxx_light_probe(struct platform_device *pdev)
{
	struct emxx_pwm_ch_config_t ch_config;
	pwm_cmpcnt.cb_info = NULL;
	emxx_pwm_get_channel_config(0, &ch_config);
	emxx_pwm_get_compare_counter(0, &pwm_cmpcnt);

	ch_config.mode = 0;
	ch_config.use_cmp = PWM_CMP_EN0_BIT;
	emxx_pwm_set_channel_config(0, &ch_config);

	pwm_cmpcnt.cmp = EMXX_PWM_COMPARE0;
	pwm_cmpcnt.delay = 0;
	pwm_cmpcnt.lead_edge = 5;
	pwm_cmpcnt.trail_edge = 1200;
	pwm_cmpcnt.total_cycle = 2000;		//about 14.2KHz
	pwm_cmpcnt.loop_count = 0;
	pwm_cmpcnt.cb_info = NULL;
	emxx_pwm_set_compare_counter(0, &pwm_cmpcnt);

	emxx_pwm_start(0);
	/* Init LCD Backlight */
	led_classdev_register(&pdev->dev, &emxx_backlight_led);

	/* Init LED */
	platform_device_register(&emxx_leds);

	return 0;
}

static int emxx_light_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&emxx_backlight_led);

	platform_device_unregister(&emxx_leds);

	return 0;
}

static struct platform_driver emxx_light_driver = {
	.probe		= emxx_light_probe,
	.remove		= emxx_light_remove,
	.driver		= {
		.name	= "emxx-light",
		.owner	= THIS_MODULE,
	},
};

static int __init emxx_light_init(void)
{
	return platform_driver_register(&emxx_light_driver);
}

late_initcall(emxx_light_init);

