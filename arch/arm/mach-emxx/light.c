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
#include <mach/pwc.h>
#include "pwm.h"

#define DEFAULT_BACKLIGHT_BRIGHTNESS 255
#define HW_MAX_BRIGHTNESS 95
#define HW_MIN_BRIGHTNESS 16

/*******************************/
/* LCD BackLight */
void
emxx_brightness_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	if(value>255) value = 255;
	if(value<1) value = 1;

	writel(0x10,PWM_CH0_CTRL);
	writel(value,PWM_CH0_LEDGE0);
	writel(0xFF,PWM_CH0_TEDGE0);
	writel(0xFF,PWM_CH0_TOTAL0);
	writel(0x11,PWM_CH0_CTRL);
}
EXPORT_SYMBOL( emxx_brightness_set );

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
		.gpio = GPIO_PWC_LED1,
		.retain_state_suspended = 1,
	},
	{
		.name = "led2",
		.gpio = GPIO_PWC_LED2,
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

static void emxx_light_gpio_configure(void)
{
	int value;
	value = readl(CHG_PINSEL_G096);
	value &= 0xFFEFFFFF;
	writel(value, CHG_PINSEL_G096);

	value = readl(CHG_PINSEL_G128);
	value |= 0x400000;
	writel(value , CHG_PINSEL_G128);

	value = readl(CHG_PINSEL_G096);
	value |= 0x100; 
	writel(value, CHG_PINSEL_G096);

	value = readl(CHG_PULL11);
	value = (value&0xFFF0FFFF)|0x50000;
	writel(value, CHG_PULL11);

	value = readl(CHG_PINSEL_G096) ;
	value |= 0x88;
	writel(value,CHG_PINSEL_G096);

	gpio_direction_output(150,1);
}
/*******************************/
static int emxx_light_probe(struct platform_device *pdev)
{
	/* Init LCD Backlight */
	writel(PWM_INIT_CLOCK, SMU_PWMPWCLKDIV);
	emxx_open_clockgate(EMXX_CLK_PWM0);
	emxx_open_clockgate(EMXX_CLK_PWM_P);
	emxx_reset_device(EMXX_RST_PWM);
	mdelay(1);
	emxx_unreset_device(EMXX_RST_PWM);

	emxx_light_gpio_configure();

	//Setup PWM registers
	writel(0x00,PWM_CH0_DELAY0);
	writel(0x01,PWM_CH0_LOOP0);
	writel(0x10,PWM_CH0_CTRL);
	writel(0x01,PWM_CH0_MODE);
	led_classdev_register(&pdev->dev, &emxx_backlight_led);

	/* Init LED */
	platform_device_register(&emxx_leds);

	return 0;
}

static int emxx_light_remove(struct platform_device *pdev)
{
	emxx_close_clockgate(EMXX_CLK_PWM0);
	emxx_close_clockgate(EMXX_CLK_PWM_P);
	emxx_reset_device(EMXX_RST_PWM);
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

