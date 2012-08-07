/*
 *  File Name          : driver/net/emev-rfkill.c
 *  Function           : emev_board
 *  Release Version    : Ver 1.07
 *  Release Date       : 2010/10/29
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
#include <mach/hardware.h>
#include <mach/smu.h>
#include <linux/gpio.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/emev-rfkill.h>
#include <linux/delay.h>

#define DEBUG
#define TAG "bt_rfkill "

#ifdef DEBUG
#define debug_print(fmt, args...)	\
		printk( TAG "%s:%d: " fmt , __FUNCTION__ , __LINE__ , ## args);
#else
#define debug_print(fmt, args...)
#endif

/* GPIO controls for Renesas EMEV - Livall tablet board */
#define BT_RST		GPIO_BCM_BT_RST
#define BCM_BT_RST	GPIO_BCM_BT_RST
#define BCM_WLAN_RST	GPIO_BCM_WLAN_RST 
#define BCM_WLAN_BT_EN	GPIO_BCM_WLAN_BT_EN

static void bt_hw_init(void)
{
	debug_print("Entering\n");
	/* Reset BCM chipset for BT */
	writel(readl(CHG_PINSEL_G096)|0xC0000, CHG_PINSEL_G096);
	writel(readl(CHG_PINSEL_G128)&(~0x60000000), CHG_PINSEL_G128);
	writel(readl(CHG_PINSEL_UART)&(~0x00000003), CHG_PINSEL_UART);
}

static int emev_rfkill_set_radio_block(void *data, bool blocked)
{
	debug_print("Rfkill bt set power: %d\n", blocked);
	
	if (blocked) {
		debug_print("rfkill bt Blocked\n");
		
		gpio_direction_output(BCM_BT_RST, 0);
		
		if (!gpio_get_value(BCM_WLAN_RST)) {
			debug_print("WLAN BLOCKED, set EN to 0 \n");
			gpio_direction_output(BCM_WLAN_BT_EN, 0);
		} else {
			debug_print("WLAN UNBLOCKED, EN not change \n");
		}
	} else {
		debug_print("rfkill bt Unblocked\n");
		
		if (!gpio_get_value(BCM_WLAN_BT_EN)) {
		   gpio_direction_output(BCM_WLAN_BT_EN, 1);
		}
		
		gpio_direction_output(BCM_BT_RST, 0);
		mdelay(300);            
		gpio_direction_output(BCM_BT_RST, 1); 
	}
	
   return 0;
}

static const struct rfkill_ops emev_bt_rfkill_ops = {
	.set_block = emev_rfkill_set_radio_block,
};

static int emev_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct emev_rfkill_platform_data *pdata = pdev->dev.platform_data;
	debug_print("Rfkill bt probe\n");

	bt_hw_init();

	pdata->rfkill = rfkill_alloc("emev_bt", 
				&pdev->dev, 
				RFKILL_TYPE_BLUETOOTH, 
				&emev_bt_rfkill_ops, 
								NULL);

	if (unlikely(!pdata->rfkill)) {
		return -ENOMEM;
	}

	/* set default status */
	rfkill_set_states(pdata->rfkill, true, false);

	rc = rfkill_register(pdata->rfkill);

	if (unlikely(rc)) {
		rfkill_destroy(pdata->rfkill);
	}

	return 0;
}


static int emev_rfkill_remove(struct platform_device *pdev)
{
	struct emev_rfkill_platform_data *pdata = pdev->dev.platform_data;
	debug_print("Entering\n");

	rfkill_unregister(pdata->rfkill);
	rfkill_destroy(pdata->rfkill);

	return 0;
}

static struct platform_driver emev_rfkill_platform_driver = {
	.probe = emev_rfkill_probe,
	.remove = emev_rfkill_remove,
	.driver.name = "emev-rfkill",
	.driver.bus  = &platform_bus_type,
};

static int __init emev_rfkill_init(void)
{
	int ret;
	ret=platform_driver_register(&emev_rfkill_platform_driver);
	debug_print("Rfkill bt init %d\n", ret);
	
	return ret;
}

static void __exit emev_rfkill_exit(void)
{
    platform_driver_unregister(&emev_rfkill_platform_driver);
}

module_init(emev_rfkill_init);
module_exit(emev_rfkill_exit);

MODULE_ALIAS("platform:emev");
MODULE_DESCRIPTION("emev-rfkill");
MODULE_LICENSE("GPL");

