/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

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

#include <asm/gpio.h>
#include <mach/gpio.h>
#include <mach/smu.h>
#define POWER_KEY_ID	2

/* #define DEBUG 1 */
#define TAG "EMXX GPIOKEY"

#ifdef DEBUG
#define dbg_printk(fmt, args...) \
	printk(TAG " %s;%d " fmt , __FUNCTION__, __LINE__, ## args);
#else
#define dbg_printk(fmt, args...)
#endif

struct gpio_button_data {
	struct gpio_keys_button *button;
	struct input_dev *input;
	struct timer_list timer;
};

struct gpio_keys_drvdata {
	struct input_dev *input;
	struct gpio_button_data data[0];
};

static struct gpio_keys_button gpio_keys_buttons[] = {
	[0] = {
		.code        = KEY_VOLUMEDOWN,
		.gpio           = GPIO_BUTT_VOLUMEDOWN,
		.active_levl     = 0,
		.desc           = "P13",
		.debounce_interval  =50,
	},
	[1] = {
		.code        = KEY_VOLUMEUP,
		.gpio           = GPIO_BUTT_VOLUMEUP,
		.active_levl     = 0,
		.desc           = "P14",
		.debounce_interval  =50,
	},
	[2] = {
		.code        = KEY_POWER,
		.gpio           = GPIO_BUTT_POWER,
		.active_levl     = 0,
		.desc           = "P143",
		.debounce_interval  =50,
	},
};

static struct gpio_keys_platform_data gpio_keys_data = {
    .buttons   = gpio_keys_buttons,
    .nbuttons   = ARRAY_SIZE(gpio_keys_buttons),
    .rep = 1,
};

static void gpio_check_button(unsigned long _data)
{
	struct gpio_button_data *data = (struct gpio_button_data *)_data;
	struct gpio_keys_button *button = data->button;
	struct input_dev *input = data->input;
	unsigned int type = button->type ?: EV_KEY;
#ifdef CONFIG_LEDS_TRIGGER_EVENT
	extern void led_event_on(void);		//add by heyu 2011.12.23
#endif	
	int state = (gpio_get_value(button->gpio) ? 0 : 1);// ^ button->active_low;
	dbg_printk("gpio check button: %d\n",state);

	if(state) {
		if(button->prev_state==0 ) {
		dbg_printk("report Key down\n");
#ifdef CONFIG_LEDS_TRIGGER_EVENT
			if (button->code != KEY_POWER)
				led_event_on();		//add by heyu 2011.12.23
#endif
			input_event(input, type, button->code, 1);
			button->prev_state = state;
			dbg_printk("gpio_keys dn   code[%d],type=%d\n",button->code,state);
			input_sync(input);
		}
		else {
			if(button->code != KEY_POWER)
			{

				dbg_printk("gpio_keys auto repeat\n");
				input_event(input, type, button->code, 2);
				button->prev_state = state;
				//dbg_printk("gpio_keys dn   code[%d],type=%d\n",button->code,state);
				input_sync(input);
			}
		}
		mod_timer(&data->timer,jiffies + msecs_to_jiffies(300));
	}
	else {
		if(button->prev_state==1 ) {
		dbg_printk("report Key up\n");
			input_event(input, type, button->code, 0);
			button->prev_state = state;
			//dbg_printk("gpio_keys up   code[%d],type=%d\n",button->code,0);
			input_sync(input);
		}
		enable_irq(gpio_to_irq(button->gpio));
	}
}

static irqreturn_t gpio_keys_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	struct gpio_keys_button *button = bdata->button;

	BUG_ON(irq != gpio_to_irq(button->gpio));
	disable_irq_nosync(irq);

	mod_timer(&bdata->timer,jiffies + msecs_to_jiffies(button->debounce_interval));

	return IRQ_HANDLED;
}

static int __devinit gpio_keys_probe(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata =&gpio_keys_data;// pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata;
	struct input_dev *input;
	int i, error;
	int wakeup = 0;
	dbg_printk("gpio_keys_probe\n");

	ddata = kzalloc(sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		error = -ENOMEM;
		goto fail1;
	}

	platform_set_drvdata(pdev, ddata);

	input->name = pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

#if 0
	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);
#endif

	ddata->input = input;

	//GPIO 13 and GPIO14 configuration
	writel(readl(CHG_PINSEL_G000)|0x6000, CHG_PINSEL_G000);
	writel((readl(CHG_PULL14)&0xfff00fff)|0xdd000, CHG_PULL14);

	//GPIO 143 configuration
	writel(readl(CHG_PINSEL_G128)|0x8000, CHG_PINSEL_G128);
	writel((readl(CHG_PULL21)&0xfffffff0)|0x5, CHG_PULL21);

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];
		int irq;
		unsigned int type = button->type ?: EV_KEY;

		bdata->input = input;
		bdata->button = button;

		setup_timer(&bdata->timer,
				gpio_check_button, (unsigned long)bdata);

		error = gpio_request(button->gpio, button->desc ?: "gpio_keys");
		if (error < 0) {
			pr_err("gpio-keys: failed to request GPIO %d,"
				" error %d\n", button->gpio, error);
			goto fail2;
		}

		error = gpio_direction_input(button->gpio);
		if (error < 0) {
			pr_err("gpio-keys: failed to configure input"
					" direction for GPIO %d, error %d\n",
					button->gpio, error);
			gpio_free(button->gpio);
			goto fail2;
		}

		irq = gpio_to_irq(button->gpio);
		if (irq < 0) {
			error = irq;
			pr_err("gpio-keys: Unable to get irq number"
					" for GPIO %d, error %d\n",
					button->gpio, error);
			gpio_free(button->gpio);
			goto fail2;
		}

		error = request_irq(irq, gpio_keys_isr,
				IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_RISING |
				IRQF_TRIGGER_FALLING,
				button->desc ? button->desc : "gpio_keys",
				bdata);
		if (error) {
			pr_err("gpio-keys: Unable to claim irq %d; error %d\n",
				irq, error);
			gpio_free(button->gpio);
			goto fail2;
		}

		if (button->wakeup)
			wakeup = 1;

		dbg_printk("probe btn %x, wakeup %d\n", button->code, wakeup); 
		input_set_capability(input, type, button->code);
	}

	error = input_register_device(input);
	if (error) {
		pr_err("gpio-keys: Unable to register input device, "
			"error: %d\n", error);
		goto fail2;
	}
	
	device_init_wakeup(&pdev->dev, wakeup);

	return 0;

 fail2:
	while (--i >= 0) {
		free_irq(gpio_to_irq(pdata->buttons[i].gpio), &ddata->data[i]);
		if (pdata->buttons[i].debounce_interval)
			del_timer_sync(&ddata->data[i].timer);
		gpio_free(pdata->buttons[i].gpio);
	}

	platform_set_drvdata(pdev, NULL);
 fail1:
	input_free_device(input);
	kfree(ddata);

	return error;
}

static int __devexit gpio_keys_remove(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata =&gpio_keys_data;// pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < pdata->nbuttons; i++) {
		int irq = gpio_to_irq(pdata->buttons[i].gpio);
		free_irq(irq, &ddata->data[i]);
		if (pdata->buttons[i].debounce_interval)
			del_timer_sync(&ddata->data[i].timer);
		gpio_free(pdata->buttons[i].gpio);
	}

	input_unregister_device(input);

	return 0;
}


#ifdef CONFIG_PM
static int gpio_keys_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct gpio_keys_platform_data *pdata =&gpio_keys_data;// pdev->dev.platform_data;
	int i;

dbg_printk("GPIO key suspend\n");
	if (device_may_wakeup(&pdev->dev)) {
dbg_printk("device_may_wakeup\n");
		for (i = 0; i < pdata->nbuttons; i++) {
dbg_printk("i=%d\n", i);
			struct gpio_keys_button *button = &pdata->buttons[i];
			if (button->wakeup) {
dbg_printk("wakeup");
				int irq = gpio_to_irq(button->gpio);
				enable_irq_wake(irq);
			}
		}
	}

	return 0;
}

static int gpio_keys_resume(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata =&gpio_keys_data;// pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	int i;

	if (device_may_wakeup(&pdev->dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct gpio_keys_button *button = &pdata->buttons[i];
			if (button->wakeup) {
				int irq = gpio_to_irq(button->gpio);
				disable_irq_wake(irq);
			}
		}
	}
	{
		struct gpio_keys_button *button = &pdata->buttons[POWER_KEY_ID];
		struct input_dev *input = ddata->input;
		int state = (gpio_get_value(button->gpio) ? 0 : 1);
		//dbg_printk("GPIO key read %d=%d", button->gpio, state);

		dbg_printk("report Key down\n");
		input_event(input, EV_KEY, button->code, 1);
		//button->prev_state = state;
		dbg_printk("gpio_keys dn code[%d],type=%d\n",button->code,state);
		input_sync(input);

		dbg_printk("report Key up\n");
		input_event(input, EV_KEY, button->code, 0);
		//button->prev_state = state;
		input_sync(input);

	}
	dbg_printk("GPIO key resume\n");

	return 0;
}
#else
#define gpio_keys_suspend	NULL
#define gpio_keys_resume	NULL
#endif

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= __devexit_p(gpio_keys_remove),
	.suspend	= gpio_keys_suspend,
	.resume		= gpio_keys_resume,
	.driver		= {
		.name	= "gpio-keys",
		.owner	= THIS_MODULE,
	}
};

static int __init gpio_keys_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

module_init(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for CPU GPIOs");
MODULE_ALIAS("platform:gpio-keys");
