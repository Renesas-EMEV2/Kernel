#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/init.h>

static DEFINE_SPINLOCK(event_lock);
static unsigned event_count = 0;
static unsigned event_flag = 0;
static unsigned event_supend = 0;
static struct timer_list *event_led_timer = NULL;
#define LED_BRIGHT_JIFFIES		500		//ms
#define LED_BRIGHT_TIMES		60		//LED_BRIGHT_JIFFIES*LED_BRIGHT_TIMES

static void led_event_get(unsigned cnt)
{
	spin_lock(&event_lock);
	event_count = cnt;
	spin_unlock(&event_lock);
	if (event_led_timer && !event_flag)
	{	
		spin_lock(&event_lock);
		event_flag = 1;
		spin_unlock(&event_lock);
		mod_timer(event_led_timer, jiffies+1);
	}	
}

void led_event_on(void)
{
	if (!event_supend)
		led_event_get(LED_BRIGHT_TIMES);
}
EXPORT_SYMBOL_GPL(led_event_on);

void led_event_off(void)
{	
	led_event_get(0);	
}
EXPORT_SYMBOL_GPL(led_event_off);

void led_event_suspend(void)
{
	spin_lock(&event_lock);
	event_supend = 1;
	spin_unlock(&event_lock);
}
EXPORT_SYMBOL_GPL(led_event_suspend);

void led_event_wakeup(void)
{
	spin_lock(&event_lock);
	event_supend = 0;
	spin_unlock(&event_lock);
}
EXPORT_SYMBOL_GPL(led_event_wakeup);

static void event_led_function(unsigned long data)
{
	struct led_classdev *led_cdev = (struct led_classdev *) data;	
	struct led_trigger *trigger = led_cdev->trigger;	
	
	if (event_count > 0)		// led on
	{		
		spin_lock(&event_lock);
		event_count--;
		spin_unlock(&event_lock);		
		led_trigger_event(trigger, LED_FULL);
		mod_timer(event_led_timer, jiffies+msecs_to_jiffies(LED_BRIGHT_JIFFIES));		
	}
	else
	{
		led_trigger_event(trigger, LED_OFF);
		spin_lock(&event_lock);
		event_flag = 0;
		spin_unlock(&event_lock);
	}
}

static void event_led_trig_activate(struct led_classdev *led_cdev)
{	
	if (!led_cdev)
		return;

	if (!led_cdev->trigger_data)
	{
		event_led_timer = kzalloc(sizeof(struct timer_list), GFP_KERNEL);
		if (!event_led_timer)
			return;
		
		init_timer(event_led_timer);
		event_led_timer->function = event_led_function;
		event_led_timer->data = (unsigned long) led_cdev;		
	}
	
	if (event_count)
	{
		spin_lock(&event_lock);
		event_flag = 1;
		spin_unlock(&event_lock);
		mod_timer(event_led_timer, jiffies+1);
	}	
}

static void event_led_trig_deactivate(struct led_classdev *led_cdev)
{	
	if (event_led_timer) {
		spin_lock(&event_lock);
		if (event_flag)
			del_timer_sync(event_led_timer);		
		event_count = 0;
		event_flag = 0;
		spin_unlock(&event_lock);
		kfree(event_led_timer);
		event_led_timer = NULL;
	}
}

static struct led_trigger event_led_trigger = {
	.name     = "event",
	.activate = event_led_trig_activate,
	.deactivate = event_led_trig_deactivate,
};

static int __init event_led_trig_init(void)
{
	return led_trigger_register(&event_led_trigger);
}

static void __exit event_led_trig_exit(void)
{
	led_trigger_unregister(&event_led_trigger);
}

module_init(event_led_trig_init);
module_exit(event_led_trig_exit);

MODULE_AUTHOR("HeYu <heyu@smallart.com.cn>");
MODULE_DESCRIPTION("EMXX LED trigger");
MODULE_LICENSE("GPL");

