/*
 * linux/drivers/power/axp192_fake_battery.c
 *
 * Battery measurement code for emev platform.
 *
 * based on palmtx_battery.c
 *
 * Copyright (C) 2009 Samsung Electronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <linux/timer.h>
#include <linux/threads.h>
#include <linux/slab.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>

#define DRIVER_NAME	"axp192_battery"

#define WAKE_LOCK 1
#if WAKE_LOCK
static struct wake_lock vbus_wake_lock;
#endif

#define DBUG_ENABLE     0
#if DBUG_ENABLE
#define BATT(x...)      printk(KERN_INFO "[BATT:] " x)
#else
#define BATT(x...)      do {} while (0)
#endif

/* Prototypes */
extern int axp192_adc_get_adc_data(int channel);

static ssize_t axp192_bat_show_property(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf);
static ssize_t axp192_bat_store(struct device *dev, 
			     struct device_attribute *attr,
			     const char *buf, size_t count);

extern int axp192_battery_vol(void);
extern int axp192_is_battery_in(void);
extern int set_poweroff_vol(unsigned char vol);
extern int axp192_battery_current_direction(void);
extern int axp192_ac_status(void);
extern int axp192_usb_status(void);
extern int axp192_battery_discharging_cur(void);
extern int axp192_battery_charging_status(void);
extern int axp192_battery_charging_cur(void);

static struct device *dev;
static struct timer_list my_timer;
static int axp192_battery_initial;
static int force_update;
static struct work_struct *hardwork;
static struct workqueue_struct *batt_wq;

static char *status_text[] = {
	[POWER_SUPPLY_STATUS_UNKNOWN] =		"Unknown",
	[POWER_SUPPLY_STATUS_CHARGING] =	"Charging",
	[POWER_SUPPLY_STATUS_DISCHARGING] =	"Discharging",
	[POWER_SUPPLY_STATUS_NOT_CHARGING] =	"Not Charging",
	[POWER_SUPPLY_STATUS_FULL] =		"Full",
};

typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC,
	CHARGER_DISCHARGE
} charger_type_t;

struct battery_info {
	u32 batt_id;		/* Battery ID from ADC */
	u32 batt_vol;		/* Battery voltage from ADC */
	u32 batt_vol_adc;	/* Battery ADC value */
	u32 batt_vol_adc_cal;	/* Battery ADC value (calibrated)*/
	u32 batt_temp;		/* Battery Temperature (C) from ADC */
	u32 batt_temp_adc;	/* Battery Temperature ADC value */
	u32 batt_temp_adc_cal;	/* Battery Temperature ADC value (calibrated) */
	u32 batt_charging_cur;       /* Battery current while charging */
	u32 batt_discharging_cur;    /* Battery current while discharging */
	u32 batt_current;       /* Battery current = discharging - charging */
	u32 level;		/* formula */
	u32 charging_source;	/* 0: no cable, 1:usb, 2:AC */
	u32 charging_enabled;	/* 0: Disable, 1: Enable */
	u32 batt_health;	/* Battery Health (Authority) */
	u32 batt_is_full;       /* 0 : Not full 1: Full */
};

/* lock to protect the battery info */
static DEFINE_MUTEX(work_lock);

//[][0]: Percent; [][1]:Voltage
static const unsigned long s_arrPercent[][2] = {
    {100, 4050}, {98, 3950}, {10, 3535}, {0, 3450}, 
};

#define BATTERY_VOLUME_MAX 4200 // mV
#define UPDATE_CYCLE 5	// secconds

struct axp192_battery_info {
	int present;
	int polling;
	unsigned long polling_interval;

	struct battery_info bat_info;
};
static struct axp192_battery_info axp192_bat_info;
static int axp192_get_bat_level(struct power_supply *bat_ps)
{
	int level;
	if(axp192_bat_info.bat_info.batt_vol >= s_arrPercent[0][1])
		level = s_arrPercent[0][0];
	else if((axp192_bat_info.bat_info.batt_vol >= s_arrPercent[1][1]))
			level = ((s_arrPercent[0][0] - s_arrPercent[1][0]) * 1000 /
			(s_arrPercent[0][1]-s_arrPercent[1][1]) * 
			(axp192_bat_info.bat_info.batt_vol - s_arrPercent[1][1])) / 1000 + s_arrPercent[1][0];
	else if((axp192_bat_info.bat_info.batt_vol >= s_arrPercent[2][1]))
			level = ((s_arrPercent[1][0] - s_arrPercent[2][0]) * 1000 /
			(s_arrPercent[1][1]-s_arrPercent[2][1]) * 
			(axp192_bat_info.bat_info.batt_vol - s_arrPercent[2][1])) / 1000 + s_arrPercent[2][0];
	else if((axp192_bat_info.bat_info.batt_vol >= s_arrPercent[3][1]))
			level = ((s_arrPercent[2][0] - s_arrPercent[3][0]) * 1000 /
			(s_arrPercent[2][1]-s_arrPercent[3][1]) * 
			(axp192_bat_info.bat_info.batt_vol - s_arrPercent[3][1])) / 1000 + s_arrPercent[3][0];
	else level = 0;
	if(axp192_is_battery_in() == 0)
		level = 100;
	BATT("level = %d\n",level);
	return level;
}

static int axp192_get_bat_vol(struct power_supply *bat_ps)
{	
	static int old_vol,vol_flag = 1;
	int max_vol,min_vol,vol,i;
	int bat_vol,bat_discharge_cur,bat_charge_cur;
	static int bat_vol_old = BATTERY_VOLUME_MAX;
	if(axp192_is_battery_in() == 1){
		max_vol = axp192_battery_vol();
		min_vol = axp192_battery_vol();
		if(min_vol > max_vol){
			vol = min_vol;
			min_vol = max_vol;
			max_vol = vol;
		}
		bat_vol = max_vol + min_vol;
		for(i = 8;i > 0;i--){
			vol = axp192_battery_vol();
			bat_vol += vol;
			if(vol > max_vol)
				max_vol = vol;
			else if(vol < min_vol)
				min_vol = vol;
		}
		bat_vol = (bat_vol - min_vol - max_vol)/8;
		//bat_vol = axp192_battery_vol();
		bat_discharge_cur = axp192_battery_discharging_cur();
		bat_charge_cur = axp192_battery_charging_cur();
                BATT("bat_vol = %d; charg_cur = %d; discharge_cur = %d\n", bat_vol, bat_charge_cur, bat_discharge_cur);
		bat_vol = bat_vol + bat_discharge_cur * 2 / 10;
		bat_vol = bat_vol - bat_charge_cur * 2 / 10;
		if(vol_flag == 0){
                        if((axp192_ac_status() == 1) && (bat_vol < bat_vol_old))
                                bat_vol = bat_vol_old;
		}
		else 
			vol_flag = 0;	
		if (0 == bat_vol) {
			bat_vol = bat_vol_old;
			BATT("%s bat_vol read failed, use previous!!!\n",__func__,bat_vol);
		}
		else{
			bat_vol_old = bat_vol;
		}
	}	
	else {
		bat_vol = BATTERY_VOLUME_MAX;
		BATT("%s axp192_is_battery_in read failed!!!\n",__func__,bat_vol);
	}
	BATT("%s bat_vol = %d\n",__func__,bat_vol);
	//return bat_vol;
	return bat_vol;
}

static u32 axp192_get_bat_health(void)
{
	return axp192_bat_info.bat_info.batt_health;
}

static int axp192_get_bat_temp(struct power_supply *bat_ps)
{
	int temp = 0;

	return temp;
}

static int axp192_bat_get_charging_status(void)
{
	charger_type_t charger = CHARGER_BATTERY; 
	int ret = 0;
        
	charger = axp192_bat_info.bat_info.charging_source;
        
	switch (charger) {
	case CHARGER_BATTERY:
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case CHARGER_USB:
	case CHARGER_AC:
		if (axp192_bat_info.bat_info.level == 100 
			&& axp192_bat_info.bat_info.batt_is_full) {
			ret = POWER_SUPPLY_STATUS_FULL;
		} else {
			ret = POWER_SUPPLY_STATUS_CHARGING;
		}
		break;
	case CHARGER_DISCHARGE:
		ret = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		ret = POWER_SUPPLY_STATUS_UNKNOWN;
	}
	dev_dbg(dev, "%s : %s\n", __func__, status_text[ret]);

	return ret;
}

static int axp192_bat_get_property(struct power_supply *bat_ps, 
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	dev_dbg(bat_ps->dev, "%s : psp = %d\n", __func__, psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = axp192_bat_get_charging_status();
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = axp192_get_bat_health();
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = axp192_bat_info.present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = axp192_bat_info.bat_info.level;
		dev_dbg(dev, "%s : level = %d\n", __func__, 
				val->intval);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = axp192_bat_info.bat_info.batt_temp;
		dev_dbg(bat_ps->dev, "%s : temp = %d\n", __func__, 
				val->intval);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int axp192_power_get_property(struct power_supply *bat_ps, 
		enum power_supply_property psp, 
		union power_supply_propval *val)
{
	charger_type_t charger;
	
	dev_dbg(bat_ps->dev, "%s : psp = %d\n", __func__, psp);

	charger = axp192_bat_info.bat_info.charging_source;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (bat_ps->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (charger == CHARGER_AC ? 1 : 0);
		else if (bat_ps->type == POWER_SUPPLY_TYPE_USB)
			val->intval = (charger == CHARGER_USB ? 1 : 0);
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}
	
	return 0;
}

#define SEC_BATTERY_ATTR(_name)								\
{											\
	.attr = { .name = #_name, .mode = S_IRUGO | S_IWUGO, .owner = THIS_MODULE },	\
	.show = axp192_bat_show_property,							\
	.store = axp192_bat_store,								\
}

static struct device_attribute axp192_battery_attrs[] = {
	SEC_BATTERY_ATTR(batt_vol),
	SEC_BATTERY_ATTR(batt_vol_adc),
	SEC_BATTERY_ATTR(batt_vol_adc_cal),
	SEC_BATTERY_ATTR(batt_temp),
	SEC_BATTERY_ATTR(batt_temp_adc),
	SEC_BATTERY_ATTR(batt_temp_adc_cal),
	SEC_BATTERY_ATTR(batt_charging_cur),
	SEC_BATTERY_ATTR(batt_discharging_cur),
	SEC_BATTERY_ATTR(batt_current)
};

enum {
	BATT_VOL = 0,
	BATT_VOL_ADC,
	BATT_VOL_ADC_CAL,
	BATT_TEMP,
	BATT_TEMP_ADC,
	BATT_TEMP_ADC_CAL,
        BATT_CHARGING_CUR,
        BATT_DISCHARGING_CUR,
        BATT_CURRENT
};

static int axp192_bat_create_attrs(struct device * dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(axp192_battery_attrs); i++) {
		rc = device_create_file(dev, &axp192_battery_attrs[i]);
		if (rc)
		goto axp192_attrs_failed;
	}
	goto succeed;

axp192_attrs_failed:
	while (i--)
	device_remove_file(dev, &axp192_battery_attrs[i]);
succeed:
	return rc;
}

static ssize_t axp192_bat_show_property(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - axp192_battery_attrs;

	switch (off) {
	case BATT_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				       axp192_bat_info.bat_info.batt_vol);
	break;
	case BATT_VOL_ADC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				       axp192_bat_info.bat_info.batt_vol_adc);
		break;
	case BATT_VOL_ADC_CAL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				       axp192_bat_info.bat_info.batt_vol_adc_cal);
		break;
	case BATT_TEMP:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				       axp192_bat_info.bat_info.batt_temp);
		break;
	case BATT_TEMP_ADC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				       axp192_bat_info.bat_info.batt_temp_adc);
		break;	
	case BATT_TEMP_ADC_CAL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				       axp192_bat_info.bat_info.batt_temp_adc_cal);
		break;
	case BATT_CHARGING_CUR:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				       axp192_bat_info.bat_info.batt_charging_cur);
		break;
	case BATT_DISCHARGING_CUR:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				       axp192_bat_info.bat_info.batt_discharging_cur);
		break;
	case BATT_CURRENT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				       axp192_bat_info.bat_info.batt_current);
		break;
	default:
		i = -EINVAL;
	}       

	return i;
}

static ssize_t axp192_bat_store(struct device *dev, 
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int x = 0;
	int ret = 0;
	const ptrdiff_t off = attr - axp192_battery_attrs;

	switch (off) {
	case BATT_VOL_ADC_CAL:
		if (sscanf(buf, "%d\n", &x) == 1) {
			axp192_bat_info.bat_info.batt_vol_adc_cal = x;
			ret = count;
		}
		dev_info(dev, "%s : batt_vol_adc_cal = %d\n", __func__, x);
		break;
	case BATT_TEMP_ADC_CAL:
		if (sscanf(buf, "%d\n", &x) == 1) {
			axp192_bat_info.bat_info.batt_temp_adc_cal = x;
			ret = count;
		}
		dev_info(dev, "%s : batt_temp_adc_cal = %d\n", __func__, x);
		break;
	default:
		ret = -EINVAL;
	}       

	return ret;
}

static enum power_supply_property axp192_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property axp192_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};

static struct power_supply axp192_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = axp192_battery_properties,
		.num_properties = ARRAY_SIZE(axp192_battery_properties),
		.get_property = axp192_bat_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = axp192_power_properties,
		.num_properties = ARRAY_SIZE(axp192_power_properties),
		.get_property = axp192_power_get_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = axp192_power_properties,
		.num_properties = ARRAY_SIZE(axp192_power_properties),
		.get_property = axp192_power_get_property,
	},
};

static int axp192_cable_status_update(int status)
{
	int ret = 0;
	charger_type_t source = CHARGER_BATTERY;

	dev_dbg(dev, "%s\n", __func__);

	if(!axp192_battery_initial)
		return -EPERM;

	switch(status) {
	case CHARGER_BATTERY:
		dev_dbg(dev, "%s : cable NOT PRESENT\n", __func__);
		axp192_bat_info.bat_info.charging_source = CHARGER_BATTERY;
		break;
	case CHARGER_USB:
		dev_dbg(dev, "%s : cable USB\n", __func__);
		axp192_bat_info.bat_info.charging_source = CHARGER_USB;
		break;
	case CHARGER_AC:
		dev_dbg(dev, "%s : cable AC\n", __func__);
		axp192_bat_info.bat_info.charging_source = CHARGER_AC;
		break;
	case CHARGER_DISCHARGE:
		dev_dbg(dev, "%s : Discharge\n", __func__);
		axp192_bat_info.bat_info.charging_source = CHARGER_DISCHARGE;
		break;
	default:
		dev_err(dev, "%s : Nat supported status\n", __func__);
		ret = -EINVAL;
	}
	source = axp192_bat_info.bat_info.charging_source;
	/* if the power source changes, all power supplies may change state */
	power_supply_changed(&axp192_power_supplies[CHARGER_BATTERY]);
	/*
	power_supply_changed(&axp192_power_supplies[CHARGER_USB]);
	power_supply_changed(&axp192_power_supplies[CHARGER_AC]);
	*/
	dev_dbg(dev, "%s : call power_supply_changed\n", __func__);
	return ret;
}

void axp192_cable_check_status(int flag)
{
    charger_type_t status = 0;
    if (flag == CHARGER_BATTERY)  // Battery
		status = CHARGER_BATTERY;
    else if(flag == CHARGER_USB)   // USB
		status = CHARGER_USB;
	else if(flag == CHARGER_AC)
		status = CHARGER_AC;
	else status = CHARGER_DISCHARGE;
    axp192_cable_status_update(status);
}
EXPORT_SYMBOL(axp192_cable_check_status);

static void axp192_bat_status_update(struct power_supply *bat_ps)
{
	int old_level, old_temp, old_is_full ;
	dev_dbg(dev, "%s ++\n", __func__);

	if(!axp192_battery_initial)
		return;

	mutex_lock(&work_lock);
	old_temp = axp192_bat_info.bat_info.batt_temp;
	old_level = axp192_bat_info.bat_info.level; 
	old_is_full = axp192_bat_info.bat_info.batt_is_full;
	axp192_bat_info.bat_info.batt_temp = axp192_get_bat_temp(bat_ps);
	axp192_bat_info.bat_info.batt_vol = axp192_get_bat_vol(bat_ps);
	axp192_bat_info.bat_info.batt_charging_cur = axp192_battery_charging_cur();
	axp192_bat_info.bat_info.batt_discharging_cur = axp192_battery_discharging_cur();
	axp192_bat_info.bat_info.batt_current = 
             axp192_bat_info.bat_info.batt_discharging_cur -
	     axp192_bat_info.bat_info.batt_charging_cur;
	axp192_bat_info.bat_info.level = axp192_get_bat_level(bat_ps);
	if((axp192_battery_charging_status()==0) && axp192_is_battery_in() && axp192_battery_current_direction())
		axp192_bat_info.bat_info.batt_is_full = 1;
	else axp192_bat_info.bat_info.batt_is_full = 0;
	/*if (old_level != axp192_bat_info.bat_info.level 
			|| old_temp != axp192_bat_info.bat_info.batt_temp
			|| old_is_full != axp192_bat_info.bat_info.batt_is_full
			|| force_update) {
		force_update = 0;
		power_supply_changed(bat_ps);
		BATT("power supply changed\n",__func__);
		dev_dbg(dev, "%s : call power_supply_changed\n", __func__);
	}*/
	#if WAKE_LOCK
		if(axp192_usb_status() && (gpio_get_value(GPIO_P101) == 1))
				wake_lock(&vbus_wake_lock);
		else
			wake_lock_timeout(&vbus_wake_lock, 3*HZ);
	#endif
	if(axp192_ac_status() && axp192_is_battery_in() && (axp192_battery_charging_status() == 0))
		axp192_bat_info.bat_info.batt_is_full = 1;
	if(axp192_ac_status())
		axp192_cable_check_status(CHARGER_AC);
	else if(axp192_usb_status() && (gpio_get_value(GPIO_P101) == 1))//Display USB Charging
		axp192_cable_check_status(CHARGER_USB);
	else if((axp192_battery_current_direction() == 0) && axp192_is_battery_in())
		axp192_cable_check_status(CHARGER_DISCHARGE);
	mutex_unlock(&work_lock);
	dev_dbg(dev, "%s --\n", __func__);
}


void update_battery_info(void)
{
		axp192_bat_status_update(&axp192_power_supplies[CHARGER_BATTERY]);
}

static void battery_timer_func(void){
	queue_work(batt_wq,hardwork);
	mod_timer(&my_timer,jiffies + UPDATE_CYCLE*HZ);
}

#ifdef CONFIG_PM
static int axp192_bat_suspend(struct platform_device *pdev, 
		pm_message_t state)
{
	dev_info(dev, "%s\n", __func__);

	return 0;
}

static int axp192_bat_resume(struct platform_device *pdev)
{
	dev_info(dev, "%s\n", __func__);

	return 0;
}
#else
#define axp192_bat_suspend NULL
#define axp192_bat_resume NULL
#endif /* CONFIG_PM */

static int __devinit axp192_bat_probe(struct platform_device *pdev)
{
	int i;
	int ret = 0;

	dev = &pdev->dev;
	dev_info(dev, "%s\n", __func__);
	hardwork = kmalloc(sizeof(struct work_struct),GFP_KERNEL);
	if(hardwork == NULL)
		goto out;
	INIT_WORK(hardwork,axp192_bat_status_update);
	axp192_bat_info.present = 1;

	axp192_bat_info.bat_info.batt_id = 0;
	axp192_bat_info.bat_info.batt_vol = 0;
	axp192_bat_info.bat_info.batt_vol_adc = 0;
	axp192_bat_info.bat_info.batt_vol_adc_cal = 0;
	axp192_bat_info.bat_info.batt_temp = 0;
	axp192_bat_info.bat_info.batt_temp_adc = 0;
	axp192_bat_info.bat_info.batt_temp_adc_cal = 0;
	axp192_bat_info.bat_info.batt_current = 0;
	axp192_bat_info.bat_info.level = 0;
	axp192_bat_info.bat_info.charging_source = CHARGER_BATTERY;
	axp192_bat_info.bat_info.charging_enabled = 0;
	axp192_bat_info.bat_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;

	/* init power supplier framework */
	for (i = 0; i < ARRAY_SIZE(axp192_power_supplies); i++) {
		ret = power_supply_register(&pdev->dev, 
				&axp192_power_supplies[i]);
		if (ret) {
			dev_err(dev, "Failed to register"
					"power supply %d,%d\n", i, ret);
			goto __end__;
		}
	}
	printk("register power supply successful\n");
	/* create sec detail attributes */
	axp192_bat_create_attrs(axp192_power_supplies[CHARGER_BATTERY].dev);

	axp192_battery_initial = 1;
	force_update = 0;

	axp192_bat_status_update(
			&axp192_power_supplies[CHARGER_BATTERY]);
	init_timer(&my_timer);
	my_timer.expires = jiffies + UPDATE_CYCLE*HZ;
	my_timer.function = battery_timer_func;
	add_timer(&my_timer);
__end__:
	return ret;
out:
	kfree(hardwork);
	return -1;
}

static int __devexit axp192_bat_remove(struct platform_device *pdev)
{
	int i;
	dev_info(dev, "%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(axp192_power_supplies); i++) {
		power_supply_unregister(&axp192_power_supplies[i]);
	}
 	del_timer(&my_timer);
	return 0;
}

static struct platform_driver axp192_bat_driver = {
	.driver.name	= DRIVER_NAME,
	.driver.owner	= THIS_MODULE,
	.probe		= axp192_bat_probe,
	.remove		= __devexit_p(axp192_bat_remove),
	.suspend	= axp192_bat_suspend,
	.resume		= axp192_bat_resume,
};

/* Initailize GPIO */
static void axp192_bat_init_hw(void)
{
}

static int __init axp192_bat_init(void)
{
	pr_info("%s\n", __func__);

	axp192_bat_init_hw();
	batt_wq = create_singlethread_workqueue("battery");
	if(batt_wq == NULL)
		return -ENOMEM;
	#if WAKE_LOCK
	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
	#endif

	return platform_driver_register(&axp192_bat_driver);
}

static void __exit axp192_bat_exit(void)
{
	pr_info("%s\n", __func__);
	platform_driver_unregister(&axp192_bat_driver);
}

module_init(axp192_bat_init);
module_exit(axp192_bat_exit);

MODULE_AUTHOR("HuiSung Kang <hs1218.kang@samsung.com>");
MODULE_DESCRIPTION("S3C battery driver for SMDK Board");
MODULE_LICENSE("GPL");
