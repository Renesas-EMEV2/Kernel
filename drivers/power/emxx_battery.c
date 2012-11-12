/*
 *  File Name       : drivers/power/emxx_battery.c
 *  Function        : EMMA Mobile series dummy Battery
 *  Release Version : Ver 1.10
 *  Release Date    : 2010/09/10
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
 * Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA.
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/delay.h>
#ifdef CONFIG_EMXX_ANDROID
#include <linux/wakelock.h>
#endif
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <mach/charger.h>

#include <mach/pwc.h>
#include <asm/irq.h>
#include "emxx_battery.h"
#include <mach/axp192.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/smu.h>


/* Set ADC sample rate (30sec)*/
#define ADC_SAMPLE_RATE		30
#define BATTVOLSAMPLE 		10

#ifndef TRUE
#define TRUE    1
#define FALSE   0
#endif

/* module debugging
#define EMXX_BATTERY_DEBUG 1
*/
#ifdef EMXX_BATTERY_DEBUG
#define DEBUG_PRINT(FMT, ARGS...) \
		printk(KERN_INFO "%s(): " FMT, __func__ , ##ARGS)
#else
#define DEBUG_PRINT(FMT, ARGS...)
#endif

unsigned long   gIndex = 0,volIndex = 0;
unsigned long   gAvrVoltage[BATTVOLSAMPLE];


#define bprintk(x)  printk x


#define  BUFFER_LONG 10//1000

_drv_power_fuelgauge Fuelguage;

uint16 Bat_Rdc,Bat_Vol,Bat_Ocv_Vol;
uint16 bati = 0,batj=0,batk=0;
uint32 Total_Cap = 0,Total_Time = 0,Total_BatVol = 0;
uint8 Rdc_Flag = 0,Pre_Rdc_Flag = 0,Init_Flag = 0;
uint8    Bat_Cap_Buffer[BUFFER_LONG];

uint8 AverVoltageBuffer_Clrflg = 0;
	
uint8  flag_lastvol = 0;
uint8  Flg_otg_on = 0;


_drv_power_charge_stat Charge_Stat;
//static int g_usb_online;
//static int g_ac_plugin;
_drv_power_fuelgauge rt_Fuelguage,ocv_Fuelguage,cou_Fuelguage; //cou_Fuelguage


extern int axp192_setbattry_callback(void (*callback)(int));

uint8 Get_Bat_RestVol(uint16 Bat_Ocv_Vol) ;

static int initliased = 0;
enum charger_type_t {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
};
#define charger_type_t enum charger_type_t


struct emxx_battery_data {
	struct power_supply battery;
	struct power_supply usb;
	struct power_supply ac;

	charger_type_t charger;

	unsigned int battery_present;
//	unsigned int voltage_level;
	uint16 voltage_level;
	uint16 battery_temp;
	unsigned int battery_voltage;
//	long battery_voltage;

	unsigned int flag_adc_battery;
	int usb_state;
	spinlock_t lock;

#ifdef CONFIG_EMXX_DUMMPYBATTRY
	int lock_status;
	struct wake_lock vbus_suspend_lock;
#else
	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
#endif
};

static struct emxx_battery_data *battery_data;

static enum power_supply_property emxx_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,	
	POWER_SUPPLY_PROP_TEMP,			//add
	POWER_SUPPLY_PROP_VOLTAGE_NOW,	//add
};

static enum power_supply_property emxx_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

	
static unsigned int emxx_charger_get_status(void)
{
	unsigned char pwr_status = 0;
	unsigned char chg_status = 0;
	unsigned char vbus_status = 0;
	/* Charger status */
	axp192_read(AXP_PWR_STATE, &pwr_status);
	DEBUG_PRINT("Read AXP192 register AXP_PWR_STATE(%02xH)=0x%02x\n", AXP_PWR_STATE, pwr_status);

	axp192_read(AXP_VBUS_IPSOUT, &vbus_status);
	if(gpio_get_value(GPIO_P101))
	{
		if(!(vbus_status&0x80))
		{
			axp192_write(AXP_VBUS_IPSOUT,vbus_status|0x80);			
		}	
		DEBUG_PRINT("VBUS device\n");
	}
	else
	{		
		if(vbus_status&0x80)
		{
			axp192_write(AXP_VBUS_IPSOUT, vbus_status&0x7f);
		}
		pwr_status &= ~AXP_PWR_VBUS;
		Flg_otg_on = 1;
		DEBUG_PRINT("VBUS OTG\n");
	}			
	if (pwr_status & AXP_PWR_ACIN)
		battery_data->charger = CHARGER_AC;
	else if (pwr_status & AXP_PWR_VBUS)			
		battery_data->charger = CHARGER_USB;
	else
		battery_data->charger = CHARGER_BATTERY;

	if(chg_status & AXP_CHG_BAT )
		battery_data->battery_present = 1;
	else
		battery_data->battery_present = 0;
	return 0;
}

static int emxx_battery_get_status(void)
{
	int ret;
//	unsigned char status = 0;

#if 0
	axp192_read( AXP_PWR_STATE, &status);
	if(status & AXP_PWR_BAT_CUR)
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else
		ret = POWER_SUPPLY_STATUS_CHARGING;
#else

	switch (battery_data->charger) {
	case CHARGER_BATTERY:
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case CHARGER_USB:
	case CHARGER_AC:
		if (battery_data->voltage_level == 100)
			ret = POWER_SUPPLY_STATUS_FULL;
		else
			ret = POWER_SUPPLY_STATUS_CHARGING;
		break;
	default:
		ret = POWER_SUPPLY_STATUS_UNKNOWN;
	}
#endif	
	return ret;
}

static int emxx_battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:		/* 0 */
		val->intval = emxx_battery_get_status();
		break;
	case POWER_SUPPLY_PROP_HEALTH:		/* 1 */
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:		/* 2 */
		val->intval = battery_data->battery_present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:	/* 4 */
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:	/* 26 */
		val->intval = battery_data->voltage_level;
		break;	
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = battery_data->battery_temp;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:	
		val->intval = battery_data->battery_voltage;
		break;	
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

#ifndef CONFIG_EMXX_DUMMPYBATTRY
#if 0
static unsigned int emxx_charger_select_temperature(
	unsigned int bat_temperature)
{
//	unsigned int temp_temperature = 0;
//	unsigned int index = 0;

#if 0
	for (index = 0; index < (NUM_OF_LOOKUP_TABLE - 1); index++) {
		temp_temperature = (temperature_lookup_ref[index] +
					temperature_lookup_ref[index+1]) / 2;
		if (bat_temperature >= temp_temperature)
			continue;
		else
			break;
	}
	return index;
#endif
	return 25;
}
#endif

#if 0
static unsigned int emxx_charger_check_voltage(unsigned int access_index,
						unsigned int bat_voltage)
{
	unsigned int voltage_level = 0;
	unsigned int index = 0;

	for (index = 0; index < (LOOKUP_TABLE_SIZE - 1); index++) {
		if ((bat_voltage <=
			vbat_capacity_look_up[access_index][index][0]) &&
			(bat_voltage >
			vbat_capacity_look_up[access_index][index+1][0])) {
			voltage_level =
				vbat_capacity_look_up[access_index][index][1];
			break;
		}
	}
	if ((bat_voltage <=
		vbat_capacity_look_up[access_index][LOOKUP_TABLE_SIZE-1][0]))
		voltage_level =
		vbat_capacity_look_up[access_index][LOOKUP_TABLE_SIZE-1][1];

	if (bat_voltage > vbat_capacity_look_up[access_index][0][0])
		voltage_level = vbat_capacity_look_up[access_index][0][1];

	return voltage_level;
}
#endif

static unsigned int emxx_charger_update_status(void)
{
	charger_type_t old_charger = battery_data->charger;

	unsigned int status;
	status = emxx_charger_get_status();
	DEBUG_PRINT("old_charger:%d  cur_charger:%d",old_charger,battery_data->charger);	
	if (old_charger != battery_data->charger) {
		DEBUG_PRINT("power_supply_changed\n");		
		AverVoltageBuffer_Clrflg = 1;
		power_supply_changed(&battery_data->battery);
		power_supply_changed(&battery_data->usb);
		power_supply_changed(&battery_data->ac);
	}
	return status;
}

static irqreturn_t emxx_charger_irq(int irq, void *dev_id)
{
	unsigned int status;
	status = emxx_charger_update_status();
	return IRQ_HANDLED;
}

static void emxx_battry_status_changed(int event)
{
	if(initliased)
	{
		emxx_charger_irq(0, NULL);
	}
}
/**************************************************************************************************************/  //add by smallart



uint8 Get_Bat_RestVol(uint16 Bat_Ocv_Vol)
{
	if(Bat_Ocv_Vol >= FUELGUAGE_TOP_VOL)								//4160 //zdl
		return FUELGUAGE_TOP_LEVEL;
	else if(Bat_Ocv_Vol < FUELGUAGE_LOW_VOL)					//<3400
		return FUELGUAGE_LOW_LEVEL;
	else if(Bat_Ocv_Vol < FUELGUAGE_VOL1)							//3500
		return FUELGUAGE_LOW_LEVEL + (FUELGUAGE_LEVEL1 - FUELGUAGE_LOW_LEVEL) * (Bat_Ocv_Vol - FUELGUAGE_LOW_VOL) / (FUELGUAGE_VOL1 - FUELGUAGE_LOW_VOL);
	else if(Bat_Ocv_Vol < FUELGUAGE_VOL2)							//3600
		return FUELGUAGE_LEVEL1 + (FUELGUAGE_LEVEL2 - FUELGUAGE_LEVEL1) * (Bat_Ocv_Vol - FUELGUAGE_VOL1) / (FUELGUAGE_VOL2 - FUELGUAGE_VOL1);
	else if(Bat_Ocv_Vol < FUELGUAGE_VOL3)							//3700
		return FUELGUAGE_LEVEL2 + (FUELGUAGE_LEVEL3 - FUELGUAGE_LEVEL2) * (Bat_Ocv_Vol - FUELGUAGE_VOL2) / (FUELGUAGE_VOL3 - FUELGUAGE_VOL2);
	else if(Bat_Ocv_Vol < FUELGUAGE_VOL4)							//3800
		return FUELGUAGE_LEVEL3 + (FUELGUAGE_LEVEL4 - FUELGUAGE_LEVEL3) * (Bat_Ocv_Vol - FUELGUAGE_VOL3) / (FUELGUAGE_VOL4 - FUELGUAGE_VOL3);
	else if(Bat_Ocv_Vol < FUELGUAGE_VOL5)							//3900
		return FUELGUAGE_LEVEL4 + (FUELGUAGE_LEVEL5 - FUELGUAGE_LEVEL4) * (Bat_Ocv_Vol - FUELGUAGE_VOL4) / (FUELGUAGE_VOL5 - FUELGUAGE_VOL4);
	else if(Bat_Ocv_Vol < FUELGUAGE_VOL6)							//4000
		return FUELGUAGE_LEVEL5 + (FUELGUAGE_LEVEL6 - FUELGUAGE_LEVEL5) * (Bat_Ocv_Vol - FUELGUAGE_VOL5) / (FUELGUAGE_VOL6 - FUELGUAGE_VOL5);
	else if(Bat_Ocv_Vol < FUELGUAGE_VOL7)							//4100
		return FUELGUAGE_LEVEL6 + (FUELGUAGE_LEVEL7 - FUELGUAGE_LEVEL6) * (Bat_Ocv_Vol - FUELGUAGE_VOL6) / (FUELGUAGE_VOL7 - FUELGUAGE_VOL6);	
	else if(Bat_Ocv_Vol <FUELGUAGE_TOP_VOL)							//4100
		return FUELGUAGE_LEVEL7 + (FUELGUAGE_TOP_LEVEL - FUELGUAGE_LEVEL7) * (Bat_Ocv_Vol - FUELGUAGE_VOL7) / (FUELGUAGE_TOP_VOL - FUELGUAGE_VOL7);	
	else
		return 0;
}

void ADC_Enable_Set(uint16 Enable_Bit)
{			//Enable_Bit :Reg83h << 8 + Reg82h
	uint8  temp;
	temp = Enable_Bit & 0x00ff;
	axp192_write(POWER_ADC_EN1,temp);		
	temp = (Enable_Bit & 0xff00) >> 8;
	axp192_write(POWER_ADC_EN2,temp);		
}


void ADC_Range_Freq_Set(uint8 Adc_Freq,uint16 Adc_Type,uint8 Range)
{			//Enable_Bit :Reg83h << 8 + Reg82h
	uint8  temp,tmp;
	if(Adc_Freq != 0)			//Level1 2 3 4,Level0 not change,Range :Level1/2
	{
		temp = axp192_read(POWER_ADC_SPEED,&temp);
		temp &= 0x3f;
		temp |= (Adc_Freq - 1) << 6;
		axp192_write(POWER_ADC_SPEED,temp);
		axp192_read(POWER_ADC_SPEED,&tmp);
	}
	if(Adc_Type != 0)			//Adc_Type == 0,not change
	{
		temp = axp192_read(POWER_ADC_INPUTRANGE,&temp);	
		switch(Adc_Type)
		{
			case ADC_BATSENSE:
				temp &= 0xf7;
				temp |= (Range - 1) << 3;
				break;
			case ADC_GPIO2:
				temp &= 0xfb;
				temp |= (Range - 1) << 2;
				break;
			case ADC_GPIO1:
				temp &= 0xfd;
				temp |= (Range - 1) << 1;
				break;
			case ADC_GPIO0:
				temp &= 0xfe;
				temp |= (Range - 1) << 0;
				break;												
			default:
				break;
		}
		axp192_write(POWER_ADC_INPUTRANGE,temp);		
	}
}

uint16 Get_Buffer_Rdc(void)
{
	uint8 temp[2];
	axp192_read(POWER_DATA_BUFFER3,&temp[0]);	
	axp192_read(POWER_DATA_BUFFER4,&temp[1]);
	return (((temp[0] << 8) + temp[1]) & 0x0fff);
}


uint8 ADC_EnableBATDet(void)
{
	uint8  temp;
	axp192_read(POWER_ADC_EN1,&temp);
	if(!(temp & 0x80)){
		temp &= 0x80;
		axp192_write(POWER_ADC_EN1,temp);
	}
	return temp;
}

uint8 ADC_Range_Get(uint16 Adc_Type)
{			//Enable_Bit:Reg83h << 8 + Reg82h
	uint8  temp =0 ,rValue = 0;
	temp = axp192_read(POWER_ADC_INPUTRANGE,&temp);

	switch(Adc_Type)
	{
		case ADC_BATSENSE:
			temp &= 0x08;
			rValue = temp >> 3;
			break;
		case ADC_GPIO2:
			temp &= 0x04;
			rValue = temp >> 2;
			break;
		case ADC_GPIO1:
			temp &= 0x02;
			rValue = temp >> 1;
			break;
		case ADC_GPIO0:
			temp &= 0x01;
			rValue = temp >> 0;
			break;												
		default:
			break;
	}	
	return rValue;
}

uint16 Get_Single_Adc_Data(uint16 Adc_Type)
{
	 //modify by ck for  floating to fixed-point operation
	uint8  temp[2];
	uint32  rValue = 0;
	switch(Adc_Type)
	{
		case ACIN_VOL:
			axp192_read(POWER_ACIN_VOL_H8,&temp[0]);//zdl
			axp192_read(POWER_ACIN_VOL_L4,&temp[1]);
			//Sys_I2CRead(POWER_BAT_AVERVOL_H8,temp,2);
			//rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 1.7;
#if 1
			rValue = ((temp[0] << 4) + (temp[1] & 0x0f));
			rValue += rValue/10;
			DEBUG_PRINT("get battery adc rValue:%d\n", rValue);
#else
			rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 17 / 10;
#endif
			break;
		case ACIN_CUR:
			axp192_read(POWER_ACIN_CUR_H8,&temp[0]);
			axp192_read(POWER_ACIN_CUR_L4,&temp[1]);			
			//rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 0.625;
			rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 625 / 1000;
			break;	
		case VBUS_VOL:
#if 0
			axp192_read(POWER_ACIN_CUR_H8,&temp[0]);
			axp192_read(POWER_ACIN_CUR_L4,&temp[1]);
#else				
			axp192_read(POWER_VBUS_VOL_H8,&temp[0]);
			axp192_read(POWER_VBUS_VOL_L4,&temp[1]);
#endif
			//rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 1.7;
			rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 17 / 10;
			break;
		case VBUS_CUR:
			axp192_read(POWER_ACIN_CUR_H8,&temp[0]);
			axp192_read(POWER_ACIN_CUR_L4,&temp[1]);
			//rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 0.375;
			rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 375 / 1000;
			break;	
		case INT_TEMP:
			axp192_read(POWER_INT_TEMP_H8,&temp[0]);
			axp192_read(POWER_INT_TEMP_L4,&temp[1]);
//			rValue = (((temp[0] << 4) + (temp[1] & 0x0f))  -1447 ) /10 ;
			rValue = (((temp[0] << 4) + (temp[1] & 0x0f))  -1500 ) /10 ;			
			break;
		case TS_VOL:
			axp192_read(POWER_TS_VOL_H8,&temp[0]);
			axp192_read(POWER_TS_VOL_L4,&temp[1]);
			//rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 0.8;
			rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 8 / 10;
			break;	
		case GPIO0_VOL:
			axp192_read(POWER_GPIO0_VOL_H8,&temp[0]);
			axp192_read(POWER_GPIO0_VOL_L4,&temp[1]);
			//rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 0.5;
			rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 5 / 10;
			
			if(ADC_Range_Get(ADC_GPIO0))
				rValue += 700;
			break;
		case GPIO1_VOL:
			axp192_read(POWER_GPIO1_VOL_H8,&temp[0]);
			axp192_read(POWER_GPIO1_VOL_L4,&temp[1]);
			//rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 0.5;
			rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 5 / 10;
			if(ADC_Range_Get(ADC_GPIO1))
				rValue += 700;			
			break;
		case GPIO2_VOL:
			axp192_read(POWER_GPIO2_VOL_H8,&temp[0]);
			axp192_read(POWER_GPIO2_VOL_L4,&temp[0]);
			//rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 0.5;
			rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 5 / 10;
			if(ADC_Range_Get(ADC_GPIO2))
				rValue += 700;			
			break;	
		case BATSENSE_VOL:
			axp192_read(POWER_BATSENSE_VOL_H8,&temp[0]);
			axp192_read(POWER_BATSENSE_VOL_L4,&temp[1]);
			//rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 0.5;
			rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 5 / 10;
			if(ADC_Range_Get(ADC_BATSENSE))
				rValue += 700;			
			break;	
		case APS_VOL:
			axp192_read(POWER_APS_AVERVOL_H8,&temp[0]);
			axp192_read(POWER_APS_AVERVOL_L4,&temp[1]);
			//rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 1.4;
			rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 14 / 10;
			break;	
		case BAT_VOL:
			axp192_read(POWER_BAT_AVERVOL_H8,&temp[0]);
			axp192_read(POWER_BAT_AVERVOL_L4,&temp[1]);
			//rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 1.1;
			rValue = ((temp[0] << 4) + (temp[1] & 0x0f)) * 11 / 10;
			break;	
		case BAT_CUR:
			axp192_read(POWER_BAT_AVERCHGCUR_H8,&temp[0]);
			axp192_read(POWER_BAT_AVERCHGCUR_L5,&temp[1]);
			//rValue = ((temp[0] << 5) + (temp[1] & 0x1f)) * 0.5;
			rValue = ((temp[0] << 5) + (temp[1] & 0x1f)) * 5 / 10;
			axp192_read(POWER_BAT_AVERDISCHGCUR_H8,&temp[0]);
			axp192_read(POWER_BAT_AVERDISCHGCUR_L5,&temp[1]);
			
			DEBUG_PRINT("AverchgCur + AverdischgCur = %d - %d = %d\n",
                                     rValue,((temp[0] << 5) + (temp[1] & 0x1f)) * 5 / 10,
				     rValue+(((temp[0] << 5) + (temp[1] & 0x1f)) * 5 / 10));

			//rValue += ((temp[0] << 5) + (temp[1] & 0x1f)) * 0.5;	
			rValue += ((temp[0] << 5) + (temp[1] & 0x1f)) * 5 / 10;
			
			break;	
		case POWER_INPUT_STATUS:
			axp192_read(POWER_STATUS,temp);
			rValue = temp[0];
			break;

		case POWER_CHARGEING_MODE:
			axp192_read(POWER_MODE_CHGSTATUS,temp);
			rValue = temp[0];
			break;
		default:
			break;
	}
	return (rValue & 0xffff);
}

void Get_Charge_Status(_drv_power_charge_stat* Charge_Stat)
{
	uint8  temp[2];
	axp192_read(POWER_STATUS,&temp[0]);
	axp192_read(POWER_MODE_CHGSTATUS,&temp[1]);
	
	Charge_Stat->acin_valid = (temp[0] & 0x40) >> 6;
	Charge_Stat->vbus_valid = (temp[0] & 0x10) >> 4;
	Charge_Stat->ext_power_valid = Charge_Stat->acin_valid | Charge_Stat->vbus_valid;
	Charge_Stat->bat_current_direction = (temp[0] & 0x04) >> 2;
	Charge_Stat->in_Short = (temp[0] & 0x02) >> 1;
	Charge_Stat->int_over_temp = (temp[1] & 0x80) >> 7;
	Charge_Stat->charge_indication = (temp[1] & 0x40) >> 6;
	Charge_Stat->battery_exist = (temp[1] & 0x20) >> 5;
	Charge_Stat->battery_active = (temp[1] & 0x08) >> 3;
	Charge_Stat->low_charge_current = (temp[1] & 0x04) >> 2;	
	Charge_Stat->battery_current = Get_Single_Adc_Data(BAT_CUR);

	DEBUG_PRINT("POWER_STATUS:%x   POWER_MODE_CHGSTATUS:%x\n",temp[0],temp[1]);
}

void Get_Charge_Para(_drv_power_charge_para* Charge_Para)
{
	uint8  temp[2];
	axp192_read(POWER_CHARGE1,&temp[0]);
	axp192_read(POWER_CHARGE2,&temp[1]);

	if(temp[0] & 0x80)
	{  
		if(temp[1] & 0x04)
			Charge_Para->charge_onoff = CHARGE_ON | EXT_CHARGE_ON;
		else
			Charge_Para->charge_onoff = CHARGE_ON | EXT_CHARGE_OFF;
	}
	else
		Charge_Para->charge_onoff = CHARGE_OFF;
		
	switch((temp[0] & 0x60) >> 5)	
	{
		case 0:
			Charge_Para->target_vol = 4100;
			break;
		case 1:
			Charge_Para->target_vol = 4150;
			break;
		case 2:
			Charge_Para->target_vol = 4200;
			break;
		case 3:
			Charge_Para->target_vol = 4360;
			break;									
		default:
			break;
	}
	
	if(temp[0] & 0x10)
		Charge_Para->end_charge_rate = 15;
	else
		Charge_Para->end_charge_rate = 10;
		
	switch(temp[0] & 0x0f)
	{
		case 0:
			Charge_Para->int_cur_level = 100;
			break;
		case 1:
			Charge_Para->int_cur_level = 190;
			break;
		case 2:
			Charge_Para->int_cur_level = 280;
			break;
		case 3:
			Charge_Para->int_cur_level = 360;
			break;
		case 4:
			Charge_Para->int_cur_level = 450;
			break;
		case 5:
			Charge_Para->int_cur_level = 550;
			break;
		case 6:
			Charge_Para->int_cur_level = 630;
			break;
		case 7:
			Charge_Para->int_cur_level = 700;
			break;
		case 8:
			Charge_Para->int_cur_level = 780;
			break;
		case 9:
			Charge_Para->int_cur_level = 880;
			break;
		case 10:
			Charge_Para->int_cur_level = 960;
			break;
		case 11:
			Charge_Para->int_cur_level = 1000;
			break;
		case 12:
			Charge_Para->int_cur_level = 1080;
			break;
		case 13:
			Charge_Para->int_cur_level = 1160;
			break;
		case 14:
			Charge_Para->int_cur_level = 1240;
			break;
		case 15:
			Charge_Para->int_cur_level = 1320;
			break;									
		default:
			break;
	}
	switch((temp[1] & 0x38) >> 3)	
	{
		case 0:
			Charge_Para->ext_cur_level = 100;
			break;
		case 1:
			Charge_Para->ext_cur_level = 200;
			break;
		case 2:
			Charge_Para->ext_cur_level = 300;
			break;
		case 3:
			Charge_Para->ext_cur_level = 400;
			break;
		case 4:
			Charge_Para->ext_cur_level = 500;
			break;
		case 5:
			Charge_Para->ext_cur_level = 600;
			break;
		case 6:
			Charge_Para->ext_cur_level = 700;
			break;
		case 7:
			Charge_Para->ext_cur_level = 800;
			break;																					
		default:
			break;
	}
	switch((temp[1] & 0xc0) >> 6)
	{
		case 0:
			Charge_Para->prechg_time_level = LEVEL1;
			break;	
		case 1:
			Charge_Para->prechg_time_level = LEVEL2;
			break;	
		case 2:
			Charge_Para->prechg_time_level = LEVEL3;
			break;	
		case 3:
			Charge_Para->prechg_time_level = LEVEL4;
			break;																														
		default:
			break;			
	}
	switch((temp[1] & 0x03) >> 0)
	{
		case 0:
			Charge_Para->cchg_time_level = LEVEL1;
			break;	
		case 1:
			Charge_Para->cchg_time_level = LEVEL2;
			break;	
		case 2:
			Charge_Para->cchg_time_level = LEVEL3;
			break;	
		case 3:
			Charge_Para->cchg_time_level = LEVEL4;
			break;																														
		default:
			break;			
	}	
}

void Charge_Para_Set(_drv_power_charge_para* Charge_Para)	
{					//Charge_OnOff:CHARGE_ON,CHARGE_OFF,EXT_CHARGE_ON,EXT_CHARGE_OFF;
	uint8  temp[2]={0,0};
	
	if(Charge_Para->charge_onoff)
	{
		switch(Charge_Para->charge_onoff)
		{
			case CHARGE_ON:
				temp[0] |= 1 << 7;
				break;
			case CHARGE_OFF:
				temp[0] &= 0x7f;
				break;
			case EXT_CHARGE_ON:
				temp[1] |= 1 << 2;
				break;
			case EXT_CHARGE_OFF:
				temp[1] &= 0xfb;
				break;									
			default:
				break;
		}
	}
	if(Charge_Para->target_vol)
	{
		switch(Charge_Para->target_vol)
		{
			case 4100:
				temp[0] &= 0x9f;
				temp[0] |= 0 << 5;
				break;
			case 4150:
				temp[0] &= 0x9f;
				temp[0] |= 1 << 5;
				break;
			case 4200:
				temp[0] &= 0x9f;
				temp[0] |= 2 << 5;
				break;
			case 4360:
				temp[0] &= 0x9f;
				temp[0] |= 3 << 5;
				break;									
			default:
				break;
		}
	}
	if(Charge_Para->end_charge_rate)
	{
		switch(Charge_Para->end_charge_rate)							//10%,15%
		{
			case 10:
				temp[0] &= 0xef;
				temp[0] |= 0 << 4;
				break;
			case 15:
				temp[0] &= 0xef;
				temp[0] |= 1 << 4;
				break;							
			default:
				break;
		}	
	}
//Int/Ext Charge Current Set
	if(Charge_Para->int_cur_level)	
	{	
		switch(Charge_Para->int_cur_level)
		{
			case 100:
				temp[0] &= 0xf0;
				temp[0] |= 0x00;
				break;
			case 190:
				temp[0] &= 0xf0;
				temp[0] |= 0x01;
				break;
			case 280:
				temp[0] &= 0xf0;
				temp[0] |= 0x02;
				break;
			case 360:
				temp[0] &= 0xf0;
				temp[0] |= 0x03;
				break;
			case 450:
				temp[0] &= 0xf0;
				temp[0] |= 0x04;
				break;
			case 550:
				temp[0] &= 0xf0;
				temp[0] |= 0x05;
				break;
			case 630:
				temp[0] &= 0xf0;
				temp[0] |= 0x06;
				break;
			case 700:
				temp[0] &= 0xf0;
				temp[0] |= 0x07;
				break;
			case 780:
				temp[0] &= 0xf0;
				temp[0] |= 0x08;
				break;
			case 880:
				temp[0] &= 0xf0;
				temp[0] |= 0x09;
				break;
			case 960:
				temp[0] &= 0xf0;
				temp[0] |= 0x0a;
				break;
			case 1000:
				temp[0] &= 0xf0;
				temp[0] |= 0x0b;
				break;
			case 1080:
				temp[0] &= 0xf0;
				temp[0] |= 0x0c;
				break;
			case 1160:
				temp[0] &= 0xf0;
				temp[0] |= 0x0d;
				break;
			case 1240:
				temp[0] &= 0xf0;
				temp[0] |= 0x0e;
				break;
			case 1320:
				temp[0] &= 0xf0;
				temp[0] |= 0x0f;
				break;
			default:
				break;
		}
	}
	if(Charge_Para->ext_cur_level)
	{
		temp[1] &= 0xc7;
		temp[1] |= (Charge_Para->ext_cur_level / 100 - 1) << 3;
	}
//Timer Timeout Set	
	if(Charge_Para->prechg_time_level)
	{
		temp[1] &= 0x3f;
		temp[1] |= (Charge_Para->prechg_time_level - 1) << 6;
	}
	if(Charge_Para->cchg_time_level)
	{
		temp[1] &= 0xfc;
		temp[1] |= (Charge_Para->cchg_time_level - 1) << 0;
	}
	axp192_write(POWER_CHARGE1,temp[0]);	
	axp192_write(POWER_CHARGE2,temp[1]);
}


uint16 Get_Bat_Rdc(void)
{
	uint8 temp[6],i;
	uint16 Chg_BatVol[RDC_COUNT],Chg_BatCur[RDC_COUNT],Dischg_BatVol[RDC_COUNT],Dischg_BatCur[RDC_COUNT];
	uint16 Aver_Chg_BatVol = 0,Aver_Chg_BatCur = 0,Aver_Dischg_BatVol = 0,Aver_Dischg_BatCur = 0;
	_drv_power_charge_para Charge_Para;
	_drv_power_charge_stat Charge_Stat;

	msleep_interruptible(2000);
	Get_Charge_Status(&Charge_Stat);
	
	DEBUG_PRINT("RDC_DETECT START\n");
	for(i = 0;i < RDC_COUNT;i++)
	{
		axp192_read(POWER_BAT_AVERVOL_H8,&temp[0]);
		axp192_read(POWER_BAT_AVERVOL_L4,&temp[1]);
		
		axp192_read(POWER_BAT_AVERCHGCUR_H8,&temp[2]);
		axp192_read(POWER_BAT_AVERCHGCUR_L5,&temp[3]);
		
		axp192_read(POWER_BAT_AVERDISCHGCUR_H8,&temp[4]);
		axp192_read(POWER_BAT_AVERDISCHGCUR_L5,&temp[5]);
		
		Chg_BatVol[i] = ((temp[0] << 4) + (temp[1] & 0x0f)) * 11 / 10;		
		Chg_BatCur[i] = ((temp[2] << 5) + (temp[3] & 0x1f))*5/10;
		Aver_Chg_BatVol += Chg_BatVol[i];
		Aver_Chg_BatCur += Chg_BatCur[i];
		DEBUG_PRINT("chg_BatVol[%d]=%d,chg_BatCur[%d]=%d\n",i,Chg_BatVol[i],i,Chg_BatCur[i]);//
		msleep_interruptible(200);					//Delay 50mS
	}
	Aver_Chg_BatVol /= RDC_COUNT;
	Aver_Chg_BatCur /= RDC_COUNT;
	DEBUG_PRINT("Aver_Chg_BatVol:%d,Aver_Chg_BatCur:%d\n",Aver_Chg_BatVol,Aver_Chg_BatCur);
	Get_Charge_Para(&Charge_Para);
	Charge_Para.charge_onoff = CHARGE_OFF;
	Charge_Para_Set(&Charge_Para);
	msleep_interruptible(3000);
	DEBUG_PRINT("Charger_off\n");

	for(i = 0;i < RDC_COUNT;i++)
	{
		axp192_read(POWER_BAT_AVERVOL_H8,&temp[0]);
		axp192_read(POWER_BAT_AVERVOL_L4,&temp[1]);
		
		axp192_read(POWER_BAT_AVERCHGCUR_H8,&temp[2]);
		axp192_read(POWER_BAT_AVERCHGCUR_L5,&temp[3]);
		
		axp192_read(POWER_BAT_AVERDISCHGCUR_H8,&temp[4]);
		axp192_read(POWER_BAT_AVERDISCHGCUR_L5,&temp[5]);

		Dischg_BatVol[i] = ((temp[0] << 4) + (temp[1] & 0x0f)) * 11 / 10;		
		Dischg_BatCur[i] = ((temp[4] << 5) + (temp[5] & 0x1f))*5/10;
		Aver_Dischg_BatVol += Dischg_BatVol[i];
		Aver_Dischg_BatCur += Dischg_BatCur[i];
		DEBUG_PRINT("dischg_BatVol[%d]=%d,dischg_BatCur[%d]=%d\n",i,Dischg_BatVol[i],i,Dischg_BatCur[i]);
		msleep_interruptible(200);				//Delay 100mS
	}
	Aver_Dischg_BatVol /= RDC_COUNT;
	Aver_Dischg_BatCur /= RDC_COUNT;
	DEBUG_PRINT("Aver_disChg_BatVol:%d,Aver_disChg_BatCur:%d\n",Aver_Dischg_BatVol,Aver_Dischg_BatCur);
	Get_Charge_Para(&Charge_Para);
	Charge_Para.charge_onoff = CHARGE_ON;
	
	DEBUG_PRINT("Charge_stat:acin_valid=%d vbus_valid=%d charge_indication=%d\n",
		Charge_Stat.acin_valid,Charge_Stat.vbus_valid,Charge_Stat.charge_indication);

	Charge_Para_Set(&Charge_Para);
	msleep_interruptible(1000);				//Delay 100mS
	DEBUG_PRINT("Charger_on again\n");
	
	Get_Charge_Status(&Charge_Stat);

	DEBUG_PRINT("Charge_stat:acin_valid=%d vbus_valid=%d charge_indication=%d\n",
		Charge_Stat.acin_valid,Charge_Stat.vbus_valid,Charge_Stat.charge_indication);
	if(!Charge_Stat.ext_power_valid){
		if(Get_Buffer_Rdc() & 0x800)
			return ((Get_Buffer_Rdc()&0x7ff)*3);
		else
			return RDC_DEFAULT;
	}
		
	if(Charge_Stat.ext_power_valid && Charge_Stat.charge_indication) 
	{
		if(ABS(Aver_Chg_BatCur - Aver_Dischg_BatCur) < 300)
		{
			if(Get_Buffer_Rdc() & 0x800)
				return ((Get_Buffer_Rdc()&0x7ff)*3);
			else
				return RDC_DEFAULT;
		}
		else
		{
			DEBUG_PRINT("charge read:RDC = %d\n",
					1000 * ABS(Aver_Chg_BatVol - Aver_Dischg_BatVol) 
					/ ABS(Aver_Chg_BatCur - Aver_Dischg_BatCur));
			return 1000 * ABS(Aver_Chg_BatVol - Aver_Dischg_BatVol) / ABS(Aver_Chg_BatCur - Aver_Dischg_BatCur);
		}
	}	
	else
	{
		if(Get_Buffer_Rdc() & 0x800)
			return ((Get_Buffer_Rdc()&0x7ff)*3);
		else
			return RDC_DEFAULT;
	}
}

void Buffer_Rdc_Set(uint16 Rdc)			//Rdc save in Buffer 3/4,Buffer3 \B5\CD4bit,Buffer4 ȫ8bit
{
	uint8 temp[2];
	axp192_read(POWER_DATA_BUFFER3,&temp[0]);
	axp192_read(POWER_DATA_BUFFER4,&temp[1]);
	temp[0] |= ((Rdc & 0x0f00) >> 8);
	temp[1] = Rdc & 0x00ff;
	axp192_write(POWER_DATA_BUFFER3,temp[0]);
	axp192_write(POWER_DATA_BUFFER4,temp[1]);
}


uint16 Get_Bat_OcvVol(uint8 Flag,uint16 Bat_Vol,uint16 Bat_Cur,uint16 Rdc)
{
	if(Flag)	//charge status
	{
		DEBUG_PRINT("charge status:Bat_Vol-(Bat_Cur * Rdc + 500) / 1000 = %d - (%d * %d + 500)/1000 = %d\n"
			, Bat_Vol,Bat_Cur,Rdc,Bat_Vol - (Bat_Cur * Rdc + 500) / 1000);
		return Bat_Vol - (Bat_Cur * Rdc + 500) / 1000; 
	}
	else
	{
		DEBUG_PRINT("discharge status:Bat_Vol+(Bat_Cur * Rdc + 500) / 1000 = %d + (%d * %d + 500)/1000 = %d\n"
			, Bat_Vol,Bat_Cur,Rdc,Bat_Vol + (Bat_Cur * Rdc + 500)/ 1000);		
		return Bat_Vol + (Bat_Cur * Rdc + 500) / 1000;
	}
}


void Set_Rest_Cap(uint8 rest_cap)
{
	uint8 temp = 0x00;
	axp192_read(POWER_DATA_BUFFER1, &temp);
	temp &= 0x80;
	if(rest_cap>100) rest_cap=100;
	temp |= rest_cap;
	axp192_write(POWER_DATA_BUFFER1,temp);
}



void Cou_Count_Clear(void)
{
	uint8 temp = 0xff;
	axp192_read(POWER_COULOMB_CTL,&temp);
//	temp &= 0x7f;
	temp |= 0x20;
	temp &= 0xbf;
	axp192_write(POWER_COULOMB_CTL,temp);
	temp |= 0x80;
	temp &= 0xbf;
	axp192_write(POWER_COULOMB_CTL,temp);
}

uint8 ADC_Freq_Get(void)
{			//Enable_Bit :Reg83h << 8 + Reg82h
	uint8  temp;
	uint8  rValue = 25;
	//temp = axp192_read(POWER_ADC_SPEED,&temp);
	axp192_read(POWER_ADC_SPEED,&temp);
	temp &= 0xc0;
	switch(temp >> 6)
	{
		case 0:
			rValue = 25;
			break;
		case 1:
			rValue = 50;
			break;
		case 2:
			rValue = 100;
			break;
		case 3:
			rValue = 200;
			break;
		default:
			break;
	}
	return rValue;
}


long  Get_Bat_Coulomb_Count(uint32 bat_cap)
{
	uint8  temp[8];
	uint8  type;
	uint8  read_saved_cap;
	int32  rValue1,rValue2;
	long   Cur_CoulombCounter_tmp;

//^0^Chris add,
	axp192_read(POWER_BAT_CHGCOULOMB3,&temp[0]);
	axp192_read(POWER_BAT_CHGCOULOMB2,&temp[1]);
	axp192_read(POWER_BAT_CHGCOULOMB1,&temp[2]);
	axp192_read(POWER_BAT_CHGCOULOMB0,&temp[3]);
	axp192_read(POWER_BAT_DISCHGCOULOMB3,&temp[4]);
	axp192_read(POWER_BAT_DISCHGCOULOMB2,&temp[5]);
	axp192_read(POWER_BAT_DISCHGCOULOMB1,&temp[6]);
	axp192_read(POWER_BAT_DISCHGCOULOMB0,&temp[7]);

	//i2c_smbus_read_i2c_block_data(axp192_i2c_client, POWER_BAT_CHGCOULOMB3,8,temp);
	rValue1 = ((temp[0] << 24) + (temp[1] << 16) + (temp[2] << 8) + temp[3]);
	rValue2 = ((temp[4] << 24) + (temp[5] << 16) + (temp[6] << 8) + temp[7]);
	
	DEBUG_PRINT("Get_Bat_Coulomb_Count -     CHARGINGOULB:[0]=0x%x,[1]=0x%x,[2]=0x%x,[3]=0x%x\n",temp[0],temp[1],temp[2],temp[3]);
	DEBUG_PRINT("Get_Bat_Coulomb_Count - DISCHARGINGCLOUB:[4]=0x%x,[5]=0x%x,[6]=0x%x,[7]=0x%x\n",temp[4],temp[5],temp[6],temp[7]);

	axp192_read(0x03,&type);
	if((type&0x0f) == 0x03)
	{
		Cur_CoulombCounter_tmp = (65535 * (rValue1 - rValue2) / ADC_Freq_Get() / 3600 / 2);
		DEBUG_PRINT("0x03: 1628\n");
	}
	else
	{
		DEBUG_PRINT("POWER_BAT_CHGCOULOMB:%ld  POWER_BAT_DISCHGCOULOMB:%ld ADC_Freq:%d\n",rValue1,rValue2,ADC_Freq_Get());
		Cur_CoulombCounter_tmp =  (rValue1 - rValue2) / ADC_Freq_Get() / 3600 / 2;
		DEBUG_PRINT("0x03: 1612\n");
	}
	DEBUG_PRINT("Get_Bat_Coulomb_Count - (65535 * (rValue1 - rValue2) / ADC_Freq_Get() / 3600 / 2) = %ld\n",Cur_CoulombCounter_tmp);

	axp192_read(POWER_DATA_BUFFER1,&read_saved_cap);
	DEBUG_PRINT("Get_Bat_Coulomb_Count - read_saved_cap = %d\n",read_saved_cap);

////Please do not use global varable here!
//	Cur_CoulombCounter_tmp += read_saved_cap * Fuelguage.bat_cap / 100;
	Cur_CoulombCounter_tmp += (read_saved_cap & 0x7f) * bat_cap / 100;


	Cur_CoulombCounter_tmp += 1;	//Compensation for the calculation loss ,501 * 0.99 = 495.99 = 495

	axp192_read(0x33,&temp[0]);
	axp192_read(0x34,&temp[1]);
	DEBUG_PRINT("0x33 = 0x%x 0x34 = 0x%x\n",temp[0],temp[1]);
	DEBUG_PRINT("Get_Bat_Coulomb_Count - (Cur_CoulombCounter_tmp += read_saved_cap * Fuelguage.bat_cap / 100) = %d\n",Cur_CoulombCounter_tmp);
	DEBUG_PRINT("(65535 * (rValue1 - rValue2) / ADC_Freq_Get() / 3600 / 2) = %d (mAh)\n",Cur_CoulombCounter_tmp);
	return Cur_CoulombCounter_tmp;				//unit mAh	
}

uint8 Get_Lastvol_Flag(void)
{
	uint8 temp;
	axp192_read(POWER_DATA_BUFFER1,&temp);
	return ((temp&0x80) >> 7);
}

void Set_Flag_Lastvol(void)
{
	uint8 temp;
	axp192_read(POWER_DATA_BUFFER1,&temp);
	temp &= 0x7f;
	axp192_write(POWER_DATA_BUFFER1,temp);
}


uint16 Get_Buffer_Cou(void)
{
	uint8 temp[2];
	uint16 rValue;
	axp192_read(POWER_DATA_BUFFER2,&temp[0]);
	axp192_read(POWER_DATA_BUFFER3,&temp[1]);
	rValue = ((temp[0] << 4) + ((temp[1] & 0xf0) >> 4));
	if(rValue & 0x800)
		return ((rValue & 0x7ff) * 5);
	else
		return 0;
}

void Buffer_Cou_Set(uint16 Cou_Counter)
{
	unsigned char temp[2];
	
	axp192_read(POWER_DATA_BUFFER2,&temp[0]);
	axp192_read(POWER_DATA_BUFFER3,&temp[1]);

	Cou_Counter /= 5;
	Cou_Counter |= 0x800;
	temp[0] = ((Cou_Counter & 0xff0) >> 4);
	temp[1] &= 0x0f;
	temp[1] |= (Cou_Counter & 0x0f) << 4;	

	axp192_write(POWER_DATA_BUFFER2,temp[0]);
	axp192_write(POWER_DATA_BUFFER3,temp[1]);	
}

#if 0
static unsigned long tcc_read_adc(void)
{
	unsigned long adcValue = 0;
	
	ADC_EnableBATDet();
	adcValue = Get_Single_Adc_Data(BAT_VOL);
	
	gAvrVoltage[gIndex%BATTVOLSAMPLE] = adcValue;
	gIndex++;

	return adcValue;
}	
#endif

uint16 Get_Bat_Vol(void)
{
	static uint16 avrBatVol = 0;
//	int i;
	
	avrBatVol = Get_Single_Adc_Data(BAT_VOL);
	DEBUG_PRINT("avrBatVol:%d\n",avrBatVol);

	Total_BatVol -= gAvrVoltage[gIndex];
	gAvrVoltage[gIndex] = avrBatVol;
	Total_BatVol += gAvrVoltage[gIndex];
	gIndex++;
	if(gIndex == BATTVOLSAMPLE)
	{
		gIndex = 0;
	}
	if(volIndex < BATTVOLSAMPLE)
	{
		volIndex++;
	}
/*	
	for(i = 0;i<BATTVOLSAMPLE;i++)
	{
		DEBUG_PRINT("gAvrVoltage[%d]=%d\n",i,gAvrVoltage[i]);

	}
	DEBUG_PRINT("AvrVoltage:%d\n",Total_BatVol/volIndex);	
*/	
	return Total_BatVol / volIndex;
}

void Initial_Process(void)
{
	ADC_Enable_Set(0xffff);
	ADC_Range_Freq_Set(LEVEL3,0,0);

//	flag_lastvol = Get_Lastvol_Flag();

	Fuelguage.bat_cap = Get_Buffer_Cou();
	
	if(Fuelguage.bat_cap == 0)
	{
		Fuelguage.bat_cap = BAT_CAP;
	}
	
	Bat_Rdc = RDC_DEFAULT;
	Bat_Rdc = Get_Buffer_Rdc();

	Pre_Rdc_Flag = (Bat_Rdc & 0x800) >> 11;
	Bat_Rdc = (Bat_Rdc & 0x7ff) * 3;

	if(!Pre_Rdc_Flag){
		Bat_Rdc = RDC_DEFAULT;
	}
	
	batj = 0;
	for(bati = 0; bati < BUFFER_LONG; bati ++)	//Initial
		Bat_Cap_Buffer[bati] = 0;
		
	bati = 0;
	Total_Cap = 0;
	

	for(gIndex = 0;gIndex < BATTVOLSAMPLE;gIndex++)
		gAvrVoltage[gIndex] = 0;
	gIndex = 0;
	
	DEBUG_PRINT("Initial read:flag_lastvol=%d\n",flag_lastvol);
	DEBUG_PRINT("Initial read:Fuelguage.bat_cap=%d\n",Fuelguage.bat_cap);
	DEBUG_PRINT("Initial read:Bat_Rdc=%d\n",Bat_Rdc);
}


uint16 Get_Final_BatCap(void)
{
//	uint16 prev_rest_cap=0xffff;
//	uint16 array_fillup_flag=0;

//	uint16	 Bat_Time_Buffer[TIME_BUFFER_LONG]; 
//	int32	 Pre_CoulombCounter;
	int32	 Cur_CoulombCounter;
	static uint8	 Pre_ocv_rest_cap = 0;	
	static uint8	 Pre_rest_cap = 0;
//	static uint8	 rt_pre_rest_cap = 0;
	static uint8	 ocv_pre_rest_cap = 0;
	_drv_power_fuelgauge rt_Fuelguage,ocv_Fuelguage,cou_Fuelguage; //cou_Fuelguage

	uint8 Cou_Correction_Flag = 0;
//	uint8 i;

	if(!Init_Flag)
	{
		Initial_Process();
		Init_Flag = 1;
	}

	flag_lastvol = Get_Lastvol_Flag();
	
	Get_Charge_Status(&Charge_Stat);

/*************************************************************/
	if(AverVoltageBuffer_Clrflg)
	{		
		AverVoltageBuffer_Clrflg = 0;
		volIndex = 0;
		for(gIndex = 0;gIndex < BATTVOLSAMPLE;gIndex++)
			gAvrVoltage[gIndex] = 0;
		
		gIndex = 0;
		Total_BatVol = 0;		
	}
/*************************************************************/	

//	Bat_Rdc = Get_Bat_Rdc();
#if 1
	if(Charge_Stat.bat_current_direction && Charge_Stat.charge_indication && (Charge_Stat.battery_current > 300) && (!Rdc_Flag))//charge and current > 300mA 	
	{
		if(Pre_Rdc_Flag)
		{
			DEBUG_PRINT("Rdc detect start!\n");
			Bat_Rdc += Get_Bat_Rdc();
			Bat_Rdc /= 2;
		}
		else
		{
			Bat_Rdc = Get_Bat_Rdc();
		}
		Buffer_Rdc_Set(((10 * Bat_Rdc + 15) / 30) | 0x800);
		Rdc_Flag = 1;

		batj = 0;
		for(bati = 0; bati < BUFFER_LONG; bati ++)	//Initial
			Bat_Cap_Buffer[bati] = 0;
//		for(batk = 0; batk < TIME_BUFFER_LONG; batk ++)	//Initial
//			Bat_Time_Buffer[batk] = 0;			
		bati = 0;
		gIndex = 0;
//		batk = 0;
		Total_Cap = 0;
		Total_Time = 0;
	}
#endif

	
	Bat_Vol = Get_Bat_Vol();  
//	Bat_Vol = Get_Single_Adc_Data(BAT_VOL);
	Charge_Stat.battery_current = Get_Single_Adc_Data(BAT_CUR);
	Bat_Ocv_Vol = Get_Bat_OcvVol(Charge_Stat.ext_power_valid && Charge_Stat.bat_current_direction,
		Bat_Vol,Charge_Stat.battery_current,Bat_Rdc);

	DEBUG_PRINT("Bat_Rdc = %d,Bat_Vol = %d,Bat_Cur = %d,Bat_Ocv_Vol=%d\n",Bat_Rdc,Bat_Vol,
		Charge_Stat.battery_current,Bat_Ocv_Vol);

	rt_Fuelguage.rest_cap = Get_Bat_RestVol(Bat_Ocv_Vol);
	DEBUG_PRINT("rt_Fuelguae.rest_cap:%d\n",rt_Fuelguage.rest_cap);
	rt_Fuelguage.charge_status = Charge_Stat.ext_power_valid << 2 | Charge_Stat.battery_exist << 1 | Charge_Stat.charge_indication;	

	
	Total_Cap-=Bat_Cap_Buffer[bati];
	Bat_Cap_Buffer[bati] = rt_Fuelguage.rest_cap;
	Total_Cap += Bat_Cap_Buffer[bati];
	bati++;
	if(bati == BUFFER_LONG)
	{
		bati = 0;
	}
	if(batj < BUFFER_LONG)
	{
		batj++;
	}
/*
	for(i=0;i<BUFFER_LONG;i++)
	{
		DEBUG_PRINT("Bat_Cap_Buffer[%d]=%d\n",i,Bat_Cap_Buffer[i]);
	}
*/	
	ocv_Fuelguage.rest_cap = (Total_Cap+batj/2)/batj;

	if((ocv_Fuelguage.rest_cap > ocv_pre_rest_cap) && (rt_Fuelguage.charge_status < 0x04) && ocv_pre_rest_cap) //\B7ŵ粻\C4\DC\D4\F6\BC\D3
		ocv_Fuelguage.rest_cap = ocv_pre_rest_cap;	
	
	if((ocv_Fuelguage.rest_cap < ocv_pre_rest_cap) && (rt_Fuelguage.charge_status > 0x04) && ocv_pre_rest_cap) //\B3\E4\B5粻\C4\DC\D4\F6\BC\D3
		ocv_Fuelguage.rest_cap = ocv_pre_rest_cap;	
	
	ocv_pre_rest_cap = ocv_Fuelguage.rest_cap;
		

	DEBUG_PRINT("ocv_Fuelguage.rest_cap:%d\n",ocv_Fuelguage.rest_cap);

	Fuelguage.rest_cap = ocv_Fuelguage.rest_cap;
	if ((batj == BUFFER_LONG) || (!flag_lastvol))
	{  
		DEBUG_PRINT("batj == BUFFER_LONG - [Fuelguage.bat_cap] = %d\n",Fuelguage.bat_cap);			
		DEBUG_PRINT("Fuelguage.bat_cap = %d \n", Fuelguage.bat_cap);
		Cur_CoulombCounter = Get_Bat_Coulomb_Count(Fuelguage.bat_cap);
		DEBUG_PRINT("Cur_CoulombCounter = %d \n", Cur_CoulombCounter);
		DEBUG_PRINT("Cur_CoulombCounter = %d\n",Cur_CoulombCounter);
		
		if((ocv_Fuelguage.rest_cap < 5) && Rdc_Flag && (rt_Fuelguage.charge_status == 7) && (!Cou_Correction_Flag))
		{			
				Cou_Correction_Flag = 0x01;
				Cou_Count_Clear();
				Pre_ocv_rest_cap = ocv_Fuelguage.rest_cap;	
				DEBUG_PRINT("Battery coulomb calibration start:Pre_rest_cap = %d\n",Pre_ocv_rest_cap);
		}
		
		if(Cou_Correction_Flag && (rt_Fuelguage.charge_status == 6) && (ocv_Fuelguage.rest_cap == 100))
		{
			Fuelguage.bat_cap = ABS(0 - Cur_CoulombCounter);
			Buffer_Cou_Set(Fuelguage.bat_cap);
			Cou_Correction_Flag = 0x00;    
			DEBUG_PRINT("Battery coulomb calibration finished:Fuelguage.bat_cap = %d\n",Fuelguage.bat_cap);
		}
		cou_Fuelguage.rest_cap = (100 * ABS(Cur_CoulombCounter) / Fuelguage.bat_cap);  
/******************************
\B3\F6\B4\ED\B4\A6\C0\ED\A3\AC\C8\E7\B9\FB\B3\F5ʼPre_rest_cap
\B9\C0\BC\C6ƫ\B4󣬵\BC\D6³\E4\B5\E7ʱ\D3ÿ\E2\C2ؼƼ\C6\CB\E3\B5ĵ\E7\C1\BF\B4\F3\D3\DA100%
**************************/
		if(cou_Fuelguage.rest_cap > 100)
			cou_Fuelguage.rest_cap = 100;
/*********************************************************
***************************************************************************/
		Fuelguage.rest_cap = cou_Fuelguage.rest_cap;
		DEBUG_PRINT("Fuelguage.bat_cap = %d\%, Cur_CoulombCounter = %d, <==> "
                             "cou_Fuelguage.rest_cap = %d\%\n",Fuelguage.bat_cap,Cur_CoulombCounter, cou_Fuelguage.rest_cap);	
		DEBUG_PRINT("ocv_Fuelguage.rest_cap = %d\%, <==> "
                             "cou_Fuelguage.rest_cap = %d\%\n",ocv_Fuelguage.rest_cap,cou_Fuelguage.rest_cap);
		
		Fuelguage.charge_status = rt_Fuelguage.charge_status;
		DEBUG_PRINT(" (Fuelguage.charge_status = %x)\n", Fuelguage.charge_status);
/******************************\B3\F6\B4\ED\B4\A6\C0\ED\A3\AC\C8\E7\B9\FB\B3\F5ʼPre_rest_cap
\B9\C0\BC\C6ƫС\A3\AC\B5\BC\D6³\E4\C2\FA\BA\F3\D3ÿ\E2\C2ؼƼ\C6\CB\E3\B5ĵ\E7\C1\BF\B5\CD\D3\DA100%****************/
		if((ocv_Fuelguage.rest_cap > 98) && (cou_Fuelguage.rest_cap < 99) && (Fuelguage.charge_status == 0x07))
		{
			if(cou_Fuelguage.rest_cap < 99)
			{
				cou_Fuelguage.rest_cap ++;
				Set_Rest_Cap(cou_Fuelguage.rest_cap);									
				Cou_Count_Clear(); 
			}
		}

		if((ocv_Fuelguage.rest_cap > 99) && (cou_Fuelguage.rest_cap < 100) && (Fuelguage.charge_status == 0x06))
		{
			if(cou_Fuelguage.rest_cap < 100)
			{
				cou_Fuelguage.rest_cap ++;
				Set_Rest_Cap(cou_Fuelguage.rest_cap);
				Cou_Count_Clear(); 
			}
		}
/******************************\B3\F6\B4\ED\B4\A6\C0\ED\A3\AC\C8\E7\B9\FB\B3\F5ʼPre_rest_cap
\B9\C0\BC\C6ƫС\A3\AC\B5\BC\D6³\E4\C2\FA\BA\F3\D3ÿ\E2\C2ؼƼ\C6\CB\E3\B5ĵ\E7\C1\BF\B5\CD\D3\DA100%****************/
/******************************\B3\F6\B4\ED\B4\A6\C0\ED\A3\AC\C8\E7\B9\FB\B3\F5ʼPre_rest_cap
\B9\C0\BC\C6ƫС\A3\AC\B5\BC\D6·ŵ\E7\BA\F3\D3ÿ\E2\C2ؼƼ\C6\CB\E3\B5ĵ\E7\C1\BF\B5\CD\D3\DA0%****************/
		if((ocv_Fuelguage.rest_cap > 9) && (cou_Fuelguage.rest_cap < 10) && (Fuelguage.charge_status < 0x04))
		{
			cou_Fuelguage.rest_cap ++;
			//cou_Fuelguage.rest_cap = 9;
			Set_Rest_Cap(cou_Fuelguage.rest_cap);
			Cou_Count_Clear(); 
		}
/******************************\B3\F6\B4\ED\B4\A6\C0\ED\A3\AC\C8\E7\B9\FB\B3\F5ʼPre_rest_cap
\B9\C0\BC\C6ƫС\A3\AC\B5\BC\D6·ŵ\E7\BA\F3\D3ÿ\E2\C2ؼƼ\C6\CB\E3\B5ĵ\E7\C1\BF\B5\CD\D3\DA0%****************/

/******************************\B3\F6\B4\ED\B4\A6\C0\ED\A3\AC\C8\E7\B9\FB\B3\F5ʼPre_rest_cap
\B9\C0\BC\C6ƫС\A3\AC\B5\BC\D6·ŵ\E7\BA\F3\D3ÿ\E2\C2ؼƼ\C6\CB\E3\B5ĵ\E7\C1\BF\B5\CD\D3\DA100%****************/
		if((ocv_Fuelguage.rest_cap < 100) && (cou_Fuelguage.rest_cap > 100) && (Fuelguage.charge_status < 0x04))
		{
			if(cou_Fuelguage.rest_cap > ocv_Fuelguage.rest_cap)
			{
				cou_Fuelguage.rest_cap --;
				Set_Rest_Cap(cou_Fuelguage.rest_cap);
				Cou_Count_Clear();
			}
		}
/******************************\B3\F6\B4\ED\B4\A6\C0\ED\A3\AC\C8\E7\B9\FB\B3\F5ʼPre_rest_cap
\B9\C0\BC\C6ƫС\A3\AC\B5\BC\D6·ŵ\E7\BA\F3\D3ÿ\E2\C2ؼƼ\C6\CB\E3\B5ĵ\E7\C1\BF\B5\CD\D3\DA100%****************/
/******************************\B3\F6\B4\ED\B4\A6\C0\ED\A3\AC\C8\E7\B9\FB\B3\F5ʼPre_rest_cap
\B9\C0\BC\C6ƫ\B4󣬵\BC\D6·ŵ\E7\BA\F3\D3ÿ\E2\C2ؼƼ\C6\CB\E3\B5ĵ\E7\C1\BF\B9\FD\B8\DF********************/
		if((ocv_Fuelguage.rest_cap < 10) && (cou_Fuelguage.rest_cap > 9) && (Fuelguage.charge_status < 0x04))
		{
			if(cou_Fuelguage.rest_cap > ocv_Fuelguage.rest_cap)
			{
				cou_Fuelguage.rest_cap --;
				Set_Rest_Cap(cou_Fuelguage.rest_cap);
				Cou_Count_Clear();
			}
		}

/*****************************************
\B3\F6\B4\ED\B4\A6\C0\ED\A3\AC\C8\E7\B9\FB\BB\B9\D4ڳ\E4\B5磬\BC\B4ʹ\B7\B5\BB\D8100%
\A3\ACҲ\D6\C3Ϊ99%**********************************/
		if((Fuelguage.charge_status == 0x07) && (Fuelguage.rest_cap >= 100))
		{
			DEBUG_PRINT("[Before fix] Fuelguage.rest_cap = %d\n", Fuelguage.rest_cap);
			Fuelguage.rest_cap = 99;
		}
		if((cou_Fuelguage.rest_cap > Pre_rest_cap) && (Fuelguage.charge_status < 0x04) && Pre_rest_cap) 
			//\B7ŵ粻\C4\DC\D4\F6\BC\D3
		{
			Fuelguage.rest_cap = Pre_rest_cap;
			Set_Rest_Cap(Fuelguage.rest_cap);
			Cou_Count_Clear();				
		}
		if((cou_Fuelguage.rest_cap < Pre_rest_cap) && (Fuelguage.charge_status > 0x04) && Pre_rest_cap) 
			//\B3\E4\B5粻\C4\DC\D4\F6\BC\D3
		{
			Fuelguage.rest_cap = Pre_rest_cap;
			Set_Rest_Cap(Fuelguage.rest_cap);
			Cou_Count_Clear();				
		}
		Pre_rest_cap = Fuelguage.rest_cap;
	}
	else if(flag_lastvol)
	{
		DEBUG_PRINT("excute when flag_lastvol=1\n");
		Fuelguage.charge_status = rt_Fuelguage.charge_status;
		Pre_ocv_rest_cap = ocv_Fuelguage.rest_cap;
		Fuelguage.rest_cap = ocv_Fuelguage.rest_cap;

		if((batj == BUFFER_LONG - 1))
		{
			DEBUG_PRINT("@batj == BUFFER_LONG - 1 [Pre_rest_cap = %d\%]\n",Pre_ocv_rest_cap);
			Set_Rest_Cap(Pre_ocv_rest_cap);
			Pre_rest_cap = Pre_ocv_rest_cap;
			Cou_Count_Clear();
			Set_Flag_Lastvol();
		}
	}	
	DEBUG_PRINT("ocv_Fuelguage.rest_cap=%d  cou_Fuelguage.rest_cap=%d Pre_rest_cap=%d Pre_ocv_rest_cap=%d flag_lastval=%d\n",
			ocv_Fuelguage.rest_cap,cou_Fuelguage.rest_cap,Pre_rest_cap,Pre_ocv_rest_cap,flag_lastvol);

	return ocv_Fuelguage.rest_cap;
}

/******************************************************************/

static void emxx_battery_work(struct work_struct *work)
{
	unsigned char status;
	const int interval = HZ * ADC_SAMPLE_RATE;
	unsigned int old_voltage_level;
	unsigned int old_battery_voltage;
	unsigned int old_battery_temp;
//	uint adc_value;
	
	if(Flg_otg_on == 1)
	{
		if(gpio_get_value(GPIO_P101))
		{
			axp192_read(AXP_VBUS_IPSOUT, &status);
			axp192_write(AXP_VBUS_IPSOUT,status|0x80);

			Flg_otg_on = 0;
		}
	}
	old_voltage_level = battery_data->voltage_level;
	old_battery_voltage = battery_data->battery_voltage;
	old_battery_temp = battery_data->battery_temp;

//	unsigned char vbat_h, vbat_l;
//  axp192_read( AXP_VBAT_RESH, &vbat_h);
//	axp192_read( AXP_VBAT_RESL, &vbat_l);
	
//	adc_value=Get_Single_Adc_Data(BAT_VOL);
//	battery_data->voltage_level=100*(((unsigned int)vbat_h<<4)|(vbat_l&0xf))/4200;
	battery_data->voltage_level=Get_Final_BatCap();	
	battery_data->battery_voltage = Bat_Ocv_Vol*1000;
	battery_data->battery_temp = Get_Single_Adc_Data(INT_TEMP)*10;

	if(old_battery_temp > battery_data->battery_temp + 100)
	{
		battery_data->battery_temp = old_battery_temp - 100;
	}
	else if(old_battery_temp < battery_data->battery_temp - 100)
	{
		battery_data->battery_temp = old_battery_temp + 100;
	}

	if(battery_data->battery_temp > 600)
		battery_data->battery_temp = 600;
	
	DEBUG_PRINT("voltage_level=%d  battery_voltage=%d  battery_temp=%d\n",battery_data->voltage_level,
		battery_data->battery_voltage,battery_data->battery_temp);

//err:
	/* If status have changed, update the status */
	if (old_voltage_level != battery_data->voltage_level || old_battery_voltage != battery_data->battery_voltage ||
		old_battery_temp != battery_data->battery_temp) {
		DEBUG_PRINT("Update charger status...\n");
		power_supply_changed(&battery_data->battery);
	}
	queue_delayed_work(battery_data->monitor_wqueue,
			&(battery_data->monitor_work), interval);
}
#endif


static int emxx_power_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	charger_type_t charger;

	charger = battery_data->charger;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:	/* 3 */
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (charger ==  CHARGER_AC ? 1 : 0);
		else if (psy->type == POWER_SUPPLY_TYPE_USB) {
			if (battery_data->usb_state == EMXX_USB_OFF)
				val->intval = 0;
			else
				val->intval = 1;
		} else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int emxx_battery_usb_state(int state)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&battery_data->lock, flags);

	if (battery_data->usb_state != state) {
		switch (state) {
		case EMXX_USB_DEVICE:
#ifdef CONFIG_EMXX_DUMMPYBATTRY
			if (battery_data->lock_status == 0) {
				battery_data->lock_status = 1;
				wake_lock(&battery_data->vbus_suspend_lock);
			}
#endif
			break;
		case EMXX_USB_DEVICE_100:
#ifdef CONFIG_EMXX_DUMMPYBATTRY
			if (battery_data->lock_status == 0) {
				battery_data->lock_status = 1;
				wake_lock(&battery_data->vbus_suspend_lock);
			}
#endif
			break;
		case EMXX_USB_CHARGER:
#ifdef CONFIG_EMXX_DUMMPYBATTRY
			if (battery_data->lock_status == 0) {
				battery_data->lock_status = 1;
				wake_lock(&battery_data->vbus_suspend_lock);
			}
#endif
			break;
		case EMXX_USB_OFF:
#ifdef CONFIG_EMXX_DUMMPYBATTRY
			if (battery_data->lock_status == 1) {
				battery_data->lock_status = 0;
				wake_unlock(&battery_data->vbus_suspend_lock);
			}
#endif
			break;
		default:
			DEBUG_PRINT("ERROR state: (%d)\n", state);
			ret = -EINVAL;
			goto err;
		}
		battery_data->usb_state = state;
	}
err:
	spin_unlock_irqrestore(&battery_data->lock, flags);
	return ret;
}
EXPORT_SYMBOL(emxx_battery_usb_state);

static int emxx_battery_probe(struct platform_device *pdev)
{
	int ret;
	u32 p_value;
	struct emxx_battery_data *data;
	unsigned int status;

	printk( "emxx_battery_probe...\n");
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	spin_lock_init(&data->lock);

	/* Battey */
	data->battery.name = "battery";
	data->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	data->battery.properties = emxx_battery_props;
	data->battery.num_properties = ARRAY_SIZE(emxx_battery_props);
	data->battery.get_property = emxx_battery_get_property;

	/* USB */
	data->usb.name = "usb";
	data->usb.type = POWER_SUPPLY_TYPE_USB;
	data->usb.properties = emxx_power_props;
	data->usb.num_properties = ARRAY_SIZE(emxx_power_props);
	data->usb.get_property = emxx_power_get_property;

	/* AC */
	data->ac.name = "ac";
	data->ac.type = POWER_SUPPLY_TYPE_MAINS;
	data->ac.properties = emxx_power_props;
	data->ac.num_properties = ARRAY_SIZE(emxx_power_props);
	data->ac.get_property = emxx_power_get_property;

	battery_data = data;	
	writel(readl(CHG_PINSEL_G096)|0x00000020, CHG_PINSEL_G096);	
	gpio_direction_input(GPIO_P101);
	p_value = readl(CHG_PULL11)&0xffffffdf;
	p_value |= 0x000000d0;
	writel(p_value, CHG_PULL11);

#ifdef CONFIG_EMXX_DUMMPYBATTRY
	/* Dummy battery setting (always 100%) */
	battery_data->voltage_level = 100;
	battery_data->usb_state = EMXX_USB_OFF;
	battery_data->lock_status = 0;

	wake_lock_init(&data->vbus_suspend_lock,
			WAKE_LOCK_SUSPEND, "vbus_suspend");

	status = emxx_charger_get_status();
#else
	/* Dummy battery setting (100%) */
	battery_data->voltage_level = 100;
	battery_data->usb_state = EMXX_USB_OFF;
	battery_data->flag_adc_battery = FALSE;

	status = emxx_charger_get_status();

#if 0
	ret = request_irq(INT_PWC_E_DCIN_DET, emxx_charger_irq,
		IRQF_SHARED, "charger", data);
	if (ret)
		goto err_irq_ac1;

	ret = request_irq(INT_PWC_E_DCIN_REM, emxx_charger_irq,
		IRQF_SHARED, "charger", data);
	if (ret)
		goto err_irq_ac2;

	ret = request_irq(INT_PWC_E_VBUS_DET, emxx_charger_irq,
		IRQF_SHARED, "charger", data);
	if (ret)
		goto err_irq_usb1;

	ret = request_irq(INT_PWC_E_VBUS_REM, emxx_charger_irq,
		IRQF_SHARED, "charger", data);
	if (ret)
		goto err_irq_usb2;
#else
#endif
#endif
	DEBUG_PRINT("Call axp192_setbattry_callback\n");
	axp192_setbattry_callback( emxx_battry_status_changed);

	ret = power_supply_register(&pdev->dev, &data->battery);
	if (ret)
		goto err_battery_failed;

	ret = power_supply_register(&pdev->dev, &data->usb);
	if (ret)
		goto err_usb_failed;

	ret = power_supply_register(&pdev->dev, &data->ac);
	if (ret)
		goto err_ac_failed;

#ifndef CONFIG_EMXX_DUMMPYBATTRY
	/* Init DA9052 for Charger */
	//pwc_write(DA9052_CHGBUCK_REG, 0x0F, 0x0F);
	//pwc_write(DA9052_ISET_REG, 0xF8, 0xFF);
	//pwc_write(DA9052_BATCHG_REG, 0x37, 0x3F);
	//pwc_write(DA9052_CHGCONT_REG, 0xD1, 0xFF);
	//pwc_write(DA9052_BBATCONT_REG, 0x1E, 0xFF);

	INIT_DELAYED_WORK(&data->monitor_work, emxx_battery_work);
	data->monitor_wqueue =
		create_singlethread_workqueue("axp192");
	if (!data->monitor_wqueue) {
		ret = -ESRCH;
		goto err_workqueue_failed;
	}
	queue_delayed_work(data->monitor_wqueue, &data->monitor_work, HZ);
#endif

	initliased = 1;
	platform_set_drvdata(pdev, data);
	printk( "emxx_battery_probe done\n");
	return 0;

#ifdef CONFIG_EMXX_DUMMPYBATTRY
err_ac_failed:
	power_supply_unregister(&data->usb);
err_usb_failed:
	power_supply_unregister(&data->battery);
err_battery_failed:
	wake_lock_destroy(&data->vbus_suspend_lock);
	kfree(data);
#else
err_workqueue_failed:
	power_supply_unregister(&data->ac);
err_ac_failed:
	power_supply_unregister(&data->usb);
err_usb_failed:
	power_supply_unregister(&data->battery);
err_battery_failed:
	//free_irq(INT_PWC_E_VBUS_REM, data);
//err_irq_usb2:
	//free_irq(INT_PWC_E_VBUS_DET, data);
//err_irq_usb1:
	//free_irq(INT_PWC_E_DCIN_REM, data);
//err_irq_ac2:
	//free_irq(INT_PWC_E_DCIN_DET, data);
//err_irq_ac1:
//	kfree(data);
#endif
err_data_alloc_failed:
	printk( "emxx_battery_probe error %x\n", ret);
	return ret;
}

static int emxx_battery_remove(struct platform_device *pdev)
{
	struct emxx_battery_data *data = platform_get_drvdata(pdev);

	DEBUG_PRINT("Battery driver remove...\n");

#ifdef CONFIG_EMXX_DUMMPYBATTRY
	wake_lock_destroy(&data->vbus_suspend_lock);
#else
	//free_irq(INT_PWC_E_DCIN_DET, data);
	//free_irq(INT_PWC_E_DCIN_REM, data);
	//free_irq(INT_PWC_E_VBUS_DET, data);
	//free_irq(INT_PWC_E_VBUS_REM, data);
#endif
	power_supply_unregister(&data->battery);
	power_supply_unregister(&data->usb);
	power_supply_unregister(&data->ac);

	kfree(data);
	battery_data = NULL;
	return 0;
}

#ifdef CONFIG_PM
static int emxx_battery_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	return 0;
}

static int emxx_battery_resume(struct platform_device *pdev)
{
#ifndef CONFIG_EMXX_DUMMPYBATTRY
	struct emxx_battery_data *data = platform_get_drvdata(pdev);

	power_supply_changed(&data->battery);

	cancel_delayed_work(&data->monitor_work);
	queue_delayed_work(data->monitor_wqueue, &data->monitor_work, HZ);

	battery_data->flag_adc_battery = FALSE;
#endif
	return 0;
}
#else
#define emxx_battery_suspend NULL
#define emxx_battery_resume NULL
#endif /* CONFIG_PM */

static struct platform_driver emxx_battery_driver = {
	.probe		= emxx_battery_probe,
	.remove		= emxx_battery_remove,
	.suspend	= emxx_battery_suspend,
	.resume 	= emxx_battery_resume,
	.driver = {
		.name = "emxx-battery"
	}
};

static int __init emxx_battery_init(void)
{
	return platform_driver_register(&emxx_battery_driver);
}

static void __exit emxx_battery_exit(void)
{
	platform_driver_unregister(&emxx_battery_driver);
}

module_init(emxx_battery_init);
module_exit(emxx_battery_exit);

MODULE_AUTHOR("Renesas");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Battery driver for the EMMA Mobile series");
