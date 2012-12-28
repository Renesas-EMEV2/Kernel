#ifndef __AXP192_BATTERY_H
#define __AXP192_BATTERY_H

struct axp192_param{
	struct i2c_client client;
};

//电源控制类寄存器
#define AXP192_REG0 	0x00
#define AXP192_REG1 	0x01

#define AXP192_REG4 	0x04

#define AXP192_REG6 	0x06
#define AXP192_REG7 	0x07
#define AXP192_REG8 	0x08
#define AXP192_REG9 	0x09
#define AXP192_REG10 	0x10
#define AXP192_REG12	0x12

#define AXP192_REG23 	0x23

#define AXP192_REG25 	0x25
#define AXP192_REG26 	0x26
#define AXP192_REG27 	0x27
#define AXP192_REG28 	0x28
#define AXP192_REG29 	0x29
#define AXP192_REG30 	0x30
#define AXP192_REG31 	0x31
#define AXP192_REG32 	0x32
#define AXP192_REG33 	0x33
#define AXP192_REG34 	0x34
#define AXP192_REG35 	0x35
#define AXP192_REG36 	0x36
#define AXP192_REG37 	0x37
#define AXP192_REG38 	0x38
#define AXP192_REG39 	0x39
#define AXP192_REG3A	0x3a
#define AXP192_REG3B	0x3b
#define AXP192_REG3C	0x3c
#define AXP192_REG3D	0x3d

#define AXP192_REG80 	0x80

#define AXP192_REG82 	0x82
#define AXP192_REG83 	0x83
#define AXP192_REG84 	0x84
#define AXP192_REG85 	0x85

#define AXP192_REG8A	0x8a
#define AXP192_REG8B	0x8b
#define AXP192_REG8F	0x8f

//GPIO控制寄存器
#define AXP192_REG90 	0x90
#define AXP192_REG91 	0x91
#define AXP192_REG92 	0x92
#define AXP192_REG93 	0x93
#define AXP192_REG94 	0x94
#define AXP192_REG95 	0x95
#define AXP192_REG96	0x96
#define AXP192_REG97 	0x97
#define AXP192_REG98 	0x98
#define AXP192_REG99 	0x99
#define AXP192_REG9A	0x9a
#define AXP192_REG9B	0x9b
#define AXP192_REG9C	0x9c
#define	AXP192_REG9D	0x9d	 
#define AXP192_REG9E	0x9e

//中断控制寄存器
#define AXP192_REG40 	0x40
#define AXP192_REG41 	0x41
#define AXP192_REG42 	0x42
#define AXP192_REG43 	0x43
#define AXP192_REG44 	0x44
#define AXP192_REG45 	0x45
#define AXP192_REG46 	0x46
#define AXP192_REG47 	0x47

//ADC数据寄存器
#define AXP192_REG56 	0x56
#define AXP192_REG57 	0x57
#define AXP192_REG58 	0x58
#define AXP192_REG59 	0x59
#define AXP192_REG5A	0x5a
#define AXP192_REG5B	0x5b
#define AXP192_REG5C	0x5c
#define AXP192_REG5D	0x5d
#define AXP192_REG5E	0x5e
#define AXP192_REG5F	0x5f

#define AXP192_REG62 	0x62
#define AXP192_REG63 	0x63
#define AXP192_REG64 	0x64
#define AXP192_REG65 	0x65
#define AXP192_REG66 	0x66
#define AXP192_REG67 	0x67
#define AXP192_REG68 	0x68
#define AXP192_REG69 	0x69
#define AXP192_REG6A	0x6a
#define AXP192_REG6B	0x6b

#define AXP192_REG70 	0x70
#define AXP192_REG71 	0x71
#define AXP192_REG72	0x72

#define AXP192_REG78 	0x78
#define AXP192_REG79 	0x79
#define AXP192_REG7A	0x7a
#define AXP192_REG7B	0x7b
#define AXP192_REG7C	0x7c
#define AXP192_REG7D	0x7d
#define AXP192_REG7E	0x7e
#define AXP192_REG7F	0x7f

#define AXP192_REGB0	0xb0
#define AXP192_REGB1	0xb1
#define AXP192_REGB2	0xb2
#define AXP192_REGB3	0xb3
#define AXP192_REGB4	0xb4
#define AXP192_REGB5	0xb5
#define AXP192_REGB6	0xb6	
#define AXP192_REGB7	0xb7	
#define AXP192_REGB8	0xb8

#define IRQ40H_ALL		0xff
#define IRQ41H_ALL		0xff
#define IRQ42H_ALL		0xff
#define IRQ43H_ALL		0xff

#define IRQ_PEK_MASK	0xfc

#define MASK_IRQ44H		0xff
#define MASK_IRQ45H		0xff
#define MASK_IRQ46H		0xff
#define MASK_IRQ47H		0xff

/******************************/
#define MASK_BIT(bit)       (1<<(bit))
/******************************/

//内部充电电流寄存器
#define AXP192_mAIN_100		(MASK_BIT(7) | 0)
#define AXP192_mAIN_190		(MASK_BIT(7) | MASK_BIT(0))
#define AXP192_mAIN_280 	(MASK_BIT(7) | MASK_BIT(1))
#define AXP192_mAIN_360		(MASK_BIT(7) | MASK_BIT(1) | MASK_BIT(0))
#define AXP192_mAIN_450		(MASK_BIT(7) | MASK_BIT(2))
#define AXP192_mAIN_550		(MASK_BIT(7) | MASK_BIT(2) | MASK_BIT(0))
#define AXP192_mAIN_630		(MASK_BIT(7) | MASK_BIT(2) | MASK_BIT(1))
#define AXP192_mAIN_700		(MASK_BIT(7) | MASK_BIT(2) | MASK_BIT(1) | MASK_BIT(0))
#define AXP192_mAIN_780		(MASK_BIT(7) | MASK_BIT(3))
#define AXP192_mAIN_880 	(MASK_BIT(7) | MASK_BIT(3) | MASK_BIT(0))
#define AXP192_mAIN_960		(MASK_BIT(7) | MASK_BIT(3) | MASK_BIT(1))
#define AXP192_mAIN_1000	(MASK_BIT(7) | MASK_BIT(3) | MASK_BIT(1) | MASK_BIT(0))
#define AXP192_mAIN_1160	(MASK_BIT(7) | MASK_BIT(3) | MASK_BIT(2))
#define AXP192_mAIN_1240	(MASK_BIT(7) | MASK_BIT(3) | MASK_BIT(2) | MASK_BIT(1))
#define AXP192_mAIN_1320	(MASK_BIT(7) | MASK_BIT(3) | MASK_BIT(2) | MASK_BIT(1) | MASK_BIT(0))

#define AXP192_mAIN_MASK	(MASK_BIT(7)	| MASK_BIT(3) | MASK_BIT(2) | MASK_BIT(1) | MASK_BIT(0))

//充电目标电压设置
#define AXP192_Charging_4100	(0)
#define AXP192_Charging_4150	(MASK_BIT(5))
#define AXP192_Charging_4200	(MASK_BIT(6))
#define AXP192_Charging_4360	(MASK_BIT(6) | MASK_BIT(5))

#define AXP192_Charging_MASK	(MASK_BIT(6) | MASK_BIT(5))

//充电结束电流设置
#define AXP192_Percent10	(0)
#define AXP192_Percent15	(MASK_BIT(4))

#define AXP192_Percent_MASK	(MASK_BIT(4))
/************************************************/
#define BATTERY_IN_MASK		(BIT(5))
#define BATTERY_CURRENT_MASK	(BIT(2))
#define CHARGING_STATUS_MASK	(BIT(6))
#define AC_STATUS_MASK		(BIT(7))
#define USB_STATUS_MASK		(BIT(5))

//自动关机电压设置
#define POWEROFF_2600		(0)
#define POWEROFF_2700		(MASK_BIT(0))
#define POWEROFF_2800		(MASK_BIT(1))
#define POWEROFF_2900		(MASK_BIT(1) | MASK_BIT(0))
#define POWEROFF_3000		(MASK_BIT(2))
#define POWEROFF_3100		(MASK_BIT(2) | MASK_BIT(0))
#define POWEROFF_3200		(MASK_BIT(2) | MASK_BIT(1))
#define POWEROFF_3300		(MASK_BIT(2) | MASK_BIT(1) | MASK_BIT(0))

#define POWEROFF_VOL_MASK	(MASK_BIT(2) | MASK_BIT(1) | MASK_BIT(0))

//电池ADC 使能
#define BATTERY_VOL_ADC_ON	(MASK_BIT(7))
#define BATTERY_CUR_ADC_ON	(MASK_BIT(6))
#define BATTERY_ADC_OFF		(0)
#define BATTERY_ADC_MASK	(MASK_BIT(7) | MASK_BIT(6))

//电源ADC 使能
#define AC_VOL_ADC_ON		(MASK_BIT(5))
#define AC_CUR_ADC_ON		(MASK_BIT(4))
#define AC_ADC_OFF			(0)
#define AC_ADC_MASK			(MASK_BIT(5) | MASK_BIT(4))

//AC,USB,BATTERY状态改变
#define	AC_CHANGE_MASK		(MASK_BIT(6) | MASK_BIT(5))
#define USB_CHANGE_MASK		(MASK_BIT(3) | MASK_BIT(2))	
#define BATT_CHANGE_MASK	(MASK_BIT(7) | MASK_BIT(6))

//Set GPS Voltage
#define GPS_VOL_MASK		(MASK_BIT(7) | MASK_BIT(6) | MASK_BIT(5) | MASK_BIT(4))
#define GPS_VOL_1800		(0)
#define GPS_VOL_1900		(MASK_BIT(4))
#define GPS_VOL_2000		(MASK_BIT(5))
#define GPS_VOL_2100		(MASK_BIT(4) | MASK_BIT(5))
#define GPS_VOL_2200		(MASK_BIT(6))
#define GPS_VOL_2300		(MASK_BIT(6) | MASK_BIT(4))
#define GPS_VOL_2400		(MASK_BIT(6) | MASK_BIT(5))
#define GPS_VOL_2500		(MASK_BIT(6) | MASK_BIT(5) | MASK_BIT(4))
#define GPS_VOL_2600		(MASK_BIT(7))
#define GPS_VOL_2700		(MASK_BIT(7) | MASK_BIT(4))
#define GPS_VOL_2800		(MASK_BIT(7) | MASK_BIT(5))
#define GPS_VOL_2900		(MASK_BIT(7) | MASK_BIT(5) | MASK_BIT(4))
#define GPS_VOL_3000		(MASK_BIT(7) | MASK_BIT(6))
#define GPS_VOL_3100		(MASK_BIT(7) | MASK_BIT(6) | MASK_BIT(4))
#define GPS_VOL_3200		(MASK_BIT(7) | MASK_BIT(6) | MASK_BIT(5))
#define GPS_VOL_3300		(MASK_BIT(7) | MASK_BIT(6) | MASK_BIT(5) | MASK_BIT(4))

#define GPS_POWER_MASK		(MASK_BIT(2))
#define GPS_POWER_ON		(MASK_BIT(2))
#define GPS_POWER_OFF		(0)

//Set CPU Core Voltage
#define CPUCORE_VOL_MASK	(MASK_BIT(5) | MASK_BIT(4) | MASK_BIT(3) | MASK_BIT(2) | MASK_BIT(1) | MASK_BIT(0))
#define CPUCORE_VOL_700		(0)
#define CPUCORE_VOL_1100	(MASK_BIT(4))
#endif
