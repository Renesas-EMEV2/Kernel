/****************************************************************
 * arch/arm/mach-emxx/include/mach/axp192.h
 *
 * Copyright (C) 2010, Renesas Electornics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 ****************************************************************/
#ifndef _AXP192_H_
#define _AXP192_H_
#include <linux/i2c.h>

#define AXP192_ADDRESS 			0x34

#define AXP_PWR_STATE			0x00
#define AXP_PWR_ACIN 			0x80
#define AXP_PWR_ACIN_EN 		0x40
#define AXP_PWR_VBUS 			0x20
#define AXP_PWR_VBUS_EN 		0x10
#define AXP_PWR_BAT_CUR 		0x04
#define AXP_PWR_ACIN_VBUS 		0x02
#define AXP_PWR_MODE 			0x01

#define AXP_CHG_STATE			0x01
#define AXP_CHG_TOVER 			0x80
#define AXP_CHG_ING 			0x40
#define AXP_CHG_BAT 			0x20
#define AXP_CHG_RESERVED4 		0x10
#define AXP_CHG_ACT_MODE 		0x08
#define AXP_CHG_CUR	 		0x04
#define AXP_CHG_PWR_MODE 		0x02
#define AXP_CHG_RESERVED0 		0x01

#define AXP_OTG_STATE			0x04

#define AXP_DATA0			0x06
#define AXP_DATA1			0x07
#define AXP_DATA2			0x08
#define AXP_DATA3			0x09
#define AXP_BUCK2EN			0x10

#define AXP_BUCK13_LDO23EN		0x12

#define AXP_BUCK2_VOL			0x23

#define AXP_BUCK2_DVC			0x25
#define AXP_BUCK1_VOL			0x26
#define AXP_BUCK3_VOL			0x27
#define AXP_LDO23_VOL			0x28

#define AXP_VBUS_IPSOUT			0x30
#define AXP_PWROFF_VOL			0x31
#define AXP_PWROFF_CTRL			0x32
#define AXP_PWROFF_SHUTDOWN 		0x80
#define AXP_CHG_CTRL1			0x33
#define AXP_CHG_CTRL2			0x34
#define AXP_BBAT_CHG_CTRL		0x35
#define AXP_BBAT_CHG_EN 		0x80

#define AXP_PWRKEY_CTRL			0x36
#define AXP_PWRKEY_SHUTDOWN 		0x08
#define AXP_BUCK_CTRL			0x37
#define AXP_TBAT_CHGLOWER		0x38
#define AXP_TBAT_CHGHIGHER		0x39
#define AXP_APS_LEVEL1			0x3A
#define AXP_APS_LEVEL2			0x3B
#define AXP_TBAT_LOWER			0x3C
#define AXP_TBAT_HIGHER			0x3D

#define AXP_IRQ_MASK1			0x40
#define AXP_IRQ_STATE1			0x44
#define AXP_IRQ_ACIN_OV 		0x80
#define AXP_IRQ_ACIN_IN 		0x40
#define AXP_IRQ_ACIN_OUT 		0x20
#define AXP_IRQ_VBUS_OV 		0x10
#define AXP_IRQ_VBUS_IN 		0x08
#define AXP_IRQ_VBUS_OUT 		0x04
#define AXP_IRQ_VBUS_VHOLD 		0x02

#define AXP_IRQ_MASK2			0x41
#define AXP_IRQ_STATE2			0x45
#define AXP_IRQ_BAT_IN 			0x80
#define AXP_IRQ_BAT_OUT 		0x40
#define AXP_IRQ_BAT_ACT 		0x20
#define AXP_IRQ_BAT_UNACT 		0x10
#define AXP_IRQ_BAT_CHG 		0x08
#define AXP_IRQ_BAT_FULL 		0x04
#define AXP_IRQ_BAT_OTH 		0x02
#define AXP_IRQ_BAT_OTL 		0x01

#define AXP_IRQ_MASK3			0x42
#define AXP_IRQ_STATE3			0x46
#define AXP_IRQ_INT_OT 			0x80
#define AXP_IRQ_CHG_CURL 		0x40
#define AXP_IRQ_DCDC1_VOLL 		0x20
#define AXP_IRQ_DCDC2_VOLL 		0x10
#define AXP_IRQ_DCDC3_VOLL 		0x08
#define AXP_IRQ_KEY_SHORT 		0x02
#define AXP_IRQ_KEY_LONG 		0x01

#define AXP_IRQ_MASK4			0x43
#define AXP_IRQ_STATE4			0x47
#define AXP_IRQ_NOE_PWRON 		0x80
#define AXP_IRQ_NOE_PWROFF 		0x40
#define AXP_IRQ_VBUS_VALID 		0x20
#define AXP_IRQ_VBUS_UNVALID 		0x10
#define AXP_IRQ_VBUS_SESSION 		0x08
#define AXP_IRQ_VBUS_END 		0x04
#define AXP_IRQ_APS_LOWER 		0x01

#define AXP_ACIN_VOL_RESH		0x56
#define AXP_ACIN_VOL_RESL		0x57
#define AXP_ACIN_CUR_RESH		0x58
#define AXP_ACIN_CUR_RESL		0x59
#define AXP_VBUS_VOL_RESH		0x5A
#define AXP_VBUS_VOL_RESL		0x5B
#define AXP_VBUS_CUR_RESH		0x5C
#define AXP_VBUS_CUR_RESL		0x5D
#define AXP_TEMP_INT_RESH		0x5E
#define AXP_TEMP_INT_RESL		0x5F
#define AXP_TEMP_EXT_RESH		0x62
#define AXP_TEMP_EXT_RESL		0x63
#define AXP_GPIO0_ADC_RESH		0x64
#define AXP_GPIO0_ADC_RESL		0x65
#define AXP_GPIO1_ADC_RESH		0x66
#define AXP_GPIO1_ADC_RESL		0x67
#define AXP_GPIO2_ADC_RESH		0x68
#define AXP_GPIO2_ADC_RESL		0x69
#define AXP_GPIO3_ADC_RESH		0x6A
#define AXP_GPIO3_ADC_RESL		0x6B
#define AXP_POWER_RESH			0x70
#define AXP_POWER_RESM			0x71
#define AXP_POWER_RESL			0x72
#define AXP_VBAT_RESH			0x78
#define AXP_VBAT_RESL			0x79
#define AXP_CHG_CUR_RESH		0x7A
#define AXP_CHG_CUR_RESL		0x7B
#define AXP_UNCHG_CUR_RESH		0x7C
#define AXP_UNCHG_CUR_RESL		0x7D
#define AXP_APS_RESH			0x7E
#define AXP_APS_RESL			0x7F

#define AXP_BUCK_MODE			0x80
#define AXP_ADC_CTRL1			0x82
#define AXP_ADC_CTRL2			0x83
#define AXP_ADC_CTRL3			0x84
#define AXP_GPIO_OUT_RANGE		0x85
#define AXP_TIMER_CTRL			0x8A
#define AXP_VBUS_MONI			0x8B
#define AXP_INT_THRESHOLD		0x8F
#define AXP_INT_SHUTDOWN 		0x20

#define AXP_GPIO0_CTRL			0x90
#define AXP_GPIO0_LDO_VOL		0x91
#define AXP_GPIO1_CTRL			0x92
#define AXP_GPIO2_CTRL			0x93
#define AXP_GPIO_STATE1			0x94
#define AXP_GPIO_FUN_CTRL1		0x95
#define AXP_GPIO_STATE2			0x96
#define AXP_GPIO_PULL			0x97
#define AXP_PWM1_CTRL1			0x98
#define AXP_PWM1_CTRL2			0x99
#define AXP_PWM1_CTRL3			0x9A
#define AXP_PWM2_CTRL1			0x9B
#define AXP_PWM2_CTRL2			0x9C
#define AXP_PWM2_CTRL3			0x9D
#define AXP_NRSTO_CTRL			0x9E

/* CM: COULOMETER */
#define AXP_CHG_CM_DATA3		0xB0
#define AXP_CHG_CM_DATA2		0xB1
#define AXP_CHG_CM_DATA1		0xB2
#define AXP_CHG_CM_DATA0		0xB3
#define AXP_UNCHG_CM_DATA3		0xB4
#define AXP_UNCHG_CM_DATA2		0xB5
#define AXP_UNCHG_CM_DATA1		0xB6
#define AXP_UNCHG_CM_DATA0		0xB7

#define AXP_CM_CTRL			0xB8
#define AXP_CM_EN 			0x80
#define AXP_CM_STOP 			0x40
#define AXP_CM_CLEAN 			0x20

/* BUCK define */
#define AXP_VBUCK1_BASE		700
#define AXP_VBUCK1_MAX		3500
#define AXP_VBUCK1_STEP		25

#define AXP_VBUCK2_BASE		700
#define AXP_VBUCK2_MAX		2275
#define AXP_VBUCK2_STEP		25

#define AXP_VBUCK3_BASE		700
#define AXP_VBUCK3_MAX		3500
#define AXP_VBUCK3_STEP		25

#define AXP_LDO1_BASE		1250
#define AXP_LDO1_MAX		3300

#define AXP_LDO2_BASE		1800
#define AXP_LDO2_MAX		3300
#define AXP_LDO2_STEP		100

#define AXP_LDO3_BASE		1800
#define AXP_LDO3_MAX		3300
#define AXP_LDO3_STEP		100


struct  emxx_pmic_regs
{
	uint32_t data;
	uint32_t hit;
	uint32_t mask;
} ;

#define PMIC_EVENT_ACIN 	0x00000001
#define PMIC_EVENT_VBUS		0x00000002
#define PMIC_EVENT_BAT		0x00000004
#define PMIC_EVENT_CHG		0x00000008
#define PMIC_EVENT_TBAT		0x00000010
#define PMIC_EVENT_TINT		0x00000020
#define PMIC_EVENT_CURRENT	0x00000040
#define PMIC_EVENT_KEY		0x00000080
#define PMIC_EVENT_POWER	0x00000100
#define PMIC_EVENT_USB		0x00000200
#define PMIC_EVENT_APS		0x00000400

#define MAX_CMD			10
#define VCC_CORE 		0

enum {
	BUCK1 = 1,
	BUCK2,
	BUCK3,
	LDO1,
	LDO2,
	LDO3,
	USB_OTG,
	LDO_GPADC,
	LDO_AUDIO,
	LDO_PMCORE,
	LDO_BBAT,
};

struct power_supply_module {
	int command;
	int power_module;
};

struct axp192_platform_data {
	int			(*init_irq)(void);
	int			(*ack_irq)(void);
	void			(*platform_init)(void);
	spinlock_t		lock;
	struct work_struct	work;
	struct power_supply_module *power_supply_modules;
};

struct pmic_ops 
{
	int (*get_voltage)(int cmd, int *pmv);
	int (*set_voltage)(int cmd, int mv);

	int (*is_vbus_assert)(void);
	int (*is_avbusvld )(void);
	int (*is_asessvld )(void);
	int (*is_bsessvld )(void);
	int (*is_srp_ready)(void);

	int (*set_pump)(void);
	int (*set_vbus_supply)(void);
	int (*set_usbotg_a_mask)(void);
	int (*set_usbotg_b_mask)(void);
};

/* General */
extern int axp192_read(u8 reg, u8 *val);
extern int axp192_write(u8 reg, u8 val);
extern int axp192_write_mask(u8 reg, u8 val, u8 mask);
#endif

