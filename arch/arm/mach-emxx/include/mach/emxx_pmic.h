/****************************************************************
 * arch/arm/mach-emxx/include/mach/axp192.h
 *
 * Copyright (C) 2010, Renesas Electornics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 ****************************************************************/
#ifndef EMXX_PMIC_H__
#define EMXX_PMIC_H__
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

#if 0
#define BUCK1			0
#define BUCK2			1
#define BUCK3			2
#define LDO1			3
#define LDO2			4
#define LDO3			5
#define USB_OTG			6
#define LDO_GPADC		7
#define LDO_AUDIO		8
#define LDO_PMCORE		9
#define LDO_BBAT		10
#endif


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
#endif /* EMXX_PMIC_H__*/

