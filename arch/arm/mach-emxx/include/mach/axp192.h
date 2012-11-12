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
#define AXP_PWROFF_SHUTDOWN 	0x80
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


//add by smallart

#define   POWER_STATUS                        (0x00)
#define   POWER_MODE_CHGSTATUS                (0x01)
#define   POWER_OTG_STATUS                    (0x02)
#define   POWER_DATA_BUFFER1                  (0x06)
#define   POWER_DATA_BUFFER2                  (0x07)
#define   POWER_DATA_BUFFER3                  (0x08)
#define   POWER_DATA_BUFFER4                  (0x09)
#define   POWER_DATA_BUFFER5                  (0x0A)
#define   POWER_DATA_BUFFER6                  (0x0B)
#define   POWER_VERSION                  			(0x0C)
#define   POWER_LDO3_DC2_CTL                  (0x10)
#define   POWER_LDO24_DC13_CTL                (0x12)
#define   POWER_DC2OUT_VOL                   	(0x23)
#define   POWER_LDO3_DC2_VRC                  (0x25)
#define   POWER_DC1OUT_VOL                   	(0x26)
#define   POWER_DC3OUT_VOL                   	(0x27)
#define   POWER_LDO24OUT_VOL                  (0x28)
#define   POWER_LDO3OUT_VOL                   (0x29)
#define   POWER_IPS_SET                       (0x30)
#define   POWER_VOFF_SET                      (0x31)
#define   POWER_OFF_CTL                       (0x32)
#define   POWER_CHARGE1                       (0x33)
#define   POWER_CHARGE2                       (0x34)
#define   POWER_BACKUP_CHG                    (0x35)
#define   POWER_PEK_SET                       (0x36)
#define   POWER_DCDC_FREQSET                  (0x37)
#define   POWER_VLTF_CHGSET                   (0x38)
#define   POWER_VHTF_CHGSET                   (0x39)
#define   POWER_APS_WARNING1          				(0x3A)
#define   POWER_APS_WARNING2          				(0x3B)
#define   POWER_VLTF_DISCHGSET                (0x3C)
#define   POWER_VHTF_DISCHGSET                (0x3D)
#define   POWER_DCDC_MODESET                  (0x80)
#define   POWER_VOUT_MONITOR                  (0x81)
#define   POWER_ADC_EN1                       (0x82)
#define   POWER_ADC_EN2                       (0x83)
#define   POWER_ADC_SPEED                     (0x84)
#define   POWER_ADC_INPUTRANGE                (0x85)
#define   POWER_TIMER_CTL                     (0x8A)
#define   POWER_VBUS_DET_SRP                  (0x8B)
#define   POWER_HOTOVER_CTL                   (0x8F)
#define   POWER_GPIO0_CTL                     (0x90)
#define   POWER_GPIO0_VOL                     (0x91)
#define   POWER_GPIO1_CTL                     (0x92)
#define   POWER_GPIO2_CTL                     (0x93)
#define   POWER_GPIO_SIGNAL                   (0x94)
#define   POWER_SENSE_CTL                     (0x95)
#define   POWER_SENSE_SIGNAL                  (0x96)
#define   POWER_GPIO20_PDCTL                  (0x97)
#define   POWER_PWM1_FREQ                     (0x98)
#define   POWER_PWM1_DUTYDE                   (0x99)
#define   POWER_PWM1_DUTY                     (0x9A)
#define   POWER_PWM2_FREQ                     (0x9B)
#define   POWER_PWM2_DUTYDE                   (0x9C)
#define   POWER_PWM2_DUTY                     (0x9D)
#define   POWER_RSTO_CTL                      (0x9E)
#define   POWER_GPIO67_CTL                    (0x9F)
#define   POWER_INTEN1                        (0x40)
#define   POWER_INTEN2                        (0x41)
#define   POWER_INTEN3                        (0x42)
#define   POWER_INTEN4                        (0x43)
#define   POWER_INTSTS1                       (0x44)
#define   POWER_INTSTS2                       (0x45)
#define   POWER_INTSTS3                       (0x46)
#define   POWER_INTSTS4                       (0x47)
#define   POWER_COULOMB_CTL                   (0xB8)

//adc data register
#define   POWER_ACIN_VOL_H8                   (0x56)
#define   POWER_ACIN_VOL_L4                   (0x57)
#define   POWER_ACIN_CUR_H8                   (0x58)
#define   POWER_ACIN_CUR_L4                   (0x59)
#define   POWER_VBUS_VOL_H8             			(0x5A)
#define   POWER_VBUS_VOL_L4             			(0x5B)
#define   POWER_VBUS_CUR_H8                   (0x5C)
#define   POWER_VBUS_CUR_L4                   (0x5D)
#define   POWER_INT_TEMP_H8                   (0x5E)
#define   POWER_INT_TEMP_L4                   (0x5F)
#define   POWER_TS_VOL_H8                     (0x62)
#define   POWER_TS_VOL_L4                     (0x63)
#define   POWER_GPIO0_VOL_H8             			(0x64)
#define   POWER_GPIO0_VOL_L4             			(0x65)
#define   POWER_GPIO1_VOL_H8             			(0x66)
#define   POWER_GPIO1_VOL_L4             			(0x67)
#define   POWER_GPIO2_VOL_H8             			(0x68)
#define   POWER_GPIO2_VOL_L4             			(0x69)
#define   POWER_BATSENSE_VOL_H8             	(0x6A)
#define   POWER_BATSENSE_VOL_L4             	(0x6B)

#define   POWER_BAT_AVERVOL_H8                   (0x78)
#define   POWER_BAT_AVERVOL_L4                   (0x79)
#define   POWER_BAT_AVERCHGCUR_H8                (0x7A)
#define   POWER_BAT_AVERCHGCUR_L5                (0x7B)
#define   POWER_BAT_AVERDISCHGCUR_H8             (0x7C)
#define   POWER_BAT_AVERDISCHGCUR_L5             (0x7D)
#define   POWER_APS_AVERVOL_H8                   (0x7E)
#define   POWER_APS_AVERVOL_L4                   (0x7F)
#define   POWER_BAT_CHGCOULOMB3                  (0xB0)
#define   POWER_BAT_CHGCOULOMB2                  (0xB1)
#define   POWER_BAT_CHGCOULOMB1                  (0xB2)
#define   POWER_BAT_CHGCOULOMB0                  (0xB3)
#define   POWER_BAT_DISCHGCOULOMB3               (0xB4)
#define   POWER_BAT_DISCHGCOULOMB2               (0xB5)
#define   POWER_BAT_DISCHGCOULOMB1               (0xB6)
#define   POWER_BAT_DISCHGCOULOMB0               (0xB7)
#define   POWER_BAT_POWERH8					             (0x70)
#define   POWER_BAT_POWERM8                 		 (0x71)
#define   POWER_BAT_POWERL8                  		 (0x72)     


#define  DVS_DELAY 100				//dvs Delay time,uS,should be more than 500
#define  ABS(x)     ( (x) >0 ? (x) : -(x) )

#define  CHG_END_CTL         1          //Alow time to end charge
#define  TIME_BUFFER_LONG    20
//#define  BAT_CAP             890	
#define  BAT_CAP             4100	

#define  RDC_COUNT           5
//#define  RDC_DEFAULT         250
#define  RDC_DEFAULT         220

#define  END_VOLTAGE_APS     3300
#define  END_VOLTAGE_BAT     0
#define FUELGUAGE_LOW_VOL    3400		//<3.4v,2%
#define FUELGUAGE_VOL1  	 3500		//<3.5v,3%
#define FUELGUAGE_VOL2  	 3600
#define FUELGUAGE_VOL3  	 3700
#define FUELGUAGE_VOL4  	 3800
#define FUELGUAGE_VOL5  	 3900
#define FUELGUAGE_VOL6  	 4000
#define FUELGUAGE_VOL7  	 4100
#define FUELGUAGE_TOP_VOL    4150				//>4.16v,100%	



#define FUELGUAGE_LOW_VOL    3400		//<3.4v,2%
#define FUELGUAGE_VOL1  	 3500		//<3.5v,3%
#define FUELGUAGE_VOL2  	 3600
#define FUELGUAGE_VOL3  	 3700
#define FUELGUAGE_VOL4  	 3800
#define FUELGUAGE_VOL5  	 3900
#define FUELGUAGE_VOL6  	 4000
#define FUELGUAGE_VOL7  	 4100
#define FUELGUAGE_TOP_VOL    4150				//>4.16v,100%	
	
#define FUELGUAGE_LOW_LEVEL  1			//<3.4v,2%
#define FUELGUAGE_LEVEL1  	 1			//<3.5v,3%
#define FUELGUAGE_LEVEL2  	 4
#define FUELGUAGE_LEVEL3  	 24
#define FUELGUAGE_LEVEL4  	 55
#define FUELGUAGE_LEVEL5  	 71
#define FUELGUAGE_LEVEL6  	 86
#define FUELGUAGE_LEVEL7  	 96
#define FUELGUAGE_TOP_LEVEL  100		//>4.15v,97%	

#define  LEVEL1      1
#define  LEVEL2      2
#define  LEVEL3      3
#define  LEVEL4      4
#define  LEVEL5      5
#define  LEVEL6      6
#define  LEVEL7      7
#define  LEVEL8      8
#define  LEVEL9      9
#define  LEVEL10     10
#define  LEVEL11     11
#define  LEVEL12     12
#define  LEVEL13     13
#define  LEVEL14     14
#define  LEVEL15     15
#define  LEVEL16     16
#define  LEVEL17     17
#define  LEVEL18     18
#define  LEVEL19     19
#define  LEVEL20     20 
#define  LEVEL21     21
#define  LEVEL22     22
#define  LEVEL23     23
#define  LEVEL24     24
#define  LEVEL25     25
#define  LEVEL26     26
#define  LEVEL27     27
#define  LEVEL28     28
#define  LEVEL29     29
#define  LEVEL30     30
#define  LEVEL31     31
#define  LEVEL32     32




#define BAT_AVER_VOL         3820		//Aver Vol:3.82V
#define TEMP_COMPENSATION    0

#define  uint8   unsigned char
#define  uint16  unsigned short
#define  int16   short
#define  uint32  unsigned int
#define  int32   int
#define  uint64  unsigned long
#define  I2C_WRITEOK    1101

#define	 RADDR          0x69
#define	 WADDR          0x68
#define  VRC_LDO3       1101
#define  VRC_DCDC2		1102

#define  ACIN_VOL       1001
#define  ACIN_CUR		1002
#define  VBUS_VOL		1003
#define  VBUS_CUR		1004
#define  BAT_VOL		1005
#define  BAT_CUR		1006
#define  APS_VOL		1007
#define  TS_VOL			1008
#define  GPIO0_VOL	    1009
#define  GPIO1_VOL	    1010
#define  GPIO2_VOL	    1011
#define  BATSENSE_VOL   1012
#define  INT_TEMP		1013
#define  CHARGE_ON		0x80
#define  CHARGE_OFF     0x40
#define  EXT_CHARGE_ON	0x20
#define  EXT_CHARGE_OFF	0x10
#define  ADC_BATSENSE   1020
#define  ADC_GPIO2      1021
#define  ADC_GPIO1      1022
#define  ADC_GPIO0      1023
#define  LTF_CHARGE     1030
#define  HTF_CHARGE     1031
#define  LTF_DISCHARGE  1032
#define  HTF_DISCHARGE  1033

#define POWER_INPUT_STATUS    1034
#define POWER_CHARGEING_MODE      1044

typedef struct _DRV_POWER_CHARGE_STATUS
{
	uint8 acin_valid;
	uint8 vbus_valid;
	uint8 ext_power_valid;
	uint8 bat_current_direction;    //电池电流方向,0-放电,1-充电
	uint8 in_Short;									//1-short
	uint8 int_over_temp;    //  1 过温， 0 不过温
	uint8 charge_indication;
	uint8 battery_exist;
	uint8 battery_active;
	uint8 low_charge_current;
	uint16 battery_current;
}_drv_power_charge_stat;

typedef struct _DRV_POWER_CHARGE_PARA
{
	uint16 charge_onoff;
	uint16 target_vol;
	uint8 end_charge_rate;
	uint16 int_cur_level;
	uint16 ext_cur_level;
	uint8 prechg_time_level;
	uint8 cchg_time_level;
}_drv_power_charge_para;

typedef struct _DRV_POWER_FUELGAUGE
{
	uint8  charge_status;			//111:charge,110:charge finished,1xx:no Bat
	uint16 rest_cap;
	uint16 rest_time;
	uint32 bat_cap;
	uint8  end_voltage_rate;
}_drv_power_fuelgauge;


/************************************************************************/

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
extern int axp192_read(uint8 reg, uint8 *val);
extern int axp192_write(uint8 reg, uint8 val);
extern int axp192_write_mask(uint8 reg, uint8 val, uint8 mask);
#endif

