/*******************Download EEP CRC Define*****************/
//#define EEPCRC_OPEN

/**********************Solution Choose**********************/
//#define ATMEL_168
//#define R8C_3GA_2TG
//#define R8C_AUO_I2C
//#define	M48_F32
#define M48


#ifdef ATMEL_168
	#define	MAXX	32
	#define	MAXY	32
#endif

#ifdef R8C_3GA_2TG
	#define	MAXX	32
	#define	MAXY	32
#endif

#ifdef R8C_AUO_I2C
  	#ifndef R8C_3GA_2TG
  	#define R8C_3GA_2TG
  	#endif
	#define	MAXX	32
	#define	MAXY	32
#endif

#ifdef	M48
	#define	MAXX	48
	#define	MAXY	48
	#define POWERON_CAL //for power on calibration
#endif

/*********************************V2.5-hang-110714-TOP****************************************/

#ifdef M48_F32
	#define MAXX    48
	#define MAXY    32
#endif
/*********************************V2.5-hang-110714-BOTTOM****************************************/

#ifdef R8C_AUO_I2C
  #ifndef R8C_3GA_2TG
  #define R8C_3GA_2TG
  #endif
#endif

/*****************touchscreen resolution setting*************/
#define TOUCHSCREEN_MINX 0
#define TOUCHSCREEN_MAXX 1024
#define TOUCHSCREEN_MINY 0
#define TOUCHSCREEN_MAXY 768

/*****************board setting and test passed*************/
//#define	S5PC1XX
#define	EMEV

#ifdef S5PC1XX
	#include <plat/gpio-bank-e1.h> //reset pin GPE1_5
	#include <plat/gpio-bank-h1.h> //attb pin GPH1_3
	#include <mach/gpio.h>
	#include <plat/gpio-cfg.h>

	#define ATTB		S5PC1XX_GPH1(3)
	#define get_attb_value	gpio_get_value
	#define	RESETPIN_CFG	s3c_gpio_cfgpin(S5PC1XX_GPE1(5),S3C_GPIO_OUTPUT)
	#define	RESETPIN_SET0 	gpio_direction_output(S5PC1XX_GPE1(5),0)
	#define	RESETPIN_SET1	gpio_direction_output(S5PC1XX_GPE1(5),1)
#endif
#ifdef MINI6410
	#include <plat/gpio-cfg.h>
	#include <mach/gpio-bank-e.h>
	#include <mach/gpio-bank-n.h>
	#include <mach/gpio.h>

	#define ATTB		S3C64XX_GPN(11)
	#define get_attb_value	gpio_get_value
	#define	RESETPIN_CFG	s3c_gpio_cfgpin(S3C64XX_GPE(1),S3C_GPIO_OUTPUT)
	#define	RESETPIN_SET0 	gpio_direction_output(S3C64XX_GPE(1),0)
	#define	RESETPIN_SET1	gpio_direction_output(S3C64XX_GPE(1),1)
#endif
#ifdef EMEV
	#include <mach/gpio.h>
	#include <mach/smu.h>
        #include <mach/emev_board.h>

	#define ATTB		(29)
	#define get_attb_value	gpio_get_value
	#define	RESETPIN_CFG	emxx_setintpin()
	#define	RESETPIN_SET0 	gpio_direction_output(97,0)
	#define	RESETPIN_SET1	gpio_direction_output(97,1)
#endif


