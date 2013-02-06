/*
   File Name		: sound/arm/emxx-mixer.h
 *  Function		: ALC5621 CODEC
 *  Release Version 	: Ver 1.00
 *  Release Date	: 2011/09/22
 *
 *  Copyright (C) NEC Electronics Corporation 2010
 *
 *  This program is free software;you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by Free
 *  Softwere Foundation; either version 2 of License, or (at your option) any
 *  later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY;
 *  without even the implied warrnty of MERCHANTABILITY or FITNESS FOR A
 *  PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program;
 *  If not, write to the Free Software Foundation, Inc., 59 Temple Place -
 *  Suite 330, Boston,
 *  MA 02111-1307, USA.
 *
 */

#include <linux/version.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <mach/pwc.h>
#include <mach/smu.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/initval.h>

#ifdef CONFIG_PM
#include <linux/pm.h>
#endif

#include "emxx-mixer.h"
#include "emxx-pcm.h"
#define REALTEK_CODEC 1
#ifdef REALTEK_CODEC
#include "rt5621.h"
#endif

#define XYP_DEBUG_DEVICE 1

/* static initial value */
#define ID_VALUE		NULL
#define I2C_CODEC_CLIENT_VALUE	NULL;

#ifdef AUDIO_MAKING_DEBUG
#include <linux/moduleparam.h>
#endif

/*** debug code by the making ->*/
#ifdef AUDIO_MAKING_DEBUG
#define FNC_ENTRY	\
	if (debug == 1 || debug >= 9) {	\
		printk(KERN_INFO "entry:%s\n", __FUNCTION__); \
	}
#define FNC_EXIT	\
	if (debug == 1 || debug >= 9) {	\
		printk(KERN_INFO "exit:%s:%d\n", __FUNCTION__ , __LINE__); \
	}
#define d0b(fmt, args...)	\
	printk(KERN_INFO "%s:%d: " fmt , __FUNCTION__ , __LINE__ , ## args);
#define d1b(fmt, args...)	\
	if (debug == 1 || debug >= 9) {	\
		printk(KERN_INFO "%s:%d: " fmt , __FUNCTION__ , __LINE__ , ## args); \
	}
#define d7b(fmt, args...)	\
	if (debug == 7 || debug >= 9) {	\
		printk(KERN_INFO "%s:%d: " fmt , __FUNCTION__ , __LINE__ , ## args); \
	}
#define d8b(fmt, args...)	\
	if (debug == 8 || debug >= 9) {	\
		printk(KERN_INFO "%s:%d: " fmt , __FUNCTION__ , __LINE__ , ## args); \
	}
#define d9b(fmt, args...)	\
	if (debug == 9 || debug >= 9) {	\
		printk(KERN_INFO fmt , ## args); \
	}
#else
#define FNC_ENTRY
#define FNC_EXIT
#define d0b(fmt, args...)
#define d1b(fmt, args...)
#define d7b(fmt, args...)
#define d8b(fmt, args...)
#define d9b(fmt, args...)
#endif
/*<- debug code by the making ***/

#ifdef REALTEK_CODEC
#define RT5621_MAX_REG RT5621_VENDOR_ID2
#define CODEC_WRITE(a, d)    i2c_codec_write_mask(a, d, 0xffff)
#define CODEC_WRITE_M(a, d, m)    i2c_codec_write_mask(a, d, m)
#define CODEC_READ(a)    i2c_codec_read(a)

//Init registers setting
struct rt5621_reg{
	unsigned char reg_index;
	unsigned short reg_value;
};

static struct rt5621_reg init_data[] = {
	{RT5621_ADC_REC_MIXER,		0x3F3F},			//default record source
	{RT5621_ADC_REC_GAIN,		0xFC18},			//ADC record gain: left channel: +19.5dB, right channel: +19.5dB
	{RT5621_STEREO_DAC_VOL, 	0x6000},			//DAC->HP Mixer, DAC volume:+0dB
	{RT5621_AUXIN_VOL,		0x6000},			//AUXIN->HP Mixer, AUXIN volume:+0dB
	{RT5621_MIC_ROUTING_CTRL,	0xD0D0},			//MIC1->MONO Mixer, MIC1:differential mode; MIC2->MONO Mixer, MIC2:differential mode
	{RT5621_ADD_CTRL_REG,		0x5300},			//AUXOUT: differential mode
	{RT5621_OUTPUT_MIXER_CTRL,	0x87C0},			//LN->SPK_OUT_N, SPK_OUT type:calss-AB, HP mixer->SPK_OUT, HP mixer->HP_OUT, MONO mixer->AUX_OUT.
//	{RT5621_HID_CTRL_INDEX,		0x0046},			//Class-D setting
//	{RT5621_HID_CTRL_DATA,		0xCF00},			//power on Class-D Internal register
};
#define RT5621_INIT_REG_NUM ARRAY_SIZE(init_data)
#endif

#define TYPE_VOL        0x01
#define TYPE_SW         0x02
#define TYPE_BL         0x04

#define IDX(a)  (a & 0xffff)

/* VOL(integer) */
enum {
	MIXER_VOL_DAC = TYPE_VOL << 16 | 0x0000,
	MIXER_VOL_HP,
	MIXER_VOL_SPK,
	MIXER_VOL_MIC1,
	MIXER_VOL_MIC2,
	MIXER_VOL_AUXIN,
	MIXER_VOL_AUXOUT,

	MIXER_INT_NUM = IDX(MIXER_VOL_AUXOUT) + 1,
};

/* SW (enum) */
enum {
	MIXER_SW_CAPTURE = TYPE_SW << 16 | 0x0000,
	MIXER_SW_SAMPLING_RATE,
	MIXER_SW_PLAYBACK,

	MIXER_ENUM_NUM = IDX(MIXER_SW_PLAYBACK) + 1,
};

/* SW (boolean) */
enum {
	MIXER_SW_CODEC_POWER_BL = TYPE_BL << 16 | 0x0000,

	MIXER_BL_NUM = IDX(MIXER_SW_CODEC_POWER_BL) + 1,
};

static const char *in_sw_control_texts[] = {
	"OFF", "MIC_normal", "MIC_ringtone", "MIC_incall", "Headset_normal", "Headset_ringtone", "Headset_incall"
};

static const char *fs_sw_control_texts[] = {
	"7.35kHz", "8kHz", "11.025kHz", "12kHz", "14.7kHz", "16kHz",
	"22.05kHz", "24kHz", "29.4kHz", "32kHz", "44.1kHz", "48kHz"
};

static const char *out_sw_control_texts[] = {
	"OFF", "Speaker_normal", "Speaker_ringtone", "Speaker_incall", "Earpiece_ringtone", "Earpiece_incall", "Headset_normal", "Headset_ringtone", "Headset_incall"
};

struct integer_info {
	unsigned int count;     /* count of values */
	long val_int_min;       /* R: minimum value */
	long val_int_max;       /* R: maximum value */
	long val_int_step;      /* R: step (0 variable) */
	int value[2];
};

struct enum_info {
	char **texts;
	unsigned int items;
	int value;
};

struct boolean_info {
	int value;
};

struct emxx_codec_mixer {
	struct snd_card *card;
	spinlock_t mixer_lock;
	struct integer_info *vol_info;
	struct enum_info *enum_info;
	struct boolean_info *bl_info;
	int power_on;
	struct mutex power_mutex;
};

static struct integer_info volume_info[MIXER_INT_NUM] = {
	{ /* MIXER_VOL_DAC */
		.count = 2,
		.val_int_min = 0,
		.val_int_max = 0x1f,
		.val_int_step = 1,
	},
	{ /* MIXER_VOL_HP */
		.count = 2,
		.val_int_min = 0,
		.val_int_max = 0x1f,
		.val_int_step = 1,
	},
	{ /* MIXER_VOL_SPK */
		.count = 2,
		.val_int_min = 0,
		.val_int_max = 0x1f,
		.val_int_step = 1,
	},
	{ /* MIXER_VOL_MIC1 */
		.count = 2,
		.val_int_min = 0,
		.val_int_max = 0x1f,
		.val_int_step = 1,
	},
	{ /* MIXER_VOL_MIC2 */
		.count = 2,
		.val_int_min = 0,
		.val_int_max = 0x1f,
		.val_int_step = 1,
	},
	{ /* MIXER_VOL_AUXIN*/
		.count = 2,
		.val_int_min = 0,
		.val_int_max = 0x1f,
		.val_int_step = 1,
	},
	{ /* MIXER_VOL_AUXOUT*/
		.count = 2,
		.val_int_min = 0,
		.val_int_max = 0x1f,
		.val_int_step = 1,
	},
};

#define NUM_OF(v) (sizeof(v) / sizeof(v[0]))
static struct enum_info enum_info[MIXER_ENUM_NUM] = {
	{ /* MIXER_SW_CAPTURE */
		.texts = (char **)in_sw_control_texts,
		.items = NUM_OF(in_sw_control_texts),
	},
	{ /* MIXER_SW_SAMPLING_RATE */
		.texts = (char **)fs_sw_control_texts,
		.items = NUM_OF(fs_sw_control_texts),
	},
	{ /* MIXER_SW_PLAYBACK */
		.texts = (char **)out_sw_control_texts,
		.items = NUM_OF(out_sw_control_texts),
	},
};

static struct boolean_info boolean_info[MIXER_BL_NUM];

static struct emxx_codec_mixer codec_mixer = {
	.vol_info = &volume_info[0],
	.enum_info = &enum_info[0],
	.bl_info = &boolean_info[0],
	.power_on = 0, /* off */
};

static pcm_ctrl_t pcm_sett = {
	.func = {
		.mode_sel       = PCM_MODE_3,
		.m_s            = PCM_SLAVE_MODE,
		.tx_tim         = PCM_TX_04_WORD,
	},
	.cyc = {
		.cyc_val        = 0x1f,
		.sib            = 0x0f,
		.rx_pd          = PCM_PADDING_ON,
		.sob            = 0x0f,
		.tx_pd          = PCM_PADDING_ON,
	},
	.cyc2 = {
		.cyc_val2       = 0x1f,
		.sib2           = 0x1f,
		.sob2           = 0x1f,
	},
};

struct _coeff_div {
        u32 rate;
        u16 clk_regvalue;
        u16 pll_regvalue;
};

static struct _coeff_div coeff_div[] = {
        /* 8k */
        {8000, 0x166d, 0x2c7d},

        /* 11.025k */
        {11025, 0x366d, 0x2323},

        /* 16k */
        {16000, 0x366d, 0x452b},

        /* 22.05k */
        {22050, 0x266d, 0xfc7d},

        /* 32k */
        {32000, 0x166d, 0x452b},

        /* 44.1k */
        {44100, 0x166d, 0x2323},
};

static int get_coeff(int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate)
			return i;
	}
	return -EINVAL;
}

static struct i2c_client *i2c_codec_client = I2C_CODEC_CLIENT_VALUE;

#ifdef REALTEK_CODEC
/* Realtek ALC5621 i2c read/write function */
static int i2c_codec_read(unsigned int reg)
{
	int res = 0, value;
	unsigned char buf[2] = {0};

	if (i2c_codec_client == NULL) {
		printk(KERN_ERR "i2c codec not available!\n");
		return -EIO;
	}

	if (RT5621_MAX_REG  < reg) {
		printk(KERN_ERR "i2c codec read check failed!\n");
		return -EINVAL;
	}

	buf[0] = reg;
	res = i2c_master_send(i2c_codec_client, buf, 1);
	if (res <= 0) {
		printk(KERN_ERR "i2c codec send failed!\n");
		return -EIO;
	}

	res = i2c_master_recv(i2c_codec_client, buf, 2);
	if (res > 0) {
		value = (buf[0]<<8) | buf[1];
		return value;
	} else {
		printk(KERN_ERR "i2c codec recv failed!\n");
		return -EIO;
	}
}

static int i2c_codec_write(unsigned int reg, unsigned int data)
{
	int res = 0;
	unsigned char buf[3];

	if (i2c_codec_client == NULL) {
		printk(KERN_ERR "i2c codec not available!\n");
		return -EIO;
	}

	if (RT5621_MAX_REG < reg) {
		printk(KERN_ERR "i2c codec write check failed!\n");
		return -EINVAL;
	}

	buf[0] = reg;
	buf[1] = (0xFF00 & data) >> 8;
	buf[2] = 0x00FF & data;

	res = i2c_master_send(i2c_codec_client, buf, 3);
	if (res > 0) {
		res = 0;
	} else {
		printk(KERN_ERR "i2c codec write failed!res=%d\n", res);
	}

	return res;
}

static int i2c_codec_write_mask(unsigned int reg, unsigned int data, unsigned int mask)
{
	int ret=0;
	unsigned  int wdata;

	if(!mask)
		return 0; 

	if(mask != 0xffff) {
		wdata = ((i2c_codec_read(reg) & ~mask) | (data & mask));		
		ret = i2c_codec_write(reg, wdata);
	} else {
		ret = i2c_codec_write(reg, data);
	}

	return ret;
}


#if 0
enum{
	NORMAL = 0,
	CLUB,
	DANCE,
	LIVE,	
	POP,
	ROCK,
	OPPO,
	TREBLE,
	BASS,
	RECORD,	
	HFREQ,	
};

typedef struct  _HW_EQ_PRESET {
	u16 HwEqType;
	u16 EqValue[14];
	u16 HwEQCtrl;
	u16 HwEQModeCtrl;
}HW_EQ_PRESET;

HW_EQ_PRESET HwEq_Preset[] = {
		/*0x0    0x1    0x2    0x3    0x4    0x5    0x6    0x7    0x8    0x9    0xa    0xb    0xc      0x62    0x66*/    
	{NORMAL, {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000}, 0x0000, 0x1f},
	{CLUB  , {0x1C10,0x0000,0xC1CC,0x1E5D,0x0699,0xCD48,0x188D,0x0699,0xC3B6,0x1CD0,0x0699,0x0436,0x0000}, 0x800E, 0x1f},
	{DANCE , {0x1F2C,0x095B,0xC071,0x1F95,0x0616,0xC96E,0x1B11,0xFC91,0xDCF2,0x1194,0xFAF2,0x0436,0x0000}, 0x800F, 0x1f},
	{LIVE  , {0x1EB5,0xFCB6,0xC24A,0x1DF8,0x0E7C,0xC883,0x1C10,0x0699,0xDA41,0x1561,0x0295,0x0436,0x0000}, 0x800F, 0x1f},
	{POP   , {0x1EB5,0xFCB6,0xC1D4,0x1E5D,0x0E23,0xD92E,0x16E6,0xFCB6,0x0000,0x0969,0xF988,0x0436,0x0000}, 0x800F, 0x1f},
	{ROCK  , {0x1EB5,0xFCB6,0xC071,0x1F95,0x0424,0xC30A,0x1D27,0xF900,0x0C5D,0x0FC7,0x0E23,0x0436,0x0000}, 0x800F, 0x1f},
	{OPPO  , {0x0000,0x0000,0xCA4A,0x17F8,0x0FEC,0xCA4A,0x17F8,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000}, 0x800F, 0x1f},
	{TREBLE, {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x188D,0x1699}, 0x8010, 0x1f},
	{BASS  , {0x1A43,0x0C00,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000}, 0x8001, 0x1f},
	{RECORD, {0x1E3C,0xF405,0xC1E0,0x1E39,0x2298,0xDF29,0x0701,0x1D18,0xF34B,0x0CA9,0xF805,0xF9CC,0xF405}, 0x881F, 0x1f},
	{HFREQ , {0x1BBC,0x0000,0xC9A4,0x1BBC,0x0000,0x2997,0x142D,0xFCB6,0xEF01,0x1BBC,0x0000,0xE835,0x0FEC}, 0x8014, 0x1f},
//	{HFREQ , {0x17F8,0x0000,0x18C3,0xFD8C,0xF405,0x07C0,0x17F8,0x0000,0x2EE8,0x17F8,0x0000,0xE835,0x0000}, 0x8002, 0x02},
};

#define rt5621_write_index_reg(addr, data) \
{ \
	i2c_codec_write(RT5621_HID_CTRL_INDEX, addr); \
	i2c_codec_write(RT5621_HID_CTRL_DATA, data); \
}

static void rt5621_update_eqmode(int mode)
{
	u16 HwEqIndex = 0;

	if(mode == NORMAL) {
		//clear EQ parameter
		for(HwEqIndex = 0; HwEqIndex <= 0x0C; HwEqIndex++){
			rt5621_write_index_reg(HwEqIndex, HwEq_Preset[mode].EqValue[HwEqIndex])
		}
		i2c_codec_write(RT5621_EQ_CTRL, 0x0);//disable EQ block

	} else {
		i2c_codec_write(RT5621_EQ_CTRL, HwEq_Preset[mode].HwEQCtrl);
		//Fill EQ parameter
		for(HwEqIndex = 0; HwEqIndex <= 0x0C; HwEqIndex++){
			rt5621_write_index_reg(HwEqIndex, HwEq_Preset[mode].EqValue[HwEqIndex]) 
		}		
		//update EQ parameter
		i2c_codec_write(RT5621_EQ_MODE_ENABLE, HwEq_Preset[mode].HwEQModeCtrl);

		schedule_timeout_uninterruptible(msecs_to_jiffies(1));
		i2c_codec_write(RT5621_EQ_MODE_ENABLE, 0x0);
	}
}
#endif


#ifdef XYP_DEBUG_DEVICE
static void rt5621_dumpRegs(void)
{
	int i;

	printk("ALC5621 registers\n");
	for (i = 0; i <= 0x7E; i += 2)
		printk("reg[0x%02x] = 0x%04x\n", i, CODEC_READ(i));
}
#endif

void hp_depop_mode2(void)
{
	CODEC_WRITE_M(RT5621_PWR_MANAG_ADD3, 0x8000, 0x8000);
	CODEC_WRITE_M(RT5621_HP_OUT_VOL, 0x8080, 0x8080);
	CODEC_WRITE_M(RT5621_PWR_MANAG_ADD1, 0x0100, 0x0100);
	CODEC_WRITE_M(RT5621_PWR_MANAG_ADD2, 0x2000, 0x2000);
	CODEC_WRITE_M(RT5621_PWR_MANAG_ADD3, 0x0600, 0x0600);
	CODEC_WRITE_M(RT5621_MISC_CTRL, 0x0200, 0x0200);//enable Depop Mode 2 of HP_OUT

	schedule_timeout_uninterruptible(msecs_to_jiffies(300));
}
#endif


#ifdef XYP_DEBUG_DEVICE
ssize_t reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	rt5621_dumpRegs();

	return 0;
}
#endif

static int vol = 30; 
static ssize_t incall_vol_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	int direction;

        if (sscanf(buf, "%d", &direction) == 1) {
		if (direction == 1) {
			if ((vol >= 6) && (vol <= 30)) {
				vol -= 6;
				CODEC_WRITE_M(RT5621_HP_OUT_VOL, VOL_L_GAIN(vol), VOL_L_GAIN_MASK);
				CODEC_WRITE_M(RT5621_HP_OUT_VOL, VOL_R_GAIN(vol), VOL_R_GAIN_MASK);
				CODEC_WRITE_M(RT5621_SPK_OUT_VOL, VOL_L_GAIN(vol), VOL_L_GAIN_MASK);
				CODEC_WRITE_M(RT5621_SPK_OUT_VOL, VOL_R_GAIN(vol), VOL_R_GAIN_MASK);
#ifdef XYP_DEBUG_DEVICE
				printk("==== vol+ ,current volume: +%d dB\n", vol*3/2);
#endif
			}

		} else if (direction == -1) {
			if ((vol >= 0) && (vol <= 24)) {
				vol += 6;
				CODEC_WRITE_M(RT5621_HP_OUT_VOL, VOL_L_GAIN(vol), VOL_L_GAIN_MASK);
				CODEC_WRITE_M(RT5621_SPK_OUT_VOL, VOL_L_GAIN(vol), VOL_L_GAIN_MASK);
				CODEC_WRITE_M(RT5621_HP_OUT_VOL, VOL_R_GAIN(vol), VOL_R_GAIN_MASK);
				CODEC_WRITE_M(RT5621_SPK_OUT_VOL, VOL_R_GAIN(vol), VOL_R_GAIN_MASK);
#ifdef XYP_DEBUG_DEVICE
				printk("==== vol- , current volume: +%d dB\n", vol*3/2);
#endif

			}

		} else {
			printk("invalid command!\n");
		}
	}

        return len;
}

#ifdef XYP_DEBUG_DEVICE
static DEVICE_ATTR(incall_vol, 0777, reg_show, incall_vol_store);
#else
static DEVICE_ATTR(incall_vol, 0777, NULL, incall_vol_store);
#endif
static struct class *codec_class;
static struct device *codec_dev;

static int setup_codec_dev(void)
{
	codec_class = class_create(THIS_MODULE, "codec_class");
	if (codec_class == NULL) {
		printk("class_create() failed\n");
		return -1;
	}

	codec_dev = device_create(codec_class, NULL, MKDEV(0, 1), NULL, "codec_dev");	
	if (codec_dev == NULL) {
		printk("device_create() failed\n");
		return -2;
	}

	return device_create_file(codec_dev, &dev_attr_incall_vol);
}

static void destory_codec_dev(void)
{
	device_remove_file(codec_dev, &dev_attr_incall_vol);
	device_destroy(codec_class, MKDEV(0, 1));
	class_destroy(codec_class);
}

static int codec_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
printk("===========%s %d:", __func__, __LINE__);
	i2c_codec_client = client;
	setup_codec_dev();

	return 0;
}

static int codec_i2c_remove(struct i2c_client *client)
{
	i2c_codec_client = NULL;
	destory_codec_dev();

	return 0;
}

static struct i2c_device_id codec_i2c_idtable[] = {
	{ "ALC5621", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, codec_i2c_idtable);

static struct i2c_driver i2c_codec_driver = {
	.driver.name    = "i2c for codec",
//	.id             = I2C_DRIVERID_I2CDEV, /* Fake ID */
	.id_table       = codec_i2c_idtable,
	.probe          = codec_i2c_probe,
	.remove         = codec_i2c_remove,
};

static int i2c_codec_init(void)
{
	int res = 0;

	res = i2c_add_driver(&i2c_codec_driver);
	if (res == 0) {
		if (i2c_codec_client == NULL) {
			i2c_del_driver(&i2c_codec_driver);
			printk(KERN_ERR "codec_i2c_found_proc() not called!\n");
			return -EIO;
		}
		if (res != 0) {
			i2c_del_driver(&i2c_codec_driver);
			printk(KERN_ERR "send or recv failed!\n");
		}
	} else {
		printk(KERN_ERR "i2c codec inserted failed!\n");
	}

	return res;
}


static int codec_init(void)
{
	struct emxx_codec_mixer *codec = &codec_mixer;
	int i, j;
	FNC_ENTRY

	/* volume(integer) */
	for (i = 0; i < MIXER_INT_NUM; i++) {
		for (j = 0; j < (codec->vol_info[IDX(i)].count); j++) {
			codec->vol_info[IDX(i)].value[j] = 2;
		}
	}

	/* switch control(enum) */
	/* capture input path: OFF */
	codec->enum_info[IDX(MIXER_SW_CAPTURE)].value = 0;
	/* 8kHz
	codec->enum_info[IDX(MIXER_SW_SAMPLING_RATE)].value = 1;*/
	/* 44.1kHz */
	codec->enum_info[IDX(MIXER_SW_SAMPLING_RATE)].value = 10;
	/* playback output path : OFF */
	codec->enum_info[IDX(MIXER_SW_PLAYBACK)].value = 0;

	/* switch control(boolean) */
	/* "OFF" */
	codec->bl_info[IDX(MIXER_SW_CODEC_POWER_BL)].value = 0;

	FNC_EXIT return 0;
}

#ifdef REALTEK_CODEC
static int rt5621_ChangeCodecPowerStatus(int power_state);
#endif

#ifdef CONFIG_EMXX_ANDROID
int codec_power_on(void)
#else
static int codec_power_on(void)
#endif
{
	struct emxx_codec_mixer *codec = &codec_mixer;
	int res = 0;
	int power_value = 0;
#ifdef REALTEK_CODEC
	int i = 0;
#endif

	mutex_lock(&codec->power_mutex);
	if (codec->power_on) {
		mutex_unlock(&codec->power_mutex);
		return 0;
	}

	power_value = codec->bl_info[IDX(MIXER_SW_CODEC_POWER_BL)].value;
	codec_init();
	codec->bl_info[IDX(MIXER_SW_CODEC_POWER_BL)].value = power_value;

	/* codec PDN pin "L" -> "H" */
	//gpio_set_value(GPIO_AUDIO_RST, 1);
	/* PDN pin "L" needs 150ns */
	outl(SMU_PLLSEL_OSC1 | SMU_DIV(2), SMU_REFCLKDIV);
	emxx_open_clockgate(EMXX_CLK_REF);

	res = i2c_codec_init();
	if (res != 0) {
		printk( "codec init error\n");
		goto err1;
	}

	/* Clock Set Up Sequence */
#ifdef REALTEK_CODEC
	//reset
	res = CODEC_WRITE(0x0, 0);
	if (res != 0)
		goto err1;

	hp_depop_mode2();

	for (i = 0; i < RT5621_INIT_REG_NUM; i++) {
		res = CODEC_WRITE(init_data[i].reg_index, init_data[i].reg_value);
		if (res < 0) {
			goto err1;
		}
	}

#if 0
	rt5621_write_index_reg(EQ_INPUT_VOL_CONTROL, EQ_INPUT_VOL_MINUS_6DB);
	rt5621_write_index_reg(EQ_OUTPUT_VOL_CONTROL, EQ_OUTPUT_VOL_ADD_6DB);
	rt5621_update_eqmode(HFREQ);
#endif
	res = CODEC_WRITE(RT5621_PLL_CTRL, 0x2323);	/* 44.1KHz */
	/* res = CODEC_WRITE(RT5621_PLL_CTRL, 0xfc7d);	/* 22.05KHz */
	if (res != 0) {
		goto err1;
	}

	res = CODEC_WRITE_M(RT5621_GLOBAL_CLK_CTRL_REG, SYSCLK_SOUR_SEL_PLL, SYSCLK_SOUR_SEL_MASK);
	if (res != 0) {
		goto err1;
	}

	res = CODEC_WRITE(RT5621_STEREO_AD_DA_CLK_CTRL, 0x166d);	/* 44.1KHz */
	/* res = CODEC_WRITE(RT5621_STEREO_AD_DA_CLK_CTRL, 0x266d);	/* 22.05KHz */
	if (res != 0) {
		goto err1;
	}

	res = CODEC_WRITE_M(RT5621_AUDIO_INTERFACE, 0, SDP_SLAVE_MODE);
	if (res != 0) {
		goto err1;
	}

	/* ADC & DAC LRCK L/R swap & PCM Left Justified */
	res = CODEC_WRITE_M(RT5621_AUDIO_INTERFACE, ADC_DATA_L_R_SWAP | DAC_DATA_L_R_SWAP | I2S_DF_LEFT, ADC_DATA_L_R_SWAP | DAC_DATA_L_R_SWAP | I2S_DF_MASK);
	if (res != 0) {
		goto err1;
	}

	res = CODEC_WRITE(RT5621_JACK_DET_CTRL,0x0);	
	if (res != 0) {
		goto err1;
	}

	schedule_timeout_uninterruptible(msecs_to_jiffies(500));
	rt5621_ChangeCodecPowerStatus(POWER_STATE_D1); //low on of playback & record
#endif

	codec->power_on++;
	mutex_unlock(&codec->power_mutex);

	printk("codec_power_on\n");
	return res;
err1:
	printk( "audio init error %d \n", res);
	mutex_unlock(&codec->power_mutex);

	return res;
}

#ifdef CONFIG_EMXX_ANDROID
int codec_power_off(void)
#else
static int codec_power_off(void)
#endif
{
	struct emxx_codec_mixer *codec = &codec_mixer;
	int res = 0;

	mutex_lock(&codec->power_mutex);
	if (codec->power_on == 0) {
		mutex_unlock(&codec->power_mutex);
		return 0;
	}

#ifdef REALTEK_CODEC
	res = CODEC_WRITE_M(RT5621_SPK_OUT_VOL, RT_L_MUTE | RT_R_MUTE, RT_L_MUTE | RT_R_MUTE);
	if (res < 0) goto out;
	res = CODEC_WRITE_M(RT5621_HP_OUT_VOL, RT_L_MUTE | RT_R_MUTE, RT_L_MUTE | RT_R_MUTE);
	if (res < 0) goto out;
	res = CODEC_WRITE_M(RT5621_MONO_AUX_OUT_VOL, RT_L_MUTE | RT_R_MUTE, RT_L_MUTE | RT_R_MUTE);
	if (res < 0) goto out;
	res = CODEC_WRITE(RT5621_PWR_MANAG_ADD3, 0x0000);
	if (res < 0) goto out;
	res = CODEC_WRITE(RT5621_PWR_MANAG_ADD2, 0x0000);
	if (res < 0) goto out;
	res = CODEC_WRITE(RT5621_PWR_MANAG_ADD1, 0x0000);
	if (res < 0) goto out;
#endif

	i2c_del_driver(&i2c_codec_driver);

	emxx_close_clockgate(EMXX_CLK_REF);

	codec->power_on--;
#ifdef REALTEK_CODEC
out:
#endif
	mutex_unlock(&codec->power_mutex);

	return res;
}
EXPORT_SYMBOL(codec_power_off);	//added by xyp @ 2011-12-20, this function will be called in shutdown function of arch/arm/mach-emxx/axp192.c

#ifdef REALTEK_CODEC
static int rt5621_ChangeCodecPowerStatus(int power_state)
{
	unsigned short int PowerDownState=0;

	switch(power_state)
	{
		case POWER_STATE_D0: //FULL ON-----power on all power
			CODEC_WRITE(RT5621_PWR_MANAG_ADD1, ~PowerDownState);
			CODEC_WRITE(RT5621_PWR_MANAG_ADD2, ~PowerDownState);
			CODEC_WRITE(RT5621_PWR_MANAG_ADD3, ~PowerDownState);
			break;	

		case POWER_STATE_D1: //Low on of playback & record
			CODEC_WRITE_M(RT5621_PWR_MANAG_ADD1, 
				PWR_MAIN_I2S_EN|PWR_MIC1_BIAS_EN|PWR_AUX_OUT_AMP|PWR_AUX_OUT_ENH_AMP|PWR_HP_OUT_ENH_AMP|PWR_HP_OUT_AMP,
				PWR_MAIN_I2S_EN|PWR_MIC1_BIAS_EN|PWR_AUX_OUT_AMP|PWR_AUX_OUT_ENH_AMP|PWR_HP_OUT_ENH_AMP|PWR_HP_OUT_AMP);
			CODEC_WRITE_M(RT5621_PWR_MANAG_ADD2,
				PWR_VREF|PWR_L_ADC_CLK_GAIN|PWR_R_ADC_CLK_GAIN|PWR_L_HP_MIXER|PWR_R_HP_MIXER|PWR_L_ADC_REC_MIXER|PWR_R_ADC_REC_MIXER
				|PWR_PLL|PWR_CLASS_AB|PWR_CLASS_D|PWR_DAC_REF_CIR|PWR_L_DAC_CLK|PWR_R_DAC_CLK|PWR_MONO_MIXER,
				PWR_VREF|PWR_L_ADC_CLK_GAIN|PWR_R_ADC_CLK_GAIN|PWR_L_HP_MIXER|PWR_R_HP_MIXER|PWR_L_ADC_REC_MIXER|PWR_R_ADC_REC_MIXER
				|PWR_PLL|PWR_CLASS_AB|PWR_CLASS_D|PWR_DAC_REF_CIR|PWR_L_DAC_CLK|PWR_R_DAC_CLK|PWR_MONO_MIXER);
			CODEC_WRITE_M(RT5621_PWR_MANAG_ADD3,
				PWR_MAIN_BIAS|PWR_HP_R_OUT_VOL|PWR_HP_L_OUT_VOL|PWR_SPK_OUT|PWR_AUXIN_L_VOL|PWR_AUXIN_R_VOL
				|PWR_AUXOUT_L_VOL_AMP|PWR_AUXOUT_R_VOL_AMP|PWR_MIC1_FUN_CTRL|PWR_MIC2_FUN_CTRL,
				PWR_MAIN_BIAS|PWR_HP_R_OUT_VOL|PWR_HP_L_OUT_VOL|PWR_SPK_OUT|PWR_AUXIN_L_VOL|PWR_AUXIN_R_VOL
				|PWR_AUXOUT_L_VOL_AMP|PWR_AUXOUT_R_VOL_AMP|PWR_MIC1_FUN_CTRL|PWR_MIC2_FUN_CTRL);
			break;

		case POWER_STATE_D1_PLAYBACK: //Low on of Playback
			CODEC_WRITE_M(RT5621_PWR_MANAG_ADD1,
				PWR_MAIN_I2S_EN|PWR_HP_OUT_ENH_AMP|PWR_HP_OUT_AMP, PWR_MAIN_I2S_EN|PWR_HP_OUT_ENH_AMP|PWR_HP_OUT_AMP);
			CODEC_WRITE_M(RT5621_PWR_MANAG_ADD2,
				PWR_VREF|PWR_PLL|PWR_CLASS_AB|PWR_CLASS_D|PWR_DAC_REF_CIR|PWR_L_DAC_CLK|PWR_R_DAC_CLK|PWR_L_HP_MIXER|PWR_R_HP_MIXER,
				PWR_VREF|PWR_PLL|PWR_CLASS_AB|PWR_CLASS_D|PWR_DAC_REF_CIR|PWR_L_DAC_CLK|PWR_R_DAC_CLK|PWR_L_HP_MIXER|PWR_R_HP_MIXER);
			CODEC_WRITE_M(RT5621_PWR_MANAG_ADD3,
				PWR_MAIN_BIAS|PWR_HP_R_OUT_VOL|PWR_HP_L_OUT_VOL|PWR_SPK_OUT|PWR_AUXIN_L_VOL|PWR_AUXIN_R_VOL,
				PWR_MAIN_BIAS|PWR_HP_R_OUT_VOL|PWR_HP_L_OUT_VOL|PWR_SPK_OUT|PWR_AUXIN_L_VOL|PWR_AUXIN_R_VOL);		
			break;

		case POWER_STATE_D1_RECORD: //Low on of Record
			CODEC_WRITE_M(RT5621_PWR_MANAG_ADD1, PWR_MAIN_I2S_EN|PWR_MIC1_BIAS_EN|PWR_AUX_OUT_AMP|PWR_AUX_OUT_ENH_AMP, 
				PWR_MAIN_I2S_EN|PWR_MIC1_BIAS_EN|PWR_AUX_OUT_AMP|PWR_AUX_OUT_ENH_AMP);
			CODEC_WRITE_M(RT5621_PWR_MANAG_ADD2,
				PWR_VREF|PWR_PLL|PWR_L_ADC_CLK_GAIN|PWR_R_ADC_CLK_GAIN|PWR_MONO_MIXER|PWR_L_ADC_REC_MIXER|PWR_R_ADC_REC_MIXER,
				PWR_VREF|PWR_PLL|PWR_L_ADC_CLK_GAIN|PWR_R_ADC_CLK_GAIN|PWR_MONO_MIXER|PWR_L_ADC_REC_MIXER|PWR_R_ADC_REC_MIXER);
			CODEC_WRITE_M(RT5621_PWR_MANAG_ADD3,
				PWR_MAIN_BIAS|PWR_AUXOUT_L_VOL_AMP|PWR_AUXOUT_R_VOL_AMP|PWR_MIC1_FUN_CTRL|PWR_MIC2_FUN_CTRL,
				PWR_MAIN_BIAS|PWR_AUXOUT_L_VOL_AMP|PWR_AUXOUT_R_VOL_AMP|PWR_MIC1_FUN_CTRL|PWR_MIC2_FUN_CTRL);
			break;

		case POWER_STATE_D2: //STANDBY of playback & record
			CODEC_WRITE_M(RT5621_PWR_MANAG_ADD1, 0,
				PWR_MAIN_I2S_EN|PWR_MIC1_BIAS_EN|PWR_AUX_OUT_AMP|PWR_AUX_OUT_ENH_AMP|PWR_HP_OUT_ENH_AMP|PWR_HP_OUT_AMP);
			CODEC_WRITE_M(RT5621_PWR_MANAG_ADD2, 0,
				PWR_VREF|PWR_L_ADC_CLK_GAIN|PWR_R_ADC_CLK_GAIN|PWR_L_HP_MIXER|PWR_R_HP_MIXER|PWR_L_ADC_REC_MIXER|PWR_R_ADC_REC_MIXER
				|PWR_PLL|PWR_CLASS_AB|PWR_CLASS_D|PWR_DAC_REF_CIR|PWR_L_DAC_CLK|PWR_R_DAC_CLK|PWR_MONO_MIXER);
			CODEC_WRITE_M(RT5621_PWR_MANAG_ADD3, 0,
				PWR_MAIN_BIAS|PWR_HP_R_OUT_VOL|PWR_HP_L_OUT_VOL|PWR_SPK_OUT|PWR_AUXIN_L_VOL|PWR_AUXIN_R_VOL
				|PWR_AUXOUT_L_VOL_AMP|PWR_AUXOUT_R_VOL_AMP|PWR_MIC1_FUN_CTRL|PWR_MIC2_FUN_CTRL);
			break;

		case POWER_STATE_D2_PLAYBACK: //STANDBY of playback
			CODEC_WRITE_M(RT5621_PWR_MANAG_ADD1, 0, PWR_MAIN_I2S_EN|PWR_HP_OUT_ENH_AMP|PWR_HP_OUT_AMP);
			CODEC_WRITE_M(RT5621_PWR_MANAG_ADD2, 0, 
				PWR_VREF|PWR_PLL|PWR_CLASS_AB|PWR_CLASS_D|PWR_DAC_REF_CIR|PWR_L_DAC_CLK|PWR_R_DAC_CLK|PWR_L_HP_MIXER|PWR_R_HP_MIXER);
			CODEC_WRITE_M(RT5621_PWR_MANAG_ADD3, 0, 
				PWR_MAIN_BIAS|PWR_HP_R_OUT_VOL|PWR_HP_L_OUT_VOL|PWR_SPK_OUT|PWR_AUXIN_L_VOL|PWR_AUXIN_R_VOL);		
			break;

		case POWER_STATE_D2_RECORD: //STANDBY of record
			CODEC_WRITE_M(RT5621_PWR_MANAG_ADD1, 0,  
				PWR_MAIN_I2S_EN|PWR_MIC1_BIAS_EN|PWR_AUX_OUT_AMP|PWR_AUX_OUT_ENH_AMP);
			CODEC_WRITE_M(RT5621_PWR_MANAG_ADD2, 0, 
				PWR_VREF|PWR_PLL|PWR_L_ADC_CLK_GAIN|PWR_R_ADC_CLK_GAIN|PWR_MONO_MIXER|PWR_L_ADC_REC_MIXER|PWR_R_ADC_REC_MIXER);
			CODEC_WRITE_M(RT5621_PWR_MANAG_ADD3, 0, 
				PWR_MAIN_BIAS|PWR_AUXOUT_L_VOL_AMP|PWR_AUXOUT_R_VOL_AMP|PWR_MIC1_FUN_CTRL|PWR_MIC2_FUN_CTRL);
			break;		

		case POWER_STATE_D3: //SLEEP
		case POWER_STATE_D4: //OFF----power off all power
			CODEC_WRITE(RT5621_PWR_MANAG_ADD1, PowerDownState);
			CODEC_WRITE(RT5621_PWR_MANAG_ADD2, PowerDownState);
			CODEC_WRITE(RT5621_PWR_MANAG_ADD3, PowerDownState);
			break;	

		default:
			break;
	}

	return 0;
}
#endif

static inline int emxx_codec_volume(int addr, struct emxx_codec_mixer *codec)
{
	int res = 0;
	if (codec->vol_info[IDX(addr)].value[0] > 0x1f)
		codec->vol_info[IDX(addr)].value[0] = 0x1f;

	if (codec->vol_info[IDX(addr)].value[1] > 0x1f)
		codec->vol_info[IDX(addr)].value[1] = 0x1f;

#ifdef REALTEK_CODEC
	switch(addr) {
		case MIXER_VOL_DAC:
			/* DAC left Volume */
			res = CODEC_WRITE_M(RT5621_STEREO_DAC_VOL, VOL_L_GAIN(codec->vol_info[IDX(addr)].value[0]), VOL_L_GAIN_MASK);
			if (res < 0)
				return res;
			/* DAC right volume */
			res = CODEC_WRITE_M(RT5621_STEREO_DAC_VOL, VOL_R_GAIN(codec->vol_info[IDX(addr)].value[1]), VOL_R_GAIN_MASK);
			if (res < 0)
				return res;
			break;

		case MIXER_VOL_HP:
			/* HP_OUT left volume */
			res = CODEC_WRITE_M(RT5621_HP_OUT_VOL, VOL_L_GAIN(codec->vol_info[IDX(addr)].value[0]), VOL_L_GAIN_MASK);
			if (res < 0)
				return res;
			/* HP_OUT right volume */
			res = CODEC_WRITE_M(RT5621_HP_OUT_VOL, VOL_R_GAIN(codec->vol_info[IDX(addr)].value[1]), VOL_R_GAIN_MASK);
			if (res < 0)
				return res;
			break;

		case MIXER_VOL_SPK:
			/* SPK_OUT left volume */
			res = CODEC_WRITE_M(RT5621_SPK_OUT_VOL, VOL_L_GAIN(codec->vol_info[IDX(addr)].value[0]), VOL_L_GAIN_MASK);
			if (res < 0)
				return res;
			/* SPK_OUT right volume */
			res = CODEC_WRITE_M(RT5621_SPK_OUT_VOL, VOL_R_GAIN(codec->vol_info[IDX(addr)].value[1]), VOL_R_GAIN_MASK);
			if (res < 0)
				return res;
			break;

		case MIXER_VOL_MIC1:
			/* MIC1 volume */
			res = CODEC_WRITE_M(RT5621_MIC_VOL, VOL_L_GAIN(codec->vol_info[IDX(addr)].value[0]), VOL_L_GAIN_MASK);
			if (res < 0)
				return res;
			break;

		case MIXER_VOL_MIC2:
			/* MIC2 volume */
			res = CODEC_WRITE_M(RT5621_MIC_VOL, VOL_R_GAIN(codec->vol_info[IDX(addr)].value[0]), VOL_R_GAIN_MASK);
			if (res < 0)
				return res;
			break;

		case MIXER_VOL_AUXIN:
			/* AUXIN left volume */
			res = CODEC_WRITE_M(RT5621_AUXIN_VOL, VOL_L_GAIN(codec->vol_info[IDX(addr)].value[0]), VOL_L_GAIN_MASK);
			if (res < 0)
				return res;
			/* AUXIN right volume */
			res = CODEC_WRITE_M(RT5621_AUXIN_VOL, VOL_R_GAIN(codec->vol_info[IDX(addr)].value[1]), VOL_R_GAIN_MASK);
			if (res < 0)
				return res;
			break;

		case MIXER_VOL_AUXOUT:
			/* AUXOUT left volume */
			res = CODEC_WRITE_M(RT5621_MONO_AUX_OUT_VOL, VOL_L_GAIN(codec->vol_info[IDX(addr)].value[0]), VOL_L_GAIN_MASK);
			if (res < 0)
				return res;
			/* AUXOUT right volume */
			res = CODEC_WRITE_M(RT5621_MONO_AUX_OUT_VOL, VOL_R_GAIN(codec->vol_info[IDX(addr)].value[1]), VOL_R_GAIN_MASK);
			if (res < 0)
				return res;
			break;

		default:
			break;
	}
#endif
	return 0;
}

#ifdef REALTEK_CODEC
//*****************************************************************************
//function rt5621_AudioMute : Mute/Unmute audio input/output channel
//Path: input/output channel
//Mute : Mute/Unmute the channel
//*****************************************************************************
static int rt5621_AudioMute(unsigned short int Path, bool Mute)
{
	int RetVal = 0;	

	if (Mute) {
		switch (Path) {
		//wave in
			case RT_WAVIN_ALL_OFF:
				RetVal = CODEC_WRITE_M(RT5621_MONO_AUX_OUT_VOL, RT_L_MUTE|RT_R_MUTE, RT_L_MUTE|RT_R_MUTE);
				RetVal = CODEC_WRITE_M(RT5621_MIC_ROUTING_CTRL, M_MIC1_TO_MONO_MIXER|M_MIC2_TO_MONO_MIXER, M_MIC1_TO_MONO_MIXER|M_MIC2_TO_MONO_MIXER); 
				RetVal = CODEC_WRITE_M(RT5621_ADC_REC_MIXER, RT_WAVIN_L_MONO_MIXER_MUTE|RT_WAVIN_R_MONO_MIXER_MUTE, 
										RT_WAVIN_L_MONO_MIXER_MUTE|RT_WAVIN_R_MONO_MIXER_MUTE);
				break;

			case RT_WAVIN_AUXOUT://Mute AuxOut right&left channel
				RetVal = CODEC_WRITE_M(RT5621_MONO_AUX_OUT_VOL, RT_L_MUTE|RT_R_MUTE, RT_L_MUTE|RT_R_MUTE);
				break;
			case RT_WAVIN_AUXOUT_R://Mute AuxOut right channel
				RetVal = CODEC_WRITE_M(RT5621_MONO_AUX_OUT_VOL, RT_R_MUTE, RT_R_MUTE);
				break;
			case RT_WAVIN_AUXOUT_L://Mute AuxOut left channel
				RetVal = CODEC_WRITE_M(RT5621_MONO_AUX_OUT_VOL, RT_L_MUTE, RT_L_MUTE);
				break;

			case RT_WAVIN_MIC1_2_MONOMIXER://Mute MIC1 to MONO mixer
				RetVal = CODEC_WRITE_M(RT5621_MIC_ROUTING_CTRL, M_MIC1_TO_MONO_MIXER, M_MIC1_TO_MONO_MIXER); 
				break;
			case RT_WAVIN_MIC2_2_MONOMIXER://Mute MIC2 to MONO mixer
				RetVal = CODEC_WRITE_M(RT5621_MIC_ROUTING_CTRL, M_MIC2_TO_MONO_MIXER, M_MIC2_TO_MONO_MIXER); 
				break;

			case RT_WAVIN_MONOMIXER_2_ADCMIXER://Mute MONO mixer to ADC Record Mixer
				RetVal = CODEC_WRITE_M(RT5621_ADC_REC_MIXER, RT_WAVIN_L_MONO_MIXER_MUTE|RT_WAVIN_R_MONO_MIXER_MUTE,
										RT_WAVIN_L_MONO_MIXER_MUTE|RT_WAVIN_R_MONO_MIXER_MUTE); 
				break;
			case RT_WAVIN_L_MONOMIXER_2_ADCMIXER://Mute left MONO mixer to ADC Record Mixer
				RetVal = CODEC_WRITE_M(RT5621_ADC_REC_MIXER, RT_WAVIN_L_MONO_MIXER_MUTE, RT_WAVIN_L_MONO_MIXER_MUTE); 
				break;
			case RT_WAVIN_R_MONOMIXER_2_ADCMIXER://Mute right MONO mixer to ADC Record Mixer
				RetVal = CODEC_WRITE_M(RT5621_ADC_REC_MIXER, RT_WAVIN_R_MONO_MIXER_MUTE, RT_WAVIN_R_MONO_MIXER_MUTE); 
				break;
		//wave out
			case RT_WAVOUT_ALL_OFF:
				RetVal = CODEC_WRITE_M(RT5621_SPK_OUT_VOL, RT_L_MUTE|RT_R_MUTE, RT_L_MUTE|RT_R_MUTE);
				RetVal = CODEC_WRITE_M(RT5621_HP_OUT_VOL, RT_L_MUTE|RT_R_MUTE, RT_L_MUTE|RT_R_MUTE);
				RetVal = CODEC_WRITE_M(RT5621_AUXIN_VOL, M_AUXIN_TO_HP_MIXER, M_AUXIN_TO_HP_MIXER); 
				RetVal = CODEC_WRITE_M(RT5621_STEREO_DAC_VOL, RT_M_HP_MIXER, RT_M_HP_MIXER);
				break;

			case RT_WAVOUT_HP://Mute headphone right&left channel
				RetVal = CODEC_WRITE_M(RT5621_HP_OUT_VOL, RT_L_MUTE|RT_R_MUTE, RT_L_MUTE|RT_R_MUTE);
				break;
			case RT_WAVOUT_HP_R://Mute headphone right channel
				RetVal = CODEC_WRITE_M(RT5621_HP_OUT_VOL, RT_R_MUTE, RT_R_MUTE);
				break;
			case RT_WAVOUT_HP_L://Mute headphone left channel
				RetVal = CODEC_WRITE_M(RT5621_HP_OUT_VOL, RT_L_MUTE, RT_L_MUTE);
				break;

			case RT_WAVOUT_SPK://Mute Speaker right&left channel
				RetVal = CODEC_WRITE_M(RT5621_SPK_OUT_VOL, RT_L_MUTE|RT_R_MUTE, RT_L_MUTE|RT_R_MUTE);
				break;
			case RT_WAVOUT_SPK_R://Mute Speaker right channel
				RetVal = CODEC_WRITE_M(RT5621_SPK_OUT_VOL, RT_R_MUTE, RT_R_MUTE);
				break;
			case RT_WAVOUT_SPK_L://Mute Speaker left channel
				RetVal = CODEC_WRITE_M(RT5621_SPK_OUT_VOL, RT_L_MUTE, RT_L_MUTE);
				break;

			case RT_WAVOUT_AUX2HPMIXER://Mute AUXIN to HP Mixer
				RetVal = CODEC_WRITE_M(RT5621_AUXIN_VOL, M_AUXIN_TO_HP_MIXER, M_AUXIN_TO_HP_MIXER); 
				break;

			case RT_WAVOUT_DAC2HPMIXER://Mute DAC to HP Mixer
				RetVal = CODEC_WRITE_M(RT5621_STEREO_DAC_VOL, RT_M_HP_MIXER, RT_M_HP_MIXER);
				break;

			default:
				return 0;
		}
	} else {
		switch (Path) {
		//wave in
			case RT_WAVIN_AUXOUT://unMute AuxOut right&left channel
				RetVal = CODEC_WRITE_M(RT5621_MONO_AUX_OUT_VOL, 0, RT_L_MUTE|RT_R_MUTE); 
				break;
			case RT_WAVIN_AUXOUT_R://unMute AuxOut right channel
				RetVal = CODEC_WRITE_M(RT5621_MONO_AUX_OUT_VOL, 0, RT_R_MUTE); 
				break;
			case RT_WAVIN_AUXOUT_L://unMute AuxOut left channel
				RetVal = CODEC_WRITE_M(RT5621_MONO_AUX_OUT_VOL, 0, RT_L_MUTE); 
				break;

			case RT_WAVIN_MIC1_2_MONOMIXER://unMute MIC1 to MONO mixer
				RetVal = CODEC_WRITE_M(RT5621_MIC_ROUTING_CTRL, 0, M_MIC1_TO_MONO_MIXER); 
				break;
			case RT_WAVIN_MIC2_2_MONOMIXER://unMute MIC2 to MONO mixer
				RetVal = CODEC_WRITE_M(RT5621_MIC_ROUTING_CTRL, 0, M_MIC2_TO_MONO_MIXER); 
				break;

			case RT_WAVIN_MONOMIXER_2_ADCMIXER://unMute MONO mixer to ADC Record Mixer
				RetVal = CODEC_WRITE_M(RT5621_ADC_REC_MIXER, 0, RT_WAVIN_L_MONO_MIXER_MUTE|RT_WAVIN_R_MONO_MIXER_MUTE); 
				break;
			case RT_WAVIN_L_MONOMIXER_2_ADCMIXER://unMute left MONO mixer to ADC Record Mixer
				RetVal = CODEC_WRITE_M(RT5621_ADC_REC_MIXER, 0, RT_WAVIN_L_MONO_MIXER_MUTE); 
				break;
			case RT_WAVIN_R_MONOMIXER_2_ADCMIXER://unMute right MONO mixer to ADC Record Mixer
				RetVal = CODEC_WRITE_M(RT5621_ADC_REC_MIXER, 0, RT_WAVIN_R_MONO_MIXER_MUTE); 
				break;
		//wave out
			case RT_WAVOUT_ALL_ON:
				RetVal = CODEC_WRITE_M(RT5621_SPK_OUT_VOL, 0, RT_L_MUTE|RT_R_MUTE);
				RetVal = CODEC_WRITE_M(RT5621_HP_OUT_VOL, 0, RT_L_MUTE|RT_R_MUTE);
				RetVal = CODEC_WRITE_M(RT5621_STEREO_DAC_VOL, 0, RT_M_HP_MIXER);
				RetVal = CODEC_WRITE_M(RT5621_AUXIN_VOL, 0, M_AUXIN_TO_HP_MIXER); 
				break;

			case RT_WAVOUT_HP://UnMute headphone right&left channel
				RetVal = CODEC_WRITE_M(RT5621_HP_OUT_VOL, 0, RT_L_MUTE|RT_R_MUTE);	
				break;
			case RT_WAVOUT_HP_R://UnMute headphone right channel
				RetVal = CODEC_WRITE_M(RT5621_HP_OUT_VOL, 0, RT_R_MUTE);	
				break;
			case RT_WAVOUT_HP_L://UnMute headphone left channel
				RetVal = CODEC_WRITE_M(RT5621_HP_OUT_VOL, 0, RT_L_MUTE);	
				break;

			case RT_WAVOUT_SPK://unMute Speaker right&left channel
				RetVal = CODEC_WRITE_M(RT5621_SPK_OUT_VOL, 0, RT_L_MUTE|RT_R_MUTE); 			
				break;
			case RT_WAVOUT_SPK_R://unMute Speaker right channel
				RetVal = CODEC_WRITE_M(RT5621_SPK_OUT_VOL, 0, RT_R_MUTE); 			
				break;
			case RT_WAVOUT_SPK_L://unMute Speaker left channel
				RetVal = CODEC_WRITE_M(RT5621_SPK_OUT_VOL, 0, RT_L_MUTE); 			
				break;

			case RT_WAVOUT_AUX2HPMIXER://unMute AUXIN to HP Mixer
				RetVal = CODEC_WRITE_M(RT5621_AUXIN_VOL, 0, M_AUXIN_TO_HP_MIXER); 
				break;

			case RT_WAVOUT_DAC2HPMIXER://unMute DAC to HP Mixer
				RetVal = CODEC_WRITE_M(RT5621_STEREO_DAC_VOL, 0, RT_M_HP_MIXER); 
				break;

			default:
				return 0;
		}
	}

	return RetVal;
}
#endif


static int emxx_codec_capture_sw(int *val)
{
#ifdef REALTEK_CODEC
	rt5621_AudioMute(RT_WAVIN_ALL_OFF, true);//mute all in
#endif
	switch (val[0]) {
		case 0://OFF
			break;

		case 1://MIC_normal
#ifdef REALTEK_CODEC
			rt5621_AudioMute(RT_WAVIN_MIC1_2_MONOMIXER, false); //unmute MIC1 to MONO mixer
			rt5621_AudioMute(RT_WAVIN_MONOMIXER_2_ADCMIXER, false);	//unmute MONO mixer to ADC Record Mixer
#endif
			break;

		case 2://MIC_ringtone
#ifdef REALTEK_CODEC
			rt5621_AudioMute(RT_WAVIN_MIC1_2_MONOMIXER, false); //unmute MIC1 to MONO mixer
			rt5621_AudioMute(RT_WAVIN_MONOMIXER_2_ADCMIXER, false);	//unmute MONO mixer to ADC Record Mixer
#endif
			break;
		case 3://MIC_incall
#ifdef REALTEK_CODEC
			rt5621_AudioMute(RT_WAVIN_MIC1_2_MONOMIXER, false); //unmute MIC1 to MONO mixer
			rt5621_AudioMute(RT_WAVIN_AUXOUT, false); //unmute AUXOUT
#endif
			break;

		case 4://Headset_normal
#ifdef REALTEK_CODEC
			rt5621_AudioMute(RT_WAVIN_MIC2_2_MONOMIXER, false); //unmute MIC2 to MONO mixer
			rt5621_AudioMute(RT_WAVIN_MONOMIXER_2_ADCMIXER, false);	//unmute MONO mixer to ADC Record Mixer
#endif
			break;

		case 5://Headset_ringtone
#ifdef REALTEK_CODEC
			rt5621_AudioMute(RT_WAVIN_MIC2_2_MONOMIXER, false); //unmute MIC2 to MONO mixer
			rt5621_AudioMute(RT_WAVIN_MONOMIXER_2_ADCMIXER, false);	//unmute MONO mixer to ADC Record Mixer
#endif
			break;

		case 6://Headset_incall
#ifdef REALTEK_CODEC
			schedule_timeout_uninterruptible(msecs_to_jiffies(1000));
			rt5621_AudioMute(RT_WAVIN_MIC2_2_MONOMIXER, false); //unmute MIC2 to MONO mixer
			rt5621_AudioMute(RT_WAVIN_AUXOUT, false); //unmute AUXOUT
#endif
			break;
		default:
			break;
	}

	return 0;
}

static int emxx_codec_playback_sw(int *val)
{
#ifdef REALTEK_CODEC
	rt5621_AudioMute(RT_WAVOUT_ALL_OFF, true); //mute all out
#endif
	switch (val[0]) {
		case 0: /* OFF */
			break;

		case 1: /* Speaker_normal */
#ifdef REALTEK_CODEC
			rt5621_AudioMute(RT_WAVOUT_DAC2HPMIXER, false); //unmute DAC to HP Mixer
			rt5621_AudioMute(RT_WAVOUT_SPK, false); //unmute speaker out
#endif
			break;

		case 2: /* Speaker_ringtone */
#ifdef REALTEK_CODEC
			rt5621_AudioMute(RT_WAVOUT_DAC2HPMIXER, false); //unmute DAC to HP Mixer
			rt5621_AudioMute(RT_WAVOUT_SPK, false); //unmute speaker out
#endif
			break;

		case 3: /* Speaker_incall */
#ifdef REALTEK_CODEC
			rt5621_AudioMute(RT_WAVOUT_AUX2HPMIXER, false); //unmute AUXIN_L/R to HP Mixer
			rt5621_AudioMute(RT_WAVOUT_SPK, false); //unmute speaker out
#endif

			break;

		case 4: /* Earpiece_ringtone */
#ifdef REALTEK_CODEC
			rt5621_AudioMute(RT_WAVOUT_DAC2HPMIXER, false); //unmute DAC to HP Mixer
			rt5621_AudioMute(RT_WAVOUT_SPK, false); //unmute speaker out
#endif
			break;

		case 5: /* Earpiece_incall */
#ifdef REALTEK_CODEC
			rt5621_AudioMute(RT_WAVOUT_AUX2HPMIXER, false); //unmute AUXIN_L/R to HP Mixer
			rt5621_AudioMute(RT_WAVOUT_SPK, false); //unmute speaker out
#endif
			break;

		case 6: /* Headset_normal */
#ifdef REALTEK_CODEC
			rt5621_AudioMute(RT_WAVOUT_DAC2HPMIXER, false); //unmute DAC to HP Mixer
			rt5621_AudioMute(RT_WAVOUT_HP, false); //unmute hp out
#endif
			break;

		case 7: /* Headset_ringtone */
#ifdef REALTEK_CODEC
			rt5621_AudioMute(RT_WAVOUT_DAC2HPMIXER, false); //unmute DAC to HP Mixer
			rt5621_AudioMute(RT_WAVOUT_HP, false); //unmute hp out
#endif
			break;

		case 8: /* Headset_incall */
#ifdef REALTEK_CODEC
			rt5621_AudioMute(RT_WAVOUT_AUX2HPMIXER, false); //unmute AUXIN_L/R to HP Mixer
			rt5621_AudioMute(RT_WAVOUT_HP, false); //unmute hp out
#endif
			break;

		default:
			break;
	}

	return 0;
}

static int emxx_codec_sampling_rate_sw(long val)
{
        int i;

        i = get_coeff(val);
        if( i >= 0 ) {
                CODEC_WRITE(RT5621_STEREO_AD_DA_CLK_CTRL, coeff_div[i].clk_regvalue);
                CODEC_WRITE(RT5621_PLL_CTRL, coeff_div[i].pll_regvalue);
        }

        return 0;

}

static int emxx_codec_mixer_write(int addr, struct emxx_codec_mixer *codec)
{
	int res = 0;
	FNC_ENTRY

	switch (addr) {
		case MIXER_VOL_DAC:
		case MIXER_VOL_HP:
		case MIXER_VOL_SPK:
		case MIXER_VOL_MIC1:
		case MIXER_VOL_MIC2:
		case MIXER_VOL_AUXIN:
		case MIXER_VOL_AUXOUT:
			res = emxx_codec_volume(addr, codec);
			break;
		case MIXER_SW_PLAYBACK:
			res = emxx_codec_playback_sw(
					&codec->enum_info[IDX(addr)].value);
			break;
		case MIXER_SW_CAPTURE:
			res = emxx_codec_capture_sw(
					&codec->enum_info[IDX(addr)].value);
			break;
		case MIXER_SW_SAMPLING_RATE:
			{
				long val = 0;

				switch (codec->enum_info[IDX(addr)].value) {
					case 0:
						val = 7350;
						break;
					case 1:
						val = 8000;
						break;
					case 2:
						val = 11025;
						break;
					case 3:
						val = 12000;
						break;
					case 4:
						val = 14700;
						break;
					case 5:
						val = 16000;
						break;
					case 6:
						val = 22050;
						break;
					case 7:
						val = 24000;
						break;
					case 8:
						val = 29400;
						break;
					case 9:
						val = 32000;
						break;
					case 10:
						val = 44100;
						break;
					case 11:
						val = 48000;
						break;
				}

				res = emxx_codec_sampling_rate_sw(val);

				break;
			}
		case MIXER_SW_CODEC_POWER_BL:
			if (codec->bl_info[IDX(addr)].value)
				res = codec_power_on();
			else
				res = codec_power_off();

			break;
	}

	FNC_EXIT return res;
}

#define EMXX_CODEC_INTEGER(xname, xindex, addr) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, .index = xindex, \
	.info = emxx_codec_integer_info, .get = emxx_codec_integer_get, \
	.put = emxx_codec_integer_put, .private_value = addr }

static int emxx_codec_integer_info(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *uinfo)
{
	struct emxx_codec_mixer *codec = snd_kcontrol_chip(kcontrol);
	int addr = kcontrol->private_value;
	FNC_ENTRY

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = codec->vol_info[IDX(addr)].count;
	uinfo->value.integer.min  = codec->vol_info[IDX(addr)].val_int_min;
	uinfo->value.integer.max  = codec->vol_info[IDX(addr)].val_int_max;
	uinfo->value.integer.step = codec->vol_info[IDX(addr)].val_int_step;

	FNC_EXIT return 0;
}

static int emxx_codec_integer_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct emxx_codec_mixer *codec = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	int addr = kcontrol->private_value;
	int cnt = codec->vol_info[IDX(addr)].count;
	int i;
	FNC_ENTRY

	spin_lock_irqsave(&codec->mixer_lock, flags);
	for (i = 0; i < cnt; i++) {
		ucontrol->value.integer.value[i]  =
			codec->vol_info[IDX(addr)].value[i];
	}
	spin_unlock_irqrestore(&codec->mixer_lock, flags);

	FNC_EXIT return 0;
}

static int emxx_codec_integer_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct emxx_codec_mixer *codec = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	int addr = kcontrol->private_value;
	int min  = codec->vol_info[IDX(addr)].val_int_min;
	int max  = codec->vol_info[IDX(addr)].val_int_max;
	int step = codec->vol_info[IDX(addr)].val_int_step;
	int i, change = 0;
	int cnt = codec->vol_info[IDX(addr)].count;
	int volume[2];
	int res = 0;
	FNC_ENTRY

	if (codec->power_on == 0)
		FNC_EXIT return -EINVAL;

	for (i = 0; i < cnt; i++) {

#ifdef XYP_DEBUG_DEVICE
		printk("==== volume[%d] : +%ld dB ====\n", i, (ucontrol->value.integer.value[i])*3/2);
#else
		d8b("value.integer.value[%d]=%ld\n",
				i, ucontrol->value.integer.value[i]);
#endif
		volume[i] = (ucontrol->value.integer.value[i] / step) * step;
		if (volume[i] < min)
			volume[i] = min;
		if (volume[i] > max)
			volume[i] = max;
	}

	spin_lock_irqsave(&codec->mixer_lock, flags);
	for (i = 0; i < cnt; i++) {
		change |= (codec->vol_info[IDX(addr)].value[i] != volume[i]);
		codec->vol_info[IDX(addr)].value[i] = volume[i];
		d8b("volume[%d]=%d\n", i, volume[i]);
	}
	spin_unlock_irqrestore(&codec->mixer_lock, flags);

	if (change) {
		res = emxx_codec_mixer_write(addr, codec);
		if (res < 0)
			FNC_EXIT return res;

	}

	FNC_EXIT return change;
}

#define EMXX_CODEC_ENUM(xname, xindex, addr) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, .index = xindex, \
	.info = emxx_codec_enum_info, .get = emxx_codec_enum_get, \
	.put = emxx_codec_enum_put, .private_value = addr }

static int emxx_codec_enum_info(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *uinfo)
{
	struct emxx_codec_mixer *codec = snd_kcontrol_chip(kcontrol);
	int addr = kcontrol->private_value;
	int items = codec->enum_info[IDX(addr)].items;
	FNC_ENTRY

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = items;
	if (uinfo->value.enumerated.item > (items - 1))
		uinfo->value.enumerated.item = (items - 1);
	strcpy(uinfo->value.enumerated.name,
			codec->enum_info[IDX(addr)].texts[uinfo->value.enumerated.item]);

	FNC_EXIT return 0;
}

static int emxx_codec_enum_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct emxx_codec_mixer *codec = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	int addr = kcontrol->private_value;
	FNC_ENTRY

	spin_lock_irqsave(&codec->mixer_lock, flags);
	ucontrol->value.enumerated.item[0] = codec->enum_info[IDX(addr)].value;
	spin_unlock_irqrestore(&codec->mixer_lock, flags);

	FNC_EXIT return 0;
}

static int emxx_codec_enum_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct emxx_codec_mixer *codec = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	int change, val;
	int addr = kcontrol->private_value;
	int items = codec->enum_info[IDX(addr)].items;
	int res = 0;
	FNC_ENTRY

	if (codec->power_on == 0)
		FNC_EXIT return -EINVAL;

#ifdef XYP_DEBUG_DEVICE
	printk("\n==== %s ====\n", codec->enum_info[IDX(addr)].texts[ucontrol->value.enumerated.item[0]]);
#else
	d8b("value.enumerated.item[0]=%d\n",
			ucontrol->value.enumerated.item[0]);
#endif
	if (ucontrol->value.enumerated.item[0] > (items - 1))
		return -EINVAL;
	val = ucontrol->value.enumerated.item[0];

	spin_lock_irqsave(&codec->mixer_lock, flags);
	change = (codec->enum_info[IDX(addr)].value != val);
	codec->enum_info[IDX(addr)].value = val;
	d8b("val=%d\n", val);
	spin_unlock_irqrestore(&codec->mixer_lock, flags);

	if (change) {
		res = emxx_codec_mixer_write(addr, codec);
		if (res < 0)
			FNC_EXIT return res;

	}

	FNC_EXIT return change;
}

#define EMXX_CODEC_BOOLEAN(xname, xindex, addr) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, .index = xindex, \
	.info = emxx_codec_boolean_info, .get = emxx_codec_boolean_get, \
	.put = emxx_codec_boolean_put, .private_value = addr }

static int emxx_codec_boolean_info(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *uinfo)
{
	FNC_ENTRY

	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;

	FNC_EXIT return 0;
}

static int emxx_codec_boolean_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct emxx_codec_mixer *codec = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	int addr = kcontrol->private_value;
	FNC_ENTRY

	spin_lock_irqsave(&codec->mixer_lock, flags);
	ucontrol->value.integer.value[0] = codec->bl_info[IDX(addr)].value;
	spin_unlock_irqrestore(&codec->mixer_lock, flags);

	FNC_EXIT return 0;
}

static int emxx_codec_boolean_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct emxx_codec_mixer *codec = snd_kcontrol_chip(kcontrol);
	unsigned long flags;
	int change, addr = kcontrol->private_value;
	int val;
	int res = 0;
	FNC_ENTRY

	if ((codec->power_on == 0) && (addr != MIXER_SW_CODEC_POWER_BL))
		FNC_EXIT return -EINVAL;

#ifdef XYP_DEBUG_DEVICE
	if (ucontrol->value.integer.value[0]) {
		printk("==== codec power on\n");
	} else {
		printk("==== codec power off\n");
	}
#else
	d8b("value.integer.value[0]=%ld\n", ucontrol->value.integer.value[0]);
#endif
	val = ucontrol->value.integer.value[0] & 1;
	spin_lock_irqsave(&codec->mixer_lock, flags);
	change = (codec->bl_info[IDX(addr)].value != val);
	codec->bl_info[IDX(addr)].value = val;
	d8b("val=%d\n", val);
	spin_unlock_irqrestore(&codec->mixer_lock, flags);

	if (change) {
		res = emxx_codec_mixer_write(addr, codec);
		if (res < 0)
			FNC_EXIT return res;
	}

	FNC_EXIT return change;
}

static struct snd_kcontrol_new emxx_codec_controls[] = {
	EMXX_CODEC_INTEGER("DAC Volume", 0, MIXER_VOL_DAC),
	EMXX_CODEC_INTEGER("Headphone Volume", 0, MIXER_VOL_HP),
	EMXX_CODEC_INTEGER("Speaker Volume", 0, MIXER_VOL_SPK),
	EMXX_CODEC_INTEGER("MIC1 Volume", 0, MIXER_VOL_MIC1),
	EMXX_CODEC_INTEGER("MIC2 Volume", 0, MIXER_VOL_MIC2),
	EMXX_CODEC_INTEGER("AUXIN Volume", 0, MIXER_VOL_AUXIN),
	EMXX_CODEC_INTEGER("AUXOUT Volume", 0, MIXER_VOL_AUXOUT),
	EMXX_CODEC_ENUM("Capture Switch", 0, MIXER_SW_CAPTURE),
	EMXX_CODEC_ENUM("Sampling Rate Switch", 0, MIXER_SW_SAMPLING_RATE),
	EMXX_CODEC_ENUM("Playback Switch", 0, MIXER_SW_PLAYBACK),
	EMXX_CODEC_BOOLEAN("CODEC Power Switch", 0, MIXER_SW_CODEC_POWER_BL),
};

static int __init emxx_codec_mixer_new(struct snd_card *card)
{
	unsigned int idx;
	int err;
	FNC_ENTRY

	if (snd_BUG_ON(!card))
		return -EINVAL;

	spin_lock_init(&codec_mixer.mixer_lock);
	mutex_init(&codec_mixer.power_mutex);
	strcpy(card->mixername, "emxx mixer");

	codec_mixer.card = card;

	for (idx = 0; idx < ARRAY_SIZE(emxx_codec_controls); idx++) {
		err = snd_ctl_add(card,
				snd_ctl_new1(&emxx_codec_controls[idx], &codec_mixer));
		if (err < 0)
			FNC_EXIT return err;

	}

	FNC_EXIT return 0;
}


static struct snd_pcm *emxx_codec;

static unsigned int rates[] = {
	7350, 8000, 11025, 12000, 14700, 16000,
	22050, 24000, 29400, 32000, 44100, 48000,
};

static struct snd_pcm_hw_constraint_list hw_constraints_rates = {
	.count  = ARRAY_SIZE(rates),
	.list   = rates,
	.mask   = 0,
};

#define RT5621_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
		SNDRV_PCM_FMTBIT_S24_LE)

static int emxx_codec_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct emxx_codec_mixer *codec = &codec_mixer;
	int err;

	FNC_ENTRY

	if (codec->power_on == 0) {
		if(codec_power_on() < 0) {
			printk( "re-poweron error\n");
			FNC_EXIT return -EINVAL;
		}
	}
	runtime->hw.formats = RT5621_FORMATS; //SNDRV_PCM_FMTBIT_S16_LE;
	runtime->hw.rates = (SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_11025 |
			SNDRV_PCM_RATE_16000 |
			SNDRV_PCM_RATE_22050 |
			SNDRV_PCM_RATE_32000 |
			SNDRV_PCM_RATE_44100 |
			SNDRV_PCM_RATE_48000 |
			SNDRV_PCM_RATE_KNOT);
	runtime->hw.rate_min = 8000;
	runtime->hw.rate_max = 48000;
	runtime->hw.channels_min = 2;
	runtime->hw.channels_max = 2;

	err = snd_pcm_hw_constraint_integer(runtime,
			SNDRV_PCM_HW_PARAM_PERIODS);
	if (err < 0)
		FNC_EXIT return err;

	err = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
			&hw_constraints_rates);
	if (err < 0)
		FNC_EXIT return err;

	FNC_EXIT return 0;
}

static void emxx_codec_shutdown(struct snd_pcm_substream *substream)
{
	FNC_ENTRY
	FNC_EXIT return;
}

static int emxx_codec_prepare(struct snd_pcm_substream *substream)
{
	FNC_ENTRY
	FNC_EXIT return 0;
}

static struct audio_stream emxx_codec_out = {
	.id                     = "pcm_m2p",
	.dma_ch                 = EMXX_DMAC_M2P_SIO1,
};

static struct audio_stream emxx_codec_in = {
	.id                     = "pcm_p2m",
	.dma_ch                 = EMXX_DMAC_P2M_SIO1,
};

static struct emxx_pcm_client emxx_codec_client = {
	.pcm_ch                 = EMXX_PCM_CH0,
	.startup                = emxx_codec_startup,
	.shutdown               = emxx_codec_shutdown,
	.prepare                = emxx_codec_prepare,
};


/* codec sound card */

struct snd_card *emxx_codec_card;

static char *id = ID_VALUE; /* ID for this card */
module_param(id, charp, 0444);

#ifdef CONFIG_PM
static int emxx_codec_suspend(struct platform_device *dev, pm_message_t state)
{
	struct snd_card *card = platform_get_drvdata(dev);
	int ret = 0;

	switch (state.event) {
		case PM_EVENT_SUSPEND:
			break;
		default:
			break;
	}

	if (card)
		ret = emxx_pcm_suspend(dev, state);
#if 1
	rt5621_ChangeCodecPowerStatus(POWER_STATE_D2); //standby of playback & record
#else
	rt5621_ChangeCodecPowerStatus(POWER_STATE_D2_RECORD); //standby of record
	CODEC_WRITE_M(RT5621_AUXIN_VOL, M_AUXIN_TO_HP_MIXER, M_AUXIN_TO_HP_MIXER); //mute AUXIN to HP Mixer
#endif
	return ret;
}

static int emxx_codec_resume(struct platform_device *dev)
{
	struct snd_card *card = platform_get_drvdata(dev);
	int ret = 0;
#if 1
	rt5621_ChangeCodecPowerStatus(POWER_STATE_D1); //low on playback & record
#else
	rt5621_ChangeCodecPowerStatus(POWER_STATE_D1_RECORD); //low on record
	CODEC_WRITE_M(RT5621_AUXIN_VOL, 0, M_AUXIN_TO_HP_MIXER); //unmute AUXIN to HP Mixer
#endif
	if (card)
		ret = emxx_pcm_resume(dev);

	return ret;
}
#else
#define emxx_codec_suspend     NULL
#define emxx_codec_resume      NULL
#endif

static int emxx_codec_probe(struct platform_device *devptr)
{
	struct snd_card *card = NULL;
	int ret;
	FNC_ENTRY

	codec_init();

	ret = snd_card_create(SNDRV_DEFAULT_IDX1, id, THIS_MODULE, 0, &card);
	if (ret < 0) {
		printk("%s: err\n", __func__);
		FNC_EXIT return -ENOMEM;
	}

	snd_card_set_dev(card, &devptr->dev);

	emxx_codec_card = card;

	emxx_codec_client.s[SNDRV_PCM_STREAM_PLAYBACK] = &emxx_codec_out;
	emxx_codec_client.s[SNDRV_PCM_STREAM_CAPTURE] = &emxx_codec_in;
	emxx_codec_client.sett = &pcm_sett;

	ret = emxx_pcm_new(card, &emxx_codec_client, &emxx_codec);
	if (ret)
		goto err;

	ret = emxx_pcm_set_ctrl(emxx_codec_client.pcm_regs, &pcm_sett);
	if (ret)
		goto err;

	ret = emxx_codec_mixer_new(card);
	if (ret)
		goto err;

	snprintf(card->shortname, sizeof(card->shortname),
			"%s", "emxx-codec");
	snprintf(card->longname, sizeof(card->longname),
			"%s (%s)", "sound codec", card->mixername);

	ret = snd_card_register(card);
	if (ret == 0) {
#ifdef AUDIO_MAKING_DEBUG
		printk(KERN_INFO "Starting sound codec. with debug : M%d\n",
				debug);
#else
		printk(KERN_INFO "Starting sound codec.\n");
#endif
		platform_set_drvdata(devptr, card);
		FNC_EXIT return 0;
	}

err:
	if (card)
		snd_card_free(card);

	FNC_EXIT return ret;
}

static int emxx_codec_remove(struct platform_device *devptr)
{
	FNC_ENTRY
	emxx_pcm_free(&emxx_codec_client);
	snd_card_free(platform_get_drvdata(devptr));
	platform_set_drvdata(devptr, NULL);
	FNC_EXIT return 0;
}


static struct platform_driver emxx_codec_driver = {
	.probe          = emxx_codec_probe,
	.remove         = __devexit_p(emxx_codec_remove),
#ifdef CONFIG_PM
	.suspend        = emxx_codec_suspend,
	.resume         = emxx_codec_resume,
#endif
	.driver         = {
		.name   = "pcm",
	},
};

static int __init emxx_codec_init(void)
{
	return platform_driver_register(&emxx_codec_driver);
}

static void __exit emxx_codec_exit(void)
{
	platform_driver_unregister(&emxx_codec_driver);

}

#ifdef CODEC_TEST_DEBUG
EXPORT_SYMBOL(i2c_codec_read);
#endif /* CODEC_TEST_DEBUG */

module_init(emxx_codec_init);
module_exit(emxx_codec_exit);

MODULE_DESCRIPTION("sound codec driver for emxx chip");
MODULE_LICENSE("GPL");

