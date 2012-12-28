/*
 *  File Name       : emxx_mega.c
 *  Function        : MEGA for CAMERA I/F Driver
 *  Release Version : Ver 0.01
 *  Release Date    : 2010/11/05
 *
 *  Copyright (C) Renesas Electronics Corporation 2010
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
 *  Suite 330, Boston, MA 02111-1307, USA.
 *
 */
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <mach/spi.h>
#include <mach/gpio.h>
#include <mach/pmu.h>
#include <mach/smu.h>
#include <linux/kernel.h>

/* #define EMXX_CAM_MAKING_DEBUG */

#include "emxx_cam.h"
#include "emxx_ov767x.h"

#define DEV_NAME "emxx_ov767x"

#define CAM_DEBUG 0

#define X1R_NOT_ZERO 0

//#define EMXX_CAM_CE14X_MAKING_DEBUG

/*** DEBUG code by the making ->*/
#ifdef EMXX_CAM_CE14X_MAKING_DEBUG

int mega_debug = 10;

#include <linux/moduleparam.h>

#define FNC_ENTRY	\
	if (mega_debug == 1 || mega_debug >= 9) {	\
		printk(KERN_INFO "[++OV]:%s\n", __func__); \
	}

#define FNC_EXIT_N	\
	if (mega_debug == 1 || mega_debug >= 9) {	\
		printk(KERN_INFO "[--OV]:%s:%d\n", __func__, __LINE__); \
	}

#define FNC_EXIT(r)	\
	if (mega_debug == 1 || mega_debug >= 9) {	\
		printk(KERN_INFO "[--OV]:%d :%s:%d\n", r, __func__, __LINE__); \
	}
	
#define d0b(fmt, args...)	\
	{ \
		printk(KERN_INFO "%s:%d: " fmt, __func__, __LINE__, ## args); \
	}
#define d1b(fmt, args...)	\
	if (mega_debug == 1 || mega_debug >= 9) {	\
		printk(KERN_INFO "%s:%d: " fmt, __func__, __LINE__, ## args); \
	}
#define d2b(fmt, args...)	\
	if (mega_debug == 2 || mega_debug >= 9) {	\
		printk(KERN_INFO "%s:%d: " fmt, __func__, __LINE__, ## args); \
	}
#define d3b(fmt, args...)	\
	if (mega_debug == 3 || mega_debug >= 9) {	\
		printk(KERN_INFO " --M%d-- " fmt, debug, ## args); \
	}
#define d4b(fmt, args...)	\
	if (mega_debug == 4 || mega_debug >= 9) {	\
		printk(KERN_INFO " --M%d-- " fmt, debug, ## args); \
	}
#define d5b(fmt, args...)	\
	if (mega_debug == 5 || mega_debug >= 9) {	\
		printk(KERN_INFO " --M%d-- " fmt, debug, ## args); \
	}
#define d6b(fmt, args...)	\
	if (mega_debug == 6 || mega_debug >= 9) {	\
		printk(KERN_INFO " --M%d-- " fmt, debug, ## args); \
	}
	
#else
#define FNC_ENTRY do { } while (0);
#define FNC_EXIT_N  do { } while (0);
#define FNC_EXIT(r) do { } while (0);
#define d0b(fmt, args...) do { } while (0);
#define d1b(fmt, args...) do { } while (0);
#define d2b(fmt, args...) do { } while (0);
#define d3b(fmt, args...) do { } while (0);
#define d4b(fmt, args...) do { } while (0);
#define d5b(fmt, args...) do { } while (0);
#define d6b(fmt, args...) do { } while (0);
#endif



/*===============================================================*/
/* I2C Functions                                                 */
/*===============================================================*/

/* flags */
static int i2c_stop;

/* prottyped */
static int ov767x_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
static int ov767x_i2c_remove(struct i2c_client *client);
static inline int ov767x_i2c_write(unsigned char cmd,
				      unsigned char *buf, unsigned char len);
static inline int ov767x_i2c_read(unsigned char cmd,
				     unsigned char *buf, unsigned char len);

static struct i2c_device_id cam_i2c_idtable[] = {
	{ I2C_SLAVE_CAM_NAME_OV767x, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cam_i2c_idtable);

static struct i2c_driver ov767x_i2c_driver = {
	.driver.name    = "i2c for ov767x",
	.id_table       = cam_i2c_idtable,
	.probe          = ov767x_i2c_probe,
	.remove         = ov767x_i2c_remove,
};
static struct i2c_client *ov767x_i2c_client;


/* i2c registerd function */
static int ov767x_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	ov767x_i2c_client = client;
	return 0;
}

static int ov767x_i2c_remove(struct i2c_client *client)
{
	ov767x_i2c_client = NULL;
	return 0;
}

/* usable i2c funcrion */
static inline int ov767x_i2c_write(unsigned char cmd,
				      unsigned char *buf, unsigned char len)
{
	int ret = 0;
	char s_buf[15];
	struct i2c_msg msg = { .addr = I2C_SLAVE_CAM_ADDR_OV767x, .flags = 0,
			       .buf = s_buf, .len = len + 1 };

	s_buf[0] = cmd;
	memcpy(&s_buf[1], buf, len);

	//if (i2c_stop)
	//	return -EIO;

	assert(ov767x_i2c_client);

	ret = i2c_transfer(ov767x_i2c_client->adapter, &msg, 1);
	if (1 != ret) {
		err("i2c_transfer: cmd[Write] : 0x%02x ret = %d\n", cmd, ret);
		i2c_stop = 1;
		ret = -EIO;
	} else {
		ret = 0;
	}
	return ret;
}

static inline int ov767x_i2c_read(unsigned char cmd,
				     unsigned char *buf, unsigned char len)
{
	int ret = 0;
	struct i2c_msg msg[] =  {{ .addr = I2C_SLAVE_CAM_ADDR_OV767x, .flags = 0,
				   .buf = &cmd, .len = 1 },
				 { .addr = I2C_SLAVE_CAM_ADDR_OV767x, .flags = I2C_M_RD,
				   .buf = buf,  .len = len } };

	//printk("ov767x i2c addr:0x%x\n", msg[0].addr);
	//if (i2c_stop)
	//	return -EIO;

	assert(ov767x_i2c_client);

	ret = i2c_transfer(ov767x_i2c_client->adapter, msg, 2);
	if (2 != ret) {
		err("i2c_transfer: cmd : 0x%02x ret = %d\n", cmd, ret);
		i2c_stop = 1;
		ret = -EIO;
	} else {
		ret = 0;
	}
	return ret;
}

static int write_regs(const struct sensor_reg reglist[])
{
	int err;
	int err_cnt = 0;
	unsigned char data[132];
	unsigned char bytes;
	const struct sensor_reg *next = reglist;
	
	while (!((next->reg == REG_TERM) && (next->val == VAL_TERM)))
	{
		//msleep(2);
		bytes = 0;
		data[bytes]= next->reg&0xff; 	bytes++;
		data[bytes]= next->val&0xff;
		err = ov767x_i2c_write(data[0], &data[1], 1);
		if (next->reg == REG_COM7 && (next->val & COM7_RESET))
			msleep(2);  /* Wait for reset to run */
#if 0
		ov767x_i2c_read(data[0], &bytes, 1);
		if(bytes != data[1]) {
			printk("write %x err, read=%x<-------%x\n", data[0], bytes, data[1]);
		}
#endif
		if (err) {
			printk("write err, reg=%x<----%x\n", data[0], next->reg&0xff);
			err_cnt++;
			if(err_cnt >= 3) {
				printk("ERROR: Sensor I2C !!!! \n"); 
				return err;
			}
		}
		else {
			err_cnt = 0;
			next++;
		}
	}

	return 0;
}

/*===============================================================*/
/* ov767x Camera control flags                                */
/*===============================================================*/
enum {
	EMXX_ov767x_IDLE = 0,
	EMXX_ov767x_LIVE,
	EMXX_ov767x_CAPTURE,
	EMXX_ov767x_BREAK,
};

struct emxx_ov767x {
	int state;
	struct mutex lock;

	__u32 active:1;
	__u32 reset:1;

	__u8 firmware_version;
	__u8 capture_size;
	__u8 outimg_fmt;
	__u8 imgdata_fmt;
	__u8 flicker_manual;
	__u8 wb_mode;
	__u32 wb_manual_gain;
	__u8 brightness;
	__u8 contrast;
	__u32 sharpness;
	__u8 mirror;
	__u8 efct_color;
	__u8 efct_emboss;
	__u8 efct_negative;
	__u8 efct_sketch;
};

struct emxx_ov767x *ov767x;


/*===============================================================*/
/* ov767x data structure. for VIDIOC_S_CTRL                   */
/*===============================================================*/
struct control_menu_info {
	int value;
	char name[32];
};


struct control_resize_info {
	int value;
	char name[32];
	__s32 x;
	__s32 y;
	__s32 width;
	__s32 height;
};


/* EMXX_RJ6ABA100_CID_OUTIMG_SIZE */
static const struct control_resize_info ov767x_outimg_size_menus[] =
{
	{ 0x00, "VGA   : 640x480",  7, 7, 646, 486 },
};
#define NUM_OUTIMG_SIZE_MENUS ARRAY_SIZE(ov767x_outimg_size_menus)
#define BOUNDARY_OUT_IMG_RESIZE 0x01 /* 640x480 */


/* EMXX_RJ6ABA100_CID_OUTIMG_FORMAT */
#define OUT_FORMAT_YCBCR422 0x00
#define OUT_FORMAT_YUV422   0x01
#define OUT_FORMAT_RGB_RAW  0x02
#define OUT_FORMAT_RGB565   0x03
#define OUT_FORMAT_RGB444   0x04
#define OUT_FORMAT_MONO     0x05
static const struct control_menu_info ov767x_outimg_fmt_menus[] =
{
	{ 0x00, "YCbCr422",       },
	{ 0x01, "YUV422",         },     /* default */
	{ 0x02, "RGB Bayer(RAW)", },
	{ 0x03, "RGB565",         },
	{ 0x04, "RGB444",         },
	{ 0x05, "Mono",           },
};
#define NUM_OUTIMG_FORMAT_MENUS ARRAY_SIZE(ov767x_outimg_fmt_menus)


/* EMXX_RJ6ABA100_CID_OUTDATA_FORMAT */
static const struct control_menu_info
ov767x_outdata_fmt_menus[NUM_OUTIMG_FORMAT_MENUS][4] =
{
	{       /* YCbCr422 */
		{ 0x00, "CbYCrY...",   },
		{ 0x01, "CrYCbY...",   },
		{ 0x02, "YCbYCr...",   },
		{ 0x03, "YCrYCb...",   },
	}, {     /* YUV422 */
		{ 0x00, "UYVY...",     },        /* default */
		{ 0x01, "VYUY...",     },
		{ 0x02, "YUYV...",     },
		{ 0x03, "YVYU...",     },
	}, {     /* RGB Bayer */
		{ 0x04, "RGRG...GBGB", },
		{ 0x05, "GBGB...RGRG", },
		{ 0x06, "GRGR...BGBG", },
		{ 0x07, "BGBG...GRGR", },
	}, {     /* RGB565 */
		{ 0x08, "R5G6B5...",   },
		{ 0x09, "B5G6R5...",   },
		{ 0xFF, "Reserved",   },
		{ 0xFF, "Reserved",   },
	}, {     /* RGB444 */
		{ 0x0E, "R4G4B4...",   },
		{ 0xFF, "Reserved",   },
		{ 0xFF, "Reserved",   },
		{ 0xFF, "Reserved",   },
	}, {     /* Mono */
		{ 0x0D, "YYYY...",     },
		{ 0xFF, "Reserved",   },
		{ 0xFF, "Reserved",   },
		{ 0xFF, "Reserved",   },
	}
};
#define NUM_OUTDATA_FORMAT_MENUS ARRAY_SIZE(ov767x_outdata_fmt_menus)


/* EMXX_RJ6ABA100_CID_MIRROR */
static const struct control_menu_info ov767x_mirror_menus[] =
{
	{ 0x00, "Mirror Off"                       }, /* default */
	{ 0x01, "Mirror Horizontally"              },
	{ 0x02, "Mirror Vertically"                },
	{ 0x03, "Mirror Horizontally & Vertically" },
};
#define NUM_MIRROR_MENUS ARRAY_SIZE(ov767x_mirror_menus)

/*===============================================================*/
/* ov767x data structure. for VIDIOC_QUERYCTRL                */
/*===============================================================*/
static const struct v4l2_queryctrl no_ctrl = {
	.name  = "ov767x",
	.flags = V4L2_CTRL_FLAG_DISABLED,
};

/* default settings */
static const struct v4l2_queryctrl ov767x_ctrls[] = {
	{
		.id            = EMXX_RJ6ABA100_CID_OUTIMG_SIZE,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "Output image size",
#if 0
/* @@		.minimum       = 0, */
/* @@		.maximum       = (NUM_OUTIMG_SIZE_MENUS - 1), */
#else /* VGA only */
		.minimum       = 0,
		.maximum       = (NUM_OUTIMG_SIZE_MENUS - 1),
#endif
		.step          = 1,
		.default_value = 0, /* VGA */
		.flags         = 0,
	}, {
		.id            = EMXX_RJ6ABA100_CID_OUTIMG_FORMAT,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "Output image Format",
#if 0
/* @@		.minimum       = 0, */
/* @@		.maximum       = (NUM_OUTIMG_FORMAT_MENUS - 1), */
#else /* YUV422 only */
		.minimum       = 1,
		.maximum       = 1,
#endif
		.step          = 1,
		.default_value = 1, /* YUV422 */
		.flags         = 0,
	}, {
		.id            = EMXX_RJ6ABA100_CID_OUTDATA_FORMAT,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "Output data Format",
		.minimum       = 0,
#if 0
/* @@		.maximum       = (NUM_OUTDATA_FORMAT_MENUS
   / NUM_OUTIMG_FORMAT_MENUS - 1), */
#else /* UYVY... only */
		.maximum       = 0,
#endif
		.step          = 1,
		.default_value = 0, /* UYVY... */
		.flags         = 0,
	}, {
		.id            = EMXX_RJ6ABA100_CID_MIRROR,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "Mirror Horizontally & Vertically",
		.minimum       = 0,
		.maximum       = (NUM_MIRROR_MENUS - 1),
		.step          = 1,
		.default_value = 0, /* Mirror Off */
		.flags         = 0,
	}, 
};
#define NUM_ov767x_CTRLS ARRAY_SIZE(ov767x_ctrls)


/*===============================================================*/
/* ov767x register data structure                             */
/*===============================================================*/
struct register_info {
	unsigned char bank;
	unsigned char address;
	unsigned char data;
	unsigned char mask;
};


/* ov767x register groupe : output image format */
static const struct register_info
ov767x_register_outimg_fmt[NUM_OUTIMG_FORMAT_MENUS][9] =
{
	{       /* YCbCr422 */
		{0x01, 0x49, 0xE0, 0xFF},       {0x01, 0x4A, 0x37, 0xFF},
		{0x01, 0x4B, 0x10, 0xFF},       {0x01, 0x4C, 0xEB, 0xFF},
		{0x01, 0x0C, 0xFF, 0xFF},       {0x01, 0x0D, 0x03, 0x03},
		{0x01, 0x0E, 0x01, 0x01},       {0x01, 0x0F, 0x08, 0x08},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* YUV422 */
		{0x01, 0x49, 0xFF, 0xFF},       {0x01, 0x4A, 0x40, 0xFF},
		{0x01, 0x4B, 0x01, 0xFF},       {0x01, 0x4C, 0xFE, 0xFF},
		{0x01, 0x0C, 0xFF, 0xFF},       {0x01, 0x0D, 0x03, 0x03},
		{0x01, 0x0E, 0x01, 0x01},       {0x01, 0x0F, 0x08, 0x08},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* RGB Bayer(RAW) */
		{0x01, 0x49, 0xFF, 0xFF},       {0x01, 0x4A, 0x40, 0xFF},
		{0x01, 0x4B, 0x01, 0xFF},       {0x01, 0x4C, 0xFE, 0xFF},
		{0x01, 0x0C, 0x43, 0xFF},       {0x01, 0x0D, 0x00, 0x03},
		{0x01, 0x0E, 0x00, 0x01},       {0x01, 0x0F, 0x00, 0x08},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* RGB565 */
		{0x01, 0x49, 0xFF, 0xFF},       {0x01, 0x4A, 0x40, 0xFF},
		{0x01, 0x4B, 0x01, 0xFF},       {0x01, 0x4C, 0xFE, 0xFF},
		{0x01, 0x0C, 0xFF, 0xFF},       {0x01, 0x0D, 0x03, 0x03},
		{0x01, 0x0E, 0x01, 0x01},       {0x01, 0x0F, 0x08, 0x08},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* RGB444 */
		{0x01, 0x49, 0xFF, 0xFF},       {0x01, 0x4A, 0x40, 0xFF},
		{0x01, 0x4B, 0x01, 0xFF},       {0x01, 0x4C, 0xFE, 0xFF},
		{0x01, 0x0C, 0xFF, 0xFF},       {0x01, 0x0D, 0x03, 0x03},
		{0x01, 0x0E, 0x01, 0x01},       {0x01, 0x0F, 0x08, 0x08},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* Mono */
		{0x01, 0x49, 0xFF, 0xFF},       {0x01, 0x4A, 0x40, 0xFF},
		{0x01, 0x4B, 0x01, 0xFF},       {0x01, 0x4C, 0xFE, 0xFF},
		{0x01, 0x0C, 0xFF, 0xFF},       {0x01, 0x0D, 0x03, 0x03},
		{0x01, 0x0E, 0x01, 0x01},       {0x01, 0x0F, 0x08, 0x08},
		{0x00, 0x00, 0x00, 0x00},
	}
};
#define NUM_OUTIMG_FORMAT_REGS ARRAY_SIZE(ov767x_register_outimg_fmt)


/* ov767x register groupe : output image format & output image size */
#define NUM_PCLK_RATE_RGBRAW_MONO   0  /* RGB Bayer, Mono */
#define NUM_PCLK_RATE_YCBCR_YUV_RGB 1  /* YCbCr422, YUV422, RGB565 */
#define NUM_PCLK_RATE_MAX           2
#define SELECT_PCLK_RATE_DESCRIPTION(format) ((format == OUT_FORMAT_RGB_RAW \
					       || format == OUT_FORMAT_MONO) \
					      ? NUM_PCLK_RATE_RGBRAW_MONO \
					      : NUM_PCLK_RATE_YCBCR_YUV_RGB)


/* EMXX_RJ6ABA100_CID_MIRROR */
static const struct register_info
ov767x_register_mirror[NUM_MIRROR_MENUS + 1][3] =
{
	{       /* Mirror Off */
		{0x01, 0x19, 0x60, 0x60},       {0x01, 0x0F, 0x00, 0x03},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* Mirror Horizontally */
		{0x01, 0x19, 0x60, 0x60},       {0x01, 0x0F, 0x01, 0x03},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* Mirror Virtically */
		{0x01, 0x19, 0x60, 0x60},       {0x01, 0x0F, 0x02, 0x03},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* Mirror Horizontally & Vertically */
		{0x01, 0x19, 0x60, 0x60},       {0x01, 0x0F, 0x03, 0x03},
		{0x00, 0x00, 0x00, 0x00},
	}, {     /* Common Postprocessing */
		{0x01, 0x19, 0x00, 0x60},       {0x00, 0x00, 0x00, 0x00},
		{0x00, 0x00, 0x00, 0x00},
	}
};
#define NUM_MIRROR ARRAY_SIZE(ov767x_register_mirror)

/*===============================================================*/
/* ov767x Camera control function                             */
/*===============================================================*/

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov767x_chkreg_bank
 * RETURN   :
 * NOTE     : check & change ov767x register bank
 * UPDATE   :
 ******************************************************************************/
#define NUM_B03_REGBANK_A  0x00
#define NUM_B03_REGBANK_B  0x01
#define NUM_B03_REGBANK_C  0x02
#define NUM_B03_REGBANK_D  0x03

static inline int ov767x_chkreg_bank(__u8 value)
{
#if 0
	int ret;
	unsigned char bank;

	ret = ov767x_i2c_read(0x03, &bank, 1); /* get bank */
	if (!ret) {
		if (bank != value)
			ret = ov767x_i2c_write(0x03, &value, 1);
			/* set bank */
	}
	return ret;
#endif
	return 0;
}


#if CAM_DEBUG
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mage_register_debug
 * RETURN   :
 * NOTE     : check ov767x register (debug only)
 * UPDATE   :
 ******************************************************************************/
static inline void emxx_mage_register_debug(void)
{
	unsigned char buf[16];
	int i, j, bank, ret;

	for (bank = NUM_B03_REGBANK_B; bank <= NUM_B03_REGBANK_D; bank++) {
		ret = ov767x_chkreg_bank(bank);
		if (ret)
			return;

		printk(KERN_DEBUG"========================================"
		       "========================\n");
		printk(KERN_DEBUG" Group %c\n",
		       ((bank == NUM_B03_REGBANK_B) ? 'B'
			: ((bank == NUM_B03_REGBANK_C) ? 'C' : 'D')));
		printk(KERN_DEBUG"-----------------------------------------"
		       "-----------------------\n");

		for (i = 0; i < 0xFF; i += 16) {
			for (j = 0; j < 16; j++) {
				ret = ov767x_i2c_read(i + j, &buf[j], 1);
				if (ret)
					return;
			}

			printk(KERN_DEBUG" %c-%02x:  %02x %02x %02x %02x  "
			       "%02x %02x %02x %02x  %02x %02x "
			       "%02x %02x  %02x %02x %02x %02x\n",
			       ((bank == NUM_B03_REGBANK_B) ? 'B'
				: ((bank == NUM_B03_REGBANK_C) ? 'C' : 'D')),
			       i, buf[0], buf[1], buf[2], buf[3], buf[4],
			       buf[5], buf[6], buf[7], buf[8], buf[9],
			       buf[10], buf[11], buf[12], buf[13], buf[14],
			       buf[15]);
		}
	}
	printk(KERN_DEBUG"================================================"
	       "================\n");

}
#endif /* CAM_DEBUG */


/*****************************************************************************
 * MODULE   : emxx_mega
 * FUNCTION : ov767x_setreg_array
 * RETURN   :
 * NOTE     : set ov767x register group
 * UPDATE   :
 ******************************************************************************/
static inline int ov767x_setreg_array(struct register_info *reg_info)
{
	return 0;
#if 0
	int i, ret = 0;
	unsigned char data;

	for (i = 0; reg_info[i].bank != 0x00; i++) {
		ret = ov767x_chkreg_bank(reg_info[i].bank); /* check bank */
		if (!ret) {
			ret = ov767x_i2c_read(reg_info[i].address, &data, 1);
			if (!ret) {
				data = (data & ~reg_info[i].mask)
					| reg_info[i].data;
				ret  = ov767x_i2c_write(reg_info[i].address,
							   &data, 1);
			}
		}
	}
	return ret;
#endif
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov767x_setreg_pll_control
 * RETURN   :
 * NOTE     : set ov767x PLL control register group
 * UPDATE   :
 ******************************************************************************/
#define NUM_B09_PLL_CTRL01      0x09
#define NUM_B0A_PLL_CTRL02      0x0A
#define NUM_B0B_PLL_CTRL03      0x0B
#define NUM_B09_PLL_CTRL01_MASK 0x30
#define NUM_B0A_PLL_CTRL02_MASK 0xFF
#define NUM_B0B_PLL_CTRL03_MASK 0xFF

#define NUM_B09_PLL_CTRL01_X10  0x00
#define NUM_B09_PLL_CTRL01_X12  0x10
#define NUM_B09_PLL_CTRL01_X14  0x20
#define NUM_B09_PLL_CTRL01_X16  0x30

#define NUM_B0A_PLL_CTRL02_X1   0x00
#define NUM_B0A_PLL_CTRL02_X2   0x01
#define NUM_B0A_PLL_CTRL02_X3   0x02
#define NUM_B0A_PLL_CTRL02_X4   0x03
#define NUM_B0A_PLL_CTRL02_X5   0x04
#define NUM_B0A_PLL_CTRL02_X6   0x05
#define NUM_B0A_PLL_CTRL02_X7   0x06
#define NUM_B0A_PLL_CTRL02_X8   0x07

#define NUM_B0B_PLL_CTRL03_X1   0x00
#define NUM_B0B_PLL_CTRL03_X2   0x01
#define NUM_B0B_PLL_CTRL03_X3   0x02
#define NUM_B0B_PLL_CTRL03_X4   0x03
#define NUM_B0B_PLL_CTRL03_X5   0x04
#define NUM_B0B_PLL_CTRL03_X6   0x05
#define NUM_B0B_PLL_CTRL03_X7   0x06
#define NUM_B0B_PLL_CTRL03_X8   0x07

static inline int ov767x_setreg_pll_control(int arg)
{
	return 0;
#if 0
	int i, ret;
	unsigned char add[3], buf[3];

	ret = ov767x_chkreg_bank(NUM_B03_REGBANK_B); /* check bank B */
	if (!ret) {
		ret  = ov767x_i2c_read(NUM_B0A_PLL_CTRL02, &buf[0], 1);
		ret |= ov767x_i2c_read(NUM_B09_PLL_CTRL01, &buf[1], 1);
		ret |= ov767x_i2c_read(NUM_B0B_PLL_CTRL03, &buf[2], 1);
		if (!ret) {
			/* camera module clock setting */
			add[0] = NUM_B0A_PLL_CTRL02;
			add[1] = NUM_B09_PLL_CTRL01;
			add[2] = NUM_B0B_PLL_CTRL03;

			if (arg == 344) {
				buf[0] = (buf[0] &
				 (unsigned char)~NUM_B0A_PLL_CTRL02_MASK)
					| NUM_B0A_PLL_CTRL02_X2;
				/* x1/2 : 11.47/2 => 5.74    */
				buf[1] = (buf[1] &
				 (unsigned char)~NUM_B09_PLL_CTRL01_MASK)
					| NUM_B09_PLL_CTRL01_X12;
				/* x12  : 5.74x12 => 68.8    */
				buf[2] = (buf[2] &
				 (unsigned char)~NUM_B0B_PLL_CTRL03_MASK)
					| NUM_B0B_PLL_CTRL03_X2;
				/* x1/2 : 68.8/2  => 34.4Mhz */
			} else if (arg == 574)    {
				buf[0] = (buf[0] &
				 (unsigned char)~NUM_B0A_PLL_CTRL02_MASK)
					| NUM_B0A_PLL_CTRL02_X2;
				/* x1/2 : 11.47/2 => 5.74    */
				buf[1] = (buf[1] &
				 (unsigned char)~NUM_B09_PLL_CTRL01_MASK)
					| NUM_B09_PLL_CTRL01_X10;
				/* x10  : 5.74x10 => 57.4    */
				buf[2] = (buf[2] &
				 (unsigned char)~NUM_B0B_PLL_CTRL03_MASK)
					| NUM_B0B_PLL_CTRL03_X1;
				/* x1/1 : 57.4/1  => 57.4Mhz */
			} else if (arg == 688)    {
				buf[0] = (buf[0] &
				 (unsigned char)~NUM_B0A_PLL_CTRL02_MASK)
					| NUM_B0A_PLL_CTRL02_X2;
				/* x1/2 : 11.47/2 => 5.74    */
				buf[1] = (buf[1] &
				 (unsigned char)~NUM_B09_PLL_CTRL01_MASK)
					| NUM_B09_PLL_CTRL01_X12;
				/* x12  : 5.74x12 => 68.8    */
				buf[2] = (buf[2] &
				 (unsigned char)~NUM_B0B_PLL_CTRL03_MASK)
					| NUM_B0B_PLL_CTRL03_X1;
				/* x1/1 : 68.8/1  => 68.8Mhz */
			}

			for (i = 0; i < ARRAY_SIZE(buf); i++) {
				ret = ov767x_i2c_write(add[i], &buf[i], 1);
				if (ret)
					return ret;
			}
		}
	}
	return ret;
#endif
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov767x_setreg_outimg_size
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_OUTIMG_SIZE
 *          : set ov767x output image size register group
 * UPDATE   :
 ******************************************************************************/
static inline int ov767x_setreg_outimg_size(__u8 value)
{
	//Only support VGA
	return 0;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov767x_setreg_outimg_fmt
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_OUTIMG_FORMAT
 *          : set ov767x output image format register group
 * UPDATE   :
 ******************************************************************************/
static inline int ov767x_setreg_outimg_fmt(__u8 value)
{
	return ov767x_setreg_array(
	      (struct register_info *)(&ov767x_register_outimg_fmt[value]));
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov767x_setreg_outdata_fmt
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_OUTDATA_FORMAT
 *          : set ov767x output data array format register group
 * UPDATE   :
 ******************************************************************************/
#define NUM_B18_FORMAT       0x18
#define NUM_B18_FORMAT_MASK  0xFF
#define NUM_B18_FORMAT_SFT   0x00

static inline int ov767x_setreg_outdata_fmt(__u8 value)
{
	return 0;
#if 0
	char buf;
	int ret;

	ret = ov767x_chkreg_bank(NUM_B03_REGBANK_B); /* check bank B */
	if (!ret) {
		ret = ov767x_i2c_read(NUM_B18_FORMAT, &buf, 1);
		if (!ret) {
			buf = (buf & ~NUM_B18_FORMAT_MASK) | value;
			ret = ov767x_i2c_write(NUM_B18_FORMAT, &buf, 1);
			/* OutputFormat */
		}
	}
	return ret;
#endif
}

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov767x_setreg_mirror
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_MIRROR
 *          : set ov767x Mirror register
 * UPDATE   :
 ******************************************************************************/
static inline int ov767x_setreg_mirror(__u8 value)
{
	int ret;

	/* Each Priprocessing */
	ret = ov767x_setreg_array(
		(struct register_info *)(&ov767x_register_mirror[value]));
	if (!ret) {
		/* Common Postprocessing */

		/*
		 * wait 1frame
		 */
		msleep(30);

		ret = ov767x_setreg_array(
			(struct register_info *)
			(&ov767x_register_mirror[NUM_MIRROR_MENUS]));
	}
	return ret;
}

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov767x_setreg_softreset
 * RETURN   :
 * NOTE     : set ov767x register SoftReset
 * UPDATE   :
 ******************************************************************************/

static inline int ov767x_setreg_softreset(__u8 value)
{
	write_regs(sensor_initialize);
	return 0;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov767x_getreg_firmware_version
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_FW
 *          : get ov767x register revesion number
 * UPDATE   :
 ******************************************************************************/
#define NUM_B02_REVNUMBER  0x02

static inline int ov767x_getreg_firmware_version(__u8 *val)
{
	int ret;
	char v;
	ret = ov767x_i2c_read(REG_VER, &v, 1);
	if( (ret!=0) || (v != 0x73) )/* PID + VER = 0x76 / 0x73 */
		return -ENODEV;
	*val = v;
	return 0;
}

/* etc */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov767x_set_liveview
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
#define SET_LIVEVIEW_STOP  0
#define SET_LIVEVIEW_START 1

static inline int ov767x_set_liveview(__u8 val)
{
	if (val == SET_LIVEVIEW_START) {
		write_regs(sensor_set_preview);
		ov767x->state = EMXX_ov767x_LIVE;
	} else {
		write_regs(sensor_set_preview);
		ov767x->state = EMXX_ov767x_IDLE;
	}
	return 0;
}

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov767x_set_capture_read
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
static inline int ov767x_set_capture_read(__u8 data)
{
	int ret = 0;

	if (EMXX_ov767x_IDLE != ov767x->state)
		return -EBUSY;

	ret = ov767x_setreg_outimg_size(data);
	if (!ret)
		ov767x->state = EMXX_ov767x_CAPTURE;

	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov767x_shutdown
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
static inline int ov767x_shutdown(void)
{
	int ret = 0;

	/* ov767x power off
	 */
	gpio_direction_output(GPIO_P121, 1);

	return ret;
}





/* EMXX_RJ6ABA100_CID_FW */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov767x_get_firmware_version
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_FW
 * UPDATE   :
 ******************************************************************************/
static inline int ov767x_get_firmware_version(__u8 *value)
{
	*value = ov767x->firmware_version;
	return 0;
}


/* EMXX_RJ6ABA100_CID_OUTIMG_SIZE */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov767x_get_outimg_size / ov767x_set_outimg_size
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_FW
 * UPDATE   :
 ******************************************************************************/
static inline int ov767x_get_outimg_size(__u8 *value)
{
	//printk("[%s:%d] size=%d\n", __func__, __LINE__, ov767x->capture_size);
	*value = ov767x->capture_size;
	return 0;
}

static inline int ov767x_set_outimg_size(__u8 value)
{
	int ret = 0;
	printk("[%s:%d] set size=%d\n", __func__, __LINE__, value);
	if (value != ov767x->capture_size) {
		ov767x->capture_size = value;
		ov767x->reset = 1;
	}

	return ret;
}

/* EMXX_RJ6ABA100_CID_OUTIMG_FORMAT */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov767x_get_outimg_fmt / ov767x_set_outimg_fmt
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_FW
 * UPDATE   :
 ******************************************************************************/
static inline int ov767x_get_outimg_fmt(__u8 *value)
{
	printk("[%s:%d] fmt=%d\n", __func__, __LINE__, ov767x->outimg_fmt);
	*value = ov767x->outimg_fmt;
	return 0;
}

static inline int ov767x_set_outimg_fmt(__u8 value)
{
	int ret = 0;
	printk("[%s:%d] set fmt=%d\n", __func__, __LINE__, value);
	if (value != ov767x->outimg_fmt)
		ov767x->outimg_fmt = value;

	return ret;
}

/* EMXX_RJ6ABA100_CID_OUTDATA_FORMAT */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov767x_get_imgdata_fmt / ov767x_set_imgdata_fmt
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_FW
 * UPDATE   :
 ******************************************************************************/
static inline int ov767x_get_imgdata_fmt(__u8 *value)
{
	printk("[%s:%d] fmt=%d\n", __func__, __LINE__, ov767x->imgdata_fmt);
	*value = ov767x->imgdata_fmt;
	return 0;
}

static inline int ov767x_set_imgdata_fmt(__u8 value)
{
	int ret = 0;
	printk("[%s:%d] set fmt=%d\n", __func__, __LINE__, value);
	if (value != ov767x->imgdata_fmt)
		ov767x->imgdata_fmt = value;

	return ret;
}

/* EMXX_RJ6ABA100_CID_MIRROR */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov767x_get_mirror / ov767x_set_mirror
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_MIRROR
 * UPDATE   :
 ******************************************************************************/
static inline int ov767x_get_mirror(__u8 *value)
{
	*value = ov767x->mirror;
	return 0;
}

static inline int ov767x_set_mirror(__u8 value)
{
	int ret = 0;

	ret = ov767x_setreg_mirror(value);
	if (!ret)
		ov767x->mirror = value;
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov767x_start_capture_read
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
static inline int ov767x_start_capture_read(void)
{
	int ret = 0;

	ret = ov767x_set_capture_read(ov767x->capture_size);

	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ctrl_by_id
 * RETURN   :
 * NOTE     : get ov767x_ctrls id
 * UPDATE   :
 ******************************************************************************/
static const struct v4l2_queryctrl *ctrl_by_id(unsigned int id)
{
	unsigned int i;

	for (i = 0; i < NUM_ov767x_CTRLS; i++)
		if (ov767x_ctrls[i].id == id)
			return ov767x_ctrls + i;
	return NULL;
}


/* i/f function with emxx_cam.c */

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_vidioc_queryctrl
 * RETURN   :
 * NOTE     : called from emxx_cam.c ioctl(VIDIOC_QUERYCTRL)
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_vidioc_queryctrl(struct file *file,
				       void *fh, struct v4l2_queryctrl *a)
{
	int ret = 0;
	const struct v4l2_queryctrl *ctrl;
	FNC_ENTRY;

	if (a->id < EMXX_RJ6ABA100_CID_FW
	    || a->id > EMXX_RJ6ABA100_CID_EFFECT_SKETCH_MANUAL) {
		ret = -EINVAL;
	} else{
		ctrl = ctrl_by_id(a->id);
		*a = (NULL != ctrl) ? *ctrl : no_ctrl;
	}

	FNC_EXIT(ret)
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_vidioc_querymenu
 * RETURN   :
 * NOTE     : called from emxx_cam.c ioctl(VIDIOC_QUERYMENU)
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_vidioc_querymenu(struct file *file,
				       void *fh, struct v4l2_querymenu *m)
{
	const struct v4l2_queryctrl *ctrl;
	int ret = 0;
	FNC_ENTRY;

	ctrl = ctrl_by_id(m->id);
	if (NULL == ctrl) {
		FNC_EXIT(-EINVAL)
		return -EINVAL;
	}
	switch (ctrl->type) {
	case V4L2_CTRL_TYPE_MENU:
		if (m->index < ctrl->minimum || m->index > ctrl->maximum)
			ret = -EINVAL;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	if (ret) {
		FNC_EXIT(ret)
		return ret;
	}

	mutex_lock(&ov767x->lock);
	switch (m->id) {
	case EMXX_RJ6ABA100_CID_OUTIMG_SIZE:
		strcpy(m->name, ov767x_outimg_size_menus[m->index].name);
		break;
	case EMXX_RJ6ABA100_CID_OUTIMG_FORMAT:
		strcpy(m->name, ov767x_outimg_fmt_menus[m->index].name);
		break;
	case EMXX_RJ6ABA100_CID_OUTDATA_FORMAT:
		strcpy(m->name,
		       ov767x_outdata_fmt_menus
		       [ov767x->outimg_fmt][m->index].name);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	FNC_EXIT(ret)
	mutex_unlock(&ov767x->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_vidioc_g_ctrl
 * RETURN   :
 * NOTE     : called from emxx_cam.c ioctl(VIDIOC_G_CTRL)
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_vidioc_g_ctrl(struct file *file, void *fh,
				    struct v4l2_control *c)
{
	__u32 val = 0;
	int ret = 0;
	const struct v4l2_queryctrl *ctrl;
	FNC_ENTRY;

	ctrl = ctrl_by_id(c->id);
	if (NULL == ctrl) {
		FNC_EXIT(-EINVAL)
		return -EINVAL;
	}
	mutex_lock(&ov767x->lock);
	switch (c->id) {
	case EMXX_RJ6ABA100_CID_FW:
		ret = ov767x_get_firmware_version((__u8 *)&val);
		break;
	case EMXX_RJ6ABA100_CID_OUTIMG_SIZE:
		ret = ov767x_get_outimg_size((__u8 *)&val);
		break;
	case EMXX_RJ6ABA100_CID_OUTIMG_FORMAT:
		ret = ov767x_get_outimg_fmt((__u8 *)&val);
		break;
	case EMXX_RJ6ABA100_CID_OUTDATA_FORMAT:
		ret = ov767x_get_imgdata_fmt((__u8 *)&val);
		break;
	default:
		ret = -EINVAL;
	}

	if (!ret)
		c->value = val;

	FNC_EXIT(ret)
	mutex_unlock(&ov767x->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_vidioc_s_ctrl
 * RETURN   :
 * NOTE     : called from emxx_cam.c ioctl(VIDIOC_S_CTRL)
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_vidioc_s_ctrl(struct file *file, void *fh,
				    struct v4l2_control *c)
{
	int ret = 0;
	int moving = (int)fh;
	const struct v4l2_queryctrl *ctrl;
	FNC_ENTRY;

	ctrl = ctrl_by_id(c->id);
	if (NULL == ctrl) {
		FNC_EXIT(-EINVAL)
		return -EINVAL;
	}
	switch (ctrl->type) {
	case V4L2_CTRL_TYPE_BOOLEAN:
	case V4L2_CTRL_TYPE_MENU:
	case V4L2_CTRL_TYPE_INTEGER:
#if 0
		if (c->value < ctrl->minimum)
			c->value = ctrl->minimum;
		if (c->value > ctrl->maximum)
			c->value = ctrl->maximum;
#else
		if (c->id == EMXX_RJ6ABA100_CID_EFFECT_SKETCH_MANUAL) {
			if ((__u32)c->value < (__u32)ctrl->minimum)
				c->value = ctrl->minimum;
			if ((__u32)c->value > (__u32)ctrl->maximum)
				c->value = ctrl->maximum;
		} else{
			if (c->value < ctrl->minimum)
				c->value = ctrl->minimum;
			if (c->value > ctrl->maximum)
				c->value = ctrl->maximum;
		}
#endif
		break;
	default:
		/* nothing */;
	};
	mutex_lock(&ov767x->lock);
	switch (c->id) {
#if 0 /* READ ONLY */
	case EMXX_RJ6ABA100_CID_FW:
		break;
#endif
	case EMXX_RJ6ABA100_CID_OUTIMG_SIZE:
		if (moving) {
			warn("ov767x : s_ctrl OUT_IMG_RESIZE :"
			     "streaming already exists.\n");
			ret = -EBUSY;
			break;
		}
		ret = ov767x_set_outimg_size(c->value);
		break;
	case EMXX_RJ6ABA100_CID_OUTIMG_FORMAT:
		ret = ov767x_set_outimg_fmt(c->value);
		break;
	case EMXX_RJ6ABA100_CID_OUTDATA_FORMAT:
		ret = ov767x_set_imgdata_fmt(c->value);
		break;
	default:
		ret = -EINVAL;
	}

	FNC_EXIT(ret)
	mutex_unlock(&ov767x->lock);
	return ret;
}

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov767x_set_prepare
 * RETURN   :
 * NOTE     : called from emxx_cam.c
 * UPDATE   :
 ******************************************************************************/
static inline int ov767x_set_prepare(struct emxx_cam_prepare *prepare)
{
	FNC_ENTRY;

	prepare->syncmode = 0;/* CAM_HS, CAM_VS mode */
	prepare->synctype = 0;/* Enable signal sampling mode */
	prepare->data_id  = 0;/* U->Y->V->Y */
	prepare->vs_det   = 0;/* rising edge */
	prepare->hs_det   = 0;/* rising edge */
	prepare->clk_edge = 0;/* single edge transfer */
	prepare->data_det = 0;/* rising edge */
	prepare->vs_pol   = 0;/* positive logic */
	prepare->hs_pol   = 0;/* positive logic */

	FNC_EXIT(0)
	return 0;
}

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_sync
 * RETURN   :
 * NOTE     : called from emxx_cam.c
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_sync(struct emxx_cam_prepare *prepare)
{
	int ret = 0;
	__u8 size;
	FNC_ENTRY;
	mutex_lock(&ov767x->lock);

	ret = ov767x_get_outimg_size(&size);
	if (ret) {
		FNC_EXIT(ret)
		mutex_unlock(&ov767x->lock);
		return ret;
	}

	if (BOUNDARY_OUT_IMG_RESIZE < size) {
		struct control_resize_info *s;

		if (EMXX_ov767x_BREAK == ov767x->state) {
			FNC_EXIT(-EIO)
			mutex_unlock(&ov767x->lock);
			return -EIO;
		}

		ret = ov767x_get_outimg_size(&size);
		if (ret) {
			FNC_EXIT(ret)
			mutex_unlock(&ov767x->lock);
			return ret;
		}

		s = (struct control_resize_info *)ov767x_outimg_size_menus;
		s += size;

#if X1R_NOT_ZERO
		prepare->bounds.left   = s->x;
		prepare->bounds.top    = s->y;
		prepare->bounds.width  = s->width  - (s->x);
		prepare->bounds.height = s->height - (s->y );

		prepare->c = prepare->bounds;
		prepare->width  = prepare->c.width ;
		prepare->height = prepare->c.height;	
#else
		prepare->bounds.left   = 0;
		prepare->bounds.top    = 0;
		prepare->bounds.width  = s->width  - (s->x - 1);
		prepare->bounds.height = s->height - (s->y - 1);

		prepare->c = prepare->bounds;
		prepare->width  = prepare->c.width  - (prepare->c.left - 1);
		prepare->height = prepare->c.height - (prepare->c.top  - 1);

#endif

		if (!prepare->actions) {
			FNC_EXIT(ret)
			mutex_unlock(&ov767x->lock);
			return ret;
		}

		prepare->actions = 1;

		#if 0
		ret = ov767x_set_liveview(SET_LIVEVIEW_START);
		#endif
	} else {
		prepare->actions = 0;
	}

	FNC_EXIT(ret)
	mutex_unlock(&ov767x->lock);
	return ret;
}

static int sensor_detect(void)
{
	int ret;
	unsigned char v;

	ret = ov767x_i2c_read(REG_MIDH, &v, 1);
	//printk("sensor:mid_h=0x%x.\n", v);
	if (v != 0x7f) /* OV manuf. id. */
		return -ENODEV;
	ov767x_i2c_read(REG_MIDL, &v, 1);
	//printk("sensor:mid_l=0x%x.\n", v);
	if (v != 0xa2)
		return -ENODEV;
	/*
	 * OK, we know we have an OmniVision chip...but which one?
	 */
	ov767x_i2c_read(REG_PID, &v, 1);
	//printk("sensor:pid=0x%x.\n", v);
	if (v != 0x76)  /* PID + VER = 0x76 / 0x73 */
		return -ENODEV;
	ov767x_i2c_read(REG_VER, &v, 1);
	//printk("sensor:ver=0x%x.\n", v);
	if (v != 0x73)  /* PID + VER = 0x76 / 0x73 */
		return -ENODEV;
	return 0;
}

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_prepare
 * RETURN   :
 * NOTE     : called from emxx_cam.c
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_prepare(struct emxx_cam_prepare *prepare)
{
	__u8 index;
	int ret = 0;
	FNC_ENTRY;

	mutex_lock(&ov767x->lock);

	ret = ov767x_get_outimg_size(&index);
	if (!ret) {
		struct control_resize_info *s;

		s = (struct control_resize_info *)ov767x_outimg_size_menus;
		s += index;

		#if X1R_NOT_ZERO
		prepare->bounds.left   = s->x;
		prepare->bounds.top    = s->y;
		prepare->bounds.width  = s->width  - (s->x );
		prepare->bounds.height = s->height - (s->y );
		
		#else
		prepare->bounds.left   = 0;
		prepare->bounds.top    = 0;
		prepare->bounds.width  = s->width  - (s->x - 1);
		prepare->bounds.height = s->height - (s->y - 1);
		
		#endif

		ov767x_set_prepare(prepare);

		prepare->reset = ov767x->reset;
		d1b("chk reset 0x%02x\n", ov767x->reset);

		if (prepare->actions && !ov767x->reset
		    && BOUNDARY_OUT_IMG_RESIZE < index)
			ov767x->state = EMXX_ov767x_IDLE;

		ov767x->reset = 0;
	}

	FNC_EXIT(ret)
	mutex_unlock(&ov767x->lock);
	return ret;
}

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_trigger
 * RETURN   :
 * NOTE     : called from emxx_cam.c
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_trigger(int flag)
{
	__u8 size;
	int ret = 0;
	FNC_ENTRY;

	mutex_lock(&ov767x->lock);

	ret = ov767x_get_outimg_size(&size);
	if (ret) {
		FNC_EXIT(ret)
		mutex_unlock(&ov767x->lock);
		return ret;
	}

	if (BOUNDARY_OUT_IMG_RESIZE < size) {
		write_regs(sensor_set_capture);
		ret = ov767x_start_capture_read();
		if (EMXX_ov767x_BREAK == ov767x->state) {
			FNC_EXIT(-EIO)
			mutex_unlock(&ov767x->lock);
			return -EIO;
		}
	}

	FNC_EXIT(ret)
	mutex_unlock(&ov767x->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_stream_on
 * RETURN   :
 * NOTE     : called from emxx_cam.c ioctl(VIDIOC_STREAMON)
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_stream_on(int flag)
{
	int ret = 0;
	FNC_ENTRY;
	mutex_lock(&ov767x->lock);

	ret = ov767x_set_liveview(SET_LIVEVIEW_START);
	msleep(500);

	FNC_EXIT(ret)
	mutex_unlock(&ov767x->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_stream_off
 * RETURN   :
 * NOTE     : called from emxx_cam.c ioctl(VIDIOC_STREAMOFF)
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_stream_off(int flag)
{
	int ret = 0;
	FNC_ENTRY;
	mutex_lock(&ov767x->lock);

	ret = ov767x_set_liveview(SET_LIVEVIEW_STOP);

	FNC_EXIT(ret)
	mutex_unlock(&ov767x->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_startup
 * RETURN   :
 * NOTE     : initialize ov767x. called from emxx_cam.c open()
 * UPDATE   :
 ******************************************************************************/
#define END_MARK 0xFF
static int emxx_mega_startup(int flag)
{
	int ret = 0;
	FNC_ENTRY;
	mutex_lock(&ov767x->lock);

	i2c_stop = 0;
	ov767x->state = EMXX_ov767x_IDLE;

	/* Reset Camera Module
	 */
	gpio_direction_output(GPIO_P121, 0);
	udelay(1000);

	ret = i2c_add_driver(&ov767x_i2c_driver);
	printk("i2c_add_driver(%d, %p)\n", ret, ov767x_i2c_client);

	if (ret < 0) {
		printk("i2c: Driver registration failed,"
		    "module not inserted.\n");
		goto done_mega_startup;
	} else if (NULL == ov767x_i2c_client)    {
		i2c_del_driver(&ov767x_i2c_driver);
		err("i2c: Device was not detected.\n");
		ret = -EINVAL;

		goto done_mega_startup;
	} else {
		ret = sensor_detect();
		printk("sensor_detect = %d\n", ret);
		if(ret != 0) {
			goto done_mega_startup;
		}

		ret = ov767x_setreg_softreset(0);
		if (ret) {
			printk("i2c: Failed in writing to ");
			printk("the softreset[B-06:Softreset].\n");
			goto done_mega_startup;
		}

		ret = ov767x_set_liveview(SET_LIVEVIEW_STOP);
		if (ret) {
			err("i2c: Failed in writing to ");
			err("the softstandby[B-04:Control_01].\n");
			goto done_mega_startup;
		}

		ret = ov767x_getreg_firmware_version(
			&ov767x->firmware_version);
		if (ret) {
			printk("i2c: Failed in reading of the version");
			printk("information[x-02: RevNumber].\n");
			goto done_mega_startup;
		}
	}

	ov767x->active = 1;

	FNC_EXIT(ret)
	mutex_unlock(&ov767x->lock);
	return ret;

done_mega_startup:

	ov767x_shutdown();
	if (ov767x_i2c_client) {
		i2c_del_driver(&ov767x_i2c_driver);
	}
	ov767x_i2c_client = NULL;
	ov767x->active = 0;

	FNC_EXIT(ret)
	mutex_unlock(&ov767x->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_shutdown
 * RETURN   :
 * NOTE     : uninitialize ov767x. called from emxx_cam.c close()
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_shutdown(int flag)
{
	int ret = 0;
	FNC_ENTRY;
	mutex_lock(&ov767x->lock);

	ret = ov767x_shutdown();

	if (ov767x_i2c_client) {
		i2c_del_driver(&ov767x_i2c_driver);
	}
	//ov767x_i2c_client = NULL;

	FNC_EXIT(ret)
	mutex_unlock(&ov767x->lock);
	return 0;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_unregister
 * RETURN   :
 * NOTE     : unregist ov767x. called from emxx_cam.c shutdown
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_unregister(int flag)
{
	int ret = 0;
	FNC_ENTRY;

	kfree(ov767x);
	ov767x = NULL;

	FNC_EXIT(ret)
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_cam_hw_register
 * RETURN   :
 * NOTE     : regist ov767x. called from emxx_cam.c startup
 * UPDATE   :
 ******************************************************************************/
int emxx_cam_hw_register(struct emxx_cam_hw_operations *hw)
{
	int ret = 0;
	const struct v4l2_queryctrl *ctrl;
	FNC_ENTRY;

	ov767x = kzalloc(sizeof(*ov767x), GFP_KERNEL);

	if (ov767x == NULL) {
		err("ov767x: ov767x allocation failed!\n");
		FNC_EXIT(-ENOMEM)
		return -ENOMEM;
	}

	strlcpy(hw->name, "OV7675", sizeof(hw->name));

	hw->vidioc_queryctrl = emxx_mega_vidioc_queryctrl;
	hw->vidioc_querymenu = emxx_mega_vidioc_querymenu;
	hw->vidioc_g_ctrl    = emxx_mega_vidioc_g_ctrl;
	hw->vidioc_s_ctrl    = emxx_mega_vidioc_s_ctrl;
	hw->prepare          = emxx_mega_prepare;
	hw->trigger          = emxx_mega_trigger;
	hw->sync             = emxx_mega_sync;
	hw->stream_on        = emxx_mega_stream_on;
	hw->stream_off       = emxx_mega_stream_off;
	hw->startup          = emxx_mega_startup;
	hw->shutdown         = emxx_mega_shutdown;
	hw->unregister       = emxx_mega_unregister;

	/* initial ov767x */
	mutex_init(&ov767x->lock);

	ov767x->firmware_version = 0xff;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_OUTIMG_SIZE);
	ov767x->capture_size  = ctrl->default_value;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_OUTIMG_FORMAT);
	ov767x->outimg_fmt = ctrl->default_value;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_OUTDATA_FORMAT);
	ov767x->imgdata_fmt = ctrl->default_value;

	hw->private = ov767x;

	FNC_EXIT(ret)
	return ret;
}
EXPORT_SYMBOL(emxx_cam_hw_register);


