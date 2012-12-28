/*
 *  File Name       : emxx_mega.c
 *  Function        : MEGA for CAMERA I/F Driver
 *  Release Version : Ver 1.02
 *  Release Date    : 2011/02/02
 *
 *  Copyright (C) Renesas Electronics Corporation 2011
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
#include "emxx_ov3640.h"
#define I2C_SLAVE_CAM_NAME "ov3640"

#define DEV_NAME "emxx_mega"

#define CAM_DEBUG 0

#define X1R_NOT_ZERO 0

#define EMXX_CAM_CE14X_MAKING_DEBUG

/*** DEBUG code by the making ->*/
#ifdef EMXX_CAM_CE14X_MAKING_DEBUG

int mega_debug = 4;

#include <linux/moduleparam.h>

#define FNC_ENTRY	\
	if (mega_debug == 1 || mega_debug >= 9) {	\
		printk(KERN_INFO "entry:%s\n", __func__); \
	}

#define FNC_EXIT_N	\
	if (mega_debug == 1 || mega_debug >= 9) {	\
		printk(KERN_INFO "exit: %s:%d\n", __func__, __LINE__); \
	}

#define FNC_EXIT(r)	\
	if (mega_debug == 1 || mega_debug >= 9) {	\
		printk(KERN_INFO "exit:%d :%s:%d\n", r, __func__, __LINE__); \
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
		printk(KERN_INFO " --M%d-- " fmt, mega_debug, ## args); \
	}
#define d4b(fmt, args...)	\
	if (mega_debug == 4 || mega_debug >= 9) {	\
		printk(KERN_INFO " --M%d-- " fmt, mega_debug, ## args); \
	}
#define d5b(fmt, args...)	\
	if (mega_debug == 5 || mega_debug >= 9) {	\
		printk(KERN_INFO " --M%d-- " fmt, mega_debug, ## args); \
	}
#define d6b(fmt, args...)	\
	if (mega_debug == 6 || mega_debug >= 9) {	\
		printk(KERN_INFO " --M%d-- " fmt, mega_debug, ## args); \
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

/* flags */
static int i2c_stop;

/* prottyped */
static int ov3640_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
static int ov3640_i2c_remove(struct i2c_client *client);
static inline int ov3640_i2c_write(unsigned short cmd,
				      unsigned char *buf, unsigned char len);
static inline int ov3640_i2c_read(unsigned short cmd,
				     unsigned char *buf, unsigned char len);
static inline int ov3640_set_prepare(struct emxx_cam_prepare *prepare);
static inline int ov3640_get_outimg_size(__u8 *value);

static struct i2c_device_id cam_i2c_idtable[] = {
	{ I2C_SLAVE_CAM_NAME_OV3640, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cam_i2c_idtable);

static struct i2c_driver ov3640_i2c_driver = {
	.driver.name    = "i2c for OV3640",
	.id_table       = cam_i2c_idtable,
	.probe          = ov3640_i2c_probe,
	.remove         = ov3640_i2c_remove,
};
static struct i2c_client *ov3640_i2c_client;
	

/* i2c registerd function */
static int ov3640_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	ov3640_i2c_client = client;
	return 0;
}

static int ov3640_i2c_remove(struct i2c_client *client)
{
	ov3640_i2c_client = NULL;
	return 0;
}

/* usable i2c funcrion */
static inline int ov3640_i2c_write(unsigned short cmd,
				      unsigned char *buf, unsigned char len)
{
	#define MAX_WRITE_BYTE	16
	int ret = 0;
	char s_buf[MAX_WRITE_BYTE + sizeof(cmd)] = {0};
	struct i2c_msg msg;

	if (len > MAX_WRITE_BYTE) {
		err("%s max buffer is %d\n", __func__, MAX_WRITE_BYTE);
		return -ENOMEM;
	}

	msg.addr  = I2C_SLAVE_CAM_ADDR_OV3640;
	msg.flags = 0;
	msg.buf   = s_buf;
	msg.len   = len + sizeof(cmd);

	FNC_ENTRY;

	s_buf[0] = cmd >> 8;
	s_buf[1] = cmd & 0xff;
	memcpy(&s_buf[2], buf, len);

	if (i2c_stop)
		return -EIO;

	assert(ov3640_i2c_client);

	ret = i2c_transfer(ov3640_i2c_client->adapter, &msg, 1);
	if (1 != ret) {
		err("i2c_transfer: cmd : 0x%02x ret = %d\n", cmd, ret);
		i2c_stop = 1;
	} else {
		ret = 0;
	}

	FNC_EXIT(ret);
	return ret;
}

static inline int ov3640_i2c_read(unsigned short cmd,
				     unsigned char *buf, unsigned char len)
{
	int ret = 0;
	unsigned char RegBuf[2] = {0};
	struct i2c_msg msg[2];
	RegBuf[0] = cmd >> 8;
	RegBuf[1] = cmd & 0xff;

	msg[0].addr  = I2C_SLAVE_CAM_ADDR_OV3640;
	msg[0].flags = 0;
	msg[0].buf   = RegBuf;
	msg[0].len   = 2;

	msg[1].addr  = I2C_SLAVE_CAM_ADDR_OV3640;
	msg[1].flags = I2C_M_RD;
	msg[1].buf   = buf;
	msg[1].len   = len;
	

	
	FNC_ENTRY;

	if (i2c_stop)
		return -EIO;

	assert(ov3640_i2c_client);

	ret = i2c_transfer(ov3640_i2c_client->adapter, msg, 2);
	if (2 != ret) {
		err("i2c_transfer: cmd : 0x%04x ret = %d\n", cmd, ret);
		i2c_stop = 1;
	} else {
		ret = 0;
	}

	FNC_EXIT(ret);
	return ret;
}

static int write_regs(struct sensor_reg *p, int max)
{
	int i;
	int ret = 0;
	FNC_ENTRY;

	for (i=0;i<max; i++) {
		ret = ov3640_i2c_write(p[i].reg, &p[i].val, 1);
		//udelay(10);
		if (ret) {
			printk("ERROR: write_regs Sensor I2C [0x%04x]=0x%02x\n", p[i].reg, p[i].val );
			return ret;
		}

		if ( 0x3012==p[i].reg && 0x80==p[i].val )
			msleep(10);  /* delay 10ms after soft reset */	
	}

	FNC_EXIT(ret);
	return ret;
}

/*===============================================================*/
/* OV3640 Camera control flags                                */
/*===============================================================*/
enum {
	EMXX_OV3640_IDLE = 0,
	EMXX_OV3640_LIVE,
	EMXX_OV3640_CAPTURE,
	EMXX_OV3640_BREAK,
};

struct emxx_ov3640 {
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

struct emxx_ov3640 *ov3640;


/*===============================================================*/
/* OV3640 data structure. for VIDIOC_S_CTRL                   */
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
static const struct control_resize_info ov3640_outimg_size_menus[] = {

	{0x01, "CIF : 352x288",   0, 0, 352,  288},
	{0x02, "VGA : 640x480",   0, 0, 640,  480},    /* default in EM1*/
	{0x03, "SXGA: 1024x768",  0, 0, 1024, 768},
	{0x04, "UXGA: 1600x1200", 0, 0, 1600, 1200},
	{0x05, "QXGA: 2048x1536", 0, 0, 2048, 1536},    /* default */
	{0x06,"QXVGA:320x240", 0, 0, 320, 240},
	{0x07,"QCIF:176x144",0,0,176,144},

};
#define NUM_OUTIMG_SIZE_MENUS ARRAY_SIZE(ov3640_outimg_size_menus)
#define BOUNDARY_OUT_IMG_RESIZE 0x04 /* 640x480 */

/*===============================================================*/
/* OV3640 data structure. for VIDIOC_QUERYCTRL                */
/*===============================================================*/
static const struct v4l2_queryctrl no_ctrl = {
	.name  = "ov3640",
	.flags = V4L2_CTRL_FLAG_DISABLED,
};

/* default settings */
static const struct v4l2_queryctrl ov3640_ctrls[] = {
	{
		.id            = EMXX_RJ6ABA100_CID_OUTIMG_SIZE,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "Output image size",
		/* VGA only */
		.minimum       = 0,
		.maximum       = 6,
		.step          = 1,
		.default_value = 1, /* VGA */
		.flags         = 0,
	},
	{	.id            = V4L2_CID_FOCUS_AUTO,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "fouce",
		.minimum       = 0,
		.maximum       = 3,
		.step          = 1,
		.default_value = 0,
		.flags         = 0,
	},
	{
		.id            = V4L2_CID_COLORFX,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "effect",
		.minimum       = 0,
		.maximum       = 5,
		.step          = 1,
		.default_value = 0,
		.flags         = 0,
	},
	{
		.id            = V4L2_CID_EXPOSURE,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "exposure",
		.minimum       = 0,
		.maximum       = 4,
		.step          = 1,
		.default_value = 2,
		.flags         = 0,
	},
	{
		.id            = V4L2_CID_AUTO_WHITE_BALANCE,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "whitbalance_auto",
		.minimum       = 0,
		.maximum       = 0,
		.step          = 0,
		.default_value = 0,
		.flags         = 0,
	},
	{
		.id            = V4L2_CID_DO_WHITE_BALANCE,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "whitbalance",
		.minimum       = 0,
		.maximum       = 3,
		.step          = 1,
		.default_value = 0,
		.flags         = 0,
	},
	{
		.id            = V4L2_CID_CONTRAST,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "whitbalance",
		.minimum       = 0,
		.maximum       = 4,
		.step          = 1,
		.default_value = 2,
		.flags         = 0,
	},
};
#define NUM_OV3640_CTRLS ARRAY_SIZE(ov3640_ctrls)

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov3640_setreg_outimg_size
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_OUTIMG_SIZE
 *          : set OV3640 output image size register group
 * UPDATE   :
 ******************************************************************************/
static inline int ov3640_setreg_outimg_size(__u8 value)
{
	struct control_resize_info *s;
	
	s = (struct control_resize_info *)ov3640_outimg_size_menus;
	s += value;

	if ((s->width == 640) && (s->height == 480)) {

		write_regs(sensor_set_preview,
			sizeof(sensor_set_preview)/sizeof(struct sensor_reg));

	} else if ((s->width == 2048) && (s->height == 1536)) {


		write_regs(sensor_set_capture,
			sizeof(sensor_set_capture)/sizeof(struct sensor_reg));
		msleep(50);

	} else if ((s->width == 352) && (s->height == 288)) {


		write_regs(sensor_set_cif,
			sizeof(sensor_set_cif)/sizeof(struct sensor_reg));

	}else if ((s->width == 320) && (s->height == 240)) {


		write_regs(sensor_set_qxvga,
			sizeof(sensor_set_qxvga)/sizeof(struct sensor_reg));
	}else if ((s->width == 1600) && (s->height == 1200) ) {

		write_regs(sensor_set_capture,
			sizeof(sensor_set_capture)/sizeof(struct sensor_reg));

		write_regs(size_uxga,
			sizeof(size_uxga)/sizeof(struct sensor_reg));
	} else if ((s->width == 1024) && (s->height == 768)) {


		write_regs(sensor_set_capture,
			sizeof(sensor_set_capture)/sizeof(struct sensor_reg));

		write_regs(size_xga,
			sizeof(size_xga)/sizeof(struct sensor_reg));
	} else if ((s->width == 176) && (s->height == 144)) {


		write_regs(size_qcif,
			sizeof(size_qcif)/sizeof(struct sensor_reg));
	}
	return 0;
}

/* etc */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov3640_set_liveview
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
#define SET_LIVEVIEW_STOP  0
#define SET_LIVEVIEW_START 1

static inline int ov3640_set_liveview(__u8 val)
{
	unsigned char mask, data;
	int ret = 0;

	if(SET_LIVEVIEW_START == val){
		__u8 size;

		ret = ov3640_get_outimg_size(&size);
		if(ret)
			return ret;
		ret = ov3640_setreg_outimg_size(size);
	
	}
	if(!ret){
		if(val == SET_LIVEVIEW_START)
			ov3640->state = EMXX_OV3640_LIVE;
		else 
			ov3640->state = EMXX_OV3640_IDLE;
	}
	return ret;
}

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov3640_set_capture_read
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
static inline int ov3640_set_capture_read(__u8 data)
{
	int ret = 0;

	if (EMXX_OV3640_IDLE != ov3640->state)
		return -EBUSY;

	ret = ov3640_setreg_outimg_size(data);
	if (!ret)
		ov3640->state = EMXX_OV3640_CAPTURE;

	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov3640_shutdown
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
static inline int ov3640_shutdown(int flag)
{
	int ret = 0;
	static int init_once = 1;

	if(init_once)
	{
		//printk("++++++++++++++++++++++++++++++++++power on once\n");
		/* Disable front camera */
		gpio_direction_output(GPIO_FRONT_CAMERA_PWDN, 0x1); /*GPIO_P121*/
		/* Mask input & not pull */
		writel((readl(CHG_PULL16) & 0xf0ffffff), CHG_PULL16);
		writel((readl(CHG_PINSEL_G096) | (1 << 25)), CHG_PINSEL_G096);

		/* Enable rear camera, they share a data bus. */
		gpio_direction_output(GPIO_REAR_CAMERA_PWDN, 0x1); /*GPIO_P68*/
		/* Mask input & not pull */
		writel((readl(CHG_PULL7) & 0xfffffff0), CHG_PULL7);
		writel((readl(CHG_PINSEL_G064) | (1 << 4)), CHG_PINSEL_G064);
		mdelay(10);
		gpio_direction_output(GPIO_REAR_CAMERA_PWDN, 0x0); /*GPIO_P68*/
		init_once = 0;
		return 0;
	}

	if(flag)
		{
		//printk("++++++++++++++++++++++++++++++++++power down\n");
		gpio_direction_output(GPIO_REAR_CAMERA_PWDN, 0x1); /*GPIO_P68*/
		gpio_direction_output(GPIO_FRONT_CAMERA_PWDN, 0x1); /*GPIO_P121*/
	}else{
		//printk("++++++++++++++++++++++++++++++++++power on\n");
		gpio_direction_output(GPIO_FRONT_CAMERA_PWDN, 0x1); /*GPIO_P121*/
		msleep(5);
		gpio_direction_output(GPIO_REAR_CAMERA_PWDN, 0x0); /*GPIO_P68*/

	}


	return ret;
}

/*****************************************************************************
 * MODULE   : emxx_ov3640.c
 * FUNCTION : ov3640_reset
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
static inline int ov3640_reset(void)
{
	int ret = 0;
	static int init_once = 1;

	if (init_once) {
		printk("reset camera once\n");
		gpio_direction_output(GPIO_FRONT_CAMERA_RST, 0x0); /*GPIO_P98*/
		/* Mask input & not pull */
		writel((readl(CHG_PULL10) & 0xf0ffffff), CHG_PULL10);
		writel((readl(CHG_PINSEL_G096) | (1 << 2)), CHG_PINSEL_G096);
		init_once = 0;
	} else {
		gpio_set_value(GPIO_FRONT_CAMERA_RST, 0x0); /*GPIO_P98*/
	}

	msleep(10);
	gpio_set_value(GPIO_FRONT_CAMERA_RST, 0x1);
	printk("reset pin pull high \n");

	return ret;
}

/* EMXX_RJ6ABA100_CID_OUTIMG_SIZE */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov3640_get_outimg_size / ov3640_set_outimg_size
 * RETURN   :
 * NOTE     : EMXX_OV3640_CID_FW
 * UPDATE   :
 ******************************************************************************/
static inline int ov3640_get_outimg_size(__u8 *value)
{
	*value = ov3640->capture_size;
	return 0;
}

static inline int ov3640_set_outimg_size(struct emxx_cam_prepare *prepare,__u8 value)
{
		int ret = 0;
	struct control_resize_info *s;

	s = (struct control_resize_info *)ov3640_outimg_size_menus;
	s += value;

	prepare->bounds.left   = s->x;
	prepare->bounds.top    = s->y;
	prepare->bounds.width  = s->width  - (s->x);
	prepare->bounds.height = s->height - (s->y);

	prepare->c = prepare->bounds;
	prepare->width  = prepare->c.width ;
	prepare->height = prepare->c.height;

	ov3640->capture_size = value;
	/* ov3640->reset = 1; */

	return ret;
}

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov3640_start_capture_read
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
static inline int ov3640_start_capture_read(void)
{
	int ret = 0;

	ret = ov3640_set_capture_read(ov3640->capture_size);

	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ctrl_by_id
 * RETURN   :
 * NOTE     : get ov3640_ctrls id
 * UPDATE   :
 ******************************************************************************/
static const struct v4l2_queryctrl *ctrl_by_id(unsigned int id)
{
	unsigned int i;

	for (i = 0; i < NUM_OV3640_CTRLS; i++)
		if (ov3640_ctrls[i].id == id)
			return ov3640_ctrls + i;
	return NULL;
}


static int emxx_focus_auto(void)
{
	int count=0,focus_status=0;
	unsigned char value;
	write_regs(fouce_init,  sizeof(fouce_init) \
			/sizeof(struct sensor_reg));
	msleep(130);

	write_regs(auto_focuse_turn_off,sizeof(auto_focuse_turn_off)/sizeof(struct sensor_reg));
	msleep(5);
	ov3640_i2c_read(0x3f07, &value, 1);
	if(1)//value==0
	{
		write_regs(auto_focuse_turn_on,sizeof(auto_focuse_turn_on)/sizeof(struct sensor_reg));

		for(count=0;count <=50;count++)
		{
			ov3640_i2c_read(0x3f07, &value, 1);
			if(value==0x02)
			{
				focus_status = 1;//focus success
				break;
			}

			focus_status = 0;
			msleep(50);
		}
	}
	return focus_status;
}

static int emxx_focus(struct v4l2_control *c)
{
	int focus_status;
	switch(c->value)
	{
		case 1:
			write_regs(auto_focuse_turn_off,sizeof(auto_focuse_turn_off)/sizeof(struct sensor_reg));
			break;
		default:
		//auto
			focus_status=emxx_focus_auto();
			c->value=focus_status;
			break;
	}
	return 0;
}

static int emxx_color_effect(struct v4l2_control *c)
{	
	unsigned char value;
	switch(c->value)
	{
		case 1:
		write_regs(black_white,sizeof(black_white)/sizeof(struct sensor_reg));
			break;
		case 2:
		write_regs(negative,sizeof(negative)/sizeof(struct sensor_reg));
			break;
		case 3:
		write_regs(sepia,sizeof(sepia)/sizeof(struct sensor_reg));
			break;
		case 4:
		write_regs(greenish,sizeof(greenish)/sizeof(struct sensor_reg));
			break;
		default:
		write_regs(normal,sizeof(normal)/sizeof(struct sensor_reg));
			break;
	}
	return 0;
}

static int emxx_exposure(struct v4l2_control *c)
{

	switch(c->value)
	{
		case 0://.-2
			write_regs(low_1_7ev,sizeof(low_1_7ev)/sizeof(struct sensor_reg));
			break;
		case 1://.-1
			write_regs(low_0_7ev,sizeof(low_0_7ev)/sizeof(struct sensor_reg));
			break;
		case 2://.0
			write_regs(default_ev,sizeof(default_ev)/sizeof(struct sensor_reg));
			break;
		case 3: // .1
			write_regs(height_0_7ev,sizeof(height_0_7ev)/sizeof(struct sensor_reg));
			break;
		case 4: //. 2
			write_regs(height_1_7ev,sizeof(height_1_7ev)/sizeof(struct sensor_reg));
			break;
		default:
			printk("%s: no such  ctrl\n", __func__);
			break;
	}
	return 0;
}

static int emxx_auto_white_balance(struct v4l2_control *c)
{
	write_regs(white_blance_auto,sizeof(white_blance_auto)/sizeof(struct sensor_reg));
	return 0;
}

static int emxx_do_white_balance(struct v4l2_control *c)
{
	switch(c->value)
	{
		case 1:
			write_regs(home,sizeof(home)/sizeof(struct sensor_reg));
			break;
		case 2:
			write_regs(sunny,sizeof(sunny)/sizeof(struct sensor_reg));
			break;
		case 3:
			write_regs(cloudy,sizeof(cloudy)/sizeof(struct sensor_reg));
			break;
		default:
			write_regs(office,sizeof(office)/sizeof(struct sensor_reg));
			break;
	}
	return 0;
}
static int emxx_contrast(struct v4l2_control *c)
{

	switch(c->value)
	{
		case 0://.-2
			write_regs(contrast_2l,sizeof(contrast_2l)/sizeof(struct sensor_reg));
			break;
		case 1://.-1
			write_regs(contrast_1l,sizeof(contrast_1l)/sizeof(struct sensor_reg));
			break;
		case 2://.0
			write_regs(contrast_0,sizeof(contrast_0)/sizeof(struct sensor_reg));
			break;
		case 3: // .1
			write_regs(contrast_1h,sizeof(contrast_1h)/sizeof(struct sensor_reg));
			break;
		case 4: //. 2
			write_regs(contrast_2h,sizeof(contrast_2h)/sizeof(struct sensor_reg));
			break;
		default:
			printk("%s: no such  ctrl\n", __func__);
			break;
	}
	return 0;
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

	mutex_lock(&ov3640->lock);
	switch (m->id) {
	
	default:
		ret = -EINVAL;
		break;
	}

	FNC_EXIT(ret)
	mutex_unlock(&ov3640->lock);
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
	mutex_lock(&ov3640->lock);
	switch (c->id) {
	
	default:
		ret = -EINVAL;
	}

	if (!ret)
		c->value = val;

	FNC_EXIT(ret)
	mutex_unlock(&ov3640->lock);
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
		break;
	default:
		/* nothing */;
	};
	mutex_lock(&ov3640->lock);
	switch (c->id) {
		case V4L2_CID_FOCUS_AUTO:
			emxx_focus(c);
		break;
		case V4L2_CID_COLORFX:
			emxx_color_effect(c);
		break;
		case V4L2_CID_EXPOSURE:
			emxx_exposure(c);
		break;
		case V4L2_CID_AUTO_WHITE_BALANCE:
			emxx_auto_white_balance(c);
		break;
		case V4L2_CID_DO_WHITE_BALANCE:
			emxx_do_white_balance(c);
		break;
		case V4L2_CID_CONTRAST:
			emxx_contrast(c);
		break;
	default:
		ret = -EINVAL;
	}

	FNC_EXIT(ret)
	mutex_unlock(&ov3640->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_ov3640.c
 * FUNCTION : emxx_ov3640_vidioc_s_fmt
 * RETURN   :
 * NOTE     : called from emxx_rear_cam.c ioctl(vidioc_s_fmt_vid_cap)
 * UPDATE   :
 ******************************************************************************/
int emxx_ov3640_vidioc_s_fmt(struct emxx_cam_prepare *prepare,
					struct v4l2_format *f)
{
	__u8 index;
	int ret = -EINVAL;
	struct control_resize_info *s;
	FNC_ENTRY;

	mutex_lock(&ov3640->lock);

	ret = ov3640_get_outimg_size(&index);
	if (ret)
		goto out;

	s = (struct control_resize_info *)ov3640_outimg_size_menus;
	s += index;

	for(index = 0; index < NUM_OUTIMG_SIZE_MENUS; index ++) {
		if ((f->fmt.pix.width == ov3640_outimg_size_menus[index].width) &&
			(f->fmt.pix.height == ov3640_outimg_size_menus[index].height)) {

			ov3640_set_prepare(prepare);

			ret = ov3640_set_outimg_size(prepare, index);
			if (ret)
				goto out;

			goto out;
		}
	}
out:
	FNC_EXIT(ret)
	mutex_unlock(&ov3640->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ov3640_set_prepare
 * RETURN   :
 * NOTE     : called from emxx_cam.c
 * UPDATE   :
 ******************************************************************************/
static inline int ov3640_set_prepare(struct emxx_cam_prepare *prepare)
{
	FNC_ENTRY;

	prepare->syncmode = 0;/* CAM_HS, CAM_VS mode */
	prepare->synctype = 1;/* Enable signal sampling mode */
	prepare->data_id  = 0;/* U->Y->V->Y */
	prepare->vs_det   = 1;/* falling edge */
	prepare->hs_det   = 0;/* rising edge */
	prepare->clk_edge = 0;/* single edge transfer */
	prepare->data_det = 0;/* rising edge */
	prepare->vs_pol   = 1;/* negitive logic */
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
	return 0;

	int ret = 0;
	__u8 size;
	FNC_ENTRY;
	mutex_lock(&ov3640->lock);

	ret = ov3640_get_outimg_size(&size);
	if (ret) {
		FNC_EXIT(ret)
		mutex_unlock(&ov3640->lock);
		return ret;
	}

	if (BOUNDARY_OUT_IMG_RESIZE < size) {
		struct control_resize_info *s;

		if (EMXX_OV3640_BREAK == ov3640->state) {
			FNC_EXIT(-EIO)
			mutex_unlock(&ov3640->lock);
			return -EIO;
		}

		ret = ov3640_get_outimg_size(&size);
		if (ret) {
			FNC_EXIT(ret)
			mutex_unlock(&ov3640->lock);
			return ret;
		}

		s = (struct control_resize_info *)ov3640_outimg_size_menus;
		s += size;

		prepare->bounds.left   = 0;
		prepare->bounds.top    = 0;
		prepare->bounds.width  = s->width  - (s->x - 1);
		prepare->bounds.height = s->height - (s->y - 1);

		prepare->c = prepare->bounds;
		prepare->width  = prepare->c.width  - (prepare->c.left - 1);
		prepare->height = prepare->c.height - (prepare->c.top  - 1);

		if (!prepare->actions) {
			FNC_EXIT(ret)
			mutex_unlock(&ov3640->lock);
			return ret;
		}

		prepare->actions = 1;

	} else {
		prepare->actions = 0;
	}

	FNC_EXIT(ret)
	mutex_unlock(&ov3640->lock);
	return ret;

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

	mutex_lock(&ov3640->lock);

	ret = ov3640_get_outimg_size(&index);
	if (!ret) {
		struct control_resize_info *s;

		s = (struct control_resize_info *)ov3640_outimg_size_menus;
		s += index;

		prepare->bounds.left   = 0;
		prepare->bounds.top    = 0;
		prepare->bounds.width  = s->width;
		prepare->bounds.height = s->height;

		prepare->c      = prepare->bounds;
		prepare->width  = prepare->c.width;
		prepare->height = prepare->c.height;

		ov3640_set_prepare(prepare);

		prepare->reset = ov3640->reset;
		d1b("chk reset 0x%02x\n", ov3640->reset);

		if (prepare->actions && !ov3640->reset
		    && BOUNDARY_OUT_IMG_RESIZE < index)
			ov3640->state = EMXX_OV3640_IDLE;

		ov3640->reset = 0;
	}

	FNC_EXIT(ret)
	mutex_unlock(&ov3640->lock);
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
	return 0;
	__u8 size;
	int ret = 0;
	FNC_ENTRY;

	mutex_lock(&ov3640->lock);

	ret = ov3640_get_outimg_size(&size);
	if (ret) {
		FNC_EXIT(ret)
		mutex_unlock(&ov3640->lock);
		return ret;
	}

	if (BOUNDARY_OUT_IMG_RESIZE < size) {
		ret = ov3640_start_capture_read();
		if (EMXX_OV3640_BREAK == ov3640->state) {
			FNC_EXIT(-EIO)
			mutex_unlock(&ov3640->lock);
			return -EIO;
		}
	}

	FNC_EXIT(ret)
	mutex_unlock(&ov3640->lock);
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
	mutex_lock(&ov3640->lock);

	ret = ov3640_set_liveview(SET_LIVEVIEW_START);

	FNC_EXIT(ret)
	mutex_unlock(&ov3640->lock);
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
	mutex_lock(&ov3640->lock);

	ret = ov3640_set_liveview(SET_LIVEVIEW_STOP);

	FNC_EXIT(ret)
	mutex_unlock(&ov3640->lock);
	return ret;
}

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_startup
 * RETURN   :
 * NOTE     : initialize OV3640. called from emxx_cam.c open()
 * UPDATE   :
 ******************************************************************************/
#define END_MARK 0xFF
static int emxx_mega_startup(int flag)
{
	int ret = 0;
	FNC_ENTRY;
	mutex_lock(&ov3640->lock);

	i2c_stop = 0;
	ov3640->state = EMXX_OV3640_IDLE;

	ov3640_shutdown(0);
	ov3640_reset();
	
	ret = i2c_add_driver(&ov3640_i2c_driver);
	d1b("i2c_add_driver(%d, %p)\n", ret, ov3640_i2c_client);

	if (ret < 0) {
		err("i2c: Driver registration failed,"
		    "module not inserted.\n");
		goto done_mega_startup;
	} else if (NULL == ov3640_i2c_client)    {
		i2c_del_driver(&ov3640_i2c_driver);
		err("i2c: Device was not detected.\n");
		ret = -EINVAL;

		goto done_mega_startup;
	}

	{
		write_regs(sensor_initialize,  sizeof(sensor_initialize) \
			/sizeof(struct sensor_reg));

			//	write_regs(sensor_set_preview,
		//	sizeof(sensor_set_preview)/sizeof(struct sensor_reg));				
		unsigned char value;
		ov3640_i2c_read(0x300a,&value,1);
		printk("++++++++++++++++++++++++++ov364 ID =0x%02x\n",value);
				ov3640_i2c_read(0x300b,&value,1);
		printk("++++++++++++++++++++++++++ov364 ID =0x%02x\n",value);
		ov3640->state = EMXX_OV3640_IDLE;

	}

	ov3640->active = 1;

	FNC_EXIT(ret)
	mutex_unlock(&ov3640->lock);
	return ret;

done_mega_startup:

	ov3640_shutdown(1);
	if (ov3640_i2c_client)
		i2c_del_driver(&ov3640_i2c_driver);
	ov3640_i2c_client = NULL;
	ov3640->active = 0;

	FNC_EXIT(ret)
	mutex_unlock(&ov3640->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_shutdown
 * RETURN   :
 * NOTE     : uninitialize OV3640. called from emxx_cam.c close()
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_shutdown(int flag)
{
	int ret = 0;
	FNC_ENTRY;
	mutex_lock(&ov3640->lock);

	ret = ov3640_shutdown(1);

	if (ov3640_i2c_client)
		i2c_del_driver(&ov3640_i2c_driver);
	/* ov3640_i2c_client = NULL; */

	FNC_EXIT(ret)
	mutex_unlock(&ov3640->lock);
	return 0;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_unregister
 * RETURN   :
 * NOTE     : unregist OV3640. called from emxx_cam.c shutdown
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_unregister(int flag)
{
	int ret = 0;
	FNC_ENTRY;

	kfree(ov3640);
	ov3640 = NULL;

	FNC_EXIT(ret)
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_cam_hw_register
 * RETURN   :
 * NOTE     : regist OV3640. called from emxx_cam.c startup
 * UPDATE   :
 ******************************************************************************/
int emxx_rear_cam_hw_register(struct emxx_cam_hw_operations *hw)
{
	int ret = 0;
	const struct v4l2_queryctrl *ctrl;
	FNC_ENTRY;

	ov3640 = kzalloc(sizeof(*ov3640), GFP_KERNEL);

	if (ov3640 == NULL) {
		err("OV3640: ov3640 allocation failed!\n");
		FNC_EXIT(-ENOMEM)
		return -ENOMEM;
	}

	strlcpy(hw->name, "OV3640", sizeof(hw->name));

	hw->vidioc_queryctrl = emxx_mega_vidioc_queryctrl;
	hw->vidioc_querymenu = emxx_mega_vidioc_querymenu;
	hw->vidioc_g_ctrl    = emxx_mega_vidioc_g_ctrl;
	hw->vidioc_s_ctrl    = emxx_mega_vidioc_s_ctrl;
	hw->vidioc_s_fmt     = emxx_ov3640_vidioc_s_fmt;
	hw->prepare          = emxx_mega_prepare;
	//hw->trigger          = emxx_mega_trigger;
	//hw->sync             = emxx_mega_sync;
	hw->stream_on        = emxx_mega_stream_on;
	hw->stream_off       = emxx_mega_stream_off;
	hw->startup          = emxx_mega_startup;
	hw->shutdown         = emxx_mega_shutdown;
	hw->unregister       = emxx_mega_unregister;

	/* initial ov3640 */
	mutex_init(&ov3640->lock);

	ov3640->firmware_version = 0xff;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_OUTIMG_SIZE);
	ov3640->capture_size  = ctrl->default_value;

	hw->private = ov3640;

	FNC_EXIT(ret)
	return ret;
}
EXPORT_SYMBOL(emxx_rear_cam_hw_register);


