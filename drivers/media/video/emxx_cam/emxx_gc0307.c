/*
 * Driver for gc0307 CMOS Image Sensor
 *
 * Copyright (C) 2008, zbunix <zbunix@gmail.com>
 *
 * this code was modify/ported from the emxx_mega and emxx_ov767x driver
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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
#include "emxx_cam.h"
#include "emxx_gc0307.h"


#define DEV_NAME "emxx_mega"
#define GPIO_SENSOR_PWDN    GPIO_P121


#define CAM_DEBUG 0
//#define EMXX_CAM_CE14X_MAKING_DEBUG

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

/*===============================================================*/
/* I2C Functions                                                 */
/*===============================================================*/


/* flags */
static int i2c_stop;

/* prottyped */
static int gc0307_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
static int gc0307_i2c_remove(struct i2c_client *client);
static inline int gc0307_i2c_write(unsigned char cmd,
				      unsigned char *buf, unsigned char len);
static inline int gc0307_i2c_read(unsigned char cmd,
				     unsigned char *buf, unsigned char len);
static inline int gc0307_set_prepare(struct emxx_cam_prepare *prepare);
static inline int gc0307_get_outimg_size(__u8 *value);

static struct i2c_device_id cam_i2c_idtable[] = {
	{ I2C_SLAVE_CAM_NAME_GC0307, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cam_i2c_idtable);

static struct i2c_driver gc0307_i2c_driver = {
	.driver.name    = "i2c for GC0307",
	.id_table       = cam_i2c_idtable,
	.probe          = gc0307_i2c_probe,
	.remove         = gc0307_i2c_remove,
};
static struct i2c_client *gc0307_i2c_client;


/* i2c registerd function */
static int gc0307_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	gc0307_i2c_client = client;
	return 0;
}

static int gc0307_i2c_remove(struct i2c_client *client)
{
	gc0307_i2c_client = NULL;
	return 0;
}

/* usable i2c funcrion */
static inline int gc0307_i2c_write(unsigned char cmd,
				      unsigned char *buf, unsigned char len)
{
	int ret = 0;
	char s_buf[15];
	struct i2c_msg msg = { .addr = I2C_SLAVE_CAM_ADDR_GC0307, .flags = 0,
			       .buf = s_buf, .len = len + 1 };

	s_buf[0] = cmd;
	memcpy(&s_buf[1], buf, len);

	if (i2c_stop)
		return -EIO;

	assert(gc0307_i2c_client);

	ret = i2c_transfer(gc0307_i2c_client->adapter, &msg, 1);
	if (1 != ret) {
		err("i2c_transfer: cmd : 0x%02x ret = %d\n", cmd, ret);
		i2c_stop = 1;
		ret = -EIO;
	} else {
		ret = 0;
	}
	return ret;
}

static inline int gc0307_i2c_read(unsigned char cmd,
				     unsigned char *buf, unsigned char len)
{
	int ret = 0;
	struct i2c_msg msg[] =  {{ .addr = I2C_SLAVE_CAM_ADDR_GC0307, .flags = 0,
				   .buf = &cmd, .len = 1 },
				 { .addr = I2C_SLAVE_CAM_ADDR_GC0307, .flags = I2C_M_RD,
				   .buf = buf,  .len = len } };
	if (i2c_stop)
		return -EIO;

	assert(gc0307_i2c_client);

	ret = i2c_transfer(gc0307_i2c_client->adapter, msg, 2);
	if (2 != ret) {
		err("i2c_transfer: cmd : 0x%02x ret = %d\n", cmd, ret);
		i2c_stop = 1;
		ret = -EIO;
	} else {
		ret = 0;
	}
	return ret;
}


static int write_regs(struct sensor_reg *p, int max)
{
	int i;
	int ret = 0;
	FNC_ENTRY;

	for (i=0;i<max; i++) {
		ret = gc0307_i2c_write(p[i].reg, &p[i].val, 1);
		//udelay(10);
		if (ret) {
			printk("ERROR: write_regs Sensor I2C [0x%04x]=0x%02x\n", p[i].reg, p[i].val );
			return ret;
		}
	}

	FNC_EXIT(ret);
	return ret;
}


/*===============================================================*/
/* GC0307 Camera control flags                                */
/*===============================================================*/
enum {
	EMXX_RJ6ABA100_IDLE = 0,
	EMXX_RJ6ABA100_LIVE,
	EMXX_RJ6ABA100_CAPTURE,
	EMXX_RJ6ABA100_BREAK,
};

struct emxx_gc0307 {
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

struct emxx_gc0307 *gc0307;


/*===============================================================*/
/* GC0307 data structure. for VIDIOC_S_CTRL                   */
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
static const struct control_resize_info gc0307_outimg_size_menus[] =
{
	{0x01, "VGA  : 640x480",   0, 0, 640,  480},    /* default in EM1*/
	{0x02, "CIF  : 352x288",   0, 0, 352,  288},
	{0x03, "QVGA : 320x240",   0, 0, 320,  240},
	{0x04, "QCIF : 176x144",   0, 0, 176,  144},
	{0x05, "QQVGA: 160x120",   0, 0, 160,  120},
};
#define NUM_OUTIMG_SIZE_MENUS ARRAY_SIZE(gc0307_outimg_size_menus)
#define BOUNDARY_OUT_IMG_RESIZE 0x04 /* 640x480 */


/*===============================================================*/
/* GC0307 data structure. for VIDIOC_QUERYCTRL                */
/*===============================================================*/
static const struct v4l2_queryctrl no_ctrl = {
	.name  = "gc0307",
	.flags = V4L2_CTRL_FLAG_DISABLED,
};

/* default settings */
static const struct v4l2_queryctrl gc0307_ctrls[] = {
	{
		.id            = EMXX_RJ6ABA100_CID_OUTIMG_SIZE,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "Output image size",
		.minimum       = 4,
		.maximum       = 4,
		.step          = 1,
		.default_value = 0, /* VGA */
		.flags         = 0,
	},
};
#define NUM_GC0307_CTRLS ARRAY_SIZE(gc0307_ctrls)


/*===============================================================*/
/* GC0307 register data structure                             */
/*===============================================================*/
struct register_info {
	unsigned char bank;
	unsigned char address;
	unsigned char data;
	unsigned char mask;
};

/*===============================================================*/
/* GC0307 Camera control function                             */
/*===============================================================*/

#if CAM_DEBUG
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mage_register_debug
 * RETURN   :
 * NOTE     : check GC0307 register (debug only)
 * UPDATE   :
 ******************************************************************************/
static inline void emxx_mage_register_debug(void)
{
	unsigned char buf[16];
	int i, j, bank, ret;

	for (bank = NUM_B03_REGBANK_B; bank <= NUM_B03_REGBANK_D; bank++) {
		ret = gc0307_chkreg_bank(bank);
		if (ret)
			return;

		d1b("========================================\n");
		d1b(" Group %c\n",
		       ((bank == NUM_B03_REGBANK_B) ? 'B'
			: ((bank == NUM_B03_REGBANK_C) ? 'C' : 'D')));
		d1b("-----------------------------------------\n");

		for (i = 0; i < 0xFF; i += 16) {
			for (j = 0; j < 16; j++) {
				ret = gc0307_i2c_read(i + j, &buf[j], 1);
				if (ret)
					return;
			}

			d1b(" %c-%02x:  %02x %02x %02x %02x  "
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
	d1b("============================================\n");

}
#endif /* CAM_DEBUG */


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : gc0307_setreg_outimg_size
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_OUTIMG_SIZE
 *          : set GC0307 output image size register group
 * UPDATE   :
 ******************************************************************************/
static inline int gc0307_setreg_outimg_size(__u8 value)
{
	static __s32 cur_width, cur_height;
	struct control_resize_info *s;

	s = (struct control_resize_info *)gc0307_outimg_size_menus;
	s += value;

	if ((cur_width == s->width) && (cur_height == s->height))
		return 0;

	if ((s->width == 640) && (s->height == 480)) {
		write_regs(sensor_set_vga,
			sizeof(sensor_set_vga)/sizeof(struct sensor_reg));
	} else if ((s->width == 352) && (s->height == 288)) {
		write_regs(sensor_set_cif,
			sizeof(sensor_set_cif)/sizeof(struct sensor_reg));

	} else if ((s->width == 320) && (s->height == 240)) {
		write_regs(sensor_set_vga,
			sizeof(sensor_set_vga)/sizeof(struct sensor_reg));

	} else if ((s->width == 176) && (s->height == 144)) {
		write_regs(sensor_set_qcif,
			sizeof(sensor_set_qcif)/sizeof(struct sensor_reg));

	} else if ((s->width == 160) && (s->height == 120)) {
		write_regs(sensor_set_qqvga,
			sizeof(sensor_set_qqvga)/sizeof(struct sensor_reg));
	} else {
		err("Not supported image size.\n");
		return -EINVAL;
	}
	cur_width = s->width;
	cur_width = s->height;
	return 0;
	return 0;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : gc0307_getreg_firmware_version
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_FW
 *          : get GC0307 register revesion number
 * UPDATE   :
 ******************************************************************************/
#define NUM_B02_REVNUMBER  0x02

static inline int gc0307_getreg_firmware_version(__u8 *val)
{
	char buf;
	int ret;

	ret = gc0307_i2c_read(0x00, &buf, 1);
	printk("\n###Detect sensor idcode=0x%02x\n", buf );
	/* revesion number */
	if (!ret)
		*val = buf;
	if (0x99!=buf) {
		*val = buf;
		printk("Error: sensor idcode=0x%02x(gc0307 idcode=0x99)\n", buf );
	}
	return ret;
}


/* etc */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : gc0307_set_liveview
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
#define SET_LIVEVIEW_STOP  0
#define SET_LIVEVIEW_START 1

static inline int gc0307_set_liveview(__u8 val)
{
	int ret = 0;
	if (val == SET_LIVEVIEW_START) {
		__u8 size;
		ret = gc0307_get_outimg_size(&size);
		if(ret)
			return ret;
		ret = gc0307_setreg_outimg_size(size);
	} 
	if(!ret){

		if (val == SET_LIVEVIEW_START)
			gc0307->state = EMXX_RJ6ABA100_LIVE;
		else
			gc0307->state = EMXX_RJ6ABA100_IDLE;
	}
	return 0;
}

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : gc0307_set_capture_read
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
static inline int gc0307_set_capture_read(__u8 data)
{
	int ret = 0;

	if (EMXX_RJ6ABA100_IDLE != gc0307->state)
		return -EBUSY;

	ret = gc0307_setreg_outimg_size(data);
	if (!ret)
		gc0307->state = EMXX_RJ6ABA100_CAPTURE;

	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : gc0307_shutdown
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
static inline int gc0307_shutdown(int flags)
{
	int ret = 0;
	static int init;
	if(!init){

	/* Reset Camera Module
	 */
	mdelay(100);
//	printk("+++++++++++++back camera power down pin pull up\n");
	/* Enable rear camera, they share a data bus. */
	gpio_direction_output(GPIO_P68, 0x1); /*GPIO_P68*/
	/* Mask input & not pull */
	writel((readl(CHG_PULL7) & 0xfffffff0), CHG_PULL7);
	writel((readl(CHG_PINSEL_G064) | (1 << 4)), CHG_PINSEL_G064);

//	printk("++++++++++++++pull down power down pin to enable frant camera\n");
	gpio_direction_output(GPIO_SENSOR_PWDN, 0x0); /*GPIO_P121*/
	/* Mask input & not pull */
	writel((readl(CHG_PULL16) & 0xf0ffffff), CHG_PULL16);
	writel((readl(CHG_PINSEL_G096) | (1 << 25)), CHG_PINSEL_G096);
	mdelay(10);
	return 0;
	}

	if(flags){
	/* gc0307 power off
	 */
		gpio_direction_output(GPIO_SENSOR_PWDN, 0x1); //0:normal  1:POWER DOWN
	}else{
		gpio_direction_output(GPIO_P68, 0x1); /*GPIO_P68*/
		gpio_direction_output(GPIO_SENSOR_PWDN, 0x0); //0:normal  1:POWER DOWN
	}
	return ret;
}

/* EMXX_RJ6ABA100_CID_FW */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : gc0307_get_firmware_version
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_FW
 * UPDATE   :
 ******************************************************************************/
static inline int gc0307_get_firmware_version(__u8 *value)
{
	*value = gc0307->firmware_version;
	return 0;
}


/* EMXX_RJ6ABA100_CID_OUTIMG_SIZE */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : gc0307_get_outimg_size / gc0307_set_outimg_size
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_FW
 * UPDATE   :
 ******************************************************************************/
static inline int gc0307_get_outimg_size(__u8 *value)
{
	*value = gc0307->capture_size;
	return 0;
}

static inline int gc0307_set_outimg_size(struct emxx_cam_prepare *prepare,__u8 value)
{
	int ret = 0;
	struct control_resize_info *s;
	//printk("[%s:%d] set size=%d\n", __func__, __LINE__, value);
	s = (struct control_resize_info *)gc0307_outimg_size_menus;
	s += value;

	prepare->bounds.left = s->x;
	prepare->bounds.top    = s->y;
	prepare->bounds.width  = s->width  - (s->x);
	prepare->bounds.height = s->height - (s->y);

	prepare->c = prepare->bounds;
	prepare->width  = prepare->c.width ;
	prepare->height = prepare->c.height;

	gc0307->capture_size = value;


	return ret;
}

/* EMXX_RJ6ABA100_CID_OUTIMG_FORMAT */
/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : gc0307_get_outimg_fmt / gc0307_set_outimg_fmt
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_FW
 * UPDATE   :
 ******************************************************************************/
static inline int gc0307_get_outimg_fmt(__u8 *value)
{
	*value = gc0307->outimg_fmt;
	return 0;
}

static inline int gc0307_set_outimg_fmt(__u8 value)
{
	int ret = 0;

	if (value != gc0307->outimg_fmt)
		gc0307->outimg_fmt = value;

	return ret;
}

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : gc0307_start_capture_read
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
static inline int gc0307_start_capture_read(void)
{
	int ret = 0;

	ret = gc0307_set_capture_read(gc0307->capture_size);

	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : ctrl_by_id
 * RETURN   :
 * NOTE     : get gc0307_ctrls id
 * UPDATE   :
 ******************************************************************************/
static const struct v4l2_queryctrl *ctrl_by_id(unsigned int id)
{
	unsigned int i;

	for (i = 0; i < NUM_GC0307_CTRLS; i++)
		if (gc0307_ctrls[i].id == id)
			return gc0307_ctrls + i;
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

	mutex_lock(&gc0307->lock);
	switch (m->id) {

	default:
		ret = -EINVAL;
		break;
	}

	FNC_EXIT(ret)
	mutex_unlock(&gc0307->lock);
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
	mutex_lock(&gc0307->lock);
	switch (c->id) {

	default:
		ret = -EINVAL;
	}

	if (!ret)
		c->value = val;

	FNC_EXIT(ret)
	mutex_unlock(&gc0307->lock);
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
	mutex_lock(&gc0307->lock);
	switch (c->id) {

	default:
		ret = -EINVAL;
	}

	FNC_EXIT(ret)
	mutex_unlock(&gc0307->lock);
	return ret;
}

static inline int emxx_mega_vidioc_s_fmt(struct emxx_cam_prepare *prepare,
					struct v4l2_format *f)
{
		__u8 index;
	int ret = -EINVAL;
	struct control_resize_info *s;
	FNC_ENTRY;

	mutex_lock(&gc0307->lock);

	ret = gc0307_get_outimg_size(&index);
	if (ret)
		goto out;

	s = (struct control_resize_info *)gc0307_outimg_size_menus;
	s += index;

	for(index = 0; index < NUM_OUTIMG_SIZE_MENUS; index ++) {
		if ((f->fmt.pix.width == gc0307_outimg_size_menus[index].width) &&
			(f->fmt.pix.height == gc0307_outimg_size_menus[index].height)) {

			gc0307_set_prepare(prepare);

			ret = gc0307_set_outimg_size(prepare, index);
			if (ret)
				goto out;
 
			goto out;
		}
	}
out:
	FNC_EXIT(ret)
	mutex_unlock(&gc0307->lock);
	return ret;
}

/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : gc0307_set_prepare
 * RETURN   :
 * NOTE     : called from emxx_cam.c
 * UPDATE   :
 ******************************************************************************/
static inline int gc0307_set_prepare(struct emxx_cam_prepare *prepare)
{
	FNC_ENTRY;

	prepare->syncmode = 0;/* CAM_HS, CAM_VS mode */
	prepare->synctype = 1;/* Enable signal sampling mode */
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
	mutex_lock(&gc0307->lock);

	ret = gc0307_get_outimg_size(&size);
	if (ret) {
		FNC_EXIT(ret)
		mutex_unlock(&gc0307->lock);
		return ret;
	}

	if (BOUNDARY_OUT_IMG_RESIZE < size) {
		struct control_resize_info *s;

		if (EMXX_RJ6ABA100_BREAK == gc0307->state) {
			FNC_EXIT(-EIO)
			mutex_unlock(&gc0307->lock);
			return -EIO;
		}

		ret = gc0307_get_outimg_size(&size);
		if (ret) {
			FNC_EXIT(ret)
			mutex_unlock(&gc0307->lock);
			return ret;
		}

		s = (struct control_resize_info *)gc0307_outimg_size_menus;
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
			mutex_unlock(&gc0307->lock);
			return ret;
		}

		prepare->actions = 1;

		#if 0
		ret = gc0307_set_liveview(SET_LIVEVIEW_START);
		#endif
	} else {
		prepare->actions = 0;
	}

	FNC_EXIT(ret)
	mutex_unlock(&gc0307->lock);
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

	mutex_lock(&gc0307->lock);

	ret = gc0307_get_outimg_size(&index);
	if (!ret) {
		struct control_resize_info *s;

		s = (struct control_resize_info *)gc0307_outimg_size_menus;
		s += index;

		prepare->bounds.left   = 0;
		prepare->bounds.top    = 0;
		prepare->bounds.width  = s->width;
		prepare->bounds.height = s->height;

		prepare->c      = prepare->bounds;
		prepare->width  = prepare->c.width;
		prepare->height = prepare->c.height;

		gc0307_set_prepare(prepare);

		prepare->reset = gc0307->reset;
		d1b("chk reset 0x%02x\n", gc0307->reset);

		if (prepare->actions && !gc0307->reset
		    && BOUNDARY_OUT_IMG_RESIZE < index)
			gc0307->state = EMXX_RJ6ABA100_IDLE;

		gc0307->reset = 0;
	}

	FNC_EXIT(ret)
	mutex_unlock(&gc0307->lock);
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

	mutex_lock(&gc0307->lock);

	ret = gc0307_get_outimg_size(&size);
	if (ret) {
		FNC_EXIT(ret)
		mutex_unlock(&gc0307->lock);
		return ret;
	}

	if (BOUNDARY_OUT_IMG_RESIZE < size) {
		ret = gc0307_start_capture_read();
		if (EMXX_RJ6ABA100_BREAK == gc0307->state) {
			FNC_EXIT(-EIO)
			mutex_unlock(&gc0307->lock);
			return -EIO;
		}
	}

	FNC_EXIT(ret)
	mutex_unlock(&gc0307->lock);
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
	mutex_lock(&gc0307->lock);

	ret = gc0307_set_liveview(SET_LIVEVIEW_START);
	msleep(500);
#if CAM_DEBUG
	emxx_mage_register_debug();
#endif

	FNC_EXIT(ret)
	mutex_unlock(&gc0307->lock);
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
	mutex_lock(&gc0307->lock);

	ret = gc0307_set_liveview(SET_LIVEVIEW_STOP);
#if CAM_DEBUG
	emxx_mage_register_debug();
#endif

	FNC_EXIT(ret)
	mutex_unlock(&gc0307->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_startup
 * RETURN   :
 * NOTE     : initialize GC0307. called from emxx_cam.c open()
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_startup(int flag)
{
	int ret = 0;
	FNC_ENTRY;
	mutex_lock(&gc0307->lock);

	i2c_stop = 0;
	gc0307->state = EMXX_RJ6ABA100_IDLE;

	gc0307_shutdown(0);

	ret = i2c_add_driver(&gc0307_i2c_driver);
	d1b("i2c_add_driver(%d, %p)\n", ret, gc0307_i2c_client);

	if (ret < 0) {
		err("i2c: Driver registration failed,"
		    "module not inserted.\n");
		goto done_mega_startup;
	} else if (NULL == gc0307_i2c_client)    {
		i2c_del_driver(&gc0307_i2c_driver);
		err("i2c: Device was not detected.\n");
		ret = -EINVAL;

		goto done_mega_startup;
	} else {

		ret = gc0307_set_liveview(SET_LIVEVIEW_STOP);
		if (ret) {
			err("i2c: Failed in writing to ");
			err("the softstandby[B-04:Control_01].\n");
			goto done_mega_startup;
		}

		ret = gc0307_getreg_firmware_version(
			&gc0307->firmware_version);
		if (ret) {
			err("i2c: Failed in reading of the version");
			err("information[x-02: RevNumber].\n");
			goto done_mega_startup;
		}
	}

	/* gc0307 initialization */
	{


		write_regs(sensor_initialize,
			sizeof(sensor_initialize)/sizeof(struct sensor_reg));
		gc0307->state = EMXX_RJ6ABA100_IDLE;

	}

	gc0307->active = 1;

	FNC_EXIT(ret)
	mutex_unlock(&gc0307->lock);
	return ret;

done_mega_startup:

	gc0307_shutdown(1);
	if (gc0307_i2c_client)
		i2c_del_driver(&gc0307_i2c_driver);
	gc0307_i2c_client = NULL;
	gc0307->active = 0;

	FNC_EXIT(ret)
	mutex_unlock(&gc0307->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_shutdown
 * RETURN   :
 * NOTE     : uninitialize GC0307. called from emxx_cam.c close()
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_shutdown(int flag)
{
	int ret = 0;
	FNC_ENTRY;
	mutex_lock(&gc0307->lock);

	ret = gc0307_shutdown(1);

	if (gc0307_i2c_client)
		i2c_del_driver(&gc0307_i2c_driver);
	gc0307_i2c_client = NULL; 

	FNC_EXIT(ret)
	mutex_unlock(&gc0307->lock);
	return 0;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_mega_unregister
 * RETURN   :
 * NOTE     : unregist GC0307. called from emxx_cam.c shutdown
 * UPDATE   :
 ******************************************************************************/
static int emxx_mega_unregister(int flag)
{
	int ret = 0;
	FNC_ENTRY;

	kfree(gc0307);
	gc0307 = NULL;

	FNC_EXIT(ret)
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_mega.c
 * FUNCTION : emxx_cam_hw_register
 * RETURN   :
 * NOTE     : regist GC0307. called from emxx_cam.c startup
 * UPDATE   :
 ******************************************************************************/
int emxx_cam_hw_register(struct emxx_cam_hw_operations *hw)
{
	int ret = 0;
	const struct v4l2_queryctrl *ctrl;
	FNC_ENTRY;

	gc0307 = kzalloc(sizeof(*gc0307), GFP_KERNEL);
	if (gc0307 == NULL) {
		err("GC0307: gc0307 allocation failed!\n");
		FNC_EXIT(-ENOMEM)
		return -ENOMEM;
	}

	strlcpy(hw->name, "GC0307", sizeof(hw->name));

	hw->vidioc_queryctrl = emxx_mega_vidioc_queryctrl;
	hw->vidioc_querymenu = emxx_mega_vidioc_querymenu;
	hw->vidioc_g_ctrl    = emxx_mega_vidioc_g_ctrl;
	hw->vidioc_s_ctrl    = emxx_mega_vidioc_s_ctrl;
	hw->vidioc_s_fmt     = emxx_mega_vidioc_s_fmt;
	hw->prepare          = emxx_mega_prepare;
	//hw->trigger          = emxx_mega_trigger;
	//hw->sync             = emxx_mega_sync;
	hw->stream_on        = emxx_mega_stream_on;
	hw->stream_off       = emxx_mega_stream_off;
	hw->startup          = emxx_mega_startup;
	hw->shutdown         = emxx_mega_shutdown;
	hw->unregister       = emxx_mega_unregister;

	/* initial gc0307 */
	mutex_init(&gc0307->lock);

	gc0307->firmware_version = 0xff;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_OUTIMG_SIZE);
	gc0307->capture_size  = ctrl->default_value;

	hw->private = gc0307;

	FNC_EXIT(ret)
	return ret;
}
EXPORT_SYMBOL(emxx_cam_hw_register);


