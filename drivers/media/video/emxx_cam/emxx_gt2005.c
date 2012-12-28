
/*
 *  File Name       : emxx_gt2005.c
 *  Function        : gt2005 for CAMERA I/F Driver
 *
 *  Copyright (C) Renesas Electronics Corporation 2010
 *  Copyright (C) ShenZhen Livall Network Technology Co.,Ltd 2011
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

#include "emxx_cam.h"
#include "emxx_gt2005.h"


#define MODULE_NAME "gt2005"
#define KERN_CUSTOM KERN_DEBUG
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

/*===============================================================*/
/* I2C Functions                                                 */
/*===============================================================*/

/* flags */
static int i2c_stop;

/* prottyped */
static int gt2005_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
static int gt2005_i2c_remove(struct i2c_client *client);
static inline int gt2005_i2c_write(unsigned short cmd,
				      unsigned char *buf, unsigned char len);
static inline int gt2005_i2c_read(unsigned short cmd,
				     unsigned char *buf, unsigned char len);
static inline int gt2005_get_outimg_size(__u8 *value);
static inline int gt2005_set_prepare(struct emxx_cam_prepare *prepare);

static struct i2c_device_id cam_i2c_idtable[] = {
	{ I2C_SLAVE_CAM_NAME_GT2005, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cam_i2c_idtable);

static struct i2c_driver gt2005_i2c_driver = {
	.driver.name    = "i2c for gt2005",
	.id             = I2C_DRIVERID_I2CDEV, /* Fake ID */
	.id_table       = cam_i2c_idtable,
	.probe          = gt2005_i2c_probe,
	.remove         = gt2005_i2c_remove,
};
static struct i2c_client *gt2005_i2c_client;


/* i2c registerd function */
static int gt2005_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	gt2005_i2c_client = client;
	return 0;
}

static int gt2005_i2c_remove(struct i2c_client *client)
{
	gt2005_i2c_client = NULL;
	return 0;
}

static inline int gt2005_i2c_write(unsigned short cmd,
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

	msg.addr  = I2C_SLAVE_CAM_ADDR_GT2005;
	msg.flags = 0;
	msg.buf   = s_buf;
	msg.len   = len + sizeof(cmd);

	FNC_ENTRY;

	s_buf[0] = cmd >> 8;
	s_buf[1] = cmd & 0xff;
	memcpy(&s_buf[2], buf, len);

	if (i2c_stop)
		return -EIO;

	assert(gt2005_i2c_client);

	ret = i2c_transfer(gt2005_i2c_client->adapter, &msg, 1);
	if (1 != ret) {
		err("i2c_transfer: cmd : 0x%02x ret = %d\n", cmd, ret);
		i2c_stop = 1;
	} else {
		ret = 0;
	}

	FNC_EXIT(ret);
	return ret;
}

static inline int gt2005_i2c_read(unsigned short cmd,
				     unsigned char *buf, unsigned char len)
{

	int ret = 0;
	unsigned char RegBuf[2] = {0};
	struct i2c_msg msg[2];
	RegBuf[0] = cmd >> 8;
	RegBuf[1] = cmd & 0xff;

	msg[0].addr  = I2C_SLAVE_CAM_ADDR_GT2005;
	msg[0].flags = 0;
	msg[0].buf   = RegBuf;
	msg[0].len   = 2;

	msg[1].addr  = I2C_SLAVE_CAM_ADDR_GT2005;
	msg[1].flags = I2C_M_RD;
	msg[1].buf   = buf;
	msg[1].len   = len;
	

	
	FNC_ENTRY;

	if (i2c_stop)
		return -EIO;

	assert(gt2005_i2c_client);

	ret = i2c_transfer(gt2005_i2c_client->adapter, msg, 2);
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
		ret = gt2005_i2c_write(p[i].reg, &p[i].val, 1);
		//udelay(10);
		if (ret) {
			printk("ERROR: write_regs Sensor I2C [0x%04x]=0x%02x\n", p[i].reg, p[i].val );
			return ret;
		}

		//if ( 0x0201==p[i].reg && 0x00==p[i].val )
		//msleep(10);  /* delay 10ms after soft reset */	
	}

	FNC_EXIT(ret);
	return ret;
}


/*===============================================================*/

/*===============================================================*/
/* gt2005 Camera control flags                                */
/*===============================================================*/
enum {
	EMXX_GT2005_IDLE = 0,
	EMXX_GT2005_LIVE,
	EMXX_GT2005_CAPTURE,
	EMXX_GT2005_BREAK,
};

struct emxx_gt2005 {
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

struct emxx_gt2005 *gt2005;


/*===============================================================*/
/* gt2005 data structure. for VIDIOC_S_CTRL                   */
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
static const struct control_resize_info gt2005_outimg_size_menus[] =
{
	{0x01, "CIF : 352x288",   0, 0, 352,  288},
	{0x02, "VGA : 640x480",   0, 0, 640,  480},    /* default in EM1*/
	{0x03, "SXGA: 1024x768",  0, 0, 1024, 768},
	{0x04, "UXGA: 1600x1200", 0, 0, 1600, 1200},
};
#define NUM_OUTIMG_SIZE_MENUS ARRAY_SIZE(gt2005_outimg_size_menus)



/*===============================================================*/
/* gt2005 data structure. for VIDIOC_QUERYCTRL                */
/*===============================================================*/
static const struct v4l2_queryctrl no_ctrl = {
	.name  = "gt2005",
	.flags = V4L2_CTRL_FLAG_DISABLED,
};

/* default settings */
static const struct v4l2_queryctrl gt2005_ctrls[] = {
	{
		.id            = EMXX_RJ6ABA100_CID_OUTIMG_SIZE,
		.type          = V4L2_CTRL_TYPE_MENU,
		.name          = "Output image size",
		.minimum       = 0,
		.maximum       = (NUM_OUTIMG_SIZE_MENUS - 1),
		.step          = 1,
		.default_value = 4, /* VGA */
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
#define NUM_GT2005_CTRLS ARRAY_SIZE(gt2005_ctrls)


/*****************************************************************************
 * MODULE   : emxx_gt2005.c
 * FUNCTION : gt2005_setreg_outimg_size
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_OUTIMG_SIZE
 *          : set gt2005 output image size register group
 * UPDATE   :
 ******************************************************************************/

static inline int gt2005_setreg_outimg_size(__u8 value)
{
	struct control_resize_info *s;
	printk(KERN_CUSTOM "%s-%s: %d\n", MODULE_NAME, __func__, value);

	s = (struct control_resize_info *)gt2005_outimg_size_menus;
	s += value;

	if ((s->width == 640) && (s->height == 480)) {
		printk(KERN_CUSTOM "%s-%s: sensor_set_vga\n",
			MODULE_NAME, __func__);

		write_regs(sensor_set_preview,
			sizeof(sensor_set_preview)/sizeof(struct sensor_reg));

	} else if ((s->width == 2048) && (s->height == 1536)) {

		printk(KERN_CUSTOM "%s-%s: sensor_set_qxga\n",
			MODULE_NAME, __func__);
		write_regs(sensor_set_capture,
			sizeof(sensor_set_capture)/sizeof(struct sensor_reg));

	} else if ((s->width == 352) && (s->height == 288)) {

		printk(KERN_CUSTOM "%s-%s: sensor_set_cif\n",
			MODULE_NAME, __func__);
		write_regs(sensor_set_cif,
			sizeof(sensor_set_cif)/sizeof(struct sensor_reg));

	} else if ((s->width == 1600) && (s->height == 1200) ) {
		printk(KERN_CUSTOM "%s-%s: sensor_set_uxga\n",
			MODULE_NAME, __func__);
		write_regs(size_uxga,
			sizeof(size_uxga)/sizeof(struct sensor_reg));
		msleep(100);
	} else if ((s->width == 1024) && (s->height == 768)) {
		printk(KERN_CUSTOM "%s-%s: sensor_set_xga\n",
			MODULE_NAME, __func__);
		write_regs(sensor_set_capture,
			sizeof(sensor_set_capture)/sizeof(struct sensor_reg));

		write_regs(size_xga,
			sizeof(size_xga)/sizeof(struct sensor_reg));
	}
	return 0;
}



/* etc */
/*****************************************************************************
 * MODULE   : emxx_gt2005.c
 * FUNCTION : gt2005_set_liveview
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
#define SET_LIVEVIEW_STOP  0
#define SET_LIVEVIEW_START 1

static inline int gt2005_set_liveview(__u8 val)
{
	int ret = 0;

	printk(KERN_CUSTOM "%s-%s: %d\n", MODULE_NAME, __func__, val);

	if (SET_LIVEVIEW_START == val) {
		__u8 size;

		ret = gt2005_get_outimg_size(&size);
		if (ret)
			return ret;

		ret = gt2005_setreg_outimg_size(size);
	}
	if (!ret) {
		if (val == SET_LIVEVIEW_START)
			gt2005->state = EMXX_GT2005_LIVE;
		else
			gt2005->state = EMXX_GT2005_IDLE;
	}

	return ret;
}

/*****************************************************************************
 * MODULE   : emxx_gt2005.c
 * FUNCTION : gt2005_set_capture_read
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
static inline int gt2005_set_capture_read(__u8 data)
{
	int ret = 0;

	printk(KERN_CUSTOM "%s-%s: %d\n", MODULE_NAME, __func__, data);

	if (EMXX_GT2005_IDLE != gt2005->state)
		return -EBUSY;

	ret = gt2005_setreg_outimg_size(data);
	if (!ret)
		gt2005->state = EMXX_GT2005_CAPTURE;

	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_gt2005.c
 * FUNCTION : gt2005_shutdown
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
static inline int gt2005_shutdown(int flag)
{
	int ret = 0;
	static int init_once = 1;

	if (init_once) {
		printk(KERN_CUSTOM "%s-%s: Power up rear camera(init)-%d\n",
			MODULE_NAME, __func__, flag);
		/* 1.Disable front camera */
		gpio_direction_output(GPIO_FRONT_CAMERA_PWDN, 1); /*GPIO_P121*/
		/* Mask input & not pull */
		writel((readl(CHG_PULL16) & 0xf0ffffff), CHG_PULL16);
		writel((readl(CHG_PINSEL_G096) | (1 << 25)), CHG_PINSEL_G096);

		printk("2.reset pin and pwd pin pull down\n");
		gpio_direction_output(GPIO_REAR_CAMERA_RST, 0); /*GPIO_P98*/
		/* Mask input & not pull */
		writel((readl(CHG_PULL10) & 0xf0ffffff), CHG_PULL10);
		writel((readl(CHG_PINSEL_G096) | (1 << 2)), CHG_PINSEL_G096);
		msleep(3);

		/* Enable rear camera, they share a data bus. */
		gpio_direction_output(GPIO_REAR_CAMERA_PWDN, 0); /*GPIO_P68*/
		/* Mask input & not pull */
		writel((readl(CHG_PULL7) & 0xfffffff0), CHG_PULL7);
		writel((readl(CHG_PINSEL_G064) | (1 << 4)), CHG_PINSEL_G064);
		msleep(3);

		printk("3.reset pin and pwd pin pull up\n");
		gpio_direction_output(GPIO_REAR_CAMERA_RST, 1); /*GPIO_P98*/
		msleep(3);
		gpio_direction_output(GPIO_REAR_CAMERA_PWDN, 1); /*GPIO_P68*/

		init_once = 0;
		return ret;
	}

	if (flag) {
		printk(KERN_CUSTOM "%s-%s: Power up rear camera\n",
			MODULE_NAME, __func__);
		//printk("power  off\n");
		/* Disable front camera */
		gpio_direction_output(GPIO_FRONT_CAMERA_PWDN, 1);
		gpio_direction_output(GPIO_REAR_CAMERA_PWDN, 0);
		//gpio_direction_output(GPIO_REAR_CAMERA_RST, 0);
	} else {
		printk(KERN_CUSTOM "%s-%s: Power down rear camera\n",
			MODULE_NAME, __func__);
		//printk("power  on\n");
		gpio_direction_output(GPIO_FRONT_CAMERA_PWDN, 1);
		gpio_direction_output(GPIO_REAR_CAMERA_PWDN, 1);
		msleep(5);
	}

	return ret;
}

/* EMXX_RJ6ABA100_CID_OUTIMG_SIZE */
/*****************************************************************************
 * MODULE   : emxx_gt2005.c
 * FUNCTION : gt2005_get_outimg_size / gt2005_set_outimg_size
 * RETURN   :
 * NOTE     : EMXX_RJ6ABA100_CID_FW
 * UPDATE   :
 ******************************************************************************/
static inline int gt2005_get_outimg_size(__u8 *value)
{
	/* printk(KERN_CUSTOM "%s-%s: %d\n", MODULE_NAME, __func__,
		gt2005->capture_size); */

	*value = gt2005->capture_size;
	return 0;
}

static inline int gt2005_set_outimg_size(struct emxx_cam_prepare *prepare,
						__u8 value)
{
	int ret = 0;
	struct control_resize_info *s;

	printk(KERN_CUSTOM "%s-%s: %d\n", MODULE_NAME, __func__, value);

	s = (struct control_resize_info *)gt2005_outimg_size_menus;
	s += value;

	prepare->bounds.left   = s->x;
	prepare->bounds.top    = s->y;
	prepare->bounds.width  = s->width  - (s->x);
	prepare->bounds.height = s->height - (s->y);

	prepare->c = prepare->bounds;
	prepare->width  = prepare->c.width ;
	prepare->height = prepare->c.height;

	gt2005->capture_size = value;
	/* gt2005->reset = 1; */

	return ret;
}

/*****************************************************************************
 * MODULE   : emxx_gt2005.c
 * FUNCTION : gt2005_start_capture_read
 * RETURN   :
 * NOTE     :
 * CREATE   :
 * UPDATE   :
 ******************************************************************************/
static inline int gt2005_start_capture_read(void)
{
	int ret = 0;

	printk(KERN_CUSTOM "%s-%s\n", MODULE_NAME, __func__);

	ret = gt2005_set_capture_read(gt2005->capture_size);

	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_gt2005.c
 * FUNCTION : ctrl_by_id
 * RETURN   :
 * NOTE     : get gt2005_ctrls id
 * UPDATE   :
 ******************************************************************************/
static const struct v4l2_queryctrl *ctrl_by_id(unsigned int id)
{
	unsigned int i;

	for (i = 0; i < NUM_GT2005_CTRLS; i++)
		if (gt2005_ctrls[i].id == id)
			return gt2005_ctrls + i;
	return NULL;
}

static int emxx_color_effect(struct v4l2_control *c)
{
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
			write_regs(default_ev,sizeof(default_ev)/sizeof(struct sensor_reg));
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
			write_regs(contrast_0,sizeof(contrast_0)/sizeof(struct sensor_reg));
			break;
	}
	return 0;
}
/* i/f function with emxx_rear_cam.c */

/*****************************************************************************
 * MODULE   : emxx_gt2005.c
 * FUNCTION : emxx_gt2005_vidioc_queryctrl
 * RETURN   :
 * NOTE     : called from emxx_rear_cam.c ioctl(VIDIOC_QUERYCTRL)
 * UPDATE   :
 ******************************************************************************/
static int emxx_gt2005_vidioc_queryctrl(struct file *file,
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
 * MODULE   : emxx_gt2005.c
 * FUNCTION : emxx_gt2005_vidioc_querymenu
 * RETURN   :
 * NOTE     : called from emxx_rear_cam.c ioctl(VIDIOC_QUERYMENU)
 * UPDATE   :
 ******************************************************************************/
static int emxx_gt2005_vidioc_querymenu(struct file *file,
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

	mutex_lock(&gt2005->lock);
	switch (m->id) {
	default:
		ret = -EINVAL;
		break;
	}

	FNC_EXIT(ret)
	mutex_unlock(&gt2005->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_gt2005.c
 * FUNCTION : emxx_gt2005_vidioc_g_ctrl
 * RETURN   :
 * NOTE     : called from emxx_rear_cam.c ioctl(VIDIOC_G_CTRL)
 * UPDATE   :
 ******************************************************************************/
static int emxx_gt2005_vidioc_g_ctrl(struct file *file, void *fh,
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
	mutex_lock(&gt2005->lock);
	switch (c->id) {
	default:
		ret = -EINVAL;
	}

	if (!ret)
		c->value = val;

	FNC_EXIT(ret)
	mutex_unlock(&gt2005->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_gt2005.c
 * FUNCTION : emxx_gt2005_vidioc_s_ctrl
 * RETURN   :
 * NOTE     : called from emxx_rear_cam.c ioctl(VIDIOC_S_CTRL)
 * UPDATE   :
 ******************************************************************************/
static int emxx_gt2005_vidioc_s_ctrl(struct file *file,
				    struct v4l2_control *c)
{
	int ret = 0;
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
	mutex_lock(&gt2005->lock);
	switch (c->id) {
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
	mutex_unlock(&gt2005->lock);
	return ret;
}

/*****************************************************************************
 * MODULE   : emxx_gt2005.c
 * FUNCTION : emxx_gt2005_vidioc_s_fmt
 * RETURN   :
 * NOTE     : called from emxx_rear_cam.c ioctl(vidioc_s_fmt_vid_cap)
 * UPDATE   :
 ******************************************************************************/
int emxx_gt2005_vidioc_s_fmt(struct emxx_cam_prepare *prepare,
					struct v4l2_format *f)
{
	__u8 index;
	int ret = -EINVAL;
	struct control_resize_info *s;
	FNC_ENTRY;

	mutex_lock(&gt2005->lock);

	ret = gt2005_get_outimg_size(&index);
	if (ret)
		goto out;

	s = (struct control_resize_info *)gt2005_outimg_size_menus;
	s += index;

	for(index = 0; index < NUM_OUTIMG_SIZE_MENUS; index ++) {
		if ((f->fmt.pix.width == gt2005_outimg_size_menus[index].width) &&
			(f->fmt.pix.height == gt2005_outimg_size_menus[index].height)) {

			gt2005_set_prepare(prepare);

			ret = gt2005_set_outimg_size(prepare, index);
			if (ret)
				goto out;

			/* ret = gt2005_setreg_outimg_size(index);
			if (ret)
				goto out; */
			goto out;
		}
	}
out:
	FNC_EXIT(ret)
	mutex_unlock(&gt2005->lock);
	return ret;
}

/****************************************************************************
 * MODULE   : emxx_gt2005.c
 * FUNCTION : gt2005_set_prepare
 * RETURN   :
 * NOTE     : called from emxx_rear_cam.c
 * UPDATE   :
 ******************************************************************************/
static inline int gt2005_set_prepare(struct emxx_cam_prepare *prepare)
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
 * MODULE   : emxx_gt2005.c
 * FUNCTION : emxx_gt2005_stream_on
 * RETURN   :
 * NOTE     : called from emxx_rear_cam.c ioctl(VIDIOC_STREAMON)
 * UPDATE   :
 ******************************************************************************/
static int emxx_gt2005_stream_on(int flag)
{
	int ret = 0;
	FNC_ENTRY;
	mutex_lock(&gt2005->lock);

	ret = gt2005_set_liveview(SET_LIVEVIEW_START);

	FNC_EXIT(ret)
	mutex_unlock(&gt2005->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_gt2005.c
 * FUNCTION : emxx_gt2005_stream_off
 * RETURN   :
 * NOTE     : called from emxx_rear_cam.c ioctl(VIDIOC_STREAMOFF)
 * UPDATE   :
 ******************************************************************************/
static int emxx_gt2005_stream_off(int flag)
{
	int ret = 0;
	FNC_ENTRY;
	mutex_lock(&gt2005->lock);

	ret = gt2005_set_liveview(SET_LIVEVIEW_STOP);

	FNC_EXIT(ret)
	mutex_unlock(&gt2005->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_gt2005.c
 * FUNCTION : emxx_gt2005_startup
 * RETURN   :
 * NOTE     : initialize gt2005. called from emxx_rear_cam.c open()
 * UPDATE   :
 ******************************************************************************/
static int emxx_gt2005_startup(int flag)
{
	int ret = 0;
	FNC_ENTRY;
	mutex_lock(&gt2005->lock);

	i2c_stop = 0;
	gt2005->state = EMXX_GT2005_IDLE;

	/* Reset Camera Module
	 */
	gt2005_shutdown(0);

	ret = i2c_add_driver(&gt2005_i2c_driver);
	d1b("i2c_add_driver(%d, %p)\n", ret, gt2005_i2c_client);

	if (ret < 0) {
		err("i2c: Driver registration failed,"
		    "module not inserted.\n");
		goto done_mega_startup;
	} else if (NULL == gt2005_i2c_client)    {
		i2c_del_driver(&gt2005_i2c_driver);
		err("i2c: Device was not detected.\n");
		ret = -EINVAL;

		goto done_mega_startup;
	}

	/* gt2005 initialization */
	{
		printk(KERN_CUSTOM "%s-%s: sensor_initialize\n",
			MODULE_NAME, __func__);
		unsigned char value;
		
		gt2005_i2c_read(0x0000, &value, 1);
		printk("gt2005 chip id = %02x\n",value);

		gt2005_i2c_read(0x0001, &value, 1);
		printk("gt2005 chip id = %02x\n",value);

		write_regs(sensor_initialize,  sizeof(sensor_initialize) \
			/sizeof(struct sensor_reg));
		gt2005->state = EMXX_GT2005_IDLE;
	}

	gt2005->active = 1;

	FNC_EXIT(ret)
	mutex_unlock(&gt2005->lock);
	return ret;

done_mega_startup:

	gt2005_shutdown(1);
	if (gt2005_i2c_client)
		i2c_del_driver(&gt2005_i2c_driver);
	gt2005_i2c_client = NULL;
	gt2005->active = 0;

	FNC_EXIT(ret)
	mutex_unlock(&gt2005->lock);
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_gt2005.c
 * FUNCTION : emxx_gt2005_shutdown
 * RETURN   :
 * NOTE     : uninitialize gt2005. called from emxx_rear_cam.c close()
 * UPDATE   :
 ******************************************************************************/
static int emxx_gt2005_shutdown(int flag)
{
	int ret = 0;
	FNC_ENTRY;
	mutex_lock(&gt2005->lock);
	ret = gt2005_shutdown(1);

	if (gt2005_i2c_client)
		i2c_del_driver(&gt2005_i2c_driver);
	/* gt2005_i2c_client = NULL; */

	FNC_EXIT(ret)
	mutex_unlock(&gt2005->lock);
	return 0;
}


/*****************************************************************************
 * MODULE   : emxx_gt2005.c
 * FUNCTION : emxx_gt2005_unregister
 * RETURN   :
 * NOTE     : unregist gt2005. called from emxx_rear_cam.c shutdown
 * UPDATE   :
 ******************************************************************************/
static int emxx_gt2005_unregister(int flag)
{
	int ret = 0;
	FNC_ENTRY;

	kfree(gt2005);
	gt2005 = NULL;

	FNC_EXIT(ret)
	return ret;
}


/*****************************************************************************
 * MODULE   : emxx_gt2005.c
 * FUNCTION : emxx_rear_cam_hw_register
 * RETURN   :
 * NOTE     : regist gt2005. called from emxx_rear_cam.c startup
 * UPDATE   :
 ******************************************************************************/
int emxx_rear_cam_hw_register(struct emxx_cam_hw_operations *hw)
{
	int ret = 0;
	const struct v4l2_queryctrl *ctrl;
	FNC_ENTRY;

	gt2005 = kzalloc(sizeof(*gt2005), GFP_KERNEL);

	if (gt2005 == NULL) {
		err("gt2005: gt2005 allocation failed!\n");
		FNC_EXIT(-ENOMEM)
		return -ENOMEM;
	}

	strlcpy(hw->name, "GT2005", sizeof(hw->name));

	hw->vidioc_queryctrl = emxx_gt2005_vidioc_queryctrl;
	hw->vidioc_querymenu = emxx_gt2005_vidioc_querymenu;
	hw->vidioc_g_ctrl    = emxx_gt2005_vidioc_g_ctrl;
	hw->vidioc_s_ctrl    = emxx_gt2005_vidioc_s_ctrl;
	hw->vidioc_s_fmt     = emxx_gt2005_vidioc_s_fmt;
	/* hw->prepare          = emxx_gt2005_prepare;
	hw->trigger          = emxx_gt2005_trigger;
	hw->sync             = emxx_gt2005_sync; */
	hw->stream_on        = emxx_gt2005_stream_on;
	hw->stream_off       = emxx_gt2005_stream_off;
	hw->startup          = emxx_gt2005_startup;
	hw->shutdown         = emxx_gt2005_shutdown;
	hw->unregister       = emxx_gt2005_unregister;

	/* initial gt2005 */
	mutex_init(&gt2005->lock);

	gt2005->firmware_version = 0xff;

	ctrl = ctrl_by_id(EMXX_RJ6ABA100_CID_OUTIMG_SIZE);
	gt2005->capture_size  = ctrl->default_value;

	hw->private = gt2005;

	FNC_EXIT(ret)
	return ret;
}
EXPORT_SYMBOL(emxx_rear_cam_hw_register);


