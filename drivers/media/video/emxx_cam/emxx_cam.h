/*
 *  File Name	    : emxx_cam.h
 *  Function	    : CAMERA I/F Driver local header
 *  Release Version : Ver 1.00
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
#ifndef EMXX_CAM_H
#define EMXX_CAM_H

#include <linux/types.h>
#include <linux/videodev2.h>
#include <mach/emxx_cam.h>
#include <linux/proc_fs.h>


#define EMXX_CAM_MAJ_VER       0
#define EMXX_CAM_MIN_VER       1
#define EMXX_CAM_PATCH_VER     0

#ifndef V4L2_PIX_FMT_NV422
#define V4L2_PIX_FMT_NV422   V4L2_PIX_FMT_NV16
#endif

struct emxx_cam_prepare {
	__u32 actions:1;
	__u32 reset:1;
	__u32 syncmode:1;        /* CA_CSR SYNCMODE */
	__u32 synctype:1;        /* CA_CSR SYNCTYPE */
	__u32 data_id:1;         /* CA_CSR DATA_ID */
	__u32 vs_det:1;          /* CA_CSR VS_DET */
	__u32 hs_det:1;          /* CA_CSR HS_DET */
	__u32 clk_edge:1;        /* CA_CSR CLK_EDGE */
	__u32 data_det:1;        /* CA_CSR DATA_DET */
	__u32 vs_pol:1;          /* CA_CSR VS_POL */
	__u32 hs_pol:1;          /* CA_CSR HS_POL */
	__u32 width;             /* Image width in pixels. */
	__u32 height;            /* Image height in pixels. */
	struct v4l2_rect c;      /* Cropping rectangle */
	struct v4l2_rect bounds; /* Defines the window within capturing */
};

struct emxx_cam_hw_operations {
	__u8 name[32];
	int (*vidioc_queryctrl)(struct file *file, void *fh,
				struct v4l2_queryctrl *a);
	int (*vidioc_g_ctrl)(struct file *file, void *fh,
			     struct v4l2_control *a);
	int (*vidioc_s_ctrl)(struct file *file, void *fh,
			     struct v4l2_control *a);
	int (*vidioc_querymenu)(struct file *file, void *fh,
				struct v4l2_querymenu *a);
	int (*prepare)(struct emxx_cam_prepare *);
	int (*trigger)(int);
	int (*sync)(struct emxx_cam_prepare *);
	int (*stream_on)(int);
	int (*stream_off)(int);
	int (*startup)(int);
	int (*shutdown)(int);
	void *private;
	int (*unregister)(int);
};

extern int emxx_cam_hw_register(struct emxx_cam_hw_operations *hw);
extern int emxx_cam_hw_info(void);

#define err(format, arg...) printk(KERN_INFO format, ## arg)
#define info(format, arg...) printk(KERN_INFO format, ## arg)
#define warn(format, arg...) printk(KERN_INFO format, ## arg)
#define emerg(format, arg...) printk(KERN_INFO format, ## arg)
#define assert(expr) \
	if (unlikely(!(expr))) {					\
		printk(KERN_ERR "Assertion failed! %s,%s,%s,line=%d\n",	\
		       # expr, __FILE__, __func__, __LINE__);		\
	}

/*<- end of DEBUG code ***/
#endif

