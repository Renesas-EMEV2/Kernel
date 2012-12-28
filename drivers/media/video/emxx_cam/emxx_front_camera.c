/*
 *  File Name       : emxx_cam.c
 *  Function        : CAMERA I/F Driver
 *
 *  Copyright (C) Renesas Electronics Corporation 2011
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


/*
 * Howto porting drivers to the new generic fifo API:
 *
 * - Modify the declaration of the "struct kfifo *" object into a
 *   in-place "struct kfifo" object
OK!
 
 * - Init the in-place object with kfifo_alloc() or kfifo_init()

 OK!

 
 *   Note: The address of the in-place "struct kfifo" object must be
 *   passed as the first argument to this functions


 * - Replace the use of __kfifo_put into kfifo_in and __kfifo_get
 *   into kfifo_out
OK

 
 * - Replace the use of kfifo_put into kfifo_in_locked and kfifo_get
 *   into kfifo_out_locked
NO NEED

 
 *   Note: the spinlock pointer formerly passed to kfifo_init/kfifo_alloc
 *   must be passed now to the kfifo_in_locked and kfifo_out_locked
 *   as the last parameter.


 
 * - All formerly name __kfifo_* functions has been renamed into kfifo_*
 */

#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/poll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kfifo.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/io.h>
#ifdef CONFIG_EMXX_ANDROID
#include <linux/wakelock.h>
#endif /* CONFIG_EMXX_ANDROID */
#include <mach/pmu.h>
#include <mach/smu.h>
#include <mach/emxx_mem.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#if defined(CONFIG_PM) || defined(CONFIG_DPM)
#include <linux/freezer.h>
#include <mach/pm.h>
#include <mach/pwc.h>
#endif /* CONFIG_PM || CONFIG_DPM */
#ifdef CONFIG_VIDEO_EMXX
#include <mach/emxx_v4l2.h>
#endif /* CONFIG_VIDEO_EMXX */

#define EMXX_CAM_MAKING_DEBUG


//* - Replace the use of __kfifo_put into kfifo_in and __kfifo_get
//*	into kfifo_out
#define __kfifo_put kfifo_in
#define __kfifo_get kfifo_out


#include "emxx_cam.h"

#define CAM_NAME "emxx_camera"
#if 1
#define EMXX_CAM_MAX_BUFNBRS 4
#else
#define EMXX_CAM_MAX_BUFNBRS VIDEO_MAX_FRAME
#endif

/* #define CAM_FPS_DEBUG */
#define EMXX_CAM_USE_MMAP    1

#ifdef CAM_FPS_DEBUG
#define ZERO_VALUE 0
unsigned int transmit_end_cnt = ZERO_VALUE;
unsigned int vsync_detect_cnt = ZERO_VALUE;
unsigned int deq_done_push_cnt = ZERO_VALUE;
unsigned int deq_done_pull_cnt = ZERO_VALUE;
unsigned int grab_push_cnt = ZERO_VALUE;
unsigned int grab_pull_cnt = ZERO_VALUE;

unsigned int dma_run_cnt = ZERO_VALUE;
#endif

#define MODULE_NAME "emxx_cam"
#define KERN_CUSTOM KERN_DEBUG

/*** DEBUG code by the making ->*/
#ifdef EMXX_CAM_MAKING_DEBUG
#define logv printk

static int debug = 10;

#include <linux/moduleparam.h>

#define FNC_ENTRY	\
	if (debug == 1 || debug >= 9) {	\
		printk(KERN_INFO "entry:%s\n", __func__); \
	}

#define FNC_EXIT_N	\
	if (debug == 1 || debug >= 9) {	\
		printk(KERN_INFO "exit: %s:%d\n", __func__, __LINE__); \
	}

#define FNC_EXIT(r)	\
	if (debug == 1 || debug >= 9) {	\
		printk(KERN_INFO "exit:%d :%s:%d\n", r, __func__, __LINE__); \
	}

#define d0b(fmt, args...)	\
	{ \
		printk(KERN_INFO "%s:%d: " fmt, __func__, __LINE__, ## args); \
	}
#define d1b(fmt, args...)	\
	if (debug == 1 || debug >= 9) {	\
		printk(KERN_INFO "%s:%d: " fmt, __func__, __LINE__, ## args); \
	}
#define d2b(fmt, args...)	\
	if (debug == 2 || debug >= 9) {	\
		printk(KERN_INFO "%s:%d: " fmt, __func__, __LINE__, ## args); \
	}
#define d3b(fmt, args...)	\
	if (debug == 3 || debug >= 9) {	\
		printk(KERN_INFO " --M%d-- " fmt, debug, ## args); \
	}
#define d4b(fmt, args...)	\
	if (debug == 4 || debug >= 9) {	\
		printk(KERN_INFO " --M%d-- " fmt, debug, ## args); \
	}
#define d5b(fmt, args...)	\
	if (debug == 5 || debug >= 9) {	\
		printk(KERN_INFO " --M%d-- " fmt, debug, ## args); \
	}
#define d6b(fmt, args...)	\
	if (debug == 6 || debug >= 9) {	\
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

enum {
	CAM_MMAP = 0,
	CAM_READ,
};

enum {
	CAM_OFF = 0,
	CAM_ON,
};

#define CAM_IPU_OFF  CAM_OFF
#define CAM_IPU_ON   CAM_ON

enum {
	BUF_FIXED = 0,
	BUF_IPU,
};

enum {
	CAM_BUF_IDLE = 0,
	CAM_BUF_QUEUED,
	CAM_BUF_GRABBING,
	CAM_BUF_DONE,
	CAM_BUF_BREAK,
};

struct emxx_cam_mapping {
	unsigned int count;
	unsigned long start;
	unsigned long end;
#ifdef CONFIG_VIDEO_EMXX
	struct vm_operations_struct *vm_ops;
#endif
};

//liuxu: new defined
struct emxx_cam_buffer{
	int index;
	__u32 padr;
	__u32 vadr;
	__u32 offset;
	int blocksize;
	struct timeval timestamp;
	unsigned long sequence;
};

struct emxx_cam_fmt {
	__u8 description[32];      /* Description string */
	__u32 pixelformat;         /* Format fourcc      */
	int depth;                 /* bit/pixel          */
	int flags;
	int boundary;
};

struct emxx_cam_frames {
	struct kfifo enq;
	spinlock_t enq_lock;
	struct emxx_cam_buffer *buff;
	wait_queue_head_t proc_list;
	unsigned int cnt;
	unsigned int max;
	int (*update)(void *);
	__u32 blocksize;
	enum v4l2_memory memory;
	const struct emxx_cam_fmt *fmt;
	int buf_type;
};

/* CAMIF status */
#define B_MAINOR (0x01 << 3)
#define B_MAINTC (0x01 << 2)
#define B_DMAERR (0x01 << 1)
#define B_CAMVS  (0x01 << 0)

struct emxx_cam {
	__u32 status;

	__u32 setup:1;
	__u32 reset:1;
	__u32 stop:1;
	__u32 action:1; /* 0:mmap 1:read */
	__u32 reading:1;
	__u32 streaming:1;
	__u32 frames_active:1;
	__u32 userptr:1;
	unsigned int mapping;

	__u32 ipu:1;

	struct mutex lock;

	struct kfifo deq_done;
	spinlock_t deq_done_lock;
	unsigned long sequence;

#ifdef CONFIG_EMXX_ANDROID
	struct wake_lock idle_lock; /* suspend control */
#endif /* CONFIG_EMXX_ANDROID */

	spinlock_t cam_lock;

	struct task_struct *th;

	__u32 width;              /* Image width in pixels. */
	__u32 height;             /* Image height in pixels. */
	struct v4l2_rect c;       /* Cropping rectangle */
	struct v4l2_rect bounds;  /* Defines the window within capturing */
	const struct emxx_cam_fmt *fmt;
	struct mutex frames_lock;
	struct emxx_cam_frames *grab;
	struct emxx_cam_prepare pre;

	struct emxx_cam_hw_operations hw;
	struct video_device *vdev;
	struct proc_dir_entry *proc_entry;
	int open_count;
	__u32 active_number;
	__u32 used_number;
};

#if 1 /* XXX */
static int warming_up = 1;
#endif


struct emxx_cam_private {
	int number;
};

/*
 * Camera I/F Functions
 */

struct emxx_camif {
	__u32 status;
	struct mutex lock;

	__u32 update:1;

	__u8 mirror;
	__u8 bngr;
	__u8 cbgr;
	__u8 crgr;
	__s8 bnzr;
	__s8 cbzr;
	__s8 crzr;
};

static struct emxx_camif *camif;

#define CA_STATUS        IO_ADDRESS(EMXX_CAM_BASE + 0x0000)
#define CA_RAWSTATUS     IO_ADDRESS(EMXX_CAM_BASE + 0x0004)
#define CA_ENSET         IO_ADDRESS(EMXX_CAM_BASE + 0x0008)
#define CA_ENCLR         IO_ADDRESS(EMXX_CAM_BASE + 0x000C)
#define CA_FFCLR         IO_ADDRESS(EMXX_CAM_BASE + 0x0010)
#define CA_ERRORADR      IO_ADDRESS(EMXX_CAM_BASE + 0x0014)
#define CA_CSR           IO_ADDRESS(EMXX_CAM_BASE + 0x0020)
#define CA_X1R           IO_ADDRESS(EMXX_CAM_BASE + 0x0030)
#define CA_X2R           IO_ADDRESS(EMXX_CAM_BASE + 0x0034)
#define CA_Y1R           IO_ADDRESS(EMXX_CAM_BASE + 0x0038)
#define CA_Y2R           IO_ADDRESS(EMXX_CAM_BASE + 0x003C)
#define CA_BNZR          IO_ADDRESS(EMXX_CAM_BASE + 0x0040)
#define CA_BNGR          IO_ADDRESS(EMXX_CAM_BASE + 0x0044)
#define CA_CBZR          IO_ADDRESS(EMXX_CAM_BASE + 0x0048)
#define CA_CBGR          IO_ADDRESS(EMXX_CAM_BASE + 0x004C)
#define CA_CRZR          IO_ADDRESS(EMXX_CAM_BASE + 0x0050)
#define CA_CRGR          IO_ADDRESS(EMXX_CAM_BASE + 0x0054)
#define CA_DMACNT        IO_ADDRESS(EMXX_CAM_BASE + 0x0080)
#define CA_FRAME         IO_ADDRESS(EMXX_CAM_BASE + 0x0084)
#define CA_DMAREQ        IO_ADDRESS(EMXX_CAM_BASE + 0x0088)
#define CA_DMASTOP       IO_ADDRESS(EMXX_CAM_BASE + 0x008C)
#define CA_LINESIZE_MAIN IO_ADDRESS(EMXX_CAM_BASE + 0x0100)
#define CA_XRATIO_MAIN   IO_ADDRESS(EMXX_CAM_BASE + 0x0104)
#define CA_YRATIO_MAIN   IO_ADDRESS(EMXX_CAM_BASE + 0x0108)
#define CA_DMAX_MAIN     IO_ADDRESS(EMXX_CAM_BASE + 0x010C)
#define CA_DMAY_MAIN     IO_ADDRESS(EMXX_CAM_BASE + 0x0110)
#define CA_YPLANE_A      IO_ADDRESS(EMXX_CAM_BASE + 0x0114)
#define CA_UVPLANE_A     IO_ADDRESS(EMXX_CAM_BASE + 0x0118)
#define CA_VPLANE_A      IO_ADDRESS(EMXX_CAM_BASE + 0x0244)
#define CA_YPLANE_B      IO_ADDRESS(EMXX_CAM_BASE + 0x011C)
#define CA_UVPLANE_B     IO_ADDRESS(EMXX_CAM_BASE + 0x0120)
#define CA_VPLANE_B      IO_ADDRESS(EMXX_CAM_BASE + 0x0248)
#define CA_MODULECONT    IO_ADDRESS(EMXX_CAM_BASE + 0x022C)
#define CA_UPDATE        IO_ADDRESS(EMXX_CAM_BASE + 0x0230)
#define CA_MIRROR        IO_ADDRESS(EMXX_CAM_BASE + 0x0234)
#define CA_OD_BYTELANE   IO_ADDRESS(EMXX_CAM_BASE + 0x0238)
#define CA_X3R           IO_ADDRESS(EMXX_CAM_BASE + 0x0240)
#define CA_OD_BYTELANE2  IO_ADDRESS(EMXX_CAM_BASE + 0x0254)
#define CA_QOS           IO_ADDRESS(EMXX_CAM_BASE + 0x0258)

/* #define M_CA_ENSET (B_MAINOR | B_MAINTC | B_DMAERR | B_CAMVS) */
#define M_CA_ENSET (B_MAINOR | B_MAINTC)

#define S_656MODE       14
#define S_PIXEL_YUV     13
#define S_SYNCTYPE      12
#define S_PIXELMODE     11
#define S_DATA_OD       10
#define S_DATA_ID       9
#define S_LD_TMG        8
#define S_VS_DET        7
#define S_HS_DET        6
#define S_LIMITSEL      5
#define S_SYNCMODE      4
#define S_CLK_EDGE      3
#define S_DATA_DET      2
#define S_VS_POL        1
#define S_HS_POL        0

#define S_SUBYUV        13
#define S_MAINYUV       12
#define S_SUBREC        10
#define S_MAINREC       8
#define S_SUBMODE       6
#define S_MAINMODE      4
#define S_SBRESIZE      3
#define S_MNRESIZE      2
#define S_PCULLR        0

#define S_SUBFRM        2
#define S_MAINFRM       0

struct camif_reg {
	__u32 csr;
	__u32 x1r;
	__u32 x2r;
	__u32 x3r;
	__u32 y1r;
	__u32 y2r;
	__u32 dmacnt;
	__u32 od_bytelane;
	__u32 od_bytelane2;
	__u32 yplane_a;
	__u32 uvplane_a;
	__u32 vplane_a;
	__u32 frame;
	__u32 dmax_main;
	__u32 dmay_main;
	__u32 linesize_main;
	__u32 xratio_main;
	__u32 yratio_main;
};



/*!
 * start CAM module power supply
 * @param void
 * @retval 0 successful
 * @retval -1 failed
 */

#define P2_POWER_WAIT 20000000

#define P2_SEQ_BUSY 0x80

#define CAM_POWERDOWN 1
#define CAM_RETENTION	0

#define P2_SWON		0x1
#define P2_PDON		0x100


static int emxx_camif_vidioc_queryctrl(struct file *file, void *fh,
					struct v4l2_queryctrl *a)
{
//TODO:Cam IF set formate function should be added
	return 0;
}

static int emxx_camif_vidioc_querymenu(struct file *file, void *fh,
					struct v4l2_querymenu *m)
{
	return 0;
}

static int emxx_camif_vidioc_g_ctrl(struct file *file, void *fh,
				     struct v4l2_control *c)
{
	return 0;
}

static int emxx_camif_vidioc_s_ctrl(struct file *file, void *fh,
				     struct v4l2_control *c)
{
	return 0;
}

#define FORMAT_FLAGS_PACKED       0x00
#define FORMAT_FLAGS_PLANAR       0x01

static const struct emxx_cam_fmt emxx_cam_formats[] = {
	{
		.description    = "YUYV : YUV422 Interleave",
		/* Y U Y V Y U Y V Y U Y V Y U Y V Y U Y V */
		.pixelformat    = V4L2_PIX_FMT_YUYV,
		/* Y U Y V Y U Y V Y U Y V Y U Y V Y U Y V */
		.depth          = 16,
		.flags          = FORMAT_FLAGS_PACKED,
		.boundary       = 1,
	}, {
#if 1  /*need to reconfirm whether support this format*/
		.description	= "UYVY : YUV 422 Interleave",
		 /* U Y V Y U Y V Y U Y V Y U Y V Y U Y V Y */
		.pixelformat	= V4L2_PIX_FMT_UYVY,
		 /* U Y V Y U Y V Y U Y V Y U Y V Y U Y V Y */
		.depth		= 16,
		.flags		= FORMAT_FLAGS_PACKED,
		.boundary	= 1,
#endif
	}, {
		.description    = "YUV422P : YUV 422 Planar",
		/* Y Y Y Y Y Y Y Y */
		.pixelformat    = V4L2_PIX_FMT_YUV422P,
		/* Y Y Y Y Y Y Y Y */
		.depth          = 16,
		/* U U U U U U U U */
		.flags          = FORMAT_FLAGS_PLANAR,
		/* V V V V V V V V */
		.boundary       = 3,
	}, {
		.description    = "NV422 : YUV 422 Semi-Planar",
		/* Y Y Y Y Y Y Y Y */
		.pixelformat    = V4L2_PIX_FMT_NV422,
		/* Y Y Y Y Y Y Y Y */
		.depth          = 16,
		/* U V U V U V U V */
		.flags          = FORMAT_FLAGS_PLANAR,
		/* U V U V U V U V */
		.boundary       = 2,
	}, {
		.description    = "YUV420 : YUV 420 Planar",
		/* Y Y Y Y Y Y Y Y */
		.pixelformat    = V4L2_PIX_FMT_YUV420,
		/* Y Y Y Y Y Y Y Y */
		.depth          = 12,
		/* U U U U         */
		.flags          = FORMAT_FLAGS_PLANAR,
		/* V V V V         */
		.boundary       = 3,
#if 0 /* not supported */
/* @@	}, { */
/* @@		.description	= "YVU420 : YUV 420 Planar",
/ * Y Y Y Y Y Y Y Y * / */
/* @@		.pixelformat	= V4L2_PIX_FMT_YVU420,
/ * Y Y Y Y Y Y Y Y * / */
/* @@		.depth		= 12,
/ * V V V V         * / */
/* @@		.flags		= FORMAT_FLAGS_PLANAR,
/ * U U U U         * / */
/* @@		.boundary	= 3, */
#endif
	}, {
		.description    = "NV12 : YUV 420 Semi-Planar",
		/* Y Y Y Y Y Y Y Y */
		.pixelformat    = V4L2_PIX_FMT_NV12,
		/* Y Y Y Y Y Y Y Y */
		.depth          = 12,
		/* U V U V U V U V */
		.flags          = FORMAT_FLAGS_PLANAR,
		.boundary       = 2,
#if 0 /* not supported */
/* @@	}, { */
/* @@		.description	= "NV21 : YUV 420 Semi-Planar",
/ * Y Y Y Y Y Y Y Y * / */
/* @@		.pixelformat	= V4L2_PIX_FMT_NV21,
/ * Y Y Y Y Y Y Y Y * / */
/* @@		.depth		= 12,
/ * V U V U V U V U * / */
/* @@		.flags		= FORMAT_FLAGS_PLANAR, */
/* @@		.boundary	= 2, */
#endif
	}
};
#define NUM_CAM_FORMATS ARRAY_SIZE(emxx_cam_formats)
#define CAM_VERSION KERNEL_VERSION(EMXX_CAM_MAJ_VER, EMXX_CAM_MIN_VER, \
				   EMXX_CAM_PATCH_VER)

//Global variables by liuxu
static struct video_device *vdev;
static spinlock_t cam_lock;
static struct emxx_cam_hw_operations sensor_ops;
static struct emxx_cam_buffer buffers[EMXX_CAM_MAX_BUFNBRS];
static wait_queue_head_t proc_list;
static struct kfifo  empty_fifo;
static struct kfifo  filled_fifo;
static struct emxx_cam_fmt current_fmt;
static int current_width, current_height;
static int current_sequence=0;
static int transfering_buf;
static int cam_opened=0;
static int streaming=0;
static struct wake_lock idle_lock;

static int emxx_camif_set(struct emxx_cam_prepare * pre);
static int emxx_cam_dma_transfer(struct emxx_cam_buffer *buffer);

static int camif_power_on(int mode)
{

	int pv_seq;
	int count = P2_POWER_WAIT;
	FNC_ENTRY;

	do {
		pv_seq = readl(SMU_SEQ_BUSY) & P2_SEQ_BUSY;
	} while (pv_seq != 0 && count--);

	if (count == 0) {
		err("power busy flag\n");
		return -1;
	}
	if (mode == CAM_POWERDOWN) {
		writel(readl(SMU_P2_SWON) | P2_SWON | P2_PDON,
		       SMU_P2_SWON);
	} else {
		writel((readl(SMU_P2_SWON) | P2_SWON) & ~P2_PDON,
		       SMU_P2_SWON);
	}
	count = P2_POWER_WAIT;

	do {
		pv_seq = readl(SMU_SEQ_BUSY) & P2_SEQ_BUSY;
	} while (pv_seq != 0 && count--);

	if (count == 0) {
		err("power busy flag\n");
		return -1;
	}

	// mdelay(100);
	FNC_EXIT_N;
	return 0;
}

static inline const struct emxx_cam_fmt *format_by_pixelformat(
	__u32 pixelformat)
{
	unsigned int i;
	FNC_ENTRY;

	for (i = 0; i < NUM_CAM_FORMATS; i++)
		if (emxx_cam_formats[i].pixelformat == pixelformat)
			return emxx_cam_formats + i;


		
	FNC_EXIT_N;
//	return NULL;
	return emxx_cam_formats + 0;// Avoid Kernel shut down.
}

static inline __u32 get_sizeimage(const struct emxx_cam_fmt *fmt,
				  __u32 width, __u32 height,
				  __u32 *bytesperline)
{
	__u32 sizeimage;
	FNC_ENTRY;

	if (fmt->flags & FORMAT_FLAGS_PLANAR) {
		*bytesperline = width; /* Y plane */
		sizeimage = (width * height * fmt->depth) >> 3;
	} else {
		*bytesperline = (width * fmt->depth) >> 3;
		sizeimage = height * *bytesperline;
	}

	FNC_EXIT(sizeimage)
	return sizeimage;
}

static inline void pix_format_set_size(struct v4l2_pix_format *f,
				       const struct emxx_cam_fmt *fmt,
				       __u32 width, __u32 height)
{
	FNC_ENTRY;
	f->width = width;
	f->height = height;

	f->sizeimage = get_sizeimage(fmt, width, height, &f->bytesperline);
	FNC_EXIT_N;
}

static inline int check_reset_fmt(const struct emxx_cam_fmt *src,
				  const struct emxx_cam_fmt *dir)
{
	int reset = 0;
	FNC_ENTRY;

	if (src->depth != dir->depth)
		reset = 1;
	if (V4L2_PIX_FMT_RGB24 == src->pixelformat
	    && V4L2_PIX_FMT_RGB24 != dir->pixelformat) {
		reset = 1;
	}
	if (V4L2_PIX_FMT_RGB565 == src->pixelformat
	    && V4L2_PIX_FMT_RGB565 != dir->pixelformat) {
		reset = 1;
	}

	FNC_EXIT(reset)
	return reset;
}

static int emxx_cam_vidioc_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	int ret = 0;
	FNC_ENTRY;

	memset(cap, 0, sizeof(*cap));
	strlcpy(cap->driver, "cam", sizeof(cap->driver));
	strlcpy(cap->card, "emxx", sizeof(cap->card));
	strlcpy(cap->bus_info, "own", sizeof(cap->bus_info));
	cap->version = CAM_VERSION;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	FNC_EXIT(ret)
	return ret;
}

static inline __u32 camif_fmt_boundary(const struct emxx_cam_fmt *fmt, __u32 width)
{
	return (width>>(fmt->boundary))<<(fmt->boundary);
}
static inline __u32 camif_width_ratio_limit(const struct emxx_cam_fmt *fmt, __u32 size)
{
	u32 limit = 64 * size / 1023; 
	size = camif_fmt_boundary(fmt, limit); 
	if (limit >= size)
		size += (1 << fmt->boundary); 
	return size;
}
static inline __u32 camif_height_ratio_limit(const struct emxx_cam_fmt *fmt, __u32 size)
{
	u32 limit = 64 * size / 1023;
	if (64 * size % 1023)
		limit += 1;
	size = limit;
	return size;
}
static inline __u32 camif_width_limit(const struct emxx_cam_fmt *fmt, __u32 width)
{
	u32 limit = 1;

	width = camif_width_ratio_limit(fmt, width); 
	limit = limit << fmt->boundary; 
	if (limit > width)
		width = limit; 
	return width;
}
static inline __u32 camif_height_limit(const struct emxx_cam_fmt *fmt, __u32 height)
{
	u32 limit = 1;
	height = camif_height_ratio_limit(fmt, height);
	if (limit > height)
		height = limit;
	return height;
}

static int emxx_cam_vidioc_enum_input(struct file *file, void *fh, struct v4l2_input *inp)
{
	int ret = 0;

	if(inp->index != 0){
		return -EINVAL;
	}
	memset(inp,0,sizeof(*inp));
	strlcpy(inp->name,sensor_ops.name,sizeof(inp->name));
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	return ret;

}
static int emxx_cam_vidioc_g_input(struct file *file, void *fh, unsigned int *i)
{
	return 0;
}
static int emxx_cam_vidioc_s_input(struct file *file, void *fh, unsigned int i)
{
	return 0;
}
static int emxx_cam_vidioc_queryctrl(struct file *file, void *fh, struct v4l2_queryctrl *a)
{
	return 0;
}
static int emxx_cam_vidioc_querymenu(struct file *file, void *fh, struct v4l2_querymenu *m)
{
	return 0;
}
static int emxx_cam_vidioc_g_ctrl(struct file *file, void *fh, struct v4l2_control *c)
{
	return 0;
}
static int emxx_cam_vidioc_s_ctrl(struct file *file, void *fh, struct v4l2_control *c)
{
	const struct v4l2_queryctrl ctrl;

	if(sensor_ops.vidioc_s_ctrl){
		sensor_ops.vidioc_s_ctrl(&file,c);
	}
	return 0;
}
static int emxx_cam_vidioc_enum_fmt_cap(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	return 0;
}
static int emxx_cam_vidioc_try_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	int ret = 0;
	const struct emxx_cam_fmt *fmt;
	FNC_ENTRY;

	if (f->fmt.pix.field != V4L2_FIELD_ANY &&
	    f->fmt.pix.field != V4L2_FIELD_NONE) {
		FNC_EXIT(-EINVAL)
		return -EINVAL;
	}

	f->fmt.pix.field = V4L2_FIELD_NONE;

	fmt = format_by_pixelformat(f->fmt.pix.pixelformat);
	if (NULL == fmt) {
		FNC_EXIT(-EINVAL)
		return -EINVAL;
	}

	{
		u32 max_width  = current_width;
		u32 max_height = current_height;
		u32 min_width  = camif_width_limit(fmt, max_width);
		u32 min_height = camif_height_limit(fmt, max_height);

		if (min_width > f->fmt.pix.width)
			f->fmt.pix.width = min_width;
		if (max_width < f->fmt.pix.width)
			f->fmt.pix.width = max_width;
		if (min_height > f->fmt.pix.height)
			f->fmt.pix.height = min_height;
		if (max_height < f->fmt.pix.height)
			f->fmt.pix.height = max_height;

		f->fmt.pix.width = camif_fmt_boundary(fmt, f->fmt.pix.width);
	}

	pix_format_set_size(&f->fmt.pix, fmt,
			    f->fmt.pix.width, f->fmt.pix.height);

	f->fmt.pix.colorspace = 0;
	f->fmt.pix.priv = 0;

	FNC_EXIT(ret)
	return ret;
}
static int emxx_cam_vidioc_g_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	return 0;
}
static int emxx_cam_vidioc_s_fmt_cap(struct file *file, void *fh, struct v4l2_format *f) 
{
	int ret = 0;
	const struct emxx_cam_fmt *fmt;
	struct emxx_cam_prepare preset;
	FNC_ENTRY;

/*	ret = emxx_cam_vidioc_try_fmt_cap(file, fh, f);
	if (ret < 0) {
		FNC_EXIT(ret)
		return ret;
	}
*/
	fmt = format_by_pixelformat(f->fmt.pix.pixelformat);
	current_fmt    = *fmt;

aa
	printk("LLLLLLLLLLLLLLLLLLLLLLLL %d,%d",current_width,current_height);
	current_width  = f->fmt.pix.width;
	current_height = f->fmt.pix.height;
	preset.width = preset.c.width = preset.bounds.width = current_width;
	preset.height = preset.c.height = preset.bounds.height = current_height;
	preset.c.left = preset.bounds.left = 0;
	preset.c.top = preset.bounds.top = 0;
	/*printk(KERN_INFO "current_width = %d\n", current_width);
	printk(KERN_INFO "current_height = %d\n", current_height);
	printk(KERN_INFO "preset.width = %d\n", preset.width);
	printk(KERN_INFO "preset.height = %d\n", preset.height);
	printk(KERN_INFO "preset.c.left = %d\n", preset.c.left);
	printk(KERN_INFO "preset.c.top = %d\n", preset.c.top);
	printk(KERN_INFO "preset.c.width = %d\n", preset.c.width);
	printk(KERN_INFO "preset.c.height = %d\n", preset.c.height);
	printk(KERN_INFO "preset.bounds.left = %d\n", preset.bounds.left);
	printk(KERN_INFO "preset.bounds.top = %d\n", preset.bounds.top);
	printk(KERN_INFO "preset.bounds.width = %d\n", preset.bounds.width);
	printk(KERN_INFO "preset.bounds.height = %d\n", preset.bounds.height);*/

	/* module reset */
	outl(0, CA_MODULECONT);
	schedule_timeout_uninterruptible(1);
	/* initial */
	/*if (sensor_ops.prepare) {
		preset.actions = 0;
		ret = sensor_ops.prepare(&preset);
		if (ret) {
			d1b("prepare failed\n");
			return ret;
		}
	}
	else {
		return -EINVAL;
	}*/
	if (sensor_ops.vidioc_s_fmt) {
		ret = sensor_ops.vidioc_s_fmt(&preset, f);
		if (ret) {
			FNC_EXIT(ret)
			return ret;
		}
	}
	emxx_camif_set( &preset );
	/* module unreset */
	outl(1, CA_MODULECONT);
	outl(1, CA_UPDATE);

	FNC_EXIT(ret)
	return ret;
}
static int emxx_cam_vidioc_cropcap(struct file *file, void *fh, struct v4l2_cropcap *c)
{
	return 0;
}
static int emxx_cam_vidioc_g_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	return 0;
}
static int emxx_cam_vidioc_s_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	return 0;
}

#define irq_logv(fmt, args...) do { } while (0);

static irqreturn_t emxx_camif_handler(int irq, void *dev_id)
{
	//liuxu

	//If error
		//Set byteused to 0
irq_logv("IRQ\n");
	__u32 status = inl(CA_STATUS);
	if (B_CAMVS & status) {
		//TODO:Update settings here if necessary
	}
	if(!streaming){
		//We stop the capture and send finished message to emxx_cam_vidioc_stremoff
		//TODO
		goto handler_end;
	}
	
	if (B_MAINTC & status) {
		do_gettimeofday(&buffers[transfering_buf].timestamp);
		buffers[transfering_buf].sequence=current_sequence;
		current_sequence++;
		//__kfifo_put(filled_fifo, (unsigned char*)&transfering_buf, sizeof(int));
		kfifo_in(&filled_fifo, (void *)&transfering_buf,sizeof(int));

		//if there are any buffer in empty fifo
		if(kfifo_len(&empty_fifo)) {
			//Start next DMA transfer
			int available_buf;
//			__kfifo_get(empty_fifo,(unsigned char*)&available_buf,sizeof(int));

			kfifo_out(&empty_fifo, (unsigned char*)&available_buf,sizeof(int));


			emxx_cam_dma_transfer( buffers+available_buf);
			transfering_buf = available_buf;
			//Inform the poll function there are data availiable
			wake_up_interruptible(&proc_list);
		}
		else {
			// (no empty buffer) stop camera streaming??
			//TODO: ToThink
			err("No empty buffer\n");
			emxx_cam_dma_transfer(buffers+transfering_buf);
		}
	}

handler_end:
	//Clear interrupt status
	outl(status, CA_FFCLR);
	return IRQ_HANDLED;
}

static int emxx_cam_vidioc_reqbufs(struct file *file, void *fh,
				    struct v4l2_requestbuffers *req)
{
	//Here application will call this function with
	//	type = V4L2_BUF_TYPE_VIDEO_CAPTURE
	//	memory = V4L2_MEMORY_MMAP
	//	count = 4
	struct emxx_cam_prepare pre;
	int blocksize, count;
	char * vadr;
	__u32 bpl;
	int i;
	FNC_ENTRY;

	printk("AAAA");
	kfifo_reset(&empty_fifo);
	kfifo_reset(&filled_fifo);
	printk("BBBB");
	//TODO: spin_lock
	count = req->count;
	if (sensor_ops.prepare) {
		pre.actions = 0;
		sensor_ops.prepare(&pre);
	}

	printk("CCCC %d,%d\n",current_width,current_height);
	blocksize = get_sizeimage(&current_fmt, current_width, current_height, &bpl);
	blocksize = PAGE_ALIGN(blocksize);
	printk("DDDD %d %d",blocksize,CAMERA_FRAME_SIZE);
	if(blocksize > CAMERA_FRAME_SIZE)
		return -EINVAL;
	vadr = ioremap( CAMERA_FRAME_BASE , blocksize * count);
	for(i=0;i<count;i++)
	{
		buffers[i].index = i;
		buffers[i].padr = CAMERA_FRAME_BASE + i*blocksize;
		buffers[i].vadr = (__u32)vadr + (blocksize * i);
		buffers[i].offset=i*blocksize; 
		buffers[i].blocksize = blocksize;
		do_gettimeofday(&(buffers[i].timestamp));
		buffers[i].sequence = 0;

	}
	
	printk("EEEE");
	return 0;
}

static int emxx_cam_vidioc_querybuf(struct file *file, void *fh,
				     struct v4l2_buffer *b)
{
	//This is called by parameter
	//	index = i
	//	type = V4L2_BUF_TYPE_VIDEO_CAPTURE
	//	memory = V4L2_MEMORY_MMAP
	FNC_ENTRY;
printk("KKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKK\n");
	b->type     = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	b->memory   = V4L2_MEMORY_MMAP;

	//case V4L2_MEMORY_MMAP: We only support MMAP
	b->m.offset  = buffers[b->index].offset; //TODO: add offset member in buffer struct
	b->length    = buffers[b->index].blocksize; //TODO: add length member to buffer struct
	
	printk("EEEEEEEEEEEEEE::::%ld,%d\n",b->m.offset,b->length);
	b->flags = V4L2_BUF_FLAG_MAPPED;
	b->field = V4L2_FIELD_NONE;
	do_gettimeofday(&(b->timestamp));
	b->bytesused = 0;
	b->sequence  = 0;

	FNC_EXIT(0)
	return 0;
}

static int emxx_cam_vidioc_qbuf(struct file *file, void *fh,
				 struct v4l2_buffer *b)
{
	FNC_ENTRY;
	//liuxu: put buffer to fifo
	//__kfifo_put(empty_fifo, (unsigned char*)&(b->index), sizeof(int));


	//__kfifo_put(filled_fifo, (unsigned char*)&transfering_buf, sizeof(int));
	kfifo_in(&empty_fifo, (unsigned char*)&(b->index),sizeof(int));




	FNC_EXIT(0)
	return 0;
}

static int emxx_cam_vidioc_dqbuf(struct file *file, void *fh,
				  struct v4l2_buffer *b)
{
	//liuxu
	//if available buffer in fifo, return it.
	//Otherwise return error
	int buffer_index;
	struct emxx_cam_buffer * buf;
	static int bpl;
	FNC_ENTRY;
	printk("11111\n");
	if(kfifo_len(&filled_fifo)==0)
	{
		printk("%s: dqbuf: no active.Wating...\n", CAM_NAME);
		wait_event_interruptible(proc_list, kfifo_len(&filled_fifo)!=0);
	}

//			__kfifo_get(empty_fifo,(unsigned char*)&available_buf,sizeof(int));

	printk("22222\n");
	kfifo_out(&filled_fifo, (unsigned char*)&buffer_index,sizeof(int));


	
	memset(b, 0, sizeof(*b));

	buf=buffers+buffer_index;
	b->index    = buffer_index;
	b->type     = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	b->memory   = V4L2_MEMORY_MMAP;
	b->m.offset = buf->offset;
	b->length   = buf->blocksize;
	b->flags    = V4L2_BUF_FLAG_MAPPED | V4L2_BUF_FLAG_DONE;
	b->field = V4L2_FIELD_NONE;
	b->timestamp = buf->timestamp;
	b->bytesused = get_sizeimage( &current_fmt,
			current_width, current_height, &bpl);
	b->sequence  = buf->sequence;

	//TODO: support other formats
	b->m.phys_add.PhysAddr_Y = buf->padr;
	b->m.phys_add.PhysAddr_UV = b->m.phys_add.PhysAddr_Y +
		(current_width * current_height);
	b->m.phys_add.PhysAddr_V = b->m.phys_add.PhysAddr_UV +
		(current_width * current_height / 4);
	FNC_EXIT(0)
	return 0;
}

//liuxu: new function
static int emxx_cam_vidioc_streamon(struct file *file, void *fh,
				     enum v4l2_buf_type i)
{
	__u32 buffer_index;
	int ret=0;
	FNC_ENTRY;
	//TODO: Start 1st fram capture here
	if(sensor_ops.stream_on){
		ret = sensor_ops.stream_on(0);
		if (ret) {
			d1b("stop\n");
		}
	}
	streaming = 1;
	//TODO: get buffer from empty fifo and start_dma on it;
	//Set framebuffer
	//__kfifo_get(empty_fifo, (unsigned char*)&buffer_index, sizeof(int));

	
	kfifo_out(&empty_fifo,  (unsigned char*)&buffer_index,sizeof(int));




	//TODO: hw.trigger() operation should be called after setting image size, but not here

	transfering_buf = buffer_index;

	//Enable interrupt
	outl(M_CA_ENSET, CA_ENSET);

	//Start DMA transfer
	emxx_cam_dma_transfer(buffers+buffer_index);

	FNC_EXIT(0);
	return 0;
}

static int emxx_cam_dma_transfer(struct emxx_cam_buffer *buffer)
{
	int ret = 0;
	__u32 yplane_a=0, uvplane_a=0, vplane_a=0;
	int width=current_width;
	int height=current_height;

	FNC_ENTRY;
	yplane_a = buffer->padr;
	switch(current_fmt.pixelformat) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV422:
		uvplane_a = yplane_a + (width * height);
		vplane_a = 0;
		break;
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		uvplane_a = 0;
		vplane_a = 0;
		break;
	case V4L2_PIX_FMT_YUV422P:
		uvplane_a = yplane_a + (width * height);
		vplane_a = uvplane_a + (width * height/2);
		break;
	case V4L2_PIX_FMT_YUV420:
		uvplane_a = yplane_a + (width * height);
		vplane_a = uvplane_a + (width * height / 4);
		break;
	case V4L2_PIX_FMT_YVU420:
		vplane_a = yplane_a + (width * height);
		uvplane_a = vplane_a + (width * height / 4);
		break;
	default:
		ret = -EINVAL;
	}

	//Set DMA address
	outl(yplane_a, CA_YPLANE_A);
	outl(uvplane_a, CA_UVPLANE_A);
	outl(vplane_a, CA_VPLANE_A);

	//Start DMA tranfer
	outl(0x01, CA_DMAREQ);

	FNC_EXIT_N;
	return ret;
}

static int emxx_cam_vidioc_streamoff(struct file *file, void *fh,
				      enum v4l2_buf_type i)
{
	FNC_ENTRY;
	streaming = 0;
	//Wait for the DMA finish
	//TODO:This should be done by event, not by sleep
	schedule_timeout_uninterruptible(30);
	//Disable interrupt
	outl( M_CA_ENSET , CA_ENCLR);
	//add transfering_buf to filled queue
	//__kfifo_put(filled_fifo, (unsigned char*)&transfering_buf, sizeof(int));
	kfifo_in(&filled_fifo,  (unsigned char*)&transfering_buf,sizeof(int));

	
	//TODO: reset camera IF hardware
	if(sensor_ops.stream_off){
		sensor_ops.stream_off(0);
	}
	//Reset all Queues
	//kfifo_reset(empty_fifo);
	//kfifo_reset(filled_fifo);
	return 0;
}

static unsigned int emxx_cam_poll(struct file *file, poll_table *wait)
{
	FNC_ENTRY;
	//Check if there is any buffer available
	if(kfifo_len(&filled_fifo)){
		FNC_EXIT_N;
		return POLLIN | POLLRDNORM;
	}
	//TODO: wait for DMA completion
	//poll_wait(file, &proc_list, wait);
	wait_event_interruptible(proc_list, kfifo_len(&filled_fifo)!=0);
	//TODO: Check status and return POLLERR;
	FNC_EXIT_N;
	return POLLIN | POLLRDNORM;
}

static int emxx_cam_mmap(struct file *file, struct vm_area_struct *vma)
{
	FNC_ENTRY;
	vma->vm_pgoff = vma->vm_pgoff + (CAMERA_FRAME_BASE >> PAGE_SHIFT);

	if (emxx_v4l2_mmap(vma)) {
		return -EAGAIN;
	}
	//TODO: maybe we need to remember some information
	FNC_EXIT_N;
	return 0;
}

static int emxx_cam_thread(void *p)
{
	do{
		msleep(3000);

		d1b("  CA_STATUS        is 0x%08x\n", inl(CA_STATUS));
		d1b("  CA_RAWSTATUS     is 0x%08x\n", inl(CA_RAWSTATUS));
		d1b("  CA_ENSET         is 0x%08x\n", inl(CA_ENSET));
		d1b("  CA_ENCLR         is 0x%08x\n", inl(CA_ENCLR));
		d1b("  CA_FFCLR         is 0x%08x\n", inl(CA_FFCLR));
		d1b("  CA_ERRORADR      is 0x%08x\n", inl(CA_ERRORADR));
		d1b("  CA_CSR           is 0x%08x\n", inl(CA_CSR));
		d1b("  CA_X1R           is 0x%08x\n", inl(CA_X1R));
		d1b("  CA_X2R           is 0x%08x\n", inl(CA_X2R));
		d1b("  CA_Y1R           is 0x%08x\n", inl(CA_Y1R));
		d1b("  CA_Y2R           is 0x%08x\n", inl(CA_Y2R));
		d1b("  CA_BNZR          is 0x%08x\n", inl(CA_BNZR));
		d1b("  CA_BNGR          is 0x%08x\n", inl(CA_BNGR));
		d1b("  CA_CBZR          is 0x%08x\n", inl(CA_CBZR));
		d1b("  CA_CBGR          is 0x%08x\n", inl(CA_CBGR));
		d1b("  CA_CRZR          is 0x%08x\n", inl(CA_CRZR));
		d1b("  CA_CRGR          is 0x%08x\n", inl(CA_CRGR));
		d1b("  CA_DMACNT        is 0x%08x\n", inl(CA_DMACNT));
		d1b("  CA_FRAME         is 0x%08x\n", inl(CA_FRAME));
		d1b("  CA_DMAREQ        is 0x%08x\n", inl(CA_DMAREQ));
		d1b("  CA_DMASTOP       is 0x%08x\n", inl(CA_DMASTOP));
		d1b("  CA_LINESIZE_MAIN is 0x%08x\n", inl(CA_LINESIZE_MAIN));
		d1b("  CA_XRATIO_MAIN   is 0x%08x\n", inl(CA_XRATIO_MAIN));
		d1b("  CA_YRATIO_MAIN   is 0x%08x\n", inl(CA_YRATIO_MAIN));
		d1b("  CA_DMAX_MAIN     is 0x%08x\n", inl(CA_DMAX_MAIN));
		d1b("  CA_DMAY_MAIN     is 0x%08x\n", inl(CA_DMAY_MAIN));
		d1b("  CA_YPLANE_A      is 0x%08x\n", inl(CA_YPLANE_A));
		d1b("  CA_UVPLANE_A     is 0x%08x\n", inl(CA_UVPLANE_A));
		d1b("  CA_VPLANE_A      is 0x%08x\n", inl(CA_VPLANE_A));
		d1b("  CA_YPLANE_B      is 0x%08x\n", inl(CA_YPLANE_B));
		d1b("  CA_UVPLANE_B     is 0x%08x\n", inl(CA_UVPLANE_B));
		d1b("  CA_VPLANE_B      is 0x%08x\n", inl(CA_VPLANE_B));
		d1b("  CA_MODULECONT    is 0x%08x\n", inl(CA_MODULECONT));
		d1b("  CA_UPDATE        is 0x%08x\n", inl(CA_UPDATE));
		d1b("  CA_MIRROR        is 0x%08x\n", inl(CA_MIRROR));
		d1b("  CA_OD_BYTELANE   is 0x%08x\n", inl(CA_OD_BYTELANE));
		d1b("  CA_X3R           is 0x%08x\n", inl(CA_X3R));
		d1b("  CA_OD_BYTELANE2  is 0x%08x\n", inl(CA_OD_BYTELANE2));
		d1b("  CA_QOS           is 0x%08x\n", inl(CA_QOS));
	} while(0);
	return 0;
}
//New open function by liuxu
static int emxx_cam_open(struct file *file)
{
	struct emxx_cam_prepare pre;
	int ret;
	int align;
	FNC_ENTRY;

	/* start clock */
	emxx_clkctrl_off(EMXX_CLKCTRL_CAMPCLK);
	emxx_open_clockgate(EMXX_CLK_CAM | EMXX_CLK_CAM_P|EMXX_CLK_CAM_S);

	// 24Mhz from OSC1
	writel(0x00000300, SMU_CAMSCLKDIV);

	/* unreset CAM module*/
	camif_power_on(CAM_POWERDOWN);

	emxx_unreset_device(EMXX_RST_CAM);
	emxx_unreset_device(EMXX_RST_CAM_SAFE);

	/* change pin select 
	 * GIO_P131 : CAM_CLKO
	 * GIO_P132 : CAM_CLKI
	 * GIO_P133 : CAM_VS
	 * GIO_P134 : CAM_HS
	 * GIO_P135~142 : CAM_YUV0~YUV7 */

	/* GIO_P131~P142 switch to extend function pin(not GPIO)*/
	writel((readl(CHG_PINSEL_G128) & ~0x00007FF8), CHG_PINSEL_G128);
	/* chose mode0->CAM function */
	writel(0x00000000, CHG_PINSEL_CAM);

	/*input enable, PU/PD setting*/
	/*all input enable and PU/PD disable, except for CLKO*/
	writel((readl(CHG_PULL18) & ~0xffff0000) | 0x44400000, CHG_PULL18);
	writel(0x44444444, CHG_PULL19);

	ret = request_irq(INT_CAM, emxx_camif_handler,
			  IRQF_DISABLED, "CAMIF", (void *)buffers);
	if(ret < 0){
		printk("camera request irq fail.....\n");
		goto cam_open_failed;
	}

	wake_lock(&idle_lock); /* lock suspend */

	/*sensor hw startup*/
	if(sensor_ops.startup){
		ret = sensor_ops.startup(0);
		if (ret) {
			goto cam_open_failed;
		}
	}
/*	// will do these in emxx_cam_vidioc_s_fmt_cap
	if (sensor_ops.prepare) {
		pre.actions = 0;
		ret = sensor_ops.prepare(&pre);
		if (ret) {
			d1b("prepare failed\n");
			goto cam_open_hw_error;
		}
	}

	if (sensor_ops.sync) {
		pre.actions = 0;
		ret = sensor_ops.sync(&pre);
		if (ret) {
			goto cam_open_hw_error;
		}
	}

	current_width = pre.bounds.width;
	align = current_fmt.boundary;
	current_width >>= align;
	current_width <<= align;
	current_height = pre.bounds.height;

	//Camera IF reset
	outl(0, CA_MODULECONT);
	schedule_timeout_uninterruptible(1);
	ret = emxx_camif_set( &pre );
	if(ret){
		goto cam_open_failed;
	}
	outl(1, CA_MODULECONT);
	outl(1, CA_UPDATE);
*/
	cam_opened=1;

	//DEBUG
	//kthread_run(emxx_cam_thread, buffers, "emxx_cam_thread");
	FNC_EXIT_N;
	return ret;

cam_open_hw_error:
	if (sensor_ops.shutdown)
		sensor_ops.shutdown(0);
	wake_unlock(&idle_lock);
cam_open_failed:
	FNC_EXIT_N;
	return -EINVAL;
}

static const char * dump_v4l2_formt(int fmt)
{
	static char numbers[256];
#if 0
	static int fmt_values[]={
		V4L2_PIX_FMT_RGB332  , V4L2_PIX_FMT_RGB444  , V4L2_PIX_FMT_RGB555  , V4L2_PIX_FMT_RGB565  ,
		V4L2_PIX_FMT_RGB555X , V4L2_PIX_FMT_RGB565X , V4L2_PIX_FMT_BGR24   , V4L2_PIX_FMT_RGB24   ,
		V4L2_PIX_FMT_BGR32   , V4L2_PIX_FMT_RGB32   , V4L2_PIX_FMT_GREY    , V4L2_PIX_FMT_Y16     ,
		V4L2_PIX_FMT_PAL8    , V4L2_PIX_FMT_YVU410  , V4L2_PIX_FMT_YVU420  , V4L2_PIX_FMT_YUYV    ,
		V4L2_PIX_FMT_YYUV    , V4L2_PIX_FMT_YVYU    , V4L2_PIX_FMT_UYVY    , V4L2_PIX_FMT_VYUY    ,
		V4L2_PIX_FMT_YUV422P , V4L2_PIX_FMT_YUV411P , V4L2_PIX_FMT_Y41P    , V4L2_PIX_FMT_YUV444  ,
		V4L2_PIX_FMT_YUV555  , V4L2_PIX_FMT_YUV565  , V4L2_PIX_FMT_YUV32   , V4L2_PIX_FMT_NV12    ,
		V4L2_PIX_FMT_NV21    , V4L2_PIX_FMT_NV16    , V4L2_PIX_FMT_NV61    , V4L2_PIX_FMT_NV422   ,
	};

	static char * fmt_strings[]={
		"V4L2_PIX_FMT_RGB332  ", "V4L2_PIX_FMT_RGB444  ", "V4L2_PIX_FMT_RGB555  ", "V4L2_PIX_FMT_RGB565  ",
		"V4L2_PIX_FMT_RGB555X ", "V4L2_PIX_FMT_RGB565X ", "V4L2_PIX_FMT_BGR24   ", "V4L2_PIX_FMT_RGB24   ",
		"V4L2_PIX_FMT_BGR32   ", "V4L2_PIX_FMT_RGB32   ", "V4L2_PIX_FMT_GREY    ", "V4L2_PIX_FMT_Y16     ",
		"V4L2_PIX_FMT_PAL8    ", "V4L2_PIX_FMT_YVU410  ", "V4L2_PIX_FMT_YVU420  ", "V4L2_PIX_FMT_YUYV    ",
		"V4L2_PIX_FMT_YYUV    ", "V4L2_PIX_FMT_YVYU    ", "V4L2_PIX_FMT_UYVY    ", "V4L2_PIX_FMT_VYUY    ",
		"V4L2_PIX_FMT_YUV422P ", "V4L2_PIX_FMT_YUV411P ", "V4L2_PIX_FMT_Y41P    ", "V4L2_PIX_FMT_YUV444  ",
		"V4L2_PIX_FMT_YUV555  ", "V4L2_PIX_FMT_YUV565  ", "V4L2_PIX_FMT_YUV32   ", "V4L2_PIX_FMT_NV12    ",
		"V4L2_PIX_FMT_NV21    ", "V4L2_PIX_FMT_NV16    ", "V4L2_PIX_FMT_NV61    ", "V4L2_PIX_FMT_NV422   ",
	};
#endif

	static int fmt_values[]={
		V4L2_PIX_FMT_YUYV	, V4L2_PIX_FMT_UYVY	, V4L2_PIX_FMT_YUV422P	, V4L2_PIX_FMT_YUV420	,
		V4L2_PIX_FMT_YVU420	, V4L2_PIX_FMT_NV12	, V4L2_PIX_FMT_NV21	, V4L2_PIX_FMT_NV422	,
	};

	static char * fmt_strings[]={
		"V4L2_PIX_FMT_YUYV	", "V4L2_PIX_FMT_UYVY	", "V4L2_PIX_FMT_YUV422P	", "V4L2_PIX_FMT_YUV420	",
		"V4L2_PIX_FMT_YVU420	", "V4L2_PIX_FMT_NV12	", "V4L2_PIX_FMT_NV21	", "V4L2_PIX_FMT_NV422	",
	};

	int i;

	d1b("--V4L2_PIX_FMT_YUYV	= 0x%08x\n", V4L2_PIX_FMT_YUYV	 );
	d1b("--V4L2_PIX_FMT_UYVY	= 0x%08x\n", V4L2_PIX_FMT_UYVY	 );
	d1b("--V4L2_PIX_FMT_YUV422P	= 0x%08x\n", V4L2_PIX_FMT_YUV422P);	
	d1b("--V4L2_PIX_FMT_YUV420	= 0x%08x\n", V4L2_PIX_FMT_YUV420 );       
	d1b("--V4L2_PIX_FMT_YVU420	= 0x%08x\n", V4L2_PIX_FMT_YVU420 );       
	d1b("--V4L2_PIX_FMT_NV12	= 0x%08x\n", V4L2_PIX_FMT_NV12	 );
	d1b("--V4L2_PIX_FMT_NV21	= 0x%08x\n", V4L2_PIX_FMT_NV21	 );
	d1b("--V4L2_PIX_FMT_NV422	= 0x%08x\n", V4L2_PIX_FMT_NV422	 );

	for(i=0;i<sizeof(fmt_values)/sizeof(int);i++)
	{
		if(fmt == fmt_values[i])
			return fmt_strings[i];
	}
	sprintf(numbers, "0x%08x", fmt);
	return numbers;
}

static int emxx_camif_set(struct emxx_cam_prepare * pre)
{
	int cpe;
	int ret=0;
	FNC_ENTRY;

	__u32 csr = 0;
	__u32 x1r, x2r, x3r, y1r, y2r;
	__u32 dmacnt;
	__u32 od_bytelane, od_bytelane2 = 0xe4e4;
	__u32 frame;
	__u32 dmax_main, dmay_main;
	__u32 linesize_main;
	__u32 xratio_main, yratio_main;


	pre->c = pre->bounds;
#if 0
	{//DEBUG
		d1b("clk_edge 		= %d\n", pre->clk_edge);
		d1b("synctype 		= %d\n", pre->synctype);
		d1b("data_id 		= %d\n", pre->data_id);
		d1b("vs_det  		= %d\n", pre->vs_det  );
		d1b("hs_det  		= %d\n", pre->hs_det  );
		d1b("clk_edge		= %d\n", pre->clk_edge);
		d1b("data_det		= %d\n", pre->data_det);
		d1b("vs_pol  		= %d\n", pre->vs_pol  );
		d1b("hs_pol  		= %d\n", pre->hs_pol  );

		d1b("c.left  		- %d\n", pre->c.left  		);
		d1b("c.width		- %d\n", pre->c.width		);	
		d1b("bounds.left	- %d\n", pre->bounds.left	);	
		d1b("bounds.width	- %d\n", pre->bounds.width	);	
		d1b("c.top		- %d\n", pre->c.top		);	
		d1b("c.height		- %d\n", pre->c.height		);

		d1b("current_width		= %d\n", current_width	       	);
		d1b("current_height		= %d\n", current_height	       	);
		d1b("current_fmt.pixelformat	= %s\n", dump_v4l2_formt(current_fmt.pixelformat));
	}
#endif

	cpe = ((pre->clk_edge) ? 1 : 2);

	/* input setting */
	csr |= (pre->syncmode << S_SYNCMODE);
	csr |= (pre->synctype << S_SYNCTYPE);
	csr |= (pre->data_id << S_DATA_ID);
	csr |= (pre->vs_det << S_VS_DET);
	csr |= (pre->hs_det << S_HS_DET);
	csr |= (pre->clk_edge << S_CLK_EDGE);
	csr |= (pre->data_det << S_DATA_DET);
	csr |= (pre->vs_pol << S_VS_POL);
	csr |= (pre->hs_pol << S_HS_POL);
	csr |= (0x00 << S_LIMITSEL); /* ITU-R BT656(601) */

	/* input setting : range */
	x1r =  pre->c.left * cpe;
	x2r = (pre->c.left + pre->c.width) * cpe;
	x3r = (pre->bounds.left + pre->bounds.width) * cpe;
	y1r =  pre->c.top;
	y2r =  pre->c.top + pre->c.height;

	dmacnt = (0x01 << S_MNRESIZE); /* resize */
	od_bytelane = 0xe4;

	/* output setting : address */
	frame = (0x01 << S_MAINFRM);

	/* output setting : size */
	dmax_main = current_width;
	dmay_main = current_height;

	switch (current_fmt.pixelformat) {
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_UYVY:
			/*case V4L2_PIX_FMT_RGB565:
			  case V4L2_PIX_FMT_RGB24:*/
			linesize_main = current_width * 2;
			break;
		default:
			linesize_main = current_width;
	}

	xratio_main = ((pre->c.width - current_width) * 64)
		/ current_width;

	yratio_main = ((pre->c.height - current_height) * 64)
		/ current_height;


	switch (current_fmt.pixelformat) {
	case V4L2_PIX_FMT_YUYV:
		csr |= (0x01 << S_PIXELMODE);
		od_bytelane  = 0xd8;
		od_bytelane2 = 0xe4e4;
		break;
	case V4L2_PIX_FMT_UYVY:
		csr |= (0x01 << S_PIXELMODE);
		od_bytelane  = 0x72;
		od_bytelane2 = 0xe4e4;
		break;
	case V4L2_PIX_FMT_YUV422P:
		csr |= (0x01 << S_PIXEL_YUV);
		od_bytelane2 = 0xe4e4;
		break;
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		csr |= (0x01 << S_PIXEL_YUV);
		/* add for tp_value */
		csr |= (0x01 << S_DATA_OD);
		dmacnt |= (0x01 << S_MAINYUV);
		od_bytelane2 = 0xe4e4;
		break;
	case V4L2_PIX_FMT_NV12:
		csr |= (0x01 << S_DATA_OD);
		dmacnt |= (0x01 << S_MAINYUV);
		od_bytelane2 = 0xe4e4;
		break;
	case V4L2_PIX_FMT_NV21:
		csr |= (0x01 << S_DATA_OD);
		dmacnt |= (0x01 << S_MAINYUV);
		od_bytelane2 = 0xe4b1;
		break;
	case V4L2_PIX_FMT_NV422:
		csr |= (0x01 << S_DATA_OD);
		od_bytelane2 = 0xe4e4;
		break;
	default:
		ret = -EINVAL;
	}

	outl(csr, CA_CSR);
	outl(x1r, CA_X1R);
	outl(x2r, CA_X2R);
	outl(x3r, CA_X3R);
	outl(y1r, CA_Y1R);
	outl(y2r, CA_Y2R);
	outl(dmacnt, CA_DMACNT);
	outl(od_bytelane, CA_OD_BYTELANE);
	outl(od_bytelane2, CA_OD_BYTELANE2);
	outl(frame, CA_FRAME);
	outl(dmax_main, CA_DMAX_MAIN);
	outl(dmay_main, CA_DMAY_MAIN);
	outl(linesize_main, CA_LINESIZE_MAIN);
	outl(xratio_main, CA_XRATIO_MAIN);
	outl(yratio_main, CA_YRATIO_MAIN);
	
	FNC_EXIT_N;
	return ret;
}

static int emxx_cam_close(struct file *file)
{
	FNC_ENTRY;
	//TODO
	cam_opened=0;
	free_irq(INT_CAM,(void *)buffers);
	wake_unlock(&idle_lock);
	if(sensor_ops.shutdown){
		sensor_ops.shutdown(0);
	}
	FNC_EXIT_N;
	return 0;
}

/*
 * register V4L2 device
 */

static int video_nr = -1;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 29))
static const struct file_operations emxx_cam_fops = {
	.llseek         = no_llseek,
#else
static const struct v4l2_file_operations emxx_cam_fops = {
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 29)) */
	.owner          = THIS_MODULE,
	.open           = emxx_cam_open,
	.release        = emxx_cam_close,
	.poll           = emxx_cam_poll,
	.ioctl          = video_ioctl2,
#if EMXX_CAM_USE_MMAP /* need support V4L2_MEMORY_MMAP */
	.mmap           = emxx_cam_mmap,
#endif
};

static const struct v4l2_ioctl_ops emxx_cam_ioctl_ops = {
	.vidioc_querycap        = emxx_cam_vidioc_querycap,
	.vidioc_enum_input      = emxx_cam_vidioc_enum_input,
	.vidioc_g_input         = emxx_cam_vidioc_g_input,
	.vidioc_s_input         = emxx_cam_vidioc_s_input,
	.vidioc_queryctrl       = emxx_cam_vidioc_queryctrl,
	.vidioc_querymenu       = emxx_cam_vidioc_querymenu,
	.vidioc_g_ctrl          = emxx_cam_vidioc_g_ctrl,
	.vidioc_s_ctrl          = emxx_cam_vidioc_s_ctrl,
	.vidioc_enum_fmt_vid_cap        = emxx_cam_vidioc_enum_fmt_cap,
	.vidioc_g_fmt_vid_cap   = emxx_cam_vidioc_g_fmt_cap,
	.vidioc_s_fmt_vid_cap   = emxx_cam_vidioc_s_fmt_cap,
	.vidioc_try_fmt_vid_cap         = emxx_cam_vidioc_try_fmt_cap,
	.vidioc_cropcap         = emxx_cam_vidioc_cropcap,
	.vidioc_g_crop          = emxx_cam_vidioc_g_crop,
	.vidioc_s_crop          = emxx_cam_vidioc_s_crop,
	.vidioc_reqbufs         = emxx_cam_vidioc_reqbufs,
	.vidioc_querybuf        = emxx_cam_vidioc_querybuf,
	.vidioc_qbuf            = emxx_cam_vidioc_qbuf,
	.vidioc_dqbuf           = emxx_cam_vidioc_dqbuf,
	.vidioc_streamon        = emxx_cam_vidioc_streamon,
	.vidioc_streamoff       = emxx_cam_vidioc_streamoff,
};

static struct video_device emxx_cam_template = {
	.ioctl_ops              = &emxx_cam_ioctl_ops,
	.fops                   = &emxx_cam_fops,
	.name                   = CAM_NAME,
	.minor                  = -1,
	.release                = video_device_release,
};


static int emxx_cam_unregister(void)
{
	//TODO
	return 0;
}

/*
 * Driver init and exit Functions
 */

#ifdef CONFIG_PM
static int emxx_cam_suspend(struct platform_device *dev, pm_message_t state)
{
	int ret = 0;
	FNC_ENTRY;
#define DEV_SUSPEND_IDLE_1 0x1000
	switch (state.event) {
	case DEV_SUSPEND_IDLE_1:
	case PM_EVENT_SUSPEND:
		if (cam_opened)
			ret = -EBUSY;
		break;
	default:
		break;
	}

	FNC_EXIT(ret)
	return ret;
}
static int emxx_cam_resume(struct platform_device *dev)
{
	FNC_ENTRY;
	FNC_EXIT(0)
	return 0;
}
#endif

//Added by liuxu
static int emxx_cam_probe(struct platform_device *devptr)
{
	int ret = 0;
	struct emxx_cam_fmt * fmt;
	static spinlock_t empty_fifo_lock;
	static spinlock_t filled_fifo_lock;
	FNC_ENTRY;

	platform_set_drvdata(devptr, buffers);
	//Alloc empty and filled fifo
	kfifo_alloc(&empty_fifo, sizeof(int)* EMXX_CAM_MAX_BUFNBRS, GFP_KERNEL);
	kfifo_alloc(&filled_fifo, sizeof(int)* EMXX_CAM_MAX_BUFNBRS, GFP_KERNEL);

/*
	empty_fifo = kfifo_alloc(sizeof(int)* EMXX_CAM_MAX_BUFNBRS,
				    GFP_KERNEL, &empty_fifo_lock);
	filled_fifo = kfifo_alloc(sizeof(int)* EMXX_CAM_MAX_BUFNBRS,
				    GFP_KERNEL, &filled_fifo_lock);
*/
	init_waitqueue_head(&proc_list);

	/*allocate memory space for the video device member in 'cam'*/
	vdev = video_device_alloc();

	if (NULL == vdev) {
		err("%s: video_device_alloc() failed!\n", CAM_NAME);
		goto out_kfifo_free_grab;
	}

	/*copy the inforamtion to em_cam->vdev,construct this member structure
	 why not setting in directly? */
	memcpy(vdev, &emxx_cam_template, sizeof(emxx_cam_template));
	/* connect cam with the cam->video_dev->driver_data, *
	 * make the driver and device as one gather          */
	//video_set_drvdata(em_cam->vdev, em_cam);

	/* register v4l device */
	ret = video_register_device(vdev, VFL_TYPE_GRABBER, video_nr);

	if (ret) {
		err("%s: video_register_device() failed!\n", CAM_NAME);
		goto out_video_device_release;
	}

	/* initialize the spinlock */
	spin_lock_init(&cam_lock);

#ifdef CONFIG_EMXX_ANDROID
	/* Android power management wake lock init */
	/* suspend control*/
	wake_lock_init(&idle_lock, WAKE_LOCK_IDLE, CAM_NAME);
#endif /* CONFIG_EMXX_ANDROID */

	/*register cam hw*/
	ret = emxx_cam_hw_register(&sensor_ops);
	if (ret)
	{
		goto cam_probe_error;
	}

	//Set default image format here
	fmt = format_by_pixelformat(V4L2_PIX_FMT_YUV420);
	current_fmt = *fmt;

	return 0;
out_kfifo_free_grab:
	//TODO
out_video_device_release:
	//TODO
cam_probe_error:
	return -EINVAL;
}

static int emxx_cam_remove(struct platform_device *devptr)
{
	int ret = 0;
	FNC_ENTRY;

	sensor_ops.unregister(0);
	//TODO:emxx_camif_unregister();
	emxx_cam_unregister();
	platform_set_drvdata(devptr, NULL);

	FNC_EXIT(ret)
	return ret;
}

static struct platform_device *emxx_cam_device;
static struct platform_driver emxx_cam_driver = {
	.probe          = emxx_cam_probe,
	.remove         = __devexit_p(emxx_cam_remove),
#ifdef CONFIG_PM
	.suspend        = emxx_cam_suspend,
	.resume         = emxx_cam_resume,
#endif
	.driver.name    = CAM_NAME,
};

static int __init emxx_cam_init(void)
{
	int ret = 0;
	FNC_ENTRY;

#if 1 /* XXX */
	warming_up = 1;
#endif
	ret = platform_driver_register(&emxx_cam_driver);

	if (0 > ret) {
		err("%s: platform_driver_register() failed!\n", CAM_NAME);
		FNC_EXIT(ret)
		return ret;
	}

	emxx_cam_device =
		platform_device_register_simple(CAM_NAME, -1, NULL, 0);

	if (!IS_ERR(emxx_cam_device)) {
		if (platform_get_drvdata(emxx_cam_device)) {
			FNC_EXIT(ret)
			return 0;
		}
		err("%s: platform_get_drvdata() failed!\n", CAM_NAME);
		platform_device_unregister(emxx_cam_device);
		ret = -ENODEV;
	} else {
		err("%s: platform_device_register_simple() failed!\n",
		    CAM_NAME);
		ret = PTR_ERR(emxx_cam_device);
	}

	platform_driver_unregister(&emxx_cam_driver);

	FNC_EXIT(ret)
	return ret;
}

static void __exit emxx_cam_exit(void)
{
	FNC_ENTRY;

	platform_device_unregister(emxx_cam_device);
	platform_driver_unregister(&emxx_cam_driver);

	FNC_EXIT_N;
	return;
}

module_init(emxx_cam_init);
module_exit(emxx_cam_exit);


module_param(video_nr, int, 0);
MODULE_PARM_DESC(video_nr, "video device to register (0=/dev/video1, etc)");

MODULE_DESCRIPTION("Mega Camera driver for emxx chip");
MODULE_SUPPORTED_DEVICE("video");
MODULE_LICENSE("GPL");

