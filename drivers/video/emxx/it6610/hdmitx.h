///*****************************************
//  Copyright (C) 2009-2014
//  ITE Tech. Inc. All Rights Reserved
//  Proprietary and Confidential
///*****************************************
//   @file   <hdmitx.h>
//   @author Jau-Chih.Tseng@ite.com.tw
//   @date   2010/12/30
//   @fileversion: CAT6611_SAMPLE_1.15
//******************************************/

#ifndef _HDMITX_H_
#define _HDMITX_H_

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>

#include <stdarg.h>
#include "config.h"
#include "mcu.h"
#include "typedef.h"
#include "cat6611_sys.h"
#include "cat6611_drv.h"
#include "edid.h"



//////////////////////////////////////////////////////////////////////
// Function Prototype
//////////////////////////////////////////////////////////////////////

//#define DEBUG 1

// dump
#ifdef DEBUG
#define ErrorF(fmt, args...)	printk("HDMI%s:" fmt, __func__, ## args)
void DumpCat6611Reg() ;
#else
#define ErrorF(fmt, args...)	do {} while(0)
#endif
// I2C

//////////////////////////////////////////////////////////////////////////////
// main.c
//////////////////////////////////////////////////////////////////////////////

#endif // _HDMITX_H_

