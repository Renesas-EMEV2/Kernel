///*****************************************
//  Copyright (C) 2009-2014
//  ITE Tech. Inc. All Rights Reserved
//  Proprietary and Confidential
///*****************************************
//   @file   <mcu.h>
//   @author Jau-Chih.Tseng@ite.com.tw
//   @date   2010/12/30
//   @fileversion: CAT6611_SAMPLE_1.15
//******************************************/

#ifndef _MCU_H_
#define _MCU_H_

#include "typedef.h"
///////////////////////////////////////////////////////////////////////////////
// Type Definition
///////////////////////////////////////////////////////////////////////////////
//typedef bit BOOL, bool ;
//typedef unsigned char BYTE, byte;
typedef unsigned int UINT;
///////////////////////////////////////////////////////////////////////////////
// Constant Definition
///////////////////////////////////////////////////////////////////////////////

#define RXADR			0x90

#define EDID_ADR		0xA0	// alex 070321

#define DELAY_TIME		1		// unit = 1 us;
#define IDLE_TIME		100		// unit = 1 ms;

#define HIGH			1
#define LOW				0

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//void SelectHDMIDev(BYTE dev) ;
BYTE HDMITX_ReadI2C_Byte(BYTE RegAddr);
SYS_STATUS HDMITX_WriteI2C_Byte(BYTE RegAddr, BYTE d);
SYS_STATUS HDMITX_ReadI2C_ByteN(BYTE RegAddr, BYTE *pData, int N);
SYS_STATUS HDMITX_WriteI2C_ByteN(SHORT RegAddr, BYTE *pData, int N);

void DelayMS(USHORT ms) ;

#endif	// _MCU_H_
