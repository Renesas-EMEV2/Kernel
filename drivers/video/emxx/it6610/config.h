///*****************************************
//  Copyright (C) 2009-2014
//  ITE Tech. Inc. All Rights Reserved
//  Proprietary and Confidential
///*****************************************
//   @file   <config.h>
//   @author Jau-Chih.Tseng@ite.com.tw
//   @date   2010/12/30
//   @fileversion: CAT6611_SAMPLE_1.15
//******************************************/
#ifndef _CONFIG_H_
#define _CONFIG_H_

#pragma message("included <config.h>")
// #define REFERENCE_RXSENSE
// #define SUPPORT_DEGEN
// #define SUPPORT_SYNCEMB

#define SUPPORT_EDID
#define SUPPORT_HDCP
#define SUPPORT_INPUTRGB
#define SUPPORT_INPUTYUV444
#define SUPPORT_INPUTYUV422

#if defined(SUPPORT_INPUTYUV444) || defined(SUPPORT_INPUTYUV422)
#define SUPPORT_INPUTYUV
#endif

// #define ENABLE_EXTERNAL_MCLK_SAMPLING_SPDIFAUDIO
// #define USE_MCLK_128FS
// #define USE_MCLK_512FS
// #define USE_MCLK_1024FS
#define USE_MCLK_256FS

#ifdef Debug_Message
    #define HDMITX_DEBUG_PRINTF(x)
    #define HDMITX_DEBUG_PRINTF1(x)
    #define HDMITX_DEBUG_PRINTF2(x)
    #define HDMITX_DEBUG_PRINTF3(x)
    #define EDID_DEBUG_PRINTF(x)
    #define EDID_DEBUG_PRINTF1(x)
    #define EDID_DEBUG_PRINTF2(x)
    #define EDID_DEBUG_PRINTF3(x)
#else
    #define HDMITX_DEBUG_PRINTF(x)
    #define HDMITX_DEBUG_PRINTF1(x)
    #define HDMITX_DEBUG_PRINTF2(x)
    #define HDMITX_DEBUG_PRINTF3(x)
    #define EDID_DEBUG_PRINTF(x)
    #define EDID_DEBUG_PRINTF1(x)
    #define EDID_DEBUG_PRINTF2(x)
    #define EDID_DEBUG_PRINTF3(x)
#endif

#endif // _CONFIG_H_
