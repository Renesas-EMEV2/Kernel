/*
 * drivers/video/emxx/neon_alphablend_type2_0_0_0.S
 *  Alpha blending of type2 of ABGR8888 data format is performed.
 *
 * Copyright (C) 2010 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA.
 */
    .cpu cortex-a8
#   /* http://infocenter.arm.com/help/topic/com.arm.doc.ihi0045c/ */
#   /* IHI0045C_ABI_addenda.pdf                                   */
#   /* Tag_ABI_HardFP_use: Single Precision/Double Precision enabled */
    .eabi_attribute 27, 3
    .fpu neon
#   /* Tag_ABI_FP_denormali      : inexact result        */
    .eabi_attribute 20, 1
#   /* Tag_ABI_FP_user_exceptions: enable user exception */
    .eabi_attribute 21, 1
#   /* Tag_ABI_FP_number_model   : IEEE754 format        */
    .eabi_attribute 23, 3
#   /* Tag_ABI_align_needed      : code depends on 8 byte alignment. */
    .eabi_attribute 24, 1
#   /* Tag_ABI_align8_preserved  : code preserve 8 byte alignment.   */
    .eabi_attribute 25, 1
#   /* Tag_ABI_enum_size         : use enum 32bit */
    .eabi_attribute 26, 2
#   /* Tag_ABI_optimization goal : optimize agressively for speed. */
    .eabi_attribute 30, 2
#   /* Tag_ABI_PCS_wchar_t       : wchar_t 32bit */
    .eabi_attribute 18, 4
#   /* (currently not supported.) arm v7 MP extension enables */
#   .eabi_attribute 42, 1
	.file	"neon_alphablend_type2_0_0_0.s"

#include "arg_struct.h"
#;//--------------------------------------
#;//use define of _AlphaARG struct info.
#;//--------------------------------------
	usestruct _AlphaARG

	.text

.equ	USE_SATULATE,  1
.equ	USE_PRELOAD,   1
.equ	USE_PRELOADW,  0

#;//----------------------------------------
#;//func         AlphaComposit_neon_type2_0_0_0
#;//arg          r0   parames structure
#;//explanation  ABGR8888 is used for a data format of source picture 1.
#;//             ABGR8888 is used for a data format of source picture 2.
#;//             ABGR8888 is used for a data format of an output picture.
#;//             This function deals with source picture 1 as input in
#;//             the front, deals with source picture 2 as input in the
#;//             back and does alpha blending processing of type2.
#;//----------------------------------------
	.align	3
	.p2align 4,,15
	.global	AlphaComposit_neon_type2_0_0_0
	.type	AlphaComposit_neon_type2_0_0_0, %function

# remove .fnstart because object contain
# undefined reference to __aeabi_unwind_cpp_pr0
#	.fnstart
AlphaComposit_neon_type2_0_0_0:
	push    {r4, r5, r6, r7, r8, r9, r10, r11,r12,lr}

#;//----------------------------------------------------------------
#;// if not use q4-q8, d8-d15, s16-s31 then not need preserve q3-q7.
#;//	vpush	{q4}

#;//--------------------
#;//set constant
#;//--------------------
	mov       r3, #0x100;
#;//--------------------
#;//get arguments
#;//--------------------
	ldr    r5,  [r0, #__struct(_AlphaARG,dst_vsize)]
	ldr    r4,  [r0, #__struct(_AlphaARG,dst_hsize)]
	add    r8,  r0,#__struct(_AlphaARG,src_ptr)
	cmp    r5,  #0
	vdup.u16  q15, r3
	ldmia  r8,  {r1,r2,r3,r10,r11,r12}
#;//--------------------
#;//r1: src_ptr       r4: width   r10: src_size
#;//r2: src_b_ptr     r5: height  r11: src_b_size
#;//r3: dst_ptr                   r12: dst_size
#;//--------------------
	ble    .L8


	.align 3
.L1:
#;//------------------
#;//make work variables
#;//------------------
.if  USE_PRELOAD
	pld    [r1]
	pld    [r2]
.endif
.if  USE_PRELOADW
	pldw   [r3]  /* for write address */
.endif
	cmp    r4, #8
	mov    r9, r4
	mov    r6, r1
	mov    r7, r2
	mov    r8, r3
#;//--------------------
#;//scratch
#;//--------------------
#;//r6: src_ptr   address of next load
#;//r7: src_b_ptr address of next load
#;//r8: dst_ptr
#;//r9: width


#;//--------------------
#;//8 pix per per loop
#;//--------------------
	blt   .L3

	.align 3
.L2:
#;//--------------------
#;//load pixel data
#;//--------------------
	vld4.8 {d0-d3}, [r6]!
#;  * interlock-1: 3 cycle instruction
#;  * interlock-1: 3 cycle instruction
#;  * interlock-1: 3 cycle instruction
	vld4.8 {d4-d7}, [r7]!
#;  * interlock-1: 3 cycle instruction
#;  * interlock-1: 3 cycle instruction
#;  * interlock-1: 3 cycle instruction
#;//--------------------
#;//decrement counter
#;//--------------------

#;//--------------------
#;//calculate Alpha
#;//--------------------
	vmovl.u8    q12, d3
.if  USE_PRELOAD
	pld   [r6,#32]
.endif
	vmovl.u8    q8,  d4
.if  USE_PRELOAD
	pld   [r7,#32]
.endif
	vshr.u16    q13, q12, #7
.if  USE_PRELOADW
	pldw   [r8,#32] /* for write address */
.endif
	vmovl.u8    q9,  d5
	sub  r9, r9, #8
	vadd.u16    q14, q13, q12
	vmovl.u8    q10, d6
	vsub.u16    q12, q15, q14
	vmovl.u8    q11, d7
#;//q12: 256- (Asrc + (Asrc>>7))

#;//--------------------
#;//calculate dst*(256-alpha)
#;//--------------------
#;//q12: 256-src
	vmul.u16    q8,  q8,  q12
	vmul.u16    q9,  q9,  q12
	vmul.u16    q10, q10, q12
	vmul.u16    q11, q11, q12

#;// q8: R * alpha    q10: B * alpha
#;// q9: G * alpha    q11: A * alpha
#;//divide 256
	cmp  r9, #8
	vshrn.u16   d24,  q8,  #8
	vshrn.u16   d25,  q9,  #8
	vshrn.u16   d26,  q10, #8
	vshrn.u16   d27,  q11, #8

#;//--------------------
#;// add front image
.if USE_SATULATE
	vqadd.u8    q8, q0, q12
	vqadd.u8    q9, q1, q13
.else
	vadd.u32    q8, q0, q12
	vadd.u32    q9, q1, q13
.endif
	vst4.8	{d16,d17,d18,d19}, [r8]!


	bge .L2
.L3:
	cmp   r9, #0
#;//--------------------
#;//last 1-7 pixel, if remainig.
#;//--------------------
	bgt   .L10LastBlock

.L7:
#;//--------------------
#;//move to next line
#;//--------------------
#;// r1: src_ptr,   r10: src_size
#;// r2: src_b_ptr, r11: src_b_size
#;// r3: dst_ptr,   r12: dst_size
#;// r5: height

	subs	r5, #1

	add		r1, r10
	add		r2, r11
	add		r3, r12

	bne 	.L1

.L8:
#;//	vpop	{q4}
	pop   	{r4, r5, r6, r7, r8, r9, r10, r11, r12, pc}

;//---------------------------------------------
;// process last 1-7 pixel
;//---------------------------------------------
	.align 3
.L10LastBlock:
#;//--------------------
#;//load pixel data
#;//--------------------
	tst r9, #4
	beq .L10LastBlockR1
#;//--load 4 elements
	vld1.u32 {d0,d1}, [r6]!
	vld1.u32 {d4,d5}, [r7]!
.L10LastBlockR1:
	tst r9, #2
	beq .L10LastBlockR2
#;//--load 2 elements
	vld1.u32 {d2}, [r6]!
	vld1.u32 {d6}, [r7]!
.L10LastBlockR2:
	tst r9, #1
	beq .L10LastBlockRF
#;//--load 1 elements
	vld1.u32 {d3[0]}, [r6]!
	vld1.u32 {d7[0]}, [r7]!
	.align 3
.L10LastBlockRF:
#;//---------------------------
#;// reordering blending data
#;//---------------------------
	vuzp.u8 d0, d1
	vuzp.u8 d2, d3
	vuzp.u8 d4, d5
	vuzp.u8 d6, d7
	vuzp.u8 d0, d2
	vuzp.u8 d1, d3
	vuzp.u8 d4, d6
	vuzp.u8 d5, d7

#;//--------------------
#;//calculate Alpha
#;//--------------------
	vmovl.u8    q12, d3
	vmovl.u8    q8,  d4
	vshr.u16    q13, q12, #7
	vmovl.u8    q9,  d5
	vadd.u16    q14, q13, q12
	vmovl.u8    q10, d6
	vsub.u16    q12, q15, q14
	vmovl.u8    q11, d7
#;//q12: 256- (Asrc + (Asrc>>7))

#;//--------------------
#;//calculate dst*(256-alpha)
#;//--------------------
#;//q12: 256-src
	vmul.u16    q8,  q8,  q12
	vmul.u16    q9,  q9,  q12
	vmul.u16    q10, q10, q12
	vmul.u16    q11, q11, q12

#;// q8: R * alpha    q10: B * alpha
#;// q9: G * alpha    q11: A * alpha
#;//divide 256
	tst r9, #4
	vshrn.u16   d24,  q8,  #8
	vshrn.u16   d25,  q9,  #8
	vshrn.u16   d26,  q10, #8
	vshrn.u16   d27,  q11, #8

#;//--------------------
#;// add front image
.if USE_SATULATE
	vqadd.u8    q8, q0, q12
	vqadd.u8    q9, q1, q13
.else
	vadd.u32    q8, q0, q12
	vadd.u32    q9, q1, q13
.endif

#;//---------------------------
#;// reordering blending data
#;//---------------------------
	vzip.u8 d16, d18
	vzip.u8 d17, d19
	vzip.u8 d16, d17
	vzip.u8 d18, d19
    beq .L10LastBlockW2
#;//---------------------
#;// store d0-d1 (4 pixel)
    vst1.32  {d16, d17}, [r8]!
.align 3
.L10LastBlockW2:
    tst r9, #2
    beq .L10LastBlockW3
#;//---------------------
#;// store d0    (2 pixel)
    vst1.32  {d18}, [r8]!
.align 3
.L10LastBlockW3:
    tst r9, #1
    beq .L7
#;//---------------------
#;// store 32bit (1 pixel)
    vst1.32 {d19[0]}, [r8]
	b	.L7


# remove .fnstart because object contain
# undefined reference to __aeabi_unwind_cpp_pr0
#	.fnend

	.size	AlphaComposit_neon_type2_0_0_0, .-AlphaComposit_neon_type2_0_0_0



