/*
 * drivers/video/emxx/neon_alphablend_type2_0_1_1.S
 *  Alpha blending of type2 of BGR888 data format is performed.
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
    .file   "neon_alphablend_type2_0_1_1.s"

#include "arg_struct.h"
#;//--------------------------------------
#;//use define of _AlphaARG struct info.
#;//--------------------------------------
    usestruct _AlphaARG

    .text

.equ    USE_SATULATE,  1
.equ    USE_PRELOAD,   1
.equ    USE_PRELOADW,  0

#;//----------------------------------------
#;//func         AlphaComposit_neon_type2_0_1_1
#;//arg          r0   parames structure
#;//explanation  ABGR8888 is used for a data format of source picture 1.
#;//             BGR888   is used for a data format of source picture 2.
#;//             BGR888   is used for a data format of an output picture.
#;//             This function deals with source picture 1 as input in
#;//             the front, deals with source picture 2 as input in the
#;//             back and does alpha blending processing of type2.
#;//----------------------------------------
    .align  3
    .p2align 4,,15
    .global AlphaComposit_neon_type2_0_1_1
    .type   AlphaComposit_neon_type2_0_1_1, %function

# remove .fnstart because object contain
# undefined reference to __aeabi_unwind_cpp_pr0
#   .fnstart
AlphaComposit_neon_type2_0_1_1:
    push    {r4, r5, r6, r7, r8, r9, r10, r11,r12,lr}

#;//----------------------------------------------------------------
#;// if not use q4-q8, d8-d15, s16-s31 then not need preserve q3-q7.
#;//    vpush   {q4}

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
    vld3.8 {d4-d6}, [r7]!
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
    pld   [r7,#24]
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

#;//q12: 256- (Asrc + (Asrc>>7))

#;//--------------------
#;//calculate dst*(256-alpha)
#;//--------------------
#;//q12: 256-src
    vmul.u16    q8,  q8,  q12
    vmul.u16    q9,  q9,  q12
    vmul.u16    q10, q10, q12

#;// q8: R * alpha    q10: B * alpha
#;// q9: G * alpha    q11: A * alpha
#;//divide 256
    cmp  r9, #8
    vshrn.u16   d24,  q8,  #8
    vshrn.u16   d25,  q9,  #8
    vshrn.u16   d26,  q10, #8

#;//--------------------
#;// add front image
.if USE_SATULATE
    vqadd.u8    q8,  q0, q12
    vqadd.u8    d18, d2, d26
.else
    vadd.u8     q8,  q0, q12
    vadd.u8     d18, d2, d26
.endif
#;//--------------------
#;//check width end and store result
#;  *interlock-1*/
    vst3.8  {d16,d17,d18}, [r8]!

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

    subs    r5, #1

    add     r1, r10
    add     r2, r11
    add     r3, r12

    bne     .L1

.L8:
#;//    vpop    {q4}
    pop     {r4, r5, r6, r7, r8, r9, r10, r11, r12, pc}

;//---------------------------------------------
;// process last 1-7 pixel
;//---------------------------------------------
    .align 3
.L10LastBlock:
#;//--------------------
#;//load pixel data
#;//--------------------
    cmp r9, #4
    beq .L10LastBlock_R4
    bgt .L10LastBlock_RL0
#;//read 1-3
    cmp r9, #2
    blt .L10LastBlock_R1
    bgt .L10LastBlock_R3
    b   .L10LastBlock_R2
#;//read 5-7
    .align 3
.L10LastBlock_RL0:
    cmp r9, #6
    blt .L10LastBlock_R5
    bgt .L10LastBlock_R7
    b   .L10LastBlock_R6

    .align 3
.L10LastBlock_R7:
    vld4.u8 {d0[6],d1[6],d2[6],d3[6]}, [r6]!
    vld3.u8 {d4[6],d5[6],d6[6]},       [r7]!
.L10LastBlock_R6:
    vld4.u8 {d0[5],d1[5],d2[5],d3[5]}, [r6]!
    vld3.u8 {d4[5],d5[5],d6[5]},       [r7]!
.L10LastBlock_R5:
    vld4.u8 {d0[4],d1[4],d2[4],d3[4]}, [r6]!
    vld3.u8 {d4[4],d5[4],d6[4]},       [r7]!
.L10LastBlock_R4:
    vld4.u8 {d0[3],d1[3],d2[3],d3[3]}, [r6]!
    vld3.u8 {d4[3],d5[3],d6[3]},       [r7]!
.L10LastBlock_R3:
    vld4.u8 {d0[2],d1[2],d2[2],d3[2]}, [r6]!
    vld3.u8 {d4[2],d5[2],d6[2]},       [r7]!
.L10LastBlock_R2:
    vld4.u8 {d0[1],d1[1],d2[1],d3[1]}, [r6]!
    vld3.u8 {d4[1],d5[1],d6[1]},       [r7]!
.L10LastBlock_R1:
    vld4.u8 {d0[0],d1[0],d2[0],d3[0]}, [r6]!
    vld3.u8 {d4[0],d5[0],d6[0]},       [r7]!
.L10LastBlockRF:
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
#;//q12: 256- (Asrc + (Asrc>>7))

#;//--------------------
#;//calculate dst*(256-alpha)
#;//--------------------
#;//q12: 256-src
    vmul.u16    q8,  q8,  q12
    vmul.u16    q9,  q9,  q12
    vmul.u16    q10, q10, q12

#;// q8: R * alpha    q10: B * alpha
#;// q9: G * alpha    q11: A * alpha
#;//divide 256
    cmp r9, #4
    vshrn.u16   d24,  q8,  #8
    vshrn.u16   d25,  q9,  #8
    vshrn.u16   d26,  q10, #8

#;//--------------------
#;// add front image
.if USE_SATULATE
    vqadd.u8    q8,  q0, q12
    vqadd.u8    d18, d2, d26
.else
    vadd.u8     q8,  q0, q12
    vadd.u8     d18, d2, d26
.endif


#;//--------------------
#;//save pixel data
#;//--------------------
    beq .L10LastBlock_W4
    bgt .L10LastBlock_WL0
#;//store 1-3
    cmp r9, #2
    blt .L10LastBlock_W1
    bgt .L10LastBlock_W3
    b   .L10LastBlock_W2
#;//store 5-7
    .align 3
.L10LastBlock_WL0:
    cmp r9, #6
    blt .L10LastBlock_W5
    bgt .L10LastBlock_W7
    b   .L10LastBlock_W6

    .align 3
.L10LastBlock_W7:
    vst3.8  {d16[6], d17[6], d18[6]}, [r8]!
.L10LastBlock_W6:
    vst3.8  {d16[5], d17[5], d18[5]}, [r8]!
.L10LastBlock_W5:
    vst3.8  {d16[4], d17[4], d18[4]}, [r8]!
.L10LastBlock_W4:
    vst3.8  {d16[3], d17[3], d18[3]}, [r8]!
.L10LastBlock_W3:
    vst3.8  {d16[2], d17[2], d18[2]}, [r8]!
.L10LastBlock_W2:
    vst3.8  {d16[1], d17[1], d18[1]}, [r8]!
.L10LastBlock_W1:
    vst3.8  {d16[0], d17[0], d18[0]}, [r8]!
    b   .L7


# remove .fnstart because object contain
# undefined reference to __aeabi_unwind_cpp_pr0
#   .fnend

    .size   AlphaComposit_neon_type2_0_1_1, .-AlphaComposit_neon_type2_0_1_1



