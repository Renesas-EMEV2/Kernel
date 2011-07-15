/*
 * arch/arm/mach-emxx/include/mach/debug-macro.S
 *
 * Debugging macro include header
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

#include <mach/io.h>

	.macro	addruart,rx,tmp
	mrc	p15, 0, \rx, c1, c0
	tst	\rx, #1			@ MMU enabled?
	mov	\rx, #0xe1000000	@ physical base
	bicne	\rx, \rx, #0xf0000000	@ virtual base
	orrne	\rx, \rx, #IO_BASE
	orr	\rx, \rx, #0x00020000	@ UART0
	.endm

#define UART_SHIFT	2
#include <linux/serial_reg.h>

		.macro	senduart,rd,rx
		strb	\rd, [\rx, #UART_TX << UART_SHIFT]
		.endm

		.macro	busyuart,rd,rx
1002:		ldr	\rd, [\rx, #UART_LSR << UART_SHIFT]
		and	\rd, \rd, #UART_LSR_TEMT | UART_LSR_THRE
		teq	\rd, #UART_LSR_TEMT | UART_LSR_THRE
		bne	1002b
		.endm

		.macro	waituart,rd,rx
		.endm
