/*
 * arch/arm/mach-emxx/include/mach/entry-macro.S
 *
 * Low-level IRQ helper macros
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

#include <mach/hardware.h>
#include <asm/hardware/gic.h>

	.macro	disable_fiq
	.endm

	.macro  get_irqnr_preamble, base, tmp
	ldr	\base, =IO_ADDRESS(EMXX_INTA_CPU_BASE)
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm

	/*
	 * Interrupts
	 * 0-15 are IPI
	 * 17-20 are local.
	 * 21-31 are reserved
	 * 32-1020 are global
	 * 1021-1022 are reserved
	 * 1023 is "spurious" (no interrupt)
	 */

	.macro  get_irqnr_and_base, irqnr, irqstat, base, tmp

	/* bits 12-10 = src CPU, 9-0 = int # */
	ldr     \irqstat, [\base, #GIC_CPU_INTACK]

	ldr	\tmp, =1021

	bic     \irqnr, \irqstat, #0x1c00

	cmp     \irqnr, #17
	it	cc
	cmpcc	\irqnr, \irqnr
	it	ne
	cmpne	\irqnr, \tmp
	it	cs
	cmpcs	\irqnr, \irqnr

	.endm

	/* We assume that irqstat (the raw value of the IRQ acknowledge
	 * register) is preserved from the macro above.
	 * If there is an IPI, we immediately signal end of interrupt on the
	 * controller, since this requires the original irqstat value which
	 * we won't easily be able to recreate later.
	 */

	.macro test_for_ipi, irqnr, irqstat, base, tmp
	bic	\irqnr, \irqstat, #0x1c00
	cmp	\irqnr, #16
	it	cc
	strcc	\irqstat, [\base, #GIC_CPU_EOI]
	it	cs
	cmpcs	\irqnr, \irqnr
	.endm

	/* As above, this assumes that irqstat and base are preserved.. */

	.macro test_for_ltirq, irqnr, irqstat, base, tmp
	bic	\irqnr, \irqstat, #0x1c00
	mov 	\tmp, #0
	cmp	\irqnr, #17
	itt	eq
	moveq	\tmp, #1
	streq	\irqstat, [\base, #GIC_CPU_EOI]
	cmp	\tmp, #0
	.endm
