/*
 *  File Name       : arch/arm/mach-emxx/include/mach/smp.h
 *  Function        : smp
 *  Release Version : Ver 1.01
 *  Release Date    : 2011/01/21
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
 *
 */

#ifndef __ASM_ARCH_SMP_H
#define __ASM_ARCH_SMP_H

#include <asm/hardware/gic.h>
#include <asm/smp_mpidr.h>

extern volatile int pen_release;
extern void emxx_secondary_startup(void);

/*
 * We use IRQ1 as the IPI
 */
static inline void smp_cross_call(const struct cpumask *mask)
{
	gic_raise_softirq(mask, 1);
}

#endif	/* __ASM_ARCH_SMP_H */
