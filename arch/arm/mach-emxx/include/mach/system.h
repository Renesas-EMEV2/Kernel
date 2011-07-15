/*
 *  File Name       : arch/arm/mach-emxx/include/mach/hardware.h
 *  Function        : system
 *  Release Version : Ver 1.02
 *  Release Date    : 2011/04/04
 *
 * Copyright (C) 2010-2011 Renesas Electronics Corporation
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

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/pwc.h>

extern int emxx_set_reboot_alarm(void);
extern void emxx_pm_power_off(void);
extern void emxx_pm_power_off_restart(void);

static void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char *cmd)
{
	printk(KERN_DEBUG "mode = '%c'\n", mode);
	if (mode == 'h') { /* HALT */
		emxx_pm_power_off();
	} else if (mode == 's') { /* Soft Reboot */
#ifdef CONFIG_RTC_DRV_EMXX
		emxx_set_reboot_alarm();
#endif
		emxx_pm_power_off_restart();
	}
}

#endif	/* __ASM_ARCH_SYSTEM_H */
