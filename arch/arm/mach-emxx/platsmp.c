/*
 *  linux/arch/arm/mach-emxx/platsmp.c
 *
 *  Copyright (C) 2010 Renesas Electronics Corporation
 *
 *  This file is based on the arch/arm/mach-realview/platsmp.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/smp.h>
#include <linux/io.h>

#include <asm/cacheflush.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>

#include <mach/smu.h>
#include <asm/smp_scu.h>
#include <asm/localtimer.h>

/* SCU base address */
static void __iomem *scu_base = (void __iomem *)EMXX_SCU_VIRT;


/*
 * control for which core is the next to come out of the secondary
 * boot "holding pen"
 */
volatile int pen_release = -1;

static inline unsigned int get_core_count(void)
{
	if (scu_base)
		return scu_get_core_count(scu_base);
	else
		return 1;
}

static DEFINE_SPINLOCK(boot_lock);

void __cpuinit platform_secondary_init(unsigned int cpu)
{
	trace_hardirqs_off();

	gic_cpu_init(0, __io_address(EMXX_INTA_CPU_BASE));

	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	pen_release = -1;
	smp_wmb();
}

int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;

	/*
	 * set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	pen_release = cpu;
	flush_cache_all();

	smp_cross_call(cpumask_of(cpu));

	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (pen_release == -1)
			break;
		udelay(10);
	}

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return pen_release != -1 ? -ENOSYS : 0;
}

static void __init wakeup_secondary(void)
{
	/* nobody is to be released from the pen yet */
	pen_release = -1;

	/*
	 * Write the address of secondary startup into the system-wide flags
	 * register. The BootMonitor waits for this register to become
	 * non-zero.
	 */
	__raw_writel(virt_to_phys(emxx_secondary_startup),
				SMU_GENERAL_REG0);

	mb();
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init smp_init_cpus(void)
{
	unsigned int i, ncores = get_core_count();

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);
}

void __init smp_prepare_cpus(unsigned int max_cpus)
{
	unsigned int ncores = get_core_count();
	unsigned int cpu = smp_processor_id();
	int i;

	smp_store_cpu_info(cpu);

	/*
	 * are we trying to boot more cores than exist?
	 */
	if (max_cpus > ncores)
		max_cpus = ncores;

	/*
	 * Initialise the present map, which describes the set of CPUs
	 * actually populated at the present time.
	 */
	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);

	/*
	 * Initialise the SCU if there are more than one CPU and let
	 * them know where to start. Note that, on modern versions of
	 * MILO, the "poke" doesn't actually do anything until each
	 * individual core is sent a soft interrupt to get it out of
	 * WFI
	 */
	if (max_cpus > 1) {
		/*
		 * Enable the local timer or broadcast device for the
		 * boot CPU, but only if we have more than one CPU.
		 */
		percpu_timer_setup();

		/*
		 * Initialise the SCU and wake up the secondary core using
		 * wakeup_secondary().
		 */
		scu_enable(scu_base);
		wakeup_secondary();
	}
}
