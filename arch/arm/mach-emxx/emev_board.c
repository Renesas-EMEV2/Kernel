/*
 *  File Name		: arch/arm/mach-emxx/emev_board.c
 *  Function		: emev_board
 *  Release Version	: Ver 1.11
 *  Release Date	: 2011/06/07
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
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mm.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/smsc911x.h>
#include <linux/dm9000.h>
#ifdef CONFIG_EMXX_ANDROID
#include <linux/android_pmem.h>
#include <linux/usb/android_composite.h>
#endif

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/hardware/gic.h>
#include <asm/hardware/cache-l2x0.h>

#include <mach/hardware.h>
#include <mach/smu.h>
#include <mach/emxx_mem.h>
#include <mach/gpio.h>
#ifdef CONFIG_EMXX_PWC
#include <mach/pwc.h>
#endif

#include "generic.h"
#include "timer.h"

#ifdef CONFIG_MACH_EMEV
static int __initdata emxx_serial_ports[] = { 1, 0, 0, 0 };
#else
static int __initdata emxx_serial_ports[] = { 1, 0, 0, 0, 0, 0};
#endif

#ifdef CONFIG_EMXX_PWC
/* PWC */
static struct pwc_reg_init __initdata pwc_board_init_data[] = {
	{DA9052_BUCKMEM_REG, 0x40, 0x40},
	{DA9052_BUCKPERI_REG, 0x40, 0x40},
	{DA9052_LDO2_REG, 0x00, 0x40}, /* disable */
	{DA9052_LDO3_REG, 0x40, 0x40},
	{DA9052_LDO4_REG, 0x40, 0x40},
	{DA9052_LDO6_REG, 0x00, 0x40}, /* disable */
	{DA9052_LDO7_REG, 0x40, 0x40},
#if defined(CONFIG_MACH_EMEV)
	{DA9052_LDO8_REG, 0x00, 0x40}, /* disable */
#elif defined(CONFIG_MACH_EMGR)
	{DA9052_LDO8_REG, 0x40, 0x40},
#endif
	{DA9052_LDO9_REG, 0x40, 0x40},
	{DA9052_LDO10_REG, 0x40, 0x40},
	{DA9052_SUPPLY_REG, 0x1c, 0x1c}, /* Ramp BUCKMEM, VLDO2, VLDO3 */

	{0xff, 0, 0}, /* end */
};
#endif

/* Ether */
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
static struct resource smsc911x_resources[] = {
	[0] = {
		.start	= EMEV_ETHER_BASE,
		.end	= EMEV_ETHER_BASE + SZ_64K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= INT_ETHER,
		.end	= INT_ETHER,
		.flags	= IORESOURCE_IRQ | IRQ_TYPE_LEVEL_HIGH,
	},
};
static struct smsc911x_platform_config smsc911x_platdata = {
	.flags		= SMSC911X_USE_32BIT,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,
};
static struct platform_device smc91x_device = {
	.name	= "smsc911x",
	.id	= 0,
	.dev	= {
		  .platform_data = &smsc911x_platdata,
		},
	.num_resources	= ARRAY_SIZE(smsc911x_resources),
	.resource	= smsc911x_resources,
};
#elif defined(CONFIG_DM9000) || defined(CONFIG_DM9000_MODULE)
static struct resource dm9000_resource[] = {
	/* IO_PORT */
	[0] = {
		.start = EMEV_ETHER_BASE,
		.end   = EMEV_ETHER_BASE + 3,
		.flags = IORESOURCE_MEM,
	},
	/* DATA_PORT */
	[1] = {
		.start = EMEV_ETHER_BASE + 0x00020000,
		.end   = EMEV_ETHER_BASE + 0x00020000 + 3,
		.flags = IORESOURCE_MEM,
	},
	/* IRQ */
	[2] = {
		.start = INT_ETHER,
		.end   = INT_ETHER,
		/*.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE, */
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	}
};
static struct dm9000_plat_data dm9000_platdata = {
	.flags		= DM9000_PLATF_8BITONLY,
};
static struct platform_device dm9000_device = {
	.name		= "dm9000",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(dm9000_resource),
	.resource	= dm9000_resource,
	.dev		= {
		.platform_data = &dm9000_platdata,
	}
};
#endif

/* Touch Panel */
static struct platform_device da9052_ts_device = {
	.name	= "da9052-ts",
	.id	= -1,
};

static struct platform_device max7318_key_device = {
	.name	= "max7318_key",
	.id	= -1,
};

/* Light */
static struct platform_device emxx_light_device = {
	.name	= "emxx-light",
	.id	= -1,
};

/* Battery */
static struct platform_device emxx_battery_device = {
	.name	= "emxx-battery",
	.id	= -1,
};

/* NAND */
static struct mtd_partition emxx_nand_partition[] = {
	{
		.name = "nand data",
		.offset = 0,
		.size = MTDPART_SIZ_FULL,
	},
};
static struct platform_nand_chip emxx_nand_data = {
	.nr_chips	  = 1,
	.chip_delay	= 15,
	.options	   = 0,
	.partitions	= emxx_nand_partition,
	.nr_partitions = ARRAY_SIZE(emxx_nand_partition),
};
static struct resource emxx_nand_resource[] = {
	{
		.start = EMEV_NAND_DATA_BASE,
		.end   = EMEV_NAND_DATA_BASE + 4 - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = EMEV_NAND_COMMAND_BASE,
		.end   = EMEV_NAND_COMMAND_BASE + 4 - 1 ,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = EMEV_NAND_ADDRESS_BASE,
		.end   = EMEV_NAND_ADDRESS_BASE + 4 - 1,
		.flags = IORESOURCE_MEM,
	},
};
static struct platform_device emxx_nand_device = {
	.name = "emxx_nand",
	.id   = -1,
	.dev  = {
		.platform_data = &emxx_nand_data,
		},
	.num_resources = ARRAY_SIZE(emxx_nand_resource),
	.resource = emxx_nand_resource,
};

#ifdef CONFIG_EMXX_ANDROID
#ifdef CONFIG_ANDROID_PMEM
/* PMEM */
static struct android_pmem_platform_data android_pmem_pdata = {
	.name	= "pmem",
	.start	= EMXX_PMEM_BASE,
	.size	= EMXX_PMEM_SIZE,
	.no_allocator = 1,
	.cached	= 1,
};
static struct platform_device android_pmem_device = {
	.name	= "android_pmem",
	.id	= 0,
	.dev	= {
		  .platform_data = &android_pmem_pdata
		},
};
#endif

/* Android USB gadget  */
static char *usb_functions_ums[] = { "usb_mass_storage" };
static char *usb_functions_ums_adb[] = { "usb_mass_storage", "adb" };

static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x0001,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= 0x0002,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
};
static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x18d1,
	.product_id	= 0x0001,
	.version	= 0x0100,
	.product_name		= "EMMA Mobile",
	.manufacturer_name	= "Renesas.",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_ums_adb),
	.functions = usb_functions_ums_adb,
};
static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "Renesas.",
	.product	= "EMMA Mobile",
	.release	= 0x0100,
};
static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};
#endif

static struct platform_device *devs[] __initdata = {
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
	&smc91x_device,
#elif defined(CONFIG_DM9000) || defined(CONFIG_DM9000_MODULE)
	&dm9000_device,
#endif
	&da9052_ts_device,
	&max7318_key_device,
	&emxx_light_device,
	&emxx_battery_device,
	&emxx_nand_device,
#ifdef CONFIG_EMXX_ANDROID
#ifdef CONFIG_ANDROID_PMEM
	&android_pmem_device,
#endif
	&usb_mass_storage_device,
	&android_usb_device,
#endif
};


static struct i2c_board_info emev_i2c_devices[] = {
	{
	  I2C_BOARD_INFO(I2C_SLAVE_RTC_NAME,    I2C_SLAVE_RTC_ADDR),
	},
#if defined(CONFIG_VIDEO_EMXX_CAMERA) || \
		defined(CONFIG_VIDEO_EMXX_CAMERA_MODULE)
	{
	  I2C_BOARD_INFO(I2C_SLAVE_CAM_NAME,    I2C_SLAVE_CAM_ADDR),
	},
	{
	  I2C_BOARD_INFO(I2C_SLAVE_CAM_AF_NAME, I2C_SLAVE_CAM_AF_ADDR),
	},
#endif
	{
	  I2C_BOARD_INFO("pixcir", 0x5c),
	  .irq = INT_GPIO_29,
	},
};

static void __init emev_board_map_io(void)
{
	emxx_map_io();
	system_rev = readl(EMXX_SRAM_VIRT + 0x1ffe0);
}

static void __init emev_init_irq(void)
{
	/* core tile GIC, primary */
	gic_dist_init(0, __io_address(EMXX_INTA_DIST_BASE), INT_CPU_TIM);
	gic_cpu_init(0, __io_address(EMXX_INTA_CPU_BASE));
}

static void __init emev_board_init(void)
{
#ifdef CONFIG_EMXX_L310
	void __iomem *l2cc_base = (void *)EMXX_L2CC_VIRT;
#endif

	emxx_serial_init(emxx_serial_ports);

#ifdef CONFIG_SMP
	writel(0xfff00000, EMXX_SCU_VIRT + 0x44);
	writel(0xffe00000, EMXX_SCU_VIRT + 0x40);
	writel(0x00000003, EMXX_SCU_VIRT + 0x00);
#endif

#ifdef CONFIG_EMXX_L310
#ifdef CONFIG_EMXX_L310_NORAM

	writel(0, l2cc_base + L2X0_TAG_LATENCY_CTRL);
	writel(0, l2cc_base + L2X0_DATA_LATENCY_CTRL);

	/* 8-way 16KB cache, Early BRESP, Shared attribute override */
	l2x0_init(l2cc_base, 0x40400000, 0x00000000);

#else	/* CONFIG_EMXX_L310_NORAM */

#ifdef CONFIG_EMXX_L310_WT
	/* Force L2 write through */
	writel(0x2, l2cc_base + L2X0_DEBUG_CTRL);
#endif
	writel(0x111, l2cc_base + L2X0_TAG_LATENCY_CTRL);
	writel(0x111, l2cc_base + L2X0_DATA_LATENCY_CTRL);
#ifdef CONFIG_SMP
	writel(0xfff00000, l2cc_base + 0xc04);
	writel(0xffe00001, l2cc_base + 0xc00);
#endif

#ifdef CONFIG_EMXX_L310_8WAY
	/* 8-way 32KB cache, Early BRESP */
	writel(0, SMU_CPU_ASSOCIATIVITY);	/* 0:8-way 1:16-way */
	writel(2, SMU_CPU_WAYSIZE);		/* 0,1:16KB 2:32KB */
	l2x0_init(l2cc_base, 0x40040000, 0x00000fff);
#else
	/* 16-way 16KB cache, Early BRESP */
	writel(1, SMU_CPU_ASSOCIATIVITY);	/* 0:8-way 1:16-way */
	writel(1, SMU_CPU_WAYSIZE);		/* 0,1:16KB 2:32KB */
	l2x0_init(l2cc_base, 0x40030000, 0x00000fff);
#endif

#endif	/* CONFIG_EMXX_L310_NORAM */
#endif	/* CONFIG_EMXX_L310 */

	platform_add_devices(devs, ARRAY_SIZE(devs));
	i2c_register_board_info(0, emev_i2c_devices,
			ARRAY_SIZE(emev_i2c_devices));

	printk(KERN_INFO "chip revision %x\n", system_rev);

#ifdef CONFIG_EMXX_QR
#ifdef CONFIG_MACH_EMEV
#ifdef CONFIG_SMP
	if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1)
		return;
#endif /* CONFIG_SMP */
	if ((system_rev & EMXX_REV_MASK) == EMXX_REV_ES1)
		writel(0x01111101, SMU_PC_SWENA);
	writel(0x00444444, SMU_QR_WAITCNT);
	writel(0x00000003, SMU_QR_WFI);
#elif defined(CONFIG_MACH_EMGR)
	writel(0x00444444, SMU_QR_WAITCNT);
	writel(0x00000001, SMU_QR_WFI);
#endif /* CONFIG_MACH_EMEV */
#endif /* CONFIG_EMXX_QR */

#ifdef CONFIG_EMXX_PWC
	{
		extern struct pwc_reg_init *pwc_init_data;
		pwc_init_data = pwc_board_init_data;
	}
#endif
/*Setup LCD*/
	{
		int val;
		gpio_direction_output(SCREEN_RST , 1);
		writel(readl(CHG_PINSEL_G096) | 0x88, CHG_PINSEL_G096);
		gpio_direction_output(SCREEN_DISP , 1);
		gpio_direction_output(SCREEN_POERON, 1);

		val = readl(CHG_PINSEL_G128);
		writel(val|0x8000, CHG_PINSEL_G128);
		val=readl(CHG_PINSEL_G000);
		writel(val|(0x1f << 13) | (0x1 << 26), CHG_PINSEL_G000);
		val=readl(CHG_PULL21);
		writel(val|0x00000005, CHG_PULL21);
		val=readl(CHG_PULL14);
		writel(val|0x55555000, CHG_PULL14);
		val=readl(CHG_PULL1) ;
		writel(val| 0x00000005, CHG_PULL1);
	}
}

MACHINE_START(EMXX, "EMXX")
	.phys_io      = EMXX_UART0_BASE,
	.io_pg_offst  = (IO_ADDRESS(EMXX_UART0_BASE) >> 18) & 0xfffc,
	.boot_params  = PHYS_OFFSET + 0x100,
	.soft_reboot  = 1,
	.map_io       = emev_board_map_io,
	.init_irq     = emev_init_irq,
	.init_machine = emev_board_init,
	.timer        = &emxx_timer,
MACHINE_END
