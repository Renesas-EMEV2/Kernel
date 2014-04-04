#include <linux/module.h>
#include <linux/gpio_keys.h>
#include <asm/gpio.h>
#include <mach/smu.h>




static int __init emxx_regs_init(void)
{
	//gpio-keys +++++++++++++++++++++++++++++++++++++++++++++
	// set pin 143,13~17,26 to gpio mode
	writel(readl(CHG_PINSEL_G128) | (0x1 << 15), CHG_PINSEL_G128);
	writel(readl(CHG_PINSEL_G000) | (0x1f << 13) | (0x1 << 26), CHG_PINSEL_G000);
	// set gpio 143,13~17,26 to enable input to pin
	writel((readl(CHG_PULL21) | 0x00000005), CHG_PULL21);
	writel((readl(CHG_PULL14) | 0x55555000), CHG_PULL14);
	writel((readl(CHG_PULL1) | 0x00000005), CHG_PULL1);
	//goio-keys ---------------------------------------------

        // HDMI HPD input
        // set pin 27 to gpio mode and enable input
        writel(readl(CHG_PINSEL_G000) | (0x1 << 27), CHG_PINSEL_G000);
        writel((readl(CHG_PULL1) | 0x00000050), CHG_PULL1);

	return 0;
}

static void __exit emxx_regs_exit(void)
{
}

module_init(emxx_regs_init);
module_exit(emxx_regs_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("hengai <qiang@livall.cn>");
MODULE_DESCRIPTION("Renesas EV2 regs init");

