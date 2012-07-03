/****************************************************************
 * arch/arm/mach-emxx/include/mach/axp192.h
 *
 * Copyright (C) 2010, Renesas Electornics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 ****************************************************************/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/sysctl.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/suspend.h>

#include <asm/ioctl.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include <mach/hardware.h>
//#include <mach/emxx_pmic.h>
#include <mach/axp192.h>
#include <mach/gpio.h>

#define	DEBUG

#define AXP192_REG_NUM		(0xC0)
#define GPIO_PWIC_INT 	0
#define INT_PWC			INT_GPIO_0

extern int get_pm_state(void);
static struct emxx_pmic_regs axp192_regs[AXP192_REG_NUM];
static struct power_supply_module *axp192_power_module;

/* Unique ID allocation */
static struct i2c_client *axp192_client;

static int axp192_init_irq(void)
{
	gpio_direction_input(GPIO_PWIC_INT);
	return 0;
}

static int axp192_ack_irq(void)
{
	return 0;
}

static void emev_axp192_init(void)
{
	/* Mask All interrupts that are not needed */
	axp192_write(AXP_IRQ_MASK1, 0x0);
	axp192_write(AXP_IRQ_MASK2, 0x0);
	axp192_write(AXP_IRQ_MASK3, 0x0);
	axp192_write(AXP_IRQ_MASK4, 0x0);

	/* Enable Coulometer function and clean Coulometer counter */
	axp192_write_mask(AXP_CM_CTRL, AXP_CM_EN | AXP_CM_CLEAN, 0xFF);

	/* Enable system shutdown when Internal Temperature Overflow */
	axp192_write_mask(AXP_INT_THRESHOLD, AXP_INT_SHUTDOWN, AXP_INT_SHUTDOWN);

	/* Disable system shutdown when Power Key Long Press */
	//axp192_write_mask(AXP_PWRKEY_CTRL, AXP_PWRKEY_SHUTDOWN, AXP_PWRKEY_SHUTDOWN);
        axp192_write(0x0E, AXP_BUCK_MODE);
}

/* 
 * axp192_power_module[] should be consistent with enum
 * in arch/arm/mach-pxa/include/mach/pxa3xx_pmic.h 
 */
static struct power_supply_module axp192_power_modules[] = {
	/* {command,            power_module}, */
	   {VCC_CORE,              BUCK1},
};

static struct axp192_platform_data axp192_data = {
	.init_irq = axp192_init_irq,
	.ack_irq = axp192_ack_irq,
	.platform_init = emev_axp192_init,
	.power_supply_modules = axp192_power_modules,
};

int axp192_read(u8 reg, u8 *pval)
{
	//struct axp192_platform_data *pdata = axp192_client->dev.platform_data;
	int ret;
	int status;

	if (axp192_client == NULL)	/* No global client pointer? */
		return -1;

	/* Cache read of the Axp192 register. Disabled temporary */
#if	0
	if (axp192_regs[reg].hit) {
		*pval = axp192_regs[reg].data;
		return 0;
	}
#endif

	//spin_lock(&pdata->lock);
	ret = i2c_smbus_read_byte_data(axp192_client, reg);
	if (ret >= 0) {
		*pval = ret;
		axp192_regs[reg].hit = ~axp192_regs[reg].mask;
		axp192_regs[reg].data = ret;
		status = 0;
	} else
		status = -EIO;
	//spin_unlock(&pdata->lock);

	return status;
}

int axp192_write(u8 reg, u8 val)
{
	//struct axp192_platform_data *pdata = axp192_client->dev.platform_data;
	int ret;
	int status;

	if (axp192_client == NULL)	/* No global client pointer? */
		return -1;

	//spin_lock(&pdata->lock);
	ret = i2c_smbus_write_byte_data(axp192_client, reg, val);
	if (ret == 0) {
		axp192_regs[reg].hit = ~axp192_regs[reg].mask;
		axp192_regs[reg].data = val;
		status = 0;
	} else
		status = -EIO;
	//spin_unlock(&pdata->lock);

	return status;
}

int axp192_write_mask(u8 reg, u8 val, u8 mask)
{
	int ret = 0;
	unsigned char value;
	ret = axp192_read(reg, &value);
	if (ret < 0)
		goto out;

	value &= ~mask;
	value |= val&mask;
	ret = axp192_write(reg, value);
out:
	return ret;
}

static int axp192_initchip(void)
{
	int i;

	memset(&axp192_regs, 0,
			(sizeof(struct emxx_pmic_regs) * AXP192_REG_NUM));
	/* TODO: Mask all axp192 registers uncacheable now.
	 * We can do some optimization here later.
	 */
	for (i = 0; i < AXP192_REG_NUM; i++)
		axp192_regs[i].mask = 1;

	//return axp192_write(AXP192_SYSCTRL_A, 0xE8);
	return 0;
}

static unsigned long adc_ldo_refcount;

void enable_axp192_adc_ldo(void)
{
	/*
	   u8 val;
	   unsigned long flags;

	   local_irq_save(flags);
	   if (!adc_ldo_refcount++) {
	   axp192_read(AXP192_ADC_MAN_CONTROL, &val);
	   val |= AXP192_ADC_MAN_CONT_LDOADC_EN;
	   axp192_write(AXP192_ADC_MAN_CONTROL, val);
	   }
	   local_irq_restore(flags);
	 */	return;
}

void disable_axp192_adc_ldo(void)
{
	/*
	u8 val;
	unsigned long flags;

	local_irq_save(flags);
	if (!adc_ldo_refcount) {
	printk("WARNING: axp192 adc count had been zero\n");
	} else if (!--adc_ldo_refcount) {
	axp192_read(AXP192_ADC_MAN_CONTROL, &val);	
	val &= ~AXP192_ADC_MAN_CONT_LDOADC_EN;
	axp192_write(AXP192_ADC_MAN_CONTROL, val);
	}
	local_irq_restore(flags);
	*/
	return;
}

/* USB Device/OTG functions */
static int axp192_set_pump(int enable)
{
	int status = 0;
#if 0
	u8 val;
	unsigned long flags;
	static int set_pump_count;

	local_irq_save(flags);
	if (enable) {
		if (set_pump_count++ != 0)
			goto out;
		status = micco_read(MICCO_MISC, &val);
		if (status)
			goto out;
		val |= MICCO_MISC_VBUS_COMPS_EN;
		status = micco_write(MICCO_MISC, val);
		if (status)
			goto out;

		/* FIXME: Littleton didn't connect EXTON as cable detection
		 * signal. We use USB_DEV detection in event B as cable
		 * detection.
		 */
		/* TODO: This is a platform related code. Need split
		 * to platform related code.
		 */
		status = micco_read(MICCO_IRQ_MASK_B, &val);
		if (status)
			goto out;
		val &= ~IRQ_MASK_B_USB_DEV;
		status = micco_write(MICCO_IRQ_MASK_B, val);
		if (status)
			goto out;
	} else {
		if (set_pump_count == 0 || set_pump_count-- != 0)
			goto out;

		status = micco_read(MICCO_IRQ_MASK_B, &val);
		if (status)
			goto out;
		val |= (IRQ_MASK_B_SESSION_VALID_1_8 | IRQ_MASK_B_VBUS_VALID_3_8
				| IRQ_MASK_B_VBUS_VALID_4_55);
		status = micco_write(MICCO_IRQ_MASK_B, val);
		if (status)
			goto out;

		status = micco_read(MICCO_MISC, &val);
		if (status)
			goto out;
		val &= ~MICCO_MISC_VBUS_COMPS_EN;
		status = micco_write(MICCO_MISC, val);
		if (status)
			goto out;

		status = micco_read(MICCO_IRQ_MASK_B, &val);
		if (status)
			goto out;
		val |= IRQ_MASK_B_USB_DEV;
		status = micco_write(MICCO_IRQ_MASK_B, val);
		if (status)
			goto out;
	}
out:
	local_irq_restore(flags);
#endif
	return status;
}

/* 1. enable: 1, srp: 0, 100ma mode
 * 2. enable: 1, srp: 1, 10ma mode
 *
 * Axp192 before e: 1:s: 1, must e:0, s:1.
 */
static int axp192_set_vbus_supply(int enable, int srp)
{
	int ret = 0;
#if 0
	u8 val, tmp;

	ret = axp192_read(AXP192_MISC, &val);
	if (ret) {
		printk(KERN_ERR "I2C operation error 0x%x\n", ret);
		return ret;
	}

	if (enable) {
		/* When enable the USB pump, need check the OTGCP_IOVER. */
		axp192_read(AXP192_IRQ_MASK_B, &tmp);
		tmp &= ~IRQ_MASK_B_OTGCP_IOVER;
		axp192_write(AXP192_IRQ_MASK_B, tmp);

		val |= AXP192_MISC_USBCP_EN;
		if (srp) {
			val |= AXP192_MISC_USBSR_EN;
		} else {
			val &= ~AXP192_MISC_USBSR_EN;
		}
	} else {
		/* When disable the USB pump, needn't check the OTGCP_IOVER. */
		axp192_read(AXP192_IRQ_MASK_B, &tmp);
		tmp |= IRQ_MASK_B_OTGCP_IOVER;
		axp192_write(AXP192_IRQ_MASK_B, tmp);

		val &= ~(AXP192_MISC_USBCP_EN | AXP192_MISC_USBSR_EN);
	}
	ret = axp192_write(AXP192_MISC, val);
	if (ret)
		printk(KERN_ERR "I2C operation error 0x%x\n", ret);
#endif
	return ret;
}

static int axp192_set_usbotg_a_mask(void)
{
	int ret = 0;
#if 0
	u8 val;

	ret = axp192_read(AXP192_IRQ_MASK_B, &val);
	if (ret) {
		printk(KERN_ERR "I2C operation error 0x%x\n", ret);
		return ret;
	}

	/* Enable the interrupt for VUBS > 4.4V and Session valid
	 * which A device care about.
	 *
	 * NOTESSSSSSSSS: We care about the SRP detection signal (0.8v ~ 2.0v)
	 * on Axp192. Which map to SESSION_VALID_1_8.
	 */
	val |= (IRQ_MASK_B_VBUS_VALID_3_8 | IRQ_MASK_B_SRP_READY_0_6);
	val &= ~(IRQ_MASK_B_VBUS_VALID_4_55 | IRQ_MASK_B_SESSION_VALID_1_8);

	ret = axp192_write(AXP192_IRQ_MASK_B, val);
#endif
	return ret;
}

static int axp192_set_usbotg_b_mask(void)
{
	int ret = 0;
#if 0
	u8 val;

	ret = axp192_read(AXP192_IRQ_MASK_B, &val);
	if (ret) {
		printk(KERN_ERR "I2C operation error 0x%x\n", ret);
		return ret;
	}

	/* Mask all USB VBUS interrupt for B device */
	val |= (IRQ_MASK_B_VBUS_VALID_3_8 | IRQ_MASK_B_SESSION_VALID_1_8 | \
			IRQ_MASK_B_VBUS_VALID_4_55 | IRQ_MASK_B_SRP_READY_0_6);

	ret = axp192_write(AXP192_IRQ_MASK_B, val);
#endif
	return ret;
}

static int is_axp192_vbus_assert(void)
{
	int ret = 0;
#if 0
	u8 val;

	ret = axp192_read(AXP192_STATUS_B, &val);
	if (ret) {
		printk(KERN_ERR "I2C operation error 0x%x\n", ret);
		return ret;
	}

	/* USBDEV interrupt is not so reliable for vbus assert check,
	 * replace with 4.4V check 
	 * 
	 if (val & AXP192_STATUS_B_USBDEV) */
	if (val & AXP192_STATUS_B_VBUS_V_4P4 || val & AXP192_STATUS_B_USBDEV)
		return 1;
	else
		return 0;
#endif
	return ret;
}

static unsigned int axp192_event_change(unsigned char* states)
{
	unsigned int ret = 0;
	u8 val;

	axp192_read(AXP_IRQ_STATE1, &val);
	states[0] = val;
	if (val & (AXP_IRQ_ACIN_IN | AXP_IRQ_ACIN_OUT
				| AXP_IRQ_ACIN_OV))
		ret |= PMIC_EVENT_ACIN;

	if (val & (AXP_IRQ_VBUS_IN | AXP_IRQ_VBUS_OUT
				| AXP_IRQ_VBUS_OV | AXP_IRQ_VBUS_VHOLD))
		ret |= PMIC_EVENT_VBUS;


	axp192_read(AXP_IRQ_STATE2, &val);
	states[1] = val;
	if (val & (AXP_IRQ_BAT_IN | AXP_IRQ_BAT_OUT
				| AXP_IRQ_BAT_ACT | AXP_IRQ_BAT_UNACT))
		ret |= PMIC_EVENT_BAT;

	if (val & (AXP_IRQ_BAT_CHG | AXP_IRQ_BAT_FULL))
		ret |= PMIC_EVENT_CHG;

	if (val & (AXP_IRQ_BAT_OTH | AXP_IRQ_BAT_OTL))
		ret |= PMIC_EVENT_TBAT;

	axp192_read(AXP_IRQ_STATE3, &val);
	states[2] = val;	
	if (val & (AXP_IRQ_INT_OT))
		ret |= PMIC_EVENT_TINT;

	if (val & (AXP_IRQ_CHG_CURL | AXP_IRQ_DCDC1_VOLL
				| AXP_IRQ_DCDC2_VOLL | AXP_IRQ_DCDC3_VOLL))
		ret |= PMIC_EVENT_CURRENT;

	if (val & (AXP_IRQ_KEY_SHORT | AXP_IRQ_KEY_LONG))
		ret |= PMIC_EVENT_KEY;

	axp192_read(AXP_IRQ_STATE4, &val);
	states[3] = val;
	if (val & (AXP_IRQ_NOE_PWRON | AXP_IRQ_NOE_PWROFF))
		ret |= PMIC_EVENT_POWER;

	if (val & (AXP_IRQ_VBUS_VALID | AXP_IRQ_VBUS_UNVALID
				| AXP_IRQ_VBUS_SESSION | AXP_IRQ_VBUS_END))
		ret |= PMIC_EVENT_USB;

	if (val & (AXP_IRQ_APS_LOWER))
		ret |= PMIC_EVENT_APS;

	return ret;
}

static int is_axp192_avbusvld(void)
{
#if 0
	u8 val;

	axp192_read(AXP192_IRQ_MASK_B, &val);
	if (val & IRQ_MASK_B_VBUS_VALID_4_55)
		return 0;

	axp192_read(AXP192_STATUS_B, &val);
	if (val & AXP192_STATUS_B_VBUS_V_4P4)
		return 1;
	else
		return 0;
#endif
	return 0;
}

static int is_axp192_asessvld(void)
{
#if 0
	u8 val;

	axp192_read(AXP192_IRQ_MASK_B, &val);
	if (val & IRQ_MASK_B_SESSION_VALID_1_8)
		return 0;

	axp192_read(AXP192_STATUS_B, &val);
	if (val & AXP192_STATUS_B_SESS_V_1P8)
		return 1;
	else
#endif
		return 0;
}

static int is_axp192_bsessvld(void)
{
#if 0
	u8 val;

	axp192_read(AXP192_IRQ_MASK_B, &val);
	if (val & IRQ_MASK_B_VBUS_VALID_3_8)
		return 0;

	axp192_read(AXP192_STATUS_B, &val);
	if (val & AXP192_STATUS_B_VBUS_V_3P8)
		return 1;
	else
#endif
		return 0;
}

static int is_axp192_srp_ready(void)
{
#if 0
	u8 val;

	axp192_read(AXP192_IRQ_MASK_B, &val);
	if (val & IRQ_MASK_B_SRP_READY_0_6)
		return 0;

	/* FIXME: When cable detached, the SESS Valid
	 * of STATUA B can not change to 0 immediately.
	 * That will cause potential problems.
	 */
	axp192_read(AXP192_STATUS_B, &val);
	if (val & AXP192_STATUS_B_SRP_READY)
		return 1;
	else
#endif
		return 0;
}
static int get_power_supply_module(int cmd)
{
	int command, power_module;

	if (cmd < VCC_CORE || cmd >= MAX_CMD) {
		printk(KERN_WARNING "input wrong command: %d\n", cmd);
		return -EINVAL;
	}
	command = axp192_power_module[cmd - VCC_CORE].command;
	if (command != cmd) {
		printk(KERN_WARNING "axp192_power_module[] is error: %d\n", cmd);
		return -EINVAL;
	}
	power_module = axp192_power_module[cmd - VCC_CORE].power_module;
	if (power_module == 0) {
		printk(KERN_WARNING "the command not supported: %d\n", cmd);
		return -EINVAL;
	}

	return power_module;
}

/* USB Device/OTG functions end here*/
static int get_axp192_voltage(int cmd, int *pmv)
{
	int power_module, status = 0;
	u8 val;

	*pmv = 0;

	power_module = get_power_supply_module(cmd);

	switch (power_module) {
		case BUCK1:
			status = axp192_read(AXP_BUCK1_VOL, &val);
			if (status)
				return status;

			val &= 0x7f;
			*pmv = val * AXP_VBUCK1_STEP + AXP_VBUCK1_BASE;
			break;
		case BUCK2:
			status = axp192_read(AXP_BUCK2_VOL, &val);
			if (status)
				return status;

			val &= 0x3f;
			*pmv = val * AXP_VBUCK2_STEP + AXP_VBUCK2_BASE;
			break;
		case BUCK3:
			status = axp192_read(AXP_BUCK3_VOL, &val);
			if (status)
				return status;

			val &= 0x7f;
			*pmv = val * AXP_VBUCK3_STEP + AXP_VBUCK3_BASE;
			break;
		case LDO1:
			*pmv = 3300;
			break;
		case LDO2:
			status = axp192_read(AXP_LDO23_VOL, &val);
			if (status)
				return status;

			val = (val >> 4) & 0x0f;
			*pmv = val * AXP_LDO2_STEP + AXP_LDO2_BASE;
			break;
		case LDO3:
			status = axp192_read(AXP_LDO23_VOL, &val);
			if (status)
				return status;

			val = val & 0x0f;
			*pmv = val * AXP_LDO3_STEP + AXP_LDO3_BASE;
			break;
		case USB_OTG: /* TODO */
		case LDO_GPADC: /* TODO */
		case LDO_AUDIO: /* TODO */
		case LDO_PMCORE: /* TODO */
		case LDO_BBAT: /* TODO */
			printk(KERN_WARNING "power supply module TODO: %d\n", cmd);
			return -EINVAL;
		default:
			printk(KERN_WARNING "input wrong command: %d\n", cmd);
			return -EINVAL;
	}
	return status;
}

static int set_axp192_voltage(int cmd, int mv)
{
	int power_module, status = 0;
	u8 val, temp;

	power_module = get_power_supply_module(cmd);

	switch (power_module) {
		case BUCK1:
			if ((mv < AXP_VBUCK1_BASE) || (mv > AXP_VBUCK1_MAX))
				return -EINVAL;
			val = (mv - AXP_VBUCK1_BASE) / AXP_VBUCK1_STEP;
			status = axp192_write(AXP_BUCK1_VOL, val&0x7f);
			break;
		case BUCK2:
			if ((mv < AXP_VBUCK2_BASE) || (mv > AXP_VBUCK2_MAX))
				return -EINVAL;
			val = (mv - AXP_VBUCK2_BASE) / AXP_VBUCK2_STEP;
			status = axp192_write(AXP_BUCK2_VOL, val&0x3f);
			break;
		case BUCK3:
			if ((mv < AXP_VBUCK3_BASE) || (mv > AXP_VBUCK3_MAX))
				return -EINVAL;
			val = (mv - AXP_VBUCK3_BASE) / AXP_VBUCK3_STEP;
			status = axp192_write(AXP_BUCK3_VOL, val&0x7f);
			break;
		case LDO1:
			return 0;
		case LDO2:
			if ((mv < AXP_LDO2_BASE) || mv > (AXP_LDO2_MAX))
				return -EINVAL;
			status = axp192_read(AXP_LDO23_VOL, &val);
			if (status)
				return status;

			val &= 0x0f;
			temp = ((mv - AXP_LDO2_BASE) / AXP_LDO2_STEP) & 0x0f;
			val = val | (temp<<4);
			status = axp192_write(AXP_LDO23_VOL, val);
			break;
		case LDO3:
			if ((mv < AXP_LDO3_BASE) || (mv > AXP_LDO3_MAX))
				return -EINVAL;

			status = axp192_read(AXP_LDO23_VOL, &val);
			if (status)
				return status;

			val &= 0xf0;
			temp = ((mv - AXP_LDO3_BASE) / AXP_LDO3_STEP) & 0x0f;
			val = val | (temp<<0);
			status = axp192_write(AXP_LDO23_VOL, val);
			break;
		case USB_OTG: /* TODO */
		case LDO_GPADC: /* TODO */
		case LDO_AUDIO: /* TODO */
		case LDO_PMCORE: /* TODO */
		case LDO_BBAT: /* TODO */
			printk(KERN_WARNING "power supply module TODO: %d\n", cmd);
			return -EINVAL;
		default:
			printk(KERN_INFO "error command\n");
			return -EINVAL;
	}

	if (status != 0)
		return status;
	return status;
}

static void axp192_worker(struct work_struct *work)
{
	unsigned int event;
	unsigned char i, states[4];

	while (event = axp192_event_change(states)) {
		printk(KERN_INFO "%s event 0x%x, states:0x%02x 0x%02x 0x%02x 0x%02x\n",
				__func__, event, states[0],states[1],states[2],states[3]);
		//pmic_event_handle(event);

		/* clean irq states register  */
		for (i=0; i<4; i++) {
			if (states[i]) {
				/*AXP_IRQ_STATE1 AXP_IRQ_STATE2 AXP_IRQ_STATE3 AXP_IRQ_STATE4*/
				axp192_write(AXP_IRQ_STATE1+i, states[i]);
			}
		}
	}
}

/*
 * Axp192 interrupt service routine.
 * In the ISR we need to check the Status bits in Axp192 and according
 * to those bits to check which kind of IRQ had happened.
 */
static irqreturn_t axp192_irq_handler(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct axp192_platform_data *pdata = client->dev.platform_data;

	/* clear the irq */
	pdata->ack_irq();

	schedule_work(&pdata->work);

	return IRQ_HANDLED;
}

#ifdef	CONFIG_PM
/*
 * Suspend the axp192 interface.
 * Add register save/restore here.
 */
static int axp192_suspend(struct i2c_client *client, pm_message_t state)
{

	disable_irq(client->irq);
#if 0
	unsigned char val;
	if ((get_pm_state() == PM_SUSPEND_MEM)) {
		axp192_read(AXP192_OVER1, &val);
		axp192_regs[AXP192_OVER1].data = val;
		axp192_read(AXP192_APP_OVER2, &val);
		axp192_regs[AXP192_APP_OVER2].data = val;
		axp192_read(AXP192_APP_OVER3, &val);
		axp192_regs[AXP192_APP_OVER3].data = val;
		axp192_read(AXP192_BUCK_SLEEP, &val);
		axp192_regs[AXP192_BUCK_SLEEP].data = val;
		axp192_read(AXP192_SYSCTRL_A, &val);
		axp192_regs[AXP192_SYSCTRL_A].data = val;

		axp192_write(AXP192_BUCK_SLEEP, 0x6D);
		axp192_write(AXP192_SYSCTRL_A, 0xff);
	}
#endif
	return 0;
}

/*
 * Resume the Axp192 interface.
 */
static int axp192_resume(struct i2c_client *client)
{
	int i;

	for (i = 0; i < AXP192_REG_NUM; i++)
		axp192_regs[i].hit = 0;

	enable_irq(client->irq);
	axp192_irq_handler(client->irq, client);
#if 0
	if ((get_pm_state() == PM_SUSPEND_MEM)) {
		axp192_write(AXP192_OVER1, axp192_regs[AXP192_OVER1].data);
		axp192_write(AXP192_APP_OVER2, axp192_regs[AXP192_APP_OVER2].data);
		axp192_write(AXP192_APP_OVER3, axp192_regs[AXP192_APP_OVER3].data);
		axp192_write(AXP192_BUCK_SLEEP, axp192_regs[AXP192_BUCK_SLEEP].data);
		axp192_write(AXP192_SYSCTRL_A, axp192_regs[AXP192_SYSCTRL_A].data);
	}
#endif
	return 0;
}

#else				/*  */
#define	axp192_suspend		NULL
#define	axp192_resume		NULL
#endif				/*  */

void axp192_shutdown(struct i2c_client *client)
{
}

static struct pmic_ops axp192_pmic_ops = {
	.get_voltage		= get_axp192_voltage,
	.set_voltage		= set_axp192_voltage,

	.is_vbus_assert		= is_axp192_vbus_assert,
	.is_avbusvld		= is_axp192_avbusvld,
	.is_asessvld		= is_axp192_asessvld,
	.is_bsessvld		= is_axp192_bsessvld,
	.is_srp_ready		= is_axp192_srp_ready,

	.set_pump		= axp192_set_pump,
	.set_vbus_supply	= axp192_set_vbus_supply,
	.set_usbotg_a_mask	= axp192_set_usbotg_a_mask,
	.set_usbotg_b_mask	= axp192_set_usbotg_b_mask,
};

#ifdef	CONFIG_PROC_FS
#define	AXP192_PROC_FILE	"driver/axp192"
static struct proc_dir_entry *axp192_proc_file;
static int index;

static ssize_t axp192_proc_read(struct file *filp,
		char *buffer, size_t length, loff_t *offset)
{
	u8 reg_val;

	if ((index < 0) || (index > AXP192_REG_NUM))
		return 0;

	axp192_read(index, &reg_val);
	printk(KERN_INFO "register 0x%x: 0x%x\n", index, reg_val);
	return 0;
}

static ssize_t axp192_proc_write(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	u8 reg_val;
	char messages[256], vol[256];

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if ('-' == messages[0]) {
		/* set the register index */
		memcpy(vol, messages+1, len-1);
		index = (int) simple_strtoul(vol, NULL, 16);
	} else {
		/* set the register value */
		reg_val = (int)simple_strtoul(messages, NULL, 16);
		axp192_write(index, reg_val & 0xFF);
	}

	return len;
}

static struct file_operations axp192_proc_ops = {
	.read = axp192_proc_read,
	.write = axp192_proc_write,
};

static void create_axp192_proc_file(void)
{
	printk("setup proc control interface\n");
	axp192_proc_file = create_proc_entry(AXP192_PROC_FILE, 0644, NULL);
	if (axp192_proc_file) {
		//axp192_proc_file->owner = THIS_MODULE;
		axp192_proc_file->proc_fops = &axp192_proc_ops;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}

static void remove_axp192_proc_file(void)
{
	remove_proc_entry(AXP192_PROC_FILE, NULL);
}
#endif

static int __devinit axp192_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct axp192_platform_data *pdata;
	int ret;
	unsigned int user_data;
	printk("---AXP192_Probe\n");

	ret = i2c_smbus_read_i2c_block_data(client, AXP_DATA0, 4, (u8*)&user_data);
	if (ret < 0) {
		printk(KERN_WARNING "axp192 unavailable!\n");
		axp192_client = NULL;
		return -ENXIO;
	} else {
		printk(KERN_INFO "axp192 detected(user data:0x%08x).\n", user_data);
	}

	axp192_client = client;

	ret = axp192_initchip();
	if (ret != 0)
		printk(KERN_WARNING "Initialize Axp192 failed with ret 0x%x\n",
				ret);

	//pdata = client->dev.platform_data;
	pdata= &axp192_data ;

	/* init spinlock */
	spin_lock_init(&pdata->lock);
	/* init workqueue */
	INIT_WORK(&pdata->work, axp192_worker);
	/* init irq */
	pdata->init_irq();
	ret = request_irq(INT_PWC, axp192_irq_handler, IRQF_TRIGGER_FALLING,
			"axp192", client);
	if (ret) {
		printk(KERN_WARNING "Request IRQ for Axp192 failed, return:%d\n", ret);
		//return ret;
	}

	pdata->platform_init();

	axp192_power_module = pdata->power_supply_modules;

#ifdef CONFIG_PROC_FS
	create_axp192_proc_file();
#endif

	/*Set core voltage (DCDC-2) to 1.125V*/
	axp192_write(0x23, 0x11);
	//pmic_set_ops(&axp192_pmic_ops);

	return 0;
}

static int axp192_remove(struct i2c_client *client)
{
	//pmic_set_ops(NULL);

#ifdef CONFIG_PROC_FS
	remove_axp192_proc_file();
#endif

	free_irq(client->irq, NULL);

	return 0;
}

static const struct i2c_device_id axp192_id[] = {
	{ "axp192", 0 },
	{ }
};

static struct i2c_driver axp192_driver = {
	.driver = {
		.name	= "axp192",
	},
	.probe		= axp192_probe,
	.remove		= axp192_remove,
	.id_table	= axp192_id,
	.suspend	= axp192_suspend,
	.resume		= axp192_resume,
	.shutdown 	= axp192_shutdown,
};

static int __init axp192_init(void)
{
	printk("---AXP192_init\n");
	return i2c_add_driver(&axp192_driver);
}

static void __exit axp192_exit(void)
{
	flush_scheduled_work();
	i2c_del_driver(&axp192_driver);
}

/*These function is make other drivers happy, will be replaced later*/
int pwc_write(unsigned short offset, unsigned int data, unsigned int mask)
{
	return 0;
}
EXPORT_SYMBOL(pwc_write);
int pwc_reg_read(unsigned char addr, unsigned char *data)
{
	return 0;
}
EXPORT_SYMBOL(pwc_reg_read);
int pwc_reg_write(unsigned char addr, unsigned char data)
{
	return 0;
}
EXPORT_SYMBOL(pwc_reg_write);
int pwc_read(unsigned short offset, unsigned int *data)
{
	return 0;
}
EXPORT_SYMBOL(pwc_read);
int pwc_set_direction(unsigned gpio, int is_input)
{
	return 0;
}
EXPORT_SYMBOL(pwc_set_direction);
int pwc_get_value(unsigned int gpio)
{
	return 0;
}
EXPORT_SYMBOL(pwc_get_value);
void pwc_set_value(unsigned int gpio, int value)
{
	return 0;
}
EXPORT_SYMBOL(pwc_set_value);





module_init(axp192_init);
module_exit(axp192_exit);

MODULE_DESCRIPTION("Axp192 Driver");
MODULE_LICENSE("GPL");

