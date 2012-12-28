/* drivers/input/touchscreen/pixcir_i2c_ts.c
 *
 * Copyright (C) 2010 Pixcir, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <mach/smu.h>
#include "axp192.h"

#define DRIVER_AUTHOR "yb@livall.cn"
#define DRIVER_DESCRIPTION "AXP192 Driver"
#define DRIVER_LICENSE "GPL"

#define DBUG_ENABLE 0
#if DBUG_ENABLE
#define AXP192(x...) printk(KERN_INFO "[AXP192:] " x)
#else
#define AXP192(x...) do {} while (0)
#endif


struct mutex mutex_axp192;
#define Lock()          mutex_lock(&mutex_axp192)
#define UnLock()        mutex_unlock(&mutex_axp192)

static struct axp192_param *axp_param = NULL;
static struct workqueue_struct *axp192_wq;
static 	struct work_struct *hardwork;
extern void update_battery_info(void);
/*static unsigned char axp192_write(unsigned char reg,unsigned char buff){
	int ret;
	unsigned char cmd[2] = {reg,buff};
	ret = i2c_master_send(&axp_param->client,cmd,2);
	return (ret==2)?0:-1;
}

static unsigned char axp192_read(unsigned char reg){
	unsigned char buff;
	int ret;
	ret = i2c_master_send(&axp_param->client,&reg,1);
	if(ret < 0)
		return -1;
	ret = i2c_master_recv(&axp_param->client,&buff,1);
	if(ret < 0)
		return -1;
	return buff;
}*/

static unsigned char axp192_read(unsigned char reg)
{
    unsigned char value = 0;
	Lock();
    //if(axp192_i2c_client == NULL) return 0;
//      i2c_master_send(axp192_i2c_client, &reg, 1);
//      i2c_master_recv(axp192_i2c_client, &value, 1);

        i2c_master_send(&axp_param->client, &reg, 1);
        i2c_master_recv(&axp_param->client, &value, 1);
	UnLock();
    return value;
}

static int axp192_write(unsigned char reg, unsigned char value)
{
    int ret;
    unsigned char cmd[2] = {reg, value};
	Lock();
    //i2c_master_send(axp192_i2c_client, cmd, 2);
    ret = i2c_master_send(&axp_param->client, cmd, 2);
	UnLock();
    return (ret==2)?0:-1;
}

void axp192_Gsensor(int on)
{
    unsigned char reg90, reg91;
    reg90 = axp192_read(0x90);
    reg91 = axp192_read(0x91);
    if(on)
    {
        reg90 &= ~ (MASK_BIT(0)|MASK_BIT(1)|MASK_BIT(2));
        reg90 |= MASK_BIT(1);
        reg91 &= ~ (MASK_BIT(4)|MASK_BIT(5)|MASK_BIT(6)|MASK_BIT(7));
    }
    else
    {
        reg90 |= MASK_BIT(0)|MASK_BIT(1)|MASK_BIT(2);
    }
    axp192_write(0x90, reg90);
    axp192_write(0x91, reg91);
}
EXPORT_SYMBOL(axp192_Gsensor);

#if defined CONFIG_LIGHT_SENSORS
void axp192_LightSensor(int on)
{
    unsigned char reg93, reg83, reg85;
    reg93 = axp192_read(0x93);
    reg83 = axp192_read(0x83);
    reg85 = axp192_read(0x85);
    
    if(on){
    	reg93 &= ~0x07; //清空低三位
	reg93 |= 0x04;  //GPIO2[0 : 2] :100 表示adc输入  

	//enable GPIO2 function
	reg83 |=  MASK_BIT(1); 

	//set voltage range
	reg85 |= MASK_BIT(2);
    }else{
        reg93 |= 0x07;  //浮空GPIO2[0 : 2] :11X 

	//disable GPIO2 FUNCTION
	reg83 &= ~ MASK_BIT(1);

	//use default voltage range
	reg85 &= ~ MASK_BIT(2);
    }
    axp192_write(0x83, reg83);
    axp192_write(0x93, reg93);
    axp192_write(0x85, reg85);
    //printk("AXP192, reg93:0x%x, reg83:0x%x, reg85:0x%x\n", axp192_read(0x93), axp192_read(0x83), axp192_read(0x85));

}
EXPORT_SYMBOT(axp192_LightSensor);

unsigned int axp192_light_getADC(void)
{
   unsigned char reg68, reg69;
   reg68 = axp192_read(0x68); //高8位
   reg69 = axp192_read(0x69); //低4位
   return reg68<<4|reg69;
}
EXPORT_SYMBOL(axp192_light_getADC);
#endif

int axp192_set_charging(unsigned char vol,unsigned cur,unsigned char cur_percent){
	unsigned char value;
	AXP192("axp192_set_charging\n");
	value = axp192_read(AXP192_REG33);
	if(value < 0)
		return -1;
	value &= ~AXP192_Charging_MASK;
	value &= ~AXP192_mAIN_MASK;
	value &= ~AXP192_Percent_MASK;
	value |= vol | cur | cur_percent;
	value = axp192_write(AXP192_REG33,value);
	if(value < 0)
		return -1;
	return 0;
}
EXPORT_SYMBOL(axp192_set_charging);

void axp192_poweroff(void)
{
	unsigned char value;
	AXP192("axp192_poweroff\n");
	axp192_set_charging(AXP192_Charging_4200,AXP192_mAIN_1320,AXP192_Percent10);
	value = axp192_read(AXP192_REG32);
	value |= MASK_BIT(7);
	axp192_write(AXP192_REG32,value);
}
EXPORT_SYMBOL(axp192_poweroff);

int axp192_is_battery_in(void){
	unsigned char value; 
	AXP192("axp192_is_battery_in\n");
	value = axp192_read(AXP192_REG1);
	if(value < 0)
		return -1;
	return (value & BATTERY_IN_MASK)?1:0;
}
EXPORT_SYMBOL(axp192_is_battery_in);

int axp192_battery_current_direction(void){
	unsigned char value;
	AXP192("axp192_battery_current_direction\n");
	value = axp192_read(AXP192_REG0);
	if(value < 0)
		return -1;
	return (value & BATTERY_CURRENT_MASK)?1:0;
}
EXPORT_SYMBOL(axp192_battery_current_direction);

int axp192_battery_charging_status(void){
	unsigned char value;
	AXP192("axp192_battery_charging_status\n");
	value = axp192_read(AXP192_REG1);
	if(value < 0)
		return -1;
	return (value & CHARGING_STATUS_MASK)?1:0;
}
EXPORT_SYMBOL(axp192_battery_charging_status);

int axp192_ac_status(void){
	unsigned char value;
	AXP192("axp192_ac_status\n");
	value = axp192_read(AXP192_REG0);
	if(value < 0)
		return -1;
	return (value & AC_STATUS_MASK)?1:0;
}
EXPORT_SYMBOL(axp192_ac_status);

int axp192_usb_status(void){
	unsigned char value;
	AXP192("axp192_usb_status\n");
	value = axp192_read(AXP192_REG0);
	if(value < 0)
		return -1;
	return (value & USB_STATUS_MASK)?1:0;
}
EXPORT_SYMBOL(axp192_usb_status);

int set_poweroff_vol(unsigned char vol){
	unsigned char value;
	AXP192("poweroff_vol\n");
	value = axp192_read(AXP192_REG31);
	value &= ~POWEROFF_VOL_MASK;
	value |= vol;
	value = axp192_write(AXP192_REG31,value);
	if(value < 0)
		return -1;
	return 0;
}
EXPORT_SYMBOL(set_poweroff_vol);

int set_gps_vol(unsigned char vol)
{
	unsigned char value;
	AXP192("%s\n",__func__);
	value = axp192_read(AXP192_REG28);
	value &= ~GPS_VOL_MASK;
	value |= vol;
	value = axp192_write(AXP192_REG28,value);
	if(value < 0)
		return -1;
	return 0;	
}
EXPORT_SYMBOL(set_gps_vol);

/*   parm: flag == 1 is POWER ON   */
int gps_power_control(unsigned char flag)
{
	unsigned char value;
	AXP192("%s\n",__func__);
	value = axp192_read(AXP192_REG12);
	value &= ~GPS_POWER_MASK;
	if(flag)
		value |= GPS_POWER_ON;
	else
		value |= GPS_POWER_OFF;
	value = axp192_write(AXP192_REG12,value);
	if(value < 0)
		return -1;
	return 0;
}
EXPORT_SYMBOL(gps_power_control);
static int axp192_battery_adc_enable(unsigned char mode){
	unsigned char value;
	AXP192("axp192_battery_adc_enable\n");
	value = axp192_read(AXP192_REG82);
	value &= ~BATTERY_ADC_MASK;
	value |= mode;
	value = axp192_write(AXP192_REG82,value);
	if(value < 0)
		return -1;
	return 0;
}

int axp192_battery_vol(void){
	int vol,buff;
	AXP192("axp192_battery_vol\n");
	vol = axp192_read(AXP192_REG78);
	vol = (vol << 4);
	buff = axp192_read(AXP192_REG79);
	vol += (buff & 0x0f);
	vol = vol * 1100 / 1000;
	AXP192("AXP192_REG78&79 = %d\n",vol);
	return vol;
}
EXPORT_SYMBOL(axp192_battery_vol);

int axp192_battery_discharging_cur(void){
	int cur;
	AXP192("axp192_battery_cur\n");
	cur = axp192_read(AXP192_REG7C);
	cur = (cur << 5);
	cur += axp192_read(AXP192_REG7D);
	cur = cur / 2;
	AXP192("AXP192_REG7C&7D = %d\n",cur);
	return cur;
}
EXPORT_SYMBOL(axp192_battery_discharging_cur);

int axp192_battery_charging_cur(void){
	int cur;
	AXP192("axp192_battery_cur\n");
	cur = axp192_read(AXP192_REG7A);
	cur = (cur << 5);
	cur += axp192_read(AXP192_REG7B);
	cur = cur / 2;
	AXP192("AXP192_REG7A&7B = %d\n",cur);
	return cur;
}
EXPORT_SYMBOL(axp192_battery_charging_cur);

static int axp192_ac_adc_enable(unsigned char mode){
	unsigned char value;
	AXP192("axp192_ac_adc_enable\n");
	value = axp192_read(AXP192_REG82);
	AXP192("old AXP192_REG82 = %d\n",value);
	value &= ~AC_ADC_MASK;
	value |= mode;
	value = axp192_write(AXP192_REG82,value);
	AXP192("new AXP192_REG82 = %d\n",axp192_read(AXP192_REG82));
	if(value < 0)
		return -1;
	return 0;
}

int	axp192_ac_vol(void){
	int vol;
	AXP192("axp192_ac_vol\n");
	vol = axp192_read(AXP192_REG56);
	vol = (vol << 4);
	vol += axp192_read(AXP192_REG57);
	AXP192("AXP192_REG56&57 = %d\n",vol);
	return vol;
} 
EXPORT_SYMBOL(axp192_ac_vol);

int axp192_ac_cur(void){
	int cur;
	AXP192("axp192_ac_cul\n");
	cur = axp192_read(AXP192_REG58);
	cur = (cur << 4);
	cur += axp192_read(AXP192_REG59);
	cur = cur / 2; 
	AXP192("AXP192_REG58&59 = %d\n",cur);
	return cur;
}
EXPORT_SYMBOL(axp192_ac_cur);

int set_cpucore_vol(unsigned char vol)
{
	unsigned char value;
	AXP192("%s\n",__func__);
	value = axp192_read(AXP192_REG23);
	value &= ~CPUCORE_VOL_MASK;
	value |= vol;
	value = axp192_write(AXP192_REG23,value);
	if(value < 0)
		return -1;
	return 0;
}
EXPORT_SYMBOL(set_cpucore_vol);

int set_pll_vol(void)
{
	unsigned char value;
	AXP192("%s\n",__func__);
	value = axp192_write(AXP192_REG29,0x10);
	if(value < 0)
		return -1;
	return 0;
}

/*
mode == 1 is PWM mode
mode == 0 is PWM or PFM
*/
int Change_DCDC_mode(int mode)
{
	int ret;
	AXP192("%s\n",__func__);
	if(mode)
		ret = axp192_write(AXP192_REG80,0xff);
	else 
		ret = axp192_write(AXP192_REG80,0x00);
	if(ret < 0)
		return -1;
	return 0;
}
//flag == 1 wifi power on
//flag == 0 wifi power off
void Wifi_Power(int flag)
{
	AXP192("%s\n",__func__);
	if(flag == 1)
		gpio_set_value(GPIO_P115,1);//WIFI POWER ON
	if(flag == 0)
		gpio_set_value(GPIO_P115,0);//WIFI POWER OFF
}
EXPORT_SYMBOL(Wifi_Power);

//flag == 1 system voltage 3.3V
//flag == 0 system voltage 3.0V
void Change_System_vol(int flag)
{
	AXP192("%s\n",__func__);
	if(flag == 1)
		axp192_write(0x26,0x68);//DC-DC1 VOLTAGE 3.3V
	if(flag == 0)
		axp192_write(0x26,0x60);//DC-DC1 VOLTAGE 3.1V
}
EXPORT_SYMBOL(Change_System_vol);

static int axp192_irq_workqueue(int irq){
	char REG44_value,REG45_value,REG46_value,REG47_value;
	AXP192("axp192_irq_workqueue\n");
	REG44_value = axp192_read(AXP192_REG44);
	REG45_value = axp192_read(AXP192_REG45);
	REG46_value = axp192_read(AXP192_REG46);
	REG47_value = axp192_read(AXP192_REG47);

	axp192_write(AXP192_REG44,REG44_value);
	axp192_write(AXP192_REG45,REG45_value);
	axp192_write(AXP192_REG46,REG46_value);
	axp192_write(AXP192_REG47,REG47_value);

	AXP192("REG44_value = %d,REG45_value = %d,REG46_value = %d,REG47_value = %d\n",REG44_value,REG45_value,REG46_value,REG47_value);
	if((REG44_value & AC_CHANGE_MASK) || (REG44_value & USB_CHANGE_MASK) || (REG45_value & BATT_CHANGE_MASK))
		update_battery_info();
	if(axp192_usb_status() && (gpio_get_value(GPIO_P101) == 1))
		axp192_write(AXP192_REG30,(axp192_read(AXP192_REG30) | 0x80));//USB Host input Charging
	else 
		axp192_write(AXP192_REG30,(axp192_read(AXP192_REG30) & 0x7f));//USB Device input not Charging
	enable_irq(axp_param->client.irq);
	return 0;
	
}

static irqreturn_t axp192_handler_irq(int irq,void *dev_id){
	AXP192("handler_irq\n");
	disable_irq_nosync(irq);
	queue_work(axp192_wq,hardwork);
	return 0;
}

static int axp192_irq_init(struct i2c_client *client){
	int ret;
	AXP192("axp192_irq_init\n");
	axp192_write(AXP192_REG40,IRQ40H_ALL);
	axp192_write(AXP192_REG41,IRQ41H_ALL);
	axp192_write(AXP192_REG42,IRQ42H_ALL);
	axp192_write(AXP192_REG43,IRQ43H_ALL);

	gpio_direction_input(GPIO_P0);
	writel(readl(CHG_PINSEL_G000) | (0x1 << 0),CHG_PINSEL_G000);
	writel(readl(CHG_PINSEL_G096) | (0x1 << 5),CHG_PINSEL_G096);//set GPIO_P101 I/O mode
	writel(readl(CHG_PULL11) | 0x50,CHG_PULL11);//GPIO_P101 pull up 
	ret = request_irq(client->irq,axp192_handler_irq,IRQF_TRIGGER_LOW | IRQF_DISABLED,client->name,client);
	if(ret < 0){
		printk(KERN_ERR "unable to request_irq ret = %d,client->name = %s,client->irq = %d\n",ret,client->name,client->irq);
	}
	return 0;
}

static int axp192_register_init(struct i2c_client *client){
	int ret;
	AXP192("axp192_register_init\n");
	//设置充电电压时4.2V,充电电流880mA,充电结束电流为设置电流百分比为10%
	ret = axp192_set_charging(AXP192_Charging_4200,AXP192_mAIN_880,AXP192_Percent10);
	if(ret < 0){
		AXP192("set charging failed\n");
		return -1;
	}
	
	ret = axp192_battery_adc_enable(BATTERY_VOL_ADC_ON | BATTERY_CUR_ADC_ON);
	if(ret < 0){
		AXP192("battery adc enable failed\n");
		return -1;		
	}

	ret = set_poweroff_vol(POWEROFF_2800);
	if(ret < 0){
		AXP192("set poweroff is not ok\n");
		return -1;
	}

	ret = axp192_ac_adc_enable(AC_VOL_ADC_ON | AC_CUR_ADC_ON);
	if(ret < 0){
		AXP192("ac adc enable failed\n");
		return -1;
	}

	ret = set_gps_vol(GPS_VOL_3300);
	if(ret < 0){
		AXP192("set gps voltage failed\n");
		return -1;
	}
	return 0;
}

static int __devinit axp192_probe(struct i2c_client *client,const struct i2c_device_id *id){
	AXP192("axp192_probe\n");
	hardwork = kmalloc(sizeof(struct work_struct),GFP_KERNEL);
	if(hardwork == NULL)
		goto out;
	INIT_WORK(hardwork,axp192_irq_workqueue);
	axp_param = kzalloc(sizeof(*axp_param), GFP_KERNEL);
	if(axp_param == NULL){
		goto out1;
	}
	axp_param->client = *client;
	mutex_init(&mutex_axp192);
	axp192_irq_init(client);
	axp192_register_init(client);
	if(axp192_usb_status() && (gpio_get_value(GPIO_P101) == 1))
		axp192_write(AXP192_REG30,(axp192_read(AXP192_REG30) | 0x80));//USB Host input Charging
	device_init_wakeup(&client->dev, 0);
	axp192_Gsensor(1);
	return 0;
out:
	kfree(hardwork);
	return -1;
out1:
	kfree(axp_param);
	return -1;
	
}

static int __devinit axp192_remove(struct i2c_client *client){
	AXP192("axp192_remove\n");
	kfree(hardwork);
	kfree(axp_param);
	device_init_wakeup(&client->dev, 0);
	return 0;
}

static int axp192_suspend(struct i2c_client *client){
	AXP192("axp192_suspend\n");
	axp192_write(AXP192_REG40,0x00);
	axp192_write(AXP192_REG41,0x00);
	axp192_write(AXP192_REG42,0x00);
	axp192_write(AXP192_REG43,0x00);
	axp192_set_charging(AXP192_Charging_4200,AXP192_mAIN_1320,AXP192_Percent10);
	disable_irq_nosync(client->irq);
	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);
	//set_cpucore_vol(CPUCORE_VOL_1100);//CPU CORE VOLTAGE 1.1V
	return 0;
}

static int axp192_resume(struct i2c_client *client){
	AXP192("axp192_resume\n");
	axp192_write(AXP192_REG40,IRQ40H_ALL);
	axp192_write(AXP192_REG41,IRQ41H_ALL);
	axp192_write(AXP192_REG42,IRQ42H_ALL);
	axp192_write(AXP192_REG43,IRQ43H_ALL);
	axp192_set_charging(AXP192_Charging_4200,AXP192_mAIN_880,AXP192_Percent10);
	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);
	enable_irq(client->irq);
	return 0;
}

static const struct i2c_device_id axp192_i2c_id[] = {
		{"axp192",0},
		{}
};
MODULE_DEVICE_TABLE(i2c, axp192_i2c_id);

static struct i2c_driver axp192_driver = {
	.driver = {
		.name = "axp192",
		.owner = THIS_MODULE,
	},
	.probe = axp192_probe,
	.remove = axp192_remove,
	.suspend = axp192_suspend,
	.resume = axp192_resume,
	.id_table = axp192_i2c_id,
};


static int __init axp192_init(void){
	AXP192("axp192_init\n");
	axp192_wq = create_singlethread_workqueue("axp192");
	if(axp192_wq == NULL)
		return -ENOMEM;
	return i2c_add_driver(&axp192_driver);
}

static void __exit axp192_exit(void){
	i2c_del_driver(&axp192_driver);
	AXP192("axp192_exit\n");
	destroy_workqueue(axp192_wq);
}

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_LICENSE(DRIVER_LICENSE);
module_init(axp192_init);
module_exit(axp192_exit);
