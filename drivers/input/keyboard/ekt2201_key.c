/* drivers/input/touchscreen/pixcir_i2c_ts.c
 *
 * Copyright (C) 2010 Pixcir, Inc.
 *
 * pixcir_i2c_ts.c V1.0  support multi touch
 * pixcir_i2c_ts.c V2.0  add tuning function including follows function:
 *
 * CALIBRATION_FLAG	1
 * NORMAL_MODE		8
 * DEBUG_MODE		3
 * //INTERNAL_MODE	4
 * //RASTER_MODE	5
 * BOOTLOADER_MODE	7
 *  
 *
 * ekt2201_i2c_ts.c V2.3	update
 * Add m48 single solution
 * Add:
 * VERSION_FLAG		6
 * RD_EEPROM		12
 * WR_EEPROM		13						
 *					---11-04-13
 *
 * ekt2201_i2c_ts.c V2.3.1	update  client->adapter->nr -> 0
 *
 * ekt2201_i2c_ts.c V2.3.2        fix m48 show rawdata bug
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
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>
#include <linux/smp_lock.h>
#include <linux/delay.h>
#include <linux/slab.h>  //for mini6410 2.6.36 kree(),kmalloc()
#include <linux/proc_fs.h>

#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>

#include "ekt2201_key.h"

#define DRIVER_VERSION "v2.3.2"
#define DRIVER_AUTHOR "Bee<http://www.ekt2201.com.cn>"
#define DRIVER_DESC "Pixcir I2C Touchscreen Driver with tune fuction"
#define DRIVER_LICENSE "GPL"

#define	PIXCIR_DEBUG		1
#if 1
#define M48

#ifdef	M48
#define	MAXX	48
#define	MAXY	48
#else
#define	MAXX	32
#define	MAXY	32
#endif

#ifdef R8C_AUO_I2C
#ifndef R8C_3GA_2TG
#define R8C_3GA_2TG
#endif
#endif

/*****************touchscreen resolution setting*************/
#define TOUCHSCREEN_MINX 0
#define TOUCHSCREEN_MAXX 1024
#define TOUCHSCREEN_MINY 0
#define TOUCHSCREEN_MAXY 768


/*****************board setting and test passed*************/
#define	EMEV

#ifdef S5PC1XX
#include <plat/gpio-bank-e1.h> //reset pin GPE1_5
#include <plat/gpio-bank-h1.h> //attb pin GPH1_3
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>

#define ATTB		S5PC1XX_GPH1(3)
#define get_attb_value	gpio_get_value
#define	RESETPIN_CFG	s3c_gpio_cfgpin(S5PC1XX_GPE1(5),S3C_GPIO_OUTPUT)
#define	RESETPIN_SET0 	gpio_direction_output(S5PC1XX_GPE1(5),0)
#define	RESETPIN_SET1	gpio_direction_output(S5PC1XX_GPE1(5),1)

#endif	//mini6410
#ifdef MINI6410

#include <plat/gpio-cfg.h>
#include <mach/gpio-bank-e.h>
#include <mach/gpio-bank-n.h>
#include <mach/gpio.h>

#define ATTB		S3C64XX_GPN(11)
#define get_attb_value	gpio_get_value
#define	RESETPIN_CFG	s3c_gpio_cfgpin(S3C64XX_GPE(1),S3C_GPIO_OUTPUT)
#define	RESETPIN_SET0 	gpio_direction_output(S3C64XX_GPE(1),0)
#define	RESETPIN_SET1	gpio_direction_output(S3C64XX_GPE(1),1)
#endif
#ifdef EMEV
#include <mach/gpio.h>
#include <mach/smu.h>
#define ATTB		(29)
#define get_attb_value	gpio_get_value
#define	RESETPIN_CFG	emxx_setintpin()
#define	RESETPIN_SET0 	gpio_direction_output(97,0)
#define	RESETPIN_SET1	gpio_direction_output(97,1)
#endif
#endif

/*********************************V2.0-Bee-0928-TOP****************************************/

#define	SLAVE_ADDR		0x10
#define	BOOTLOADER_ADDR		0x5d

#ifndef	I2C_MAJOR
#define	I2C_MAJOR 		125
#endif

#define	I2C_MINORS 		256

#define	CALIBRATION_FLAG	1
#define	NORMAL_MODE		8
#define	PIXCIR_DEBUG_MODE	3
//#define  INTERNAL_MODE	4
//#define  RASTER_MODE		5
#define	VERSION_FLAG		6
#define	BOOTLOADER_MODE		7
#define	RD_EEPROM		12
#define	WR_EEPROM		13

#define  ENABLE_IRQ		10
#define  DISABLE_IRQ		11

#ifdef R8C_AUO_I2C	
#define SPECOP		0x78
#else
#define SPECOP		0x37
#endif

#define reset

static int global_irq;

static unsigned char status_reg = 0;
static unsigned char read_XN_YN_flag = 0;

static unsigned char global_touching, global_oldtouching;
static unsigned char global_posx1_low, global_posx1_high, global_posy1_low,
		     global_posy1_high, global_posx2_low, global_posx2_high,
		     global_posy2_low, global_posy2_high;

static unsigned char Tango_number;

static unsigned char x_nb_electrodes = 0;
static unsigned char y_nb_electrodes = 0;
static unsigned char x2_nb_electrodes = 0;
static unsigned char x1_x2_nb_electrodes = 0;

static signed char xy_raw1[(MAXX*2+3)];
static signed char xy_raw2[MAXX*2];
static signed char xy_raw12[(MAXX*4+3)];

static unsigned char data2eep[3],op2eep[2];

struct i2c_dev
{
	struct list_head list;
	struct i2c_adapter *adap;
	struct device *dev;
};

static struct i2c_driver ekt2201_i2c_ts_driver;

static struct class *i2c_dev_class;
static LIST_HEAD( i2c_dev_list);
static DEFINE_SPINLOCK( i2c_dev_list_lock);

/*static int i2cdev_check(struct device *dev, void *addrp)
  {
  struct i2c_client *client = i2c_verify_client(dev);

  if (!client || client->addr != *(unsigned int *)addrp)
  return 0;

  return dev->driver ? -EBUSY : 0;
  }

  static int i2cdev_check_addr(struct i2c_adapter *adapter,unsigned int addr)
  {
  return device_for_each_child(&adapter->dev,&addr,i2cdev_check);
  }*/
//#define TS_POLL_DELAY	(40 * 1000*1000)	/* ns delay before the first sample */
#define TS_POLL_DELAY	(80 * 1000*1000)	/* ns delay before the first sample */

static void emxx_setintpin(void)
{
	writel(readl(CHG_PINSEL_G096)|0x02, CHG_PINSEL_G096);
}
static void return_i2c_dev(struct i2c_dev *i2c_dev)
{
	spin_lock(&i2c_dev_list_lock);
	list_del(&i2c_dev->list);
	spin_unlock(&i2c_dev_list_lock);
	kfree(i2c_dev);
}

static struct i2c_dev *i2c_dev_get_by_minor(unsigned index)
{
	struct i2c_dev *i2c_dev;
	i2c_dev = NULL;

	spin_lock(&i2c_dev_list_lock);
	list_for_each_entry(i2c_dev, &i2c_dev_list, list)
	{
		if (i2c_dev->adap->nr == index)
			goto found;
	}
	i2c_dev = NULL;
found: spin_unlock(&i2c_dev_list_lock);
       return i2c_dev;
}

static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap)
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS) {
		printk(KERN_ERR "i2c-dev: Out of device minors (%d)\n",
				adap->nr);
		return ERR_PTR(-ENODEV);
	}

	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return ERR_PTR(-ENOMEM);
	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_dev_list);
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}

static  void read_XN_YN_value(struct i2c_client *client)
{}

static void read_XY_tables(struct i2c_client *client, signed char *xy_raw1_buf,
		signed char *xy_raw2_buf)
{

}

#ifdef M48
static  void cross_disable (struct i2c_client *client)
{
	unsigned char wrbuf[1],ret;
	wrbuf[0] = 0;
	ret = i2c_master_send(client, wrbuf, 1);
	if(ret != 1) {
		printk("set cross_disable error\n");
	}
}
#endif
/*********************************V2.0-Bee-0928-BOTTOM****************************************/

static struct workqueue_struct *ekt2201_wq;

struct ekt2201_i2c_ts_data
{
	struct i2c_client *client;
	struct input_dev *input;
	//struct delayed_work work;
	struct work_struct work;
	struct delayed_work work1;
	struct hrtimer	timer1;	/*monitor interrupt pin*/
	struct hrtimer	timer2;	/*monitor interrupt pin*/
	int irq;
};
static int PrevTouchKeyCode=0;
static int PrevIOKeyCode=0;
/*
#define KEY_VOLUMEDOWN	114
#define KEY_VOLUMEUP		115
#define KEY_HOME			102
#define KEY_UP				103
#define KEY_LEFT				105
#define KEY_RIGHT			106
#define KEY_DOWN			108
#define KEY_KPENTER			96
#define KEY_POWER			116
#define KEY_ESC				158
#define KEY_BACK			1
#define KEY_MENU			59
#define KEY_Confirm			28
 */
static void ekt2201_ts_poscheck(struct work_struct *work)
{
	//struct ekt2201_i2c_ts_data *tsdata = container_of(work, struct ekt2201_i2c_ts_data, work.work);
	struct ekt2201_i2c_ts_data *tsdata = container_of(work, struct ekt2201_i2c_ts_data, work);

	unsigned char Rdbuf[10],Wrbuf[1];
	int ret,keyval=0,keycode=0;

	memset(Wrbuf, 0, sizeof(Wrbuf));
	memset(Rdbuf, 0, sizeof(Rdbuf));

	Wrbuf[0] = 0x11;
	ret = i2c_master_send(tsdata->client, Wrbuf, 1);
	ret = i2c_master_recv(tsdata->client, Rdbuf, 1);

	Wrbuf[0] = 0x12;
	ret = i2c_master_send(tsdata->client, Wrbuf, 1);
	ret = i2c_master_recv(tsdata->client, &Rdbuf[1], 1);
	keyval=Rdbuf[0]|(Rdbuf[1]<<8);
	//printk("read ret:%d, keyval=%d\n", ret, keyval);

	/*************
	  return  8
	  search 256
	  menu   1
	  home   2
	 **************/
	if(keyval==8) 		keycode=KEY_BACK;
	else if(keyval==256)  	keycode=KEY_SEARCH;
	else if(keyval==1)     	keycode=KEY_MENU;
	else if(keyval==2)     	keycode=KEY_HOME;
	else 			keycode=0;

	if(keycode != 0){ //\D3а\B4\BC\FC
		if(PrevTouchKeyCode == 0){ //3 first down
			PrevTouchKeyCode=keycode;
			input_report_key(tsdata->input,keycode,1);
			input_sync(tsdata->input);
			//printk("key  down:%d\n",keycode);
		}else{
			if(PrevTouchKeyCode == keycode){//3 press
				//printk("key  press:%d\n",keycode);
			}else if( PrevTouchKeyCode!=keycode){//3 old key up,new key down
				input_report_key(tsdata->input,PrevTouchKeyCode,0);
				input_sync(tsdata->input);
				//printk("keycode=%d, key  up:%d\n",keycode, PrevTouchKeyCode);
#if 0
				input_report_key(tsdata->input,keycode,1);
				input_sync(tsdata->input);
				PrevKeyCode=keycode;
				printk("key  dn:%d\n",keycode);
#else
				PrevTouchKeyCode=0;
				goto out;
#endif
			}
		}
	}else{//\CEް\B4\BC\FC
		if(PrevTouchKeyCode!=0){
			input_report_key(tsdata->input,PrevTouchKeyCode,0);//3 key up
			input_sync(tsdata->input);
			//printk("keycode=%d, key  up:%d\n", keycode, PrevTouchKeyCode);
		}
		PrevTouchKeyCode=0;
		goto out;
	}
	hrtimer_start(&tsdata->timer1,ktime_set(0,TS_POLL_DELAY),HRTIMER_MODE_REL);	
	return;
out:
	enable_irq(tsdata->irq);

}

static enum hrtimer_restart ekt2201_dotimer1(struct hrtimer *handle)
{
	struct ekt2201_i2c_ts_data *tsdata = container_of(handle, struct ekt2201_i2c_ts_data, timer1);

	//queue_work(ekt2201_wq, &tsdata->work.work);
	schedule_work(&tsdata->work);

	return HRTIMER_NORESTART;
}

static irqreturn_t ekt2201_ts_isr(int irq, void *dev_id)
{
	struct ekt2201_i2c_ts_data *tsdata = dev_id;

	if ((status_reg == 0) || (status_reg == NORMAL_MODE)) {
		disable_irq_nosync(irq);
		hrtimer_start(&tsdata->timer1,ktime_set(0,TS_POLL_DELAY),HRTIMER_MODE_REL);	
	}

	return IRQ_HANDLED;
}
static void ekt2201_iokey_check(struct work_struct *work)
{
	struct ekt2201_i2c_ts_data *tsdata = container_of(work,	struct ekt2201_i2c_ts_data,work1.work);
	int keycode=0,keyval=0;
	//	keyval=(readl(CHG_PINSEL_G000) & 0x6000) >>13;

	keyval=gpio_get_value(GPIO_P13)|(gpio_get_value(GPIO_P14)<<1);

	if(keyval == 3) keyval=0;
	else if (keyval == 0) keyval=1;  //3 two keys dn


	if(keyval == 1){
		keycode=KEY_VOLUMEDOWN;
	}else if(keyval == 2){
		keycode=KEY_VOLUMEUP;
	}
	printk("IO val=%d,prv=%d\n",keycode,PrevIOKeyCode);
	if(keycode != 0){
		if(PrevIOKeyCode==0){
			PrevIOKeyCode=keycode;
			input_report_key(tsdata->input,keycode,1);//3 key dn
			input_sync(tsdata->input);
			printk("iokey  dn:%d\n",PrevIOKeyCode);
		}else{
			if(PrevIOKeyCode==keycode){
				printk("iokey  press:%d\n",keycode);
			}else if(PrevIOKeyCode!=keycode){
				input_report_key(tsdata->input,PrevIOKeyCode,0);
				input_sync(tsdata->input);
				printk("iokey  up1:%d\n",PrevIOKeyCode);
				PrevIOKeyCode=0;
				goto out;
			}
		}
	}else{
		if(PrevIOKeyCode != 0){
			input_report_key(tsdata->input,PrevIOKeyCode,0);//3 key up
			input_sync(tsdata->input);
			printk("iokey  up2:%d\n",PrevIOKeyCode);
			PrevIOKeyCode=0;
		}
		goto out;
	}
	hrtimer_start(&tsdata->timer2,ktime_set(0,TS_POLL_DELAY),HRTIMER_MODE_REL);	
	return;
out:
	enable_irq(INT_GPIO_13);
	enable_irq(INT_GPIO_14);
}
static enum hrtimer_restart ekt2201_keytimer(struct hrtimer *handle)
{
	struct ekt2201_i2c_ts_data *tsdata = container_of(handle, struct ekt2201_i2c_ts_data, timer2);

	queue_work(ekt2201_wq, &tsdata->work1.work);

	return HRTIMER_NORESTART;
}

static irqreturn_t ekt2201_isr13(int irq, void *dev_id)
{
	struct ekt2201_i2c_ts_data *tsdata = dev_id;

	disable_irq_nosync(INT_GPIO_13);
	disable_irq_nosync(INT_GPIO_14);
	hrtimer_start(&tsdata->timer2,ktime_set(0,TS_POLL_DELAY),HRTIMER_MODE_REL);	
	return IRQ_HANDLED;
}
static irqreturn_t ekt2201_isr14(int irq, void *dev_id)
{
	struct ekt2201_i2c_ts_data *tsdata = dev_id;

	disable_irq_nosync(INT_GPIO_13);
	disable_irq_nosync(INT_GPIO_14);
	hrtimer_start(&tsdata->timer2,ktime_set(0,TS_POLL_DELAY),HRTIMER_MODE_REL);	
	return IRQ_HANDLED;
}

static int ekt2201_ts_open(struct input_dev *dev)
{
	return 0;
}

static void ekt2201_ts_close(struct input_dev *dev)
{
}

static ssize_t ekt2201_proc_read(struct file *filp,
		char *buffer, size_t length, loff_t *offset)
{
	return 0;
}
static ssize_t ekt2201_proc_write(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[256];

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	messages[5]=0;
	if ( strcmp(messages, "start") ==0)
	{
		printk("Start to calibrate ...\n");
	}

	return len;
}
static struct proc_dir_entry *ekt2201_proc_file;
static struct file_operations ekt2201_proc_ops = {
	.read = ekt2201_proc_read,
	.write = ekt2201_proc_write,
};

static void create_ekt2201_proc_files(void)
{
	ekt2201_proc_file = create_proc_entry("driver/ekt2201", 0644, NULL);
	if (ekt2201_proc_file) {
		//ekt2201_proc_file->owner = THIS_MODULE;
		ekt2201_proc_file->proc_fops = &ekt2201_proc_ops;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}
static void remove_ekt2201_proc_file(void)
{
	remove_proc_entry("driver/ekt2201", NULL);
}

static int ekt2201_i2c_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct ekt2201_i2c_ts_data *tsdata;
	struct input_dev *input;
	struct device *dev;
	struct i2c_dev *i2c_dev;
	int error;

	printk("ekt2201_i2c_ts_probe\n");

	tsdata = kzalloc(sizeof(*tsdata), GFP_KERNEL);
	if (!tsdata) {
		dev_err(&client->dev, "failed to allocate driver data!\n");
		error = -ENOMEM;
		dev_set_drvdata(&client->dev, NULL);
		return error;
	}

	dev_set_drvdata(&client->dev, tsdata);

	input = input_allocate_device();
	if (!input) {
		dev_err(&client->dev, "failed to allocate input device!\n");
		error = -ENOMEM;
		input_free_device(input);
		kfree(tsdata);
	}


	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
	input->open = ekt2201_ts_open;
	input->close = ekt2201_ts_close;

	input_set_capability(input, EV_KEY, KEY_HOME);
	input_set_capability(input, EV_KEY, KEY_MENU);
	input_set_capability(input, EV_KEY, KEY_SEARCH);
	input_set_capability(input, EV_KEY, KEY_BACK);
	input_set_capability(input, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(input, EV_KEY, KEY_VOLUMEDOWN);

	input_set_drvdata(input, tsdata);

	tsdata->client = client;
	tsdata->input = input;

	//INIT_WORK(&tsdata->work.work, ekt2201_ts_poscheck);
	INIT_WORK(&tsdata->work, ekt2201_ts_poscheck);
	INIT_WORK(&tsdata->work1.work, ekt2201_iokey_check);
	hrtimer_init(&tsdata->timer1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	tsdata->timer1.function = ekt2201_dotimer1;
	hrtimer_init(&tsdata->timer2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	tsdata->timer2.function = ekt2201_keytimer;

	tsdata->irq = client->irq;
	global_irq = client->irq;

	if (input_register_device(input)) {
		input_free_device(input);
		kfree(tsdata);
	}

	writel(readl(CHG_PULL11)|0xC00, CHG_PULL11);
	writel(readl(CHG_PINSEL_G096)|0x40, CHG_PINSEL_G096);
	gpio_direction_input(tsdata->irq);

	if (request_irq(tsdata->irq, ekt2201_ts_isr, IRQF_TRIGGER_FALLING,client->name, tsdata)) {
		printk( "Unable to request touch key IRQ.\n");
		input_unregister_device(input);
		input = NULL;
	}

	device_init_wakeup(&client->dev, 1);

	/*********************************V2.0-Bee-0928-TOP****************************************/
	i2c_dev = get_free_i2c_dev(client->adapter);
	if (IS_ERR(i2c_dev)) {
		error = PTR_ERR(i2c_dev);
		return error;
	}
	if(0){
		int ret,add;
		unsigned char Rdbuf[9],Wrbuf[1];
		memset(Wrbuf, 0, sizeof(Wrbuf));
		memset(Rdbuf, 0, sizeof(Rdbuf));

		for(add=1;add<=63;add++){	
			tsdata->client->addr = add;

			Wrbuf[0]=0;
			ret = i2c_master_send(tsdata->client, Wrbuf, 1);

			printk("master write add(0x%x) ret:%d\n",add,ret);

			ret = i2c_master_recv(tsdata->client, Rdbuf, 1);
			printk("master read ret:%d  (0x%x)=0x%x\n",ret,Wrbuf[0],Rdbuf[0]);

		}
	}


	{
		int ret;
		unsigned char Rdbuf[9],Wrbuf[1];
		memset(Wrbuf, 0, sizeof(Wrbuf));
		memset(Rdbuf, 0, sizeof(Rdbuf));
		Wrbuf[0]=0x4;
		Wrbuf[1]=0x41;
		ret = i2c_master_send(tsdata->client, Wrbuf, 2);
		//printk("master write ret:%d\n",ret);

		Wrbuf[0]=4;
		ret = i2c_master_send(tsdata->client, Wrbuf, 1);

		//printk("master write ret:%d\n",ret);

		if (ret != 1) {
			dev_err(&tsdata->client->dev, "ekt2201 error!\n");
		}
		ret = i2c_master_recv(tsdata->client, Rdbuf, 1);
		//printk("master read ret:%x  (0x%x)=0x%x\n",ret,Wrbuf[0],Rdbuf[0]);

	}
	dev = device_create(i2c_dev_class, &client->adapter->dev, MKDEV(I2C_MAJOR,
				client->adapter->nr), NULL, "ekt2201_i2c_ts%d", 0);
	if (IS_ERR(dev)) {
		error = PTR_ERR(dev);
		return error;
	}
	/*********************************V2.0-Bee-0928-BOTTOM****************************************/
	dev_err(&tsdata->client->dev, "insmod successfully!\n");
	hrtimer_start(&tsdata->timer2,ktime_set(0,TS_POLL_DELAY),HRTIMER_MODE_REL);	

	return 0;

}

static int ekt2201_i2c_ts_remove(struct i2c_client *client)
{
	int error;
	struct i2c_dev *i2c_dev;
	struct ekt2201_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
	free_irq(tsdata->irq, tsdata);
	/*********************************V2.0-Bee-0928-TOP****************************************/
	i2c_dev = get_free_i2c_dev(client->adapter);
	if (IS_ERR(i2c_dev)) {
		error = PTR_ERR(i2c_dev);
		return error;
	}
	return_i2c_dev(i2c_dev);
	device_destroy(i2c_dev_class, MKDEV(I2C_MAJOR, client->adapter->nr));
	/*********************************V2.0-Bee-0928-BOTTOM****************************************/
	input_unregister_device(tsdata->input);
	kfree(tsdata);
	dev_set_drvdata(&client->dev, NULL);
	return 0;
}

static int ekt2201_i2c_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ekt2201_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(tsdata->irq);

	return 0;
}

static int ekt2201_i2c_ts_resume(struct i2c_client *client)
{
	struct ekt2201_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(tsdata->irq);

	return 0;
}

/*********************************V2.0-Bee-0928****************************************/
/*                        	  ekt2201_open                                         */
/*********************************V2.0-Bee-0928****************************************/
static int ekt2201_open(struct inode *inode, struct file *file)
{
	int subminor;
	struct i2c_client *client;
	struct i2c_adapter *adapter;
	struct i2c_dev *i2c_dev;
	int ret = 0;
#if PIXCIR_DEBUG
	printk("enter ekt2201_open function\n");
#endif
	subminor = iminor(inode);
#if PIXCIR_DEBUG
	printk("subminor=%d\n",subminor);
#endif
	lock_kernel();
	i2c_dev = i2c_dev_get_by_minor(subminor);
	if (!i2c_dev) {
		printk("error i2c_dev\n");
		return -ENODEV;
	}

	adapter = i2c_get_adapter(i2c_dev->adap->nr);
	if (!adapter) {
		return -ENODEV;
	}

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client) {
		i2c_put_adapter(adapter);
		ret = -ENOMEM;
	}
	snprintf(client->name, I2C_NAME_SIZE, "ekt2201_i2c_ts%d", adapter->nr);
	client->driver = &ekt2201_i2c_ts_driver;
	client->adapter = adapter;
	//if(i2cdev_check_addr(client->adapter,0x5c))
	//	return -EBUSY;
	file->private_data = client;

	return 0;
}

/*********************************V2.0-Bee-0928****************************************/
/*                        	  ekt2201_ioctl                                        */
/*********************************V2.0-Bee-0928****************************************/
static long ekt2201_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	//struct i2c_client *client = (struct i2c_client *) file->private_data;

	return 0;
}

/*********************************V2.0-Bee-0928****************************************/
/*                        	  ekt2201_read                                         */
/*********************************V2.0-Bee-0928****************************************/
static ssize_t ekt2201_read (struct file *file, char __user *buf, size_t count,loff_t *offset)
{
	//struct i2c_client *client = (struct i2c_client *)file->private_data;
	int ret=0;

	return ret;
}

/*********************************V2.0-Bee-0928****************************************/
/*                        	  ekt2201_write                                        */
/*********************************V2.0-Bee-0928****************************************/
static ssize_t ekt2201_write(struct file *file, const char __user *buf,size_t count, loff_t *ppos)
{
	//struct i2c_client *client = (struct i2c_client *)file->private_data;
	int ret=0;

	return ret;
}

/*********************************V2.0-Bee-0928****************************************/
/*                        	  ekt2201_release                                         */
/*********************************V2.0-Bee-0928****************************************/
static int ekt2201_release(struct inode *inode, struct file *file)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;

#if PIXCIR_DEBUG
	printk("enter ekt2201_release funtion\n");
#endif
	i2c_put_adapter(client->adapter);
	kfree(client);
	file->private_data = NULL;

	return 0;
}

/*********************************V2.0-Bee-0928-TOP****************************************/
static const struct file_operations ekt2201_i2c_ts_fops =
{
	.owner = THIS_MODULE, 
	.read = ekt2201_read,
	.write = ekt2201_write,
	.open = ekt2201_open,
	.unlocked_ioctl = ekt2201_ioctl,
	.release = ekt2201_release, 
};
/*********************************V2.0-Bee-0928-BOTTOM****************************************/

static const struct i2c_device_id ekt2201_i2c_ts_id[] =
{
	{ "ekt2201", 0 },
	{ }
};
MODULE_DEVICE_TABLE( i2c, ekt2201_i2c_ts_id);

static struct i2c_driver ekt2201_i2c_ts_driver =
{ 	.driver =
	{
		.owner = THIS_MODULE,
		.name = "ekt2201 touch key",
	}, 
	.probe = ekt2201_i2c_ts_probe, 
	.remove = ekt2201_i2c_ts_remove,
	.suspend = ekt2201_i2c_ts_suspend, 
	.resume = ekt2201_i2c_ts_resume,
	.id_table = ekt2201_i2c_ts_id, 
};

static int __init ekt2201_i2c_ts_init(void)
{
	int ret;
	printk("ekt2201_init\n");
	ekt2201_wq = create_singlethread_workqueue("ekt2201_tk");
	if(!ekt2201_wq){
		printk(KERN_ERR "ekt2201_init:register chrdev failed\n");
		//	return -ENOMEM;
	}

#if 0	
	/*********************************V2.0-Bee-0928-TOP****************************************/
	ret = register_chrdev(I2C_MAJOR,"ekt2201_tk",&ekt2201_i2c_ts_fops);
	if(ret) {
		printk(KERN_ERR "ekt2201_init:register chrdev failed\n");
		return ret;
	}

	i2c_dev_class = class_create(THIS_MODULE, "ekt2201_tk");
	if (IS_ERR(i2c_dev_class)) {
		ret = PTR_ERR(i2c_dev_class);
		class_destroy(i2c_dev_class);
	}
	/********************************V2.0-Bee-0928-BOTTOM******************************************/
#endif

	ret = i2c_add_driver(&ekt2201_i2c_ts_driver);
	printk("ekt2201:i2c_add_driver %d \n",ret);

	return ret;
}

static void __exit ekt2201_i2c_ts_exit(void)
{
	i2c_del_driver(&ekt2201_i2c_ts_driver);
	/********************************V2.0-Bee-0928-TOP******************************************/
	class_destroy(i2c_dev_class);
	unregister_chrdev(I2C_MAJOR,"ekt2201_tk");
	/********************************V2.0-Bee-0928-BOTTOM******************************************/
	if(ekt2201_wq)
		destroy_workqueue(ekt2201_wq);
}

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);

module_init( ekt2201_i2c_ts_init);
module_exit( ekt2201_i2c_ts_exit);


