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
 * INTERNAL_MODE	4
 * //RASTER_MODE	5
 * BOOTLOADER_MODE	7
 *  
 *
 * pixcir_i2c_ts.c V2.3	update
 * Add m48 single solution
 * Add:
 * VERSION_FLAG		6
 * RD_EEPROM		12
 * WR_EEPROM		13						
 *					---11-04-13
 *
 * pixcir_i2c_ts.c V2.3.1	update  client->adapter->nr -> 0
 *
 * pixcir_i2c_ts.c V2.3.2	fix m48 show raw data bug
 *
 * pixcir_i2c_ts.c V2.3.3	fix m48 internal_enable register adress
 *
 * pixcir_i2c_ts.c V2.4		add reset tp function                          
 *                           fix bootloader issue
 *
 * pixcir_i2c_ts.c V2.5	    add function as the follows :
 *
 * add m48 internal_mode 
 * INTERNAL_MODE	4
 *
 * add m48_f32 two Tangos solution 
 * 
 * pixcir_i2c_ts.c V2.6    add function as the follows :
 * 
 *   BUTTON_NORMAL    15
 *   BUTTON_DEBUG_MODE  14
 *   
 *   the two functions support the solution of one Tango M48 and two Tangos M48_F32
 * 
 * 
 * pixcir_i2c_ts.c V2.6.1	add power on calibration for M48
 *  
 * pixcir_i2c_ts.c V2.6.2	add power managment code 	
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

#include "pixcir_i2c_ts.h"

#define DRIVER_VERSION "v2.6.2"
#define DRIVER_AUTHOR "Bee<http://www.pixcir.com.cn>"
#define DRIVER_DESC "Pixcir I2C Touchscreen Driver with tune fuction"
#define DRIVER_LICENSE "GPL"

#define	PIXCIR_DEBUG		0

/*********************************V2.0-Bee-0928-TOP****************************************/

#define	SLAVE_ADDR		I2C_SLAVE_I2C_PIXCIR_ADDR
#define	BOOTLOADER_ADDR		0x5d

#ifndef	I2C_MAJOR
#define	I2C_MAJOR 		125
#endif

#define	I2C_MINORS 		256

#define	CALIBRATION_FLAG	1
#define	NORMAL_MODE		8
#define	PIXCIR_DEBUG_MODE	3
#define  INTERNAL_MODE	4
//#define  RASTER_MODE		5
#define	VERSION_FLAG		6
#define	BOOTLOADER_MODE		7
#define RESET_TP         	9  
#define  ENABLE_IRQ		10
#define  DISABLE_IRQ		11

#define	RD_EEPROM		12
#define	WR_EEPROM		13
#define BUTTON_NORMAL    15
#define BUTTON_DEBUG_MODE  14

#ifdef R8C_AUO_I2C	
  #define SPECOP		0x78
#else
  #define SPECOP		0x37
#endif

#define reset

static unsigned char board_version = 0;
/* extern unsigned char get_emxx_board_version(void); */

/* To save touch state */
static int SavedTouch = 0;

int global_irq;
int global_irq_state = 1;

static unsigned char status_reg = 0;
unsigned char read_XN_YN_flag = 0,mode_change_flag=0;

unsigned char global_touching, global_oldtouching;
unsigned char global_posx1_low, global_posx1_high, global_posy1_low,
		global_posy1_high, global_posx2_low, global_posx2_high,
		global_posy2_low, global_posy2_high;

unsigned char Tango_number;

unsigned char interrupt_flag = 0;

unsigned char x_nb_electrodes = 0;
unsigned char y_nb_electrodes = 0;
unsigned char x2_nb_electrodes = 0;
unsigned char x1_x2_nb_electrodes = 0;

signed char xy_raw1[(MAXY*2+3)];
signed char xy_raw2[MAXX*2];
signed char xy_raw12[(MAXY*2+MAXX*2+3)];
signed char button_raw[16];

unsigned char data2eep[3],op2eep[2];

struct i2c_dev
{
	struct list_head list;
	struct i2c_adapter *adap;
	struct device *dev;
};

static struct i2c_driver pixcir_i2c_ts_driver;
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

void read_XN_YN_value(struct i2c_client *client)
{
	char Wrbuf[4], Rdbuf[2];
	int ret;

	memset(Wrbuf, 0, sizeof(Wrbuf));
	memset(Rdbuf, 0, sizeof(Rdbuf));

	Wrbuf[0] = SPECOP; 	//specop addr
	Wrbuf[1] = 1; 		//write 1 to read eeprom

	#ifdef ATMEL_168
	Wrbuf[2] = 64;
	Wrbuf[3] = 0;		
	#endif

	#ifdef R8C_3GA_2TG
	Wrbuf[2] = 64;
	Wrbuf[3] = 0;
	#endif

	#ifdef M48
	Wrbuf[2] = 80;
	Wrbuf[3] = 0;
	#endif
	
    #ifdef M48_F32
    Wrbuf[2] = 80;
	Wrbuf[3] = 0;
	#endif
	ret = i2c_master_send(client, Wrbuf, 4);
	if (ret != 4)
		printk("send ret = %d\n", ret);
	mdelay(8);
	ret = i2c_master_recv(client, Rdbuf, 2);
	if (ret != 2)
		printk("recv x_nb, ret = %d\n", ret);
	x_nb_electrodes = Rdbuf[0];
	printk("x_nb_electrodes = %d\n", x_nb_electrodes);

	if (Tango_number == 1) {
		x2_nb_electrodes = 0;

		memset(Wrbuf, 0, sizeof(Wrbuf));
		memset(Rdbuf, 0, sizeof(Rdbuf));

		Wrbuf[0] = SPECOP; 	//specop addr
		Wrbuf[1] = 1; 		//write to eeprom
		#ifdef ATMEL_168
		Wrbuf[2] = 203;
		Wrbuf[3] = 0;		
		#endif

		#ifdef R8C_3GA_2TG
		Wrbuf[2] = 203;
		Wrbuf[3] = 0;
		#endif

		#ifdef M48
		Wrbuf[2] = 195;
		Wrbuf[3] = 0;		
		#endif
		ret = i2c_master_send(client, Wrbuf, 4);
		mdelay(4);

		ret = i2c_master_recv(client, Rdbuf, 2);
		if (ret != 2)
			printk("recv y_nb, ret = %d\n", ret);
		y_nb_electrodes = Rdbuf[0];
		printk("y_nb_electrodes = %d\n", y_nb_electrodes);
	}
	else if (Tango_number == 2) {
		memset(Wrbuf, 0, sizeof(Wrbuf));
		memset(Rdbuf, 0, sizeof(Rdbuf));

		Wrbuf[0] = SPECOP; 	//specop addr
		Wrbuf[1] = 1; 		//write to eeprom
	#ifdef R8C_3GA_2TG
		Wrbuf[2] = 151;
		Wrbuf[3] = 0;	
		ret = i2c_master_send(client, Wrbuf, 4);
		mdelay(4);
		ret = i2c_master_recv(client, Rdbuf, 2);
		x2_nb_electrodes = Rdbuf[0];
		printk("x2_nb_electrodes = %d\n", x2_nb_electrodes);
	#endif

	#ifdef	R8C_AUO_I2C
		Wrbuf[2] = 211;
		Wrbuf[3] = 0;		
		ret = i2c_master_send(client, Wrbuf, 4);
		mdelay(4);
		ret = i2c_master_recv(client, Rdbuf, 2);
		x2_nb_electrodes = Rdbuf[0];
		printk("x2_nb_electrodes = %d\n", x2_nb_electrodes);
	#endif

	#ifdef M48_F32
        x2_nb_electrodes = 0;
	#endif


		memset(Wrbuf, 0, sizeof(Wrbuf));
		memset(Rdbuf, 0, sizeof(Rdbuf));

		Wrbuf[0] = SPECOP; 	//specop addr
		Wrbuf[1] = 1; 		//write to eeprom
	#ifdef R8C_3GA_2TG
		Wrbuf[2] = 238;
		Wrbuf[3] = 0;
	#endif

	#ifdef	R8C_AUO_I2C
		Wrbuf[2] = 151;
		Wrbuf[3] = 0;
	#endif

	#ifdef M48_F32
        Wrbuf[2] = 78;
		Wrbuf[3] = 1;
	#endif
	
		ret = i2c_master_send(client, Wrbuf, 4);
		mdelay(4);

		ret = i2c_master_recv(client, Rdbuf, 2);
		y_nb_electrodes = Rdbuf[0];
		printk("y_nb_electrodes = %d\n", y_nb_electrodes);
	}
	if (x2_nb_electrodes) {
		x1_x2_nb_electrodes = x_nb_electrodes + x2_nb_electrodes - 1;
	}
	else {
		x1_x2_nb_electrodes = x_nb_electrodes;
	}
	read_XN_YN_flag = 1;
}

 void read_button_raw(struct i2c_client *client, signed char *button_raw_table)
	 {	 
	 u_int8_t Wrbuf[1];
	 int ret;
	 memset(Wrbuf, 0, sizeof(Wrbuf));
	#ifdef R8C_AUO_I2C
		Wrbuf[0] = 128; //xy_raw1[0] rawdata X register address for AUO
	#endif
 
	#ifdef R8C_3GA_2TG
		Wrbuf[0] = 61; //xy_raw1[0] rawdata X register address for PIXCIR R8C_3GA_2TG
	#endif
 
	#ifdef M48
		Wrbuf[0] = 61; //xy_raw1[0] rawdata X register address for M48
	#endif
 
	#ifdef M48_F32
		Wrbuf[0] = 61;
	#endif
	 
	 ret = i2c_master_send(client, Wrbuf, 1);
	 if (ret != 1) {
		 printk("send xy_raw1[0] register address error in read_button_raw function ret = %d\n",ret);
	 }
	 mdelay(8);
	 ret = i2c_master_recv(client, button_raw_table, 16);
	 if (ret != 16) {
		 printk("receive xy_raw1 error in read_button_raw function ret = %d\n",ret);
	 }
	 }
 

 void read_XY_tables(struct i2c_client *client, signed char *xy_raw1_buf,
		signed char *xy_raw2_buf)
{
	u_int8_t Wrbuf[1];
	int ret;

	memset(Wrbuf, 0, sizeof(Wrbuf));
	#ifdef R8C_AUO_I2C
	   Wrbuf[0] = 128; //xy_raw1[0] rawdata X register address for AUO
	#endif

	#ifdef R8C_3GA_2TG
	   Wrbuf[0] = 61; //xy_raw1[0] rawdata X register address for PIXCIR R8C_3GA_2TG
	#endif

	#ifdef M48
	   Wrbuf[0] = 61; //xy_raw1[0] rawdata X register address for M48
	#endif

	#ifdef M48_F32
	   Wrbuf[0] = 61;
	#endif
	
	ret = i2c_master_send(client, Wrbuf, 1);
	mdelay(8);
	if (ret != 1) {
		printk("send xy_raw1[0] register address error in read_XY_tables function ret = %d\n",ret);
	}
	ret = i2c_master_recv(client, xy_raw1_buf, (MAXY-1)*2);
	mdelay(10);
	if (ret != (MAXY-1)*2) {
		printk("receive xy_raw1 error in read_XY_tables function ret = %d\n",ret);
	}
	#ifdef R8C_AUO_I2C
	   Wrbuf[0] = 43;  //xy_raw2[0] rawdata Y register address for AUO
	#endif

	#ifdef R8C_3GA_2TG
	   Wrbuf[0] = 125; //xy_raw2[0] rawdata Y register address for PIXCIR R8C_3GA_2TG
	#endif

	#ifdef M48
	   Wrbuf[0] = 155; //xy_raw2[0] rawdata Y register address for PIXCIR M48
	#endif

	#ifdef M48_F32
	   Wrbuf[0] = 123; //xy_raw2[0] rawdata Y register address for two Tangos M48-F32
	#endif
	
	ret = i2c_master_send(client, Wrbuf, 1);
	mdelay(8);
	if (ret != 1) {
		printk("send xy_raw2[0] register address error in read_XY_tables function ret = %d\n",ret);
	}
	ret = i2c_master_recv(client, xy_raw2_buf, (MAXX-1)*2);
	mdelay(10);
	if (ret != (MAXX-1)*2) {
		printk("receive xy_raw2 error in read_XY_tables function ret = %d\n",ret);
	}
}

void cross_disable (struct i2c_client *client)
{
	unsigned char wrbuf[2],ret;
	memset(wrbuf,0,sizeof(wrbuf));
	#ifdef M48
	wrbuf[0] = 249;
	wrbuf[1] = 0;
	#endif

	#ifdef M48_F32
	wrbuf[0] = 217;
	wrbuf[1] = 0;
	#endif
	
	ret = i2c_master_send(client, wrbuf, 2);
	if(ret != 2) 
		printk("set cross_disable error,ret=%d\n",ret);
	
}

void button_raw_disable(struct i2c_client *client)
{
	unsigned char rdbuf[2];
	int ret =0;
	memset(rdbuf,0,sizeof(rdbuf));
	#ifdef M48
        rdbuf[0] = 36;
	rdbuf[1] = 0;
	#endif

	#ifdef M48_F32
	rdbuf[0] = 36;
	rdbuf[1] = 0;
	#endif
	
    ret = i2c_master_send(client,rdbuf,sizeof(rdbuf));
    if(ret!=sizeof(rdbuf))
		printk("button_raw_disable error ,i2c_master_send error,ret=%d\n",ret);

}

////////////////////Power On Calibration////////////////////
#ifdef POWERON_CAL
void poweron_cal(struct i2c_client *client)
{
	int ret;
	unsigned char wr2eep[5], rdeep[1];

	memset(rdeep, 0, sizeof(rdeep));
	memset(wr2eep, 0, sizeof(wr2eep));

	wr2eep[0] = SPECOP;
	wr2eep[1] = 1;
	wr2eep[2] = 325&0xFF;
	wr2eep[3] = (325>>8)&0xFF;
	ret = i2c_master_send(client, wr2eep, 4);
	if(ret != 4){
		dev_err(&client->dev,
			"%s: i2c_master_send failed(), ret=%d\n",
			__func__, ret);
	}

	ret = i2c_master_recv(client, rdeep, 1);
	if(ret!=1) {
		dev_err(&client->dev,
			"%s: i2c_master_recv failed(), ret=%d\n",
			__func__, ret);
	}
	if(rdeep[0]) {
		wr2eep[0] = SPECOP;
		wr2eep[1] = 2;
		ret = i2c_master_send(client, wr2eep, 2);
		if(ret != 2) {
			dev_err(&client->dev,
				"%s: i2c_master_send failed(), ret=%d\n",
				__func__, ret);
		}

		mdelay(4);

		wr2eep[0] = 326&0xFF;
		wr2eep[1] = (326>>8)&0xFF;
		wr2eep[2] = 1;
		ret = i2c_master_send(client, wr2eep, 3);
		if(ret != 3) {
			dev_err(&client->dev,
				"%s: i2c_master_send failed(), ret=%d\n",
				__func__, ret);
		}
	}

	mdelay(4);
	i2c_master_recv(client, wr2eep, 1);
	mdelay(100);
}
#endif
/*********************************V2.0-Bee-0928-BOTTOM****************************************/

static struct workqueue_struct *pixcir_wq;

struct pixcir_i2c_ts_data
{
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
	int irq;
};

static void pixcir_ts_poscheck(struct work_struct *work)
{
	struct pixcir_i2c_ts_data *tsdata = container_of(work,
			struct pixcir_i2c_ts_data,
			work.work);
	unsigned char touching = 0;
	unsigned char oldtouching = 0;
	int posx1, posy1, posx2, posy2;
	unsigned char Rdbuf[10],Wrbuf[1];
	int ret;
	int z = 50;
	int w = 15;
	#ifdef R8C_AUO_I2C
	unsigned char auotpnum[1];
	#endif

	interrupt_flag = 1;

	memset(Wrbuf, 0, sizeof(Wrbuf));
	memset(Rdbuf, 0, sizeof(Rdbuf));

	Wrbuf[0] = 0;

	ret = i2c_master_send(tsdata->client, Wrbuf, 1);

#if PIXCIR_DEBUG
	printk("master send ret:%d\n",ret);
#endif

	if (ret != 1) {
#if PIXCIR_DEBUG
		dev_err(&tsdata->client->dev, "Unable to write to i2c touchscreen!\n");
#endif
		goto out;
	}

#ifdef R8C_AUO_I2C
	ret = i2c_master_recv(tsdata->client, Rdbuf, 8);

	if (ret != 8) {
		dev_err(&tsdata->client->dev, "Unable to read i2c page!\n");
		goto out;
	}

	posx1 = ((Rdbuf[1] << 8) | Rdbuf[0]);
	posy1 = ((Rdbuf[3] << 8) | Rdbuf[2]);
	posx2 = ((Rdbuf[5] << 8) | Rdbuf[4]);
	posy2 = ((Rdbuf[7] << 8) | Rdbuf[6]);
#else
	ret = i2c_master_recv(tsdata->client, Rdbuf, sizeof(Rdbuf));

	if (ret != sizeof(Rdbuf)) {
		dev_err(&tsdata->client->dev, "Unable to read i2c page!\n");
		goto out;
	}

	posx1 = ((Rdbuf[3] << 8) | Rdbuf[2]);
	posy1 = ((Rdbuf[5] << 8) | Rdbuf[4]);
	posx2 = ((Rdbuf[7] << 8) | Rdbuf[6]);
	posy2 = ((Rdbuf[9] << 8) | Rdbuf[8]);

#endif

#if PIXCIR_DEBUG
	printk("master recv ret:%d\n",ret);
#endif


#ifdef R8C_AUO_I2C
	if((posx1 != 0) || (posy1 != 0) || (posx2 != 0) || (posy2 != 0)) {
		Wrbuf[0] = 113;
		ret = i2c_master_send(tsdata->client, Wrbuf, 1);
		ret = i2c_master_recv(tsdata->client, auotpnum, 1);
		touching = auotpnum[0]>>5;
		oldtouching = 0;
	}
#else
	touching = Rdbuf[0];
	oldtouching = Rdbuf[1];
	
#endif

#if PIXCIR_DEBUG
	printk("touching:%-3d,oldtouching:%-3d,x1:%-6d,y1:%-6d,x2:%-6d,y2:%-6d\n",touching, oldtouching, posx1, posy1, posx2, posy2);
#endif


	if (touching) {
		input_report_abs(tsdata->input, ABS_X, posx1);
		input_report_abs(tsdata->input, ABS_Y, posy1);
		input_report_key(tsdata->input, BTN_TOUCH, 1);
		input_report_abs(tsdata->input, ABS_PRESSURE, 1);
	}
	else {
		input_report_key(tsdata->input, BTN_TOUCH, 0);
		input_report_abs(tsdata->input, ABS_PRESSURE, 0);
	}

	if (!(touching)) {
		z = 0;
		w = 0;
	}
	if (touching == 1){
	input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, z);
	input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, w);
	input_report_abs(tsdata->input, ABS_MT_POSITION_X, posx1);
	input_report_abs(tsdata->input, ABS_MT_POSITION_Y, posy1);
	input_mt_sync(tsdata->input);
	}
	else if (touching == 2) {
		input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, z);
		input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, w);
		input_report_abs(tsdata->input, ABS_MT_POSITION_X, posx1);
		input_report_abs(tsdata->input, ABS_MT_POSITION_Y, posy1);
		input_mt_sync(tsdata->input);

		input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, z);
		input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, w);
		input_report_abs(tsdata->input, ABS_MT_POSITION_X, posx2);
		input_report_abs(tsdata->input, ABS_MT_POSITION_Y, posy2);
		input_mt_sync(tsdata->input);
	}
	else if (SavedTouch) {
             /* Send an empty MT sync on last touch release */
             SavedTouch = 0;
#if PIXCIR_DEBUG
             printk("Empty sync\n");
#endif
             input_mt_sync(tsdata->input);
	}

	input_sync(tsdata->input);

	if(status_reg == NORMAL_MODE)
	{
	#ifdef R8C_AUO_I2C
		global_touching = 	touching;
	    	global_oldtouching = 	0;
	    	global_posx1_low = 	Rdbuf[0];
		global_posx1_high = 	Rdbuf[1];
		global_posy1_low = 	Rdbuf[2];
		global_posy1_high = 	Rdbuf[3];
		global_posx2_low = 	Rdbuf[4];
		global_posx2_high = 	Rdbuf[5];
		global_posy2_low = 	Rdbuf[6];
		global_posy2_high = 	Rdbuf[7];
	#else
		global_touching = 	touching;
		global_oldtouching = 	oldtouching;
		global_posx1_low = 	Rdbuf[2];
		global_posx1_high = 	Rdbuf[3];
		global_posy1_low = 	Rdbuf[4];
		global_posy1_high = 	Rdbuf[5];
		global_posx2_low = 	Rdbuf[6];
		global_posx2_high = 	Rdbuf[7];
		global_posy2_low = 	Rdbuf[8];
		global_posy2_high = 	Rdbuf[9];
	#endif
	}
out:
	enable_irq(tsdata->irq);
}

static irqreturn_t pixcir_ts_isr(int irq, void *dev_id)
{
	struct pixcir_i2c_ts_data *tsdata = dev_id;
	if ((status_reg == 0) || (status_reg == NORMAL_MODE)) {
		disable_irq_nosync(irq);
		queue_work(pixcir_wq, &tsdata->work.work);
	}
	return IRQ_HANDLED;
}

static int pixcir_ts_open(struct input_dev *dev)
{
	return 0;
}

static void pixcir_ts_close(struct input_dev *dev)
{
}

static int pixcir_i2c_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct pixcir_i2c_ts_data *tsdata;
	struct input_dev *input;
	struct device *dev;
	struct i2c_dev *i2c_dev;
	int error;

	printk("pixcir_i2c_ts_probe\n");

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

	set_bit(EV_SYN, input->evbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(EV_ABS, input->evbit);
	set_bit(BTN_TOUCH, input->keybit);
	input_set_abs_params(input, ABS_X, TOUCHSCREEN_MINX, TOUCHSCREEN_MAXX, 0, 0);
	input_set_abs_params(input, ABS_Y, TOUCHSCREEN_MINY, TOUCHSCREEN_MAXY, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, TOUCHSCREEN_MINX, TOUCHSCREEN_MAXX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, TOUCHSCREEN_MINY, TOUCHSCREEN_MAXY, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 25, 0, 0);

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
	input->open = pixcir_ts_open;
	input->close = pixcir_ts_close;

	input_set_drvdata(input, tsdata);

	tsdata->client = client;
	tsdata->input = input;

	INIT_WORK(&tsdata->work.work, pixcir_ts_poscheck);

	tsdata->irq = client->irq;
	global_irq = client->irq;

	if (input_register_device(input)) {
		input_free_device(input);
		kfree(tsdata);
	}

#ifdef CONFIG_MACH_EMEV
	writel(readl(CHG_PULL1)|0xC000, CHG_PULL1);
	gpio_direction_input(irq_to_gpio(tsdata->irq));
		RESETPIN_CFG;
		RESETPIN_SET0;
		mdelay(20);
		RESETPIN_SET1;
		mdelay(200);
#endif

	if (request_irq(tsdata->irq, pixcir_ts_isr, IRQF_TRIGGER_LOW,client->name, tsdata)) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		input_unregister_device(input);
		input = NULL;
	}
	disable_irq_nosync(global_irq);

	device_init_wakeup(&client->dev, 1);

	/*********************************V2.0-Bee-0928-TOP****************************************/
	i2c_dev = get_free_i2c_dev(client->adapter);
	if (IS_ERR(i2c_dev)) {
		error = PTR_ERR(i2c_dev);
		return error;
	}

	dev = device_create(i2c_dev_class, &client->adapter->dev, MKDEV(I2C_MAJOR,
			    client->adapter->nr), NULL, "pixcir_i2c_ts%d", 0);
	if (IS_ERR(dev)) {
		error = PTR_ERR(dev);
		return error;
	}
	/*********************************V2.0-Bee-0928-BOTTOM****************************************/
	dev_err(&tsdata->client->dev, "insmod successfully!\n");

#ifdef POWERON_CAL
	poweron_cal(tsdata->client);
#endif

	enable_irq(global_irq);

	return 0;

}

static int pixcir_i2c_ts_remove(struct i2c_client *client)
{
	int error;
	struct i2c_dev *i2c_dev;
	struct pixcir_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
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

static int pixcir_i2c_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct pixcir_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
	unsigned char wrbuf[2] = { 0 };
	int ret;
	
	wrbuf[0] = 0x14;
	wrbuf[1] = 0x03;
	/**************************************************************
	wrbuf[1]:	0x00: Active mode
			0x01: Sleep mode
			0xA4: Sleep mode automatically switch
			0x02: Deep sleep mode
			0x03: Freeze mode
	More details see application note 710 power manangement section
	****************************************************************/
	ret = i2c_master_send(client, wrbuf, 2)
	if(ret!=2) {
		dev_err(&client->dev,

			"%s: i2c_master_send failed(), ret=%d\n",

			__func__, ret);
	}
	
	if (device_may_wakeup(&client->dev))
		enable_irq_wake(tsdata->irq);

	return 0;
}

static int pixcir_i2c_ts_resume(struct i2c_client *client)
{
	struct pixcir_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(tsdata->irq);

	//reset TP to resume
	RESETPIN_CFG;
	RESETPIN_SET0;
	mdelay(100);
	RESETPIN_SET1;
	mdelay(100);

	return 0;
}

/*********************************V2.0-Bee-0928****************************************/
/*                        	  pixcir_open                                         */
/*********************************V2.0-Bee-0928****************************************/
static int pixcir_open(struct inode *inode, struct file *file)
{
	int subminor;
	struct i2c_client *client;
	struct i2c_adapter *adapter;
	struct i2c_dev *i2c_dev;
	int ret = 0;
#if PIXCIR_DEBUG
	printk("enter pixcir_open function\n");
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
	snprintf(client->name, I2C_NAME_SIZE, "pixcir_i2c_ts%d", adapter->nr);
	client->driver = &pixcir_i2c_ts_driver;
	client->adapter = adapter;
	//if(i2cdev_check_addr(client->adapter,0x5c))
	//	return -EBUSY;
	file->private_data = client;

	return 0;
}

/*********************************V2.0-Bee-0928****************************************/
/*                        	  pixcir_ioctl                                        */
/*********************************V2.0-Bee-0928****************************************/
static long pixcir_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *) file->private_data;
	#if PIXCIR_DEBUG
	printk("cmd = %d,arg = %d\n", cmd, arg);
	#endif
	
	switch (cmd)
	{
	case CALIBRATION_FLAG: //CALIBRATION_FLAG = 1
		if(global_irq_state == 0) {
			global_irq_state = 1;
			enable_irq(global_irq);
		}
	#if PIXCIR_DEBUG
		printk("CALIBRATION\n");
	#endif
		client->addr = SLAVE_ADDR;
		status_reg = 0;
		status_reg = CALIBRATION_FLAG;
		break;

	case NORMAL_MODE:
		if(global_irq_state == 0) {
			global_irq_state = 1;
			enable_irq(global_irq);
		}
		client->addr = SLAVE_ADDR;
	#if PIXCIR_DEBUG
		printk("NORMAL_MODE\n");
	#endif		
		status_reg = 0;
		status_reg = NORMAL_MODE;
		break;

	case PIXCIR_DEBUG_MODE:
		client->addr = SLAVE_ADDR;
	#if PIXCIR_DEBUG		
		printk("PIXCIR_DEBUG_MODE\n");
	#endif		
		status_reg = 0;
		status_reg = PIXCIR_DEBUG_MODE;
		
		Tango_number = arg;
    	#ifdef R8C_3GA_2TG
        	Tango_number = 2;
    	#endif

	#ifdef M48
        	Tango_number = 1;
    	#endif
	#ifdef M48_F32
        	Tango_number = 2;
    	#endif	
		mode_change_flag=0;
		break;

	case BOOTLOADER_MODE: //BOOTLOADER_MODE = 7
		if(global_irq_state == 0) {
			global_irq_state = 1;
			enable_irq(global_irq);
		}
	#if PIXCIR_DEBUG
		printk("BOOTLOADER_MODE\n");
	#endif		
		status_reg = BOOTLOADER_MODE;
		disable_irq_nosync(global_irq);
		global_irq_state = 0;

	#ifdef reset
		client->addr = BOOTLOADER_ADDR;

		RESETPIN_CFG;
		RESETPIN_SET0;
		mdelay(20);
		RESETPIN_SET1;

	#ifdef ATMEL_168 
		mdelay(30);
    	#endif

    	#ifdef R8C_3GA_2TG
        	mdelay(50);
    	#endif

	    #ifdef M48
			mdelay(20);
		#endif

		#ifdef M48_F32
       		mdelay(20);
    	#endif	

	#else		//normal
		client->addr = SLAVE_ADDR;
		tmp[0] = SPECOP; 	//specop addr
		tmp[1] = 5; 		//change to bootloader
		ret = i2c_master_send(client,tmp,2);
	#if PIXCIR_DEBUG
		printk("BOOTLOADER_MODE,change to bootloader i2c_master_send ret = %d\n",ret);
	#endif

		client->addr = BOOTLOADER_ADDR;
		printk("client->addr = %x\n",client->addr);
	#endif
		break;
		
		case ENABLE_IRQ:
			status_reg = 0;
			global_irq_state = 1;
			enable_irq(global_irq);
		break;
		
		case DISABLE_IRQ:
			disable_irq_nosync(global_irq);
			global_irq_state = 0;
		break;

		case RD_EEPROM:
			client->addr = SLAVE_ADDR;
		#if PIXCIR_DEBUG	
			printk("WR_EEPROM\n");
		#endif
			status_reg = 0;
			status_reg = RD_EEPROM;
		break;

		case WR_EEPROM:
			client->addr = SLAVE_ADDR;
		#if PIXCIR_DEBUG
			printk("WR_EEPROM\n");
		#endif		
			status_reg = 0;
			status_reg = WR_EEPROM;
		break;

		case VERSION_FLAG:
			client->addr = SLAVE_ADDR;
		#if PIXCIR_DEBUG		
			printk("VERSION_FLAG\n");
		#endif		
			status_reg = 0;
			status_reg = VERSION_FLAG;
		
			Tango_number = arg;
    	#ifdef R8C_3GA_2TG
        	Tango_number = 2;
    	#endif

		#ifdef M48
        	Tango_number = 1;
    	#endif

		#ifdef M48_F32
        	Tango_number = 2;
    	#endif
		
		break;
		
		case RESET_TP:
			RESETPIN_CFG;
			RESETPIN_SET0;
			mdelay(100);
			RESETPIN_SET1;
			mdelay(100);
		break;
	
/*********************************V2.5-hang-110714-TOP****************************************/
		
	case INTERNAL_MODE:
		client->addr = SLAVE_ADDR;
	#if PIXCIR_DEBUG
		printk("INTERNAL_MODE\n");
	#endif
		status_reg = 0;
		status_reg = INTERNAL_MODE;
		Tango_number = arg;
		#ifdef M48
		Tango_number = 1;
    	#endif
		#ifdef M48_F32
		Tango_number = 2;
		#endif
		mode_change_flag=0;
		break;
/*********************************V2.5-hang-110714-BOTTOM****************************************/
	
        case BUTTON_NORMAL:
	 	client->addr = SLAVE_ADDR;
		#if PIXCIR_DEBUG
		printk("BUTTON_NORMAL\n");
		#endif
		status_reg = 0;
		status_reg = BUTTON_NORMAL;
	    #ifdef M48
	    Tango_number = 1;
		#endif

		#ifdef M48_F32
		Tango_number = 2;
		#endif

		break;

        case BUTTON_DEBUG_MODE:
	 	client->addr = SLAVE_ADDR;
		#if PIXCIR_DEBUG
		printk("BUTTON_DEBUG_MODE\n");
		#endif
		status_reg = 0;
		status_reg = BUTTON_DEBUG_MODE;
	    #ifdef M48
	    Tango_number = 1;				
		#endif

		#ifdef M48_F32
        Tango_number = 2;
		#endif

		mode_change_flag = 0;
		break;
			

	    default:
		break;//return -ENOTTY;
	}
	return 0;
}

/*********************************V2.0-Bee-0928****************************************/
/*                        	  pixcir_read                                         */
/*********************************V2.0-Bee-0928****************************************/
static ssize_t pixcir_read (struct file *file, char __user *buf, size_t count,loff_t *offset)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	int ret = 0;
	unsigned char normal_tmp[10];
	switch(status_reg)
	{
		case NORMAL_MODE:
		memset(normal_tmp, 0, sizeof(normal_tmp));
		if(interrupt_flag) {
			normal_tmp[0] = global_touching;
			normal_tmp[1] = global_oldtouching;
			normal_tmp[2] = global_posx1_low;
			normal_tmp[3] = global_posx1_high;
			normal_tmp[4] = global_posy1_low;
			normal_tmp[5] = global_posy1_high;
			normal_tmp[6] = global_posx2_low;
			normal_tmp[7] = global_posx2_high;
			normal_tmp[8] = global_posy2_low;
			normal_tmp[9] = global_posy2_high;
			//printk("global_touching:%-3d,global_oldtouching:%-3d\n",normal_tmp[0],normal_tmp[1]);
			ret = copy_to_user(buf, normal_tmp, 10);
			if(ret)	{
				printk("interrupt_flag = %d,NORMAL_MODE,copy_to_user error, ret = %d\n",interrupt_flag,ret);
			}
		}
		interrupt_flag = 0;
		break;
/*********************************V2.5-hang-110714-TOP****************************************/

		case PIXCIR_DEBUG_MODE:
		if(read_XN_YN_flag == 0 ) {
			unsigned char buf[2];
			
			memset(buf, 0, sizeof(buf));
			#ifdef M48
			cross_disable(client);
			#endif
			
			#ifdef M48_F32
            cross_disable(client);
			#endif
			
			read_XN_YN_value(client);
			}
		
		if(mode_change_flag==0){

			button_raw_disable(client);
            #ifdef ATMEL_168
			buf[0]=194;
			buf[1]=0;
			ret = i2c_master_send(client, buf, 2); //write internal_enable = 0
			if(ret != 2) 
				printk("PIXCIR_DEBUG_MODE,master send %d error, ret = %d\n",buf[0],ret);
					
			#endif

			#ifdef R8C_AUO_I2C
			//NULL
			#endif

			#ifdef R8C_3GA_2TG
			buf[0]=194;
			buf[1]=0;
			ret = i2c_master_send(client, buf, 2); //write internal_enable = 0
			if(ret != 2) 
				printk("PIXCIR_DEBUG_MODE,master send %d,%d error, ret = %d\n",buf[0],buf[1],ret);
			
			#endif

			#ifdef M48
			buf[0]=254;
			buf[1]=0;
			ret = i2c_master_send(client, buf, 2); //write internal_enable = 0
			if(ret != 2) 
				printk("PIXCIR_DEBUG_MODE,master send %d error, ret = %d\n",buf[0],ret);
			
			#endif

			#ifdef M48_F32
			buf[0]=222;
			buf[1]=0;
			ret = i2c_master_send(client, buf, 2); //write internal_enable = 0
			if(ret != 2) 
				printk("PIXCIR_DEBUG_MODE,M48_F32 Two Tango master send %d error, ret = %d\n",buf[0],ret);
			
			#endif	
			mode_change_flag=1;
			
		} else{
		 
			memset(xy_raw1, 0, sizeof(xy_raw1));
			memset(xy_raw2, 0, sizeof(xy_raw2));
	
			read_XY_tables(client,xy_raw1,xy_raw2);						
		}	
		
		if(Tango_number ==1 ) {

			xy_raw1[MAXY*2] = x_nb_electrodes;
			xy_raw1[MAXY*2+1] = y_nb_electrodes;
			ret = copy_to_user(buf,xy_raw1,MAXY*2+2);
			if(ret) {
				printk("PIXCIR_DEBUG_MODE,Tango_number = 1 copy_to_user error,ret =%d\n",ret);
				ret = -1;
				} else {
				ret = count;
					}
	
		
		}else if(Tango_number == 2) {
			xy_raw1[(MAXY-1)*2] = x_nb_electrodes;
			xy_raw1[(MAXY-1)*2+1] = y_nb_electrodes;
			xy_raw1[(MAXY-1)*2+2] = x2_nb_electrodes;

			for(ret=0;ret<((MAXY-1)*2+3);ret++)
				xy_raw12[ret] = xy_raw1[ret];
            
			for(ret=0;ret<(MAXX*2-1);ret++)
				xy_raw12[((MAXY-1)*2+3)+ret] = xy_raw2[ret];
		
           
			if(copy_to_user(buf,xy_raw12,(MAXX+MAXY-2)*2+3)) {
				printk("PIXCIR_DEBUG_MODE,Tango_number = 2 copy_to_user error\n");
				ret = -1;
				}else {
				ret = count;
					}
			
	
			}
		break;
/*********************************V2.5-hang-110714-BOTTOM****************************************/

		case RD_EEPROM:
		{
		unsigned char epmbytbuf[512];
		memset(epmbytbuf, 0, sizeof(epmbytbuf));
		ret = i2c_master_recv(client, epmbytbuf, count);
		if(ret!=count) {
			ret = -1;
			printk("READ_EEPROM,i2c_master_recv error, ret = %d\n",ret);
			break;
		}
		if(copy_to_user(buf,epmbytbuf,count)) {
			ret = -1;
			printk("READ_EEPROM,copy_to_user error, ret = %d\n",ret);
		}
		}
		break;


		case VERSION_FLAG:
		{
		unsigned char vaddbuf[1],verbuf[5];
		memset(vaddbuf, 0, sizeof(vaddbuf));
		memset(verbuf, 0, sizeof(verbuf));
		#ifdef R8C_AUO_I2C
		//NULL
		#endif
		
		#ifdef ATMEL_168
			vaddbuf[0] = 48;
		#endif

		#ifdef R8C_3GA_2TG
			vaddbuf[0] = 48;
		#endif		

		#ifdef M48
			vaddbuf[0] = 48;
		#endif
		
		#ifdef M48_F32
			vaddbuf[0] = 48;
		#endif
			ret = i2c_master_send(client, vaddbuf, 1); //version address
			if(ret != 1) {
				printk("VERSION_FLAG,master send %d error, ret = %d\n",vaddbuf[0],ret);
			}
			ret = i2c_master_recv(client, verbuf, 5);			
			if(copy_to_user(buf,verbuf,5)) {
				printk("VERSION_FLAG,copy_to_user error, ret = %d\n",ret);
			}	
		}	
		break;

/*********************************V2.5-hang-110714-TOP****************************************/

		case INTERNAL_MODE:
	    {
			
         #ifdef M48
		 unsigned char rdabuf[2],rett;
	
		 memset(rdabuf,0,sizeof(rdabuf));
	
						   
		  if (read_XN_YN_flag==0){
			cross_disable(client);
			read_XN_YN_value(client);
		  	}
		  
		  if(mode_change_flag==0){
		  	button_raw_disable(client);
			rdabuf[0] = 254;
			rdabuf[1] = 1;
			rett = i2c_master_send(client, rdabuf, 2);			
			if(rett!=2)
			 printk("master send internal enable error,rett=%d",rett);
			 mode_change_flag=1;													
		  	 } else {			   
			 memset(xy_raw1, 0, sizeof(xy_raw1));
			 memset(xy_raw2, 0, sizeof(xy_raw2));
			 memset(xy_raw12,0, sizeof(xy_raw12));
			 
			
			 read_XY_tables(client,xy_raw1,xy_raw2);
			}
							   
			 xy_raw1[MAXY*2] = x_nb_electrodes;
			 xy_raw1[MAXY*2+1] = y_nb_electrodes;
			 ret = copy_to_user(buf,xy_raw1,MAXY*2+2);
			if(ret) {
				printk("PIXCIR_DEBUG_MODE,Tango_number = 1 copy_to_user error, ret = %d\n",ret);
				ret = -1;
			}else {
			    ret = count;
			   // printk("ret = %d\n",ret);
				}
					   
	   #endif  

	    #ifdef M48_F32
		 unsigned char rdabuf[2],rett;
	
		 memset(rdabuf,0,sizeof(rdabuf));
	
						   
		  if (read_XN_YN_flag==0){
			cross_disable(client);
			read_XN_YN_value(client);
		  	}
		  if(mode_change_flag==0){
		  	button_raw_disable(client);
			rdabuf[0] = 222;
			rdabuf[1] = 1;
			rett = i2c_master_send(client, rdabuf, 2);			
			 if(rett!=2)
			 printk("master send internal enable error,rett=%d",rett);
			 mode_change_flag=1;
																
		  	} else{			   
			 memset(xy_raw1, 0, sizeof(xy_raw1));
			 memset(xy_raw2, 0, sizeof(xy_raw2));
			 memset(xy_raw12,0, sizeof(xy_raw12));
			 
			 read_XY_tables(client,xy_raw1,xy_raw2);
			}

			 xy_raw1[(MAXY-1)*2] = x_nb_electrodes;
			 xy_raw1[(MAXY-1)*2+1] = y_nb_electrodes;
			 xy_raw1[(MAXY-1)*2+2] = x2_nb_electrodes;

			 for(ret=0;ret<((MAXY-1)*2+3);ret++)
				xy_raw12[ret] = xy_raw1[ret];


			 for(ret=0;ret<(MAXX*2-1);ret++)
				xy_raw12[((MAXY-1)*2+3)+ret] = xy_raw2[ret];
             ret = copy_to_user(buf,xy_raw12,(MAXX+MAXY-2)*2+3);
			 if(ret) {
				printk("PIXCIR_DEBUG_MODE,Tango_number = 2 copy_to_user error, ret = %d\n",ret);
				ret = -1;
			 }else {
                ret = count;
		 //printk("ret = %d\n",ret);
			 	}
			 					   
	   #endif  
			}
		break;
/*********************************V2.5-hang-110714-BOTTOM****************************************/
      case BUTTON_NORMAL:
	{
	    unsigned char sdbuf[1],rdbuf[1],button_flag[1];
		memset(sdbuf,0,sizeof(sdbuf));
		memset(rdbuf,0,sizeof(rdbuf));
        memset(button_flag,0,sizeof(button_flag));
        #ifdef M48
        sdbuf[0] = 41;
	#endif

	#ifdef M48_F32
        sdbuf[0] = 41;
	#endif
		
		ret = i2c_master_send(client,sdbuf,1);
		if(ret!=1){
			printk("master send button address error,ret = %d\n",ret);
                 }/* else {
                        printk("master send button address sucessce,ret = %d\n",ret);
                        }*/
		ret = i2c_master_recv(client,rdbuf,1);
		if(ret!= 1)  {
			printk("master receive button flag error ,ret =%d\n",ret);
                 } else {
			   button_flag[0] = rdbuf[0];
                 // printk("master receive button flag succeed,ret = %d, button_flag register = %d\n",ret,button_flag[0]);
                     }
		ret = copy_to_user(buf,button_flag,1);
		if(ret)  {
			printk("BOTTON_NORMAL ,copy_to_user error,ret =%d\n",ret);
			ret = -1;
		}else {
                        //printk("BOTTON_NORMAL ,copy_to_user success,*buf=%d,test button_flag[0]=%d\n",*buf,button_flag[0]);
		    ret =count;
		     //printk("ret = %d\n",ret);
		}
	
	}
        break; 

         		case BUTTON_DEBUG_MODE:
              {
		unsigned char sdbuf[2],rdbuf[1];
	
		memset(sdbuf,0,sizeof(sdbuf));
		memset(rdbuf,0,sizeof(rdbuf));
        
		
		if(mode_change_flag == 0){
			#ifdef M48
			sdbuf[0] = 40;
			#endif

			#ifdef M48_F32
			sdbuf[0] = 40;
			#endif
			
			ret = i2c_master_send(client,sdbuf,1);
			if(ret!=1)
                 printk("BUTTON_DEBUG_MODE ,i2c_master_send button_scan register error,ret=%d\n",ret);
            ret = i2c_master_recv(client,rdbuf,1);
			if(ret!=1)
				 printk("BUTTON_DEBUG_MODE ,i2c_master_recv button_scan register error,ret=%d\n",ret);			 
			if(rdbuf[0]==1) {
				#ifdef M48	
				sdbuf[0] = 36;
				sdbuf[1] = 1;
				#endif

				#ifdef M48_F32
				sdbuf[0] = 36;
				sdbuf[1] = 1;
				#endif
				
				ret = i2c_master_send(client,sdbuf,2);
				if(ret!=2)
				  printk("BUTTON_DEBUG_MODE ,i2c_master_send recerved register error,ret=%d\n",ret);	
				}
			mdelay(8);
				cross_disable(client);
		  		#ifdef M48
		  		sdbuf[0]=254;
		 		sdbuf[1]=0;
		  		#endif

		 		#ifdef M48_F32
		  		sdbuf[0]=222;
		  		sdbuf[1]=0;
		 		#endif
		        ret = i2c_master_send(client, sdbuf, 2); //write internal_enable = 0
			    if(ret != 2) 
				printk("button_debug_mode error,master send internal_enable register error,ret=%d\n",ret);				
			    mode_change_flag=1;		
	
			}else {
			memset(button_raw,0,sizeof(button_raw));
			read_button_raw(client,button_raw);		
				}
                                  
			ret = copy_to_user(buf,button_raw,16);
			 if(ret) {
				printk("BUTTON_DEBUG_MODE,M48 one Tango, raw data copy_to_user error, ret = %d\n",ret);
			 ret = -1; printk("ret = %d\n",ret);
		         }else {
                               ret = count;
                              // printk("ret = %d\n",ret);
		       	}
             }
              break;

	

		default:
		break;
	}

	return ret;
}

/*********************************V2.0-Bee-0928****************************************/
/*                        	  pixcir_write                                        */
/*********************************V2.0-Bee-0928****************************************/
static ssize_t pixcir_write(struct file *file, const char __user *buf,size_t count, loff_t *ppos)
{
	struct i2c_client *client;
	char *tmp,bootload_data[143],Rdbuf[1];
	unsigned char buf1[1],tmp1[1],ret1;
	static int ret=0,ret2=0,stu;
	static int re_value=0;
	#ifdef R8C_3GA_2TG
	char Rdverbuf[2];
	#endif

	#ifdef M48
	char Rdverbuf[2];
	#endif

	client = file->private_data;

	switch(status_reg)
	{
		case CALIBRATION_FLAG: //CALIBRATION_FLAG=1
		 {
		tmp = kmalloc(count,GFP_KERNEL);
		if (tmp==NULL)
			return -ENOMEM;
		if (copy_from_user(tmp,buf,count)) { 	
			printk("CALIBRATION_FLAG copy_from_user error\n");
			kfree(tmp);
			return -EFAULT;
		}
		ret = i2c_master_send(client,tmp,count);
	#if PIXCIR_DEBUG
		printk("CALIBRATION_FLAG,i2c_master_send ret = %d\n",ret);
	#endif
		mdelay(100);
		if(ret!=count) 
			printk("CALIBRATION_FLAG,Unable to write to i2c page for calibratoion!\n");
/*********************************V2.5-hang-110714-TOP****************************************/
        #ifdef M48
		if(ret==count){
			mdelay(20);	 
		    memset(buf1,0,sizeof(buf1));
			memset(tmp1,0,sizeof(tmp1));
			buf1[0]=0x37;
			ret1 = i2c_master_send(client,buf1,1);			 
			if (ret1!=1) {
				printk("CALIBRATION_FLAG,master_sent %d error,ret1=%d\n",buf1[0],ret1);
				ret = 0;
			}
			ret1 = i2c_master_recv(client,tmp1,1);
			if (ret1!=1) {
				printk("CALIBRATION_FLAG,master_recv %d error,ret1=%d",tmp1[0],ret1);
				ret = 0;
			}
			if (!tmp1[0])
				printk("Calibration	is Passing\n"); 		
			else {
				printk("Calibration Fail\n");
				ret = 0;
			}
		}
		#endif
		kfree(tmp);

		status_reg = 0;
			}
		break;
/*********************************V2.5-hang-110714-BOTTOM****************************************/

		case BOOTLOADER_MODE: //BOOTLOADER_MODE=7
	#if PIXCIR_DEBUG
		printk("BOOT ");
	#endif
		memset(bootload_data, 0, sizeof(bootload_data));
		memset(Rdbuf, 0, sizeof(Rdbuf));

		if (copy_from_user(bootload_data,buf,count)) {
			printk("COPY FAIL ");
			return -EFAULT;
		}
		
#if PIXCIR_DEBUG
		static int i;
		for(i=0;i<143;i++) {
			if(bootload_data[i]<0x10)
			printk("0%x",bootload_data[i]);
			else
			printk("%x",bootload_data[i]);
		}
#endif

		stu = bootload_data[0];

		#ifdef R8C_3GA_2TG
		if(stu == 0x01){
			ret2 = i2c_master_recv(client,Rdverbuf,2);
			if((ret2 != 2) || (Rdverbuf[1] != 0xA5)) {
			    printk("i2c_master_recv boot status error ret2=%d,bootloader status=%x",ret2,Rdverbuf[1]);
			    ret = 0;
			    break;
			}
			printk("\n");
			printk("Bootloader Status:%x%x\n",Rdverbuf[0],Rdverbuf[1]);
		}
		#endif

		#ifdef M48
		if(stu == 0x01){
			ret2 = i2c_master_recv(client,Rdverbuf,2);
			if((ret2 != 2) || (Rdverbuf[1] != 0xA5)) {
			    printk("i2c_master_recv boot status error ret2=%d,bootloader status=%x",ret2,Rdverbuf[1]);
			    ret = 0;
			    break;
			}
			printk("\n");
			printk("Bootloader Status:%x%x\n",Rdverbuf[0],Rdverbuf[1]);
		}
		#endif

		ret = i2c_master_send(client,bootload_data,count);
		if(ret!=count) {
			status_reg = 0;
			global_irq_state = 1;		
			enable_irq(global_irq);
			printk("bootload 143 bytes error,ret = %d\n",ret);
			break;
		}
		
		if(stu!=0x01) {
			mdelay(1);
			while(get_attb_value(ATTB));
			mdelay(1);

#ifdef EEPCRC_OPEN
		#ifdef ATMEL_168
			ret2 = i2c_master_recv(client,Rdbuf,1);
			if(ret2!=1) {
			    ret = 0;
				printk("read IIC slave status error,ret = %d\n",ret2);
				break;

			}
			re_value = Rdbuf[0];
		    #if PIXCIR_DEBUG
			printk("re_value = %d\n",re_value);
		    #endif
		#endif

		#ifdef R8C_3GA_2TG
			if(stu == 0x03) {
				ret2 = i2c_master_recv(client,Rdbuf,1);
				printk("ret2=%d re_value=%d\n",ret2,Rdbuf[0]);
				if(ret2!=1) {
			  	  	ret = 0;
					printk("read IIC slave status error,ret = %d\n",ret2);
					break;
				}
			}
		#endif

		#ifdef M48
			if(stu == 0x03) {
				ret2 = i2c_master_recv(client,Rdbuf,1);
				printk("ret2=%d re_value=%d\n",ret2,Rdbuf[0]);
				if(ret2!=1) {
					status_reg = 0;
					global_irq_state = 1;		
					enable_irq(global_irq);
			  	  	ret = 0;
					printk("read IIC slave status error,ret = %d\n",ret2);
					break;
				}
			}
		#endif
#endif
		}
		else {
			mdelay(100);
			status_reg = 0;
			global_irq_state = 1;
			enable_irq(global_irq);	
		}

		if((re_value&0x80)&&(stu!=0x01)) {
			printk("Failed:(re_value&0x80)&&(stu!=0x01)=1\n");
			ret = 0;
		}
		break;

		case RD_EEPROM://12
		{
		unsigned char epmdatabuf[2],wr2eep[4];
		memset(epmdatabuf, 0, sizeof(epmdatabuf));
		memset(wr2eep, 0, sizeof(wr2eep));
		ret = copy_from_user(epmdatabuf, buf, count);
		if(ret){
			ret = -1;
			printk("WRITE_EEPROM,copy_from_user error, ret = %d\n",ret);
			break;
		}
		wr2eep[0] = SPECOP;
		wr2eep[1] = 1;
		wr2eep[2] = epmdatabuf[0];
		wr2eep[3] = epmdatabuf[1];
		ret = i2c_master_send(client, wr2eep, 4);
		if(ret != 4){
			ret = -1;
			printk("WRITE_EEPROM wr2eep,i2c_master_send, ret = %d\n",ret);
			break;	
		}
		}
		break;

		case WR_EEPROM:		//13
		{
		unsigned char epmdatabuf[2];
		memset(epmdatabuf, 0, sizeof(epmdatabuf));
		ret = copy_from_user(epmdatabuf, buf, count);
		if(ret){
			ret = -1;
			printk("WRITE_EEPROM,copy_from_user error, ret = %d\n",ret);
			break;
		}

		if(2 == count){		// write a byte 2 eeprom
			op2eep[0] = SPECOP;
			op2eep[1] = 2;
			data2eep[0] = epmdatabuf[0];
			data2eep[1] = epmdatabuf[1];
		}
		else if(1 == count){	//write addr 2 eeprom 
			data2eep[2] = epmdatabuf[0];
			ret = i2c_master_send(client, op2eep, 2);
			if(ret != 2){
				ret = -1;
				printk("WRITE_EEPROM op2eep,i2c_master_send, ret = %d\n",ret);
				break;	
			} 
			ret = i2c_master_send(client, data2eep, 3);
			if(ret != 3){
				ret = -1;
				printk("WRITE_EEPROM data2eep,i2c_master_send, ret = %d\n",ret);
				break;	
			} 
			mdelay(4);
			i2c_master_recv(client, data2eep, 1);
			mdelay(100);
		}
		}
		break;

		default:
		break;
	}
	return ret;
}

/*********************************V2.0-Bee-0928****************************************/
/*                        	  pixcir_release                                         */
/*********************************V2.0-Bee-0928****************************************/
static int pixcir_release(struct inode *inode, struct file *file)
{
	struct i2c_client *client = file->private_data;
   #if PIXCIR_DEBUG
	printk("enter pixcir_release funtion\n");
   #endif
	i2c_put_adapter(client->adapter);
	kfree(client);
	file->private_data = NULL;

	return 0;
}

/*********************************V2.0-Bee-0928-TOP****************************************/
static const struct file_operations pixcir_i2c_ts_fops =
{
	.owner = THIS_MODULE, 
	.read = pixcir_read,
	.write = pixcir_write,
	.open = pixcir_open,
	.unlocked_ioctl = pixcir_ioctl,
	.release = pixcir_release, 
};
/*********************************V2.0-Bee-0928-BOTTOM****************************************/

static const struct i2c_device_id pixcir_i2c_ts_id[] =
{
	{ "pixcir_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE( i2c, pixcir_i2c_ts_id);

static struct i2c_driver pixcir_i2c_ts_driver =
{ 	.driver =
		{
			.owner = THIS_MODULE,
			.name = "pixcir_i2c_ts_driver_v2.6.2",
		}, 
	.probe = pixcir_i2c_ts_probe, 
	.remove = pixcir_i2c_ts_remove,
	.suspend = pixcir_i2c_ts_suspend, 
	.resume = pixcir_i2c_ts_resume,
	.id_table = pixcir_i2c_ts_id, 
};

static int __init pixcir_i2c_ts_init(void)
{
	int ret;
	printk("pixcir_i2c_init\n");
	pixcir_wq = create_singlethread_workqueue("pixcir_wq");
	if(!pixcir_wq)
	return -ENOMEM;
	/*********************************V2.0-Bee-0928-TOP****************************************/
	ret = register_chrdev(I2C_MAJOR,"pixcir_i2c_ts",&pixcir_i2c_ts_fops);
	if(ret) {
		printk(KERN_ERR "%s:register chrdev failed\n",__FILE__);
		return ret;
	}

	i2c_dev_class = class_create(THIS_MODULE, "pixcir_i2c_dev");
	if (IS_ERR(i2c_dev_class)) {
		ret = PTR_ERR(i2c_dev_class);
		class_destroy(i2c_dev_class);
	}
	/********************************V2.0-Bee-0928-BOTTOM******************************************/
	return i2c_add_driver(&pixcir_i2c_ts_driver);
}

static void __exit pixcir_i2c_ts_exit(void)
{
	i2c_del_driver(&pixcir_i2c_ts_driver);
	/********************************V2.0-Bee-0928-TOP******************************************/
	class_destroy(i2c_dev_class);
	unregister_chrdev(I2C_MAJOR,"pixcir_i2c_ts");
	/********************************V2.0-Bee-0928-BOTTOM******************************************/
	if(pixcir_wq)
	destroy_workqueue(pixcir_wq);
}

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);

module_init( pixcir_i2c_ts_init);
module_exit( pixcir_i2c_ts_exit);
