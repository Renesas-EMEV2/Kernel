#include <linux/slab.h>
#include "emxx_sensors.h"

#define	MAX_GSENSOR_VAL		16
#define CHECK_VALUE_RANGE(x)	{if(x>16) x=16; else if(x<-16) x=-16;}

struct bma220_obj {
	struct emxx_sensors_obj sensors_obj;
	int suspend_mode;
};


static int bma220_acc_get_acceleration_data(
	struct emxx_sensors_pub* sensors_pub,
	struct emxx_sensors_obj* sensors_obj,
	int *xyz)
{
	int err = -1;
	u8 reg_data[4];
	memset(reg_data, 0x00, sizeof(reg_data));
	/*
	//u8 acc_data[3];
	u8 x[2] = {0x02};
	u8 y[2] = {0x03};
	u8 z[2] = {0x04};
	*/
	reg_data[0] = 0x02;
	err = sensors_pub->i2c_read(sensors_pub, I2C_SLAVE_BMA220_ADDR, reg_data, 4);
	if (err < 0){
		printk("read bma220 data error\n");
		memset(reg_data, 0x00, sizeof(reg_data));
	}

	//这里要强制转换一下，因为读出来的数据是有符号的，在旋转的时候bma220会自动计算出数据的符号
	xyz[0] = 0 - (signed char)reg_data[2] >> 2;
	xyz[1] = 0 - (signed char)reg_data[1] >> 2;
	xyz[2] = 0 - (signed char)reg_data[3] >> 2;
	//Add calibration param
	xyz[0] -= sensors_obj->cali[0];
	xyz[1] -= sensors_obj->cali[1];
	xyz[2] -= sensors_obj->cali[2];
	CHECK_VALUE_RANGE(xyz[0]);
	CHECK_VALUE_RANGE(xyz[1]);
	CHECK_VALUE_RANGE(xyz[2]);
	//printk("[0]:%x data[0]:%d, data[1]:%d, data[2]:%d  %d:%d:%d\n", reg_data[0], xyz[0], xyz[1], xyz[2], sensors_obj->cali[0], sensors_obj->cali[1], sensors_obj->cali[2]);
	
	return err;
}

static int bma220_set_delay(struct emxx_sensors_pub* sensors_pub, struct emxx_sensors_obj* sensors_obj, int ms)
{
	return 0;
}

static int bma220_calibrate(struct emxx_sensors_pub* sensors_pub, struct emxx_sensors_obj* sensors_obj, int cali[4])
{
	//printk("[%s:%d:%d]:%d:%d:%d -- %d:%d:%d \n", __func__, __LINE__, cali[0], cali[1], cali[2], cali[3], sensors_obj->val[0], sensors_obj->val[1], sensors_obj->val[2]);
	if(cali[0] == 0) {
		sensors_obj->cali[0] = sensors_obj->cali[1] = sensors_obj->cali[2] = 0;
	} else {
		if(cali[0] == 1) {
			memcpy(sensors_obj->cali, cali+1, 3*sizeof(int));
		} else {
			//memcpy(sensors_obj->cali, sensors_obj->val, 3*sizeof(int));
			//set x=0, y=0, z=+max
			sensors_obj->cali[0] = sensors_obj->val[0];
			sensors_obj->cali[1] = sensors_obj->val[1];
			sensors_obj->cali[2] = sensors_obj->val[2] - MAX_GSENSOR_VAL;
			memcpy(cali+1, sensors_obj->cali, 3*sizeof(int));
		}
		//printk("[%s:%d:%d]:%d:%d:%d -- %d:%d:%d \n", __func__, __LINE__, cali[0], cali[1], cali[2], cali[3], sensors_obj->val[0], sensors_obj->val[1], sensors_obj->val[2]);
	}
	return 0;
}

int bma220_open(struct emxx_sensors_pub* sensors_pub, struct emxx_sensors_obj* sensors_obj)
{
	struct bma220_obj *bma220_obj = (struct bma220_obj *)sensors_obj;
	int ret;
	u8 acc_read[2];
	//read soft_reset twice
	//soft reset
	acc_read[0] = 0x19;
	acc_read[1] = 0;
	ret = sensors_pub->i2c_read(sensors_pub, I2C_SLAVE_BMA220_ADDR, acc_read, 1);
	//back into operation from soft reset
	acc_read[0] = 0x19; //identification
	acc_read[1] = 0;
	ret = sensors_pub->i2c_read(sensors_pub, I2C_SLAVE_BMA220_ADDR, acc_read, 1);
	bma220_obj->suspend_mode = 0;	//after soft reset, all register will be reset
	//read bma220 ID
	acc_read[0] = 0x00; //identification
	acc_read[1] = 0;
	ret = sensors_pub->i2c_read(sensors_pub, I2C_SLAVE_BMA220_ADDR, acc_read, 1);
	if ( (ret < 0) || (acc_read[0] != 0xDD) ) { //这里用acc_read的第0位，进行比较，我想是因为bma220不能连续读取寄存器内容的原因；读出的内容仍然放在0位
		return -ENXIO;
	}
	return 0;
}

int bma220_close(struct emxx_sensors_pub* sensors_pub, struct emxx_sensors_obj* sensors_obj)
{
	//printk("[%s:%d] \n", __func__, __LINE__);
	sensors_obj->suspend(sensors_pub, sensors_obj);
	return 0;
}

u32 bma220_get(struct emxx_sensors_pub* sensors_pub, struct emxx_sensors_obj* sensors_obj)
{
	bma220_acc_get_acceleration_data(sensors_pub, sensors_obj, sensors_obj->val);
	return 0;
}

int bma220_suspend(struct emxx_sensors_pub* sensors_pub, struct emxx_sensors_obj* sensors_obj)
{
	struct bma220_obj *bma220_obj = (struct bma220_obj *)sensors_obj;
	//printk("[%s:%d] \n", __func__, __LINE__);
	if(!bma220_obj->suspend_mode) {
		u8 acc_read[2];
		acc_read[0] = 0x18;
		acc_read[1] = 0;
		sensors_pub->i2c_read(sensors_pub, I2C_SLAVE_BMA220_ADDR, acc_read, 1);
		bma220_obj->suspend_mode = 1;
	}
	return 0;
}

int bma220_resume(struct emxx_sensors_pub* sensors_pub, struct emxx_sensors_obj* sensors_obj)
{
	struct bma220_obj *bma220_obj = (struct bma220_obj *)sensors_obj;
	if(bma220_obj->suspend_mode) {
		u8 acc_read[2];
		acc_read[0] = 0x18;
		acc_read[1] = 0;
		sensors_pub->i2c_read(sensors_pub, I2C_SLAVE_BMA220_ADDR, acc_read, 1);
		bma220_obj->suspend_mode = 0;
	}
	//printk("[%s:%d] \n", __func__, __LINE__);
	return 0;
}

int bma220_remove(struct emxx_sensors_pub *sensors_pub, struct emxx_sensors_obj* sensors_obj)
{
	struct bma220_obj *bma220_obj = (struct bma220_obj *)sensors_obj;
	bma220_suspend(sensors_pub, sensors_obj);
	kfree(bma220_obj);
	return 0;
}

struct emxx_sensors_obj* bma220_probe(struct emxx_sensors_pub* sensors_pub)
{
	int ret;
	struct bma220_obj *bma220_obj;
	struct emxx_sensors_obj *sensors_obj;
	if(sensors_pub->has_sensors & SENSORS_ACCELERATION)
		return NULL;

	bma220_obj = kmalloc(sizeof(*bma220_obj), GFP_KERNEL);
	if(bma220_obj == NULL)
		return NULL;
	memset(bma220_obj, 0x00, sizeof(*bma220_obj));
	sensors_obj = &bma220_obj->sensors_obj;
	sensors_obj->sensors_type = SENSORS_TYPE_ACC_BMA220,
	sensors_obj->i2c_addr = I2C_SLAVE_BMA220_ADDR,
	sensors_obj->remove = bma220_remove,
	sensors_obj->open = bma220_open,
	sensors_obj->close = bma220_close,
	sensors_obj->get = bma220_get,
	sensors_obj->suspend = bma220_suspend,
	sensors_obj->resume = bma220_resume,
	sensors_obj->set_delay = bma220_set_delay,
	sensors_obj->calibrate = bma220_calibrate,

	bma220_obj->suspend_mode = 0;

	return (struct emxx_sensors_obj *)bma220_obj;
}

