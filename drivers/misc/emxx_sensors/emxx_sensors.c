#include <linux/slab.h>
#include "emxx_sensors.h"
#define I2C_RETRY_DELAY	5
#define I2C_RETRIES		5

#if 0
#define db_sensor(x, arg...) printk("[%s:%d]" x "\n", __FUNCTION__, __LINE__, ##arg)
#else
#define db_sensor(x, arg...) do{}while(0)
#endif
extern void axp192_Gsensor(int on);
struct emxx_sensors_data {
	struct emxx_sensors_pub sensors_pub;
	struct mutex lock;
	//struct delayed_work input_work;
	struct workqueue_struct *work_queue;
	struct delayed_work work_node;
	int poll_interval;	//
	int min_interval;	//

	atomic_t enabled;
	atomic_t open_count;
	struct input_dev *input;
};
struct emxx_sensors_data *g_sensors_data = NULL;
////////////////////////////////////////////////////////////////////////////////
int emxx_sensors_get_type(void)
{
	int type = 0;
	if(g_sensors_data) {
		if(g_sensors_data->sensors_pub.sensor_acc)
			type |= g_sensors_data->sensors_pub.sensor_acc->sensors_type;
		if(g_sensors_data->sensors_pub.sensor_mag)
			type |= g_sensors_data->sensors_pub.sensor_mag->sensors_type;
	}
	return type;
}

void emxx_gsensor_calibrate(int cali[4])
{
	struct emxx_sensors_data *sensors_data = g_sensors_data;
	if(g_sensors_data == NULL){
		printk("ERR: no sensor for calibrating\n");
		return;
	}

	if(sensors_data->sensors_pub.sensor_acc){
		sensors_data->sensors_pub.sensor_acc->calibrate(&sensors_data->sensors_pub, sensors_data->sensors_pub.sensor_acc, cali);
	}
}

int emxx_gsensor_range(void)
{
	return 16;
}

int msensor_range(void)
{
	return 8100;	//LSM303 MAG range
}

static void emxx_sensors_lock(struct emxx_sensors_pub* sensors_pub)
{
	//mutex_lock(& ((struct emxx_sensors_data*)sensors_pub)->lock);
}

static void emxx_sensors_unlock(struct emxx_sensors_pub* sensors_pub)
{
	//mutex_unlock(& ((struct emxx_sensors_data*)sensors_pub)->lock);
}

static int emxx_sensors_i2c_read(struct emxx_sensors_pub*sensors_data, 
	int addr, u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = addr,
		 .flags = I2C_M_TEN,
		 .len = 1,
		 .buf = buf,
		 },
		{
		 .addr = addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(i2c_get_adapter(0), msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int emxx_sensors_i2c_read2(struct emxx_sensors_pub*sensors_data, 
	int addr, u8 reg, u8 *val, int len)
{
	u8 buf[64];
	if(len >= 64)
		return -1;
	buf[0] = reg;
	memcpy(buf+1, val, len);
	return emxx_sensors_i2c_read(sensors_data, addr, buf, len);
}

static int emxx_sensors_i2c_write(struct emxx_sensors_pub*sensors_data, 
	int addr, u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = addr,
		 .flags = 0,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(i2c_get_adapter(0), msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int emxx_sensors_i2c_write2(struct emxx_sensors_pub*sensors_data, 
	int addr, u8 reg, u8 *val, int len)
{
	u8 buf[64];
	if(len >= 64)
		return -1;
	buf[0] = reg;
	memcpy(buf+1, val, len);
	return emxx_sensors_i2c_write(sensors_data, addr, buf, len);
}

static void emxx_sensors_report(struct input_dev *input, int val[3], int32_t sensor_type)
{
	db_sensor("%d,%d,%d", val[0], val[1], val[2]);
	if(sensor_type & SENSORS_ACCELERATION) {
		input_report_abs(input, EVENT_TYPE_ACCEL_X, val[0]);
		input_report_abs(input, EVENT_TYPE_ACCEL_Y, val[1]);
		input_report_abs(input, EVENT_TYPE_ACCEL_Z, val[2]);
	} else if(sensor_type & SENSORS_MAGNETIC_FIELD) {
		input_report_abs(input, EVENT_TYPE_MAGV_X, val[0]);
		input_report_abs(input, EVENT_TYPE_MAGV_Y, val[1]);
		input_report_abs(input, EVENT_TYPE_MAGV_Z, val[2]);
		input_report_abs(input, EVENT_TYPE_MAGV_OK, jiffies);
	}
	input_sync(input);
}

static void emxx_sensors_input_work_func(struct work_struct *work)
{
	struct emxx_sensors_data* sensors_data = container_of((struct delayed_work *)work,
			    struct emxx_sensors_data, work_node);
	struct emxx_sensors_pub* sensors_pub = (struct emxx_sensors_pub*)&sensors_data->sensors_pub;

	emxx_sensors_lock(sensors_pub);
	if(sensors_pub->sensor_acc) {
		sensors_pub->sensor_acc->get(sensors_pub, sensors_pub->sensor_acc);
		emxx_sensors_report(sensors_data->input, sensors_data->sensors_pub.sensor_acc->val, sensors_data->sensors_pub.sensor_acc->sensors_type);
	}
	if(sensors_pub->sensor_mag) {
		sensors_pub->sensor_mag->get(sensors_pub, sensors_pub->sensor_mag);
		emxx_sensors_report(sensors_data->input, sensors_data->sensors_pub.sensor_mag->val, sensors_data->sensors_pub.sensor_mag->sensors_type);
	}
	emxx_sensors_unlock(sensors_pub);
	queue_delayed_work(sensors_data->work_queue, &(sensors_data->work_node), msecs_to_jiffies(sensors_data->poll_interval));
}

static int emxx_sensors_set_delay(struct emxx_sensors_data* sensors_data, int ms)
{
	struct emxx_sensors_pub* sensors_pub = (struct emxx_sensors_pub*)&sensors_data->sensors_pub;
	sensors_data->poll_interval = max(ms, sensors_data->min_interval);
	if(sensors_pub->sensor_acc && sensors_pub->sensor_acc->set_delay)
		sensors_pub->sensor_acc->set_delay(sensors_pub, sensors_pub->sensor_acc, sensors_data->poll_interval);
	if(sensors_pub->sensor_mag && sensors_pub->sensor_mag->set_delay)
		sensors_pub->sensor_mag->set_delay(sensors_pub, sensors_pub->sensor_mag, sensors_data->poll_interval);
	return 0;
}

static int emxx_sensors_enable(struct emxx_sensors_data* sensors_data)
{
	emxx_sensors_set_delay(sensors_data, sensors_data->poll_interval);
	queue_delayed_work(sensors_data->work_queue, &(sensors_data->work_node), msecs_to_jiffies(sensors_data->poll_interval));
	return 0;
}

static int emxx_sensors_disable(struct emxx_sensors_data* sensors_data)
{
	if(cancel_delayed_work(&sensors_data->work_node) != 0){
		flush_workqueue(sensors_data->work_queue);
	}
	return 0;
}

static int emxx_sensors_ioctl(struct inode *inode, struct file *filp, 
							unsigned int cmd, unsigned long arg)
{
	struct emxx_sensors_data *sensors_data = g_sensors_data;
	int ret = 0;
	int val;
	//printk("[%s:%d] has_sensors:%x, open_count:%d cmd:%x\n", __func__, __LINE__, sensors_data->sensors_pub.has_sensors, atomic_read(&sensors_data->open_count), cmd);
	switch(cmd) {
	case SENSORS_ACC_IOCTL_GET_DELAY:
		ret = 0;
		val = sensors_data->poll_interval;
		if (copy_to_user((void __user*)arg, &val, sizeof(int)))
			ret = -EFAULT;
		break;
	case SENSORS_ACC_IOCTL_SET_DELAY:
		if (0 == copy_from_user(&val, (const void __user*)arg, sizeof(int))) {
			ret = emxx_sensors_set_delay(sensors_data, val);
		} else {
			ret = -EFAULT;
		}
		break;
	case SENSORS_ACC_IOCTL_SET_ENABLE:
	case SENSORS_MAG_IOCTL_SET_ENABLE:
	case SENSORS_ORI_IOCTL_SET_ENABLE:
		ret = 0;
		break;
	case SENSORS_ACC_IOCTL_GET_ENABLE:
	case SENSORS_MAG_IOCTL_GET_ENABLE:
	case SENSORS_ORI_IOCTL_GET_ENABLE:
		val = atomic_read(&sensors_data->open_count)>0;
		if (copy_to_user((void __user *)arg, &val, sizeof(val)))
			return -EFAULT;
		ret = 0;
		break;
	}
	return ret;
}

static int emxx_sensors_release(struct inode *inode, struct file *filp)
{
	return 0;
}
static int emxx_sensors_open(struct inode *inode, struct file *filp)
{
	if(g_sensors_data == NULL)
		return -1;
	return 0;
}

struct file_operations emxx_sensors_fops =
{
	.owner		= THIS_MODULE,
	.open		= emxx_sensors_open,
	.release	= emxx_sensors_release,
	.ioctl		= emxx_sensors_ioctl,
};

static struct miscdevice emxx_sensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = EMXX_SENSORS_DEV_NAME,
	.fops = &emxx_sensors_fops
};

static int emxx_sensors_create_file_ops(struct emxx_sensors_data* sensors_data)
{
	int ret;
	ret = misc_register(&emxx_sensor_misc);
	if (ret < 0) {
		pr_err("%s: can not register misc device\n",
				__func__);
		return ret;
	}

	return 0;
}

int emxx_sensors_input_open(struct input_dev *dev)
{
	int ret;
	struct emxx_sensors_data *sensors_data = g_sensors_data;
	db_sensor();
	if( (sensors_data==NULL) || (sensors_data->sensors_pub.has_sensors == 0) ) {
		return -ENXIO;
	}
	if(atomic_add_return(1, &sensors_data->open_count) > 1) {
		//printk("    open count: %d\n", atomic_read(&sensors_data->open_count));
		return 0;
	}
	//printk("    open count: %d\n", atomic_read(&sensors_data->open_count));
	axp192_Gsensor(1);
	if(sensors_data->sensors_pub.sensor_acc) {
		ret = sensors_data->sensors_pub.sensor_acc->open(
		&sensors_data->sensors_pub, sensors_data->sensors_pub.sensor_acc);
		if(ret) {
			sensors_data->sensors_pub.has_sensors &= ~SENSORS_ACCELERATION;
			sensors_data->sensors_pub.sensor_acc->remove(&sensors_data->sensors_pub, sensors_data->sensors_pub.sensor_acc);
			sensors_data->sensors_pub.sensor_acc = NULL;
		}
	}
	if(sensors_data->sensors_pub.sensor_mag) {
		ret = sensors_data->sensors_pub.sensor_mag->open(
		&sensors_data->sensors_pub, sensors_data->sensors_pub.sensor_mag);
		if(ret) {
			sensors_data->sensors_pub.has_sensors &= ~SENSORS_MAGNETIC_FIELD;
			sensors_data->sensors_pub.sensor_mag->remove(&sensors_data->sensors_pub, sensors_data->sensors_pub.sensor_mag);
			sensors_data->sensors_pub.sensor_mag = NULL;
		}
	}
	emxx_sensors_enable(sensors_data);
	db_sensor();
	return 0;
}
void emxx_sensors_input_close(struct input_dev *dev)
{
	struct emxx_sensors_data *sensors_data = g_sensors_data;
	db_sensor();
	if(atomic_sub_return(1, &sensors_data->open_count) <= 0) {
		atomic_set(&sensors_data->open_count, 0);
		emxx_sensors_disable(sensors_data);
		if(sensors_data->sensors_pub.sensor_acc) {
			sensors_data->sensors_pub.sensor_acc->close(
			&sensors_data->sensors_pub, sensors_data->sensors_pub.sensor_acc);
		}
		if(sensors_data->sensors_pub.sensor_mag) {
			sensors_data->sensors_pub.sensor_mag->close(
			&sensors_data->sensors_pub, sensors_data->sensors_pub.sensor_mag);
		}
	}
}


static emxx_sensors_probe arr_sensors_probe[] = {
#if defined(CONFIG_INPUT_BMA220_G_SENSORS)
	bma220_probe,
#endif
};
static int __devinit emxx_sensors_platform_probe(struct platform_device *pdev)
{
	int ret, i;
	struct emxx_sensors_data *sensors_data = NULL;
	struct emxx_sensors_pub *sensors_pub;
	struct input_dev *input;
	sensors_data = kmalloc(sizeof(struct emxx_sensors_data), GFP_KERNEL);
	if(sensors_data == NULL)
		goto fail_malloc;
	memset(sensors_data, 0x00, sizeof(*sensors_data));
	sensors_pub = (struct emxx_sensors_pub *)sensors_data;
	ret = emxx_sensors_create_file_ops(sensors_data);
	if(ret != 0)
		goto fail_file_ops;
	input = input_allocate_device();
	if(!input)
		goto fail_probe;
	sensors_data->input = input;
	//open power
	axp192_Gsensor(1);

	sensors_data->sensors_pub.i2c_adapter = i2c_get_adapter(0);
	sensors_data->sensors_pub.lock = emxx_sensors_lock;
	sensors_data->sensors_pub.unlock = emxx_sensors_unlock;
	sensors_data->sensors_pub.i2c_read = emxx_sensors_i2c_read;
	sensors_data->sensors_pub.i2c_read2 = emxx_sensors_i2c_read2;
	sensors_data->sensors_pub.i2c_write = emxx_sensors_i2c_write;
	sensors_data->sensors_pub.i2c_write2 = emxx_sensors_i2c_write2;
	sensors_data->poll_interval = 200;
	sensors_data->min_interval = 30;

	for(i=0; i<sizeof(arr_sensors_probe)/sizeof(arr_sensors_probe[0]); i++) {
		struct emxx_sensors_obj *sensors_obj = arr_sensors_probe[i](sensors_pub);
		if(sensors_obj != NULL) {
			if(sensors_obj->sensors_type & SENSORS_ACCELERATION) {
				sensors_pub->sensor_acc = sensors_obj;
				sensors_pub->has_sensors |= SENSORS_ACCELERATION;
			} else if(sensors_obj->sensors_type & SENSORS_MAGNETIC_FIELD) {
				sensors_pub->sensor_mag = sensors_obj;
				sensors_pub->has_sensors |= SENSORS_MAGNETIC_FIELD;
			}
		}
	}
	if(sensors_pub->has_sensors == 0)
		goto fail_probe;

	//create work_queue
	sensors_data->work_queue = create_singlethread_workqueue("emxx_sensors");
	if(sensors_data->work_queue == NULL)
		goto fail_input;

	INIT_DELAYED_WORK(&sensors_data->work_node, emxx_sensors_input_work_func); 
	//craete mutex
	mutex_init(&sensors_data->lock);

	atomic_set(&sensors_data->open_count, 0);
	atomic_set(&sensors_data->enabled, 0);
	platform_set_drvdata(pdev, sensors_data);

	g_sensors_data = sensors_data;

	input->open = emxx_sensors_input_open;
	input->close = emxx_sensors_input_close;
	set_bit(EV_ABS, input->evbit);
	if (sensors_pub->has_sensors & SENSORS_ACCELERATION) 
	{
		db_sensor("emxx-sensors - setting acceleration sensor ranges");
		input_set_abs_params(input, EVENT_TYPE_ACCEL_X, -emxx_gsensor_range(), emxx_gsensor_range(), 0, 0);
		input_set_abs_params(input, EVENT_TYPE_ACCEL_Y, -emxx_gsensor_range(), emxx_gsensor_range(), 0, 0);
		input_set_abs_params(input, EVENT_TYPE_ACCEL_Z, -emxx_gsensor_range(), emxx_gsensor_range(), 0, 0);
	}
	if (sensors_pub->has_sensors & SENSORS_MAGNETIC_FIELD) 
	{
		db_sensor("emxx-sensors - setting magnetic sensor ranges");
		input_set_abs_params(input, EVENT_TYPE_MAGV_X, -msensor_range(), msensor_range(), 0, 0);
		input_set_abs_params(input, EVENT_TYPE_MAGV_Y, -msensor_range(), msensor_range(), 0, 0);
		input_set_abs_params(input, EVENT_TYPE_MAGV_Z, -msensor_range(), msensor_range(), 0, 0);	
	}
	input->name = "emxx-sensors";
	input->id.bustype = BUS_HOST;
	input->dev.parent = &pdev->dev;
	input_set_drvdata(input, g_sensors_data);
	if(input_register_device(input)){
	}

	return 0;
fail_input:
fail_probe:
	misc_deregister(&emxx_sensor_misc);
fail_file_ops:
	kfree(sensors_data);
fail_malloc:
	axp192_Gsensor(0);
	return -1;
}

static int __devexit emxx_sensors_platform_remove(struct platform_device *pdev)
{
	struct emxx_sensors_data *sensors_data = platform_get_drvdata(pdev);
	misc_deregister(&emxx_sensor_misc);
	if(sensors_data->sensors_pub.sensor_acc)
		sensors_data->sensors_pub.sensor_acc->remove(&sensors_data->sensors_pub, sensors_data->sensors_pub.sensor_acc);
	if(sensors_data->sensors_pub.sensor_mag)
		sensors_data->sensors_pub.sensor_mag->remove(&sensors_data->sensors_pub, sensors_data->sensors_pub.sensor_mag);
	mutex_destroy(&sensors_data->lock);
	kfree(sensors_data);
	sensors_data = NULL;
	return 0;
}

static int emxx_sensors_platform_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct emxx_sensors_data *sensors_data = platform_get_drvdata(pdev);
	if(sensors_data->sensors_pub.sensor_acc)
		sensors_data->sensors_pub.sensor_acc->suspend(&sensors_data->sensors_pub, sensors_data->sensors_pub.sensor_acc);
	if(sensors_data->sensors_pub.sensor_mag)
		sensors_data->sensors_pub.sensor_mag->suspend(&sensors_data->sensors_pub, sensors_data->sensors_pub.sensor_mag);
	return 0;
}
static int emxx_sensors_platform_resume(struct platform_device *pdev)
{
	struct emxx_sensors_data *sensors_data = platform_get_drvdata(pdev);
	if(sensors_data->sensors_pub.sensor_acc)
		sensors_data->sensors_pub.sensor_acc->resume(&sensors_data->sensors_pub, sensors_data->sensors_pub.sensor_acc);
	if(sensors_data->sensors_pub.sensor_mag)
		sensors_data->sensors_pub.sensor_mag->resume(&sensors_data->sensors_pub, sensors_data->sensors_pub.sensor_mag);
	return 0;
}

static struct platform_driver emxx_sensors_platform_driver = {
	.probe		= emxx_sensors_platform_probe,
	.remove		= emxx_sensors_platform_remove,
	.suspend	= emxx_sensors_platform_suspend,
	.resume		= emxx_sensors_platform_resume,
	.driver		= {
		.name	= EMXXSENSORS_PLATFORM_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init emxx_sensors_init(void)
{
	int ret;

	//Create platform driver, to suspend/resume
	ret = platform_driver_register(&emxx_sensors_platform_driver);

	return ret;
}

static void __exit emxx_sensors_exit(void)
{
	platform_driver_unregister(&emxx_sensors_platform_driver);
	return;
}

late_initcall_sync(emxx_sensors_init);
module_exit(emxx_sensors_exit);

MODULE_DESCRIPTION("emxx sensors driver");
MODULE_AUTHOR("hengai");
