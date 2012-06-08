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
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/slab.h>
#ifdef CONFIG_MACH_EMEV
#include <mach/smu.h>
#include <linux/delay.h>
#endif

#define DRIVER_VERSION "v1.0"
#define DRIVER_AUTHOR "Bee<http://www.pixcir.com.cn>"
#define DRIVER_DESC "Pixcir I2C Touchscreen Driver"
#define DRIVER_LICENSE "GPL"

/* #define DEBUG 1 */
#define	printk(x...) printk("emxx_ts:" x)

#define TOUCHSCREEN_MINX 0
#define TOUCHSCREEN_MAXX 1200
#define TOUCHSCREEN_MINY 0
#define TOUCHSCREEN_MAXY 600

#define GPIO_TOUCHSCREEN_RESET GPIO_P97

static struct workqueue_struct *pixcir_wq;

struct pixcir_i2c_ts_data{
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
	int irq;
	struct workqueue_struct *reset_wq;
	struct work_struct reset_work;
};

static struct pixcir_i2c_ts_data *m_ts;

void touchscreen_calibrate(void);

#ifdef CONFIG_MACH_EMEV
static void pixcir_do_reset(int oning)
{
	if(oning) {
		gpio_direction_output(GPIO_TOUCHSCREEN_RESET, 0);
	} else {
		writel((readl(CHG_PULL10) & 0xff0fffff), CHG_PULL10);
		writel((readl(CHG_PINSEL_G096) | 0x2), CHG_PINSEL_G096);
		gpio_direction_input(GPIO_TOUCHSCREEN_RESET);
	}
}

static void pixcir_reset(int delayMS)
{
	pixcir_do_reset(1);
	mdelay(delayMS);
	pixcir_do_reset(0);
}

static inline void emxx_ts_connect(void)
{
#ifdef DEBUG
	printk("emxx_ts_connect - set gpio\n");
#endif
	pixcir_reset(300);

	gpio_direction_input(GPIO_P29);
	//set_irq_type(INT_GPIO_29, IRQ_TYPE_EDGE_FALLING);

#ifdef DEBUG
	printk( "CHG_PULL1 = 0x%08x\n", readl(CHG_PULL1));
#endif
	writel((readl(CHG_PULL1) | 0x00005000), CHG_PULL1);
#ifdef DEBUG
	printk( "CHG_PULL1 = 0x%08x\n", readl(CHG_PULL1));
#endif
}

static void resume_reset(struct work_struct *work)
{
	printk("pixcir touchscreen resume reset\n\n\n");
	pixcir_reset(300);
}
#endif

static void pixcir_ts_poscheck(struct work_struct *work)
{
	struct pixcir_i2c_ts_data *tsdata = container_of(work,
						  struct pixcir_i2c_ts_data,
						  work.work);

	unsigned char touching, oldtouching;
	int posx1, posy1,posx2, posy2;
	u_int8_t Rdbuf[10],Wrbuf[1];
	int ret;
	int z=50;
	int w=15;

	memset(Wrbuf, 0, sizeof(Wrbuf));
	memset(Rdbuf, 0, sizeof(Rdbuf));

	Wrbuf[0] = 0;

	ret = i2c_master_send(tsdata->client, Wrbuf, 1);

	#ifdef DEBUG
	printk("master send ret:%d\n",ret);
	#endif

	if(ret!=1){
		dev_err(&tsdata->client->dev, "Unable to write to i2c touchscreen!\n");
		goto out;
	}

	ret = i2c_master_recv(tsdata->client,Rdbuf,sizeof(Rdbuf));

	#ifdef DEBUG
	printk("master recv ret:%d\n",ret);
	#endif

	if(ret!=sizeof(Rdbuf)){
		dev_err(&tsdata->client->dev, "Unable to read i2c page!\n");
		goto out;
	}

	touching=Rdbuf[0];
	oldtouching=Rdbuf[1];
	posx1 = ((Rdbuf[3] << 8) | Rdbuf[2]);
	posy1 = ((Rdbuf[5] << 8) | Rdbuf[4]);
	posx2 = ((Rdbuf[7] << 8) | Rdbuf[6]);
	posy2 = ((Rdbuf[9] << 8) | Rdbuf[8]);

	posx1 = TOUCHSCREEN_MAXX - posx1;
	posx2 = TOUCHSCREEN_MAXX - posx2;

	#ifdef DEBUG
	printk("touching:%-3d,oldtouching:%-3d,x1:%-6d,y1:%-6d,"
         "x2:%-6d,y2:%-6d\n",
         touching,oldtouching,posx1,posy1,
         posx2,posy2);
	#endif

	input_report_key(tsdata->input, BTN_TOUCH, (touching!=0?1:0));
	input_report_key(tsdata->input, BTN_2, (touching==2?1:0));

	if(touching){
		input_report_abs(tsdata->input, ABS_X, posx1);
		input_report_abs(tsdata->input, ABS_Y, posy1);
	}

	if (touching==2) {
		input_report_abs(tsdata->input, ABS_HAT0X, posx2);
		input_report_abs(tsdata->input, ABS_HAT0Y, posy2);
	}
	if(!(touching)){
		z=0;
		w=0;
	}
	input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, z);
	input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, w);
	input_report_abs(tsdata->input, ABS_MT_POSITION_X, posx1);
	input_report_abs(tsdata->input, ABS_MT_POSITION_Y, posy1);
	input_mt_sync(tsdata->input);
	if (touching==2) {
		input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, z);
		input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, w);
		input_report_abs(tsdata->input, ABS_MT_POSITION_X, posx2);
		input_report_abs(tsdata->input, ABS_MT_POSITION_Y, posy2);
		input_mt_sync(tsdata->input);
	}
	input_sync(tsdata->input);

	out:
		enable_irq(tsdata->irq);
}

static irqreturn_t pixcir_ts_isr(int irq,void *dev_id)
{
	struct pixcir_i2c_ts_data *tsdata=dev_id;
	disable_irq_nosync(irq);

	queue_work(pixcir_wq,&tsdata->work.work);
	//schedule_delayed_work(&tsdata->work, HZ/1000);//HZ = 100(2.6.32.2)

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
#ifdef DEBUG	
	printk("pixcir_i2c_ts_probe\n");
#endif

	struct pixcir_i2c_ts_data *tsdata;
	struct input_dev *input;
	int error;
	tsdata=kzalloc(sizeof(*tsdata),GFP_KERNEL);
	if(!tsdata){
		dev_err(&client->dev,"failed to allocate driver data!\n");
		error = -ENOMEM;
		dev_set_drvdata(&client->dev, NULL);
		return error;
	}
	m_ts = tsdata;

	dev_set_drvdata(&client->dev, tsdata);

	input = input_allocate_device();
	if(!input){
		dev_err(&client->dev,"failed to allocate input device!\n");
		error = -ENOMEM;
		input_free_device(input);
		kfree(tsdata);
	}

	set_bit(EV_SYN, input->evbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(EV_ABS, input->evbit);
	set_bit(BTN_TOUCH, input->keybit);
	set_bit(BTN_2, input->keybit);
	input_set_abs_params(input, ABS_X, TOUCHSCREEN_MINX, TOUCHSCREEN_MAXX, 0, 0);
	input_set_abs_params(input, ABS_Y, TOUCHSCREEN_MINY, TOUCHSCREEN_MAXY, 0, 0);
	input_set_abs_params(input, ABS_HAT0X, TOUCHSCREEN_MINX, TOUCHSCREEN_MAXX, 0, 0);
	input_set_abs_params(input, ABS_HAT0Y, TOUCHSCREEN_MINY, TOUCHSCREEN_MAXY, 0, 0);
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

	if(input_register_device(input)){
		input_free_device(input);
 		kfree(tsdata);
	}

#ifdef CONFIG_MACH_EMEV
	emxx_ts_connect();
#endif

	tsdata->reset_wq = create_singlethread_workqueue("pixcir_reset");
	if(!tsdata->reset_wq)
		dev_err(&client->dev, "Unable to create workqueue,\
				touchscreen may not work after resume\n");

	INIT_WORK(&tsdata->reset_work, resume_reset);

	INIT_WORK(&tsdata->work.work,pixcir_ts_poscheck);
	//INIT_DELAYED_WORK(&tsdata->work, pixcir_ts_poscheck);

	tsdata->irq = client->irq;
#ifdef DEBUG
	printk("irq number is %d\n", tsdata->irq);
#endif
	if(request_irq(tsdata->irq,pixcir_ts_isr,IRQF_TRIGGER_FALLING,client->name,tsdata)){
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		input_unregister_device(input);
		input=NULL;
	}

	device_init_wakeup(&client->dev, 0);

	dev_err(&tsdata->client->dev,"insmod successfully!\n");
#if 0
	//hengai 2011-03-29 Press VolumeUp+Power
	if( (gpio_get_value(GPIO_P13)==0) && (gpio_get_value(GPIO_P13)==0) ) {
		printk("touchpanel calibrate ....\n");
		touchscreen_calibrate();
	}
#endif
	return 0;
}

static int pixcir_i2c_ts_remove(struct i2c_client *client)
{
	struct pixcir_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
	free_irq(tsdata->irq, tsdata);
	if(tsdata->reset_wq)
		destroy_workqueue(tsdata->reset_wq);
	input_unregister_device(tsdata->input);
	kfree(tsdata);
	dev_set_drvdata(&client->dev, NULL);
	return 0;
}

static int pixcir_i2c_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct pixcir_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
	if (device_may_wakeup(&client->dev))
		enable_irq_wake(tsdata->irq);

	return 0;
}

static int pixcir_i2c_ts_resume(struct i2c_client *client)
{
	struct pixcir_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);

	queue_work(tsdata->reset_wq, &tsdata->reset_work);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(tsdata->irq);

	return 0;
}

void touchscreen_calibrate(void)
{
	unsigned char cmd[2] = {0x37, 0x03};
	if(m_ts && m_ts->client) {
		i2c_master_send(m_ts->client, cmd, 2);
		msleep(8000);
		pixcir_reset(300);
	}
}
EXPORT_SYMBOL(touchscreen_calibrate);

static const struct i2c_device_id pixcir_i2c_ts_id[] = {
	{ "pixcir", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pixcir_i2c_ts_id);

static struct i2c_driver pixcir_i2c_ts_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "pixcir i2c touchscreen driver",
	 },
	.probe = pixcir_i2c_ts_probe,
	.remove = pixcir_i2c_ts_remove,
	.suspend = pixcir_i2c_ts_suspend,
	.resume = pixcir_i2c_ts_resume,
	.id_table = pixcir_i2c_ts_id,
};

static int __init pixcir_i2c_ts_init(void)
{
	#ifdef DEBUG
		printk("pixcir_i2c_init\n");
	#endif
	int ret = 0;

	pixcir_wq = create_singlethread_workqueue("pixcir_wq");
	if(!pixcir_wq)
		return -ENOMEM;
	ret = i2c_add_driver(&pixcir_i2c_ts_driver);
	printk("pixcir:i2c_add_driver %d \n",ret);
	return ret;
}

static void __exit pixcir_i2c_ts_exit(void)
{
	#ifdef DEBUG
		printk("pixcir_i2c_exit\n");
	#endif

	i2c_del_driver(&pixcir_i2c_ts_driver);
	if(pixcir_wq)
		destroy_workqueue(pixcir_wq);
}

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);

module_init(pixcir_i2c_ts_init);
module_exit(pixcir_i2c_ts_exit);
