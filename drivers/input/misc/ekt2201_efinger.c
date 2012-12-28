/* drivers/input/misc/ekt2201_efinger.c
 *
 * Copyright (C) 2010 Livall, Inc.
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
//#include <linux/delay.h>
#ifdef CONFIG_MACH_EMEV
#include <mach/smu.h>
#endif

//#define DEBUG 1
#define	printk(x...) printk("touch_button:" x)


#define DRIVER_VERSION "v1.0"
#define DRIVER_AUTHOR "Xieyes<xieyes@gmail.com>"
#define DRIVER_DESC "ELAN I2C Button Touchpad Controller Driver"
#define DRIVER_LICENSE "GPL"

#define PRODUCT_ID	0x22

#define KEY_RELEASED    0
#define KEY_PRESSED     1
#define ARRAY_OF(arr) (sizeof(arr)/sizeof(arr[0]))

#define EARLY_SUSPEND 1
#if EARLY_SUSPEND
	#include <linux/earlysuspend.h>
	static int irq;
	static struct early_suspend early_suspend;
#endif

struct key_data{
    const unsigned short register_value;  // reading from 0x11 or 0x12
    const unsigned int virtual_key;
    int old_state;
};

struct key_data keys_array[] = {
        {0x0008, KEY_BACK  , KEY_RELEASED},
        {0x0100, KEY_SEARCH, KEY_RELEASED},
        {0x0001, KEY_MENU  , KEY_RELEASED},
        {0x0002, KEY_HOME  , KEY_RELEASED},
};

static struct workqueue_struct *tb_wq;
struct tb_i2c_data{
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
	int irq;
};

#ifdef CONFIG_MACH_EMEV
static inline void emxx_tb_connect(void)
{
	gpio_direction_input(GPIO_P102);
	writel(((readl(CHG_PULL11) & 0xfffff0ff) | 0x500), CHG_PULL11);
	writel((readl(CHG_PINSEL_G096) | 0x40), CHG_PINSEL_G096);
}
#endif

static int I2C_SEND_CMD(struct i2c_client *client,
	unsigned char reg, unsigned char val)
{
	unsigned char cmd[2];
	cmd[0] = reg;
	cmd[1] = val;	
	return i2c_master_send(client, cmd, 2);
}

static int I2C_READ_DAT(struct i2c_client *client,
	unsigned char reg, unsigned char *value)
{
	int ret;

	ret = i2c_master_send(client, &reg, 1);
	if(ret != 1){
		dev_err(&client->dev, "write i2c failed\n");
		return ret;
	}
	ret = i2c_master_recv(client, value, 1);
	if(ret != 1){
		dev_err(&client->dev, "read i2c failed\n");
		return ret;
	}

	return 1;
}

/*static int I2C_READ_MULTI(struct i2c_client *client,
	unsigned char reg, unsigned char *buf, int size)
{
	int ret;

	ret = i2c_master_send(client, &reg, 1);
	if(ret != 1){
		dev_err(&client->dev, "write i2c failed\n");
		return ret;
	}

	return i2c_master_recv(client, buf, size);
}*/

static int ekt2201_register_init(struct i2c_client *client)
{
	unsigned char Rdbuf;
	int ret;

	ret = I2C_READ_DAT(client, 0x0, &Rdbuf); // Address 0x00: Product ID
	if ((ret == 1) && (PRODUCT_ID == Rdbuf))
		printk( "eKT2201 Detected(Product ID:0x%x)\n", Rdbuf);
	else
		return -ENODEV;


	// set unrestricted mode, user can touch any combined button
	ret = I2C_READ_DAT(client, 0x07, &Rdbuf); // Buttonconfigure
	if(ret != 1){
		goto err;
	}

	ret = I2C_SEND_CMD(client, 0x07, Rdbuf | 0x80);
	if(ret != 2){
		dev_err(&client->dev, "Unable send cmd to touch button!\n");
		goto err;
	}
	I2C_SEND_CMD(client, 0x04, 0x40); //TPREQB mode
	return 0;
err:
	return -EIO;
}


static void tb_poscheck(struct work_struct *work)
{
	int ret, i, num = ARRAY_OF(keys_array);
	unsigned char Rdbuf[2];
	unsigned short key_reg;
	struct tb_i2c_data *tbdata = container_of(work,
						  struct tb_i2c_data,
						  work.work);
	memset(Rdbuf, 0, sizeof(Rdbuf));

	//msleep(10);
	ret = I2C_READ_DAT(tbdata->client, 0x11, &Rdbuf[0]);
	//msleep(10);
	ret = I2C_READ_DAT(tbdata->client, 0x12, &Rdbuf[1]);
	/*ret = I2C_READ_MULTI(tbdata->client, 0x11, Rdbuf, sizeof(Rdbuf));
	if(ret != sizeof(Rdbuf)) {
		dev_err(&tbdata->client->dev, "Unable to read i2c page!\n");
		goto out;
	}*/

	key_reg = Rdbuf[1] << 8 | Rdbuf[0];

	#ifdef DEBUG
		printk("register key value:0x%x\n", key_reg);
	#endif

	for (i = 0; i < num; i++) {
		int cur_state = (key_reg & keys_array[i].register_value) ? \
			KEY_PRESSED : KEY_RELEASED;
		if (cur_state != keys_array[i].old_state) {
			input_report_key(tbdata->input,
				keys_array[i].virtual_key, cur_state);
			keys_array[i].old_state = cur_state;
			#ifdef DEBUG
			printk("key %d %s\n", keys_array[i].virtual_key,
			cur_state ? "PRESSED" : "RELEASED");
			#endif
		}
	}

	input_sync(tbdata->input);
//out:
	enable_irq(tbdata->irq);
}

static irqreturn_t tb_isr(int irq,void *dev_id)
{
	struct tb_i2c_data *tbdata=dev_id;
	disable_irq_nosync(irq);

	queue_work(tb_wq,&tbdata->work.work);
	//queue_delayed_work(tb_wq, &tbdata->work, HZ/1000);//HZ = 100(2.6.32.2)

	return IRQ_HANDLED;
}

static void ekt_early_suspend(struct i2c_client *client)
{
	disable_irq(irq);
}

static void ekt_late_resume(struct i2c_client *client)
{
	enable_irq(irq);
}

static int tb_open(struct input_dev *dev)
{
	return 0;
}

static void tb_close(struct input_dev *dev)
{
}

static int tb_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct tb_i2c_data *tbdata;
	struct input_dev *input;
	int error;
	
	#ifdef DEBUG	
	printk("tb_i2c_probe\n");
	#endif

	error = ekt2201_register_init(client);
	if (error)
		return error;

	tbdata=kzalloc(sizeof(*tbdata),GFP_KERNEL);
	if(!tbdata){
		dev_err(&client->dev,"failed to allocate driver data!\n");
		error = -ENOMEM;
		dev_set_drvdata(&client->dev, NULL);
		return error;
	}

	dev_set_drvdata(&client->dev, tbdata);

	input = input_allocate_device();
	if(!input){
		dev_err(&client->dev,"failed to allocate input device!\n");
		error = -ENOMEM;
		input_free_device(input);
		kfree(tbdata);
	}

	set_bit(EV_KEY, input->evbit);
	set_bit(KEY_HOME, input->keybit);
	set_bit(KEY_SEARCH, input->keybit);
	set_bit(KEY_MENU, input->keybit);
	set_bit(KEY_BACK, input->keybit);

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	input->open = tb_open;
	input->close = tb_close;

	input_set_drvdata(input, tbdata);

	tbdata->client = client;
	tbdata->input = input;

	INIT_WORK(&tbdata->work.work,tb_poscheck);
	//INIT_DELAYED_WORK(&tbdata->work, tb_poscheck);

	tbdata->irq = client->irq;
#ifdef DEBUG
	printk("irq number is %d\n", tbdata->irq);
#endif

	if(input_register_device(input)){
		input_free_device(input);
 		kfree(tbdata);
	}

#ifdef CONFIG_MACH_EMEV
	emxx_tb_connect();
#endif
	if(request_irq(tbdata->irq,tb_isr,IRQF_TRIGGER_FALLING,client->name,tbdata)){
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		input_unregister_device(input);
		input=NULL;
	}

	device_init_wakeup(&client->dev, 0);

#if EARLY_SUSPEND
	early_suspend.level = 151;
	early_suspend.suspend = ekt_early_suspend;
	early_suspend.resume = ekt_late_resume;
	register_early_suspend(&early_suspend);
	irq = tbdata->irq;
#endif
	printk("probe successfully!\n");

	return 0;
}

static int tb_i2c_remove(struct i2c_client *client)
{
	struct tb_i2c_data *tbdata = dev_get_drvdata(&client->dev);
	free_irq(tbdata->irq, tbdata);
	input_unregister_device(tbdata->input);
	kfree(tbdata);
	dev_set_drvdata(&client->dev, NULL);
#if EARLY_SUSPEND
	unregister_early_suspend(&early_suspend);
#endif
	return 0;
}

static int tb_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct tb_i2c_data *tbdata = dev_get_drvdata(&client->dev);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(tbdata->irq);

	return 0;
}

static int tb_i2c_resume(struct i2c_client *client)
{
	struct tb_i2c_data *tbdata = dev_get_drvdata(&client->dev);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(tbdata->irq);

	return 0;
}

static const struct i2c_device_id tb_i2c_id[] = {
	{ "tb_i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tb_i2c_id);

static struct i2c_driver tb_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "i2c touch button driver",
	 },
	.probe = tb_i2c_probe,
	.remove = tb_i2c_remove,
	.suspend = tb_i2c_suspend,
	.resume = tb_i2c_resume,
	.id_table = tb_i2c_id,
};

static int __init tb_i2c_init(void)
{
	#ifdef DEBUG
		printk("tb_i2c_init\n");
	#endif

	tb_wq = create_singlethread_workqueue("tb_wq");
	if(!tb_wq)
		return -ENOMEM;
	return i2c_add_driver(&tb_i2c_driver);
}

static void __exit tb_i2c_exit(void)
{
	#ifdef DEBUG
		printk("tb_i2c_exit\n");
	#endif

	i2c_del_driver(&tb_i2c_driver);
	if(tb_wq)
		destroy_workqueue(tb_wq);
}

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);

late_initcall_sync(tb_i2c_init);
module_exit(tb_i2c_exit);
