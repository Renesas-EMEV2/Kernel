/*
 * File Name       : drivers/video/emxx/emxx_it6610.c
 * Function        : EMEV HDMI IT6610 interface
 * Release Version : Ver 0.01
 * Release Date    : 2011/03/07
 *
 */

#include <linux/init.h> /* __init */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mm.h>      /* remap_page_range */
#include <linux/slab.h> /* kmalloc */
#include <linux/poll.h> /* POLLIN */
#include <linux/interrupt.h> /* tasklet */
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/io.h>  /* ioremap */
#include <linux/i2c.h>
#include <linux/irq.h>
#include <mach/irqs.h> /* request_irq */
#include <mach/hardware.h> /* HDMI_it6610 base address */
#include <mach/pmu.h> /* clock */
#include <mach/smu.h>
#include <mach/gpio.h>
#include <mach/emxx_hdmi_it6610.h>

#include <linux/workqueue.h>
#include <stdarg.h>

//ligang
#include "../../../../arch/arm/mach-emxx/pm_pmu.h"

#include "hdmitx.h"
extern void InitCAT6611(void);
extern void HDMITX_DevLoopProc(void) ;
extern void HDMITX_ChangeDisplayOption(HDMI_Video_Type VideoMode, HDMI_OutputColorMode OutputColorMode) ;

/*
 * debug functions
 */
//#define CONFIG_EMXX_HDMI_IT6610_DEBUG 1

#ifdef	CONFIG_EMXX_HDMI_IT6610_DEBUG

/* debug functions --------------------------------------------------------- */

static int debug_level = 3;
static const char debug_prefix[] = "";
#define debug_printf printk

static int debug_nowstring(char *buf, int len)
{
	buf[0] = '\0';
	return 0;
}


static void __debug_np(int level, const char *format, ...)
{
	char sbuf[128];
	va_list args;
	if (debug_level < level)
		return;

	va_start(args, format);
	vsnprintf(sbuf, sizeof(sbuf), format, args);
	va_end(args);

	debug_printf("%s%s\n", debug_prefix, sbuf);
}

static void __debug(int level, const char *function, const char *format, ...)
{
	char sbuf[128];
	char times[20];
	va_list args;
	if (debug_level < level)
		return;

	va_start(args, format);
	vsnprintf(sbuf, sizeof(sbuf), format, args);
	va_end(args);

	debug_nowstring(times, sizeof(times));
	debug_printf("%s%s%-16s: %s\n", debug_prefix, times, function, sbuf);
}

static void __dump(int level, const char *function, void *ptr, int len)
{
	unsigned char *p = (unsigned char *)ptr;
	char sbuf[128];
	int i;
	if (debug_level < level)
		return;

	__debug(level, function, "length=%d", len);
	if (len > 32)
		len = 32;

	for (i = 0; i < len;) {
		int nl = 16;
		char *sbufp;

		sbufp = sbuf + sprintf(sbuf, " %04X: ", (unsigned int)(i));
		for (; i < len && nl > 0; i++, nl--)
			sbufp += sprintf(sbufp, "%02X ", p[i]);
		debug_printf("%s\n", sbuf);
	}
}
#endif	/* CONFIG_EMXX_HDMI_IT6610_DEBUG */

#include "emxx_it6610.h"

#define HDMI_IT6610_MODNAME     "it6610"
#define HDMI_IT6610_DEVNAME     "it6610"
#define HDMI_IT6610_MINOR_MAX   1

#define HDMI_IT6610_MAJOR 0

#define IT6610_ADDR	0x4C

static char *devname = HDMI_IT6610_DEVNAME; /* !< default device file name */
static int devmajor = HDMI_IT6610_MAJOR;  /* !< default device major number */

static struct mutex hdmi_it6610_mutex;

/* !< this driver information */
static struct hdmi_it6610_info_t s_hdmi_it6610_info;
struct hdmi_it6610_info_t *hdmi_it6610_info = &s_hdmi_it6610_info;

static DECLARE_WAIT_QUEUE_HEAD(intwaitq);

#if 0
static DECLARE_WAIT_QUEUE_HEAD(readq);
#endif

#ifdef CONFIG_PM
static int hdmi_it6610_pf_suspend(struct platform_device *dev,
 pm_message_t state);
static int hdmi_it6610_pf_resume(struct platform_device *dev);
#endif
static int hdmi_it6610_pf_probe(struct platform_device *dev);
static int hdmi_it6610_pf_remove(struct platform_device *dev);

static void hdmi_it6610_pf_release(struct device *dev);

static struct class *hdmi_it6610_class;
static struct cdev hdmi_it6610_cdev;
static struct device *hdmi_it6610_class_device;

static struct platform_device hdmi_it6610_pf_device = {
	.name = HDMI_IT6610_MODNAME,
	.id = -1,
	.dev = {
		.release = hdmi_it6610_pf_release,
	},
};

static struct platform_driver hdmi_it6610_pf_driver = {
	.probe = hdmi_it6610_pf_probe,
	.remove = hdmi_it6610_pf_remove,
#ifdef CONFIG_PM
	.suspend = hdmi_it6610_pf_suspend,
	.resume = hdmi_it6610_pf_resume,
#endif
	.driver = {
		.name  = HDMI_IT6610_MODNAME,
		.owner = THIS_MODULE,
	},
};

#define RETRY_COUNT(x)  ((loops_per_jiffy * x)/(5000/HZ))

/* prototypes for I2C client definition */

#define IT6610_WRITE(a, d, m)    i2c_hdmi_write(a, d, m)
#define IT6610_READ(a, d)    	 i2c_hdmi_read(a, d)

//#define	IT6610REG_ST	0x0E
//#define	IT6610REG_MAX	0x87

//static unsigned char i2c_hdmi_reg[IT6610REG_MAX];
static int i2c_hdmi_init_done;

static int i2c_hdmi_read(unsigned char reg, unsigned char *data);
static int hdmi_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id);
static int hdmi_i2c_remove(struct i2c_client *client);

static struct i2c_device_id hdmi_i2c_idtable[] = {
	//{ I2C_SLAVE_HDMI_NAME, 0 },
	{ "it6610", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hdmi_i2c_idtable);

static struct i2c_driver i2c_hdmi_driver = {
	.driver.name    = "it6610",
	.id_table       = hdmi_i2c_idtable,
	.probe          = hdmi_i2c_probe,
	.remove         = hdmi_i2c_remove,
};
static struct i2c_client *i2c_hdmi_client;

//TODO:
//ligang 
//static struct  work_struct hdmi_irq_work;
struct delayed_work hdmi_irq_work;

//for gpio-026
#define HDMI_GPIO	GPIO_P6
#define HDMI_IRQ	gpio_to_irq(HDMI_GPIO)
#define  GPIO_CHG_OFFSET  0x304
#define IRQ_NUM  6   //26
//gpio->int setup
static void hdmi_gpio_setup(void)
{
	unsigned int value;
#if 1
	/* set GPIO26 INT  */
	//enable input; pull up/ updown disable
/*	
	value = __raw_readl(IO_ADDRESS(EMXX_CHG_BASE) + GPIO_CHG_OFFSET);
	value = (value & ~(0xf <<20)) | (0x7<<20);
	__raw_writel(value,(IO_ADDRESS(EMXX_CHG_BASE) + GPIO_CHG_OFFSET));
*/
	writel((readl(CHG_PULL13)&0xfff0ffff)|0xd0000, CHG_PULL13); //3 config gpio ,INT configuration
	writel(readl(CHG_PINSEL_G000)|0x40, CHG_PINSEL_G000);

	value = __raw_readl(GIO_000_IDT3);
	//asynchronous falling edge mode 
	//value = (value & ~(0xf <<8)) | (0x9<<8); 
	//low level
	value = (value & ~(0xf <<8)) | (0xB<<8); 
	__raw_writel(value,GIO_000_IDT3);
	
        value = __raw_readl(GIO_000_IIR);
        value |= (0x1<<IRQ_NUM);
        __raw_writel(value,GIO_000_IIR);

        value = __raw_readl(GIO_000_IIA);
        value |= (0x1<<IRQ_NUM);
        __raw_writel(value,GIO_000_IIA);

        value = __raw_readl((IO_ADDRESS(EMXX_GPIO_BASE) + GIO_000_OFFSET + GIO_GSW));
        value |= (0x1<<IRQ_NUM);
        __raw_writel(value,(IO_ADDRESS(EMXX_GPIO_BASE) + GIO_000_OFFSET + GIO_GSW));

        value = __raw_readl(GIO_000_IEN);
        value |= (0x1<<IRQ_NUM);
        __raw_writel(value,GIO_000_IEN);
#endif
}

static void hdmi_irq_enable(void)
{
#if 1
        unsigned int value = 0;
	value = __raw_readl(GIO_000_IIA);
        value |= (0x1<<IRQ_NUM);
        __raw_writel(value,GIO_000_IIA);
#else		
 	writel((readl(CHG_PULL13)&0xfff0ffff)|0xd0000, CHG_PULL13); //3 config gpio ,INT configuration
#endif       
}
static void hdmi_irq_disable(void)
{
#if 1
	unsigned int value = 0;
        value = __raw_readl(GIO_000_IIA);
        value &= ~(0x1<<IRQ_NUM);
        __raw_writel(value,GIO_000_IIA);
#else        
	writel((readl(CHG_PULL13)&0xfff0ffff)|0x40000, CHG_PULL13); //3 config gpio ,INT configuration
#endif        
}


// emulator mcu api
void
DelayMS(USHORT ms)
{
	mdelay(ms);
}

BYTE HDMITX_ReadI2C_Byte(BYTE reg)
{
        int ret = 0;
#if 0
	u8 val;
	u8 mm1[] = {0x00};
	struct i2c_msg msgs[2];
	
	mm1[0] = reg;
	msgs[0].flags = 0;
	msgs[0].addr = i2c_hdmi_client->addr;
	msgs[0].len = 1; 
	msgs[0].buf = mm1; 

	val = 0;
	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = i2c_hdmi_client->addr;
	msgs[1].len = 1;
	msgs[1].buf = &val;

	ret = i2c_transfer(i2c_hdmi_client->adapter, msgs, 2);

	if( ret < 0 ) {
		printk( "readbyte error, ret:%x \n", ret);
	}

	return val;

#else
	ret = i2c_smbus_read_byte_data(i2c_hdmi_client, reg);

	if( ret < 0 ) {
		printk( " read byte error %x\n", ret);
		return 0;
	}

	return ret;
#endif
}

SYS_STATUS HDMITX_WriteI2C_Byte(BYTE reg, BYTE val)
{
        int ret = 0;
#if 0
#else
	ret = i2c_smbus_write_byte_data(i2c_hdmi_client, reg, val);

	if( ret < 0 )
		printk( "---writebyte error %x\n", ret);

	return !ret;
#endif
}

SYS_STATUS HDMITX_ReadI2C_ByteN(BYTE reg, BYTE *pdata, int len)
{
        int ret = 0;
#if 0
#else
	ret = i2c_smbus_read_i2c_block_data(i2c_hdmi_client, reg, len, pdata);
	
	if( ret < 0 )
		printk( "-===readbyteN error %x\n", ret);

	return !ret;
#endif
}

SYS_STATUS HDMITX_WriteI2C_ByteN(SHORT reg, BYTE *pdata, int len)
{
        int ret = 0;
#if 0
#else
	ret = i2c_smbus_write_i2c_block_data(i2c_hdmi_client, reg, len, pdata);

	if( ret < 0 )
		printk( "-===writebyteN error %x\n", ret);
	return !ret;
#endif
}

static void print_all_regs(void)
{
	unsigned int i;
	BYTE val;

	printk("\n=============0x00~0x2F==============\n");

	for (i = 0x00; i < 0x2F; i++ )
	{
		printk( "HDMI reg-0x%2x = %x\n", i, HDMITX_ReadI2C_Byte(i));
	}

	printk("\n===============BANK0================\n");
	HDMITX_WriteI2C_Byte(0xF, 0);	
	for( i = 0x31; i < 0xff; i++)
	{
		val = HDMITX_ReadI2C_Byte(i);
		printk( "BANK0 0x%2x = %x\n", i, val);
	}

	printk("\n===============BANK1================\n");
	HDMITX_WriteI2C_Byte(0xF, 1);	
	for( i = 0x31; i < 0xA2; i++)
	{
		val = HDMITX_ReadI2C_Byte(i);
		printk( "BANK1 0x1%2x = %x\n", i, val);
	}
	HDMITX_WriteI2C_Byte(0xF, 0);
	printk("\n====================================\n");
}

static u32 debug_reg = 0;
static u8  mode = 0;

void switch_bank( unsigned int reg)
{
	int value = 0;

	value = HDMITX_ReadI2C_Byte(0xF);

        if( reg < 0x100 )
	{
		if( value!= 0 ) {
			HDMITX_WriteI2C_Byte(0xF, 0);
			printk( "swith to bank 0\n");
		}
	}
	else
	{
		if( value == 0) {
			HDMITX_WriteI2C_Byte(0xF, 1);
			printk( "swith to bank 1\n");
		}
	}

}

static ssize_t hdmi_show_reg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val;

        switch_bank(debug_reg);
	val = HDMITX_ReadI2C_Byte(debug_reg);

	//print_regs();
	printk( " now reg = 0x%x \n", debug_reg);
	return snprintf(buf, PAGE_SIZE, "%x\n", debug_reg);
}

static ssize_t hdmi_store_reg (struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;
	char *str = buf;
	if (strnicmp(buf, "0x", 2) == 0 || strnicmp(buf, "0X", 2) == 0)
		str = buf + 2;

	sscanf(str, "%lx", &value);

	debug_reg = (unsigned int)value;
	printk( "input %s; set reg to 0x%x \n", buf, debug_reg);

	switch_bank(debug_reg);

	return len;
}

static DEVICE_ATTR(reg, 0777, hdmi_show_reg, hdmi_store_reg);


static ssize_t hdmi_show_val(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val;

	switch_bank(debug_reg);
	val = HDMITX_ReadI2C_Byte(debug_reg);

	printk( " read reg 0x%x :val = 0x%x \n", debug_reg, val);
	return snprintf(buf, PAGE_SIZE, "%x\n", val);
}

static ssize_t hdmi_store_val (struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
        unsigned long value;
	unsigned char val;
	char *str = buf;

	if (strnicmp(buf, "0x", 2) == 0 || strnicmp(buf, "0X", 2) == 0)
		str = buf + 2;

	sscanf(str, "%lx", &value);
	val = (unsigned char)value;
	printk( "set reg 0x%x to 0x%x \n", debug_reg, val);

	switch_bank(debug_reg);
	HDMITX_WriteI2C_Byte(debug_reg, val);
	return len;
}

static DEVICE_ATTR(val, 0777, hdmi_show_val, hdmi_store_val);

//ligang 
static void hdmi_irqwork_func(struct work_struct *work)
{
	HDMITX_DevLoopProc() ;

	//clear Int
	HDMITX_WriteI2C_Byte(0x0C, 0xff);
	HDMITX_WriteI2C_Byte(0x0D, 0xff);
	hdmi_irq_enable();
}


/*
 * i2c functions
 */
static irqreturn_t hdmi_it6610_interrupt(int irq, void *dev_id);

static int hdmi_i2c_probe(struct i2c_client *client,
 const struct i2c_device_id *id)
{
	i2c_hdmi_client = client;

#if 0
	printk("hdmi client i2c addr = %x \n", i2c_hdmi_client->addr);
	printk( "hdmi int-io status: %d \n", gpio_get_value(HDMI_GPIO));

	printk( " it6610--00 = 0x%x \n", HDMITX_ReadI2C_Byte(0x00));
	printk( " it6610--01 = 0x%x \n", HDMITX_ReadI2C_Byte(0x01));
	printk( " it6610--02 = 0x%x \n", HDMITX_ReadI2C_Byte(0x02));
	printk( " it6610--03 = 0x%x \n", HDMITX_ReadI2C_Byte(0x03));
#endif

#ifdef CONFIG_EMXX_HDMI_IT6610_DEBUG
	printk("hdmi_i2c_probe!\n");
#endif

	HDMITX_ChangeDisplayOption(HDMI_720p50, HDMI_RGB444) ;  //HDMI_480p60  HDMI_RGB444
	InitCAT6611() ;

	INIT_DELAYED_WORK(&hdmi_irq_work, hdmi_irqwork_func);
	i2c_hdmi_init_done = 1;

	//irq handler
	
        request_irq(HDMI_IRQ, hdmi_it6610_interrupt,
			IRQF_DISABLED, HDMI_IT6610_MODNAME,
			hdmi_it6610_info);
        hdmi_gpio_setup();

	set_irq_type(HDMI_IRQ, IRQ_TYPE_LEVEL_LOW);

	return 0;
}

static int hdmi_i2c_remove(struct i2c_client *client)
{
	i2c_hdmi_client = NULL;
	return 0;
}

static int i2c_hdmi_cleanup(void)
{
	i2c_del_driver(&i2c_hdmi_driver);
	i2c_hdmi_init_done = 0;
	return 0;
}

static int i2c_hdmi_inserted(void)
{
	return i2c_add_driver(&i2c_hdmi_driver);
}

static int i2c_hdmi_init(void)
{
	int res = 0;
#ifdef CONFIG_EMXX_HDMI_IT6610_DEBUG
	printk("i2c_hdmi_init!\n");
#endif

	if (i2c_hdmi_init_done != 0)
		return 0;

	res = i2c_hdmi_inserted();
#ifdef CONFIG_EMXX_HDMI_IT6610_DEBUG
	printk("i2c_hdmi_init =%d!\n",res);
#endif
#if 0
	if (res == 0) {
		if (i2c_hdmi_client == NULL) {
			i2c_hdmi_cleanup();
			printk(KERN_ERR "hdmi it6610 i2c_init failed\n");
			return -EIO;
		}
	} else {
		printk(KERN_ERR "i2c hdmi it6610 inserted failed!\n");
	}
#endif
	i2c_hdmi_init_done = -1;

	return res;
}

static int i2c_hdmi_write(unsigned char reg, unsigned char data,
			   unsigned char mask)
{
	int res = 0;
	unsigned char buf[2];

//TODO:ligang
/*
	if ((IT6610REG_MAX <= reg) || ((data & ~(mask)) != 0))
		return -EINVAL;

	data = (i2c_hdmi_reg[reg] & ~(mask)) | (data & mask);

	i2c_hdmi_reg[reg] = data;

	buf[0] = reg;
	buf[1] = data;

	res = i2c_master_send(i2c_hdmi_client, buf, 2);
	if (res > 0)
		res = 0;
	else
		printk(KERN_ERR "i2c hdmi write failed!\n");
*/
	return res;
}

static int i2c_hdmi_read(unsigned char reg, unsigned char *data)
{
	int res = 0;
#if 0
	unsigned char buf = 0;
#endif

	if (i2c_hdmi_client == NULL) {
		printk(KERN_ERR "i2c hdmi not available!\n");
		return -EIO;
	}

//TODO: ligang
/*
	if (IT6610REG_MAX <= reg)
		return -EINVAL;
*/

#if 0
	res = i2c_master_send(i2c_hdmi_client, &buf, 1);
	if (res <= 0) {
		printk(KERN_ERR "i2c hdmi send failed!\n");
		return res;
	}
#endif

//TODO:ligang
/*
	res = i2c_master_recv(i2c_hdmi_client, i2c_hdmi_reg, IT6610REG_MAX);
	if (res > 0) {
		*data = i2c_hdmi_reg[reg];
		res = 0;
	} else {
		printk(KERN_ERR "i2c hdmi recv failed!\n");
	}
*/

	return res;
}

/* controling it6610 via I2C */

static	int	hdmi_it6610_reset_seq[] = {
	-1, -1,
};

/*!
 * reset HDMI_it6610 module
 * @param void
 * @retval 0 successful
 * @retval -1 failed
 */
static int hdmi_it6610_reset(void)
{
	int res = 0;
	int i;

	mutex_lock(&hdmi_it6610_mutex);

	res = i2c_hdmi_init();
	if (res != 0) {
		debug1("hdmi it6610 \'i2c_hdmi_init on reset\' failed\n");
		goto err1;
	}

//ligang write all regs to 0xff
#if 0
	/* Reset Sequence */
	i = 0;
	while (hdmi_it6610_reset_seq[i] >= 0) {
		res = IT6610_WRITE(hdmi_it6610_reset_seq[i],
				    hdmi_it6610_reset_seq[i+1],
				    0xff);
		debug2("hdmi i2c %x %x %x\n", res,
			hdmi_it6610_reset_seq[i],
			hdmi_it6610_reset_seq[i+1]);
		i += 2;
		if (res < 0)
			goto err1;
	}
#endif

	debug1("hdmi it6610 reset\n");
	mutex_unlock(&hdmi_it6610_mutex);

	return res;

err1:
	mutex_unlock(&hdmi_it6610_mutex);

	return res;
}


static	int	hdmi_it6610_seq[] = {
#if 0
#endif
	-1, -1,
	-1, -1,
};

//#define	WAIT_CONNECTION_IN_OPEN 1

#ifdef	WAIT_CONNECTION_IN_OPEN
/*!
 * kickoff HDMI_it6610 module
 * @param void
 */
static int hdmi_it6610_connect(void)
{
	printk( " hdmi connect\n");

	if (hdmi_it6610_info->intr == 0)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}
#endif


/*!
 * start HDMI_it6610 module
 * @param void
 * @retval 0 successful
 * @retval -1 failed
 */
static int hdmi_it6610_power_on(void)
{
	int res = 0;
	int i;
	unsigned char state = 0;

	mutex_lock(&hdmi_it6610_mutex);
	if (hdmi_it6610_info->poweron_state) {
		mutex_unlock(&hdmi_it6610_mutex);
		return 0;
	}

#if 1
	outl(SMU_PLLSEL_OSC1 | SMU_DIV(2), SMU_REFCLKDIV);
	emxx_open_clockgate(EMXX_CLK_REF);
#endif

	res = i2c_hdmi_init();
	if (res != 0) {
		debug1("hdmi it6610 i2c_hdmi_init failed\n");
		goto err1;
	}

#ifdef	WAIT_CONNECTION_IN_OPEN
	debug1("hdmi it6610 waiting\n");
	while (hdmi_it6610_info->intr == 0) {
		if (wait_event_interruptible(intwaitq,
					     hdmi_it6610_connect())) {
			debug1("hdmi it6610 wakeup sequence on open()\n");
			hdmi_it6610_info->intr = 0;
			res = -ERESTARTSYS;
			goto err1;
		}
		debug1("hdmi it6610 wokeup %d\n", hdmi_it6610_info->intr);
	}
	hdmi_it6610_info->intr = 0;
#endif

//TODO:ligang read status
/*
	i = IT6610_READ(IT6610REG_ST, &state);
	if (i != 0 || (state & 0x40) == 0) {
		printk(KERN_ERR "HDMI opened before connect!\n");
	}
	debug1("hdmi it6610 state %x\n", state);
*/
	/* Set Up Sequence */
//TODO:ligang: regs init; all regs write 0xff
/*
	i = 0;
	while (hdmi_it6610_seq[i] >= 0) {
		res = IT6610_WRITE(hdmi_it6610_seq[i],
				    hdmi_it6610_seq[i+1],
				    0xff);
		debug2("hdmi i2c %x %x %x\n", res, hdmi_it6610_seq[i],
		       hdmi_it6610_seq[i+1]);
		i += 2;
		if (res < 0)
			goto err1;
	}
*/
	hdmi_it6610_info->poweron_state++;
	debug1("hdmi it6610 power on %d\n", hdmi_it6610_info->poweron_state);
	mutex_unlock(&hdmi_it6610_mutex);

	return res;

err1:
	mutex_unlock(&hdmi_it6610_mutex);

	return res;
}

static	int	hdmi_it6610_stop_seq[] = {
	-1, -1,
};

/*!
 * stop HDMI_it6610 module
 * @param void
 * @retval 0 successful
 * @retval -1 failed
 */
static int hdmi_it6610_power_off(void)
{
	int res = 0;
	int i;

	mutex_lock(&hdmi_it6610_mutex);
	if (hdmi_it6610_info->poweron_state == 0) {
		mutex_unlock(&hdmi_it6610_mutex);
		return 0;
	}

//ligang write all regs to 0xff
#if 0
	i = 0;
	while (hdmi_it6610_stop_seq[i] >= 0) {
		res = IT6610_WRITE(hdmi_it6610_stop_seq[i],
			hdmi_it6610_stop_seq[i+1], 0xff);
		debug2("hdmi i2c %x %x %x\n", res,
			hdmi_it6610_stop_seq[i],
			hdmi_it6610_stop_seq[i+1]);
		i += 2;
		if (res < 0)
			goto out;
	}
#endif
	hdmi_it6610_reset();

	i2c_hdmi_cleanup();

#if 1
	emxx_close_clockgate(EMXX_CLK_REF);
#endif

	hdmi_it6610_info->poweron_state--;
	debug1("hdmi it6610 power off %d", hdmi_it6610_info->poweron_state);
out:
	mutex_unlock(&hdmi_it6610_mutex);

	return res;
}


/* file operations ----------------------------------------------------------*/

/*!
 * read file operation
 * @param[in] filp
 * @param[out] buf to copy struct hdmi_it6610_interrupt_t
 * @param[in] count buf size
 * @param[in] offp
 * @retval size size of struct hdmi_it6610_interrupt_t
 * @retval -EINVAL count error
 */
static ssize_t hdmi_it6610_read(struct file *filp, char *buf,
			size_t count, loff_t *offp)
{
#ifdef	CONFIG_EMXX_HDMI_IT6610_DEBUG
	int minor = MINOR(filp->f_dentry->d_inode->i_rdev);
#endif

	debug1("minor %d, count %d", minor, count);

	return -EINVAL;
}

/*!
 * select file operation
 * @param[in] filp
 * @param[in] wait
 * @retval mask POLLIN | POLLRDNORM
 */
static unsigned int hdmi_it6610_poll(struct file *filp,
			     struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	debug2("minor %d", MINOR(filp->f_dentry->d_inode->i_rdev));

#if 0
	poll_wait(filp, &readq, wait);
#endif

	mask |= POLLIN | POLLRDNORM;
#if 0
	if (hdmi_it6610_info->interrupt.interrupt.l)
		mask |= POLLIN | POLLRDNORM;
#endif

	debug2("mask 0x%X", mask);
	return mask;
}

/*!
 * ioctl file operation
 * @param[in] inode
 * @param[in] filp
 * @param[in] cmd HDMI_it6610_IOCQBUFSIZE or HDMI_it6610_IOCSREGCMD
 * @param[in] arg struct hdmi_it6610_command_t with HDMI_it6610_IOCSREGCMD
 * @retval size for HDMI_it6610_IOCQBUFSIZE
 * @retval 0 command successful
 * @retval -EINVAL command error
 */
static int hdmi_it6610_ioctl(struct inode *inode, struct file *filp,
		     unsigned int cmd, unsigned long arg)
{
	printk( "==>> hdmi ioctl \n");
#if 0
	enum EMXX_HDMI_OUTPUT_MODE ra;
#endif
	int ret = 0;

	switch (cmd) {
	case EMXX_HDMI_GET_OUTPUT:
		debug1("EMXX_HDMI_GET_OUTPUT");

		if (copy_to_user((void *)arg, &hdmi_it6610_info->resolution,
				sizeof(hdmi_it6610_info->resolution))) {
			debug0("copy_to_user failed");
			ret = -EFAULT;
			break;
		}
		break;

	case EMXX_HDMI_SET_OUTPUT:
		debug1("EMXX_HDMI_SET_OUTPUT");
		ret = -EIO; /* ENOTTY */
		if (arg == 0) {
			ret = -EINVAL;
			break;
		}
#if 0
		if (copy_from_user(&ra, (void *)arg, sizeof(ra))) {
			debug0("copy_from_user failed");
			ret = -EFAULT;
			break;
		}
		hdmi_it6610_power_off();
		hdmi_it6610_info->resolution = (enum EMXX_HDMI_OUTPUT_MODE)ra;
		hdmi_it6610_power_on();
		ret = 0;
#endif
		break;


	default:
		debug1("unknown cmd 0x%08X, arg 0x%08lX", cmd, arg);
		ret = -EINVAL; /* ENOTTY */
		break;
	}

	debug1("ret %d", ret);
	return ret;
}

/*!
 * mmap file operation
 * @param[in] filp
 * @param[in] vma
 * @retval 0 successful
 * @retval -EINVAL size error
 */
static int hdmi_it6610_mmap(struct file *filp, struct vm_area_struct *vma)
{
#if 0
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long info_size = PAGE_ALIGN(sizeof(
	 struct hdmi_it6610_common_info_t));
	unsigned long info_pa;

	info_pa = (unsigned long)hdmi_it6610_info->pa_common_info;

	debug1("vm_start 0x%08lX", vma->vm_start);
	debug1("vm_end   0x%08lX", vma->vm_end);
	debug1("size     %lu", size);
	debug1("vm_pgoff 0x%08lX", vma->vm_pgoff);
	debug1("vm_page_prot 0x%08lX", pgprot_val(vma->vm_page_prot));

	if (size > info_size) {
		debug1("error request size %lu > allocated size %lu",
		       size, info_size);
		return -EINVAL;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, info_pa >> PAGE_SHIFT, size,
			     vma->vm_page_prot)) {
		debug1("remap_pfn_range failed");
		return -EAGAIN;
	}

	initialize_common_info(hdmi_it6610_info->common_info);
	debug1("mapped to 0x%08lX", info_pa);
	return 0;
#else
	return -EINVAL;
#endif
}


/*!
 * open file operation
 * @param[in] inode
 * @param[in] filp
 * @retval 0 successful
 * @retval -EBUSY double request
 * @retval -FAULT power setting failure etc
 */
static int hdmi_it6610_open(struct inode *inode, struct file *filp)
{
	if (hdmi_it6610_info->open_state != 0) {
		debug1("already opened");
		return -EBUSY;
	}

	debug1("hdmi_it6610 open");

	/* power on */
	if (hdmi_it6610_power_on() < 0) {
		debug0("hdmi_it6610_power_on failed");
		return -EFAULT;
	}

	hdmi_it6610_info->open_state = 1;

	return 0;
}

/*!
 * close file operation
 * @param[in] inode
 * @param[in] filp
 * @retval 0 successful
 */
static int hdmi_it6610_release(struct inode *inode, struct file *filp)
{
#ifdef	CONFIG_EMXX_HDMI_IT6610_DEBUG
	int minor = MINOR(inode->i_rdev);
#endif

	debug1("minor %d", minor);

	hdmi_it6610_power_off();

	hdmi_it6610_info->open_state = 0;

	return 0;
}

/* ! file operations for this driver */
static const struct file_operations hdmi_it6610_fops = {
	.owner      = THIS_MODULE,
	.llseek     = no_llseek,
	.open       = hdmi_it6610_open,
	.release    = hdmi_it6610_release,
	.read       = hdmi_it6610_read,
	.poll       = hdmi_it6610_poll,
	.ioctl      = hdmi_it6610_ioctl,
	.mmap       = hdmi_it6610_mmap,
};


/* interruption -------------------------------------------------------------*/

/*!
 * HDMI_it6610 interruption handler about stream buffer full
 * @param irq
 * @param dev_id
 * @param regs
 */
static irqreturn_t hdmi_it6610_interrupt(int irq, void *dev_id)
{
//	hdmi_it6610_info->intr++;
//	debug1("irq %d\n", hdmi_it6610_info->intr);
//	wake_up_interruptible(&intwaitq);

#ifdef CONFIG_EMXX_HDMI_IT6610_DEBUG
	printk( "==>>hdmi irq %d \n", gpio_get_value(HDMI_GPIO));
#endif
	hdmi_irq_disable();
	schedule_delayed_work(&hdmi_irq_work, msecs_to_jiffies(500));

	return IRQ_HANDLED;
}

#if 0
/*!
 * tasklet for interruption
 * @param[in] value unused
 * @return void
 */
void hdmi_it6610_tasklet_handler(unsigned long value)
{
	unsigned int factor;
	unsigned int int_a, int_c;

	wake_up_interruptible(&readq);
}
#endif

/* module functions ---------------------------------------------------------*/

#ifdef CONFIG_PM
static int hdmi_it6610_pf_suspend(struct platform_device *dev,
 pm_message_t state)
{
	if (hdmi_it6610_info->open_state)
		hdmi_it6610_power_off();

	return 0;
}

static int hdmi_it6610_pf_resume(struct platform_device *dev)
{
	if (hdmi_it6610_info->open_state)
		hdmi_it6610_power_on();

	return 0;
}
#endif

static int hdmi_it6610_pf_probe(struct platform_device *dev)
{
	int ret;
#if 0
	int i;
	void *virt;
	unsigned short data;
	unsigned short *code_addr;
#endif
	dev_t devno;
#ifdef CONFIG_EMXX_HDMI_IT6610_DEBUG
	printk("hdmi_it6610_pf_probe!\n");
#endif

	mutex_init(&hdmi_it6610_mutex);
	hdmi_it6610_info->intr = 0;
	hdmi_it6610_info->open_state = 0;

	init_waitqueue_head(&intwaitq);
	if (hdmi_it6610_reset() < 0) {
		debug0("hdmi_reset failed");
		ret = -EFAULT;
		goto error_return;
	}

	//ligang
	device_create_file(&dev->dev, &dev_attr_reg);
	device_create_file(&dev->dev, &dev_attr_val);
#if 0
	debug1("requesting irq %d\n", HDMI_IRQ);
	/* request_irq returns ENOMEM/EINVAL */
	set_irq_type(HDMI_IRQ, IRQ_TYPE_EDGE_FALLING);

	ret = request_irq(HDMI_IRQ, hdmi_it6610_interrupt,
			  IRQF_DISABLED, HDMI_IT6610_MODNAME,
			  hdmi_it6610_info);
	if (ret < 0) {
		debug0("request_irq(%d) failed %d", HDMI_IRQ, ret);
		goto error_return;
	}
	debug1("set irq type done%d\n", HDMI_IRQ);
#endif
	/* register chrdev */
	debug1("register_chrdev %d, %s", devmajor, HDMI_IT6610_MODNAME);
	if (devmajor) {
		devno = MKDEV(devmajor, 0);
		ret = register_chrdev_region(devno, HDMI_IT6610_MINOR_MAX,
						HDMI_IT6610_MODNAME);
	} else {
		ret = alloc_chrdev_region(&devno,
						0, HDMI_IT6610_MINOR_MAX,
						HDMI_IT6610_MODNAME);
		devmajor = MAJOR(devno);
	}
	if (ret) {
		debug0("register_chrdev %d, %s failed %d",
				devmajor, HDMI_IT6610_MODNAME, ret);
		goto free_irq_int;
	}

	cdev_init(&hdmi_it6610_cdev, &hdmi_it6610_fops);
	ret = cdev_add(&hdmi_it6610_cdev, devno, HDMI_IT6610_MINOR_MAX);
	if (ret) {
		debug0("cdev_add %s failed %d", HDMI_IT6610_MODNAME, ret);
		goto free_chrdev;
	}

	hdmi_it6610_class = class_create(THIS_MODULE, HDMI_IT6610_MODNAME);
	if (IS_ERR(hdmi_it6610_class)) {
		debug0("class_create failed %d", ret);
		ret = PTR_ERR(hdmi_it6610_class);
		goto free_cdev;
	}

	hdmi_it6610_class_device = device_create(hdmi_it6610_class,
		&dev->dev, MKDEV(devmajor, 0),  NULL, "%s",
		HDMI_IT6610_DEVNAME);
	if (IS_ERR(hdmi_it6610_class_device)) {
		debug0("class_device_create failed %s %d",
			HDMI_IT6610_DEVNAME, ret);
		ret = PTR_ERR(hdmi_it6610_class_device);
		goto free_class;
	}


	hdmi_it6610_info->open_state = 0;
	/* default value */
	hdmi_it6610_info->resolution = EMXX_HDMI_OUTPUT_MODE_HDMI_720P_60fps;  //ffhh EMXX_HDMI_OUTPUT_MODE_HDMI_720P_60fps

	debug1("success");
	return 0;

free_class:
	class_destroy(hdmi_it6610_class);

free_cdev:
	cdev_del(&hdmi_it6610_cdev);

free_chrdev:
	unregister_chrdev_region(MKDEV(devmajor, 0), HDMI_IT6610_MINOR_MAX);

free_irq_int:
	free_irq(HDMI_IRQ, hdmi_it6610_info);

error_return:
	return ret;
}

static int hdmi_it6610_pf_remove(struct platform_device *dev)
{
	device_destroy(hdmi_it6610_class, MKDEV(devmajor, 0));
	class_destroy(hdmi_it6610_class);
	debug1("unregister_chrdev %d, %s", devmajor, HDMI_IT6610_MODNAME);
	unregister_chrdev_region(MKDEV(devmajor, 0), HDMI_IT6610_MINOR_MAX);
	free_irq(HDMI_IRQ, hdmi_it6610_info);

	return 0;
}

static void hdmi_it6610_pf_release(struct device *dev)
{
	/* none */
}

/*!
 * initialize for insmod
 * @param void
 * @return void
 */
static int __init hdmi_it6610_init(void)
{
	int ret;
#ifdef CONFIG_EMXX_HDMI_IT6610_DEBUG
	printk("hdmi_it6610_init ret!\n");
#endif
	ret = platform_device_register(&hdmi_it6610_pf_device);
#ifdef CONFIG_EMXX_HDMI_IT6610_DEBUG
	printk("hdmi_it6610_init ret=%d!\n",ret);
#endif
	if (ret)
		return ret;

	ret = platform_driver_register(&hdmi_it6610_pf_driver);
	if (ret)
		return ret;
}

/*!
 * finalize for rmmod
 * @param void
 * @return void
 */
static void __exit hdmi_it6610_exit(void)
{
	(void)platform_driver_unregister(&hdmi_it6610_pf_driver);
	(void)platform_device_unregister(&hdmi_it6610_pf_device);
}

module_init(hdmi_it6610_init);
module_exit(hdmi_it6610_exit);
module_param(devname, charp, 0444);
module_param(devmajor, int, 0444);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("EMEV HDMI it6610 Driver");
MODULE_AUTHOR("ligang");

