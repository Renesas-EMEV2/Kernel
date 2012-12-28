#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <mach/emev_board.h>
#include <linux/poll.h>
#include <linux/workqueue.h>

#define ID_A  (0)
#define ID_M  (1)
#define ID_O  (2)
#define ID_P  (3)
#define ID_L  (4)

#define SENSORS_ACCELERATION    (1 << ID_A)
#define SENSORS_MAGNETIC_FIELD  (1 << ID_M)
#define SENSORS_ORIENTATION     (1 << ID_O)
#define SENSORS_LIGHT     	(1 << ID_L)

#define	SENSORS_ACC_LSM303		0x01000000
#define	SENSORS_ACC_BMA220		0x02000000
#define	SENSORS_MAG_LSM303		0x03000000
#define	SENSORS_TYPE_ACC_LSM303		(SENSORS_ACCELERATION|SENSORS_ACC_LSM303)
#define	SENSORS_TYPE_ACC_BMA220		(SENSORS_ACCELERATION|SENSORS_ACC_BMA220)
#define	SENSORS_TYPE_MAG_LSM303		(SENSORS_MAGNETIC_FIELD|SENSORS_MAG_LSM303)

#define EMXX_SENSORS_DEV_NAME		"sensor"
#define EMXX_SENSORS_LIGHT_DEV_NAME	"light"

#define SENSORS_ACC_IOCTL_BASE			'a'
/* The following define the IOCTL command values via the ioctl macros */
#define SENSORS_ACC_IOCTL_SET_DELAY		_IOW(SENSORS_ACC_IOCTL_BASE, 0, int)
#define SENSORS_ACC_IOCTL_GET_DELAY		_IOR(SENSORS_ACC_IOCTL_BASE, 1, int)
#define SENSORS_ACC_IOCTL_SET_ENABLE		_IOW(SENSORS_ACC_IOCTL_BASE, 2, int)
#define SENSORS_ACC_IOCTL_GET_ENABLE		_IOR(SENSORS_ACC_IOCTL_BASE, 3, int)
#define SENSORS_ACC_IOCTL_SET_G_RANGE		_IOW(SENSORS_ACC_IOCTL_BASE, 4, int)

#define SENSORS_MAG_IOCTL_BASE			'm'
/* The following define the IOCTL command values via the ioctl macros */
#define SENSORS_MAG_IOCTL_SET_DELAY		_IOW(SENSORS_MAG_IOCTL_BASE, 0, int)
#define SENSORS_MAG_IOCTL_GET_DELAY		_IOR(SENSORS_MAG_IOCTL_BASE, 1, int)
#define SENSORS_MAG_IOCTL_SET_ENABLE		_IOW(SENSORS_MAG_IOCTL_BASE, 2, int)
#define SENSORS_MAG_IOCTL_GET_ENABLE		_IOR(SENSORS_MAG_IOCTL_BASE, 3, int)
#define SENSORS_MAG_IOCTL_SET_H_RANGE		_IOW(SENSORS_MAG_IOCTL_BASE, 4, int)

#define SENSORS_ORI_IOCTL_BASE			'o'
/* The following define the IOCTL command values via the ioctl macros */
#define SENSORS_ORI_IOCTL_SET_DELAY		_IOW(SENSORS_ORI_IOCTL_BASE, 0, int)
#define SENSORS_ORI_IOCTL_GET_DELAY		_IOR(SENSORS_ORI_IOCTL_BASE, 1, int)
#define SENSORS_ORI_IOCTL_SET_ENABLE		_IOW(SENSORS_ORI_IOCTL_BASE, 2, int)
#define SENSORS_ORI_IOCTL_GET_ENABLE		_IOR(SENSORS_ORI_IOCTL_BASE, 3, int)
#define SENSORS_ORI_IOCTL_SET_H_RANGE		_IOW(SENSORS_ORI_IOCTL_BASE, 4, int)

#define SENSORS_IOCTL_BASE			'l'
#define LIGHTSENSOR_IOCTL_ENABLE         	_IOW(SENSORS_IOCTL_BASE, 1, unsigned int)
#define LIGHTSENSOR_IOCTL_GET_STATE      	_IOR(SENSORS_IOCTL_BASE, 2, unsigned int)

//define data type(input event type)
#define EVENT_TYPE_ACCEL_X          ABS_X
#define EVENT_TYPE_ACCEL_Y          ABS_Y
#define EVENT_TYPE_ACCEL_Z          ABS_Z
#define EVENT_TYPE_ACCEL_STATUS     ABS_WHEEL

#define EVENT_TYPE_YAW              ABS_RX
#define EVENT_TYPE_PITCH            ABS_RY
#define EVENT_TYPE_ROLL             ABS_RZ
#define EVENT_TYPE_ORIENT_STATUS    ABS_RUDDER

#define EVENT_TYPE_MAGV_X           ABS_HAT0X
#define EVENT_TYPE_MAGV_Y           ABS_HAT0Y
#define EVENT_TYPE_MAGV_Z           ABS_BRAKE
#define	EVENT_TYPE_MAGV_OK          ABS_HAT1X

#define EVENT_TYPE_TEMPERATURE      ABS_THROTTLE
#define EVENT_TYPE_STEP_COUNT       ABS_GAS
#define EVENT_TYPE_PROXIMITY        ABS_DISTANCE
#define EVENT_TYPE_LIGHT            ABS_MISC

struct emxx_sensors_obj;
struct emxx_sensors_pub{
	u32 has_sensors;
	struct i2c_adapter* i2c_adapter;
	void (*lock)(struct emxx_sensors_pub*);
	void (*unlock)(struct emxx_sensors_pub*);
	int (*i2c_read)(struct emxx_sensors_pub*, int addr, u8 *buf, int len);
	int (*i2c_read2)(struct emxx_sensors_pub*, int addr, u8 reg, u8 *val, int len);
	int (*i2c_write)(struct emxx_sensors_pub*, int addr, u8 *buf, int len);
	int (*i2c_write2)(struct emxx_sensors_pub*, int addr, u8 reg, u8 *val, int len);

	struct emxx_sensors_obj *sensor_acc;
	struct emxx_sensors_obj *sensor_mag;
};

struct emxx_sensors_obj{
	int sensors_type;
	int i2c_addr;
	int val[3];
	int cali[3];
	//int (*probe)(struct emxx_sensors_pub*, struct emxx_sensors_obj*);
	int (*remove)(struct emxx_sensors_pub*, struct emxx_sensors_obj*);
	int (*open) (struct emxx_sensors_pub*, struct emxx_sensors_obj*);
	int (*close) (struct emxx_sensors_pub*, struct emxx_sensors_obj*);
	u32 (*get) (struct emxx_sensors_pub*, struct emxx_sensors_obj*);
	int (*suspend)(struct emxx_sensors_pub*, struct emxx_sensors_obj*);
	int (*resume)(struct emxx_sensors_pub*, struct emxx_sensors_obj*);
	int (*set_delay)(struct emxx_sensors_pub*, struct emxx_sensors_obj*, int ms);
	int (*calibrate)(struct emxx_sensors_pub*, struct emxx_sensors_obj*, int cali[4]);
};

typedef struct emxx_sensors_obj* (*emxx_sensors_probe)(struct emxx_sensors_pub*);

#if defined(CONFIG_INPUT_BMA220_G_SENSORS)
extern struct emxx_sensors_obj* bma220_probe(struct emxx_sensors_pub*);
#endif
#if defined(CONFIG_INPUT_LSM303_SENSORS)
extern struct emxx_sensors_obj*lsm303_acc_probe(struct emxx_sensors_pub*);
extern struct emxx_sensors_obj*lsm303_mag_probe(struct emxx_sensors_pub*);
#endif

