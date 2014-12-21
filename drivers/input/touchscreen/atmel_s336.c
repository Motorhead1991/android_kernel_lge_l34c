/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2011-2012 Atmel Corporation
 * Copyright (C) 2012 Google, Inc.
 *
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include "atmel_s336.h"
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include <linux/time.h>

#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <linux/mfd/pm8xxx/cradle.h>
#include <linux/sysdev.h>

#define DEBUG_ABS	1

/* T37 debug data */
#define T37_DBG_DATA	1

/* Diagnostic command defines  */
#define MXT_DIAG_PAGE_UP		0x01
#define MXT_DIAG_PAGE_DOWN		0x02
#define MXT_DIAG_DELTA_MODE		0x10
#define MXT_DIAG_REFERENCE_MODE		0x11
#define MXT_DIAG_CTE_MODE		0x31
#define MXT_DIAG_IDENTIFICATION_MODE	0x80
#define MXT_DIAG_TOCH_THRESHOLD_MODE	0xF4
#define MXT_DIAG_MODE_MASK		0xFC
#define MXT_DIAGNOSTIC_MODE		0
#define MXT_DIAGNOSTIC_PAGE		1
#define MXT_CONFIG_VERSION_LENGTH	30

/* Configuration file */
#define MXT_CFG_MAGIC		"OBP_RAW V1"

/* Registers */
#define MXT_OBJECT_START	0x07
#define MXT_OBJECT_SIZE		6
#define MXT_INFO_CHECKSUM_SIZE	3
#define MXT_MAX_BLOCK_WRITE	256

/* Object types */
#define MXT_DEBUG_DIAGNOSTIC_T37	37
#define MXT_GEN_MESSAGE_T5		5
#define MXT_GEN_COMMAND_T6		6
#define MXT_GEN_POWER_T7		7
#define MXT_GEN_ACQUIRE_T8		8
#define MXT_TOUCH_KEYARRAY_T15		15
#define MXT_TOUCH_PROXIMITY_T23		23
#define MXT_TOUCH_MULTITOUCHSCREEN_T100 100
#define MXT_PROCI_GRIPSUPPRESSION_T40		40
#define MXT_PROCI_TOUCHSUPPRESSION_T42	42
#define MXT_PROCI_STYLUS_T47		47
#define MXT_PROCI_ADAPTIVETHRESHOLD_T55			55
#define MXT_PROCI_SHIELDLESS_T56				56
#define MXT_PROCI_LENSBENDING_T65				65
#define MXT_PROCI_PALMGESTUREPROCESSOR_T69		69
#define MXT_PROCG_NOISESUPPRESSION_T72	72
#define MXT_GLOVEDETECTION_T78				78
#define MXT_RETRANSMISSIONCOMPENSATION_T80		80
#define MXT_PROCI_GESTUREPROCESSOR_T84			84
#define MXT_PROCI_SCHNOISESUPPRESSION_T103	103
#define MXT_SPT_COMMSCONFIG_T18		18
#define MXT_SPT_GPIOPWM_T19		19
#define MXT_SPT_SELFTEST_T25		25
#define MXT_SPT_USERDATA_T38		38
#define MXT_SPT_MESSAGECOUNT_T44	44
#define MXT_SPT_CTECONFIG_T46		46
#define MXT_SPT_TIMER_T61			61
#define MXT_SPT_GOLDENREFERENCES_T66	66
#define MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70	70
#define MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71	71
#define MXT_SPT_SELFCAPCBCRCONFIG_T102		102
#define MXT_SPT_AUXTOUCHCONFIG_T104			104
#define MXT_SPT_TOUCHSCREENHOVER_T101		101
#define MXT_SPT_DRIVENPLATEHOVERCONFIG_T105	105
#ifdef MXT_GESTURE_RECOGNIZE
#define MXT_PROCI_ONETOUCH_T24		24
#define MXT_SPT_PROTOTYPE_T35		35
#endif

/* Not for ATMEL S540 */
#define MXT_GEN_DATASOURCE_T53		53
#define MXT_TOUCH_MULTI_T9		9
#define MXT_TOUCH_PROXKEY_T52		52
#define MXT_PROCI_GRIPFACE_T20		20
#define MXT_PROCG_NOISE_T22		22
#define MXT_PROCI_TWOTOUCH_T27		27
#define MXT_PROCI_PALM_T41		41
#define MXT_PROCG_NOISESUPPRESSION_T48	48
#define MXT_SPT_CTECONFIG_T28		28
#define MXT_SPT_DIGITIZER_T43		43
#define MXT_SPT_NOISESUPPRESSION_T48	48
#define MXT_PROCI_ACTIVE_STYLUS_T63	63
#define MXT_RESERVED_T255 255

/* MXT_GEN_MESSAGE_T5 object */
#define MXT_RPTID_NOMSG		0xff

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_DIAGNOSTIC	5

/* Define for T6 status byte */
#define MXT_T6_STATUS_RESET	(1 << 7)
#define MXT_T6_STATUS_OFL	(1 << 6)
#define MXT_T6_STATUS_SIGERR	(1 << 5)
#define MXT_T6_STATUS_CAL	(1 << 4)
#define MXT_T6_STATUS_CFGERR	(1 << 3)
#define MXT_T6_STATUS_COMSERR	(1 << 2)

/* MXT_GEN_POWER_T7 field */
struct t7_config {
	u8 idle;
	u8 active;
} __packed;

#define MXT_POWER_CFG_RUN		0
#define MXT_POWER_CFG_DEEPSLEEP		1

/* MXT_TOUCH_MULTI_T9 field */
#define MXT_T9_ORIENT		9
#define MXT_T9_RANGE		18

/* MXT_TOUCH_MULTI_T9 status */
#define MXT_T9_UNGRIP		(1 << 0)
#define MXT_T9_SUPPRESS		(1 << 1)
#define MXT_T9_AMP		(1 << 2)
#define MXT_T9_VECTOR		(1 << 3)
#define MXT_T9_MOVE		(1 << 4)
#define MXT_T9_RELEASE		(1 << 5)
#define MXT_T9_PRESS		(1 << 6)
#define MXT_T9_DETECT		(1 << 7)

struct t9_range {
	u16 x;
	u16 y;
} __packed;

/* MXT_TOUCH_MULTI_T9 orient */
#define MXT_T9_ORIENT_SWITCH	(1 << 0)

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1
#define MXT_COMMS_RETRIGEN      (1 << 6)

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE		0xa5
#define MXT_RESET_VALUE		0x01
#define MXT_BACKUP_VALUE	0x55
#define MXT_STOP_DYNAMIC_CONFIG	0x33

/* Define for MXT_PROCI_TOUCHSUPPRESSION_T42 */
#define MXT_T42_MSG_TCHSUP	(1 << 0)

/* T47 Stylus */
#define MXT_TOUCH_MAJOR_T47_STYLUS	1

/* T63 Stylus */
#define MXT_STYLUS_PRESS	(1 << 0)
#define MXT_STYLUS_RELEASE	(1 << 1)
#define MXT_STYLUS_MOVE		(1 << 2)
#define MXT_STYLUS_SUPPRESS	(1 << 3)

#define MXT_STYLUS_DETECT	(1 << 4)
#define MXT_STYLUS_TIP		(1 << 5)
#define MXT_STYLUS_ERASER	(1 << 6)
#define MXT_STYLUS_BARREL	(1 << 7)

#define MXT_STYLUS_PRESSURE_MASK	0x3F

/* T100 Multiple Touch Touchscreen */
#define MXT_T100_CTRL		0
#define MXT_T100_CFG1		1
#define MXT_T100_TCHAUX		3
#define MXT_T100_XRANGE		13
#define MXT_T100_YRANGE		24

#define MXT_T100_CFG_SWITCHXY	(1 << 5)

#define MXT_T100_TCHAUX_VECT	(1 << 0)
#define MXT_T100_TCHAUX_AMPL	(1 << 1)
#define MXT_T100_TCHAUX_AREA	(1 << 2)

#define MXT_T100_DETECT		(1 << 7)
#define MXT_T100_TYPE_MASK	0x70
#define MXT_T100_TYPE_FINGER	0x10
#define MXT_T100_TYPE_STYLUS	0x20
#define MXT_T100_TYPE_GLOVE		0x50
#define MXT_T100_STATUS_MASK	0x0F
#define MXT_T100_PRESS		0x04
#define MXT_T100_RELEASE	0x05
#define MXT_T100_MOVE		0x01

/* Delay times */
#define MXT_BACKUP_TIME		50	/* msec */
#define MXT_RESET_TIME		200	/* msec */
#define MXT_RESET_TIMEOUT	3000	/* msec */
#define MXT_CRC_TIMEOUT		1000	/* msec */
#define MXT_FW_RESET_TIME	1000	/* msec */
#define MXT_FW_CHG_TIMEOUT	300	/* msec */
#define MXT_WAKEUP_TIME		25	/* msec */
#define MXT_REGULATOR_DELAY	150	/* msec */
#define MXT_POWERON_DELAY	150	/* msec */
#define MXT_SELFTEST_TIME	3000	/* msec */

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK	0x02
#define MXT_FRAME_CRC_FAIL	0x03
#define MXT_FRAME_CRC_PASS	0x04
#define MXT_APP_CRC_FAIL	0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK	0x3f
#define MXT_BOOT_EXTENDED_ID	(1 << 5)
#define MXT_BOOT_ID_MASK	0x1f

/* Touchscreen absolute values */
#define MXT_MAX_AREA		0xff

#define MXT_PIXELS_PER_MM	20
#ifdef CUST_B_TOUCH
//                                                           
#define TOUCHEVENTFILTER	1
//                                                           
#endif
static bool is_probing;
static bool selftest_enable;
static bool selftest_show;
static bool update_cfg_force;
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
/* Early-suspend level */
#define MXT_SUSPEND_LEVEL 1
static void mxt_early_suspend(struct early_suspend *h);
static void mxt_late_resume(struct early_suspend *h);
#endif

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size_minus_one;
	u8 instances_minus_one;
	u8 num_report_ids;
} __packed;

#ifdef CUST_B_TOUCH
struct t_data {
	u16	id;
	u8	status;
	u16	x_position;
	u16	y_position;
	u8	touch_major;
	#if TOUCHEVENTFILTER
	u8	touch_minor;
	#endif
	u8	pressure;
	u8	orientation;
	int tool;
	bool is_pen;
	bool skip_report;
};

struct touch_data {
	u8 total_num;
	u8 prev_total_num;
	u8 state;
	u8 palm;
	struct t_data curr_data[MXT_MAX_NUM_TOUCHES];
	struct t_data prev_data[MXT_MAX_NUM_TOUCHES];
};
#endif

#ifdef T37_DBG_DATA
struct mxt_raw_data {
	u8 num_xnode;
	u8 num_ynode;
	u16 num_nodes;
	u16 *reference;
	s16 *delta;
};
#endif


/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[64];		/* device physical location */
	struct mxt_platform_data *pdata;
	struct mxt_object *object_table;
	struct mxt_info *info;
	void *raw_info_block;
	unsigned int irq;
	unsigned int max_x;
	unsigned int max_y;
	bool in_bootloader;
	u16 mem_size;
	u8 t100_aux_ampl;
	u8 t100_aux_area;
	u8 t100_aux_vect;
	u16 t100_anti_area;
	u16 t100_touch_area;
	struct bin_attribute mem_access_attr;
	bool debug_enabled;
	u8 max_reportid;
	u32 config_crc;
	u32 info_crc;
	u8 bootloader_addr;
	struct t7_config t7_cfg;
	u8 *msg_buf;
	u8 t6_status;
	bool update_input;
	u8 last_message_count;
	u8 num_touchids;
	u8 num_stylusids;
	unsigned long t15_keystatus;
	bool use_retrigen_workaround;
	bool use_regulator;
	struct regulator *vdd_ana;
	struct regulator *vcc_i2c;
	struct regulator *vcc_dig;
	char *fw_name;
	char *cfg_name;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
	struct kobject 				lge_touch_kobj;
	/* Cached parameters from object table */
	u16 T5_address;
	u8 T5_msg_size;
	u8 T6_reportid;
	u16 T6_address;
	u16 T7_address;
	u16 T8_address;
	u16 T9_address;
	u16 T15_address;
	u16 T24_address;
	u8 T9_reportid_min;
	u8 T9_reportid_max;
	u8 T15_reportid_min;
	u8 T15_reportid_max;
	u16 T18_address;
	u8 T19_reportid;
#ifdef MXT_GESTURE_RECOGNIZE
	u8 T24_reportid;
	u8 T35_reportid;
#endif
	u8 T25_reportid;
	u16 T25_address;
	u16 T42_address;
	u8 T42_reportid_min;
	u8 T42_reportid_max;
	u16 T44_address;
	u16 T46_address;
	u16 T47_address;
	u8 T48_reportid;
	u16 T56_address;
	u8 T63_reportid_min;
	u8 T63_reportid_max;
	u16 T65_address;
	u16 T72_address;
	u8 T100_reportid_min;
	u8 T100_reportid_max;
	u16 T100_address;

	/* for fw update in bootloader */
	struct completion bl_completion;

	/* for reset handling */
	struct completion reset_completion;

	/* for reset handling */
	struct completion crc_completion;

	/* Auto touch test */
	struct completion t25_completion;

	/* Enable reporting of input events */
	bool enable_reporting;

	/* Indicates whether device is in suspend */
	bool suspended;
#ifdef T37_DBG_DATA
	struct mxt_raw_data *rawdata;
#endif
#ifdef CUST_B_TOUCH
	struct touch_data ts_data;
#endif
#ifdef MXT_GESTURE_RECOGNIZE
	bool mxt_knock_on_enable;
	bool mxt_character_enable;
#endif
	bool self_test_result;
	u8 self_test_status[4];
};

struct lge_touch_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct mxt_data *ts, char *buf);
	ssize_t (*store)(struct mxt_data *ts, const char *buf, size_t count);
};

#define LGE_TOUCH_ATTR(_name, _mode, _show, _store)	\
struct lge_touch_attribute lge_touch_attr_##_name = __ATTR(_name, _mode, _show, _store)
#ifdef MXT_GESTURE_RECOGNIZE
static void mxt_gesture_mode_start(struct mxt_data *data);
static void mxt_active_mode_start(struct mxt_data *data);
#endif

#ifdef MXT_GESTURE_RECOGNIZE
static struct wake_lock touch_wake_lock;
static struct mutex i2c_suspend_lock;
static bool touch_irq_wake = 0;

static int touch_enable_irq_wake(unsigned int irq){
	int ret = 0;

	TOUCH_INFO_MSG("enable touch irq wake(%d)\n", touch_irq_wake);
	if(!touch_irq_wake){
		touch_irq_wake = 1;
		ret= enable_irq_wake(irq);
	}
	return ret;
}
static int touch_disable_irq_wake(unsigned int irq){
	int ret = 0;

	TOUCH_INFO_MSG("disable touch irq wake(%d)\n", touch_irq_wake);
	if(touch_irq_wake){
		touch_irq_wake = 0;
		ret = disable_irq_wake(irq);
	}
	return ret;
}
#endif

static bool touch_enable = 1;
static void touch_enable_irq(unsigned int irq){
	TOUCH_INFO_MSG("enable touch irq(%d)\n", touch_enable);

	if(!touch_enable){
		touch_enable = 1;
		enable_irq(irq);
	}
}
static void touch_disable_irq(unsigned int irq){
	TOUCH_INFO_MSG("disable touch irq(%d)\n", touch_enable);

	if(touch_enable){
		touch_enable = 0;
		disable_irq(irq);
	}
}

static inline size_t mxt_obj_size(const struct mxt_object *obj)
{
	return obj->size_minus_one + 1;
}

static inline size_t mxt_obj_instances(const struct mxt_object *obj)
{
	return obj->instances_minus_one + 1;
}

static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_COMMAND_T6:
	case MXT_SPT_USERDATA_T38:
	case MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_TOUCH_PROXIMITY_T23:
#ifdef MXT_GESTURE_RECOGNIZE
	case MXT_PROCI_ONETOUCH_T24:
#endif
	case MXT_SPT_SELFTEST_T25:
	case MXT_PROCI_GRIPSUPPRESSION_T40:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_SPT_CTECONFIG_T46:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCI_ADAPTIVETHRESHOLD_T55:
	case MXT_PROCI_SHIELDLESS_T56:
	/*case MXT_SPT_TIMER_T61:
	case MXT_PROCI_LENSBENDING_T65:
	case MXT_SPT_GOLDENREFERENCES_T66:
	case MXT_PROCI_PALMGESTUREPROCESSOR_T69:
	case MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70:
	case MXT_PROCG_NOISESUPPRESSION_T72:
	case MXT_GLOVEDETECTION_T78:
	case MXT_RETRANSMISSIONCOMPENSATION_T80:
	case MXT_PROCI_GESTUREPROCESSOR_T84:
	case MXT_TOUCH_MULTITOUCHSCREEN_T100:
	case MXT_SPT_TOUCHSCREENHOVER_T101:
	case MXT_SPT_SELFCAPCBCRCONFIG_T102:
	case MXT_PROCI_SCHNOISESUPPRESSION_T103:
	case MXT_SPT_AUXTOUCHCONFIG_T104:
	case MXT_SPT_DRIVENPLATEHOVERCONFIG_T105:*/
		return true;
	default:
		return false;
	}
}

static void mxt_dump_message(struct mxt_data *data, u8 *message)
{
	print_hex_dump(KERN_DEBUG, "[Touch] MXT MSG:", DUMP_PREFIX_NONE, 16, 1,
		       message, data->T5_msg_size, false);
}

static int mxt_wait_for_completion(struct mxt_data *data,
			struct completion *comp, unsigned int timeout_ms)
{
	//struct device *dev = &data->client->dev;
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	long ret;

	ret = wait_for_completion_interruptible_timeout(comp, timeout);
	if (ret < 0) {
		TOUCH_INFO_MSG("Wait for completion interrupted.\n");
		return -EINTR;
	} else if (ret == 0) {
		TOUCH_INFO_MSG("Wait for completion timed out.\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int mxt_bootloader_read(struct mxt_data *data,
			       u8 *val, unsigned int count)
{
	int ret;
	struct i2c_msg msg;

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = val;

	ret = i2c_transfer(data->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = (ret < 0) ? ret : -EIO;
		TOUCH_INFO_MSG("i2c recv failed (%d)\n", ret);
	}

	return ret;
}

static int mxt_bootloader_write(struct mxt_data *data,
				const u8 * const val, unsigned int count)
{
	int ret;
	struct i2c_msg msg;

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (u8 *)val;

	ret = i2c_transfer(data->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = (ret < 0) ? ret : -EIO;
		TOUCH_INFO_MSG("i2c send failed (%d)\n", ret);
	}

	return ret;
}

static int mxt_lookup_bootloader_address(struct mxt_data *data, u8 retry)
{
	u8 appmode = data->client->addr;
	u8 bootloader;
	u8 family_id = 0;

	if (data->info)
		family_id = data->info->family_id;

	TOUCH_INFO_MSG("%s appmode=0x%x\n", __func__, appmode);

	switch (appmode) {
	case 0x4a:
	case 0x4b:
		/* Chips after 1664S use different scheme */
		if ((retry % 2) || family_id >= 0xa2) {
			bootloader = appmode - 0x24;
			break;
		}
		/* Fall through for normal case */
	case 0x4c:
	case 0x4d:
	case 0x5a:
	case 0x5b:
		bootloader = appmode - 0x26;
		break;
	default:
		TOUCH_INFO_MSG(
			"Appmode i2c address 0x%02x not found\n",
			appmode);
		return -EINVAL;
	}

	data->bootloader_addr = bootloader;
	TOUCH_INFO_MSG("%s bootloader_addr=0x%x\n", __func__, bootloader);
	return 0;
}

static int mxt_probe_bootloader(struct mxt_data *data, u8 retry)
{
	//struct device *dev = &data->client->dev;
	int ret;
	u8 val;
	bool crc_failure;
	TOUCH_INFO_MSG("%s\n", __func__);
	ret = mxt_lookup_bootloader_address(data, retry);
	if (ret)
		return ret;

	ret = mxt_bootloader_read(data, &val, 1);
	if (ret)
		return ret;

	/* Check app crc fail mode */
	crc_failure = (val & ~MXT_BOOT_STATUS_MASK) == MXT_APP_CRC_FAIL;

	TOUCH_INFO_MSG("Detected bootloader, status:%02X%s\n",
			val, crc_failure ? ", APP_CRC_FAIL" : "");

	return 0;
}

static u8 mxt_get_bootloader_version(struct mxt_data *data, u8 val)
{
	//struct device *dev = &data->client->dev;
	u8 buf[3];

	if (val & MXT_BOOT_EXTENDED_ID) {
		if (mxt_bootloader_read(data, &buf[0], 3) != 0) {
			TOUCH_INFO_MSG("%s: i2c failure\n", __func__);
			return -EIO;
		}

		TOUCH_INFO_MSG("Bootloader ID:%d Version:%d\n", buf[1], buf[2]);

		return buf[0];
	} else {
		TOUCH_INFO_MSG("Bootloader ID:%d\n", val & MXT_BOOT_ID_MASK);

		return val;
	}
}

static int mxt_check_bootloader(struct mxt_data *data, unsigned int state)
{
	//struct device *dev = &data->client->dev;
	u8 val;
	int ret;

recheck:
	if (state != MXT_WAITING_BOOTLOAD_CMD) {
		/*
		 * In application update mode, the interrupt
		 * line signals state transitions. We must wait for the
		 * CHG assertion before reading the status byte.
		 * Once the status byte has been read, the line is deasserted.
		 */
		ret = mxt_wait_for_completion(data, &data->bl_completion,
					      MXT_FW_CHG_TIMEOUT);
		if (ret) {
			/*
			 * TODO: handle -EINTR better by terminating fw update
			 * process before returning to userspace by writing
			 * length 0x000 to device (iff we are in
			 * WAITING_FRAME_DATA state).
			 */
			TOUCH_INFO_MSG("Update wait error %d\n", ret);
			return ret;
		}
	}

	ret = mxt_bootloader_read(data, &val, 1);
	if (ret)
		return ret;

	if (state == MXT_WAITING_BOOTLOAD_CMD)
		val = mxt_get_bootloader_version(data, val);

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
	case MXT_WAITING_FRAME_DATA:
	case MXT_APP_CRC_FAIL:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK) {
			goto recheck;
		} else if (val == MXT_FRAME_CRC_FAIL) {
			TOUCH_INFO_MSG("Bootloader CRC fail\n");
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		TOUCH_INFO_MSG("Invalid bootloader state %02X != %02X\n",
			val, state);
		return -EINVAL;
	}

	return 0;
}

static int mxt_send_bootloader_cmd(struct mxt_data *data, bool unlock)
{
	int ret;
	u8 buf[2];

	TOUCH_INFO_MSG("%s unlock=%d\n", __func__, unlock);

	if (unlock) {
		buf[0] = MXT_UNLOCK_CMD_LSB;
		buf[1] = MXT_UNLOCK_CMD_MSB;
	} else {
		buf[0] = 0x01;
		buf[1] = 0x01;
	}

	ret = mxt_bootloader_write(data, buf, 2);
	if (ret)
		return ret;

	return 0;
}

static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];
#ifdef MXT_GESTURE_RECOGNIZE
	int i=0;
#else
	int ret;
	bool retry = false;
#endif


	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;
#ifdef MXT_GESTURE_RECOGNIZE
	do {
		if (i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer))==2)
			return 0;
		TOUCH_INFO_MSG("%s: i2c retry %d\n", __func__, i+1);
		msleep(MXT_WAKEUP_TIME);
	} while (++i < 10);

	TOUCH_INFO_MSG("%s: i2c transfer failed\n", __func__);
	return -EIO;
#else
retry_read:
	ret = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret != ARRAY_SIZE(xfer)) {
		if (!retry) {
			TOUCH_INFO_MSG("%s: i2c retry\n", __func__);
			msleep(MXT_WAKEUP_TIME);
			retry = true;
			goto retry_read;
		} else {
			TOUCH_INFO_MSG("%s: i2c transfer failed (%d)\n",
				__func__, ret);
			return -EIO;
		}
	}

	return 0;
#endif
}

static int __mxt_write_reg(struct i2c_client *client, u16 reg, u16 len,
			   const void *val)
{
	u8 *buf;
	size_t count;
#ifdef MXT_GESTURE_RECOGNIZE
	int i = 0;
#else
	int ret;
	bool retry = false;
#endif

	count = len + 2;
	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	memcpy(&buf[2], val, len);
#ifdef MXT_GESTURE_RECOGNIZE
	do {
		if (i2c_master_send(client, buf, count)==count){
			kfree(buf);
			return 0;
		}
		TOUCH_INFO_MSG("%s: i2c retry %d\n", __func__, i+1);
		msleep(MXT_WAKEUP_TIME);
	} while (++i < 10);
		TOUCH_INFO_MSG("%s: i2c transfer failed\n", __func__);
		kfree(buf);
		return -EIO;
#else
retry_write:
	ret = i2c_master_send(client, buf, count);
	if (ret == count) {
		ret = 0;
	} else {
		if (!retry) {
			TOUCH_INFO_MSG("%s: i2c retry\n", __func__);
			msleep(MXT_WAKEUP_TIME);
			retry = true;
			goto retry_write;
		} else {
			TOUCH_INFO_MSG("%s: i2c send failed (%d)\n",
				__func__, ret);
			ret = -EIO;
		}
	}

	kfree(buf);
	return ret;
#endif
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	return __mxt_write_reg(client, reg, 1, &val);
}

static struct mxt_object *
mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	TOUCH_INFO_MSG("Invalid object type T%u\n", type);
	return NULL;
}
#ifdef T37_DBG_DATA
static int mxt_read_object(struct mxt_data *data, u8 type, u8 offset, u8 *val)
{
	struct mxt_object *object;
	int error = 0;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(data->client, object->start_address + offset, 1, val);
	if (error)
		TOUCH_INFO_MSG("Error to read T[%d] offset[%d] val[%d]\n",
				type, offset, *val);

	return error;
}

static int mxt_write_object(struct mxt_data *data,
				 u8 type, u8 offset, u8 val)
{
	struct mxt_object *object;
	int error = 0;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	if (offset >= mxt_obj_size(object) * mxt_obj_instances(object)) {
		TOUCH_INFO_MSG("Tried to write outside object T%d offset:%d, size:%d\n",
				type, offset, mxt_obj_size(object));

		return -EINVAL;
	}

	reg = object->start_address;
	error = __mxt_write_reg(data->client, reg + offset, 1, &val);
	if (error)
		TOUCH_INFO_MSG("Error to write T[%d] offset[%d] val[%d]\n",
				type, offset, val);

	return error;
}

static int mxt_set_diagnostic_mode(struct mxt_data *data, u8 dbg_mode)
{
	//struct i2c_client *client = data->client;
	u8 cur_mode;
	int ret;

	ret = mxt_write_object(data, MXT_GEN_COMMAND_T6/*MXT_GEN_COMMANDPROCESSOR_T6*/,
			MXT_COMMAND_DIAGNOSTIC, dbg_mode);

	if (ret) {
		TOUCH_INFO_MSG("Failed change diagnositc mode to %d\n",
			 dbg_mode);
		goto out;
	}

	if (dbg_mode & MXT_DIAG_MODE_MASK) {
		do {
			ret = mxt_read_object(data, MXT_DEBUG_DIAGNOSTIC_T37,
				MXT_DIAGNOSTIC_MODE, &cur_mode);
			if (ret) {
				TOUCH_INFO_MSG("Failed getting diagnositc mode\n");
				goto out;
			}
			msleep(20);
		} while (cur_mode != dbg_mode);
		TOUCH_INFO_MSG("current dianostic chip mode is %d\n", cur_mode);
	}

out:
	return ret;
}

static bool mxt_check_xy_range(struct mxt_data *data, u16 node)
{
	u8 x_line = node / data->info->matrix_ysize;
	u8 y_line = node % data->info->matrix_ysize;
	return (y_line < data->rawdata->num_ynode) ?
		(x_line < data->rawdata->num_xnode) : false;
}



static void mxt_treat_dbg_data(struct mxt_data *data,
	struct mxt_object *dbg_object, u8 dbg_mode, u8 read_point, u16 num, char *buf, int* len)
{
	//struct i2c_client *client = data->client;
	struct mxt_raw_data *rawdata = data->rawdata;
	u8 data_buffer[DATA_PER_NODE] = { 0 };

	if (dbg_mode == MXT_DIAG_DELTA_MODE) {
		/* read delta data */
		__mxt_read_reg(data->client, dbg_object->start_address + read_point,
			DATA_PER_NODE, data_buffer);

		rawdata->delta[num] =
			((u16)data_buffer[1]<<8) + (u16)data_buffer[0];

		*len += snprintf(buf + *len , PAGE_SIZE - *len, "%4d", rawdata->delta[num]);

		TOUCH_INFO_MSG("delta[%d] = %d\n",
			num, rawdata->delta[num]);
	} else if (dbg_mode == MXT_DIAG_REFERENCE_MODE) {
		/* read reference data */
		__mxt_read_reg(data->client, dbg_object->start_address + read_point,
			DATA_PER_NODE, data_buffer);

		rawdata->reference[num] =
			((u16)data_buffer[1] << 8) + (u16)data_buffer[0]
			- REF_OFFSET_VALUE;

		TOUCH_INFO_MSG("reference[%d] = %d\n",
				num, rawdata->reference[num]);

		*len += snprintf(buf + *len , PAGE_SIZE - *len, "%6d", rawdata->reference[num]);
	}
}

static void mxt_treat_dbg_data_temp(struct mxt_data *data,
	struct mxt_object *dbg_object, u8 dbg_mode, u8 read_point, u16 num)
{
	struct mxt_raw_data *rawdata = data->rawdata;
	u8 data_buffer[DATA_PER_NODE] = { 0 };

	if (dbg_mode == MXT_DIAG_REFERENCE_MODE) {
		/* read reference data */
		__mxt_read_reg(data->client, dbg_object->start_address + read_point,
			DATA_PER_NODE, data_buffer);

		rawdata->reference[num] =
			((u16)data_buffer[1] << 8) + (u16)data_buffer[0]
			- REF_OFFSET_VALUE;

		TOUCH_INFO_MSG("reference[%d] = %d\n", num, rawdata->reference[num]);
	}
}

static void mxt_prepare_debug_data(struct mxt_data *data)
{
	struct mxt_raw_data *rawdata = NULL;
	int error = 0;

	rawdata = kzalloc(sizeof(struct mxt_raw_data), GFP_KERNEL);
	if (rawdata == NULL) {
		TOUCH_INFO_MSG("Fail to allocate sysfs data.\n");
		error = -ENOMEM;
		return ;
	}

	rawdata->num_xnode = data->max_x;
	rawdata->num_ynode = data->max_y;
	rawdata->num_nodes = rawdata->num_xnode * rawdata->num_ynode;

	TOUCH_INFO_MSG("%s: x=%d, y=%d, total=%d\n",
		__func__, rawdata->num_xnode,
		rawdata->num_ynode, rawdata->num_nodes);


	rawdata->reference = kzalloc(rawdata->num_nodes * sizeof(u16),
				   GFP_KERNEL);
	if (!rawdata->reference) {
		TOUCH_INFO_MSG("Fail to alloc reference of rawdata\n");
		error = -ENOMEM;
		goto err_alloc_reference;
	}

	rawdata->delta = kzalloc(rawdata->num_nodes * sizeof(s16), GFP_KERNEL);
	if (!rawdata->delta) {
		TOUCH_INFO_MSG("Fail to alloc delta of fdata\n");
		error = -ENOMEM;
		goto err_alloc_delta;
	}

	data->rawdata = rawdata;

	//return error;
	return ;
err_alloc_delta:
err_alloc_reference:
	kfree(rawdata->delta);
	kfree(rawdata->reference);
	kfree(rawdata);
	//return error;
	return ;

}

static void mxt_prepare_debug_data_temp(struct mxt_data *data)
{
	struct mxt_raw_data *rawdata = NULL;
	int error = 0;

	rawdata = kzalloc(sizeof(struct mxt_raw_data), GFP_KERNEL);
	if (rawdata == NULL) {
		TOUCH_INFO_MSG("Fail to allocate sysfs data.\n");
		error = -ENOMEM;
		return ;
	}

	rawdata->num_xnode = 2;
	rawdata->num_ynode = 14;
	rawdata->num_nodes = rawdata->num_xnode * rawdata->num_ynode;

	TOUCH_INFO_MSG("%s: x=%d, y=%d, total=%d\n",
		__func__, rawdata->num_xnode,
		rawdata->num_ynode, rawdata->num_nodes);

	rawdata->reference = kzalloc(rawdata->num_nodes * sizeof(u16), GFP_KERNEL);
	if (!rawdata->reference) {
		TOUCH_INFO_MSG("Fail to alloc reference of rawdata\n");
		error = -ENOMEM;
		goto err_alloc_reference;
	}
	data->rawdata = rawdata;

	return ;

err_alloc_reference:
	if (rawdata->reference)
		kfree(rawdata->reference);
	if (rawdata)
		kfree(rawdata);

	return ;
}

static int mxt_read_all_diagnostic_data(struct mxt_data *data, u8 dbg_mode, char *buf, int* len)
{
	//struct i2c_client *client = data->client;

	//struct mxt_raw_data *rawdata = data->rawdata;

	struct mxt_object *dbg_object;
	u8 read_page, cur_page, end_page, read_point;
	u16 node, num = 0,  cnt = 0;
	int ret = 0;

	/* to make the Page Num to 0 */
	ret = mxt_set_diagnostic_mode(data, MXT_DIAG_CTE_MODE);
	if (ret)
		goto out;

	/* change the debug mode */
	ret = mxt_set_diagnostic_mode(data, dbg_mode);
	if (ret)
		goto out;

	/* get object info for diagnostic */
	dbg_object = mxt_get_object(data, MXT_DEBUG_DIAGNOSTIC_T37);
	if (!dbg_object) {
		TOUCH_INFO_MSG("fail to get object_info\n");
		ret = -EINVAL;
		goto out;
	}

	//   
	mxt_prepare_debug_data(data);

	end_page = (data->info->matrix_xsize * data->info->matrix_ysize)
				/ NODE_PER_PAGE;

	/* read the dbg data */
	for (read_page = 0 ; read_page < end_page; read_page++) {
		for (node = 0; node < NODE_PER_PAGE; node++) {
			if(cnt%data->info->matrix_ysize == 0)
				*len += snprintf(buf + *len , PAGE_SIZE - *len, "\n[X%02d] ", cnt/data->info->matrix_ysize);
			read_point = (node * DATA_PER_NODE) + 2;

			if (mxt_check_xy_range(data, cnt)) {
				mxt_treat_dbg_data(data, dbg_object, dbg_mode,
					read_point, num, buf, len);
				num++;
			}
			cnt++;
			/*
			if(cnt%data->info->matrix_ysize == 0){
				*len += snprintf(buf + *len, PAGE_SIZE - *len, "-----------------------------------------------");
				*len += snprintf(buf + *len, PAGE_SIZE - *len, "-----------------------------------------------\n");
			}*/

		}
		ret = mxt_set_diagnostic_mode(data, MXT_DIAG_PAGE_UP);
		if (ret)
			goto out;
		do {
			msleep(20);
			ret = __mxt_read_reg(data->client,
				dbg_object->start_address + MXT_DIAGNOSTIC_PAGE,
				1, &cur_page);
			if (ret) {
				TOUCH_INFO_MSG("%s Read fail page\n", __func__);
				goto out;
			}
		} while (cur_page != read_page + 1);
	}

out:
	return ret;
}

static int mxt_read_one_page_diagnostic_data(struct mxt_data *data, u8 dbg_mode)
{
	struct mxt_object *dbg_object;
	u8 read_point;
	u16 node, num = 0,  cnt = 0;
	int ret = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	/* to make the Page Num to 0 */
	ret = mxt_set_diagnostic_mode(data, MXT_DIAG_CTE_MODE);
	if (ret)
		goto out;

	/* change the debug mode */
	ret = mxt_set_diagnostic_mode(data, dbg_mode);
	if (ret)
		goto out;

	/* get object info for diagnostic */
	dbg_object = mxt_get_object(data, MXT_DEBUG_DIAGNOSTIC_T37);
	if (!dbg_object) {
		TOUCH_INFO_MSG("fail to get object_info\n");
		ret = -EINVAL;
		goto out;
	}

	//   
	mxt_prepare_debug_data_temp(data);

	/* read the dbg data */
	for (node = 0; node < 28; node++) {
		read_point = (node * DATA_PER_NODE) + 2;

		if (mxt_check_xy_range(data, cnt)) {
			mxt_treat_dbg_data_temp(data, dbg_object, dbg_mode, read_point, num);
			num++;
		}
		cnt++;
	}
out:
	return ret;
}

static int run_reference_read(void *device_data, char *buf, int *len)
{
	struct mxt_data *data = (struct mxt_data *)device_data;
	//struct mxt_raw_data *rawdata = data->rawdata;
	int ret;

	ret = mxt_read_all_diagnostic_data(data,
			MXT_DIAG_REFERENCE_MODE, buf, len);

	return ret;
}


static int run_delta_read(void *device_data, char *buf, int *len)
{
	struct mxt_data *data = (struct mxt_data *)device_data;
	//struct mxt_fac_data *fdata = data->fdata;
	int ret;

	ret = mxt_read_all_diagnostic_data(data,
			MXT_DIAG_DELTA_MODE, buf, len);
	return ret;
}
#endif

static void mxt_proc_t6_messages(struct mxt_data *data, u8 *msg)
{
	//struct device *dev = &data->client->dev;
	u8 status = msg[1];
	u32 crc = msg[2] | (msg[3] << 8) | (msg[4] << 16);

	if (crc != data->config_crc) {
		data->config_crc = crc;
		TOUCH_INFO_MSG("T6 Config Checksum: 0x%06X\n", crc);
		complete(&data->crc_completion);
	}

	/* Detect transition out of reset */
	if ((data->t6_status & MXT_T6_STATUS_RESET) &&
	    !(status & MXT_T6_STATUS_RESET))
		complete(&data->reset_completion);

	/* Output debug if status has changed */
	if (status != data->t6_status)
		TOUCH_INFO_MSG("T6 Status 0x%02X%s%s%s%s%s%s%s\n",
			status,
			(status == 0) ? " OK" : "",
			(status & MXT_T6_STATUS_RESET) ? " RESET" : "",
			(status & MXT_T6_STATUS_OFL) ? " OFL" : "",
			(status & MXT_T6_STATUS_SIGERR) ? " SIGERR" : "",
			(status & MXT_T6_STATUS_CAL) ? " CAL" : "",
			(status & MXT_T6_STATUS_CFGERR) ? " CFGERR" : "",
			(status & MXT_T6_STATUS_COMSERR) ? " COMSERR" : "");

	/* Save current status */
	data->t6_status = status;
}

static void mxt_input_button(struct mxt_data *data, u8 *message)
{
	struct input_dev *input = data->input_dev;
	const struct mxt_platform_data *pdata = data->pdata;
	bool button;
	int i;

	/* do not report events if input device not yet registered */
	if (!data->enable_reporting)
		return;

	/* Active-low switch */
	for (i = 0; i < pdata->t19_num_keys; i++) {
		if (pdata->t19_keymap[i] == KEY_RESERVED)
			continue;
		button = !(message[1] & (1 << i));
		input_report_key(input, pdata->t19_keymap[i], button);
	}
}

static void mxt_input_sync(struct input_dev *input_dev)
{
	input_mt_report_pointer_emulation(input_dev, false);
	input_sync(input_dev);
}

static void mxt_proc_t9_message(struct mxt_data *data, u8 *message)
{
	//struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	int id;
	u8 status;
	int x;
	int y;
	int area;
	int amplitude;
	u8 vector;
	int tool;

	/* do not report events if input device not yet registered */
	if (!data->enable_reporting)
		return;

	id = message[0] - data->T9_reportid_min;
	status = message[1];
	x = (message[2] << 4) | ((message[4] >> 4) & 0xf);
	y = (message[3] << 4) | ((message[4] & 0xf));

	/* Handle 10/12 bit switching */
	if (data->max_x < 1024)
		x >>= 2;
	if (data->max_y < 1024)
		y >>= 2;

	area = message[5];

	amplitude = message[6];
	vector = message[7];
/*
	TOUCH_INFO_MSG("[%u] %c%c%c%c%c%c%c%c x: %5u y: %5u area: %3u amp: %3u vector: %02X\n",
		id,
		(status & MXT_T9_DETECT) ? 'D' : '.',
		(status & MXT_T9_PRESS) ? 'P' : '.',
		(status & MXT_T9_RELEASE) ? 'R' : '.',
		(status & MXT_T9_MOVE) ? 'M' : '.',
		(status & MXT_T9_VECTOR) ? 'V' : '.',
		(status & MXT_T9_AMP) ? 'A' : '.',
		(status & MXT_T9_SUPPRESS) ? 'S' : '.',
		(status & MXT_T9_UNGRIP) ? 'U' : '.',
		x, y, area, amplitude, vector);
*/
	input_mt_slot(input_dev, id);

	if (status & MXT_T9_DETECT) {
		/* Multiple bits may be set if the host is slow to read the
		 * status messages, indicating all the events that have
		 * happened */
		if (status & MXT_T9_RELEASE) {
			input_mt_report_slot_state(input_dev,
						   MT_TOOL_FINGER, 0);
			mxt_input_sync(input_dev);
		}

		/* A reported size of zero indicates that the reported touch
		 * is a stylus from a linked Stylus T47 object. */
		if (area == 0) {
			area = MXT_TOUCH_MAJOR_T47_STYLUS;
			tool = MT_TOOL_PEN;
		} else {
			tool = MT_TOOL_FINGER;
		}

		if (status & MXT_T9_PRESS)
			TOUCH_INFO_MSG("Press   <%d> : x[%3d] y[%3d] z[%3d]\n", id, x, y, area);

		/* Touch active */
		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, amplitude);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, area);
		input_report_abs(input_dev, ABS_MT_ORIENTATION, vector);
	} else {
		/* Touch no longer active, close out slot */
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
		TOUCH_INFO_MSG("Release <%d> : x[%3d] y[%3d]\n", id, x, y);
	}

	data->update_input = true;
}
static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset, u8 value, bool wait);

static void mxt_proc_t100_anti_message(struct mxt_data *data, u8 *message)
{
	//struct device *dev = &data->client->dev;

	data->t100_anti_area = (message[6] << 8) | message[5];
	data->t100_touch_area = (message[4] << 8) | message[3];
	TOUCH_INFO_MSG("anti touch area(%d), touch area(%d)\n",
				data->t100_anti_area, data->t100_touch_area);

	if(data->t100_anti_area){
		TOUCH_INFO_MSG("anti touch area(%d). touch area(%d) Run Calibration.\n", data->t100_anti_area, data->t100_touch_area);
		mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
		data->t100_anti_area = 0;
		data->t100_touch_area = 0;
	}
}

static void mxt_proc_t100_message(struct mxt_data *data, u8 *message)
{
	//struct device *dev = &data->client->dev;
#ifndef CUST_B_TOUCH
	struct input_dev *input_dev = data->input_dev;
#endif
	int id;
	u8 status;
	int x;
	int y;
	int area;
	int amplitude;
	u8 vector;

	/* do not report events if input device not yet registered */
	if (!data->enable_reporting){
		TOUCH_INFO_MSG("return event\n");
		return;
	}

	id = message[0] - data->T100_reportid_min - 2;
#ifndef CUST_B_TOUCH
	/* ignore SCRSTATUS events */
	if (id < 0 || id >= data->pdata->numtouch) {
		TOUCH_INFO_MSG("limited number of finger is %d\n", data->pdata->numtouch);
		return;
	}
	input_mt_slot(input_dev, id);
#endif

	status = message[1];
	x = (message[3] << 8) | message[2];
	y = (message[5] << 8) | message[4];

	area = message[data->t100_aux_area];
	amplitude = message[data->t100_aux_ampl];
	vector =  message[data->t100_aux_vect];

/*
	if(status & (1 << 2))
		TOUCH_INFO_MSG("[%u] %c%c %s%s (%02X) x:%u y:%u amp:%u area:%02X vec:%02X\n",
			id,
			((status & MXT_T100_STATUS_MASK) == MXT_T100_PRESS) ? 'P' : '.',
			((status & MXT_T100_STATUS_MASK) == MXT_T100_RELEASE) ? 'R' : '.',
			((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_FINGER) ? "FIN" : ".",
			((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_STYLUS) ? "PEN" : ".",
			status, x, y, amplitude, area, vector);
*/
#ifdef CUST_B_TOUCH
	if (status & MXT_T100_DETECT) {
		/* Multiple bits may be set if the host is slow to read the
		* status messages, indicating all the events that have
	 	* happened */

		if ((status & MXT_T100_STATUS_MASK) == MXT_T100_RELEASE) {
			data->ts_data.curr_data[id].id = id;
			data->ts_data.curr_data[id].status = FINGER_RELEASED;
		}

		data->ts_data.curr_data[id].id = id;
		data->ts_data.curr_data[id].x_position = x;
		data->ts_data.curr_data[id].y_position = y;
		data->ts_data.curr_data[id].touch_major = area;
		data->ts_data.curr_data[id].pressure = amplitude;
		data->ts_data.curr_data[id].orientation = vector;
		data->ts_data.curr_data[id].tool = MT_TOOL_FINGER;

		if ((status & MXT_T100_STATUS_MASK) == MXT_T100_PRESS) {
			data->ts_data.curr_data[id].status = FINGER_PRESSED;
		}else if((status & MXT_T100_STATUS_MASK) == MXT_T100_MOVE){
			data->ts_data.curr_data[id].status = FINGER_MOVED;
		}

		TOUCH_INFO_MSG("%s : curr_data[%d] x(%d), y(%d), area(%d), amplitude(%d)\n",
				__func__, id, x, y, area, amplitude);

	} else {
		/* Touch Release */
		data->ts_data.curr_data[id].id = id;
		data->ts_data.curr_data[id].status = FINGER_RELEASED;
	}
#else
	if (status & MXT_T100_DETECT) {
		/* A reported size of zero indicates that the reported touch
		 * is a stylus from a linked Stylus T47 object. */
		if ((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_STYLUS)
			tool = MT_TOOL_PEN;
		else
			tool = MT_TOOL_FINGER;

		/* Touch active */
		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);

		if (data->t100_aux_ampl)
			input_report_abs(input_dev, ABS_MT_PRESSURE,
					 message[data->t100_aux_ampl]);

		if (data->t100_aux_area) {
			if (tool == MT_TOOL_PEN)
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
						 MXT_TOUCH_MAJOR_T47_STYLUS);
			else
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
						 message[data->t100_aux_area]);
		}

		if (data->t100_aux_vect)
			input_report_abs(input_dev, ABS_MT_ORIENTATION,
					 message[data->t100_aux_vect]);
	} else {
		/* Touch no longer active, close out slot */
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}
#endif
	data->update_input = true;
}

static char *get_touch_button_string(u16 key_code)
{
	static char str[16] = {0};

	switch(key_code) {
		case KEY_BACK : /*158 0x9E*/
			sprintf(str, "BACK");
			break;
		case KEY_HOMEPAGE : /*172 0xAC*/
			sprintf(str, "HOME");
			break;
		case KEY_MENU : /* 139 0x8B*/
			sprintf(str, "MENU");
			break;
		case KEY_SIMSWITCH : /*249 0xF9*/
			sprintf(str, "SIM_SWITCH");
			break;
		default :
			sprintf(str, "Unknown");
			break;
	}

	return str;
}

#if 1
static void mxt_proc_t15_messages(struct mxt_data *data, u8 *msg)
{
	struct input_dev *input_dev = data->input_dev;
	unsigned long keystates = le32_to_cpu(msg[2]);
	u16 keycode = 0;
	static u16 prev_keycode = 0;
	int i = 0;

	/* do not report events if input device not yet registered */
	if (!data->enable_reporting)
		return;

	for (i = 0; i < data->pdata->t15_num_keys; i++) {
		if (keystates == data->pdata->t15_keystate[i]) {
			keycode = data->pdata->t15_keymap[i];
			break;
		}
	}

	if (prev_keycode == 0 && keystates == 0) {
		TOUCH_INFO_MSG("unknown keystates:%d\n", (int)keystates);
		return;
	}

	if (prev_keycode == 0) {
		TOUCH_INFO_MSG("Key press:   %d[%s]\n", keycode, get_touch_button_string(keycode));
		input_report_key(input_dev, keycode, 1);
	} else {
		TOUCH_INFO_MSG("Key release: %d[%s]\n", prev_keycode, get_touch_button_string(prev_keycode));
		input_report_key(input_dev, prev_keycode, 0);
	}

	input_sync(input_dev);

	prev_keycode = keycode;

}
#else
static void mxt_proc_t15_messages(struct mxt_data *data, u8 *msg)
{
	struct input_dev *input_dev = data->input_dev;
	//struct device *dev = &data->client->dev;
	int key;
	bool curr_state, new_state;
	bool sync = false;
	unsigned long keystates = le32_to_cpu(msg[2]);

	TOUCH_INFO_MSG("%s keystates:%d\n", __func__, (int)keystates);

	/* do not report events if input device not yet registered */
	if (!data->enable_reporting)
		return;

	for (key = 0; key < data->pdata->t15_num_keys; key++) {
		curr_state = test_bit(key, &data->t15_keystatus);
		new_state = test_bit(key, &keystates);

		TOUCH_INFO_MSG("%s curr_state:%d new_state:%d \n", __func__, curr_state, new_state);

		if (!curr_state && new_state) {
			TOUCH_INFO_MSG("Key press: %u, %d[%s]\n", key,
				data->pdata->t15_keymap[key],
				get_touch_button_string(data->pdata->t15_keymap[key]));

			__set_bit(key, &data->t15_keystatus);
			input_report_key(input_dev, data->pdata->t15_keymap[key], 1);
			//input_event(input_dev, EV_KEY, data->pdata->t15_keymap[key], 1);
			sync = true;
		} else if (curr_state && !new_state) {
		TOUCH_INFO_MSG("Key release: %u, %d[%s]\n", key,
				data->pdata->t15_keymap[key],
				get_touch_button_string(data->pdata->t15_keymap[key]));

			__clear_bit(key, &data->t15_keystatus);
			input_report_key(input_dev, data->pdata->t15_keymap[key], 0);
			//input_event(input_dev, EV_KEY, data->pdata->t15_keymap[key], 0);
			sync = true;
		}
	}

	if (sync)
		input_sync(input_dev);
}
#endif

static void mxt_proc_t42_messages(struct mxt_data *data, u8 *msg)
{
	//struct device *dev = &data->client->dev;
	u8 status = msg[1];

	if (status & MXT_T42_MSG_TCHSUP)
		TOUCH_INFO_MSG("T42 suppress\n");
	else
		TOUCH_INFO_MSG("T42 normal\n");
}

static int mxt_proc_t48_messages(struct mxt_data *data, u8 *msg)
{
	//struct device *dev = &data->client->dev;
	u8 status, state;

	status = msg[1];
	state  = msg[4];

	TOUCH_INFO_MSG("T48 state %d status %02X %s%s%s%s%s\n",
			state,
			status,
			(status & 0x01) ? "FREQCHG " : "",
			(status & 0x02) ? "APXCHG " : "",
			(status & 0x04) ? "ALGOERR " : "",
			(status & 0x10) ? "STATCHG " : "",
			(status & 0x20) ? "NLVLCHG " : "");

	return 0;
}

static void mxt_proc_t63_messages(struct mxt_data *data, u8 *msg)
{
	//struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	u8 id;
	u16 x, y;
	u8 pressure;

	/* do not report events if input device not yet registered */
	if (!data->enable_reporting)
		return;

	/* stylus slots come after touch slots */
	id = data->num_touchids + (msg[0] - data->T63_reportid_min);

	if (id < 0 || id > (data->num_touchids + data->num_stylusids)) {
		TOUCH_INFO_MSG("invalid stylus id %d, max slot is %d\n",
			id, data->num_stylusids);
		return;
	}

	x = msg[3] | (msg[4] << 8);
	y = msg[5] | (msg[6] << 8);
	pressure = msg[7] & MXT_STYLUS_PRESSURE_MASK;

	TOUCH_INFO_MSG("[%d] %c%c%c%c x: %d y: %d pressure: %d stylus:%c%c%c%c\n",
		id,
		(msg[1] & MXT_STYLUS_SUPPRESS) ? 'S' : '.',
		(msg[1] & MXT_STYLUS_MOVE)     ? 'M' : '.',
		(msg[1] & MXT_STYLUS_RELEASE)  ? 'R' : '.',
		(msg[1] & MXT_STYLUS_PRESS)    ? 'P' : '.',
		x, y, pressure,
		(msg[2] & MXT_STYLUS_BARREL) ? 'B' : '.',
		(msg[2] & MXT_STYLUS_ERASER) ? 'E' : '.',
		(msg[2] & MXT_STYLUS_TIP)    ? 'T' : '.',
		(msg[2] & MXT_STYLUS_DETECT) ? 'D' : '.');

	input_mt_slot(input_dev, id);

	if (msg[2] & MXT_STYLUS_DETECT) {
		input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, pressure);
	} else {
		input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 0);
	}

	input_report_key(input_dev, BTN_STYLUS, (msg[2] & MXT_STYLUS_ERASER));
	input_report_key(input_dev, BTN_STYLUS2, (msg[2] & MXT_STYLUS_BARREL));

	mxt_input_sync(input_dev);
}

static int mxt_proc_t25_message(struct mxt_data *data, u8 *message)
{
	//struct device *dev = &data->client->dev;
	u8 status = message[1];

	if(!selftest_enable)
		return 0;

	TOUCH_INFO_MSG("T25 Self Test completed %u\n",status);

	if(selftest_show)
		data->self_test_status[0] = status;

	if ( status == 0xFE ) {
		TOUCH_INFO_MSG("[SUCCESS] All tests passed\n");
		data->self_test_result = true;
	} else {
		if (status == 0xFD) {
			TOUCH_INFO_MSG("[FAIL] Invalid test code\n");
		} else if (status == 0xFC)  {
			TOUCH_INFO_MSG("[FAIL] Unrelated fault\n");
		} else if (status == 0x01) {
			TOUCH_INFO_MSG("[FAIL] AVdd or XVdd is not present\n");
		} else if (status == 0x12) {
			TOUCH_INFO_MSG("[FAIL] Pin fault (SEQ_NUM %u, X_PIN %u, Y_PIN %u)\n", message[2], message[3], message[4]);
			if(selftest_show){
				data->self_test_status[1] = message[2];
				data->self_test_status[2] = message[3];
				data->self_test_status[3] = message[4];
			}
		} else if (status == 0x17) {
			TOUCH_INFO_MSG("[FAIL] Signal limit fault (TYPE_NUM %u, TYPE_INSTANCE %u)\n", message[2], message[3]);
			if(selftest_show){
				data->self_test_status[1] = message[2];
				data->self_test_status[2] = message[3];
			}
		} else;
		data->self_test_result = false;
	}

	selftest_enable = false;
	complete(&data->t25_completion);
	return 0;
}

static struct sys_device lge_touch_sys_device;
#ifdef MXT_GESTURE_RECOGNIZE
char *char_A[2] = { "TOUCH_CHARACTER_A=WAKEUP", NULL };
char *char_B[2] = { "TOUCH_CHARACTER_B=WAKEUP", NULL };
char *char_C[2] = { "TOUCH_CHARACTER_C=WAKEUP", NULL };
char *char_G[2] = { "TOUCH_CHARACTER_G=WAKEUP", NULL };
char *char_M[2] = { "TOUCH_CHARACTER_M=WAKEUP", NULL };
char *char_S[2] = { "TOUCH_CHARACTER_S=WAKEUP", NULL };
char *char_W[2] = { "TOUCH_CHARACTER_W=WAKEUP", NULL };
static void mxt_proc_t35_messages(struct mxt_data *data, u8 *message)
{
	u8 msg;

	if (data->in_bootloader)
		return;

	msg = message[1];

	if(msg == 0x41){		/*A*/
		TOUCH_INFO_MSG("character A recognize.\n");
		kobject_uevent_env(&lge_touch_sys_device.kobj, KOBJ_CHANGE, char_A);
	}else if(msg == 0x42){	/*B*/
		TOUCH_INFO_MSG("character B recognize.\n");
		kobject_uevent_env(&lge_touch_sys_device.kobj, KOBJ_CHANGE, char_B);
	}else if(msg == 0x43){	/*C*/
		TOUCH_INFO_MSG("character C recognize.\n");
		kobject_uevent_env(&lge_touch_sys_device.kobj, KOBJ_CHANGE, char_C);
	}else if(msg == 0x47){	/*G*/
		TOUCH_INFO_MSG("character G recognize.\n");
		kobject_uevent_env(&lge_touch_sys_device.kobj, KOBJ_CHANGE, char_G);
	}else if(msg == 0x4D){	/*M*/
		TOUCH_INFO_MSG("character M recognize.\n");
		kobject_uevent_env(&lge_touch_sys_device.kobj, KOBJ_CHANGE, char_M);
	}else if(msg == 0x53){	/*S*/
		TOUCH_INFO_MSG("character S recognize.\n");
		kobject_uevent_env(&lge_touch_sys_device.kobj, KOBJ_CHANGE, char_S);
	}else if(msg == 0x57){	/*W*/
		TOUCH_INFO_MSG("character W recognize.\n");
		kobject_uevent_env(&lge_touch_sys_device.kobj, KOBJ_CHANGE, char_W);
	}else{
		TOUCH_INFO_MSG("Unknown pattern recognize.\n");
	}
}

char *knockon_event[2] = { "TOUCH_GESTURE_WAKEUP=WAKEUP", NULL };
static void mxt_proc_t24_messages(struct mxt_data *data, u8 *message)
{
	u8 msg;
	int x;
	int y;

	if (data->in_bootloader)
		return;

	msg = message[1];

	x = (message[2] << 4) | ((message[4] >> 4) & 0xf);
	y = (message[3] << 4) | ((message[4] & 0xf));

	if(msg == 0x04) {
		TOUCH_INFO_MSG("Double_Tap x[%3d] y[%3d] \n", x, y);
		kobject_uevent_env(&lge_touch_sys_device.kobj, KOBJ_CHANGE, knockon_event);
	}
}
#endif
static int mxt_proc_message(struct mxt_data *data, u8 *message)
{
	u8 report_id = message[0];
	bool dump = data->debug_enabled;

	if (report_id == MXT_RPTID_NOMSG)
		return 0;

	if (report_id == data->T6_reportid) {
		mxt_proc_t6_messages(data, message);
	} else if (report_id >= data->T9_reportid_min
	    && report_id <= data->T9_reportid_max) {
		mxt_proc_t9_message(data, message);
	} else if (report_id == data->T100_reportid_min){
		mxt_proc_t100_anti_message(data, message);
	} else if (report_id > data->T100_reportid_min
	    && report_id <= data->T100_reportid_max) {
		mxt_proc_t100_message(data, message);
	} else if (report_id == data->T19_reportid) {
		mxt_input_button(data, message);
		data->update_input = true;
	} else if (report_id >= data->T63_reportid_min
		   && report_id <= data->T63_reportid_max) {
		mxt_proc_t63_messages(data, message);
	} else if (report_id >= data->T42_reportid_min
		   && report_id <= data->T42_reportid_max) {
		mxt_proc_t42_messages(data, message);
	} else if (report_id == data->T48_reportid) {
		mxt_proc_t48_messages(data, message);
	} else if (report_id >= data->T15_reportid_min
		   && report_id <= data->T15_reportid_max) {
		mxt_proc_t15_messages(data, message);
#ifdef MXT_GESTURE_RECOGNIZE
	} else if (report_id == data->T24_reportid) {
		if (data->mxt_knock_on_enable)
			mxt_proc_t24_messages(data, message);
	} else if (report_id == data->T35_reportid) {
		if (data->mxt_character_enable)
			mxt_proc_t35_messages(data, message);
#endif
	} else if (report_id == data->T25_reportid){
		mxt_proc_t25_message(data, message);
	} else {
		TOUCH_INFO_MSG("%s : Unknown report_id = %d \n", __func__, report_id);
	}

	if (dump)
		mxt_dump_message(data, message);

	return 1;
}

static int mxt_read_and_process_messages(struct mxt_data *data, u8 count)
{
	//struct device *dev = &data->client->dev;
	int ret;
	int i;
	u8 num_valid = 0;

	/* Safety check for msg_buf */
	if (count > data->max_reportid)
		return -EINVAL;

	/* Process remaining messages if necessary */
	ret = __mxt_read_reg(data->client, data->T5_address,
				data->T5_msg_size * count, data->msg_buf);
	if (ret) {
		TOUCH_INFO_MSG("Failed to read %u messages (%d)\n", count, ret);
		return ret;
	}

	for (i = 0;  i < count; i++) {
		ret = mxt_proc_message(data,
			data->msg_buf + data->T5_msg_size * i);

		if (ret == 1)
			num_valid++;
	}

	/* return number of messages read */
	return num_valid;
}

#ifdef CUST_B_TOUCH
//                                                                 
#if TOUCHEVENTFILTER
int set_minor_data(struct mxt_data *data, int area, u8 vector)
{
	//struct device *dev = &data->client->dev;

	u8 tmp;
	int component1;
	int component2;
	int magnitude = 0;
	int minor = 0;

	int i;

	/* 1. get componet data */
	// component1
	tmp = ( (vector >> 4) & 0xf);
	if( tmp & 0x8 )
		component1 = (int)(((~(tmp) ) & 0xf) | 0x1 );
	else
		component1 = tmp;

	// component2
	tmp = (vector & 0xf);
	if( tmp & 0x8 )
		component2 = (int)(((~(tmp) ) & 0xf) | 0x1 );
	else
		component2 = tmp;

	/* 2. magnitude = SQRT(component1^2 + component2^2) */
	tmp = (component1 * component1) + (component2 * component2);

	/* 3. make minor date
	// when the magnitude is same, the larger area has the longer minor value.
	// that means, when the area is same, the longer magnitude has the shorter minor value.
	// 3-1. if shape is circle, minor = major.
	// when the shape is circle, vector = 0x00 / 0x01 / 0x10 / 0x11
	// then magnitude^2 = 0 /1 / 1 / 2
	*/
	if ( tmp < 3 )
	{
		minor = area;
		magnitude = 1;
	}
	else {
	/* 3-2. if shape is elipse, minor = area / magnitude.	*/
	// find SQRT(magnitude^2)
		for( i = 9 ; i > 1 ; i--) {
			if ( tmp > ((i*i) - 1) ){
				magnitude = i;
				break;
			}
		}
		minor = area / magnitude;
	}

	TOUCH_INFO_MSG("%5u area: %5u minor: %5u magnitude: %5u vector: %5u component1: %5u component2: %5u \n",
			tmp, area, minor, magnitude, vector, component1, component2);

	return minor;
}
#endif
//                                                                 

static char* get_tool_type(struct mxt_data *data, struct t_data touch_data) {
	if (touch_data.tool == MT_TOOL_FINGER) {
		if (touch_data.is_pen) {
			return "PEN";
		} else {
			return "FINGER";
		}
	} else if (touch_data.tool == MT_TOOL_PALM) {
		return "PALM";
	} else if (touch_data.tool == MT_TOOL_PEN) {
		return "PEN";
	} else {
		TOUCH_INFO_MSG("Invalid tool type : %d", touch_data.tool);
	}
	return "Unknown";
}
#endif

static irqreturn_t mxt_process_messages_t44(struct mxt_data *data)
{
	//struct device *dev = &data->client->dev;
	int ret;
	u8 count, num_left;
#ifdef CUST_B_TOUCH
	int report_num = 0;
	char *tool_type;
	int i;
#endif

	TOUCH_INFO_MSG("%s \n", __func__);

	/* Read T44 and T5 together */
	ret = __mxt_read_reg(data->client, data->T44_address,
		data->T5_msg_size + 1, data->msg_buf);

	if (ret) {
		TOUCH_INFO_MSG("Failed to read T44 and T5 (%d)\n", ret);
		return IRQ_NONE;
	}

	count = data->msg_buf[0];

	if (count == 0) {
		TOUCH_INFO_MSG("Interrupt triggered but zero messages\n");
		return IRQ_NONE;
	} else if (count > data->max_reportid) {
		TOUCH_INFO_MSG("T44 count %d exceeded max report id\n", count);
		count = data->max_reportid;
	}
#ifdef CUST_B_TOUCH
	data->ts_data.total_num = 0;
#endif

	/* Process first message */
	ret = mxt_proc_message(data, data->msg_buf + 1);
	if (ret < 0) {
		TOUCH_INFO_MSG("Unexpected invalid message\n");
		return IRQ_NONE;
	}

	num_left = count - 1;

	/* Process remaining messages if necessary */
	if (num_left) {
		ret = mxt_read_and_process_messages(data, num_left);
		if (ret < 0)
			goto end;
		else if (ret != num_left)
			TOUCH_INFO_MSG("Unexpected invalid message\n");
	}
#ifdef CUST_B_TOUCH
	for (i = 0; i < data->pdata->numtouch; i++) {

		if (data->ts_data.curr_data[i].status == FINGER_INACTIVE &&
			data->ts_data.prev_data[i].status != FINGER_INACTIVE &&
			data->ts_data.prev_data[i].status != FINGER_RELEASED) {
			memcpy(&data->ts_data.curr_data[i], &data->ts_data.prev_data[i], sizeof(data->ts_data.prev_data[i]));
			data->ts_data.curr_data[i].skip_report = true;
		}else if (data->ts_data.curr_data[i].status == FINGER_INACTIVE) {
			continue;
		}

		if (data->ts_data.curr_data[i].status == FINGER_PRESSED ||
			data->ts_data.curr_data[i].status == FINGER_MOVED) {
			data->ts_data.total_num++;
		}
		report_num++;
	}

	if (!data->enable_reporting || !report_num)
		goto out;

	for (i = 0; i < data->pdata->numtouch; i++) {
		if (data->ts_data.curr_data[i].status == FINGER_INACTIVE || data->ts_data.curr_data[i].skip_report) {
			continue;
		}

		if (data->ts_data.curr_data[i].status == FINGER_RELEASED) {
			input_mt_slot(data->input_dev, data->ts_data.curr_data[i].id);
			if (data->ts_data.prev_data[i].tool == MT_TOOL_FINGER) {
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
			} else if (data->ts_data.prev_data[i].tool == MT_TOOL_PALM) {
				input_mt_report_slot_state(data->input_dev, MT_TOOL_PALM, 0);
			} else if (data->ts_data.prev_data[i].tool == MT_TOOL_PEN) {
				input_mt_report_slot_state(data->input_dev, MT_TOOL_PEN, 0);
			} else {
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
			}
		} else {
			input_mt_slot(data->input_dev, data->ts_data.curr_data[i].id);
			input_mt_report_slot_state(data->input_dev,
				data->ts_data.curr_data[i].tool, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,
				data->ts_data.curr_data[i].id);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
				data->ts_data.curr_data[i].x_position);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
				data->ts_data.curr_data[i].y_position);
			input_report_abs(data->input_dev, ABS_MT_PRESSURE,
				data->ts_data.curr_data[i].pressure);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR,
				data->ts_data.curr_data[i].touch_major);

#ifdef MXT_GESTURE_RECOGNIZE
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MINOR,
				data->ts_data.curr_data[i].touch_minor);
#else
			input_report_abs(data->input_dev, ABS_MT_ORIENTATION,
				data->ts_data.curr_data[i].orientation);

				data->ts_data.curr_data[i].touch_minor = set_minor_data(data,
													data->ts_data.curr_data[i].touch_major,
													data->ts_data.curr_data[i].orientation);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MINOR,
					data->ts_data.curr_data[i].touch_minor);
#endif
		//                                                                   
			#if TOUCHEVENTFILTER
			TOUCH_INFO_MSG("report_data[%d] : x: %d y: %d, z: %d, M: %d, m: %d, orient: %d)\n",
					data->ts_data.curr_data[i].id,
					data->ts_data.curr_data[i].x_position,
					data->ts_data.curr_data[i].y_position,
					data->ts_data.curr_data[i].pressure,
					data->ts_data.curr_data[i].touch_major,
					data->ts_data.curr_data[i].touch_minor,
					data->ts_data.curr_data[i].orientation
			);
			#else
			TOUCH_INFO_MSG("report_data[%d] : (x %d, y %d, presure %d, touch_major %d, orient %d)\n",
					i,
					data->ts_data.curr_data[i].x_position,
					data->ts_data.curr_data[i].y_position,
					data->ts_data.curr_data[i].pressure,
					data->ts_data.curr_data[i].touch_major,
					data->ts_data.curr_data[i].orientation
			);
			#endif
			//                                                                   
		}
#if DEBUG_ABS
		if (data->ts_data.curr_data[i].status == FINGER_PRESSED) {
			tool_type = get_tool_type(data, data->ts_data.curr_data[i]);
			TOUCH_INFO_MSG("%d %s Pressed <%d> : x[%4d] y[%4d], z[%3d]\n",
					data->ts_data.total_num, tool_type,
					data->ts_data.curr_data[i].id,
					data->ts_data.curr_data[i].x_position,
					data->ts_data.curr_data[i].y_position,
					data->ts_data.curr_data[i].pressure);
		} else if (data->ts_data.curr_data[i].status == FINGER_RELEASED) {
			tool_type = get_tool_type(data, data->ts_data.prev_data[i]);
			TOUCH_INFO_MSG("%s Released <%d> <%d P>\n",
					tool_type,
					data->ts_data.curr_data[i].id, data->ts_data.total_num);
		}
#endif
	}

	if(data->ts_data.total_num < data->ts_data.prev_total_num)
		TOUCH_INFO_MSG("Total_num(move+press)= %d\n",data->ts_data.total_num);
	if (data->ts_data.total_num) {
		data->ts_data.prev_total_num = data->ts_data.total_num;
		memcpy(data->ts_data.prev_data, data->ts_data.curr_data, sizeof(data->ts_data.curr_data));
	} else{
		data->ts_data.prev_total_num = 0;
		memset(data->ts_data.prev_data, 0, sizeof(data->ts_data.prev_data));
	}
	memset(data->ts_data.curr_data, 0, sizeof(data->ts_data.curr_data));
#endif


end:
	if (data->update_input) {
		mxt_input_sync(data->input_dev);
		data->update_input = false;
	}
#ifdef CUST_B_TOUCH
out:
#endif
	return IRQ_HANDLED;
}

static int mxt_process_messages_until_invalid(struct mxt_data *data)
{
	//struct device *dev = &data->client->dev;
	int count, read;
	u8 tries = 2;

	count = data->max_reportid;

	/* Read messages until we force an invalid */
	do {
		read = mxt_read_and_process_messages(data, count);
		if (read < count)
			return 0;
	} while (--tries);

	if (data->update_input) {
		mxt_input_sync(data->input_dev);
		data->update_input = false;
	}

	TOUCH_INFO_MSG("CHG pin isn't cleared\n");
	return -EBUSY;
}

static irqreturn_t mxt_process_messages(struct mxt_data *data)
{
	int total_handled, num_handled;
	u8 count = data->last_message_count;

	if (count < 1 || count > data->max_reportid)
		count = 1;

	/* include final invalid message */
	total_handled = mxt_read_and_process_messages(data, count + 1);
	if (total_handled < 0)
		return IRQ_NONE;
	/* if there were invalid messages, then we are done */
	else if (total_handled <= count)
		goto update_count;

	/* read two at a time until an invalid message or else we reach
	 * reportid limit */
	do {
		num_handled = mxt_read_and_process_messages(data, 2);
		if (num_handled < 0)
			return IRQ_NONE;

		total_handled += num_handled;

		if (num_handled < 2)
			break;
	} while (total_handled < data->num_touchids);

update_count:
	data->last_message_count = total_handled;

	if (data->enable_reporting && data->update_input) {
		mxt_input_sync(data->input_dev);
		data->update_input = false;
	}

	return IRQ_HANDLED;
}

static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;

	if (data->in_bootloader) {
		/* bootloader state transition completion */
		complete(&data->bl_completion);
		return IRQ_HANDLED;
	}

	if (!data->object_table)
		return IRQ_NONE;

	if (data->T44_address) {
		return mxt_process_messages_t44(data);
	} else {
		return mxt_process_messages(data);
	}
}

static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset,
			  u8 value, bool wait)
{
	u16 reg;
	u8 command_register = 0;
	int timeout_counter = 0;
	int ret;

	reg = data->T6_address + cmd_offset;

	ret = mxt_write_reg(data->client, reg, value);
	if (ret)
		return ret;

	if (!wait)
		return 0;

	do {
		msleep(20);
		ret = __mxt_read_reg(data->client, reg, 1, &command_register);
		if (ret)
			return ret;
	} while ((command_register != 0) && (timeout_counter++ <= 100));

	if (timeout_counter > 100) {
		TOUCH_INFO_MSG("Command failed!\n");
		return -EIO;
	}

	return 0;
}

static int mxt_t25_command(struct mxt_data *data, u8 value, bool wait)
{
	u16 reg;
	u8 command_register;
	int timeout_counter = 0;
	int ret;

	if(!selftest_enable)
		return 0;

	reg = data->T25_address + 1 ;

	ret = mxt_write_reg(data->client, reg, value);
	if (ret){
		TOUCH_INFO_MSG("Write Self Test Command fail!\n");
		return ret;
	}

	if (!wait)
		return 0;

	do {
		msleep(20);
		ret = __mxt_read_reg(data->client, reg, 1, &command_register);
		if (ret)
			return ret;
	} while ((command_register != 0) && (timeout_counter++ <= 100));

	if (timeout_counter > 100) {
		TOUCH_INFO_MSG("Command failed!\n");
		return -EIO;
	}

	return 0;
}

static int mxt_soft_reset(struct mxt_data *data)
{
	//struct device *dev = &data->client->dev;
	int ret = 0;

	TOUCH_INFO_MSG("Resetting chip\n");

	INIT_COMPLETION(data->reset_completion);

	ret = mxt_t6_command(data, MXT_COMMAND_RESET, MXT_RESET_VALUE, false);
	if (ret)
		return ret;

#ifdef CUST_B_TOUCH
	msleep(MXT_RESET_TIME);
#else
	ret = mxt_wait_for_completion(data, &data->reset_completion,
				      MXT_RESET_TIMEOUT);
	if (ret)
		return ret;
#endif
	return 0;
}

static void mxt_update_crc(struct mxt_data *data, u8 cmd, u8 value)
{
	/* on failure, CRC is set to 0 and config will always be downloaded */
	data->config_crc = 0;
	INIT_COMPLETION(data->crc_completion);

	mxt_t6_command(data, cmd, value, true);
	/* Wait for crc message. On failure, CRC is set to 0 and config will
	 * always be downloaded */
#ifdef CUST_B_TOUCH
	msleep(MXT_CRC_TIMEOUT);
#else
	mxt_wait_for_completion(data, &data->crc_completion, MXT_CRC_TIMEOUT);
#endif
}

static void mxt_calc_crc24(u32 *crc, u8 firstbyte, u8 secondbyte)
{
	static const unsigned int crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = (secondbyte << 8) | firstbyte;
	result = ((*crc << 1) ^ data_word);

	if (result & 0x1000000)
		result ^= crcpoly;

	*crc = result;
}

static u32 mxt_calculate_crc(u8 *base, off_t start_off, off_t end_off)
{
	u32 crc = 0;
	u8 *ptr = base + start_off;
	u8 *last_val = base + end_off - 1;

	if (end_off < start_off)
		return -EINVAL;

	while (ptr < last_val) {
		mxt_calc_crc24(&crc, *ptr, *(ptr + 1));
		ptr += 2;
	}

	/* if len is odd, fill the last byte with 0 */
	if (ptr == last_val)
		mxt_calc_crc24(&crc, *ptr, 0);

	/* Mask to 24-bit */
	crc &= 0x00FFFFFF;

	return crc;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data);

static int mxt_check_retrigen(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error = 0;
	int val = 0;

	if (data->pdata->irqflags & IRQF_TRIGGER_LOW)
		return 0;

	if (data->T18_address) {
		error = __mxt_read_reg(client,
				       data->T18_address + MXT_COMMS_CTRL,
				       1, &val);
		if (error)
			return error;

		if (val & MXT_COMMS_RETRIGEN)
			return 0;
	}

	dev_warn(&client->dev, "Enabling RETRIGEN workaround\n");
	data->use_retrigen_workaround = true;
	return 0;
}

/*
 * mxt_check_reg_init - download configuration to chip
 *
 * Atmel Raw Config File Format
 *
 * The first four lines of the raw config file contain:
 *  1) Version
 *  2) Chip ID Information (first 7 bytes of device memory)
 *  3) Chip Information Block 24-bit CRC Checksum
 *  4) Chip Configuration 24-bit CRC Checksum
 *
 * The rest of the file consists of one line per object instance:
 *   <TYPE> <INSTANCE> <SIZE> <CONTENTS>
 *
 *   <TYPE> - 2-byte object type as hex
 *   <INSTANCE> - 2-byte object instance number as hex
 *   <SIZE> - 2-byte object size as hex
 *   <CONTENTS> - array of <SIZE> 1-byte hex values
 */
static int mxt_check_reg_init(struct mxt_data *data, const char *name)
{
	struct device *dev = &data->client->dev;
	struct mxt_info cfg_info = {0};
	struct mxt_object *object = NULL;
	const struct firmware *cfg = NULL;
	char *cfg_name = NULL;
	int ret = 0;
	int offset = 0;
	int data_pos = 0;
	int byte_offset = 0;
	int i = 0;
	int cfg_start_ofs = 0;
	u32 info_crc = 0, config_crc = 0, calculated_crc = 0;
	u8 *config_mem = 0;
	size_t config_mem_size = 0;
	unsigned int type = 0, instance, size = 0;
	u8 val = 0;
	u16 reg = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	if (name)
		cfg_name = (char *)name;
	else
		cfg_name = (char *)data->cfg_name;

	if (!cfg_name) {
		TOUCH_INFO_MSG("Skipping cfg download\n");
		return 0;
	}

	ret = request_firmware(&cfg, cfg_name, dev);
	if (ret < 0) {
		TOUCH_INFO_MSG("Failure to request config file [%s]\n", cfg_name);
		return -EINVAL;
	}

	TOUCH_INFO_MSG("Open [%s] configuration file \n", cfg_name);

	mxt_update_crc(data, MXT_COMMAND_REPORTALL, 1);

	if (strncmp(cfg->data, MXT_CFG_MAGIC, strlen(MXT_CFG_MAGIC))) {
		TOUCH_INFO_MSG("Unrecognised config file\n");
		ret = -EINVAL;
		goto release;
	}

	data_pos = strlen(MXT_CFG_MAGIC);

	/* Load information block and check */
	for (i = 0; i < sizeof(struct mxt_info); i++) {
		ret = sscanf(cfg->data + data_pos, "%hhx%n", (unsigned char *)&cfg_info + i, &offset);
		if (ret != 1) {
			TOUCH_INFO_MSG("Bad format\n");
			ret = -EINVAL;
			goto release;
		}

		data_pos += offset;
	}

	/* Read CRCs */
	ret = sscanf(cfg->data + data_pos, "%x%n", &info_crc, &offset);
	if (ret != 1) {
		TOUCH_INFO_MSG("Bad format: failed to parse Info CRC\n");
		ret = -EINVAL;
		goto release;
	}
	data_pos += offset;

	ret = sscanf(cfg->data + data_pos, "%x%n", &config_crc, &offset);
	if (ret != 1) {
		TOUCH_INFO_MSG("Bad format: failed to parse Config CRC\n");
		ret = -EINVAL;
		goto release;
	}
	data_pos += offset;

	TOUCH_INFO_MSG("RAW Config CRC is 0x%06X \n", config_crc);

	if (data->config_crc == config_crc) {
		TOUCH_INFO_MSG("Config already applied \n");
		ret = 0;
		goto release_mem;
	}

	if (memcmp((char *)data->info, (char *)&cfg_info, sizeof(struct mxt_info)) != 0) {
		TOUCH_INFO_MSG("Compatibility Error. Could not apply\n");
			TOUCH_INFO_MSG("Info Block [IC]   %02X %02X %02X %02X %02X %02X %02X \n", 
			data->info->family_id, data->info->variant_id, data->info->version, data->info->build, 
			data->info->matrix_xsize, data->info->matrix_ysize, data->info->object_num);

		TOUCH_INFO_MSG("Info Block [File] %02X %02X %02X %02X %02X %02X %02X \n", 
			cfg_info.family_id, cfg_info.variant_id, cfg_info.version, cfg_info.build, 
			cfg_info.matrix_xsize, cfg_info.matrix_ysize, cfg_info.object_num);

		ret = -EINVAL;
		goto release_mem;
	}

	/* Malloc memory to store configuration */
	cfg_start_ofs = MXT_OBJECT_START + data->info->object_num * sizeof(struct mxt_object) + MXT_INFO_CHECKSUM_SIZE;
	config_mem_size = data->mem_size - cfg_start_ofs;
	config_mem = kzalloc(config_mem_size, GFP_KERNEL);
	if (!config_mem) {
		TOUCH_INFO_MSG("Failed to allocate memory\n");
		ret = -ENOMEM;
		goto release;
	}

	while (data_pos < cfg->size) {
		/* Read type, instance, length */
		ret = sscanf(cfg->data + data_pos, "%x %x %x%n", &type, &instance, &size, &offset);
		if (ret == 0) {
			/* EOF */
			break;
		} else if (ret != 3) {
			TOUCH_INFO_MSG("Bad format: failed to parse object\n");
			ret = -EINVAL;
			goto release_mem;
		}
		data_pos += offset;

		object = mxt_get_object(data, type);
		if (!object) {
			/* Skip object */
			for (i = 0; i < size; i++) {
				ret = sscanf(cfg->data + data_pos, "%hhx%n", &val, &offset);
				data_pos += offset;
			}
			continue;
		}

		if (instance >= mxt_obj_instances(object)) {
			TOUCH_INFO_MSG("Object instances exceeded!\n");
			ret = -EINVAL;
			goto release_mem;
		}

		reg = object->start_address + mxt_obj_size(object) * instance;

		TOUCH_INFO_MSG("%04X %04X %04X \n", type, instance, size);

		if (size != mxt_obj_size(object)) {
			TOUCH_INFO_MSG("Size mismatched \n"
);
			ret = -EINVAL;
			goto release_mem;
		}

		for (i = 0; i < size; i++) {
			ret = sscanf(cfg->data + data_pos, "%hhx%n", &val, &offset);
			if (ret != 1) {
				TOUCH_INFO_MSG("Bad format in T%d\n", type);
				ret = -EINVAL;
				goto release_mem;
			}

			if (i > mxt_obj_size(object))
				continue;

			byte_offset = reg + i - cfg_start_ofs;

			if ((byte_offset >= 0) && (byte_offset <= config_mem_size)) {
				*(config_mem + byte_offset) = val;
			} else {
				TOUCH_INFO_MSG("Bad object: reg:%d, T%d, ofs=%d\n", reg, object->type, byte_offset);
				ret = -EINVAL;
				goto release_mem;
			}

			data_pos += offset;
		}
	}

	/* calculate crc of the received configs (not the raw config file) */
	if (data->T7_address < cfg_start_ofs) {
		TOUCH_INFO_MSG("Bad T7 address, T7addr = %x, config offset %x\n", data->T7_address, cfg_start_ofs);
		ret = -EINVAL;
		goto release_mem;
	}

	calculated_crc = mxt_calculate_crc(config_mem, data->T7_address - cfg_start_ofs, config_mem_size);

	/* Check the crc, calculated should match what is in file */
	if (config_crc > 0 && (config_crc != calculated_crc)) {
		TOUCH_INFO_MSG("CRC mismatch in config file, calculated=0x%06X, file=0x%06X\n", calculated_crc, config_crc);
		TOUCH_INFO_MSG("Config not apply \n");
		ret = -EINVAL;
		goto release_mem;
	}

	/* Write configuration as blocks */
	byte_offset = 0;
	while (byte_offset < config_mem_size) {
		size = config_mem_size - byte_offset;

		if (size > MXT_MAX_BLOCK_WRITE)
			size = MXT_MAX_BLOCK_WRITE;

		ret = __mxt_write_reg(data->client, cfg_start_ofs + byte_offset, size, config_mem + byte_offset);
		if (ret != 0) {
			TOUCH_INFO_MSG( "Config write error, ret=%d\n", ret);
			ret = -EINVAL;
			goto release_mem;
		}

		byte_offset += size;
	}

	mxt_update_crc(data, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);

	if ((config_crc > 0) && (config_crc != data->config_crc)) {
		TOUCH_INFO_MSG("Config CRC is mismatched 0x%06X \n", data->config_crc);
	}

	ret = mxt_check_retrigen(data);
	if (ret)
		goto release_mem;

	ret = mxt_soft_reset(data);
	if (ret)
		goto release_mem;

	TOUCH_INFO_MSG("Config written\n");

	/* T7 config may have changed */
	mxt_init_t7_power_cfg(data);

release_mem:
	kfree(config_mem);
release:
	release_firmware(cfg);

	return ret;
}

#if 0
static int mxt_get_checksum(struct mxt_data *data, const char *cfg_name)
{
	struct device *dev = &data->client->dev;
	struct mxt_info cfg_info = {0};
	const struct firmware *cfg = NULL;
	int ret = 0;
	int offset = 0;
	int data_pos = 0;
	int i = 0;
	u32 info_crc = 0, config_crc = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	if (!cfg_name) {
		TOUCH_INFO_MSG("Skipping cfg download\n");
		return 0;
	}

	ret = request_firmware(&cfg, cfg_name, dev);
	if (ret < 0) {
		TOUCH_INFO_MSG("Failure to request config file [%s]\n", cfg_name);
		return -EINVAL;
	}

	TOUCH_INFO_MSG("Open [%s] configuration file \n", cfg_name);

	if (strncmp(cfg->data, MXT_CFG_MAGIC, strlen(MXT_CFG_MAGIC))) {
		TOUCH_INFO_MSG("Unrecognised config file\n");
		ret = -EINVAL;
		goto release;
	}

	data_pos = strlen(MXT_CFG_MAGIC);

	/* Load information block and check */
	for (i = 0; i < sizeof(struct mxt_info); i++) {
		ret = sscanf(cfg->data + data_pos, "%hhx%n", (unsigned char *)&cfg_info + i, &offset);
		if (ret != 1) {
			TOUCH_INFO_MSG("Bad format\n");
			ret = -EINVAL;
			goto release;
		}

		data_pos += offset;
	}

	/* Read CRCs */
	ret = sscanf(cfg->data + data_pos, "%x%n", &info_crc, &offset);
	if (ret != 1) {
		TOUCH_INFO_MSG("Bad format: failed to parse Info CRC\n");
		ret = -EINVAL;
		goto release;
	}
	data_pos += offset;

	ret = sscanf(cfg->data + data_pos, "%x%n", &config_crc, &offset);
	if (ret != 1) {
		TOUCH_INFO_MSG("Bad format: failed to parse Config CRC\n");
		ret = -EINVAL;
		goto release;
	}
	data_pos += offset;

	TOUCH_INFO_MSG("Checksum is 0x%06X \n", config_crc);

release:
	release_firmware(cfg);

	return config_crc;
}
#endif

static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 sleep)
{
	//struct device *dev = &data->client->dev;
	int error;
	struct t7_config *new_config;
	struct t7_config deepsleep = { .active = 0, .idle = 0 };

	if (sleep == MXT_POWER_CFG_DEEPSLEEP)
		new_config = &deepsleep;
	else
		new_config = &data->t7_cfg;

	error = __mxt_write_reg(data->client, data->T7_address,
			sizeof(data->t7_cfg),
			new_config);
	if (error)
		return error;

	TOUCH_INFO_MSG("Set T7 ACTV:%d IDLE:%d\n",
		new_config->active, new_config->idle);

	return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data)
{
	//struct device *dev = &data->client->dev;
	int error;
	bool retry = false;

recheck:
	error = __mxt_read_reg(data->client, data->T7_address,
				sizeof(data->t7_cfg), &data->t7_cfg);
	if (error)
		return error;

	if (data->t7_cfg.active == 0 || data->t7_cfg.idle == 0) {
		if (!retry) {
			TOUCH_INFO_MSG("T7 cfg zero, resetting\n");
			mxt_soft_reset(data);
			retry = true;
			goto recheck;
		} else {
		    TOUCH_INFO_MSG("T7 cfg zero after reset, overriding\n");
		    data->t7_cfg.active = 20;
		    data->t7_cfg.idle = 100;
		    return mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
		}
	} else {
		TOUCH_INFO_MSG("Initialised power cfg: ACTV %d, IDLE %d\n",
				data->t7_cfg.active, data->t7_cfg.idle);
		return 0;
	}
}

static int mxt_acquire_irq(struct mxt_data *data)
{
	int error;

	touch_enable_irq(data->irq);

	if (data->use_retrigen_workaround) {
		error = mxt_process_messages_until_invalid(data);
		if (error)
			return error;
	}

	return 0;
}

static void mxt_free_input_device(struct mxt_data *data)
{
	if (data->input_dev) {
		TOUCH_INFO_MSG("mxt_free_input_device\n");
		input_unregister_device(data->input_dev);
		data->input_dev = NULL;
	}
}

static void mxt_free_object_table(struct mxt_data *data)
{
	TOUCH_INFO_MSG("mxt_free_object_table\n");

	if(!update_cfg_force){
	kfree(data->raw_info_block);
	data->object_table = NULL;
	data->info = NULL;
	data->raw_info_block = NULL;
	kfree(data->msg_buf);
	data->msg_buf = NULL;

	mxt_free_input_device(data);
	data->enable_reporting = false;
	}

	data->T5_address = 0;
	data->T5_msg_size = 0;
	data->T6_reportid = 0;
	data->T7_address = 0;
	data->T8_address = 0;
	data->T9_reportid_min = 0;
	data->T9_reportid_max = 0;
	data->T15_reportid_min = 0;
	data->T15_reportid_max = 0;
	data->T18_address = 0;
	data->T19_reportid = 0;
#ifdef MXT_GESTURE_RECOGNIZE
	data->T24_reportid = 0;
	data->T35_reportid = 0;
#endif
	data->T25_reportid = 0;
	data->T42_address = 0;
	data->T42_reportid_min = 0;
	data->T42_reportid_max = 0;
	data->T44_address = 0;
	data->T46_address = 0;
	data->T47_address = 0;
	data->T48_reportid = 0;
	data->T56_address = 0;
	data->T63_reportid_min = 0;
	data->T63_reportid_max = 0;
	data->T65_address = 0;
	data->T72_address = 0;
	data->T100_reportid_min = 0;
	data->T100_reportid_max = 0;
	data->T100_address = 0;
	data->max_reportid = 0;
}

static int mxt_parse_object_table(struct mxt_data *data)
{
//	struct i2c_client *client = data->client;
	int i;
	u8 reportid;
	u16 end_address;

	/* Valid Report IDs start counting from 1 */
	reportid = 1;
	data->mem_size = 0;
	for (i = 0; i < data->info->object_num; i++) {
		struct mxt_object *object = data->object_table + i;
		u8 min_id, max_id;

		le16_to_cpus(&object->start_address);

		if (object->num_report_ids) {
			min_id = reportid;
			reportid += object->num_report_ids *
					mxt_obj_instances(object);
			max_id = reportid - 1;
		} else {
			min_id = 0;
			max_id = 0;
		}
/*
		TOUCH_INFO_MSG("T%u Start:%u Size:%u Instances:%u Report IDs:%u-%u\n",
			object->type, object->start_address,
			mxt_obj_size(object), mxt_obj_instances(object),
			min_id, max_id);
*/
		switch (object->type) {
		case MXT_GEN_MESSAGE_T5:
			if (data->info->family_id == 0x80) {
				/* On mXT224 read and discard unused CRC byte
				 * otherwise DMA reads are misaligned */
				data->T5_msg_size = mxt_obj_size(object);
			} else {
				/* CRC not enabled, so skip last byte */
				data->T5_msg_size = mxt_obj_size(object) - 1;
			}
			data->T5_address = object->start_address;
		case MXT_GEN_COMMAND_T6:
			data->T6_reportid = min_id;
			data->T6_address = object->start_address;
			break;
		case MXT_GEN_POWER_T7:
			data->T7_address = object->start_address;
			break;
		case MXT_GEN_ACQUIRE_T8:
			data->T8_address = object->start_address;
			break;
		case MXT_TOUCH_MULTI_T9:
			/* Only handle messages from first T9 instance */
			data->T9_reportid_min = min_id;
			data->T9_reportid_max = min_id +
						object->num_report_ids - 1;
			data->num_touchids = object->num_report_ids;
			data->T9_address = object->start_address;
			break;
		case MXT_TOUCH_KEYARRAY_T15:
			data->T15_reportid_min = min_id;
			data->T15_reportid_max = max_id;
			data->T15_address = object->start_address;
			break;
		case MXT_SPT_COMMSCONFIG_T18:
			data->T18_address = object->start_address;
			break;
#ifdef MXT_GESTURE_RECOGNIZE
		case MXT_PROCI_ONETOUCH_T24:
			data->T24_reportid = min_id;
			data->T24_address = object->start_address;
			break;
		case MXT_SPT_PROTOTYPE_T35:
			data->T35_reportid = min_id;
			break;
#endif
		case MXT_SPT_SELFTEST_T25:
			data->T25_reportid = min_id;
			data->T25_address = object->start_address;
			break;
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			data->T42_address = object->start_address;
			data->T42_reportid_min = min_id;
			data->T42_reportid_max = max_id;
			break;
		case MXT_SPT_CTECONFIG_T46:
			data->T46_address = object->start_address;
			break;
		case MXT_PROCI_STYLUS_T47:
			data->T47_address = object->start_address;
			break;
		case MXT_SPT_MESSAGECOUNT_T44:
			data->T44_address = object->start_address;
			break;
		case MXT_SPT_GPIOPWM_T19:
			data->T19_reportid = min_id;
			break;
		case MXT_SPT_NOISESUPPRESSION_T48:
			data->T48_reportid = min_id;
			break;
		case MXT_PROCI_SHIELDLESS_T56:
			data->T56_address = object->start_address;
			break;
		case MXT_PROCI_ACTIVE_STYLUS_T63:
			/* Only handle messages from first T63 instance */
			data->T63_reportid_min = min_id;
			data->T63_reportid_max = min_id;
			data->num_stylusids = 1;
			break;
		case MXT_PROCI_LENSBENDING_T65:
			data->T65_address = object->start_address;
			break;
		case MXT_PROCG_NOISESUPPRESSION_T72:
			data->T72_address = object->start_address;
			break;
		case MXT_TOUCH_MULTITOUCHSCREEN_T100:
			data->T100_reportid_min = min_id;
			data->T100_reportid_max = max_id;
			/* first two report IDs reserved */
			data->num_touchids = object->num_report_ids - 2;
			data->T100_address = object->start_address;
			break;
		}

		end_address = object->start_address
			+ mxt_obj_size(object) * mxt_obj_instances(object) - 1;

		if (end_address >= data->mem_size)
			data->mem_size = end_address + 1;
	}

	/* Store maximum reportid */
	data->max_reportid = reportid;

	/* If T44 exists, T5 position has to be directly after */
	if (data->T44_address && (data->T5_address != data->T44_address + 1)) {
		TOUCH_INFO_MSG("Invalid T44 position\n");
		return -EINVAL;
	}

	data->msg_buf = kcalloc(data->max_reportid,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->msg_buf) {
		TOUCH_INFO_MSG("Failed to allocate message buffer\n");
		return -ENOMEM;
	}

	return 0;
}

static int mxt_read_info_block(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	size_t size;
	void *buf;
	struct mxt_info *info;
	u32 calculated_crc;
	u8 *crc_ptr;

	TOUCH_INFO_MSG("%s\n", __func__);

	/* If info block already allocated, free it */
	if (data->raw_info_block != NULL)
		mxt_free_object_table(data);

	/* Read 7-byte ID information block starting at address 0 */
	size = sizeof(struct mxt_info);
	buf = kzalloc(size, GFP_KERNEL);
	if (!buf) {
		TOUCH_INFO_MSG("Failed to allocate memory\n");
		return -ENOMEM;
	}

	error = __mxt_read_reg(client, 0, size, buf);
	if (error) {
		goto err_free_mem;
	}

	/* Resize buffer to give space for rest of info block */
	info = (struct mxt_info *)buf;
	size += (info->object_num * sizeof(struct mxt_object))
		+ MXT_INFO_CHECKSUM_SIZE;

	buf = krealloc(buf, size, GFP_KERNEL);
	if (!buf) {
		TOUCH_INFO_MSG("Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	/* Read rest of info block */
	error = __mxt_read_reg(client, MXT_OBJECT_START,
			       size - MXT_OBJECT_START,
			       buf + MXT_OBJECT_START);
	if (error)
		goto err_free_mem;

	/* Extract & calculate checksum */
	crc_ptr = buf + size - MXT_INFO_CHECKSUM_SIZE;
	data->info_crc = crc_ptr[0] | (crc_ptr[1] << 8) | (crc_ptr[2] << 16);

	calculated_crc = mxt_calculate_crc(buf, 0,
					   size - MXT_INFO_CHECKSUM_SIZE);

	/* CRC mismatch can be caused by data corruption due to I2C comms
	 * issue or else device is not using Object Based Protocol */
	if (data->info_crc != calculated_crc) {
		TOUCH_INFO_MSG("Info Block CRC error calculated=0x%06X read=0x%06X\n",
			data->info_crc, calculated_crc);
		return -EIO;
	}

	/* Save pointers in device data structure */
	data->raw_info_block = buf;
	data->info = (struct mxt_info *)buf;
	data->object_table = (struct mxt_object *)(buf + MXT_OBJECT_START);

	TOUCH_INFO_MSG("Family: %02X Variant: %02X Firmware V%u.%u.%02X Objects: %u\n",
		 data->info->family_id, data->info->variant_id, data->info->version >> 4,
		 data->info->version & 0xf, data->info->build, data->info->object_num);

	/* Parse object table information */
	error = mxt_parse_object_table(data);
	if (error) {
		TOUCH_INFO_MSG("Error %d reading object table\n", error);
		mxt_free_object_table(data);
		return error;
	}

	return 0;

err_free_mem:
	kfree(buf);
	data->raw_info_block = NULL;
	data->info = NULL;
	data->object_table = NULL;
	return error;
}

static int mxt_read_t9_resolution(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct t9_range range;
	unsigned char orient = 0;
	struct mxt_object *object;
	memset(&range, 0, sizeof(range));

	object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T9_RANGE,
			       sizeof(range), &range);
	if (error)
		return error;

	le16_to_cpus(range.x);
	le16_to_cpus(range.y);

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T9_ORIENT,
				1, &orient);
	if (error)
		return error;

	/* Handle default values */
	if (range.x == 0)
		range.x = 2159;

	if (range.y == 0)
		range.y = 3839;

	if (orient & MXT_T9_ORIENT_SWITCH) {
		data->max_x = range.y;
		data->max_y = range.x;
	} else {
		data->max_x = range.x;
		data->max_y = range.y;
	}

	TOUCH_INFO_MSG("Touchscreen size X:%u Y:%u\n", data->max_x, data->max_y);

	return 0;
}

static void mxt_regulator_enable(struct mxt_data *data)
{
	int error = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	gpio_set_value(data->pdata->gpio_reset, 0);

	error = regulator_enable(data->vdd_ana);
	if (error < 0) {
		TOUCH_INFO_MSG("vdd_ana regulator enable fail\n");
		return;
	}

	msleep(MXT_REGULATOR_DELAY);

	INIT_COMPLETION(data->bl_completion);
	gpio_set_value(data->pdata->gpio_reset, 1);
#ifdef CUST_B_TOUCH
	msleep(MXT_POWERON_DELAY);
#else
	mxt_wait_for_completion(data, &data->bl_completion, MXT_POWERON_DELAY);
#endif
}

static void mxt_regulator_disable(struct mxt_data *data)
{
	int error = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	gpio_set_value(data->pdata->gpio_reset, 0);

	error = regulator_disable(data->vdd_ana);
	if (error < 0) {
		TOUCH_INFO_MSG("vdd_ana regulator disable fail\n");
		return;
	}
}

static void mxt_probe_regulators(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error;

	/* According to maXTouch power sequencing specification, RESET line
	 * must be kept low until some time after regulators come up to
	 * voltage */

	TOUCH_INFO_MSG("%s \n", __func__);

	if (!data->pdata->gpio_reset) {
		TOUCH_INFO_MSG("Must have reset GPIO to use regulator support\n");
		goto fail;
	}

	data->vdd_ana = regulator_get(dev, "vdd_ana");
	if (IS_ERR(data->vdd_ana)) {
		error = PTR_ERR(data->vdd_ana);
		TOUCH_INFO_MSG("Error %d getting ana regulator\n", error);
		goto fail;
	}

	error = regulator_set_voltage(data->vdd_ana, 2950000, 2950000);
	if (error < 0) {
		TOUCH_INFO_MSG("Error %d cannot control ana regulator\n", error);
		goto fail;
	}

	data->use_regulator = true;
	mxt_regulator_enable(data);
	return;
/*
fail_release2:
	regulator_put(data->vcc_i2c);
fail_release:
	regulator_put(data->vdd_ana);*/
fail:
	TOUCH_INFO_MSG("Error fail\n");
	data->vdd_ana = NULL;
	data->use_regulator = false;
}

static int mxt_read_t100_config(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;
	u16 range_x = 0, range_y = 0;
	u8 cfg = 0, tchaux = 0;
	u8 aux = 0;

	object = mxt_get_object(data, MXT_TOUCH_MULTITOUCHSCREEN_T100);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T100_XRANGE,
			       sizeof(range_x), &range_x);
	if (error)
		return error;

	le16_to_cpus(range_x);

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T100_YRANGE,
			       sizeof(range_y), &range_y);
	if (error)
		return error;

	le16_to_cpus(range_y);

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_CFG1,
				1, &cfg);
	if (error)
		return error;

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_TCHAUX,
				1, &tchaux);
	if (error)
		return error;

	/* Handle default values */
	if (range_x == 0)
		range_x = 2159;

	/* Handle default values */
	if (range_x == 0)
		range_x = 2159;

	if (range_y == 0)
		range_y = 3839;

	if (cfg & MXT_T100_CFG_SWITCHXY) {
		data->max_x = range_y;
		data->max_y = range_x;
	} else {
		data->max_x = range_x;
		data->max_y = range_y;
	}

	/* allocate aux bytes */
	aux = 6;

	if (tchaux & MXT_T100_TCHAUX_VECT)
		data->t100_aux_vect = aux++;

	if (tchaux & MXT_T100_TCHAUX_AMPL)
		data->t100_aux_ampl = aux++;

	if (tchaux & MXT_T100_TCHAUX_AREA)
		data->t100_aux_area = aux++;

	TOUCH_INFO_MSG("T100 Touchscreen size X%u Y%u amp%u area%u vec%u\n",
	 		data->max_x, data->max_y, data->t100_aux_ampl,
	 		data->t100_aux_area, data->t100_aux_vect);

	return 0;
}

static int mxt_input_open(struct input_dev *dev);
static void mxt_input_close(struct input_dev *dev);

static int mxt_initialize_t100_input_device(struct mxt_data *data)
{
	//struct device *dev = &data->client->dev;
	struct input_dev *input_dev;
	int error;
	unsigned int num_mt_slots;

	error = mxt_read_t100_config(data);
	if (error)
		TOUCH_INFO_MSG("Failed to initialize T100 configuration\n");

	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		TOUCH_INFO_MSG("Failed to allocate memory\n");
		return -ENOMEM;
	}

	input_dev->name = "touch_dev"; /*must sync to idc file name*/
	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	/* input_dev->dev.parent = &data->client->dev;	remove this line for sysfs path (virtual) */
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

	/* For multi touch */
	num_mt_slots = data->num_touchids;
	error = input_mt_init_slots(input_dev, num_mt_slots);
	if (error) {
		TOUCH_INFO_MSG("Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,
			     0, data->pdata->numtouch, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MINOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			     0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
			     0, 255, 0, 0);

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		TOUCH_INFO_MSG("Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static int mxt_initialize_t9_input_device(struct mxt_data *data);
static int mxt_configure_objects(struct mxt_data *data);

static u16 mxt_parse_active_address(struct mxt_data *data, int num)
{
	u16 address = 0;

	switch(num)
	{
		case T7_GESTURE_MODE:
		case T7_GESTURE_MASK:
			address = data->T7_address;
			break;
		case T8_GESTURE_MODE:
		case T8_GESTURE_MASK:
			address = data->T8_address;
			break;
		case T9_GESTURE_MODE:
		case T9_GESTURE_MASK:
			address = data->T9_address;
			break;
		case T15_GESTURE_MODE:
		case T15_GESTURE_MASK:
			address = data->T15_address;
			break;
		case T24_GESTURE_MODE:
		case T24_GESTURE_MASK:
			address = data->T24_address;
			break;
		case T42_GESTURE_MODE:
		case T42_GESTURE_MASK:
			address = data->T42_address;
			break;
		case T46_GESTURE_MODE:
		case T46_GESTURE_MASK:
			address = data->T46_address;
			break;
		case T56_GESTURE_MODE:
		case T56_GESTURE_MASK:
			address = data->T56_address;
			break;
		case T65_GESTURE_MODE:
		case T65_GESTURE_MASK:
			address = data->T65_address;
			break;
		case T72_GESTURE_MODE:
		case T72_GESTURE_MASK:
			address = data->T72_address;
			break;
		default:
			TOUCH_INFO_MSG("%s Error, idx=%d ",__func__, num);
			return -1;
	}

	TOUCH_INFO_MSG("%s idx=%d addr = %u",__func__,num, address);

	return address;
}

static int mxt_parse_active_value(struct mxt_data *data)
{
	struct device *dev;
	u8 *temp_cfg;
	u8 *ic_val;
	u16 t_address;
	char *mode = "mode";
	char *mask = "mask";
	int i=0,j=0;
	int err = 0;
	struct gestture_config_info* g_info;

	dev = &data->client->dev;
	g_info = data->pdata->t_gesture;

	ic_val = kmalloc(sizeof(u8), GFP_KERNEL);
	if (!ic_val)
		return -ENOMEM;

	for(i=0;i<GESTURE_MAX;i++) {
			temp_cfg = devm_kzalloc(dev, sizeof(u8) * T_BYTE_MAX, GFP_KERNEL);
			if (!temp_cfg) {
				TOUCH_INFO_MSG("Unable to allocate memory\n");
				return -ENOMEM;
			}
			memset(temp_cfg, 0x00, sizeof(u8) * T_BYTE_MAX);
			t_address = mxt_parse_active_address(data, i);
			j = 0;
			if(strstr(gesture_node_name[i], mode) != NULL) {
				while(1) 	{
					if(*((g_info->T_value[i+1])+j) == 0x0) {
							break;
					}
					if(*((g_info->T_value[i+1])+j) == 0xFF) {
						err = __mxt_read_reg(data->client,t_address+j, 1, ic_val);
						if(err < 0) {
							TOUCH_INFO_MSG("%s i2c err : %d ", __func__, err);
							break;
						}
						*((temp_cfg)+j) = *ic_val;
						//TOUCH_INFO_MSG("%s j = %d temp_cfg = %u ",__func__, j,*((temp_cfg)+j));
					}
					j++;
				}

				g_info->T_active_value[i] = temp_cfg;
			}
			else if(strstr(gesture_node_name[i], mask) != NULL) {
				//TOUCH_INFO_MSG("%s mask ENTER ",__func__);
				g_info->T_active_value[i] = g_info->T_value[i];
			}
			else {
				TOUCH_INFO_MSG("%s ERROR Unable to read mode = %d \n", __func__, i);
			}
	}
	return 0;
}

static int mxt_initialize(struct mxt_data *data)
{
	int error;
	u8 retry_count = 0;
	TOUCH_INFO_MSG("%s \n", __func__);

retry_probe:
	error = mxt_read_info_block(data);
	mxt_parse_active_value(data);
	if (error) {
		error = mxt_probe_bootloader(data, retry_count);
		if (error) {
			if (++retry_count > 11)
				/* Chip is not in appmode or bootloader mode */
				return error;

			goto retry_probe;
		} else {
			if (++retry_count > 10) {
				TOUCH_INFO_MSG("Could not recover device from "
						"bootloader mode\n");
				/* this is not an error state, we can reflash
				 * from here */
				data->in_bootloader = true;
				return 0;
			}

			/* Attempt to exit bootloader into app mode */
			mxt_send_bootloader_cmd(data, false);
			msleep(MXT_FW_RESET_TIME);
			goto retry_probe;
		}
	}

	error = mxt_check_retrigen(data);
	if (error)
		return error;

	return 0;
}

static int mxt_rest_init(struct mxt_data *data)
{
	int error;

	error = mxt_acquire_irq(data);
	if (error)
		return error;

	error = mxt_configure_objects(data);
	if (error)
		return error;

	return 0;
}

static int mxt_configure_objects(struct mxt_data *data)
{
	int error = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	error = mxt_init_t7_power_cfg(data);
	if (error) {
		TOUCH_INFO_MSG("Failed to initialize power cfg\n");
		return error;
	}

	/* Check register init values */
	error = mxt_check_reg_init(data, data->pdata->cfg_name);
	if (error) {
		TOUCH_INFO_MSG("Error %d initialising configuration\n", error);
		return error;
	}

	if (data->T9_reportid_min) {
		error = mxt_initialize_t9_input_device(data);
		if (error)
			return error;
	} else if (data->T100_reportid_min) {
		error = mxt_initialize_t100_input_device(data);
		if (error)
			return error;
	} else {
		TOUCH_INFO_MSG("No touch object detected\n");
	}
	return 0;
}

/* Firmware Version is returned as Major.Minor.Build */
static ssize_t mxt_fw_version_show(struct mxt_data *data, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u.%u.%02X\n",
			 data->info->version >> 4, data->info->version & 0xf,
			 data->info->build);
}

/* Hardware Version is returned as FamilyID.VariantID */
static ssize_t mxt_hw_version_show(struct mxt_data *data, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u.%u\n",
			data->info->family_id, data->info->variant_id);
}

/* Configuration Checksum */
static ssize_t mxt_mxt_info_show(struct mxt_data *data, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "\n====== Touch IC Info ======\n");
	ret += sprintf(buf+ret, "FW version           = %u.%u.%02X\n",
					 data->info->version >> 4, data->info->version & 0xf,
					 data->info->build);
	ret += sprintf(buf+ret, "config checksum      = 0x%06X\n", data->config_crc);
	ret += sprintf(buf+ret, "Object Num           = %d\n", data->info->object_num);
	ret += sprintf(buf+ret, "Faily Id             = %d\n", data->info->family_id);
	ret += sprintf(buf+ret, "Variant              = %d\n", data->info->variant_id);
	ret += sprintf(buf+ret, "Version              = %d\n", data->info->version);
	ret += sprintf(buf+ret, "Build                = %d\n", data->info->build);

	return ret;
}

static ssize_t mxt_selftest_store(struct mxt_data *data, const char *buf, size_t count)
{
	int command = 0;
	int ret = 0;

	ret = sscanf(buf, "%u", &command);
	TOUCH_INFO_MSG("\n");

	selftest_enable = true;
	mxt_t25_command(data, command, false);

	return count;
}

static ssize_t mxt_selftest_show(struct mxt_data *data, char *buf)
{
	int ret = 0;
	int test_all_cmd = 254;

	selftest_enable = true;
	selftest_show = true;

	mxt_t25_command(data, test_all_cmd, false);
	msleep(MXT_SELFTEST_TIME);

	ret = sprintf(buf, "====== MXT Self Test Info ======\n");
	if(data->self_test_status[0] == 0){
		ret += sprintf(buf+ret, "Need more time. Try Again.\n");
		return ret;
	}

	if(data->self_test_status[0] == 0xFD){
		ret += sprintf(buf+ret, "Invalid Test Code. Try Again.");
	}else if(data->self_test_status[0] == 0xFC){
		ret += sprintf(buf+ret, "The test could not be completed due to an unrelated fault. Try again.");
	}else{
		ret += sprintf(buf+ret, "All Test Result: %s", (data->self_test_status[0] == 0xFE) ? "Pass\n" : "Fail\n");
		ret += sprintf(buf+ret, "AVdd power Test Result: %s", (data->self_test_status[0] != 0x01) ? "Pass\n" : "Fail\n");

		ret += sprintf(buf+ret, "Pin Falut Test Result: %s", (data->self_test_status[0] != 0x12) ? "Pass\n" : "Fail\n");
		if(data->self_test_status[0] == 0x12)
			ret += sprintf(buf+ret, "# Fail # seq_num(%u) x_pin(%u) y_pin(%u)",
										data->self_test_status[1], data->self_test_status[2], data->self_test_status[3]);

		ret += sprintf(buf+ret, "Signal Limit Test: %s", (data->self_test_status[0] != 0x17) ? "Pass\n" : "Fail\n");
		if(data->self_test_status[0] == 0x17)
			ret += sprintf(buf+ret, "# Fail # type_num(%u) type_instance(%u)", data->self_test_status[1], data->self_test_status[2]);
	}

	memset(&data->self_test_status, 0, sizeof(data->self_test_status));
	selftest_show = false;
	return ret;
}

static ssize_t mxt_show_instance(char *buf, int count,
				 struct mxt_object *object, int instance,
				 const u8 *val)
{
	int i;

	if (mxt_obj_instances(object) > 1)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "Instance %u\n", instance);

	for (i = 0; i < mxt_obj_size(object); i++)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"\t[%2u]: %02x (%d)\n", i, val[i], val[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t mxt_object_show(struct mxt_data *data, char *buf)
{
	struct mxt_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 *obuf;

	/* Pre-allocate buffer large enough to hold max sized object. */
	obuf = kmalloc(256, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	error = 0;
	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_readable(object->type))
			continue;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				"T%u:\n", object->type);

		for (j = 0; j < mxt_obj_instances(object); j++) {
			u16 size = mxt_obj_size(object);
			u16 addr = object->start_address + j * size;

			error = __mxt_read_reg(data->client, addr, size, obuf);
			if (error)
				goto done;

			count = mxt_show_instance(buf, count, object, j, obuf);
		}
	}

done:
	kfree(obuf);
	return error ?: count;
}

/* mxt_object_control
 * Usage
 * - read : echo read object 0 0 > object_ctrl
 * - wirte : echo write object address_offset value > object_ctrl
 */
static ssize_t mxt_object_control(struct mxt_data *data, const char *buf, size_t count)
{
	struct mxt_object *object;
	unsigned char command[6];
	int type = 0;
	int addr_offset = 0;
	int value = 0;
	int error = 0;
	int i = 0,j = 0;
	u8 *obuf;

	sscanf(buf, "%s %d %d %d", command, &type, &addr_offset, &value);

	obuf = kmalloc(256, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	if(type == 25)
		selftest_enable = true;

	object = mxt_get_object(data, type);
	if (!object) {
        TOUCH_INFO_MSG("error Cannot get object_type T%d\n", type);
        return -EINVAL;
    }

	if ((mxt_obj_size(object) == 0) || (object->start_address == 0)) {
		TOUCH_INFO_MSG("error object_type T%d\n", type);
		return -ENODEV;
	}

	if (!strncmp(command, "read", 4)){	/*read*/
		TOUCH_INFO_MSG("Object Read T%d: start_addr=%d, size=%d * instance=%d\n",
		type, object->start_address, mxt_obj_size(object), mxt_obj_instances(object));

		for (j = 0; j < mxt_obj_instances(object); j++) {
			u16 size = mxt_obj_size(object);
			u16 addr = object->start_address + j * size;

			error = __mxt_read_reg(data->client, addr, size, obuf);
			if (error)
				TOUCH_INFO_MSG("Object Read Fail\n");
		}

		for (i = 0; i < mxt_obj_size(object)*mxt_obj_instances(object); i++)
			TOUCH_INFO_MSG("T%d [%d] %d[0x%x]\n", type, i, obuf[i], obuf[i]);

	}else if (!strncmp(command, "write", 4)){	/*write*/
		TOUCH_INFO_MSG("Object Write T%d: start_addr=%d, size=%d * instance=%d\n",
		type, object->start_address, mxt_obj_size(object), mxt_obj_instances(object));

		error = mxt_write_reg(data->client, object->start_address+addr_offset, value);
		if (error)
			TOUCH_INFO_MSG("Object Write Fail\n");

		TOUCH_INFO_MSG("Object Write Success. Execute Read Object and Check Value.\n");
	}else{
		TOUCH_INFO_MSG("Command Fail. Usage: echo [read | write] object cmd_field value > object_ctrl\n");
	}
	return count;
}
static int mxt_check_firmware_format(struct device *dev,
				     const struct firmware *fw)
{
	unsigned int pos = 0;
	char c;

	while (pos < fw->size) {
		c = *(fw->data + pos);

		if (c < '0' || (c > '9' && c < 'A') || c > 'F')
			return 0;

		pos++;
	}

	/* To convert file try
	 * xxd -r -p mXTXXX__APP_VX-X-XX.enc > maxtouch.fw */
	TOUCH_INFO_MSG("Aborting: firmware file must be in binary format\n");

	return -1;
}

static int mxt_load_fw(struct device *dev, const char *name)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	char *fw_name = NULL;
	unsigned int frame_size = 0;
	unsigned int pos = 0;
	unsigned int retry = 0;
	unsigned int frame = 0;
	int ret = 0;

	if (name)
		fw_name = (char *)name;
	else
		fw_name = (char *)data->fw_name;
	
	ret = request_firmware(&fw, fw_name, dev);
	if (ret) {
		TOUCH_INFO_MSG("Unable to open firmware [%s]  ret %d\n",fw_name, ret);
		return 1;
	}
	else {
		TOUCH_INFO_MSG("Open firmware [%s]\n", fw_name);
	}

	/* Check for incorrect enc file */
	ret = mxt_check_firmware_format(dev, fw);
	if (ret)
		goto release_firmware;

	if (data->suspended) {
		if (data->use_regulator)
			mxt_regulator_enable(data);

		touch_enable_irq(data->irq);
		data->suspended = false;
	}

	if (!data->in_bootloader) {
		/* Change to the bootloader mode */
		data->in_bootloader = true;

		ret = mxt_t6_command(data, MXT_COMMAND_RESET,
				     MXT_BOOT_VALUE, false);
		if (ret)
			goto release_firmware;

		msleep(MXT_RESET_TIME);

		/* At this stage, do not need to scan since we know
		 * family ID */
		ret = mxt_lookup_bootloader_address(data, 0);
		if (ret)
			goto release_firmware;
	}

	mxt_free_object_table(data);
	INIT_COMPLETION(data->bl_completion);

	ret = mxt_check_bootloader(data, MXT_WAITING_BOOTLOAD_CMD);
	if (ret) {
		/* Bootloader may still be unlocked from previous update
		 * attempt */
		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA);
		if (ret)
			goto disable_irq;
	} else {
		TOUCH_INFO_MSG("Unlocking bootloader\n");

		/* Unlock bootloader */
		ret = mxt_send_bootloader_cmd(data, true);
		if (ret)
			goto disable_irq;
	}

	while (pos < fw->size) {
		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA);
		if (ret)
			goto disable_irq;

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* Take account of CRC bytes */
		frame_size += 2;

		/* Write one frame to device */
		ret = mxt_bootloader_write(data, fw->data + pos, frame_size);
		if (ret)
			goto disable_irq;

		ret = mxt_check_bootloader(data, MXT_FRAME_CRC_PASS);
		if (ret) {
			retry++;

			/* Back off by 20ms per retry */
			msleep(retry * 20);

			if (retry > 20) {
				TOUCH_INFO_MSG("Retry count exceeded\n");
				goto disable_irq;
			}
		} else {
			retry = 0;
			pos += frame_size;
			frame++;
		}

		if (frame % 50 == 0)
			TOUCH_INFO_MSG("Sent %d frames, %d/%zd bytes\n",
				 frame, pos, fw->size);
	}

	/* Wait for flash */
	ret = mxt_wait_for_completion(data, &data->bl_completion,
				      MXT_FW_RESET_TIME);
	if (ret)
		goto disable_irq;
	TOUCH_INFO_MSG("Sent %d frames, %zd bytes\n", frame, pos);

#if 0	/*To avoid reset timeout*/
	/* Wait for device to reset */
	mxt_wait_for_completion(data, &data->bl_completion, MXT_RESET_TIMEOUT);
#endif
	data->in_bootloader = false;

disable_irq:
	touch_disable_irq(data->irq);
release_firmware:
	release_firmware(fw);
	return ret;
}

static int mxt_update_file_name(struct device *dev, char **file_name,
				const char *buf, size_t count)
{
	char *file_name_tmp;

	/* Simple sanity check */
	if (count > 128) {
		TOUCH_INFO_MSG("File name too long %d\n", count);
		return -EINVAL;
	}

	file_name_tmp = krealloc(*file_name, count + 1, GFP_KERNEL);
	if (!file_name_tmp) {
		TOUCH_INFO_MSG("no memory\n");
		return -ENOMEM;
	}

	*file_name = file_name_tmp;
	memcpy(*file_name, buf, count);

	/* Echo into the sysfs entry may append newline at the end of buf */
	if (buf[count - 1] == '\n')
		(*file_name)[count - 1] = '\0';
	else
		(*file_name)[count] = '\0';

	return 0;
}

static ssize_t mxt_firmware_update(struct mxt_data *data)
{
    struct device *dev = &data->client->dev;
    int error;

    error = mxt_load_fw(dev, data->pdata->bin_name);
    if (error) {
		TOUCH_INFO_MSG("The firmware update failed(%d)\n", error);
    } else {
		TOUCH_INFO_MSG("The firmware update succeeded\n");
		data->suspended = false;

		mxt_regulator_disable(data);
		msleep(50);
		mxt_regulator_enable(data);

		error = mxt_read_info_block(data);
    }

    return error;
}

static ssize_t mxt_update_fw_store(struct mxt_data *data, const char *buf, size_t count)
{
	int error;

	error = mxt_update_file_name(&data->client->dev, &data->fw_name, buf, count);
	if (error)
		return error;

	error = mxt_load_fw(&data->client->dev, NULL);
	if (error) {
		TOUCH_INFO_MSG("The firmware update failed(%d)\n", error);
		count = error;
	} else {
		TOUCH_INFO_MSG("The firmware update succeeded\n");

		data->suspended = false;

		mxt_regulator_disable(data);
		msleep(50);
		mxt_regulator_enable(data);

		error = mxt_read_info_block(data);
		if (error)
			return error;

		error = mxt_rest_init(data);
		if (error)
			return error;

		TOUCH_INFO_MSG("Need update proper Configuration(RAW) \n");
	}
	return count;
}

static ssize_t mxt_update_cfg_store(struct mxt_data *data, const char *buf, size_t count)
{
	int ret = 0;
	int value = 0;

	sscanf(buf, "%d", &value);
	TOUCH_INFO_MSG("Update mxt Configuration.\n");

	if (data->in_bootloader) {
		TOUCH_INFO_MSG("Not in appmode\n");
		return -EINVAL;
	}

	ret = mxt_update_file_name(&data->client->dev, &data->cfg_name, buf, count);
	if (ret)
		return ret;

	data->enable_reporting = false;

	ret = mxt_check_reg_init(data, NULL);
	if (ret < 0) {
		TOUCH_INFO_MSG("Error mxt_check_reg_init ret=%d\n", ret);
		goto out;
	}

	TOUCH_INFO_MSG("Update mxt Configuration Success.\n");

out:
	data->enable_reporting = true;

	return count;
}

static ssize_t mxt_debug_enable_show(struct mxt_data *data, char *buf)
{
	int count;
	char c;

	c = data->debug_enabled ? '1' : '0';
	count = sprintf(buf, "%c\n", c);

	return count;
}

static ssize_t mxt_debug_enable_store(struct mxt_data *data, const char *buf, size_t count)
{
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->debug_enabled = (i == 1);

		TOUCH_INFO_MSG("%s\n", i ? "debug enabled" : "debug disabled");
		return count;
	} else {
		TOUCH_INFO_MSG("debug_enabled write error\n");
		return -EINVAL;
	}
}

static int mxt_check_mem_access_params(struct mxt_data *data, loff_t off,
				       size_t *count)
{
	data->mem_size = 32768;

	if (off >= data->mem_size)
		return -EIO;

	if (off + *count > data->mem_size)
		*count = data->mem_size - off;

	if (*count > MXT_MAX_BLOCK_WRITE)
		*count = MXT_MAX_BLOCK_WRITE;

	return 0;
}

static ssize_t mxt_mem_access_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = __mxt_read_reg(data->client, off, count, buf);

	return ret == 0 ? count : ret;
}

static ssize_t mxt_mem_access_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = __mxt_write_reg(data->client, off, count, buf);

	return ret == 0 ? count : 0;
}

#ifdef MXT_GESTURE_RECOGNIZE
static ssize_t mxt_knock_on_store(struct mxt_data *data, const char *buf, size_t size)
{
	struct input_dev *input_dev = data->input_dev;
	int value;

	TOUCH_INFO_MSG("%s : %s \n", __func__, buf);

	sscanf(buf, "%d", &value);
	mutex_lock(&input_dev->mutex);

	if (value == data->mxt_knock_on_enable || !data->suspended)
		goto no_action;

	if(value == 1) {
		data->mxt_knock_on_enable = true;
		TOUCH_INFO_MSG("Knock On Enabled\n");
	} else {
		data->mxt_knock_on_enable = false;
		TOUCH_INFO_MSG("Knock On Disabled\n");
	}

no_action:
	data->mxt_knock_on_enable = value;
	TOUCH_INFO_MSG("Knock On : %s\n", data->mxt_knock_on_enable ? "Enabled" : "Disabled");
	mutex_unlock(&input_dev->mutex);
	return size;
}

static ssize_t mxt_character_store(struct mxt_data *data, const char *buf, size_t size)
{
	struct input_dev *input_dev = data->input_dev;
	int value;

	dev_info(&data->client->dev, "%s\n", __func__);

	sscanf(buf, "%d", &value);
	mutex_lock(&input_dev->mutex);

	if(value == data->mxt_character_enable || !data->suspended)
		goto no_action;

	if(value == 1){
		data->mxt_character_enable = true;
		dev_info(&data->client->dev, "Character Recognize Enabled\n");
	}else{
		data->mxt_character_enable = false;
		dev_info(&data->client->dev, "Character Recognize Disabled\n");
	}

no_action:
	data->mxt_character_enable = value;
	dev_info(&data->client->dev, "Character Recognize : %s\n", data->mxt_character_enable ? "Enabled" : "Disabled");
	mutex_unlock(&input_dev->mutex);
	return size;
}

#endif

#ifdef T37_DBG_DATA
static ssize_t mxt_run_delta_show(struct mxt_data *data, char *buf)
{
	int len=0;
	dev_info(&data->client->dev, "%s\n", __func__);

	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");

	run_delta_read(data, buf, &len);

	len += snprintf(buf + len, PAGE_SIZE - len, "\n===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================\n");

	return len;
}

static ssize_t mxt_run_reference_show(struct mxt_data *data, char *buf)
{
	int len=0;
	dev_info(&data->client->dev, "%s\n", __func__);

	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");

	run_reference_read(data, buf, &len);

	len += snprintf(buf + len, PAGE_SIZE - len, "\n===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================\n");

	return len;
}
#endif

static LGE_TOUCH_ATTR(fw_version, S_IRUGO, mxt_fw_version_show, NULL);
static LGE_TOUCH_ATTR(hw_version, S_IRUGO, mxt_hw_version_show, NULL);
static LGE_TOUCH_ATTR(mxt_info, S_IRUGO, mxt_mxt_info_show, NULL);
static LGE_TOUCH_ATTR(self_test, S_IRUGO | S_IWUSR, mxt_selftest_show, mxt_selftest_store);
static LGE_TOUCH_ATTR(object, S_IRUGO, mxt_object_show, NULL);
static LGE_TOUCH_ATTR(object_ctrl, S_IWUSR, NULL, mxt_object_control);
static LGE_TOUCH_ATTR(update_fw, S_IWUSR, NULL, mxt_update_fw_store);
static LGE_TOUCH_ATTR(update_cfg, S_IWUSR, NULL, mxt_update_cfg_store);
static LGE_TOUCH_ATTR(debug_enable, S_IWUSR | S_IRUSR, mxt_debug_enable_show, mxt_debug_enable_store);
#ifdef MXT_GESTURE_RECOGNIZE
static LGE_TOUCH_ATTR(touch_gesture,S_IRUGO | S_IWUSR, NULL, mxt_knock_on_store);
static LGE_TOUCH_ATTR(touch_gesture_character,S_IRUGO | S_IWUSR, NULL, mxt_character_store);
#endif

#ifdef T37_DBG_DATA
static LGE_TOUCH_ATTR(delta, S_IRUGO, mxt_run_delta_show, NULL);
static LGE_TOUCH_ATTR(reference, S_IRUGO, mxt_run_reference_show, NULL);
#endif

static struct attribute *lge_touch_attribute_list[] = {
	&lge_touch_attr_fw_version.attr,
	&lge_touch_attr_hw_version.attr,
	&lge_touch_attr_mxt_info.attr,
	&lge_touch_attr_self_test.attr,
	&lge_touch_attr_object.attr,
	&lge_touch_attr_object_ctrl.attr,
	&lge_touch_attr_update_fw.attr,
	&lge_touch_attr_update_cfg.attr,
	&lge_touch_attr_debug_enable.attr,
#ifdef MXT_GESTURE_RECOGNIZE
	&lge_touch_attr_touch_gesture.attr,
	&lge_touch_attr_touch_gesture_character.attr,
#endif
#ifdef T37_DBG_DATA
	&lge_touch_attr_delta.attr,
	&lge_touch_attr_reference.attr,
#endif
	NULL
};

/* lge_touch_attr_show / lge_touch_attr_store
 *
 * sysfs bindings for lge_touch
 */
static ssize_t lge_touch_attr_show(struct kobject *lge_touch_kobj, struct attribute *attr,
			     char *buf)
{
	struct mxt_data *ts =
			container_of(lge_touch_kobj, struct mxt_data, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv =
		container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->show)
		ret = lge_touch_priv->show(ts, buf);

	return ret;
}

static ssize_t lge_touch_attr_store(struct kobject *lge_touch_kobj, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct mxt_data *ts =
			container_of(lge_touch_kobj, struct mxt_data, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv =
		container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->store)
		ret = lge_touch_priv->store(ts, buf, count);

	return ret;
}

static const struct sysfs_ops lge_touch_sysfs_ops = {
	.show	= lge_touch_attr_show,
	.store	= lge_touch_attr_store,
};

static struct kobj_type lge_touch_kobj_type = {
	.sysfs_ops		= &lge_touch_sysfs_ops,
	.default_attrs 	= lge_touch_attribute_list,
};

static struct sysdev_class lge_touch_sys_class = {
	.name = LGE_TOUCH_NAME,
};

static struct sys_device lge_touch_sys_device = {
	.id		= 0,
	.cls	= &lge_touch_sys_class,
};

static void mxt_reset_slots(struct mxt_data *data)
{
	struct input_dev *input_dev = data->input_dev;
	int id;

	for (id = 0; id < data->pdata->numtouch; id++) {
		input_mt_slot(input_dev, id);
		if (data->ts_data.prev_data[id].tool == MT_TOOL_FINGER) {
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
		} else if (data->ts_data.prev_data[id].tool == MT_TOOL_PALM) {
			input_mt_report_slot_state(input_dev, MT_TOOL_PALM, 0);
		} else {
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
		}
	}

	mxt_input_sync(input_dev);
	TOUCH_INFO_MSG("Release all touch event!\n");
}

#ifdef MXT_GESTURE_RECOGNIZE
/* reduce sleep current */
static void mxt_gesture_mode_start(struct mxt_data *data)
{
	struct mxt_platform_data *pdata = data->pdata;
	int ret = 0;
	int i = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	/* T7 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T7_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T7_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T7_address+i, *((pdata->t_gesture->T_value[T7_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("gesture mode T7 (addr %d | value %u)\n", data->T7_address + i, *((pdata->t_gesture->T_value[T7_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T7 gesture mode config.\n");
			}
		}
	}

	/* T8 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T8_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T8_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T8_address+i, *((pdata->t_gesture->T_value[T8_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("gesture mode T8 (addr %d | value %u)\n", data->T8_address + i, *((pdata->t_gesture->T_value[T8_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T8 gesture mode config.\n");
			}
		}
	}


	/* T9 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T9_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T9_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T9_address+i, *((pdata->t_gesture->T_value[T9_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("gesture mode T9 (addr %d | value %u)\n", data->T9_address + i , *((pdata->t_gesture->T_value[T9_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T9 gesture mode config.\n");
			}
		}
	}

	/* T15 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T15_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T15_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T15_address+i, *((pdata->t_gesture->T_value[T15_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("gesture mode T15 (addr %d | value %u)\n", data->T15_address+i, *((pdata->t_gesture->T_value[T15_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T15 gesture mode config.\n");
			}
		}
	}

	/* T24 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T24_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T24_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T24_address+i, *((pdata->t_gesture->T_value[T24_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("gesture mode T24 (addr %d | value %u)\n", data->T24_address + i, *((pdata->t_gesture->T_value[T24_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T24 gesture mode config.\n");
			}
		}
	}

	/* T42 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T42_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T42_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T42_address+i, *((pdata->t_gesture->T_value[T42_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("gesture mode T42 (addr %d | value %u)\n", data->T42_address + i, *((pdata->t_gesture->T_value[T42_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T42 gesture mode config.\n");
			}
		}
	}

	/* T46 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T46_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T46_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T46_address+i, *((pdata->t_gesture->T_value[T46_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("gesture mode T46 (addr %d | value %u)\n", data->T46_address + i, *((pdata->t_gesture->T_value[T46_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T46 gesture mode config.\n");
			}
		}
	}

	/* T56 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T56_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T56_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T56_address+i, *((pdata->t_gesture->T_value[T56_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("gesture mode T56 (addr %d | value %u)\n", data->T56_address + i, *((pdata->t_gesture->T_value[T56_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T56 gesture mode config.\n");
			}
		}
	}

	/* T65 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T65_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T65_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T65_address+i, *((pdata->t_gesture->T_value[T65_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("gesture mode T65 (addr %d | value %u)\n", data->T65_address + i, *((pdata->t_gesture->T_value[T65_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T65 gesture mode config.\n");
			}
		}
	}

	/* T72 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T72_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T72_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T72_address+i, *((pdata->t_gesture->T_value[T72_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("gesture mode T72 (addr %d | value %u)\n", data->T72_address + i, *((pdata->t_gesture->T_value[T72_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T72 gesture mode config.\n");
			}
		}
	}
}

static void mxt_active_mode_start(struct mxt_data *data)
{
	int ret = 0;
	int i =0;
	struct mxt_platform_data *pdata = data->pdata;

	TOUCH_INFO_MSG("%s \n", __func__);

	/* T7 setting */
	for(i=0; i <T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T7_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T7_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T7_address+i, *((pdata->t_gesture->T_active_value[T7_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("active mode T7 (addr %d | value %d)\n", data->T7_address+i, *((pdata->t_gesture->T_active_value[T7_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T7 gesture mode config.\n");
			}
		}
	}

	/* T8 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T8_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T8_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T8_address+i, *((pdata->t_gesture->T_active_value[T8_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("active mode T8 (addr %d | value %d)\n", data->T8_address+i, *((pdata->t_gesture->T_active_value[T8_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T8 gesture mode config.\n");
			}
		}
	}

	/* T9 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T9_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T9_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T9_address+i, *((pdata->t_gesture->T_active_value[T9_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("active mode T9 (addr %d | value %d)\n", data->T9_address + i, *((pdata->t_gesture->T_active_value[T9_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T9 gesture mode config.\n");
			}
		}
	}

	/* T15 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T15_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T15_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T15_address+i, *((pdata->t_gesture->T_active_value[T15_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("active mode T15 (addr %d | value %d)\n", data->T15_address + i, *((pdata->t_gesture->T_active_value[T15_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T15 gesture mode config.\n");
			}
		}
	}

	/* T24 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T24_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T24_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T24_address+i, *((pdata->t_gesture->T_active_value[T24_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("active mode T24 (addr %d | value %d)\n", data->T24_address + i, *((pdata->t_gesture->T_active_value[T24_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T24 gesture mode config.\n");
			}
		}
	}

	/* T42 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T42_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T42_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T42_address+i, *((pdata->t_gesture->T_active_value[T42_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("active mode T42 (addr %d | value %d)\n", data->T42_address + i, *((pdata->t_gesture->T_active_value[T42_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T42 gesture mode config.\n");
			}
		}
	}

	/* T46 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T46_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T46_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T46_address+i, *((pdata->t_gesture->T_active_value[T46_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("active mode T46 (addr %d | value %d)\n", data->T46_address + i, *((pdata->t_gesture->T_active_value[T46_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T46 gesture mode config.\n");
			}
		}
	}

	/* T56 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T56_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T56_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T56_address+i, *((pdata->t_gesture->T_active_value[T56_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("active mode T56 (addr %d | value %d)\n", data->T56_address + i, *((pdata->t_gesture->T_active_value[T56_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T56 gesture mode config.\n");
			}
		}
	}

	/* T65 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T65_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T65_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T65_address+i, *((pdata->t_gesture->T_active_value[T65_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("active mode T65 (addr %d | value %d)\n", data->T65_address + i, *((pdata->t_gesture->T_active_value[T65_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T65 gesture mode config.\n");
			}
		}
	}

	/* T72 setting */
	for(i=0; i < T_BYTE_MAX; i++) {
		if(*((pdata->t_gesture->T_value[T72_GESTURE_MASK]) + i) == 0x00) {
			break;
		}
		if(*((pdata->t_gesture->T_value[T72_GESTURE_MASK]) + i) == 0xFF) {
			ret = mxt_write_reg(data->client, data->T72_address+i, *((pdata->t_gesture->T_active_value[T72_GESTURE_MODE])+i));
			TOUCH_INFO_MSG("active mode T72 (addr %d | value %d)\n", data->T72_address+i, *((pdata->t_gesture->T_active_value[T72_GESTURE_MODE])+i));
			if (ret) {
				TOUCH_INFO_MSG("write error T72 gesture mode config.\n");
			}
		}
	}
}
#endif

static void mxt_start(struct mxt_data *data)
{
	if (!data->suspended || data->in_bootloader)
		return;

#ifdef MXT_GESTURE_RECOGNIZE
	if(data->mxt_knock_on_enable) {
		touch_disable_irq(data->irq);
		mxt_active_mode_start(data);
		/* Recalibrate since touch doesn't power off when lcd on */
		mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
	} else {
		/* Discard any messages still in message buffer from before
		 * chip went to sleep */
		mxt_process_messages_until_invalid(data);

		TOUCH_INFO_MSG("%s MXT_POWER_CFG_RUN\n", __func__);
		mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);

		/* Recalibrate since chip has been in deep sleep */
		mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
	}
#else
	TOUCH_INFO_MSG("%s MXT_POWER_CFG_RUN\n", __func__);

	/* Discard any messages still in message buffer from before
	 * chip went to sleep */
	mxt_process_messages_until_invalid(data);

	mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);

	/* Recalibrate since chip has been in deep sleep */
	mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
#endif

	data->enable_reporting = true;
	data->suspended = false;
	touch_enable_irq(data->irq);
}

static void mxt_stop(struct mxt_data *data)
{
	if (data->suspended || data->in_bootloader)
		return;

	data->enable_reporting = false;
	touch_disable_irq(data->irq);

#ifdef MXT_GESTURE_RECOGNIZE
	if (data->mxt_knock_on_enable) {
		mxt_gesture_mode_start(data);
		/* Recalibrate since touch doesn't power off when lcd off */
		mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
	} else {
		TOUCH_INFO_MSG("%s MXT_POWER_CFG_DEEPSLEEP\n", __func__);
		mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);
	}
#else
	mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);
#endif

	mxt_reset_slots(data);
	data->suspended = true;
#ifdef MXT_GESTURE_RECOGNIZE
	if (data->mxt_knock_on_enable || data->mxt_character_enable) {
		touch_enable_irq(data->irq);
	}
#endif

}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	mxt_start(data);
	return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	mxt_stop(data);
}

static int mxt_parse_dt(struct device *dev, struct mxt_platform_data *pdata)
{
	struct device_node *node = dev->of_node;
	struct property *prop;
	struct property *g_prop[GESTURE_MAX];
	u8 *temp_cfg;
	u32 temp_array[MXT_MAX_NUM_KEY] = {0};
	int rc = 0;
	int i = 0;
	u32 temp_val;

	TOUCH_INFO_MSG("%s\n", __func__);

	/* reset, irq gpio info */
	if (node == NULL)
		return -ENODEV;

	pdata->gpio_reset= of_get_named_gpio_flags(node, "atmel,reset-gpio", 0, NULL);
	if(pdata->gpio_reset)
		TOUCH_INFO_MSG("DT : gpio_reset = %lu\n", pdata->gpio_reset);
	else
		TOUCH_INFO_MSG("DT get gpio_reset error \n");

	pdata->gpio_int = of_get_named_gpio_flags(node, "atmel,irq-gpio", 0, NULL);
	if(pdata->gpio_int)
		TOUCH_INFO_MSG("DT : gpio_int = %lu\n", pdata->gpio_int);
	else
		TOUCH_INFO_MSG("DT get gpio_int error \n");

	rc = of_property_read_u32(node, "atmel,numtouch", &temp_val);
	if(rc){
		TOUCH_INFO_MSG("Unable to read numtouch\n");
	}else{
		pdata->numtouch = temp_val;
		TOUCH_INFO_MSG("DT : numtouch = %d\n", pdata->numtouch);
	}

	/*
	rc = of_property_read_u32(node, "atmel,max_x", &temp_val);
	if(rc){
		TOUCH_INFO_MSG("Unable to read max_x\n");
		return rc;
	}else{
		pdata->max_x = temp_val;
		TOUCH_INFO_MSG("DT : max_x = %d\n", pdata->max_x);
	}

	rc = of_property_read_u32(node, "atmel,max_y", &temp_val);
	if(rc){
		TOUCH_INFO_MSG("Unable to read max_y\n");
		return rc;
	}else{
		pdata->max_y = temp_val;
		TOUCH_INFO_MSG("DT : max_y = %d\n", pdata->max_y);
	}
	*/

	pdata->t_gesture = devm_kzalloc(dev,
				sizeof(struct gestture_config_info)*GESTURE_MAX, GFP_KERNEL);

	for (i = 0; i < GESTURE_MAX; i++) {
		g_prop[i] = of_find_property(node, gesture_node_name[i], &temp_val);

		if (g_prop[i]) {
			temp_cfg = devm_kzalloc(dev, sizeof(u8) * T_BYTE_MAX, GFP_KERNEL);
			if (!temp_cfg) {
				TOUCH_INFO_MSG("Unable to allocate memory\n");
				return -ENOMEM;
			}
			memset(temp_cfg, 0x00, sizeof(u8) * T_BYTE_MAX);
			memcpy(temp_cfg, g_prop[i]->value, temp_val);
			pdata->t_gesture->T_value[i] = temp_cfg;
		} else {
			TOUCH_INFO_MSG("DT : Unable to read gesture_mode node Num = %d \n", i);
		}
	}

	rc = of_property_read_u32(node, "atmel,auto_fw_update", &temp_val);
	if(rc){
		TOUCH_INFO_MSG("Unable to read auto_fw_update\n");
	}else{
		pdata->auto_fw_update = temp_val;
		TOUCH_INFO_MSG("DT : auto_fw_update = %d\n", pdata->auto_fw_update);
	}

	prop = of_find_property(node, "atmel,bin_ver", NULL);
	if (prop) {
		temp_val = prop->length / sizeof(temp_val);
		if (temp_val == 2) {
			rc = of_property_read_u32_array(node, "atmel,bin_ver", temp_array, temp_val);
			if (rc) {
				TOUCH_INFO_MSG("DT : Unable to read bin_ver\n");
			}

			for(i=0; i<temp_val; i++) {
				pdata->fw_ver[i] = (unsigned char)temp_array[i];
			}

			TOUCH_INFO_MSG("DT : bin_ver = %02X %02X \n", pdata->fw_ver[0], pdata->fw_ver[1]);
		}
	}

	rc = of_property_read_string(node, "atmel,bin_file",  &pdata->bin_name);
	if (rc && (rc != -EINVAL)) {
		TOUCH_INFO_MSG("DT : atmel,bin_file error \n");
		pdata->bin_name = NULL;
	}
	else
		TOUCH_INFO_MSG("DT : bin_file : %s \n", pdata->bin_name);

	rc = of_property_read_string(node, "atmel,raw_file",  &pdata->cfg_name);
	if (rc && (rc != -EINVAL)) {
		TOUCH_INFO_MSG("DT : atmel,raw_file error \n");
		pdata->cfg_name = NULL;
	}
	else
		TOUCH_INFO_MSG("DT : raw_file : %s \n", pdata->cfg_name);

	rc = of_property_read_u32(node, "atmel,panel_check", &temp_val);
	if(rc){
		pdata->panel_check = 0;
	}else{
		pdata->panel_check = temp_val;
		TOUCH_INFO_MSG("DT : panel_check = %d\n", pdata->panel_check);
	}

	if (pdata->panel_check) {
		rc = of_property_read_string(node, "atmel,extra_raw_file",  &pdata->extra_cfg_name);
		if (rc && (rc != -EINVAL)) {
			TOUCH_INFO_MSG("DT : atmel,extra_raw_file error \n");
			pdata->extra_cfg_name = NULL;
		}
		else
			TOUCH_INFO_MSG("DT : extra_raw_file : %s \n", pdata->extra_cfg_name);
	} else {
		pdata->extra_cfg_name = NULL;
	}

	rc = of_property_read_u32(node, "atmel,irqflags", &temp_val);
	if(rc){
		TOUCH_INFO_MSG("Unable to read irqflags\n");
		return rc;
	}else{
		pdata->irqflags = temp_val;
		TOUCH_INFO_MSG("DT : irqflags = %lu\n", pdata->irqflags);
	}

	prop = of_find_property(node, "atmel,t15_key_state", NULL);
	if (prop) {
		temp_val = prop->length / sizeof(temp_val);
		pdata->t15_num_keys = temp_val;
		TOUCH_INFO_MSG("DT : num_buttons = %d\n", pdata->t15_num_keys);

		if (temp_val <= MXT_MAX_NUM_KEY) {
			rc = of_property_read_u32_array(node, "atmel,t15_key_state", temp_array, temp_val);
			if (rc) {
				TOUCH_INFO_MSG("DT : Unable to read t15_key_state\n");
			}

			for(i=0; i<temp_val; i++) {
				pdata->t15_keystate[i] = temp_array[i];
				TOUCH_INFO_MSG("DT : t15_keystate[%d] = [%3d] \n", i, pdata->t15_keystate[i]);
			}
		}
	}

	if (pdata->panel_check) {
		prop = of_find_property(node, "atmel,t15_extra_key_state", NULL);
		if (prop) {
			temp_val = prop->length / sizeof(temp_val);
			if (temp_val <= MXT_MAX_NUM_KEY) {
				rc = of_property_read_u32_array(node, "atmel,t15_extra_key_state", temp_array, temp_val);
				if (rc) {
					TOUCH_INFO_MSG("DT : Unable to read t15_extra_key_state\n");
					pdata->t15_extra_keystate[0] = 0;
				} else {
					for(i=0; i<temp_val; i++) {
						pdata->t15_extra_keystate[i] = temp_array[i];
						TOUCH_INFO_MSG("DT : t15_extra_keystate[%d] = [%3d] \n", i, pdata->t15_extra_keystate[i]);
					}
				}
			}
		} else {
			pdata->t15_extra_keystate[0] = 0;
		}
	}

	prop = of_find_property(node, "atmel,t15_key_map", NULL);
	if (prop) {
		temp_val = prop->length / sizeof(temp_val);
		if (pdata->t15_num_keys != temp_val) {
			TOUCH_INFO_MSG("DT : error t15_key_map == %d\n", temp_val);
		}

		if (temp_val <= MXT_MAX_NUM_KEY) {
			rc = of_property_read_u32_array(node, "atmel,t15_key_map", temp_array, temp_val);
			if (rc) {
				TOUCH_INFO_MSG("DT : Unable to read key codes\n");
				return rc;
			}

			for(i=0; i<temp_val; i++) {
				pdata->t15_keymap[i] = temp_array[i];
				TOUCH_INFO_MSG("DT : button[%d] = [%3d:%s] \n", i, pdata->t15_keystate[i], get_touch_button_string(pdata->t15_keymap[i]));
			}
		}
	}

	return 0;

}

static int mxt_handle_pdata(struct mxt_data *data)
{
	data->pdata = dev_get_platdata(&data->client->dev);

	/* Use provided platform data if present */
	if (data->pdata) {
		if (data->pdata->cfg_name)
			mxt_update_file_name(&data->client->dev,
					     &data->cfg_name,
					     data->pdata->cfg_name,
					     strlen(data->pdata->cfg_name));

		return 0;
	}

	data->pdata = kzalloc(sizeof(*data->pdata), GFP_KERNEL);
	if (!data->pdata) {
		TOUCH_INFO_MSG("Failed to allocate pdata\n");
		return -ENOMEM;
	}

	/* Set default parameters */
	data->pdata->irqflags = IRQF_TRIGGER_FALLING;

	return 0;
}

static int mxt_initialize_t9_input_device(struct mxt_data *data)
{
	struct mxt_platform_data *pdata = data->pdata;
	struct input_dev *input_dev;
	int error;
	unsigned int num_mt_slots;
	int i;

	error = mxt_read_t9_resolution(data);
	if (error)
		TOUCH_INFO_MSG("Failed to initialize T9 resolution\n");

	input_dev = input_allocate_device();
	if (!input_dev) {
		TOUCH_INFO_MSG("Failed to allocate memory\n");
		return -ENOMEM;
	}

	input_dev->name = MXT_DEVICE_NAME;
	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	__set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

	if (pdata->t19_num_keys) {
		__set_bit(INPUT_PROP_BUTTONPAD, input_dev->propbit);

		for (i = 0; i < pdata->t19_num_keys; i++)
			if (pdata->t19_keymap[i] != KEY_RESERVED)
				input_set_capability(input_dev, EV_KEY,
						     pdata->t19_keymap[i]);

		__set_bit(BTN_TOOL_FINGER, input_dev->keybit);
		__set_bit(BTN_TOOL_DOUBLETAP, input_dev->keybit);
		__set_bit(BTN_TOOL_TRIPLETAP, input_dev->keybit);
		__set_bit(BTN_TOOL_QUADTAP, input_dev->keybit);

		input_abs_set_res(input_dev, ABS_X, MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_Y, MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_MT_POSITION_X,
				  MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_MT_POSITION_Y,
				  MXT_PIXELS_PER_MM);

		input_dev->name = MXT_DEVICE_NAME;
	}

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			     0, 255, 0, 0);

	/* For multi touch */
	num_mt_slots = data->num_touchids + data->num_stylusids;
	error = input_mt_init_slots(input_dev, num_mt_slots);
	if (error) {
		TOUCH_INFO_MSG("Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			     0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
			     0, 255, 0, 0);

	/* For T63 active stylus */
	if (data->T63_reportid_min) {
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS);
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS2);
		input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE,
			0, MT_TOOL_MAX, 0, 0);
	}

	/* For T15 key array */
	if (data->T15_reportid_min) {
		data->t15_keystatus = 0;

		input_dev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);

		for (i = 0; i < data->pdata->t15_num_keys; i++)
			input_set_capability(input_dev, EV_KEY, data->pdata->t15_keymap[i]);
			input_dev->keybit[BIT_WORD(data->pdata->t15_keymap[i])] |= BIT_MASK(data->pdata->t15_keymap[i]);
	}

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		TOUCH_INFO_MSG("Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static int __devinit mxt_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct mxt_data *data = NULL;
	int error = 0;
	u8 fw_version = 0, fw_build = 0;
	int i = 0;

	is_probing = true;
	TOUCH_INFO_MSG("%s\n", __func__);

#ifdef	MXT_GESTURE_RECOGNIZE
	wake_lock_init(&touch_wake_lock, WAKE_LOCK_SUSPEND, "touch_irq");
	mutex_init(&i2c_suspend_lock);
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TOUCH_INFO_MSG("i2c functionality check error\n");
		return -ENOMEM;
	}

	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (!data) {
		TOUCH_INFO_MSG("Failed to allocate memory\n");
		return -ENOMEM;
	}

	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
		 client->adapter->nr, client->addr);
	TOUCH_INFO_MSG("i2c-%u-%04x/input0\n", client->adapter->nr, client->addr);
	data->client = client;
	data->irq = client->irq;
	i2c_set_clientdata(client, data);

	/*read dtsi data*/
	if (client->dev.of_node) {
		data->pdata = devm_kzalloc(&client->dev,
			sizeof(struct mxt_platform_data), GFP_KERNEL);
		if (!data->pdata) {
			TOUCH_INFO_MSG("Failed to allocate memory\n");
			error = -ENOMEM;
			goto err_free_mem;
		}

		error = mxt_parse_dt(&client->dev, data->pdata);
		if (error)
			goto err_free_mem;

	} else{
		error = mxt_handle_pdata(data);
		if (error)
			goto err_free_mem;
	}
	/*read dtsi data*/

	init_completion(&data->bl_completion);
	init_completion(&data->reset_completion);
	init_completion(&data->crc_completion);
	/* Self Test */
	init_completion(&data->t25_completion);

	/* request reset pin */
	if(data->pdata->gpio_reset> 0){
		error = gpio_request(data->pdata->gpio_reset, "touch_reset");
		if (error < 0) {
			TOUCH_INFO_MSG("FAIL: touch_reset gpio_request\n");
			goto err_interrupt_failed;
		}
//		gpio_direction_output(data->pdata->gpio_reset, 1);
	}

	/* request interrupt pin */
	if(data->pdata->gpio_int > 0){
		error = gpio_request(data->pdata->gpio_int, "touch_int");
		if (error < 0) {
			TOUCH_INFO_MSG("FAIL: touch_int gpio_request\n");
			goto err_interrupt_failed;
		}
		gpio_direction_input(data->pdata->gpio_int);
	}

	error = request_threaded_irq(data->irq, NULL, mxt_interrupt,
				     data->pdata->irqflags | IRQF_ONESHOT,
				     client->name, data);
	if (error) {
		TOUCH_INFO_MSG("Failed to register interrupt\n");
		goto err_free_pdata;
	}

	mxt_probe_regulators(data);

	touch_disable_irq(data->irq);

	error = mxt_initialize(data);
	if (error)
		goto err_free_irq;

	if(data->info != NULL) {
		fw_version = data->info->version;
		fw_build = data->info->build;
	} else {
		TOUCH_INFO_MSG("%s data->info = NULL \n", __func__);
	}

	if (data->pdata->auto_fw_update && (data->in_bootloader || 
		fw_version != data->pdata->fw_ver[0] || fw_build != data->pdata->fw_ver[1])) {

		mxt_acquire_irq(data);

		TOUCH_INFO_MSG("Execute firmware update func\n");
		error = mxt_firmware_update(data);
		 if (error) {
			TOUCH_INFO_MSG("Failed to update firmware\n");
			return error;
		} else {
			TOUCH_INFO_MSG("Update firmware complete\n");
		}
	}

	error = mxt_rest_init(data);
	if (error)
		goto err_free_irq;

	if (data->pdata->panel_check) {
		mxt_read_one_page_diagnostic_data(data,	MXT_DIAG_REFERENCE_MODE);

		if(data->rawdata->reference[0] < 1000 || data->rawdata->reference[3] > 50000) {
			TOUCH_INFO_MSG("Config will change \n");
			error = mxt_check_reg_init(data, data->pdata->extra_cfg_name);
			if (error < 0) {
				TOUCH_INFO_MSG("Error mxt_check_reg_init ret=%d\n", error);
				goto err_free_irq;
			}

			if (data->pdata->t15_extra_keystate[0] > 0) {
				for(i = 0; i < data->pdata->t15_num_keys; i++) {
					data->pdata->t15_keystate[i] = data->pdata->t15_extra_keystate[i];
					TOUCH_INFO_MSG("Update t15_keystate[%d] = [%3d] \n", i, data->pdata->t15_keystate[i]);
				}
			}
		} else {
			TOUCH_INFO_MSG("No need to change Config \n");
		}
	}

#if defined(CONFIG_FB)
		data->fb_notif.notifier_call = fb_notifier_callback;

		error = fb_register_client(&data->fb_notif);
		if (error)
			TOUCH_INFO_MSG("Unable to register fb_notifier: %d\n",
				error);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
		data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
							MXT_SUSPEND_LEVEL;
		data->early_suspend.suspend = mxt_early_suspend;
		data->early_suspend.resume = mxt_late_resume;
		register_early_suspend(&data->early_suspend);
#endif

	/* disabled report touch event to prevent unnecessary event.
	* it will be enabled in open function
	*/
	mxt_stop(data);

	/* Register sysfs for making fixed communication path to framework layer */
	error = sysdev_class_register(&lge_touch_sys_class);
	if (error < 0) {
		TOUCH_INFO_MSG("sysdev_class_register is failed\n");
		goto err_lge_touch_sys_class_register;
	}

	error = sysdev_register(&lge_touch_sys_device);
	if (error < 0) {
		TOUCH_INFO_MSG("sysdev_register is failed\n");
		goto err_lge_touch_sys_dev_register;
	}

	error = kobject_init_and_add(&data->lge_touch_kobj, &lge_touch_kobj_type,
			data->input_dev->dev.kobj.parent,
			"%s", LGE_TOUCH_NAME);
	if (error < 0) {
		TOUCH_INFO_MSG("kobject_init_and_add is failed\n");
		goto err_lge_touch_sysfs_init_and_add;
	}

	sysfs_bin_attr_init(&data->mem_access_attr);
	data->mem_access_attr.attr.name = "mem_access";
	data->mem_access_attr.attr.mode = S_IRWXUGO;
	data->mem_access_attr.read = mxt_mem_access_read;
	data->mem_access_attr.write = mxt_mem_access_write;
	data->mem_access_attr.size = data->mem_size;

	if (sysfs_create_bin_file(&client->dev.kobj,
				  &data->mem_access_attr) < 0) {
		TOUCH_INFO_MSG("Failed to create %s\n",
			data->mem_access_attr.attr.name);
		goto err_lge_touch_sysfs_init_and_add;
	}

	TOUCH_INFO_MSG("probe success\n");
	is_probing = false;
	return 0;

err_lge_touch_sysfs_init_and_add:
	kobject_del(&data->lge_touch_kobj);
err_lge_touch_sys_dev_register:
	sysdev_unregister(&lge_touch_sys_device);
err_lge_touch_sys_class_register:
	sysdev_class_unregister(&lge_touch_sys_class);
	mxt_free_object_table(data);
err_free_irq:
	free_irq(data->irq, data);
err_interrupt_failed:
err_free_pdata:
err_free_mem:
	if(data)
		kfree(data);
	return error;
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

#if defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif))
		TOUCH_INFO_MSG("Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif

	kobject_del(&data->lge_touch_kobj);
	sysdev_unregister(&lge_touch_sys_device);
	sysdev_class_unregister(&lge_touch_sys_class);

	if (data->pdata->gpio_int > 0)
		gpio_free(data->pdata->gpio_int);
	free_irq(data->irq, data);
	regulator_put(data->vdd_ana);
	regulator_put(data->vcc_i2c);
	regulator_put(data->vcc_dig);
	mxt_free_object_table(data);
	if(data)
		kfree(data);
#ifdef MXT_GESTURE_RECOGNIZE
	wake_lock_destroy(&touch_wake_lock);
#endif

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mxt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	TOUCH_INFO_MSG("%s\n", __func__);

	mutex_lock(&input_dev->mutex);
#if 0 //def MXT_GESTURE_RECOGNIZE
	data->mxt_knock_on_enable = true;	/*                                                                        */
	data->mxt_character_enable = true;
#endif

	if (input_dev->users)
		mxt_stop(data);

#ifdef MXT_GESTURE_RECOGNIZE
	if (data->mxt_knock_on_enable || data->mxt_character_enable) {
		touch_enable_irq_wake(data->irq);
		dev_dbg(&client->dev, "touch enable irq wake");
	}
#endif
	mutex_unlock(&input_dev->mutex);
	return 0;
}

static int mxt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	dev_info(&client->dev, "%s\n", __func__);

	mutex_lock(&input_dev->mutex);
#ifdef MXT_GESTURE_RECOGNIZE
	if (data->mxt_knock_on_enable || data->mxt_character_enable) {
		touch_disable_irq_wake(data->irq);
		dev_dbg(&client->dev, "touch disable irq wake");
	}
#endif
	if (input_dev->users)
		mxt_start(data);

#if 0 //def MXT_GESTURE_RECOGNIZE
	data->mxt_knock_on_enable = false;	/*                                                                        */
	data->mxt_character_enable = false;
#endif
	mutex_unlock(&input_dev->mutex);
	return 0;
}
#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct mxt_data *mxt_dev_data =
		container_of(self, struct mxt_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK && mxt_dev_data &&
			mxt_dev_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			mxt_resume(&mxt_dev_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			mxt_suspend(&mxt_dev_data->client->dev);
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void mxt_early_suspend(struct early_suspend *h)
{
	struct mxt_data *data = container_of(h, struct mxt_data,
						early_suspend);
	mxt_suspend(&data->client->dev);
}

static void mxt_late_resume(struct early_suspend *h)
{
	struct mxt_data *data = container_of(h, struct mxt_data,
						early_suspend);
	mxt_resume(&data->client->dev);
}

#endif

#if defined(CONFIG_FB)
static const struct dev_pm_ops mxt_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend	= mxt_suspend,
	.resume		= mxt_resume,
#endif
};
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static const struct dev_pm_ops mxt_pm_ops = {
	.suspend	= NULL,
	.resume		= NULL,
};
#else
static SIMPLE_DEV_PM_OPS(mxt_pm_ops, mxt_suspend, mxt_resume);
#endif

static void mxt_shutdown(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	touch_disable_irq(data->irq);
}

static struct of_device_id mxt_match_table[] = {
	{ .compatible = "atmel,s336",},
	{ },
};

static const struct i2c_device_id mxt_id[] = {
	{ "qt602240_ts", 0 },
	{ "atmel_mxt_ts", 0 },
	{ "atmel_mxt_tp", 0 },
	{ "mXT224", 0 },
	{ MXT_DEVICE_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "touch_atmel",
		.of_match_table = mxt_match_table,
		.owner	= THIS_MODULE,
		.pm	= &mxt_pm_ops,
	},
	.probe		= mxt_probe,
	.remove		= __devexit_p(mxt_remove),
	.shutdown	= mxt_shutdown,
	.id_table	= mxt_id,
};

static int __init mxt_init(void)
{
	return i2c_add_driver(&mxt_driver);
}

static void __exit mxt_exit(void)
{
	i2c_del_driver(&mxt_driver);
}

module_init(mxt_init);
module_exit(mxt_exit);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");

