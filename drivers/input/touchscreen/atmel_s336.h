/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_MXT_TS_H
#define __LINUX_ATMEL_MXT_TS_H

#ifndef CUST_B_TOUCH
#define CUST_B_TOUCH
#endif

#include <linux/types.h>

#define LGE_TOUCH_NAME			"lge_touch"
#define MXT_DEVICE_NAME			"touch_dev"
#define MXT_MAX_NUM_TOUCHES		8
#define MXT_MAX_NUM_KEY			4

#define REF_OFFSET_VALUE	16384
#define REF_MIN_VALUE		(19744 - REF_OFFSET_VALUE)
#define REF_MAX_VALUE		(28884 - REF_OFFSET_VALUE)

#define NODE_PER_PAGE		64
#define DATA_PER_NODE		2

#define MXT_GESTURE_RECOGNIZE

#define TOUCH_VDD_VTG_MIN_UV 	2950000
#define TOUCH_VDD_VTG_MAX_UV 	2950000

#define T_VALUE_MAX 22
#define T_BYTE_MAX 20
#ifdef CUST_B_TOUCH
enum{
	FINGER_INACTIVE,
	FINGER_RELEASED,
	FINGER_PRESSED,
	FINGER_MOVED
};
#endif

enum{
	T7_GESTURE_MODE=0,
	T7_GESTURE_MASK,
	T8_GESTURE_MODE,
	T8_GESTURE_MASK,
	T9_GESTURE_MODE,
	T9_GESTURE_MASK,
	T15_GESTURE_MODE,
	T15_GESTURE_MASK,
	T24_GESTURE_MODE,
	T24_GESTURE_MASK,
	T42_GESTURE_MODE,
	T42_GESTURE_MASK,
	T46_GESTURE_MODE,
	T46_GESTURE_MASK,
	T56_GESTURE_MODE,
	T56_GESTURE_MASK,
	T65_GESTURE_MODE,
	T65_GESTURE_MASK,
	T72_GESTURE_MODE,
	T72_GESTURE_MASK,
	GESTURE_MAX
};
struct gestture_config_info {
	u8 *T_value[T_VALUE_MAX];
	u8 *T_active_value[T_VALUE_MAX];
};

const char *gesture_node_name[GESTURE_MAX] = {
	"atmel,t7_gesture_mode"
	 ,"atmel,t7_gesture_mask"
	 ,"atmel,t8_gesture_mode"
	 ,"atmel,t8_gesture_mask"
	 ,"atmel,t9_gesture_mode"
	 ,"atmel,t9_gesture_mask"
	 ,"atmel,t15_gesture_mode"
	 ,"atmel,t15_gesture_mask"
	 ,"atmel,t24_gesture_mode"
	 ,"atmel,t24_gesture_mask"
	 ,"atmel,t42_gesture_mode"
	 ,"atmel,t42_gesture_mask"
	 ,"atmel,t46_gesture_mode"
	 ,"atmel,t46_gesture_mask"
	 ,"atmel,t56_gesture_mode"
	 ,"atmel,t56_gesture_mask"
	 ,"atmel,t65_gesture_mode"
	 ,"atmel,t65_gesture_mask"
	 ,"atmel,t72_gesture_mode"
	 ,"atmel,t72_gesture_mask"
};

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	size_t config_array_size;
	u8    numtouch;	/* Number of touches to report	*/
	int   max_x;    /* The default reported X range   */
	int   max_y;    /* The default reported Y range   */
	bool i2c_pull_up;
	unsigned long irqflags;
	u8 t19_num_keys;
	unsigned int t19_keymap[MXT_MAX_NUM_KEY];
	int t15_num_keys;
	unsigned int t15_keystate[MXT_MAX_NUM_KEY];
	unsigned int t15_extra_keystate[MXT_MAX_NUM_KEY];
	unsigned int t15_keymap[MXT_MAX_NUM_KEY];
	unsigned long gpio_reset;
	unsigned long gpio_int;
	unsigned int panel_check;
	const char *cfg_name;
	const char *extra_cfg_name;
	const char *bin_name;
	unsigned char fw_ver[2];
	struct gestture_config_info *t_gesture ;
	unsigned char auto_fw_update;
};

#define TOUCH_INFO_MSG(fmt, args...) 	printk(KERN_ERR "[Touch] " fmt, ##args)

#endif /* __LINUX_ATMEL_MXT_TS_H */
