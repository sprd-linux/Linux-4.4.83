#ifndef __LINUX_HYN_TS_H__
#define __LINUX_HYN_TS_H__

#define SCREEN_MAX_X    240
#define SCREEN_MAX_Y    240

#define PRESS_MAX           127
#define TOUCH_VIRTUAL_KEYS

#define CST0XX_TS_NAME	   	       	"cst0xx_ts"
#define CST0XX_TS_ADDR				0x15
#define CTP_HYNITRON_EXT            1
#define CTP_HYNITRON_EXT_CST0xxSE_UPDATE    0  
#define CTP_HYNITRON_EXT_CST78xx_UPDATE    0
#define CTP_HYNITRON_EXT_CST0xx_UPDATE  0
#define CTP_HYNITRON_EXT_CST816D_UPDATE    1


struct cst0xx_platform_data{
	int irq_gpio_number;
	int reset_gpio_number;
	const char *vdd_name;
	int virtualkeys[12];
	int TP_MAX_X;
	int TP_MAX_Y;
};

#define HYN_TS_DBG
#ifdef HYN_TS_DBG
#define DBG(fmt, args...) 				printk("[HYN]" fmt, ## args)
#else
#define DBG(fmt, args...) 				do{}while(0)
#endif


#ifndef ABS_MT_TOUCH_MAJOR
#define ABS_MT_TOUCH_MAJOR	0x30	/* touching ellipse */
#define ABS_MT_TOUCH_MINOR	0x31	/* (omit if circular) */
#define ABS_MT_WIDTH_MAJOR	0x32	/* approaching ellipse */
#define ABS_MT_WIDTH_MINOR	0x33	/* (omit if circular) */
#define ABS_MT_ORIENTATION	0x34	/* Ellipse orientation */
#define ABS_MT_POSITION_X	0x35	/* Center X ellipse position */
#define ABS_MT_POSITION_Y	0x36	/* Center Y ellipse position */
#define ABS_MT_TOOL_TYPE	0x37	/* Type of touching device */
#define ABS_MT_BLOB_ID		0x38	/* Group set of pkts as blob */
#define ABS_MT_PRESSURE		0x3a	/* Pressure on contact area */
#define ABS_MT_TRACKING_ID      0x39	/* Unique ID of initiated contact */
#endif /* ABS_MT_TOUCH_MAJOR */

extern void cst0xx_reset(void);

#endif

