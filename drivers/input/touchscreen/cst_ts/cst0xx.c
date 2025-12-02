/*
 * drivers/input/touchscreen/cst0xx.c
 *
 * Copyright (c) 2017 Hynitron Ltd.
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
 * VERSION      	DATE			AUTHOR
 *    1.0		  2017-05			Gary
 *
 */

#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
//#include <soc/sprd/regulator.h>
#include <linux/input/mt.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>
#include <linux/pm_runtime.h>

#if(defined(CONFIG_I2C_SPRD) ||defined(CONFIG_I2C_SPRD_V1))
//#include <soc/sprd/i2c-sprd.h>
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#ifdef CONFIG_PM_SLEEP
#include <video/adf_notifier.h>
#endif

#define HYN_DBG
#ifdef HYN_DBG
#define ENTER printk(KERN_INFO "[HYN_DBG] func: %s  line: %04d\n", __func__, __LINE__);
#define PRINT_DBG(x...)  printk(KERN_INFO "[HYN_DBG] " x)
#define PRINT_INFO(x...)  printk(KERN_INFO "[HYN_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[HYN_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[HYN_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#else
#define ENTER
#define PRINT_DBG(x...)
#define PRINT_INFO(x...)  printk(KERN_INFO "[HYN_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[HYN_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[HYN_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#endif

#define HYN_CTL_IIC
#include "cst0xx.h"
#include "cst0xx_ex_fun.h"
#include "cst0xx_ctl.h"

#define	TOUCH_VIRTUAL_KEYS
#define	MULTI_PROTOCOL_TYPE_B	0
#define	TS_MAX_FINGER		1

#define	USE_WAIT_QUEUE	1
#define	USE_THREADED_IRQ	0
#define	USE_WORK_QUEUE	0


#define TP_UPGRADE 


//typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u8;

#if USE_WAIT_QUEUE
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int tpd_flag = 0;
#endif
#ifdef CONFIG_PM_SLEEP
static struct notifier_block adf_event_block;
#endif
struct ts_event{
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16	x3;
	u16	y3;
	u16	x4;
	u16	y4;
	u16	x5;
	u16	y5;
	u16	pressure;
    u8  touch_point;
};

struct cst0xx_data{
	struct input_dev	*input_dev;
	struct i2c_client	*client;
	struct ts_event	    event;
#if USE_WORK_QUEUE
	struct work_struct	pen_event_work;
	struct workqueue_struct	*cst0xx_workqueue;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct work_struct       resume_work;
	struct workqueue_struct *cst0xx_resume_workqueue;
	struct early_suspend	early_suspend;
#else
	struct work_struct       resume_work;
	struct workqueue_struct *cst0xx_resume_workqueue;
#endif
	struct cst0xx_platform_data	*platform_data;
};


static struct cst0xx_data *g_cst0xx;
static struct i2c_client *this_client;


//extern int ctp_hynitron_update(struct i2c_client *mclient);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cst0xx_ts_suspend(struct early_suspend *handler);
static void cst0xx_ts_resume(struct early_suspend *handler);
#endif

#ifdef TOUCH_VIRTUAL_KEYS

static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct cst0xx_data *data = i2c_get_clientdata(this_client);
	struct cst0xx_platform_data *pdata = data->platform_data;
	return sprintf(buf,"%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d\n"
		,__stringify(EV_KEY), __stringify(KEY_MENU),pdata ->virtualkeys[0],pdata ->virtualkeys[1],pdata ->virtualkeys[2],pdata ->virtualkeys[3]
		,__stringify(EV_KEY), __stringify(KEY_HOMEPAGE),pdata ->virtualkeys[4],pdata ->virtualkeys[5],pdata ->virtualkeys[6],pdata ->virtualkeys[7]
		,__stringify(EV_KEY), __stringify(KEY_BACK),pdata ->virtualkeys[8],pdata ->virtualkeys[9],pdata ->virtualkeys[10],pdata ->virtualkeys[11]);
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.cst0xx_ts",
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

static void cst0xx_virtual_keys_init(void)
{
    int ret = 0;
    struct kobject *properties_kobj;

    pr_info("[HYN] %s\n",__func__);

    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                     &properties_attr_group);
    if (!properties_kobj || ret)
        pr_err("failed to create board_properties\n");
}

#endif

static void cst0xx_clear_report_data(struct cst0xx_data *cst0xx)
{
#if MULTI_PROTOCOL_TYPE_B
	for(i = 0; i < TS_MAX_FINGER; i++) {
		input_mt_slot(cst0xx->input_dev, i);
		input_mt_report_slot_state(cst0xx->input_dev, MT_TOOL_FINGER, false);
	}
	input_report_key(cst0xx->input_dev, BTN_TOUCH, 0);
	input_sync(cst0xx->input_dev);
#else
    input_report_key(cst0xx->input_dev, BTN_TOUCH, 0);
    input_mt_sync(cst0xx->input_dev);
    input_sync(cst0xx->input_dev);
#endif
}

/*
 *
 */
static int cst0xx_update_data(void){
	struct cst0xx_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	u8 buf[33] = {0};
	int ret = -1;
	int i;
	u16 x , y;
	u8 ft_pressure , ft_size;

	ret = cst0xx_i2c_Read(this_client, buf, 1, buf, sizeof(buf));

	if (ret < 0) {
		pr_err("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x07;

	for(i = 0; i < TS_MAX_FINGER; i++){
		if((buf[6*i+3] & 0xc0) == 0xc0)
			continue;
		x = (u16)(buf[6*i+3] & 0x0F)<<8 | (u16)buf[6*i+4];	
		y = (u16)(buf[6*i+5] & 0x0F)<<8 | (u16)buf[6*i+6];
		ft_pressure = buf[6*i+7];
		if((ft_pressure>PRESS_MAX) || (ft_pressure==0)){
			ft_pressure = PRESS_MAX;
		}
		ft_size = (buf[6*i+8]>>4) & 0x0F;
		if(ft_size == 0){
			ft_size = 0x09;
		}
		if((buf[6*i+3] & 0x40) == 0x0){
#if MULTI_PROTOCOL_TYPE_B
            input_mt_slot(data->input_dev, buf[6*i+5]>>4);
            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
            input_report_abs(data->input_dev, ABS_MT_POSITION_X, x);
            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, y);
            input_report_abs(data->input_dev, ABS_MT_PRESSURE, ft_pressure);
            input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, ft_size);
            input_report_key(data->input_dev, BTN_TOUCH, 1);
#else
            input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, buf[6*i+5]>>4);
            input_report_abs(data->input_dev, ABS_MT_POSITION_X, x);
            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, y);    
         //   input_report_abs(data->input_dev, ABS_MT_POSITION_X, 240-x);
          //  input_report_abs(data->input_dev, ABS_MT_POSITION_Y, 240-y);
            input_report_abs(data->input_dev, ABS_MT_PRESSURE, ft_pressure);
            input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, ft_size);
            input_report_key(data->input_dev, BTN_TOUCH, 1);
            input_mt_sync(data->input_dev);
#endif
			PRINT_DBG("===x%d = %d,y%d = %d ====\n",i, x, i, y);
		}
		else {
		#if MULTI_PROTOCOL_TYPE_B
			input_mt_slot(data->input_dev, buf[6*i+5]>>4);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
		#endif
		}
	}
	if(0 == event->touch_point) {
#if MULTI_PROTOCOL_TYPE_B
		for(i = 0; i < TS_MAX_FINGER; i ++) {
            input_mt_slot(data->input_dev, i);
            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
		}
#endif
		input_report_key(data->input_dev, BTN_TOUCH, 0);
#if !MULTI_PROTOCOL_TYPE_B
        input_mt_sync(data->input_dev);
#endif

	}
	input_sync(data->input_dev);

	return 0;
}

/*
 *
 */
#if USE_WAIT_QUEUE
static int touch_event_handler(void *unused)
{
	struct sched_param param = { .sched_priority = 5 };
	sched_setscheduler(current, SCHED_RR, &param);
	do {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, (0 != tpd_flag));
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);
		cst0xx_update_data();

	} while (!kthread_should_stop());

	return 0;
}
#endif

/*
 *
 */
#if USE_WORK_QUEUE
static void cst0xx_pen_irq_work(struct work_struct *work)
{
	cst0xx_update_data();
	enable_irq(this_client->irq);
}
#endif

/*
 *
 */
static irqreturn_t cst0xx_interrupt(int irq, void *dev_id)
{
#if USE_WAIT_QUEUE
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
#endif

#if USE_WORK_QUEUE
	struct cst0xx_data *cst0xx = (struct cst0xx_data *)dev_id;

	if (!work_pending(&cst0xx->pen_event_work)) {
		queue_work(cst0xx->cst0xx_workqueue, &cst0xx->pen_event_work);
	}
	return IRQ_HANDLED;
#endif

#if USE_THREADED_IRQ
	cst0xx_update_data();
	return IRQ_HANDLED;
#endif

}

/*
 *
 */
void cst0xx_reset(void){
	struct cst0xx_platform_data *pdata = g_cst0xx->platform_data;

	gpio_direction_output(pdata->reset_gpio_number, 1);
	mdelay(1);
	gpio_set_value(pdata->reset_gpio_number, 0);
	mdelay(10);
	gpio_set_value(pdata->reset_gpio_number, 1);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cst0xx_ts_suspend(struct early_suspend *handler)
#else
static int cst0xx_ts_suspend(struct device *dev)
#endif
{
	int ret = -1;
    u8 cmd[2] = {0xA5,0x03};
    u8 cmd2[2] = {0xe5,0x03};
	pr_info("==%s==\n", __FUNCTION__);

	ret = cst0xx_i2c_Write(this_client, cmd, sizeof(cmd));
	if(ret<0){
		PRINT_ERR("==cst716_suspend==  cst0xx_write fail\n");
	}
	mdelay(10);
	ret = cst0xx_i2c_Write(this_client, cmd2, sizeof(cmd2));
	if(ret<0){
		PRINT_ERR("==cst816_suspend==  cst0xx_write fail\n");
	}
	disable_irq(this_client->irq);
	cst0xx_clear_report_data(g_cst0xx);
#ifdef CONFIG_PM_SLEEP
	return 0;
#else
	return;
#endif
}
/*
 *
 */
#ifdef CONFIG_HAS_EARLYSUSPEND
static void cst0xx_ts_resume(struct early_suspend *handler)
#else
static int cst0xx_ts_resume(struct device *dev)
#endif
{
	struct cst0xx_data  *cst0xx = (struct cst0xx_data *)i2c_get_clientdata(this_client);
	queue_work(cst0xx->cst0xx_resume_workqueue, &cst0xx->resume_work);
#ifdef CONFIG_PM_SLEEP
	return 0;
#else
	return;
#endif
}


/*
 *
 */
static void cst0xx_resume_work(struct work_struct *work)
{
	pr_info("==%s==\n", __FUNCTION__);

	cst0xx_reset();
    mdelay(200);
	enable_irq(this_client->irq);
	mdelay(2);
	cst0xx_clear_report_data(g_cst0xx);
}

#ifdef CONFIG_PM_SLEEP
static int ts_adf_event_handler(struct notifier_block *nb, unsigned long action, void *data)
{
	struct adf_notifier_event *event = data;
	int adf_event_data;
	struct device *pdev;

	if (action != ADF_EVENT_BLANK)
		return NOTIFY_DONE;

	adf_event_data = *(int *)event->data;
	//PRINT_INFO("receive adf event with adf_event_data=%d", adf_event_data);

	switch (adf_event_data) {
	case DRM_MODE_DPMS_ON:
		cst0xx_ts_resume(pdev);
		break;
	case DRM_MODE_DPMS_OFF:
		cst0xx_ts_suspend(pdev);
		break;
	default:
		//TS_WARN("receive adf event with error data, adf_event_data=%d",adf_event_data);
		break;
	}

	return NOTIFY_OK;
}
#endif
/*
 *
 */
static void cst0xx_hw_init(struct cst0xx_data *cst0xx)
{
	struct regulator *reg_vdd;
	struct i2c_client *client = cst0xx->client;
	struct cst0xx_platform_data *pdata = cst0xx->platform_data;

	pr_info("[HYN] %s [irq=%d];[rst=%d]\n",__func__,
		pdata->irq_gpio_number,pdata->reset_gpio_number);
	gpio_request(pdata->irq_gpio_number, "ts_irq_pin");
	gpio_request(pdata->reset_gpio_number, "ts_rst_pin");
	gpio_direction_output(pdata->reset_gpio_number, 1);
	gpio_direction_input(pdata->irq_gpio_number);

	reg_vdd = regulator_get(&client->dev, pdata->vdd_name);
	if (!WARN(IS_ERR(reg_vdd), "[HYN] cst0xx_hw_init regulator: failed to get %s.\n", pdata->vdd_name)) {
		regulator_set_voltage(reg_vdd, 2800000, 2800000);
		//regulator_enable(reg_vdd);
	}
	mdelay(100);
	cst0xx_reset();
	mdelay(200);
}

#ifdef CONFIG_OF
static struct cst0xx_platform_data *cst0xx_parse_dt(struct device *dev)
{
	struct cst0xx_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int ret;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct cst0xx_platform_data");
		return NULL;
	}
	pdata->reset_gpio_number = of_get_gpio(np, 0);
	if(pdata->reset_gpio_number < 0){
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	pdata->irq_gpio_number = of_get_gpio(np, 1);
	if(pdata->reset_gpio_number < 0){
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	ret = of_property_read_string(np, "vdd_name", &pdata->vdd_name);
	if(ret){
		dev_err(dev, "fail to get vdd_name\n");
		goto fail;
	}
	ret = of_property_read_u32_array(np, "virtualkeys", (u32 *)&pdata->virtualkeys,12);
	if(ret){
		dev_err(dev, "fail to get virtualkeys\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_X", &pdata->TP_MAX_X);
	if(ret){
		dev_err(dev, "fail to get TP_MAX_X\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_Y", &pdata->TP_MAX_Y);
	if(ret){
		dev_err(dev, "fail to get TP_MAX_Y\n");
		goto fail;
	}

	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif

static int cst0xx_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cst0xx_data *cst0xx;
	struct input_dev *input_dev;
	struct cst0xx_platform_data *pdata = client->dev.platform_data;
	int err = 0;
	struct device_node *np = client->dev.of_node;

	pr_info("[HYN] %s: probe\n",__func__);
#ifdef CONFIG_OF
	if (np && !pdata){
		pdata = cst0xx_parse_dt(&client->dev);
		if(pdata){
			client->dev.platform_data = pdata;
		}
		else{
			err = -ENOMEM;
			goto exit_alloc_platform_data_failed;
		}
	}
#endif
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	cst0xx = kzalloc(sizeof(*cst0xx), GFP_KERNEL);
	if (!cst0xx)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	g_cst0xx = cst0xx;
	cst0xx->platform_data = pdata;
	this_client = client;
	cst0xx->client = client;
	cst0xx_hw_init(cst0xx);
	i2c_set_clientdata(client, cst0xx);
	client->irq = gpio_to_irq(pdata->irq_gpio_number);

	#if(defined(CONFIG_I2C_SPRD) ||defined(CONFIG_I2C_SPRD_V1))
//	sprd_i2c_ctl_chg_clk(client->adapter->nr, 100000);
	#endif 

#if USE_WORK_QUEUE
	INIT_WORK(&cst0xx->pen_event_work, cst0xx_pen_irq_work);

	cst0xx->cst0xx_workqueue = create_singlethread_workqueue("focal-work-queue");
	if (!cst0xx->cst0xx_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
#endif

	INIT_WORK(&cst0xx->resume_work, cst0xx_resume_work);
	cst0xx->cst0xx_resume_workqueue = create_singlethread_workqueue("cst0xx_resume_work");
	if (!cst0xx->cst0xx_resume_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "[HYN] failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
#ifdef TOUCH_VIRTUAL_KEYS
	cst0xx_virtual_keys_init();
#endif
	cst0xx->input_dev = input_dev;

	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	__set_bit(KEY_MENU,  input_dev->keybit);
	__set_bit(KEY_BACK,  input_dev->keybit);
	__set_bit(KEY_HOMEPAGE,  input_dev->keybit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

#if MULTI_PROTOCOL_TYPE_B
	input_mt_init_slots(input_dev, TS_MAX_FINGER,0);
#endif
	input_set_abs_params(input_dev,ABS_MT_POSITION_X, 0, pdata->TP_MAX_X, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_POSITION_Y, 0, pdata->TP_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_TOUCH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_PRESSURE, 0, 127, 0, 0);
#if !MULTI_PROTOCOL_TYPE_B
	input_set_abs_params(input_dev,ABS_MT_TRACKING_ID, 0, 255, 0, 0);
#endif

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	input_dev->name = CST0XX_TS_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"[HYN] cst0xx_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#if USE_THREADED_IRQ
	err = request_threaded_irq(client->irq, NULL, cst0xx_interrupt, 
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND, client->name, cst0xx);
#else
	err = request_irq(client->irq, cst0xx_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND, client->name, cst0xx);
#endif
	if (err < 0) {
		dev_err(&client->dev, "[HYN] cst0xx_probe: request irq failed %d\n",err);
		goto exit_irq_request_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	cst0xx->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	cst0xx->early_suspend.suspend = cst0xx_ts_suspend;
	cst0xx->early_suspend.resume	= cst0xx_ts_resume;
	register_early_suspend(&cst0xx->early_suspend);
#endif
#ifdef CONFIG_PM_SLEEP
	adf_event_block.notifier_call = ts_adf_event_handler;
	err = adf_register_client(&adf_event_block);
#endif

	#ifdef HYN_CTL_IIC	
	if (cst0xx_rw_iic_drv_init(client) < 0)	
	{
		dev_err(&client->dev, "%s:[HYN] create cst0xx control iic driver failed\n",	__func__);
	}
	#endif
#ifdef TP_UPGRADE
	ctp_hynitron_update(client);
	cst0xx_reset(); 
        mdelay(200); 
#endif

#if USE_WAIT_QUEUE
	thread = kthread_run(touch_event_handler, 0, "focal-wait-queue");
	if (IS_ERR(thread))
	{
		err = PTR_ERR(thread);
		PRINT_ERR("failed to create kernel thread: %d\n", err);
	}
#endif

	return 0;

exit_irq_request_failed:
	input_unregister_device(input_dev);
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
exit_alloc_data_failed:
exit_create_singlethread:
exit_check_functionality_failed:
	cst0xx = NULL;
	i2c_set_clientdata(client, cst0xx);
exit_alloc_platform_data_failed:
	return err;
}

static int cst0xx_remove(struct i2c_client *client)
{
	struct cst0xx_data *cst0xx = i2c_get_clientdata(client);

	pr_info("==cst0xx_remove=\n");
	

#ifdef HYN_CTL_IIC	
	cst0xx_rw_iic_drv_exit();
#endif

	
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&cst0xx->early_suspend);
#endif
	free_irq(client->irq, cst0xx);
	input_unregister_device(cst0xx->input_dev);
	input_free_device(cst0xx->input_dev);
#if USE_WORK_QUEUE
	cancel_work_sync(&cst0xx->pen_event_work);
	destroy_workqueue(cst0xx->cst0xx_workqueue);
#endif
	cancel_work_sync(&cst0xx->resume_work);
	destroy_workqueue(cst0xx->cst0xx_resume_workqueue);
	kfree(cst0xx);
	cst0xx = NULL;
	i2c_set_clientdata(client, cst0xx);

	return 0;
}

static const struct i2c_device_id cst0xx_id[] = {
	{ CST0XX_TS_NAME, 0 },{ }
};

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops cst0xx_pm_ops = {
	SET_RUNTIME_PM_OPS(cst0xx_ts_suspend, cst0xx_ts_resume, NULL)
};
#endif

MODULE_DEVICE_TABLE(i2c, cst0xx_id);

static const struct of_device_id cst0xx_of_match[] = {
       { .compatible = "cst0xx,cst0xx_ts", },
       { }
};
MODULE_DEVICE_TABLE(of, cst0xx_of_match);
static struct i2c_driver cst0xx_driver = {
	.probe		= cst0xx_probe,
	.remove		= cst0xx_remove,
	.id_table	= cst0xx_id,
	.driver	= {
		.name	= CST0XX_TS_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = cst0xx_of_match,
#ifdef CONFIG_PM_SLEEP
		.pm    = &cst0xx_pm_ops,
#endif
		//.suspend = cst0xx_suspend,
		//.resume = cst0xx_resume,
	},
};

static int __init cst0xx_init(void)
{
	return i2c_add_driver(&cst0xx_driver);
}

static void __exit cst0xx_exit(void)
{
	i2c_del_driver(&cst0xx_driver);
}

module_init(cst0xx_init);
module_exit(cst0xx_exit);

MODULE_AUTHOR("<mshl>");
MODULE_DESCRIPTION("cst0xx driver");
MODULE_LICENSE("GPL");
