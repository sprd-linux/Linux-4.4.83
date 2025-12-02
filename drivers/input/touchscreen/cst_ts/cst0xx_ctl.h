#ifndef __CST0XX_CTL_H__
#define __CST0XX_CTL_H__

#define CST0XX_RW_IIC_DRV  "cst0xx_rw_iic_drv"
#define CST0XX_RW_IIC_DRV_MAJOR    210 

#define CST0XX_I2C_RDWR_MAX_QUEUE 	36
#define CST0XX_I2C_SLAVEADDR   			11
#define CST0XX_I2C_RW          				12

typedef struct cst0xx_rw_i2c
{
	u8 *buf;  	
	u8 flag;	/*0-write 1-read*/
	__u16 length; //the length of data 
}*pcst0xx_rw_i2c;

typedef struct cst0xx_rw_i2c_queue
{
	struct cst0xx_rw_i2c __user *i2c_queue;
	int queuenum;	
}*pcst0xx_rw_i2c_queue;

int cst0xx_rw_iic_drv_init(struct i2c_client *client);
void  cst0xx_rw_iic_drv_exit(void);

#endif

