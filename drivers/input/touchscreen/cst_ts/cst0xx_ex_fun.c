/*
 *drivers/input/touchscreen/cst0xx_ex_fun.c
 *
 *Copyright (c) 2017 Hynitron Ltd.
 *
 *This software is licensed under the terms of the GNU General Public
 *License version 2, as published by the Free Software Foundation, and
 *may be copied, distributed, and modified under those terms.
 *
 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *GNU General Public License for more details.
 *
 *Note:the error code of EIO is the general error in this file.
 */

#include <linux/netdevice.h>
#include <linux/mount.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>

#include "cst0xx.h"
#include "cst0xx_ex_fun.h"
#include "cst0xx_ctl.h"



/*
*cst0xx_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int cst0xx_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

#if 1//for normal I2c transfer
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = 1,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 1,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
#else// for DMA I2c transfer
	if(writelen!=0)
	{
		//DMA Write
		if(0)//if(writelen < 8  )
		{

			//MSE_ERR("Sensor non-dma write timing is %x!\r\n", this_client->timing);
			ret= i2c_master_send(client, writebuf, writelen);
		}
		else
		{
			for(i = 0 ; i < writelen; i++)
			{
				I2CDMABuf_va[i] = writebuf[i];
			}

			client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;

			if((ret=i2c_master_send(client, (unsigned char *)I2CDMABuf_pa, writelen))!=writelen)
				dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,I2CDMABuf_pa);
			//MSE_ERR("Sensor dma timing is %x!\r\n", this_client->timing);
			//return ret;
			client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);

		}
	}
	//DMA Read
	if(readlen!=0)
	{
		if(0)//if (readlen <8) {
		{
			ret = i2c_master_recv(client, (unsigned char *)readbuf, readlen);
		}
		else
		{

			client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
			ret = i2c_master_recv(client, (unsigned char *)I2CDMABuf_pa, readlen);

			for(i = 0; i < readlen; i++)
	        {
	            readbuf[i] = I2CDMABuf_va[i];
	        }
		client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);

		}
	}
	#endif
	return ret;
}
/*write data by i2c*/

int cst0xx_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

    #if 1
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);
	#else

	if(0)//if(writelen < 8)
	{
		//MSE_ERR("Sensor non-dma write timing is %x!\r\n", this_client->timing);
		ret = i2c_master_send(client, writebuf, writelen);
	}
	else
	{
		for(i = 0 ; i < writelen; i++)
		{
			I2CDMABuf_va[i] = writebuf[i];
		}

		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;

		if((ret=i2c_master_send(client, (unsigned char *)I2CDMABuf_pa, writelen))!=writelen)
			dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%x\n", __func__,ret,I2CDMABuf_pa);
		//MSE_ERR("Sensor dma timing is %x!\r\n", this_client->timing);
		client->addr = client->addr & I2C_MASK_FLAG &(~ I2C_DMA_FLAG);

	}
	#endif
	return ret;

}


#if CTP_HYNITRON_EXT==1
static u8 dev_addr;
static struct i2c_client *client;
/*****************************************************************/
/*
 *
 */
int hctp_write_bytes(u16 reg,u8 *buf,u16 len,u8 reg_len){
	int ret;
    u8 mbuf[600];
	struct i2c_msg msg[] = {
		{
		 .addr = dev_addr,
		 .flags = 0,
		 .len = len+reg_len,
		 .buf = mbuf,
		 },
	};
    if (reg_len == 1){
        mbuf[0] = reg;
        memcpy(mbuf+1,buf,len);
    }else{
        mbuf[0] = reg>>8;
        mbuf[1] = reg;
        memcpy(mbuf+2,buf,len);    
    }

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0){
		dev_err(&client->dev, "%s i2c write error.\n", __func__);
	}
    return ret;
}
/*
 *
 */
int hctp_read_bytes(u16 reg,u8* buf,u16 len,u8 reg_len){
	int ret;
    u8 reg_buf[2];
	struct i2c_msg msgs[] = {
		{
		 .addr = dev_addr,
		 .flags = 0,
		 .len = reg_len,
		 .buf = (__u8 *)&reg_buf,
		 },
		{
		 .addr = dev_addr,
		 .flags = 1,
		 .len = len,
		 .buf = (__u8 *)buf,
		 },
	};
    if (reg_len == 1){
        reg_buf[0] = reg;
    }else{
        reg_buf[0] = reg>>8;
        reg_buf[1] = reg;
    }
	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0){
		dev_err(&client->dev, "f%s: i2c read error.\n",__func__);
	}
    return ret;
}
#if CTP_HYNITRON_EXT_CST0xxSE_UPDATE==1
/*****************************************************************/
// For CST0xxSE update

#define CST0xxSE_REG_LEN    2
 /*
  *
  */
static int cst0xxse_enter_bootmode(void){
     char retryCnt = 10;

     cst0xx_reset();
     mdelay(5);
     while(retryCnt--){
         u8 cmd[3];
         cmd[0] = 0xAB;  //cst816S,  cst716se_b
         if (-1 == hctp_write_bytes(0xA001,cmd,1,CST0xxSE_REG_LEN)){  // enter program mode
             mdelay(2); // 4ms
             continue;                   
         }
         if (-1 == hctp_read_bytes(0xA003,cmd,1,CST0xxSE_REG_LEN)) { // read flag
             mdelay(2); // 4ms
             continue;                           
         }else{
             if (cmd[0] != 0x55){
                 mdelay(2); // 4ms
                 continue;
             }else{
                 return 0;
             }
         }
     }
     return -1;
 }
 /*
  *
  */
static void cst0xxse_update(u16 startAddr,u16 len,u8* src){
     u16 sum_len;
     u8 cmd[10];
 
     sum_len = 0;
 
#define PER_LEN	512
     do{
         if (sum_len >= len){
             return;
         }
         
         // send address
         cmd[0] = startAddr&0xFF;
         cmd[1] = startAddr>>8;
         hctp_write_bytes(0xA014,cmd,2,CST0xxSE_REG_LEN);
         
         hctp_write_bytes(0xA018,src,PER_LEN,CST0xxSE_REG_LEN);
 
         cmd[0] = 0xEE;
         hctp_write_bytes(0xA004,cmd,1,CST0xxSE_REG_LEN);
         mdelay(100);
 
         {
             u8 retrycnt = 50;
             while(retrycnt--){
                 cmd[0] = 0;
                 hctp_read_bytes(0xA005,cmd,1,CST0xxSE_REG_LEN);
                 if (cmd[0] == 0x55){
                     // success
                     break;
                 }
                 mdelay(10);
             }
         }
         startAddr += PER_LEN;
         src += PER_LEN;
         sum_len += PER_LEN;
     }while(len);
     
     // exit program mode
     cmd[0] = 0x00;
     hctp_write_bytes(0xA003,cmd,1,CST0xxSE_REG_LEN);
 }
 /*
  *
  */
static u32 cst0xxse_read_checksum(u16 startAddr,u16 len){
     union{
         u32 sum;
         u8 buf[4];
     }checksum;
	 char cmd[3];
     char readback[4] = {0};
	  if (cst0xxse_enter_bootmode() == -1){
		 return -1;
	  }

	  cmd[0] = 0;
	  if (-1 == hctp_write_bytes(0xA003,cmd,1,CST0xxSE_REG_LEN)){
		  return -1;
	  }
	  mdelay(500);
	  
     if (-1 == hctp_read_bytes(0xA000,readback,1,CST0xxSE_REG_LEN)){
         return -1;   
     }
     if (readback[0] != 1){
         return -1;
     }
     if (-1 == hctp_read_bytes(0xA008,checksum.buf,4,CST0xxSE_REG_LEN)){
         return -1;
     }
     return checksum.sum;
 }
#endif

#if CTP_HYNITRON_EXT_CST78xx_UPDATE==1
/*****************************************************************/
// For CST0xxSE update

#define CST78xx_REG_LEN    2
 /*
  *
  */
static int cst78xx_enter_bootmode(void){
     char retryCnt = 10;

     cst0xx_reset();
     mdelay(5);
     while(retryCnt--){
         u8 cmd[3];
         cmd[0] = 0xAA;//cst716
		 //cmd[0] = 0xAB;//cst816S
         if (-1 == hctp_write_bytes(0xA001,cmd,1,CST78xx_REG_LEN)){  // enter program mode
             mdelay(2); // 4ms
             continue;                   
         }
         if (-1 == hctp_read_bytes(0xA003,cmd,1,CST78xx_REG_LEN)) { // read flag
             mdelay(2); // 4ms
             continue;                           
         }else{
             if (cmd[0] != 0x55){
                 mdelay(2); // 4ms
                 continue;
             }else{
                 return 0;
             }
         }
     }
     return -1;
 }
 /*
  *
  */
static int cst78xx_update(u16 startAddr,u16 len,u8* src){
     u16 sum_len;
     u8 cmd[10];

     if (cst78xx_enter_bootmode() == -1){
        return -1;
     }
     sum_len = 0;
 
#define PER_LEN	512
     do{
         if (sum_len >= len){
             return 0;
         }
         
         // send address
         cmd[1] = startAddr>>8;
         cmd[0] = startAddr&0xFF;
         hctp_write_bytes(0xA014,cmd,2,CST78xx_REG_LEN);
         
         hctp_write_bytes(0xA018,src,PER_LEN,CST78xx_REG_LEN);
 
         cmd[0] = 0xEE;
         hctp_write_bytes(0xA004,cmd,1,CST78xx_REG_LEN);
         mdelay(100);
 
         {
             u8 retrycnt = 50;
             while(retrycnt--){
                 cmd[0] = 0;
                 hctp_read_bytes(0xA005,cmd,1,CST78xx_REG_LEN);
                 if (cmd[0] == 0x55){
                     // success
                     break;
                 }
                 mdelay(10);
             }
         }
         startAddr += PER_LEN;
         src += PER_LEN;
         sum_len += PER_LEN;
     }while(len);
     
     // exit program mode
     cmd[0] = 0x00;
     hctp_write_bytes(0xA003,cmd,1,CST78xx_REG_LEN);

	 return 0;
 }
 /*
  *
  */
static u32 cst78xx_read_checksum(u16 startAddr,u16 len){
     union{
         u32 sum;
         u8 buf[4];
     }checksum;
     char cmd[3];
     char readback[4] = {0};
 
     if (cst78xx_enter_bootmode() == -1){
        return -1;
     }
     
     cmd[0] = 0;
     if (-1 == hctp_write_bytes(0xA003,cmd,1,CST78xx_REG_LEN)){
         return -1;
     }
     mdelay(500);
     
     if (-1 == hctp_read_bytes(0xA000,readback,1,CST78xx_REG_LEN)){
         return -1;   
     }

     if (-1 == hctp_read_bytes(0xA008,checksum.buf,4,CST78xx_REG_LEN)){
         return -1;
     }
     return checksum.sum;
}
#endif 


#if CTP_HYNITRON_EXT_CST816D_UPDATE==1
 /*****************************************************************/
 // For CST0xxSE update
 
  /*
   *
   */
 static int cst816D_enter_bootmode(void){
	  char retryCnt = 10;
 
	  cst0xx_reset();
	  mdelay(5);
	  while(retryCnt--){
		  u8 cmd[3];
		  cmd[0] = 0xAB; //cst816D
		  if (-1 == hctp_write_bytes(0xA001,cmd,1,2)){  // enter program mode
			  mdelay(2); // 4ms
			  continue; 				  
		  }
		  if (-1 == hctp_read_bytes(0xA003,cmd,1,2)) { // read flag
			  mdelay(2); // 4ms
			  continue; 						  
		  }else{
			  if (cmd[0] != 0xc1){
				  mdelay(2); // 4ms
				  continue;
			  }else{
				  return 0;
			  }
		  }
	  }
	  return -1;
  }
  /*
   *
   */
 static int cst816D_update(u16 startAddr,u16 len,u8* src){
	  u16 sum_len;
	  u8 cmd[10];
 
	  if (cst816D_enter_bootmode() == -1){
		 return -1;
	  }
	  sum_len = 0;
  
#define PER_LEN	512
	  do{
		  if (sum_len >= len){
			  return 0;
		  }
		  
		  // send address
		  cmd[1] = startAddr>>8;
		  cmd[0] = startAddr&0xFF;
		  hctp_write_bytes(0xA014,cmd,2,2);
		  
		  hctp_write_bytes(0xA018,src,PER_LEN,2);
  
		  cmd[0] = 0xEE;
		  hctp_write_bytes(0xA004,cmd,1,2);
		  mdelay(100);
  
		  {
			  u8 retrycnt = 50;
			  while(retrycnt--){
				  cmd[0] = 0;
				  hctp_read_bytes(0xA005,cmd,1,2);
				  if (cmd[0] == 0x55){
					  // success
					  break;
				  }
				  mdelay(10);
			  }
		  }
		  startAddr += PER_LEN;
		  src += PER_LEN;
		  sum_len += PER_LEN;
	  }while(len);
	  
	  // exit program mode
	  cmd[0] = 0x00;
	  hctp_write_bytes(0xA003,cmd,1,2);
 
	  return 0;
  }
  /*
   *
   */
 static u32 cst816D_read_checksum(u16 startAddr,u16 len){
	  union{
		  u32 sum;
		  u8 buf[4];
	  }checksum;
	  char cmd[3];
	  char readback[4] = {0};
  
	  if (cst816D_enter_bootmode() == -1){
		 return -1;
	  }
	  
	  cmd[0] = 0;
	  if (-1 == hctp_write_bytes(0xA003,cmd,1,2)){
		  return -1;
	  }
	  mdelay(500);
	  
	  if (-1 == hctp_read_bytes(0xA000,readback,1,2)){
		  return -1;   
	  }
 
	  if (-1 == hctp_read_bytes(0xA008,checksum.buf,4,2)){
		  return -1;
	  }
	  return checksum.sum;
 }
#endif 




 /*****************************************************************/
 // For CST0xx update
#if CTP_HYNITRON_EXT_CST0xx_UPDATE==1
#define CST0xx_REG_LEN  1
 /*
 ** Parameters: void
 ** Return Values: 0: enter cst0xx boot mode success -1:enter cst0xx boot mode fail
 */
static int cst0xx_enter_bootmode(void){
     char retryCnt;
     
    cst0xx_reset();
     for(retryCnt = 10; retryCnt > 0; retryCnt--){
         char enterBootData[] = {'0', '1', '2', '3'};
         char readBackData[4] = {0, 0, 0, 0};
         
         mdelay(2);
         hctp_write_bytes(0x00, enterBootData, sizeof(enterBootData),CST0xx_REG_LEN);
         mdelay(1);
         hctp_read_bytes(0x00, readBackData, 4,CST0xx_REG_LEN);
         
         if(memcmp(readBackData, "3210", 4) == 0){
             return 0;
         }
     }
     return -1;
 }
 /*
  *
  */
static void cst0xx_update(u16 startAddr, u16 length, u8 *data){
     const u8 perLen = 128;
     u8 curLen;
     
     u8 cmdReg;
     u8 cmdData[8];
     
     do{
         cmdReg = 0xF0;
         cmdData[0] = 0x96;
         cmdData[1] = startAddr/128;
         hctp_write_bytes(cmdReg, cmdData, 2,CST0xx_REG_LEN);
         mdelay(30);
         hctp_read_bytes(cmdReg, cmdData, 2,CST0xx_REG_LEN);
         
         curLen = perLen < length ? perLen : length;
         hctp_write_bytes(0x00, data, curLen,CST0xx_REG_LEN);
         
         cmdReg = 0xF0;
         cmdData[0] = 0xE1;
         cmdData[1] = 0x80|(curLen-1);
         cmdData[2] = startAddr;
         cmdData[3] = startAddr >> 8;
         cmdData[4] = 0;
         cmdData[5] = 0;
         hctp_write_bytes(cmdReg, cmdData, 6,CST0xx_REG_LEN);
         mdelay(30);
         hctp_read_bytes(cmdReg, cmdData, 2,CST0xx_REG_LEN);
         
         startAddr += curLen;
         length -= curLen;
         data += curLen;
         
     }while(length > 0);
 }
 /*
  *
  */
static u32 cst0xx_read_checksum(u16 startAddr,u16 length)
 {
     u16 checksum;
     u8 cmdReg;
     u8 cmdData[8];
     
     cmdReg = 0xF0;
     cmdData[0] = 0xD2;
     cmdData[1] = length/128-1;
     cmdData[2] = startAddr;
     cmdData[3] = startAddr >> 8;
     hctp_write_bytes(cmdReg, cmdData, 4,CST0xx_REG_LEN);
     mdelay(200);
     hctp_read_bytes(cmdReg, cmdData, 4,CST0xx_REG_LEN);
     
     checksum = cmdData[3];
     checksum <<= 8;
     checksum |= cmdData[2];
     return checksum;
 }
 #endif
/*****************************************************************/
// common

 /*
  *
  */
 int ctp_hynitron_update(struct i2c_client *mclient){
     client = mclient;
#if CTP_HYNITRON_EXT_CST0xxSE_UPDATE==1
{
    dev_addr = 0x6A;
    if (cst0xxse_enter_bootmode() == 0){ //CST0xxSE  //CST816S
#include "capacitive_hynitron_cst816s_update.h"
        if(sizeof(app_bin) > 10){
            u16 startAddr = app_bin[1];
            u16 length = app_bin[3];
            u16 checksum = app_bin[5];
            startAddr <<= 8; startAddr |= app_bin[0];
            length <<= 8; length |= app_bin[2];
            checksum <<= 8; checksum |= app_bin[4];   
            if(cst0xxse_read_checksum(startAddr, length) != checksum){
                cst0xxse_update(startAddr, length, (u8 *)(app_bin+6));
                cst0xxse_read_checksum(startAddr, length);
            }
        }
        return 0;
    }
}
#endif
#if CTP_HYNITRON_EXT_CST0xx_UPDATE==1
{
    dev_addr = 0x15;
    if (cst0xx_enter_bootmode() == 0){
#include "capacitive_hynitron_cst0xx_update.h"
        if(sizeof(app_bin) > 10){
            u16 startAddr = app_bin[1];
            u16 length = app_bin[3];
            u16 checksum = app_bin[5];
            startAddr <<= 8; startAddr |= app_bin[0];
            startAddr -= 0x1000;
            length <<= 8; length |= app_bin[2];
            checksum <<= 8; checksum |= app_bin[4];   
            if(cst0xx_read_checksum(startAddr, length) != checksum){
                cst0xx_update(startAddr, length, app_bin+6);
                cst0xx_read_checksum(startAddr, length);
            }
        }
        return 0;
    }
}
#endif
#if CTP_HYNITRON_EXT_CST78xx_UPDATE==1
{
    dev_addr = 0x6A;
    if (cst78xx_enter_bootmode() == 0){
#include "capacitive_hynitron_cst7xx_update.h"
        if(sizeof(app_bin) > 10){
            u16 startAddr = app_bin[1];
            u16 length = app_bin[3];
            u16 checksum = app_bin[5];
            startAddr <<= 8; startAddr |= app_bin[0];
            length <<= 8; length |= app_bin[2];
            checksum <<= 8; checksum |= app_bin[4];   
            if(cst78xx_read_checksum(startAddr, length) != checksum){
                cst78xx_update(startAddr, length, (u8 *)(app_bin+6));
                cst78xx_read_checksum(startAddr, length);
            }
        }
        return 0;
    }
}
   return -1;
#endif

#if CTP_HYNITRON_EXT_CST816D_UPDATE==1
{
    dev_addr = 0x6A;
    if (cst816D_enter_bootmode() == 0){
#include "capacitive_hynitron_cst816d_update.h"
        if(sizeof(app_bin) > 10){
            u16 startAddr = app_bin[1];
            u16 length = app_bin[3];
            u16 checksum = app_bin[5];
            startAddr <<= 8; startAddr |= app_bin[0];
            length <<= 8; length |= app_bin[2];
            checksum <<= 8; checksum |= app_bin[4];   
            if(cst816D_read_checksum(startAddr, length) != checksum){
                cst816D_update(startAddr, length, (u8 *)(app_bin+6));
                cst816D_read_checksum(startAddr, length);
            }
        }
        return 0;
    }
}
   return -1;
#endif


}

#endif  //CTP_HYNITRON_EXT==1

