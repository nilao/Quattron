#include "i2c_J.h"
#define CONFIG_I2C
#include <linux/i2c.h>
#include <linux/i2c-dev.h>



int i2c_open(int* fd){
if((fd_sensor=open(I2C_DEV_DRIVER,O_RDWR))<0)
{
	perror("i2c_open");
	return -1;
}
return 0;
}


int i2c_setSlave(int fd,int dev_addr)
{
	if((ioctl(fd_sensor,I2C_SLAVE,dev_addr))<0){
		perror("i2c_ioctl_slave");
return -1;
	}
return 0;
}

int i2c_simpleWrite(int fd, uint8_t address, uint8_t value)
{
	uint8_t data[2];
	data[0]=address;
	data[1]=value;
	if(write(fd_sensor,data,2)<0){
		perror("Setting error occured");
		return -1;
	}
return 0;
}

//return size+1;
int i2c_write(int dev_addr, unsigned char *data, int size, int offsetAddr)
{
	int ret;
	struct i2c_rdwr_ioctl_data i2c_data;
	struct i2c_msg msg[1];
	unsigned char *buf;
	/*  if (size > MAX_I2C_DATA)
		size = MAX_I2C_DATA;
	 */		//16byte제한 루틴 제거
	buf = malloc(size+1);
	buf[0] = offsetAddr;
	memcpy(&buf[1], data, size);
	/*
	 * write operation
	 */
	i2c_data.msgs = msg;
	i2c_data.nmsgs = 1;             // use one message structure
	i2c_data.msgs[0].addr = dev_addr;
	i2c_data.msgs[0].flags = 0;     // don't need flags
	i2c_data.msgs[0].len = size+1;
	i2c_data.msgs[0].buf = (__u8 *)buf;
	/*
	 * ioctl() processes read & write.
	 * Operation is determined by flags field of i2c_msg
	 */
	if((ret = ioctl(fd_sensor, I2C_RDWR, &i2c_data)) < 0) {
		perror("write data fail\n");
		return ret;
	}
	return size+1;
}

//return size
int i2c_read(int dev_addr, char *data, int size, int offsetAddr)
{
	int ret;
	struct i2c_rdwr_ioctl_data i2c_data;
	struct i2c_msg msg[2];
	char tmp[2];

	/*
	 * Two operation is necessary for read operation.
	 * First determine offset address
	 * , and then receive data.
	 */
	i2c_data.msgs = msg;
	i2c_data.nmsgs = 2;     // two i2c_msg

	tmp[0] = offsetAddr;
	i2c_data.msgs[0].addr = dev_addr;
	i2c_data.msgs[0].flags = 0;     // write
	i2c_data.msgs[0].len = 1;       // only one byte
	i2c_data.msgs[0].buf = (__u8 *)tmp;

	/*
	 * Second, read data from the EEPROM
	 */
	i2c_data.msgs[1].addr = dev_addr;
	i2c_data.msgs[1].flags = I2C_M_RD;      // read command
	i2c_data.msgs[1].len = size;
	i2c_data.msgs[1].buf = data;

	ret = ioctl(fd_sensor, I2C_RDWR, &i2c_data);

	if (ret < 0) {
		perror("read data fail\n");
		return ret;
	}

	return size;

}


