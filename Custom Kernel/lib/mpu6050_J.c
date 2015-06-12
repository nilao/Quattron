#include "i2c_J.c"

#define MPU6050_ADDRESS 0x68 // Address with end write bit
#define MPU6050_RA_XG_OFFS_TC 0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC 0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC 0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN 0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN 0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN 0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H 0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC 0x07
#define MPU6050_RA_YA_OFFS_H 0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC 0x09
#define MPU6050_RA_ZA_OFFS_H 0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC 0x0B
#define MPU6050_RA_XG_OFFS_USRH 0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL 0x14
#define MPU6050_RA_YG_OFFS_USRH 0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL 0x16
#define MPU6050_RA_ZG_OFFS_USRH 0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL 0x18
#define MPU6050_RA_SMPLRT_DIV 0x19
#define MPU6050_RA_CONFIG 0x1A
#define MPU6050_RA_GYRO_CONFIG 0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C
#define MPU6050_RA_FF_THR 0x1D
#define MPU6050_RA_FF_DUR 0x1E
#define MPU6050_RA_MOT_THR 0x1F
#define MPU6050_RA_MOT_DUR 0x20
#define MPU6050_RA_ZRMOT_THR 0x21
#define MPU6050_RA_ZRMOT_DUR 0x22
#define MPU6050_RA_FIFO_EN 0x23
#define MPU6050_RA_I2C_MST_CTRL 0x24
#define MPU6050_RA_I2C_SLV0_ADDR 0x25
#define MPU6050_RA_I2C_SLV0_REG 0x26
#define MPU6050_RA_I2C_SLV0_CTRL 0x27
#define MPU6050_RA_I2C_SLV1_ADDR 0x28
#define MPU6050_RA_I2C_SLV1_REG 0x29
#define MPU6050_RA_I2C_SLV1_CTRL 0x2A
#define MPU6050_RA_I2C_SLV2_ADDR 0x2B
#define MPU6050_RA_I2C_SLV2_REG 0x2C
#define MPU6050_RA_I2C_SLV2_CTRL 0x2D
#define MPU6050_RA_I2C_SLV3_ADDR 0x2E
#define MPU6050_RA_I2C_SLV3_REG 0x2F
#define MPU6050_RA_I2C_SLV3_CTRL 0x30
#define MPU6050_RA_I2C_SLV4_ADDR 0x31
#define MPU6050_RA_I2C_SLV4_REG 0x32
#define MPU6050_RA_I2C_SLV4_DO 0x33
#define MPU6050_RA_I2C_SLV4_CTRL 0x34
#define MPU6050_RA_I2C_SLV4_DI 0x35
#define MPU6050_RA_I2C_MST_STATUS 0x36
#define MPU6050_RA_INT_PIN_CFG 0x37
#define MPU6050_RA_INT_ENABLE 0x38
#define MPU6050_RA_DMP_INT_STATUS 0x39
#define MPU6050_RA_INT_STATUS 0x3A
#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_ACCEL_XOUT_L 0x3C
#define MPU6050_RA_ACCEL_YOUT_H 0x3D
#define MPU6050_RA_ACCEL_YOUT_L 0x3E
#define MPU6050_RA_ACCEL_ZOUT_H 0x3F
#define MPU6050_RA_ACCEL_ZOUT_L 0x40
#define MPU6050_RA_TEMP_OUT_H 0x41
#define MPU6050_RA_TEMP_OUT_L 0x42
#define MPU6050_RA_GYRO_XOUT_H 0x43
#define MPU6050_RA_GYRO_XOUT_L 0x44
#define MPU6050_RA_GYRO_YOUT_H 0x45
#define MPU6050_RA_GYRO_YOUT_L 0x46
#define MPU6050_RA_GYRO_ZOUT_H 0x47
#define MPU6050_RA_GYRO_ZOUT_L 0x48
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
#define MPU6050_RA_MOT_DETECT_STATUS 0x61
#define MPU6050_RA_I2C_SLV0_DO 0x63
#define MPU6050_RA_I2C_SLV1_DO 0x64
#define MPU6050_RA_I2C_SLV2_DO 0x65
#define MPU6050_RA_I2C_SLV3_DO 0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL 0x67
#define MPU6050_RA_SIGNAL_PATH_RESET 0x68
#define MPU6050_RA_MOT_DETECT_CTRL 0x69
#define MPU6050_RA_USER_CTRL 0x6A
#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_RA_PWR_MGMT_2 0x6C
#define MPU6050_RA_BANK_SEL 0x6D
#define MPU6050_RA_MEM_START_ADDR 0x6E
#define MPU6050_RA_MEM_R_W 0x6F
#define MPU6050_RA_DMP_CFG_1 0x70
#define MPU6050_RA_DMP_CFG_2 0x71
#define MPU6050_RA_FIFO_COUNTH 0x72
#define MPU6050_RA_FIFO_COUNTL 0x73
#define MPU6050_RA_FIFO_R_W 0x74
#define MPU6050_RA_WHO_AM_I 0x75



int mpu6050_read(mpu6050_struct_t* data )
{
	uint8_t buf[14];
	i2c_read(MPU6050_DEV, buf, 14, MPU6050_OFFSET);
	data->accX = ((int16_t)(buf[0]<<8|buf[1]));
	data->accY = ((int16_t)(buf[2]<<8|buf[3]));
	data->accZ = ((int16_t)(buf[4]<<8|buf[5]));
	data->temper = (buf[6]<<8|buf[7]);
	data->gyroX = ((int16_t)(buf[8]<<8|buf[9]));
	data->gyroY = ((int16_t)(buf[10]<<8|buf[11]));
	data->gyroZ = ((int16_t)(buf[12]<<8|buf[13]));
	return 0;
}

int device_init(int fd)
{
	//Sets sample rate to 8000/1+7 = 1000Hz
	i2c_simpleWrite(fd, MPU6050_RA_SMPLRT_DIV, 0x07);
	//Disable FSync, 256Hz DLPF
	i2c_simpleWrite(fd, MPU6050_RA_CONFIG, 0x00);

	//Disable gyro self tests, scale of 500 degrees/s
//	i2c_simpleWrite(fd, MPU6050_RA_GYRO_CONFIG, 0x08);
	//Disable accel self tests, scale of +-2g, no DHPF
//	i2c_simpleWrite(fd, MPU6050_RA_ACCEL_CONFIG, 0x00);

	//Disable gyro self tests, scale of 250 degrees/s
	i2c_simpleWrite(fd, MPU6050_RA_GYRO_CONFIG, 0x00);
	//Disable accel self tests, scale of +-2g, no DHPF
	i2c_simpleWrite(fd, MPU6050_RA_ACCEL_CONFIG, 0x00);


	//Freefall threshold of |0mg|
	i2c_simpleWrite(fd, MPU6050_RA_FF_THR, 0x00);
	//Freefall duration limit of 0
	i2c_simpleWrite(fd, MPU6050_RA_FF_DUR, 0x00);
	//Motion threshold of 0mg
	i2c_simpleWrite(fd, MPU6050_RA_MOT_THR, 0x00);
	//Motion duration of 0s
	i2c_simpleWrite(fd, MPU6050_RA_MOT_DUR, 0x00);
	//Zero motion threshold
	i2c_simpleWrite(fd, MPU6050_RA_ZRMOT_THR, 0x00);
	//Zero motion duration threshold
	i2c_simpleWrite(fd, MPU6050_RA_ZRMOT_DUR, 0x00);
	//Disable sensor output to FIFO buffer
	i2c_simpleWrite(fd, MPU6050_RA_FIFO_EN, 0x00);

	//AUX I2C setup
	//Sets AUX I2C to single master control, plus other config
	i2c_simpleWrite(fd, MPU6050_RA_I2C_MST_CTRL, 0x00);
	//Setup AUX I2C slaves
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV0_ADDR, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV0_REG, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV0_CTRL, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV1_ADDR, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV1_REG, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV1_CTRL, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV2_ADDR, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV2_REG, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV2_CTRL, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV3_ADDR, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV3_REG, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV3_CTRL, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV4_ADDR, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV4_REG, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV4_DO, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV4_CTRL, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV4_DI, 0x00);

	//MPU6050_RA_I2C_MST_STATUS //Read-only
	//Setup INT pin and AUX I2C pass through
	i2c_simpleWrite(fd, MPU6050_RA_INT_PIN_CFG, 0x00);
	//Enable data ready interrupt
	i2c_simpleWrite(fd, MPU6050_RA_INT_ENABLE, 0x00);

	//Slave out, dont care
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV0_DO, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV1_DO, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV2_DO, 0x00);
	i2c_simpleWrite(fd, MPU6050_RA_I2C_SLV3_DO, 0x00);
	//More slave config
	i2c_simpleWrite(fd, MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);
	//Reset sensor signal paths
	i2c_simpleWrite(fd, MPU6050_RA_SIGNAL_PATH_RESET, 0x00);
	//Motion detection control
	i2c_simpleWrite(fd, MPU6050_RA_MOT_DETECT_CTRL, 0x00);
	//Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
	i2c_simpleWrite(fd, MPU6050_RA_USER_CTRL, 0x00);
	//Sets clock source to gyro reference w/ PLL
	i2c_simpleWrite(fd, MPU6050_RA_PWR_MGMT_1, 0x00);
	//Controls frequency of wakeups in accel low power mode plus the sensor standby modes
	i2c_simpleWrite(fd, MPU6050_RA_PWR_MGMT_2, 0x00);
	//MPU6050_RA_BANK_SEL            //Not in datasheet
	//MPU6050_RA_MEM_START_ADDR        //Not in datasheet
	//MPU6050_RA_MEM_R_W            //Not in datasheet
	//MPU6050_RA_DMP_CFG_1            //Not in datasheet
	//MPU6050_RA_DMP_CFG_2            //Not in datasheet
	//MPU6050_RA_FIFO_COUNTH        //Read-only
	//MPU6050_RA_FIFO_COUNTL        //Read-only
	//Data transfer to and from the FIFO buffer
	i2c_simpleWrite(fd, MPU6050_RA_FIFO_R_W, 0x00);
	//MPU6050_RA_WHO_AM_I             //Read-only, I2C address
	return 0;
}

int mpu6050_init()
{
	int fd;
	i2c_open(&fd);
	i2c_setSlave(fd,MPU6050_DEV);
	device_init(fd);
	return 0;
}
