#include "ICM20602.h"
#include "USER_SPI.h"
#include "LED.h"

 
/**************************实现函数********************************************
*功		能:	  使能/失能 ICM20602
*输		入：
					ena	   ena为1时使能，为0时失能
*输		出：	无
*******************************************************************************/
static void icm20602_enable(uint8_t ena)
{
	if(ena)
		HAL_GPIO_WritePin(CS_ICM_GPIO_Port,CS_ICM_Pin,GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(CS_ICM_GPIO_Port,CS_ICM_Pin,GPIO_PIN_SET);
}

/**************************实现函数********************************************
*功		能:	  读取指定长度的数据（单位字节）
*输		入：
					reg				寄存器地址
					length		数据长度
					data			数据存储地址
*输		出：	无
*******************************************************************************/
static void icm20602_readbuf(uint8_t reg, uint8_t length, uint8_t *data)
{
	icm20602_enable(1);
	SPI1_RW(reg|0x80);
	SPI1_Read(data,length);
	icm20602_enable(0);
	
}

/**************************实现函数********************************************
*功		能:	  修改指定寄存器的值
*输		入：
					reg				寄存器地址
					data			修改过后的值
*输		出：	状态
*******************************************************************************/
static uint8_t icm20602_writebyte(uint8_t reg, uint8_t data)
{
	uint8_t status;
	
	icm20602_enable(1);
	status = SPI1_RW(reg);
	SPI1_RW(data);
	icm20602_enable(0);
	return status;
}

/**************************实现函数********************************************
*功		能:	  读 修改 写 指定设备 指定寄存器一个字节 中的1个位
*输		入：
				reg	   寄存器地址
				bitNum  要修改目标字节的bitNum位
				data  为0 时，目标位将被清0 否则将被置位
*输		出：	无
*******************************************************************************/
static void icm20602_writeBit(uint8_t reg, uint8_t bitNum, uint8_t data) 
{
	uint8_t b;
	icm20602_readbuf(reg, 1, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	icm20602_writebyte(reg, b);
}

/**************************实现函数********************************************
*功		能:	  初始化icm进入可用状态。
*输		入：	无
*输		出：	无
*******************************************************************************/
uint8_t Drv_Icm20602Reg_Init(void)
{

	uint8_t tmp;
	
	icm20602_writebyte(MPU_RA_PWR_MGMT_1,0x80);
	HAL_Delay(10);
	icm20602_writebyte(MPU_RA_PWR_MGMT_1,0x01);
	HAL_Delay(10);
	
	icm20602_readbuf(MPUREG_WHOAMI, 1, &tmp);
	if(tmp != MPU_WHOAMI_20602)
	return 0;

	/*复位reg*/
	icm20602_writebyte(MPU_RA_SIGNAL_PATH_RESET,0x03);
	HAL_Delay(10);
  /*复位reg*/
	icm20602_writebyte(MPU_RA_USER_CTRL,0x01);	
	HAL_Delay(10);

	icm20602_writebyte(0x70,0x40);//dmp 
	HAL_Delay(10);
	icm20602_writebyte(MPU_RA_PWR_MGMT_2,0x00);
	HAL_Delay(10);
	icm20602_writebyte(MPU_RA_SMPLRT_DIV,0);
	HAL_Delay(10);

	icm20602_writebyte(MPU_RA_CONFIG,ICM20602_LPF_20HZ);
	HAL_Delay(10);
	icm20602_writebyte(MPU_RA_GYRO_CONFIG,(3 << 3));
	HAL_Delay(10);
	icm20602_writebyte(MPU_RA_ACCEL_CONFIG,(3 << 3));
	HAL_Delay(10);
	/*加速度计LPF 10HZ*/
	icm20602_writebyte(0X1D,0x05);
	HAL_Delay(10);
	/*关闭低功耗*/
	icm20602_writebyte(0X1E,0x00);
	HAL_Delay(10);
	/*关闭FIFO*/
	icm20602_writebyte(0X23,0x00);
	HAL_Delay(10);

	/*设置重心相对传感器的偏移量*/
	
	return 1;
}

uint8_t mpu_buffer[14];
_center_pos_st center_pos;
_sensor_st sensor;

/**************************实现函数********************************************
*功		能:	  读取ICM20602数据，存入 mpu_buffer
*输		入：	无
*输		出：	无
*******************************************************************************/
void Icm20602_Read()
{
	icm20602_readbuf(MPUREG_ACCEL_XOUT_H,14,mpu_buffer);
}
