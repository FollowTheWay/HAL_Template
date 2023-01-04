#include "ICM20602.h"
#include "USER_SPI.h"
#include "LED.h"

 
/**************************ʵ�ֺ���********************************************
*��		��:	  ʹ��/ʧ�� ICM20602
*��		�룺
					ena	   enaΪ1ʱʹ�ܣ�Ϊ0ʱʧ��
*��		����	��
*******************************************************************************/
static void icm20602_enable(uint8_t ena)
{
	if(ena)
		HAL_GPIO_WritePin(CS_ICM_GPIO_Port,CS_ICM_Pin,GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(CS_ICM_GPIO_Port,CS_ICM_Pin,GPIO_PIN_SET);
}

/**************************ʵ�ֺ���********************************************
*��		��:	  ��ȡָ�����ȵ����ݣ���λ�ֽڣ�
*��		�룺
					reg				�Ĵ�����ַ
					length		���ݳ���
					data			���ݴ洢��ַ
*��		����	��
*******************************************************************************/
static void icm20602_readbuf(uint8_t reg, uint8_t length, uint8_t *data)
{
	icm20602_enable(1);
	SPI1_RW(reg|0x80);
	SPI1_Read(data,length);
	icm20602_enable(0);
	
}

/**************************ʵ�ֺ���********************************************
*��		��:	  �޸�ָ���Ĵ�����ֵ
*��		�룺
					reg				�Ĵ�����ַ
					data			�޸Ĺ����ֵ
*��		����	״̬
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

/**************************ʵ�ֺ���********************************************
*��		��:	  �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
*��		�룺
				reg	   �Ĵ�����ַ
				bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
				data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
*��		����	��
*******************************************************************************/
static void icm20602_writeBit(uint8_t reg, uint8_t bitNum, uint8_t data) 
{
	uint8_t b;
	icm20602_readbuf(reg, 1, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	icm20602_writebyte(reg, b);
}

/**************************ʵ�ֺ���********************************************
*��		��:	  ��ʼ��icm�������״̬��
*��		�룺	��
*��		����	��
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

	/*��λreg*/
	icm20602_writebyte(MPU_RA_SIGNAL_PATH_RESET,0x03);
	HAL_Delay(10);
  /*��λreg*/
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
	/*���ٶȼ�LPF 10HZ*/
	icm20602_writebyte(0X1D,0x05);
	HAL_Delay(10);
	/*�رյ͹���*/
	icm20602_writebyte(0X1E,0x00);
	HAL_Delay(10);
	/*�ر�FIFO*/
	icm20602_writebyte(0X23,0x00);
	HAL_Delay(10);

	/*����������Դ�������ƫ����*/
	
	return 1;
}

uint8_t mpu_buffer[14];
_center_pos_st center_pos;
_sensor_st sensor;

/**************************ʵ�ֺ���********************************************
*��		��:	  ��ȡICM20602���ݣ����� mpu_buffer
*��		�룺	��
*��		����	��
*******************************************************************************/
void Icm20602_Read()
{
	icm20602_readbuf(MPUREG_ACCEL_XOUT_H,14,mpu_buffer);
}
