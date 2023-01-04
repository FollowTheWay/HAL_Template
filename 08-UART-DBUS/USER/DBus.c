#include "DBus.h"
#include "usart.h"



extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];
REMOTE_CONTROL RC;


//开启遥控器接收
void remote_control_init(void)
{
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);  //接收中断
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);  //空闲中断
	set_remote_default();	//初始化遥控器参数
	HAL_UART_Receive_DMA(&huart2,sbus_rx_buf,SBUS_RX_BUF_NUM);//开始接收（使用DMA）
}

//遥控数据恢复默认
void set_remote_default(void)
{
	RC.l_s = 3;
	RC.l_x = 1024;
	RC.l_y = 1024;
	RC.r_s = 3;
	RC.r_x = 1024;
	RC.r_y = 1024;
	RC.m_x     = 0;
	RC.m_y     = 0;
	RC.m_z     = 0; 
	RC.press_l = 0;
	RC.press_r = 0;
	RC.key     = 0;
	RC.w       = 0;
	RC.s       = 0;
	RC.a       = 0;
	RC.d       = 0;
	RC.shift   = 0;
	RC.ctrl    = 0;
	RC.q       = 0;
	RC.e       = 0;
	RC.r			 = 0;
	RC.f			 = 0;
	RC.g			 = 0;
	RC.z			 = 0;
	RC.x			 = 0;
	RC.c			 = 0;
	RC.v			 = 0;
	RC.b			 = 0;
	RC.ch      = 0;
	RC.last_time = HAL_GetTick();
}


//遥控器检测
void check_remote(void)
{
	//如果数据出错，统一处理遥控器变量数据恢复默认
	if (RC.l_x > 1684 || RC.l_x < 364	||	//左摇杆x
			RC.l_y > 1684 || RC.l_y < 364	||	//左摇杆y
			RC.r_x > 1684 || RC.r_x < 364 ||	//右摇杆x
			RC.r_y > 1684 || RC.r_y < 364 ||	//右摇杆y
			RC.l_s == 0   || RC.r_s == 0)			//左右拨杆
	{
		set_remote_default();
	}
}

//遥控器数据解析
void dbus_to_rc(uint8_t* dbus_buffer)
{
	if (dbus_buffer == NULL)
		return;
	RC.last_time = HAL_GetTick();	//记录遥控器时间接收时间，用于超时重置
	RC.r_x     = (((uint16_t)dbus_buffer[1]<<8)|(uint16_t)dbus_buffer[0])&0x07FF;//0000011111111111
	RC.r_y     = (((uint16_t)dbus_buffer[1]>>3)|((uint16_t)dbus_buffer[2]<<5))&0x07FF;
	RC.l_x     = (((uint16_t)dbus_buffer[2]>>6)|((uint16_t)dbus_buffer[3]<<2)|((uint16_t)dbus_buffer[4]<<10))&0x07FF;
	RC.l_y     = (((uint16_t)dbus_buffer[4]>>1)|((uint16_t)dbus_buffer[5]<<7))&0x07FF;
	RC.r_s     = (dbus_buffer[5]>>4)&0x03;
	RC.l_s     = (dbus_buffer[5]>>6)&0x03;
	RC.m_x     = ((int16_t)dbus_buffer[6]) | ((int16_t)dbus_buffer[7] << 8);
	RC.m_y     = ((int16_t)dbus_buffer[8]) | ((int16_t)dbus_buffer[9] << 8);
	RC.m_z     = ((int16_t)dbus_buffer[10]) | ((int16_t)dbus_buffer[11] << 8);   
	RC.press_l = dbus_buffer[12];
	RC.press_r = dbus_buffer[13];
	RC.key     = ((uint16_t)dbus_buffer[14] | ((uint16_t)dbus_buffer[15] << 8));
	RC.w       =  RC.key&0x0001;
	RC.s       = (RC.key>>1)&0x0001;
	RC.a       = (RC.key>>2)&0x0001;
	RC.d       = (RC.key>>3)&0x0001;
	RC.shift   = (RC.key>>4)&0x0001;
	RC.ctrl    = (RC.key>>5)&0x0001;
	RC.q       = (RC.key>>6)&0x0001;
	RC.e       = (RC.key>>7)&0x0001;
	RC.r			 = (RC.key>>8)&0x0001;
	RC.f			 = (RC.key>>9)&0x0001;
	RC.g			 = (RC.key>>10)&0x0001;
	RC.z			 = (RC.key>>11)&0x0001;
	RC.x			 = (RC.key>>12)&0x0001;
	RC.c			 = (RC.key>>13)&0x0001;
	RC.v			 = (RC.key>>14)&0x0001;
	RC.b			 = (RC.key>>15)&0x0001;
	RC.ch			 = ((uint16_t)dbus_buffer[16] | ((uint16_t)dbus_buffer[17] << 8));
	check_remote();
}

//串口2中断服务函数
//需要把stm32f4xx_it里的屏蔽掉
void USART2_IRQHandler(void)
{
	volatile uint8_t receive;
	if(huart2.Instance->SR & UART_FLAG_RXNE)//接收到数据
	{
			__HAL_UART_CLEAR_PEFLAG(&huart2);
	}
	//空闲中断
	if(huart2.Instance->SR & UART_FLAG_IDLE)
	{
		receive = huart2.Instance->DR;//先读SR，再读DR可以清除空闲中断标志位
		HAL_UART_DMAStop(&huart2);	//先关闭DMA传输
		HAL_UART_Receive_DMA(&huart2,sbus_rx_buf,SBUS_RX_BUF_NUM);//重新开始接收（使用DMA）
		dbus_to_rc(sbus_rx_buf);
	}
}


