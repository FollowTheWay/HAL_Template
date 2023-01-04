#ifndef _USER_CAN_H_
#define _USER_CAN_H_

#include "main.h"

#define CAN1_Motor_NUM  5
#define CAN2_Motor_NUM  2

typedef struct
{
	short Angle;								//当前角度
	short Last_Angle;						//上一次的角度
	short Speed;								//当前速度
	short Last_Speed;						//上一次的速度
	short Given_Current;				//电机当前转矩电流
	uint8_t Temperate;					//电机温度
	short Circle;								//转动的圈数
	short Angle_Difference;			//角度差
	short Angle_Radian;					//角速度
	int   Continuous_Angle;			//连续角
	int   Last_Continuous_Angle;//上一次的连续角
	uint8_t    Flag;						//
}MotorBack;


void can_filter_init(void);
void CAN1_to_Motor(uint16_t id);
void CAN2_to_Motor(uint16_t id);
void CAN1_OPerate_Motor(uint8_t id,short i);
void CAN2_OPerate_Motor(uint8_t id,short i);

#endif

