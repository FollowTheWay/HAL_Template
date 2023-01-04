#ifndef _USER_CAN_H_
#define _USER_CAN_H_

#include "main.h"

#define CAN1_Motor_NUM  5
#define CAN2_Motor_NUM  2

typedef struct
{
	short Angle;								//��ǰ�Ƕ�
	short Last_Angle;						//��һ�εĽǶ�
	short Speed;								//��ǰ�ٶ�
	short Last_Speed;						//��һ�ε��ٶ�
	short Given_Current;				//�����ǰת�ص���
	uint8_t Temperate;					//����¶�
	short Circle;								//ת����Ȧ��
	short Angle_Difference;			//�ǶȲ�
	short Angle_Radian;					//���ٶ�
	int   Continuous_Angle;			//������
	int   Last_Continuous_Angle;//��һ�ε�������
	uint8_t    Flag;						//
}MotorBack;


void can_filter_init(void);
void CAN1_to_Motor(uint16_t id);
void CAN2_to_Motor(uint16_t id);
void CAN1_OPerate_Motor(uint8_t id,short i);
void CAN2_OPerate_Motor(uint8_t id,short i);

#endif

