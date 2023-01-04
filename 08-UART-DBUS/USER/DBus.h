#ifndef _DBUS_H_
#define _DBUS_H_

#include "main.h"

#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u

typedef struct 
{
	uint16_t l_x;
	uint16_t l_y;
	uint16_t r_x;
	uint16_t r_y;
	
	uint8_t l_s;
	uint8_t r_s;
	
	int16_t m_x;
	int16_t m_y;
	int16_t m_z;
	uint8_t press_l;
	uint8_t press_r;
	
	uint16_t key;
	uint8_t w;
	uint8_t s;
	uint8_t a;
	uint8_t d;
	uint8_t shift;
	uint8_t ctrl;
	uint8_t q;
	uint8_t e;
	uint8_t r;
	uint8_t f;
	uint8_t g;
	uint8_t z;
	uint8_t x;
	uint8_t c;
	uint8_t v;
	uint8_t b;
	
	uint16_t ch;
	
	uint32_t last_time;
}REMOTE_CONTROL;

void remote_control_init(void);
void set_remote_default(void);
void check_remote(void);
void USART2_IRQHandler(void);
	
#endif

