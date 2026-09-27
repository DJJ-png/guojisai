#ifndef GIMBAL_CALC__H
#define	GIMBAL_CALC__H
#include "pid.h"
#define edge 0.5f
typedef struct{
	uint16_t cnt;
	uint16_t last_cnt;
	float speed;
	float last_mechanic_angle;
	float mechanic_angle;
	float reality_angle;
	float angle_delta;
}data_t;

extern fp32 current_gimbal;
extern fp32 set_angle,set_speed;
#endif