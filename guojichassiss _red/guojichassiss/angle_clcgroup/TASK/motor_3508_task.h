#ifndef __MOTOR_3508_TASK_H__        
#define __MOTOR_3508_TASK_H__       

#include "CAN_receive.h"
#include "main.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"
#include "tim.h"
#include "usart.h"


#define ROLLDISTANCE 430
#define ECHODISTANCE 328
#define ONEHIGH 341
#define THREEHIGH 481
#define TWOHIGH 183
#define ONEFLOOR 221
#define TWOFLOOR 341
#define THREEFLOOR 461
#define GRAVITY  -1200
#define EXTEND_DISTANCE 200
#define DELAYCOUNTMAX
#define motor1_3508 5
#define motor2_3508 6
#define WAITTIME 50
struct motor_3508_t
{
	float jiaodu;
	float quan;
	float rmp;
	uint16_t ecn;
	uint16_t ecn_last;
	float distance;
	pid_type_def pid_s;
	pid_type_def pid_j;
	fp32 PID_S[3],PID_J[3];
	fp32 pid_su_out;
	fp32 pwm_out;
};

typedef enum
{
	STATE_STOP=0,
	STATE1_FLOOR,
	STATE1_HANG,
	STATE2_EXTEND,
	STATE2_LOCK,
	STATE2_BACK,
	STATE2_FLICK,
	STATE1_DOWN,
	STATE2_UNLOCK,
	STATE1_DOWN2,
	STATE1_HANG2,
	STATE_CHECK
}state_way;
extern struct motor_3508_t motor1,motor2,motorbo;


void motor_calcjiao();
void motor_3508_init();
void motor_state(uint8_t actionway);
void motor_updata();
void limit();

#endif                        
