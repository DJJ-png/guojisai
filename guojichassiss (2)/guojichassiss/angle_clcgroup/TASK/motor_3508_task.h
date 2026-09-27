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



#define ROLLDISTANCE 501
#define ECHODISTANCE 328
#define ONEHIGH 221
#define THREEHIGH 501
#define TWOHIGH 341
#define ONEFLOOR 1000
#define TWOFLOOR 2000
#define THREEFLOOR 3000
#define GRAVITY  -600
#define EXTEND_DISTANCE 200
#define DELAYCOUNTMAX
#define motor1_3508 5
#define motor2_3508 6
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
}state_way;
extern struct motor_3508_t motor1,motor2,motorbo;


void motor_calcjiao();
void motor_3508_init();
void motor_state(uint8_t actionway);
void motor_updata();
void limit();

#endif                        

