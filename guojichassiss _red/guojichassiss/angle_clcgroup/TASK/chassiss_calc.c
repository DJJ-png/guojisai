#include "main.h"
#include "FreeRTos.h"
#include "cmsis_os.h"
#include "AHRS.h"
#include "chassiss_calc.h"
#include "math.h"
#include "CAN_receive.h"
#include "FSM_task.h"
#include "gimbal_calc.h"
#include "Vofa_send.h"
#define error 0
#define yaw_relat 0
#define CH_COUNT 2
#define RAD 0.0174533
#define pai 3.1415926
#define edge 1800
extern osThreadId CHASSISS_CALCHandle;
//extern UART_HandleTypeDef huart1;
speed_f speed;
pid_type_def motor_p,motor_s,motor_s_2,motor_s_3,motor_s_4;
current_f current;
motor_f motor;
extern state_gimbal_list gimbal_state;
extern state_chassiss_list chassiss_state;
extern fp32 INS_angle_deg[3];
extern fp32 set_angle_LST;
extern data_t data;
extern fp32 INS_angle_set[10];
eight_road channel[9];
extern road_f road_1,road_2,road_3,road_4;
fp32 error_deviate;
fp32 channel_read[8];
fp32 yaw=0;
fp32 set_yaw=180,yaw,yaw_cal,angle_relate,set_deg=180;
fp32 kp_s=700.0f,ki_s=0.1f,kd_s=0.0f;
fp32 kp_1_s=700.0f,ki_1_s=0.1f,kd_1_s=0.0f;
fp32 kp_2_s=1500.0f,ki_2_s=0.0f,kd_2_s=0.0f;
fp32 kp_3_s=950.0f,ki_3_s=0.1f,kd_3_s=1000.0f;
fp32 kp_4_s=900.0f,ki_4_s=0.2f,kd_4_s=1000.0f;
fp32 kp_p=3.0f,ki_p=0.0f,kd_p=100.0f;
float PID_s[3];
float PID_p[3]={5.0f,2.0f,2000.0f};
fp32 set_omega;
fp32 set_v,vx,vy,vx_1,vy_1;
fp32 rx=0.14195,ry=0.109,s=0.0765,r=0.03825f;//单位为米
fp32 set_vx_LST=20;
fp32 set_vy_LST=0;
fp32 set_vomega=100;
uint8_t rx_buf[8];

void road_calc(road_f *road[4],speed_f *speed);

float omega_calc(fp32 previous_yaw,fp32 now_yaw,fp32 time){//time为ms
        fp32 omega;
        if(now_yaw>previous_yaw){
            omega=(now_yaw-previous_yaw)/time*1000.0f;
            return omega;
        }
        else{
            return error;
        }
}

float yaw_calc(float yaw,float set_yaw){
	float delta=set_yaw-yaw;
	if(delta>180){
		delta-=360;
	}
	if(delta<-180){
		delta+=360;
	}
	return delta;
}

void PID_INIT(void){
        float PID1[3]={kp_s,ki_s,kd_s};
        float PID2[3]={kp_p,ki_p,kd_p};
				PID_init(&motor_s, PID_POSITION, PID1,1500,150);//PID初始化
				PID_init(&motor_p, PID_POSITION, PID2,1500,80);//PID初始化
}

float angle_calc(pid_type_def *pid_p,fp32 set_angle,fp32 angle){
		fp32 set_omega;
    PID_calc(pid_p,angle,set_angle);
    set_omega=pid_p->out;
    return set_omega;
}

void eight_receive(eight_road *channel,float channel_read[8]){
	for(int i=0;i<8;i++){
		channel[i].meast=channel_read[i];
	}
}

uint8_t is_black(uint16_t meast){
	if(meast>2000){
		return 1;//黑
	}
	else{
		return 0;//白
	}
}

void is_black_eight(eight_road *channel){
	for(int j=0;j<8;j++){
		channel[j].is_whiteorblack=is_black(channel[j].meast);
	}
}

void k_receive(eight_road *channel){
	channel[0].k=-0.2;
	channel[1].k=-0.3;
	channel[2].k=-0.4;
	channel[3].k=-1;
	channel[4].k=1;
	channel[5].k=0.4;
	channel[6].k=0.3;
	channel[7].k=0.2;
}

float k_calc(uint16_t meast,uint8_t k){
	uint16_t err=meast*k;
	return error;
}

float k_calc_eight(eight_road *channel){
	float error_gen;
	for(int i=0;i<8;i++){
		channel[i].err=k_calc(channel[i].meast,channel[i].k);
		error_gen+=channel[i].err;
	}
	error_gen/=8;
	if(error_gen>100){
		error_gen=100;
	}
	channel[8].err=error_gen;
}

void eight_calc(eight_road *channel){
	eight_receive(channel,channel_read);//读取八路灰度数据
	is_black_eight(channel);
	k_receive(channel);
	k_calc_eight(channel);
}

void pid_calc(motor_f*motor,fp32 vx,fp32 vy,fp32 omega){
		motor->motor_0=(-1*vx+vy+omega*(rx+ry))/s;
		motor->motor_1=(-1*vx+-1*vy+omega*(rx+ry))/s;
		motor->motor_2=(vx-vy+omega*(rx+ry))/s;
		motor->motor_3=(vx+vy+omega*(rx+ry))/s;
		
}

void current_calc(pid_type_def *pid , current_f *current , motor_f *motor , speed_f *speed){
	PID_calc(&motor_s , speed->speed_0 , motor->motor_0);
	current->current_0=motor_s.out;
	PID_calc(&motor_s , speed->speed_1 , motor->motor_1);
	current->current_1=motor_s.out;
	PID_calc(&motor_s , speed->speed_2 , motor->motor_2);
	current->current_2=motor_s.out;
	PID_calc(&motor_s , speed->speed_3 , motor->motor_3);
	current->current_3=motor_s.out;
}

void rpm_receive(speed_f *speed){
	speed->speed_0=get_chassis_motor_measure_point(0)->speed_rpm/36.0f;
	speed->speed_1=get_chassis_motor_measure_point(1)->speed_rpm/36.0f;
	speed->speed_2=get_chassis_motor_measure_point(2)->speed_rpm/36.0f;
	speed->speed_3=get_chassis_motor_measure_point(3)->speed_rpm/36.0f;
}

void PID_CROL(fp32 set_yaw){
	eight_calc(channel);
	rpm_receive(&speed);
	yaw=INS_angle_deg[0]+180;
	yaw_cal=yaw_calc(yaw,set_yaw);
	set_omega=angle_calc(&motor_p,yaw_relat,yaw_cal);  
	pid_calc(&motor,vx,vy,set_omega);
	current_calc(&motor_s,&current,&motor,&speed);
}

void chassiss_calc(void const * argument){
        PID_INIT();
	while(1){
//				Vofa_Send_Data4(set_deg,yaw,speed.speed_1,motor.motor_1 );
        PID_CROL(set_deg);
				vTaskDelay(1);
	}
        }