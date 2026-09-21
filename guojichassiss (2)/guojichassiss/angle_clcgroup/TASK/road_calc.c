#include "main.h"
#include "FreeRTos.h"
#include "cmsis_os.h"
#include "AHRS.h"
#include "chassiss_calc.h"
#include "math.h"
#include "CAN_receive.h"
#include "FSM_task.h"
#include "gimbal_calc.h"
#include "CAN_receive.h"
#include "chassiss_calc.h"
#include "motor_3508_task.h"

#define pai 3.1415926
extern osThreadId ROAD_CALCHandle;
road_f road_1,road_2,road_3,road_4;

extern road_f road_1,road_2,road_3,road_4;
extern road_f road_plat[5];
road_f road_gene;
extern fp32 r;//单位为米
extern uint8_t road_calc_1,road_calc_2;
extern motor_measure_t motor_chassis[7];
extern state_way gimbal_state;
uint8_t count=0;

void motor_calcjiao()
{
    static float jiaodu1=0,jiaodu2=0,jiaodub,delta1,delta2,cleardelay;
    
    if(motor_chassis[motor1_3508].ecd==motor1.ecn_last&&gimbal_state==STATE_STOP)
    {
        cleardelay++;
        if(cleardelay>1000)
        {
            cleardelay=0;
            jiaodu1=0;
        }
        
    }
    else
    cleardelay=0;
    
    
    
    motor1.ecn=motor_chassis[motor1_3508].ecd;
    if(count++<2){
        motor1.ecn_last=motor1.ecn;
    }
    
    
    if((motor1.ecn-motor1.ecn_last)>4000)
    jiaodu1+=-360.0*(motor1.ecn-motor1.ecn_last-8191)/(1.0*8191*19);
    else if((motor1.ecn-motor1.ecn_last)<-4000)
    jiaodu1+=-360.0*(motor1.ecn-motor1.ecn_last+8191)/(1.0*8191*19);
    else
    jiaodu1+=-360.0*(motor1.ecn-motor1.ecn_last)/(1.0*8191*19);
    motor1.ecn_last=motor1.ecn;
     motor1.jiaodu=jiaodu1;
    
    motor2.ecn=motor_chassis[motor2_3508].ecd;
    if(count<2){
        motor2.ecn_last=motor2.ecn;
    }
    
    if((motor2.ecn-motor2.ecn_last)>4000)
    jiaodu2+=360.0*(motor2.ecn-8191-motor2.ecn_last)/(1.0*8191);
    else if((motor2.ecn-motor2.ecn_last)<-4000)
    jiaodu2+=360.0*(motor2.ecn+8191-motor2.ecn_last)/(1.0*8191);
    else
    jiaodu2+=360.0*(motor2.ecn-motor2.ecn_last)/(1.0*8191);
    motor2.ecn_last=motor2.ecn;
    motor2.jiaodu=jiaodu2;
    
    motorbo.ecn=motor_chassis[4].ecd;
    if(count<2){
        motorbo.ecn_last=motorbo.ecn;
    }
    
    
    if(motorbo.ecn-motorbo.ecn_last>4000)
    jiaodub+=-360.0*(motorbo.ecn-motorbo.ecn_last-8191)/(1.0*8191*36);
    else if((motorbo.ecn-motorbo.ecn_last)<-4000)
    jiaodub+=-360.0*(motorbo.ecn-motorbo.ecn_last+8191)/(1.0*8191*36);
    else
    jiaodub+=-360.0*(motorbo.ecn-motorbo.ecn_last)/(1.0*8191*36);
    motorbo.ecn_last=motorbo.ecn;
     motorbo.jiaodu=jiaodub*1.01;
}

void road_averg(road_f *road_1,road_f *road_2,road_f *road_3,road_f *road_4,road_f *road_gen){
	road_gen->road=road_1->road*0.25+road_2->road*0.25+road_3->road*0.25+road_4->road*0.25;
}

void is_one_cycle(road_f *road,uint16_t cnt){
		if(road->cnt_delta<-4096){
			road->times+=1;
		}
		if(road->cnt_delta>4096){
			road->times+=1;
		}
}

void Road_calc(road_f *road,uint16_t cnt){
	road->per_lenth=2*pai*r;
	road->cnt_current=get_chassis_motor_measure_point(cnt)->ecd;	
	road->cnt_past=get_chassis_motor_measure_point(cnt)->last_ecd;
	road->cnt_delta=road->cnt_current-road->cnt_past;
	is_one_cycle(road,cnt);
	road->road=(road->times*road->per_lenth)/36.0f;
}

void road_init(road_f *road_1 , road_f *road_2 , road_f *road_3 , road_f *road_4 , road_f *road_gene){
road_averg(road_1 , road_2 , road_3 , road_4 , road_gene);
	Road_calc(road_1,0);
	Road_calc(road_2,1);
	Road_calc(road_3,2);
	Road_calc(road_4,3);
}

void road_calc(void const * argument){
while(1){
	motor_calcjiao();
	if(road_calc_1){
	road_init(&road_1 , &road_2 , &road_3 , &road_4 , &road_gene);
	}
	if(road_calc_2){
	road_init(&road_plat[0] , &road_plat[1] , &road_plat[2] , &road_plat[3] , &road_plat[4]);
	}
	vTaskDelay(1);
}
}