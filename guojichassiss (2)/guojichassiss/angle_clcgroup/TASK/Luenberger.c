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
#include "Luenberger.h"
#include "road_calc.h"

extern osThreadId LUENBERGER_CALCHandle;
extern speed_f speed;
speed_f speed_l;
position_ecd position_0,position_1,position_2,position_3,position_g;
Luenberger_data Luenberger;
L_t L;

void is_onecycle(position_ecd *p ,uint16_t cnt){
		if(p->cnt_delta<-4096){
			p->cnt_realdelta=(8191.0f-p->cnt_p)+p->cnt_c;
			p->angle_delta=(p->cnt_realdelta/8191.0f)*360.0f;
		}
		if(p->cnt_delta>4096){
			p->cnt_realdelta=(8191.0f-p->cnt_c)+p->cnt_p;
			p->angle_delta=(p->cnt_realdelta/8191.0f)*360.0f;
		}
}

void position_ecd_calc(position_ecd *p , uint16_t cnt){
	p->cnt_c=get_chassis_motor_measure_point(cnt)->ecd;
	p->cnt_p=get_chassis_motor_measure_point(cnt)->last_ecd;
	p->cnt_delta=p->cnt_c-p->cnt_p;
	is_onecycle(p , cnt);
	p->wheel_lenth=2*3.1415926f*0.03825f/36.0f;
	p->lenth_delta=(p->angle_delta/360.0f)*p->wheel_lenth;
	p->p+=p->lenth_delta;
}

void wheel_calc(){
	position_ecd_calc( &position_0 , 0);
	position_ecd_calc( &position_1 , 1);
	position_ecd_calc( &position_2 , 2);
	position_ecd_calc( &position_3 , 3);
	position_g.p=0.25*position_0.p + 0.25*position_1.p + 0.25*position_2.p + 0.25*position_3.p;
}

void speed_convert(float v){
	v*=0.03825f;
	v*=3.1415926f;
	v/=30;
}

void speed_calc(speed_f *v , speed_f *v_l){
	v_l->speed_0 = v->speed_0;
	v_l->speed_1 = v->speed_1;
	v_l->speed_2 = v->speed_2;
	v_l->speed_3 = v->speed_3;
	speed_convert(v_l->speed_0);
	speed_convert(v_l->speed_1);
	speed_convert(v_l->speed_2);
	speed_convert(v_l->speed_3);
	v_l->speed_vechicle = 0.25*v_l->speed_0 + 0.25*v_l->speed_1 + 0.25*v_l->speed_2 + 0.25*v_l->speed_3;
}

void L_INIT(L_t *L){
	L->l1=0.382;
	L->l2=0.618;
}

void Luenberger_cal(Luenberger_data *l , position_ecd *p , L_t *L){
	L_INIT(L);
	wheel_calc();
	speed_calc(&speed , &speed_l);
	l->p_k_1=(1+0.001)*l->p_k;
	l->v_k_1=l->v_k;
	l->p_k_correct=l->p_k_1 + L->l1*(p->p-l->p_k_1);
	l->v_k_correct=l->v_k_1 + L->l2*(speed_l.speed_vechicle - l->v_k_1);
	l->p_k=l->p_k_correct;
	l->v_k=l->v_k_correct;
}

void Luenberger_calc(void const * argument){
	while(1){
		if(speed.speed_vechicle>30){
		}
		vTaskDelay(1);
	}
}