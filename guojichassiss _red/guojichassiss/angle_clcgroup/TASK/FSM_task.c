#include "main.h"
#include "FreeRTos.h"
#include "cmsis_os.h"
#include "AHRS.h"
#include "FSM_task.h"
#include "bsp_can.h"
#include "remote_control.h"
#include "chassiss_calc.h"
#include "INS_Task.h"
#include "gimbal_calc.h"
#include "CAN_receive.h"
#include "pid.h"
#include "road_calc.h"
#include "motor_3508_task.h"
#include "math.h"
#define pai 3.1415926
#define v_f 0.05
#define v_s 0.03
#define v_s_1 0.01
#define plat_len_same 0.095f
#define plat_len_dif  0.10f
#define delta_rec 0.01f
#define delta_rec_1 0.03f
#define delta_rec_2 0.003f
#define storage_delta 0.15f
#define L 196
#define cor_ditance 0.02f
#define cor_ditance_1 0.1f
extern osThreadId FSM_TASKHandle;
state_chassiss_list chassiss_state;
state_road_list road_state;
platform p_state;
storage storage_state;
road_f road_plat[5],road_cor[5];

extern uint8_t re_buf[4];
extern pid_type_def motor_s,motor_p;
extern fp32 set_yaw , yaw;
extern fp32 INS_angle_deg[3];
extern fp32 set_angle;
extern speed_f speed;
extern fp32 r;
extern speed_f speed;
extern fp32 set_v,vx,vy,vx_1,vy_1,set_omega;
extern road_f road_1,road_2,road_3,road_4,road_gene;
extern fp32 set_deg;
extern uint8_t event,moveit,moveok,flickit;
extern fp32 delta;
extern uint16_t distance[2];
fp32 z;
fp32 delta_1[10],delta_2[10];
motor_f motor_2;
fp32 set_angle_LST;
uint8_t road_calc_1=1,road_calc_2=0,road_calc_3=0,rotate_2=0,x,x_delta,target,v_pla=1,rotate,rotate_1 , v_state ,v_state_1;//x,x_delta,target(立仓)
uint32_t cnt,last_cnt,cnt_con;
angle_f angle_secondmode;
angle_f angle_thirdmode;
fp32 distance_1[2];
fp32 INS_angle_set[11];
fp32 cnt_set[4][11]={0},count_1[10],he_cnt[3][10];
fp32 road_len[10],len_add,len_add_1;
fp32 plat_len[10];
fp32 v_ca;
fp32 count_cor,count_cor_1,count_cor_2;
fp32 ang;
	fp32 flag_1=0 , flag_2=0 , flag_3=0;
fp32 time_cnt,cnt_x;

void CHASSISS_PREPARE_SELF_CHECK(void);
void RC_chassiss_crol(void);
void GIMBAL_PREPARE_SELF_CHECK(void);
void RC_GIMBAL_AUTO(void);
void gimbal_stand_AUTO_correct(void);
void road_init(road_f *road_1 , road_f *road_2 , road_f *road_3 , road_f *road_4 , road_f *road_gene);
void FSM_begin(void);
void plat_exchange(road_f *road_gen);
void road_clear(road_f *road_1 , road_f *road_2 , road_f *road_3 , road_f *road_4 , road_f *road_gene);
void storage_exchange(road_f *road_gen , storage s , uint8_t x);

void chassiss_state_judge(state_chassiss_list state){
	switch(state){
		case CHASSISS_PREPARE:
				CHASSISS_PREPARE_SELF_CHECK();
				FSM_begin();
		break;
		case CHASSISS_ENERGEGIVE_RC:
				CHASSISS_PREPARE_SELF_CHECK();
				
		break;
		case CHASSISS_AUTOAIM:
			
		break;
		case CHASSISS_ENERGYLOSE:
				CHASSISS_PREPARE_SELF_CHECK();
				CAN_CMD_BASE(&hcan1,0x200,0,0,0,0);
				CAN_CMD_BASE(&hcan1,0x1FF,0,0,0,0);
				
	}
}

void CHASSISS_PREPARE_SELF_CHECK(void){
	if(re_buf[2]==0x46){
			chassiss_state=0;
	}
	if(re_buf[2]==0x45){
			chassiss_state=3;
	}
}

float v_calc(fp32 v){
		v*=30;
		v/=pai;
		v/=r;
		return v;
}

void stop_secend(){
	vx=0; vy=0; set_omega=0;
	HAL_Delay(200);
}

void stop_secend_1(){
	vx=0; vy=0;
	HAL_Delay(3000);
}
////方向     x^
////          |
////          |
////          |
////          |_____________>y

void v_x_front(fp32 v){//电池为屁股
	vx=v;
	vy=0;
	set_omega=0;
	vx=v_calc(vx);
}

void v_x_behind(fp32 v){//电池为屁股
	vx=-v;
	vy=0;
	set_omega=0;
	vx=v_calc(vx);
}

void v_y_front(fp32 v){//电池为屁股
	vx=0;
	vy=v;
	set_omega=0;
	vy=v_calc(vy);
}

void v_y_behind(fp32 v){//电池为屁股
	vx=0;
	vy=-v;
	set_omega=0;
	vy=v_calc(vy);
}

void v_init(){
	vx=0;
	vy=0;
	set_omega=0;
}

void road_len_init(){
	road_len[0] = 0.870;
	road_len[1] = 1.98f;
	road_len[2] = 1.5f;
	road_len[3] = 1.44f;
	road_len[4] = 1.43f;
	road_len[5] = 1.8f;
	road_len[6] = 1.1f;
	road_len[7] = 1.6f;
	road_len[8] = 2.0f;
	road_len[9] = 1.0f;
}

void plat_len_init(){
	plat_len[0] = plat_len_same;
	plat_len[1] = plat_len_dif;
	plat_len[2] = plat_len_same;
	plat_len[3] =	plat_len_same;
	plat_len[4] = plat_len_same;
	plat_len[5] = plat_len_dif;
	plat_len[6] = plat_len_same;
}	

float rotate_calc(fp32 set_ang , fp32 yaw_current , fp32 delta , fp32 vx , fp32 vy ){
	vx=0; vy=0; road_calc_1=0; road_calc_2=0;
	if(rotate==0){
	set_ang = yaw_current + delta;
	rotate=1;
	}
	if(set_ang>360){set_ang-=360;}
	if(set_ang<0){set_ang+=360;}
	return set_ang;
}

void speed_cal(speed_f *speed){
	speed->speed_vechicle=0.25*speed->speed_0 + 0.25*speed->speed_1 + 0.25*speed->speed_2 + 0.25*speed->speed_3;
}
		
float straight_calc(road_f *road , fp32 target , fp32 v_max , fp32 edg , fp32 cnt ){// road_f *road_1 , road_f *road_2 , road_f *road_3 , road_f *road_4 , road_f *road_gene
	speed_cal(&speed);
	fp32 k;
	fp32 v_calc;
	if(fabs(speed.speed_vechicle-0)>2)
	time_cnt=0;
	cnt_x=1;
	if(cnt == 0){
	if(road->road > target){ k=0; }
	if(road->road < edg){ 
		k=road->road / edg;
		v_calc =k * v_max; 
	}
	else if(road->road > target - edg){
		k = target - road->road;
		k /= edg;
		v_calc = k * v_max;
	}
	else{
		v_calc = v_max;
	}
	if(k>1){k=1;}
	if(k<-1){k=-1;}
	if(v_calc>v_max){v_calc=v_max;}
	z=k;
}
	
	else{v_calc=0;}

	if(fabs(speed.speed_vechicle-0)<2 && road->road < edg ){
		time_cnt++;
		v_calc+=time_cnt*0.0005*v_max;
		if(v_calc>v_max){v_calc= v_max;}
	}
	return v_calc;
}
	






////////////第一次校准

float delta_ang_calc(uint16_t distance[2] , fp32 set_ang ){
	
	if(fabs(distance[0] - distance[1])>4 && v_state==0){
	fp32 deltaL=distance[1] - distance[0];
	fp32 k=deltaL/L;
	fp32 theta=atanf(k);
	ang=theta*180.0f/pai;
	if(deltaL > 0){set_ang += ang;}
	if(deltaL < 0){set_ang += ang;}
	if(set_ang<0){set_ang+=360;}
	if(set_ang>360){set_ang-=360;}
	v_state=1;
	return set_ang;
}
}
		

void ditance_calc(uint16_t distance[2] , fp32 vy , fp32 target , fp32 edg){
	if(v_state==1){
	if(fabs(distance[0] - target) > edg){
		if(distance[0]>target){
			vy=-v_s_1;
			v_y_front(vy);
		}
		if(distance[0]<target){vy=v_s_1;v_y_front(vy);}
	}
	else{vy=0; v_state=2;}
	v_y_front(vy);
}
	else{vy=0;v_y_front(vy);}
}		

void cor_judge(road_f *road , fp32 target , fp32 cnt){
	if(road->road >= target && count_cor==0 && p_state==0){
		count_cor=1;
		road_calc_2=1;
		road_calc_3=0;
		road_clear(&road_cor[0] , &road_cor[1] , &road_cor[2] , &road_cor[3] , &road_cor[4]);
		flag_1=0;
		v_state=0;
		stop_secend();
		p_state=2;
	}
	
}

float cor_road(road_f *road , uint16_t distance_behind , uint16_t distance_front , fp32 vx , fp32 target){
	if(distance_behind < 150){
		vx=-0.01f;
		v_x_front(vx);
	}
	if(distance_behind > 250){
		vx=0;
		v_x_front(vx);
		flag_1=1;
	}
	if(flag_1){
		road_calc_2=0;
		road_calc_1=0;
		road_calc_3=1;
		cor_judge(road , target , count_cor);
		v_ca = straight_calc(road , target , v_s_1 , 0.004f , count_cor);
		v_x_front(-v_ca);
	}
}






///////////////二次检测
float delta_ang_calc_1(uint16_t distance[2] , fp32 set_ang ){
	if(fabs(distance[0] - distance[1])>4 && v_state==1){
	fp32 deltaL=distance[1] - distance[0];
	fp32 k=deltaL/L;
	fp32 theta=atanf(k);
	ang=theta*180.0f/pai;
	if(deltaL > 0){set_ang += ang;}
	if(deltaL < 0){set_ang += ang;}
	if(set_ang<0){set_ang+=360;}
	if(set_ang>360){set_ang-=360;}
	v_state=2;
	return set_ang;
}
}

void ditance_calc_1(uint16_t distance[2] , fp32 vy , fp32 target , fp32 edg){
	if(v_state==2 && flag_1==0){
	if(fabs(distance[0] - target) > edg){
		if(distance[0]>target){
			vy=-v_s_1;
			v_y_front(vy);
		}
		if(distance[0]<target){vy=v_s_1;v_y_front(vy);}
	}
	else{vy=0; flag_1=1;v_state=0;}
	v_y_front(vy);
}
else{vy=0;v_y_front(vy);stop_secend();}
}		

float cor_road_1(road_f *road , uint16_t distance_behind , uint16_t distance_front , fp32 vx , fp32 target){
	if(v_state==0){
	if(distance_front > 250){
		vx=-0.01f;
		v_x_front(vx);
	}
	if(distance_front < 150){
		vx=0;
		v_x_front(vx);
		v_state=1;
	}
}
}

////////////第三次校准

float delta_ang_calc_2(uint16_t distance[2] , fp32 set_ang ){
	
	if(fabs(distance[0] - distance[1])>4 && v_state==0){
	fp32 deltaL=distance[1] - distance[0];
	fp32 k=deltaL/L;
	fp32 theta=atanf(k);
	ang=theta*180.0f/pai;
	if(deltaL > 0){set_ang += ang;}
	if(deltaL < 0){set_ang += ang;}
	if(set_ang<0){set_ang+=360;}
	if(set_ang>360){set_ang-=360;}
	v_state=1;
	return set_ang;
}
}
		

void ditance_calc_2(uint16_t distance[2] , fp32 vy , fp32 target , fp32 edg){
	if(v_state==1){
	if(fabs(distance[0] - target) > edg){
		if(distance[0]>target){
			vy=-v_s_1;
			v_y_front(vy);
		}
		if(distance[0]<target){vy=v_s_1;v_y_front(vy);}
	}
	else{vy=0; v_state=2;}
	v_y_front(vy);
}
	else{vy=0;v_y_front(vy);}
}		

void cor_judge_2(road_f *road , fp32 target , fp32 cnt){
	if(road->road >= target && count_cor_2==0 ){
		flag_3=1;
		count_cor_2=1;
		road_calc_1=1;
		road_calc_3=0;
		road_clear(&road_cor[0] , &road_cor[1] , &road_cor[2] , &road_cor[3] , &road_cor[4]);
		flag_1=0;
		v_state=0;
		stop_secend();
		p_state=2;
	}
	
}

float cor_road_2(road_f *road , uint16_t distance_behind , uint16_t distance_front , fp32 vx , fp32 target){
	if(distance_behind < 150){
		vx=-0.01f;
		v_x_front(vx);
	}
	if(distance_behind > 250){
		vx=0;
		v_x_front(vx);
		flag_1=1;
	}
	if(flag_1){
		road_calc_2=0;
		road_calc_1=0;
		road_calc_3=1;
		cor_judge(road , target , count_cor_2);
		v_ca = straight_calc(road , target , v_s_1 , 0.004f , count_cor_2);
		v_x_front(v_ca);
	}
}

///////////////四次检测
float delta_ang_calc_3(uint16_t distance[2] , fp32 set_ang ){
	if(fabs(distance[0] - distance[1])>4 && v_state==1){
	fp32 deltaL=distance[1] - distance[0];
	fp32 k=deltaL/L;
	fp32 theta=atanf(k);
	ang=theta*180.0f/pai;
	if(deltaL > 0){set_ang += ang;}
	if(deltaL < 0){set_ang += ang;}
	if(set_ang<0){set_ang+=360;}
	if(set_ang>360){set_ang-=360;}
	v_state=2;
	return set_ang;
}
}

void ditance_calc_3(uint16_t distance[2] , fp32 vy , fp32 target , fp32 edg){
	if(v_state==2 && flag_1==0){
	if(fabs(distance[0] - target) > edg){
		if(distance[0]>target){
			vy=-v_s_1;
			v_y_front(vy);
		}
		if(distance[0]<target){vy=v_s_1;v_y_front(vy);}
	}
	else{vy=0; flag_1=1;v_state=0;}
	v_y_front(vy);
}
else{vy=0;v_y_front(vy);stop_secend();}
}		

float cor_road_3(road_f *road , uint16_t distance_behind , uint16_t distance_front , fp32 vx , fp32 target){
	if(v_state==0){
	if(distance_front < 150){
		vx=0.01f;
		v_x_front(vx);
	}
	if(distance_front > 250){
		vx=0;
		v_x_front(vx);
		v_state=1;
	}
}
}


///////////检测函数

void cor_calc(){
		road_calc_2=0;
		road_calc_1=0;
	if(v_state==0){
	set_deg=delta_ang_calc(distance , set_deg);
	}
	if(v_state){
	ditance_calc(distance , vy , 110 , 2);
	if(v_state==2){
	cor_road(&road_cor[4] , distance[1] , distance[0] , vx , cor_ditance);
	}
	}
}

void cor_calc_1(){
		road_calc_2=0;
		road_calc_1=0;
		
	if(v_state==0){
		cor_road_1(&road_cor[4] , distance[1] , distance[0] , vx , cor_ditance);
	}
	if(v_state){
		set_deg=delta_ang_calc_1(distance , set_deg);
		if(v_state==2){
			ditance_calc_1(distance , vy , 110 , 2);
		}
	}
}

void cor_calc_2(){
		road_calc_2=0;
		road_calc_1=0;
	if(v_state==0){
	set_deg=delta_ang_calc(distance , set_deg);
	}
	if(v_state){
	ditance_calc(distance , vy , 110 , 2);
	if(v_state==2){
	cor_road(&road_cor[4] , distance[1] , distance[0] , vx , cor_ditance_1);
	}
	}
}

void cor_calc_4(){
		road_calc_2=0;
		road_calc_1=0;
		
	if(v_state==0){
		cor_road_3(&road_cor[4] , distance[1] , distance[0] , vx , cor_ditance_1);
	}
	if(v_state){
		set_deg=delta_ang_calc_3(distance , set_deg);
		if(v_state==2){
			ditance_calc_3(distance , vy , 110 , 2);
		}
	}
}

		///////////////////阶梯平台状态机
void plat_state_judge(platform p_state){
			switch(p_state){
				case P_STATE_0:
					cor_calc();
					break;
				case P_STATE_1:
					
					break;
				case P_STATE_2:
					if(he_cnt[0][0]==0){
					event=3;
					moveok=1;
					he_cnt[0][0]=1;
					}
					if(moveit ){
					v_ca = straight_calc(&road_plat[4] , plat_len[0] , v_s_1 , 0.03f , cnt_set[1][0]);
					v_x_front(v_ca);
						
					}
					break;
				case P_STATE_3:
					if(he_cnt[0][1]==0){
					moveok=1;
					he_cnt[0][1]=1;
					}
//					if(v_state==0 && v_state_1==0){
//						set_deg=delta_ang_calc(distance_1 , set_deg);
//						v_state_1=1;
//					}
					if(moveit ){ //&& v_state_1
					v_ca = straight_calc(&road_plat[4] , plat_len[1] , v_s_1 , 0.03f , cnt_set[1][1]);
					v_x_front(v_ca);
					}
					break;
				case P_STATE_4:
					if(he_cnt[0][2]==0){
					moveok=1;
					he_cnt[0][2]=1;
					}
					if(moveit){
					v_ca = straight_calc(&road_plat[4] , plat_len[2] , v_s_1 , 0.03f , cnt_set[1][2]);
					v_x_front(v_ca);
					}
					break;
				case P_STATE_5:
					if(he_cnt[0][3]==0){
					moveok=1;
					he_cnt[0][3]=1;
					}
					if(moveit){
					v_ca = straight_calc(&road_plat[4] , plat_len[3] , v_s_1 , 0.03f , cnt_set[1][3]);
					v_x_front(v_ca);
					}
					break;
				case P_STATE_6:
					if(he_cnt[0][4]==0){
					moveok=1;
					he_cnt[0][4]=1;
					}
					if(moveit){
					v_ca = straight_calc(&road_plat[4] , plat_len[4] , v_s_1 , 0.03f , cnt_set[1][4]);
					v_x_front(v_ca);
					}
					break;
				case P_STATE_7:
					if(he_cnt[0][5]==0){
					moveok=1;
					he_cnt[0][5]=1;
					}
					if(moveit){
					v_ca = straight_calc(&road_plat[4] , plat_len[5] , v_s_1 , 0.03f , cnt_set[1][5]);
					v_x_front(v_ca);
					}	
					break;
				case P_STATE_8:
					if(he_cnt[0][6]==0){
					moveok=1;
					he_cnt[0][6]=1;
					}
					if(moveit){
					v_ca = straight_calc(&road_plat[4] , plat_len[6] , v_s_1 , 0.03f , cnt_set[1][6]);
					v_x_front(v_ca);
					}
					break;
				case P_STATE_9:
					v_init();
				if(he_cnt[0][7]==0){
					moveok=1;
					he_cnt[0][7]=1;
					}
					break;
				case P_STATE_10:
					if(1)
					v_init();
					break;
				case P_STATE_11:
					if(0)
					v_x_front(0);
					break;
			}
		}

		
		
		
		
		
		//////立仓状态机
//void storage_state_judge(storage s){
//	switch(s){
//		case S_STATE_0:
//			storage_exchange(&road_plat[4] , s , x);
//		break;
//		case S_STATE_1:
//			storage_exchange(&road_plat[4] , s , x);
//		break;
//		case S_STATE_2:
//			storage_exchange(&road_plat[4] , s , x);
//		break;
//		case S_STATE_3:
//			storage_exchange(&road_plat[4] , s , x);
//		break;
//		case S_STATE_4:
//			
//		break;
//		case S_STATE_5:
//			
//		break;
//		case S_STATE_6:
//			v_init();
//			road_clear(&road_plat[0] , &road_plat[1] , &road_plat[2] , &road_plat[3] , &road_plat[4]);
//			storage_exchange(&road_plat[4] , s , x);
//		break;
//	}
//}		
		
		
		
		//////路程状态机
		
void road_state_judge(state_road_list state , platform p_state , storage storage_state){
	switch(state){
		case ROAD_STATE_0:

			v_ca=straight_calc(&road_gene , road_len[0] , v_f , 0.25f , cnt_set[0][0] );
			v_x_front(v_ca);
			break;
		case ROAD_STATE_1:
		if(fabs(set_deg - yaw)<0.1f){
			road_calc_1=1;
			road_calc_2=0;
			rotate_1=1;
			v_ca=straight_calc(&road_gene , road_len[1] , v_f , 0.25f , cnt_set[0][1] );
			v_x_front(v_ca);
				}
			break;
		case ROAD_STATE_2:
			if(p_state!=9){
			road_calc_1=0;
			road_calc_2=1;
			plat_exchange(&road_plat[4]);
			plat_state_judge(p_state);
			}
			if(flag_1==0 && p_state==9){
			cor_calc_1();
			}
			if(flag_1 && p_state==9){
			road_calc_1=1;
			road_calc_2=0;
			v_ca=straight_calc(&road_gene , road_len[2] , v_f , 0.25f , cnt_set[0][2] );
			v_x_front(v_ca);
			}
			break;
		case ROAD_STATE_3:
			if(fabs(set_deg - yaw)<0.1f){
			road_calc_1=1;
			road_calc_2=0;
			rotate_1=1;
			v_ca=straight_calc(&road_gene , road_len[3] , v_f , 0.25f , cnt_set[0][3] );
		  v_x_front(v_ca);
			}
			break;
		case ROAD_STATE_4://出发去立仓
			if(he_cnt[1][0]==0){
				event=1;
				he_cnt[1][0]=1;
			}
			if(fabs(set_deg - yaw)<0.1f && moveit){
			road_calc_1=1;
			road_calc_2=0;
			rotate_1=1;
			v_ca=straight_calc(&road_gene , road_len[4] , v_f , 0.25f , cnt_set[0][4] );
		  v_x_front(v_ca);
			}
			break;
		case ROAD_STATE_5://刚到达立仓
//			if(1){
//			road_calc_1=0;
//			road_calc_2=1;
////			storage_state_judge(storage_state);
//			}
			if(v_state==0 && count_cor_2==0){
				cor_calc_2();
			}
			if(count_cor_2 && flag_3){
				v_state=0;
				event=5;
				flag_3=0;
			}
			if(moveit){
			cor_calc_4();
			flag_2=1;
			}
			if(flag_2){
				v_ca=straight_calc(&road_gene , road_len[5] , v_f , 0.25f , cnt_set[0][5] );
			v_x_front(v_ca);
			}
			break;
		case ROAD_STATE_6:
			v_ca=straight_calc(&road_gene , road_len[6] , v_f , 0.25f , cnt_set[0][6] );
			v_x_front(v_ca);
			break;
		case ROAD_STATE_7:
			v_init();
			break;
		case ROAD_STATE_8:
			v_x_behind(v_f);
			break;
		case ROAD_STATE_9:
			v_x_front(v_f);
			break;
		case ROAD_STATE_10:
			set_deg=rotate_calc(set_deg , yaw , 90.0f , vx , vy );

			break;
		case ROAD_STATE_11:
			set_deg=rotate_calc(set_deg , yaw , 90.0f , vx , vy );
			break;
		
	}
}	













////////////整体路径


////对于放球，向前0.06，回去0.14（加0.02），不同高度之间的相邻球是0.11    //或许???
void state_exchange(road_f *road_gen){
	road_len_init();
	if(road_gen->road >= road_len[0] - delta_rec && cnt_set[0][0]==0){
		cnt_set[0][0]=1;
		road_clear(&road_1 , &road_2 , &road_3 , &road_4 , &road_gene);
		stop_secend();
		road_state=10;
	}
	if(rotate && cnt_set[0][0]==1 && cnt_set[0][1]==0){
		road_state=1;
		rotate=0;
	}
	if(road_gen->road >= road_len[1] - delta_rec && cnt_set[0][1]==0){
		cnt_set[0][1]=1;
		road_clear(&road_1 , &road_2 , &road_3 , &road_4 , &road_gene);
		stop_secend();
		road_state=2;
	}
	if(road_gen->road >= road_len[2] - delta_rec && cnt_set[0][1]==1 && cnt_set[0][2]==0){
		cnt_set[0][2]=1;
		road_clear(&road_1 , &road_2 , &road_3 , &road_4 , &road_gene);
		stop_secend();
		road_state=10;
	}
	if(rotate && cnt_set[0][2]==1 && cnt_set[0][3]==0){
		road_state=3;
		rotate=0;
	}
	if(road_gen->road >= road_len[3] - delta_rec && cnt_set[0][2]==1 && cnt_set[0][3]==0){
		cnt_set[0][3]=1;
		road_clear(&road_1 , &road_2 , &road_3 , &road_4 , &road_gene);
		stop_secend();
		road_state=11;
	}
	if(rotate && cnt_set[0][3]==1 && cnt_set[0][4]==0){
		road_state=4;
		rotate=0;
	}
	if(road_gen->road >= road_len[4] - delta_rec && cnt_set[0][3]==1 && cnt_set[0][4]==0){
		moveit=0;
		cnt_set[0][4]=1;
		road_clear(&road_1 , &road_2 , &road_3 , &road_4 , &road_gene);
		stop_secend();
		road_state=5;
	}
	if(road_gen->road >= road_len[5] - delta_rec && cnt_set[0][4]==1 && cnt_set[0][5]==0){
		cnt_set[0][5]=1;
		road_clear(&road_1 , &road_2 , &road_3 , &road_4 , &road_gene);
		stop_secend();
		road_state=10;
	}
	if(rotate && cnt_set[0][5]==1 && cnt_set[0][6]==0){
		road_state=6;
		rotate=0;
	}
	if(road_gen->road >= road_len[6] - delta_rec && cnt_set[0][5]==1 && cnt_set[0][6]==0){
		cnt_set[0][6]=1;
		road_clear(&road_1 , &road_2 , &road_3 , &road_4 , &road_gene);
		stop_secend();
		road_state=7;
	}
//		if(road_gen->road>= road_len[8] &&road_gen->road< road_len[9] ){//回家_2
//		road_state=9;
//		if(cnt_set[0][9]==0){
//			stop_secend();
//			cnt_set[0][9]=1;
//			INS_angle_set[9]=INS_angle_deg[0]+180;
//			set_deg=INS_angle_set[9];
//		}
//	}
//		if(road_gen->road>= road_len[9] ){//静默
//		road_state=10;
//		if(cnt_set[0][10]==0){
//			stop_secend();
//			cnt_set[0][10]=1;
//			INS_angle_set[10]=INS_angle_deg[0]+180;
//			set_deg=INS_angle_set[10];
//		}
//	}
}









///////////////////////////////////////阶梯平台

		void plat_exchange(road_f *road_gen){
	plat_len_init();
	if(road_gen->road >= plat_len[0] && cnt_set[1][0]==0 ){
		cnt_x=0;
		cnt_set[1][0]=1;
		moveit--;
		road_clear(&road_plat[0] , &road_plat[1] , &road_plat[2] , &road_plat[3] , &road_plat[4]);
		stop_secend();
		p_state=3;
	}
	if(road_gen->road >= plat_len[1] && cnt_set[1][0]==1 && cnt_set[1][1]==0 ){
		cnt_x=0;
		cnt_set[1][1]=1;
		moveit--;
		road_clear(&road_plat[0] , &road_plat[1] , &road_plat[2] , &road_plat[3] , &road_plat[4]);
		stop_secend();
		p_state=4;
	}
	if(road_gen->road >= plat_len[2] && cnt_set[1][1]==1 && cnt_set[1][2]==0 ){
		cnt_x=0;
		cnt_set[1][2]=1;
		moveit--;
		road_clear(&road_plat[0] , &road_plat[1] , &road_plat[2] , &road_plat[3] , &road_plat[4]);
		stop_secend();
		p_state=5;
	}
	if(road_gen->road >= plat_len[3] && cnt_set[1][2]==1 && cnt_set[1][3]==0 ){
		cnt_x=0;
		cnt_set[1][3]=1;
		moveit--;
		road_clear(&road_plat[0] , &road_plat[1] , &road_plat[2] , &road_plat[3] , &road_plat[4]);
		stop_secend();
		p_state=6;
	}
	
	if(road_gen->road >= plat_len[4] && cnt_set[1][3]==1 && cnt_set[1][4]==0 ){
cnt_x=0;
		cnt_set[1][4]=1;
		moveit--;
		road_clear(&road_plat[0] , &road_plat[1] , &road_plat[2] , &road_plat[3] , &road_plat[4]);
		stop_secend();
		p_state=7;
	}
	
	if(road_gen->road >= plat_len[5] && cnt_set[1][4]==1 && cnt_set[1][5]==0 ){
		cnt_x=0;
		cnt_set[1][5]=1;
		moveit--;
		road_clear(&road_plat[0] , &road_plat[1] , &road_plat[2] , &road_plat[3] , &road_plat[4]);
		stop_secend();
		p_state=8;
	}
	
	if(road_gen->road >= plat_len[6] && cnt_set[1][5]==1 && cnt_set[1][6]==0 ){
		cnt_x=0;
		cnt_set[1][6]=1;
		moveit--;
		road_clear(&road_plat[0] , &road_plat[1] , &road_plat[2] , &road_plat[3] , &road_plat[4]);
		stop_secend();
		p_state=9;
	}
}
	








////////////////立体仓库

//void storage_exchange(road_f *road_gen , storage s , uint8_t x){
//	if(x!=s && s!=6){
//		x_delta=x-s;
//		if(x_delta<0){x_delta*=-1;v_pla=0;}
//		target = x_delta * storage_delta;
//	}
//	if(road_gen->road<=0 && road_gen->road <= target-delta_rec){
//		v_ca=v_calc_1(road_gen->road +delta_rec_2 , target , v_s , 0.03f*x_delta);
//		if(v_pla){
//			v_x_behind(v_ca);
//		}
//		else{v_x_front(v_ca);}
//	}
//	if(road_gen->road >= target-delta_rec && s!=6 && target - delta_rec>0){
//		s=6;
//		target=0;
//		x_delta=0;
//		v_pla=1;
//	}
//	if(fabs(road_gen->road - 0)<0.01f){
//		s=x;
//	}
//}


void FSM_begin(void){	
	state_exchange(&road_gene);
	road_state_judge(road_state , p_state , storage_state);
}
	
void FSM_task(void const * argument){
//	remote_control_init();
	while(1){
		chassiss_state_judge(chassiss_state);
		vTaskDelay(1);
	}
}