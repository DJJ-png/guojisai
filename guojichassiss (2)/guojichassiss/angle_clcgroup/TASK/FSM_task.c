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
#define pai 3.1415926
#define v_f 0.05
#define v_s -0.02
#define plat_len_same 0.10f
#define plat_len_dif  0.13f
#define delta_rec 0.01f
#define delta_rec_1 0.03f
#define delta_rec_2 0.003f
#define storage_delta 0.15f

extern osThreadId FSM_TASKHandle;
state_chassiss_list chassiss_state;
state_road_list road_state;
platform p_state;
storage storage_state;
road_f road_plat[5];

extern uint8_t re_buf[4];
extern pid_type_def motor_s,motor_p;
extern fp32 set_yaw , yaw;
extern fp32 INS_angle_deg[3];
extern fp32 set_angle;
extern speed_f speed;
extern fp32 r;
extern fp32 set_v,vx,vy,vx_1,vy_1,set_omega;
extern road_f road_gene;
extern fp32 set_deg;
extern uint8_t event,moveit,moveok,flickit;
extern fp32 delta;
fp32 delta_1[10],delta_2[10];
motor_f motor_2;
fp32 set_angle_LST;
uint8_t road_calc_1=1,road_calc_2=0,rotate_1=0,rotate_2=0,x,x_delta,target,v_pla=1;//x,x_delta,target(立仓)
uint32_t cnt,last_cnt,cnt_con;
angle_f angle_secondmode;
angle_f angle_thirdmode;
fp32 INS_angle_set[11];
fp32 cnt_set[4][11]={0},count_1[10];
fp32 road_len[10],len_add,len_add_1;
fp32 plat_len[10];
fp32 v_ca;

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
	HAL_Delay(400);
}
////方向     x^
////          |
////          |
////          |
////          |_____________>y

void v_x_front(fp32 v){//电池为头
	vx=-v;
	vy=0;
	set_omega=0;
	vx=v_calc(vx);
}

void v_x_behind(fp32 v){//电池为头
	vx=v;
	vy=0;
	set_omega=0;
	vx=v_calc(vx);
}

void v_y_front(fp32 v){
	vx=0;
	vy=v;
	set_omega=0;
	vy=v_calc(vy);
}

void v_y_behind(fp32 v){
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
	road_len[0] = 0.910;
	road_len[1] = road_len[0] + 1.77f;
	road_len[2] = road_len[1] + 1.2f;
	road_len[3] = road_len[2] + 1.4f;
	road_len[4] = road_len[3] + 1.0f;
	road_len[5] = road_len[4] + 1.8f;
	road_len[6] = road_len[5] + 1.2f;
	road_len[7] = road_len[6] + 1.6f;
	road_len[8] = road_len[7] + 2.0f;
	road_len[9] = road_len[8] + 1.0f;
}

void plat_len_init(){
	plat_len[0] = plat_len_same;
	plat_len[1] = plat_len[0] + plat_len_dif;
	plat_len[2] = plat_len[1] + plat_len_same;
	plat_len[3] = plat_len[2] + plat_len_same;
	plat_len[4] = plat_len[3] + plat_len_same;
	plat_len[5] = plat_len[4] + plat_len_dif;
	plat_len[6] = plat_len[5] + plat_len_same;
}

float v_calc_1(fp32 road , fp32 desti , fp32 v_stand , fp32 edg){
			if(road<0){road*=-1;}
			fp32 v;
			if(road<edg){
					fp32 k=road/edg;
					v=k*v_stand;
			}
			else if(road>desti-edg){
				fp32 j=desti-road;
				j/=edg;
				v=j*v_stand;
			}
			else{
				v=v_stand;
			}
			return v;
		}		

float v_calc_2(fp32 road , fp32 desti , fp32 v_stand , fp32 edg){
			if(road<0){road*=-1;}
			fp32 v;
			if(road<edg){
					fp32 k=road/edg;
					v=k*v_stand;
			}
			else if(road>desti-edg){
				fp32 j=desti-road;
				j/=edg;
				v=j*v_stand;
			}
			else{
				v=v_stand;
			}
			return v;
		}		
		
		
		
		
		
		
		
		
		///////////////////阶梯平台状态机
void plat_state_judge(platform p_state){
			switch(p_state){
				
				case P_STATE_0:
					
					break;
				case P_STATE_1:
					
					break;
				case P_STATE_2:
					v_ca=v_calc_1(road_plat[4].road +delta_rec_2 , plat_len_same , v_s , 0.03f);
					v_x_front(v_ca);
					break;
				case P_STATE_3:
					len_add=plat_len_same;
					v_ca=v_calc_1(road_plat[4].road - len_add +delta_rec_2 , plat_len_dif , v_s , 0.03f);
					v_x_front(v_ca);
					break;
				case P_STATE_4:
					len_add=plat_len_same + plat_len_dif;
					v_ca=v_calc_1(road_plat[4].road - len_add +delta_rec_2 , plat_len_same , v_s , 0.03f);
					v_x_front(v_ca);
					break;
				case P_STATE_5:
					len_add=2 * plat_len_same + plat_len_dif;
					v_ca=v_calc_1(road_plat[4].road - len_add +delta_rec_2 , plat_len_same , v_s , 0.03f);
					v_x_front(v_ca);
					break;
				case P_STATE_6:
					len_add=3 * plat_len_same + plat_len_dif;
					v_ca=v_calc_1(road_plat[4].road - len_add +delta_rec_2 , plat_len_same , v_s , 0.03f);
					v_x_front(v_ca);
					break;
				case P_STATE_7:
					len_add=4 * plat_len_same + plat_len_dif;
					v_ca=v_calc_1(road_plat[4].road - len_add +delta_rec_2 , plat_len_dif , v_s , 0.03f);
					v_x_front(v_ca);
					break;
				case P_STATE_8:
					len_add=4 * plat_len_same + 2 * plat_len_dif;
					v_ca=v_calc_1(road_plat[4].road - len_add +delta_rec_2 , plat_len_same , v_s , 0.03f);
					v_x_front(v_ca);
					break;
				case P_STATE_9:
					
					break;
				case P_STATE_10:
					if(1)
					v_init();
					break;
				case P_STATE_11:
					if(0)
					v_x_front(v_s);
					break;
			}
		}

		
		
		
		
		
		//////立仓状态机
void storage_state_judge(storage s){
	switch(s){
		case S_STATE_0:
			storage_exchange(&road_plat[4] , s , x);
		break;
		case S_STATE_1:
			storage_exchange(&road_plat[4] , s , x);
		break;
		case S_STATE_2:
			storage_exchange(&road_plat[4] , s , x);
		break;
		case S_STATE_3:
			storage_exchange(&road_plat[4] , s , x);
		break;
		case S_STATE_4:
			
		break;
		case S_STATE_5:
			
		break;
		case S_STATE_6:
			v_init();
			road_clear(&road_plat[0] , &road_plat[1] , &road_plat[2] , &road_plat[3] , &road_plat[4]);
			storage_exchange(&road_plat[4] , s , x);
		break;
	}
}		
		
		
		
		
		//////路程状态机
		
void road_state_judge(state_road_list state , platform p_state , storage storage_state){
	switch(state){
		case ROAD_STATE_0:
			v_ca=v_calc_1(road_gene.road , road_len[0] , v_f , 0.25f);
			v_y_front(-v_ca);
			break;
		case ROAD_STATE_1:
			v_ca=v_calc_2(road_gene.road - road_len[0] +delta_rec , road_len[1]-road_len[0] , v_f , 0.25f);//+ delta_rec - delta_1[0]
			v_x_front(-v_ca);

			break;
		case ROAD_STATE_2:
			if(p_state!=10){
			road_calc_1=0;
			road_calc_2=1;
			plat_exchange(&road_plat[4]);
			plat_state_judge(p_state);
			}
			else{
			road_calc_1=1;
			road_calc_2=0;
			v_ca=v_calc_2(road_gene.road - road_len[1] +delta_rec , road_len[2]-road_len[1] , v_f , 0.25f);//+ delta_2[1] + delta_rec_1
			v_x_front(-v_ca);
			}
			break;
		case ROAD_STATE_3:
			v_ca=v_calc_2(road_gene.road - road_len[2] +delta_rec , road_len[3]-road_len[2] , v_f , 0.25f);
		  v_y_front(-v_ca);
			break;
		case ROAD_STATE_4://出发去立仓
			v_ca=v_calc_2(road_gene.road - road_len[3] +delta_rec , road_len[4]-road_len[3] , v_f , 0.25f);
			v_x_front(0);
			break;
		case ROAD_STATE_5://刚到达立仓
			if(1){
			road_calc_1=0;
			road_calc_2=1;
			storage_state_judge(storage_state);
			}
			if(0){
			v_x_front(v_f);
			}
			break;
		case ROAD_STATE_6:
			v_y_behind(v_f);
			break;
		case ROAD_STATE_7:
			v_x_behind(v_f);
			break;
		case ROAD_STATE_8:
			v_x_behind(v_f);
			break;
		case ROAD_STATE_9:
			v_y_front(v_f);
			break;
		case ROAD_STATE_10:
			
			if(fabs(yaw - set_deg)<0.1f && rotate_2){rotate_1=1; cnt_con++;}
			if(fabs(yaw - set_deg)>2.0f && rotate_1){rotate_1=0;}
			break;
		case ROAD_STATE_11:
			v_init();
			break;
		
	}
}	













////////////整体路径


////对于放球，向前0.06，回去0.14（加0.02），不同高度之间的相邻球是0.11    //或许???
void state_exchange(road_f *road_gen){
	road_len_init();
	if(road_gen->road>=0&&road_gen->road<road_len[0] - delta_rec &&count_1[0]==0){//上移road_gen->road>=0&&road_gen->road<road_len[0] - delta_rec +  delta_1[0]&&count_1[0]==0
		delta_1[0]=delta;
		road_state=0;
		if(cnt_set[0][0]==0){
			cnt_set[0][0]=1;
			INS_angle_set[0]=INS_angle_deg[0]+180;
			set_deg=INS_angle_set[0];
		}
	}
		if(road_gen->road>= road_len[0] - delta_rec  && road_gen->road< road_len[1]  - delta_rec_1 &&count_1[1]==0 ){//阶梯平台road_gen->road>= road_len[0] - delta_rec + delta_1[0] && road_gen->road< road_len[1] - delta_2[1] - delta_rec_1 &&count_1[1]==0
			delta_1[1]=delta;
			if(delta_1[1] > delta_1[0]){
				delta_2[1]=delta_1[1]-delta_2[0];
			}
			else{
				delta_2[1]=delta_1[0]-delta_2[1];
			}
		road_state=1;
		if(cnt_set[0][1]==0){
			count_1[0]=1;
			stop_secend();
			cnt_set[0][1]=1;
			INS_angle_set[1]=INS_angle_deg[0]+180;
			set_deg=INS_angle_set[1];

		}
	}
		if(road_gen->road>= road_len[1]  - delta_rec_1 &&road_gen->road< road_len[2] &&count_1[2]==0){//立桩前面
		road_state=2;
    if(cnt_set[0][2]==0){
			count_1[1]=1;
			stop_secend();
			cnt_set[0][2]=1;
			INS_angle_set[2]=INS_angle_deg[0]+180;
			set_deg=INS_angle_set[2];
		}	
	}
		if(road_gen->road>= road_len[2] &&road_gen->road< road_len[3] ){//立桩旁边
			if(rotate_1==0 && rotate_2==0){
				road_calc_1=0;
				road_calc_2=0;
				road_state=10;
				set_deg=yaw-180;
				if(set_deg<-180){set_deg+=360;}
				rotate_2=1;
			}
		if(rotate_1&&rotate_2){
		road_state=3;
	}
		if(cnt_set[0][3]==0 && cnt_con>100){
			
			stop_secend();
			cnt_set[0][3]=1;
			INS_angle_set[3]=INS_angle_deg[0]+180;
			set_deg=INS_angle_set[3];
		}
	}
		
	
	
		if(road_gen->road>= road_len[3] &&road_gen->road< road_len[4] ){//绕过立桩
		road_state=4;
		if(cnt_set[0][4]==0){
			stop_secend();
			cnt_set[0][4]=1;
			INS_angle_set[4]=INS_angle_deg[0]+180;
			set_deg=INS_angle_set[4];
		}
	}
		if(road_gen->road>= road_len[4] &&road_gen->road< road_len[5] ){//前往白点
		road_state=5;
		if(cnt_set[0][5]==0){
			stop_secend();
			cnt_set[0][5]=1;
			INS_angle_set[5]=INS_angle_deg[0]+180;
			set_deg=INS_angle_set[5];
			road_clear(&road_plat[0] , &road_plat[1] , &road_plat[2] , &road_plat[3] , &road_plat[4]);
		}
	}
	if(road_gen->road>= road_len[5] &&road_gen->road< road_len[6] ){//前往圆盘机
		road_state=6;
		if(cnt_set[0][6]==0){
			stop_secend();
			cnt_set[0][6]=1;
			INS_angle_set[6]=INS_angle_deg[0]+180;
			set_deg=INS_angle_set[6];
		}
	}
		if(road_gen->road>= road_len[6] &&road_gen->road< road_len[7] ){//前往立仓
		road_state=7;
		if(cnt_set[0][7]==0){
			stop_secend();
			cnt_set[0][7]=1;
			INS_angle_set[7]=INS_angle_deg[0]+180;
			set_deg=INS_angle_set[7];
		}
	}
		if(road_gen->road>= road_len[7] &&road_gen->road< road_len[8] ){//回家_1
		road_state=8;
		if(cnt_set[0][8]==0){
			stop_secend();
			cnt_set[0][8]=1;
			INS_angle_set[8]=INS_angle_deg[0]+180;
			set_deg=INS_angle_set[8];
		}
	}
		if(road_gen->road>= road_len[8] &&road_gen->road< road_len[9] ){//回家_2
		road_state=9;
		if(cnt_set[0][9]==0){
			stop_secend();
			cnt_set[0][9]=1;
			INS_angle_set[9]=INS_angle_deg[0]+180;
			set_deg=INS_angle_set[9];
		}
	}
		if(road_gen->road>= road_len[9] ){//静默
		road_state=10;
		if(cnt_set[0][10]==0){
			stop_secend();
			cnt_set[0][10]=1;
			INS_angle_set[10]=INS_angle_deg[0]+180;
			set_deg=INS_angle_set[10];
		}
	}
}









/////////////////////////////////////////阶梯平台

void plat_exchange(road_f *road_gen){
	plat_len_init();
	if(road_gen->road>= 0 &&road_gen->road< plat_len[0] ){//第一个（低）-delta_rec
		road_gen->road=0.006;
	  p_state=2;
		if(cnt_set[1][0]==0){
			stop_secend();
			cnt_set[1][0]=1;
		}
	}
	if(road_gen->road>= plat_len[0] &&road_gen->road< plat_len[1] ){
									event=3;
	  p_state=3;
		if(cnt_set[1][1]==0){
			stop_secend();
			cnt_set[1][1]=1;
		}
	}
	if(road_gen->road>= plat_len[1] &&road_gen->road< plat_len[2] ){//第一个（高）
	  p_state=4;
		if(cnt_set[1][2]==0){
			stop_secend();
			cnt_set[1][2]=1;
		}
	}
	if(road_gen->road>= plat_len[2] &&road_gen->road< plat_len[3] ){
	  p_state=5;
		if(cnt_set[1][3]==0){
			stop_secend();
			cnt_set[1][3]=1;
		}
	}
	if(road_gen->road>= plat_len[3] &&road_gen->road< plat_len[4] ){
	  p_state=6;
		if(cnt_set[1][4]==0){
			stop_secend();
			cnt_set[1][4]=1;
		}
	}
	if(road_gen->road>= plat_len[4] &&road_gen->road< plat_len[5] ){
	  p_state=7;
		if(cnt_set[1][5]==0){
			stop_secend();
			cnt_set[1][5]=1;
		}
	}
	if(road_gen->road>= plat_len[5] &&road_gen->road< plat_len[6] ){//第一个（中）
	  p_state=8;
		if(cnt_set[1][6]==0){
			stop_secend();
			cnt_set[1][6]=1;
		}
	}
//	if(road_gen->road>= plat_len[6] &&road_gen->road< plat_len[7] ){
//	 	p_state=9;
//		if(cnt_set[1][7]==0){
//			stop_secend();
//			cnt_set[1][7]=1;
//		}
//	}
	if(road_gen->road>= plat_len[6]  ){//&&road_gen->road< plat_len[8]
	  p_state=10;
	}
//	if(road_gen->road>= plat_len[8] &&road_gen->road< plat_len[9] ){
//	  p_state=11;
//		stop_secend();
//	}
}
	








//////////////立体仓库

void storage_exchange(road_f *road_gen , storage s , uint8_t x){
	if(x!=s && s!=6){
		x_delta=x-s;
		if(x_delta<0){x_delta*=-1;v_pla=0;}
		target = x_delta * storage_delta;
	}
	if(road_gen->road<=0 && road_gen->road <= target-delta_rec){
		v_ca=v_calc_1(road_gen->road +delta_rec_2 , target , v_s , 0.03f*x_delta);
		if(v_pla){
			v_x_behind(v_ca);
		}
		else{v_x_front(v_ca);}
	}
	if(road_gen->road >= target-delta_rec && s!=6 && target - delta_rec>0){
		s=6;
		target=0;
		x_delta=0;
		v_pla=1;
	}
	if(fabs(road_gen->road - 0)<0.01f){
		s=x;
	}
}


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