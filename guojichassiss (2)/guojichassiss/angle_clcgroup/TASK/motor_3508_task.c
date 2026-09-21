#include "motor_3508_task.h"

extern motor_measure_t motor_chassis[7];
uint8_t actionflag,clearflag=0;

											//waijie event[4] flickit moveok










struct motor_3508_t motor1,motor2,motorbo;

uint8_t stop=0,moveit=0,moving;
uint8_t downflag,floorflag=0,catchflag,flickflag,hangflag,moveok=0,catchok=0,backok=0,stopok=0,downok=0;//,oneflag,twoflag,threeflag
uint8_t set_distance;
fp32 hangdistance,roll_set_jiao=0;
fp32    kp_s1=3,ki_s1=0.001,kd_s1,    kp_j1=-20,ki_j1=-0.1,kd_j1,     kp_s2=10,ki_s2=0.05,kd_s2,    kp_j2=20.0,ki_j2=0.1,kd_j2,    kp_sh=0.8,ki_sh,kd_sh,   kp_jh=-80,ki_jh=-0.2,kd_jh,kp_sb=1,ki_sb=0.01,kd_sb,kp_jb=-100,ki_jb=-0.2,kd_jb;
pid_type_def hang_pid_s,hang_pid_j;
fp32 HANG_PID_S[3],HANG_PID_J[3];


fp32 tempdistance=250;


state_way gimbal_state;

uint16_t delaycount=0,flickcount=0,catchcount=0,bodancount=0,flickit=0;
int pwm[4]={2000,2000,1500,2300};


uint8_t event=0;

void Motor_3508_task(void const * argument)
{
  motor_3508_init();
  while(1)
	{
		
		
		
		
	
		
//		motorbo.rmp=PID_calc(&motorbo.pid_j, motorbo.jiaodu,roll_set_jiao);
//		motorbo.pwm_out=PID_calc(&motorbo.pid_s, motor_chassis[4].speed_rpm,motorbo.rmp);
		
		
		
	
//		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pwm[0]);
		
		motor_updata();
		if(event!=0)
			actionflag=1;	
		motor_state(event);
		limit();	
//		if(stop==1)
//			motor1.pwm_out=0;

		vTaskDelay(1);
	}
  
}







void limit()
{
	if(motor1.pwm_out<=-3000)
		motor1.pwm_out=-3000;
	if(motor1.pwm_out>=3000)
		motor1.pwm_out=3000;
	if(motor2.pwm_out<=-3000)
		motor2.pwm_out=-3000;
	if(motor2.pwm_out>=3000)
		motor2.pwm_out=3000;
}
	


//void motor_calcjiao()
//{
//	
//	static float jiaodu1=0,jiaodu2=0;
//	motor1.ecn=motor_chassis[motor1_3508].ecd;
//	if(count++<2){
//		motor1.ecn_last=motor1.ecn;
//	}
//	
//	if((motor1.ecn-motor1.ecn_last)>4000)
//	jiaodu1+=-360.0*(motor1.ecn-8191-motor1.ecn_last)/(1.0*8191*19);
//	else if((motor1.ecn-motor1.ecn_last)<-4000)
//	jiaodu1+=-360.0*(motor1.ecn+8191-motor1.ecn_last)/(1.0*8191*19);
//	else
//	jiaodu1+=-360.0*(motor1.ecn-motor1.ecn_last)/(1.0*8191*19);
//	motor1.ecn_last=motor1.ecn;
//	 motor1.jiaodu=jiaodu1;
//	
//	motor2.ecn=motor_chassis[motor2_3508].ecd;
//	if(count++<2){
//		motor2.ecn_last=motor2.ecn;
//	}
//	
//	if((motor2.ecn-motor2.ecn_last)>4000)
//	jiaodu2+=360.0*(motor2.ecn-8191-motor2.ecn_last)/(1.0*8191);
//	else if((motor2.ecn-motor2.ecn_last)<-4000)
//	jiaodu2+=360.0*(motor2.ecn+8191-motor2.ecn_last)/(1.0*8191);
//	else
//	jiaodu2+=360.0*(motor2.ecn-motor2.ecn_last)/(1.0*8191);
//	motor2.ecn_last=motor2.ecn;
//	motor2.jiaodu=jiaodu2;
//}


void motor_3508_init()
{
	can_filter_init();
	HANG_PID_S[0]=kp_sh;
	HANG_PID_S[1]=ki_sh;
	HANG_PID_S[2]=kd_sh;
	HANG_PID_J[0]=kp_jh;
	HANG_PID_J[1]=ki_jh;
	HANG_PID_J[2]=kd_jh;
	motorbo.PID_J[0]=kp_jb;
	motorbo.PID_J[1]=ki_jb;
	motorbo.PID_J[2]=kd_jb;
	motorbo.PID_S[0]=kp_sb;
	motorbo.PID_S[1]=ki_sb;
	motorbo.PID_S[2]=kd_sb;
	motor1.PID_S[0]=kp_s1;
	motor1.PID_S[1]=ki_s1;
	motor1.PID_S[2]=kd_s1;
	motor1.PID_J[0]=kp_j1;
	motor1.PID_J[1]=ki_j1;
	motor1.PID_J[2]=kd_j1;
	motor2.PID_S[0]=kp_s2;
	motor2.PID_S[1]=ki_s2;
	motor2.PID_S[2]=kd_s2;
	motor2.PID_J[0]=kp_j2;
	motor2.PID_J[1]=ki_j2;
	motor2.PID_J[2]=kd_j2;
	PID_init(&motorbo.pid_s,PID_POSITION,motorbo.PID_S,350,100);
	PID_init(&motorbo.pid_j,PID_POSITION,motorbo.PID_J,2000,100);
	
	PID_init(&hang_pid_s,PID_POSITION,HANG_PID_S,2000,500);
	PID_init(&hang_pid_j,PID_POSITION,HANG_PID_J,3000,1000);
	PID_init(&motor1.pid_s,PID_POSITION,motor1.PID_S,3000,500);
	PID_init(&motor1.pid_j,PID_POSITION,motor1.PID_J,2000,500);
	PID_init(&motor2.pid_s,PID_POSITION,motor2.PID_S,2000,500);
	PID_init(&motor2.pid_j,PID_POSITION,motor2.PID_J,350,20);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pwm[0]);               //2000-500
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,pwm[1]);               //2000-1500
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,pwm[2]);								//2000-1500
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,pwm[3]);								//2500
}

void motor_updata()
{
	//motor_calcjiao();
	motor1.distance=motor1.jiaodu*1.0;
	motor2.distance=motor2.jiaodu*0.18468f;
}	



void motor_state(uint8_t actionway)
{
	switch(gimbal_state)
	{
		case STATE_STOP:
			
			if(actionflag==1)
				gimbal_state=STATE1_FLOOR;
		break;
		case STATE1_FLOOR:																																								//	FLOOR
			if(actionway==1)                                                                                          //xuanzhuanboqiu
			{
				motor1.pwm_out=PID_calc(&motor1.pid_s, motor_chassis[5].speed_rpm,-1500)+GRAVITY;
				
//				motor1.pwm_out=-2000;
				if(motor1.distance>=ROLLDISTANCE-20&&motor1.distance<=ROLLDISTANCE+20)
				{
					gimbal_state=STATE1_HANG;				
				}
			}
			if(actionway==2)																																												//raojiaqiu
			{
				motor1.pwm_out=PID_calc(&motor1.pid_s, motor_chassis[5].speed_rpm,-1500)+GRAVITY;
				if(motor1.distance>=ECHODISTANCE-20&&motor1.distance<=ECHODISTANCE+20)
				{
					gimbal_state=STATE1_HANG;				
				}
			}
			if(actionway==3)																																										//taijie
			{								
				if(floorflag==0)
					floorflag=1;
				if(floorflag==1)
				{
						motor1.pwm_out=PID_calc(&motor1.pid_s, motor_chassis[5].speed_rpm,-1500)+GRAVITY;
						if(motor1.distance>=ONEHIGH-20&&motor1.distance<=ONEHIGH+20)
						{
							hangdistance=ONEHIGH;
							gimbal_state=STATE1_HANG;				
						}
				}
				if(floorflag==2)
				{
						motor1.pwm_out=PID_calc(&motor1.pid_s, motor_chassis[5].speed_rpm,-1500)+GRAVITY;
						if(motor1.distance>=THREEHIGH-20&&motor1.distance<=THREEHIGH+20)
						{
							hangdistance=THREEHIGH;
							gimbal_state=STATE1_HANG;				
						}
				}
				if(floorflag==3)
				{
						motor1.pwm_out=PID_calc(&motor1.pid_s, motor_chassis[5].speed_rpm,-1500)+GRAVITY;
						if(motor1.distance>=TWOHIGH-20&&motor1.distance<=TWOHIGH+20)
						{
							hangdistance=TWOHIGH;
							gimbal_state=STATE1_HANG;				
						}
				}
			}
			if(actionway==4)
			{
				
				
				
				
				
				
				
				motor1.pwm_out=PID_calc(&motor1.pid_s, motor_chassis[5].speed_rpm,-1500)+GRAVITY;
				if(motor1.distance>=tempdistance-20&&motor1.distance<=tempdistance+20)
				{
					gimbal_state=STATE1_HANG;				
				}
				
				
				
				
				
				
				
				
				
				
			}
		break;
		case STATE1_HANG:																																							//HANG
			if(actionway==1)
			{			
				motor1.rmp=PID_calc(&hang_pid_j, motor1.jiaodu,ROLLDISTANCE);
				motor1.pwm_out=PID_calc(&hang_pid_s, motor_chassis[5].speed_rpm,motor1.rmp)+GRAVITY;
				delaycount++;
				if(delaycount>50)
				{
					delaycount=0;
					gimbal_state=STATE2_EXTEND;
				}
			}
			if(actionway==2)
			{					
				motor1.rmp=PID_calc(&hang_pid_j, motor1.jiaodu,ECHODISTANCE);
				motor1.pwm_out=PID_calc(&hang_pid_s, motor_chassis[5].speed_rpm,motor1.rmp)+GRAVITY;
				
				delaycount++;
				if(delaycount>50)
				{
					if(delaycount>=60000)
						delaycount=21;
					if(moveok==1)
					{
						delaycount=0;
						moveok=0;
						gimbal_state=STATE2_EXTEND;
					}				
				}													
			}
			if(actionway==3)
			{
				motor1.rmp=PID_calc(&hang_pid_j, motor1.jiaodu,hangdistance);
				motor1.pwm_out=PID_calc(&hang_pid_s, motor_chassis[5].speed_rpm,motor1.rmp)+GRAVITY;
				
				delaycount++;
				if(delaycount>50)
				{
					if(delaycount>=60000)
						delaycount=21;
					if(moveok==1)
					{
						delaycount=0;
						moveok=0;
						gimbal_state=STATE2_EXTEND;
					}				
				}		
			}
			if(actionway==4)
			{
				
				
				motor1.rmp=PID_calc(&hang_pid_j, motor1.jiaodu,ROLLDISTANCE);
				motor1.pwm_out=PID_calc(&hang_pid_s, motor_chassis[5].speed_rpm,motor1.rmp)+GRAVITY;
				
	//			motor1.pwm_out=PID_calc(&hang_pid_j, motor1.distance,tempdistance)+GRAVITY;
				delaycount++;
				if(delaycount>2000)
				{
					delaycount=0;
			//		gimbal_state=	STATE1_DOWN;
				}
				
				
				
				
				
			}
		break;
		case STATE2_EXTEND:																																					//	EXTEND				
			if(actionway==1)
			{
				motor1.rmp=PID_calc(&hang_pid_j, motor1.jiaodu,ROLLDISTANCE);
				motor1.pwm_out=PID_calc(&hang_pid_s, motor_chassis[5].speed_rpm,motor1.rmp)+GRAVITY;
				motor2.rmp=PID_calc(&motor2.pid_j, motor2.distance,EXTEND_DISTANCE);
				motor2.pwm_out=PID_calc(&motor2.pid_s, motor_chassis[motor2_3508].speed_rpm,motor2.rmp);
			
				if(motor2.distance>=EXTEND_DISTANCE-20&&motor2.distance<=EXTEND_DISTANCE+20)
					gimbal_state=STATE2_FLICK;
			
			
			}
			if(actionway==2)
			{
				motor1.rmp=PID_calc(&hang_pid_j, motor1.jiaodu,ECHODISTANCE);
				motor1.pwm_out=PID_calc(&hang_pid_s, motor_chassis[5].speed_rpm,motor1.rmp)+GRAVITY;
				motor2.rmp=PID_calc(&motor2.pid_j, motor2.distance,EXTEND_DISTANCE);
				motor2.pwm_out=PID_calc(&motor2.pid_s, motor_chassis[motor2_3508].speed_rpm,motor2.rmp);
				if(motor2.distance>=EXTEND_DISTANCE-20&&motor2.distance<=EXTEND_DISTANCE+20)
					gimbal_state=STATE2_LOCK;
			}
			if(actionway==3)
			{
				motor1.rmp=PID_calc(&hang_pid_j, motor1.jiaodu,hangdistance);
				motor1.pwm_out=PID_calc(&hang_pid_s, motor_chassis[5].speed_rpm,motor1.rmp)+GRAVITY;
				motor2.rmp=PID_calc(&motor2.pid_j, motor2.distance,EXTEND_DISTANCE);
				motor2.pwm_out=PID_calc(&motor2.pid_s, motor_chassis[motor2_3508].speed_rpm,motor2.rmp);
				if(motor2.distance>=EXTEND_DISTANCE-20&&motor2.distance<=EXTEND_DISTANCE+20)
					gimbal_state=STATE2_LOCK;
			}
			if(actionway==4)
			{
				
			}
		break;
		case STATE2_LOCK:																																									//LOCK
			if(actionway==1)
			{
				
			}
			if(actionway==2)
			{
				motor1.rmp=PID_calc(&hang_pid_j, motor1.jiaodu,ECHODISTANCE);
				motor1.pwm_out=PID_calc(&hang_pid_s, motor_chassis[5].speed_rpm,motor1.rmp)+GRAVITY;
				motor2.rmp=PID_calc(&motor2.pid_j, motor2.distance,EXTEND_DISTANCE);
				motor2.pwm_out=PID_calc(&motor2.pid_s, motor_chassis[motor2_3508].speed_rpm,motor2.rmp);
				
				
				if(flickit==1){
					delaycount++;
					if(delaycount>500)
					{
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);
					}				
					
					if(delaycount>2000)
					{
						catchcount++;
						gimbal_state=STATE2_BACK;
						delaycount=0;
						catchok=1;
						flickit=0;
						if(catchcount>=6)
						{
							catchcount=0;
							stopok=1;
							gimbal_state=STATE2_BACK;
							break;
						}
					}
				}	
				if(flickit==2)
				{
					flickit=0;
					gimbal_state=STATE2_BACK;
				}			
			}
			if(actionway==3)
			{
				motor1.rmp=PID_calc(&hang_pid_j, motor1.jiaodu,hangdistance);
				motor1.pwm_out=PID_calc(&hang_pid_s, motor_chassis[5].speed_rpm,motor1.rmp)+GRAVITY;
				motor2.rmp=PID_calc(&motor2.pid_j, motor2.distance,EXTEND_DISTANCE);
				motor2.pwm_out=PID_calc(&motor2.pid_s, motor_chassis[motor2_3508].speed_rpm,motor2.rmp);
				
				
				
				
				if(flickit==1){
					delaycount++;
					if(delaycount>500)
					{
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);
					}				
					
					if(delaycount>2000)
					{
						catchcount++;
						catchok++;
						gimbal_state=STATE2_BACK;
						delaycount=0;
						flickit=0;
					}
				}	
				if(flickit==2)
				{
					catchcount++;
					flickit=0;
					gimbal_state=STATE2_BACK;
				}			
				
			}
			if(actionway==4)
			{
				
			}
		break;
		case STATE2_BACK:																																									//BACK
			if(actionway==1)
			{
				motor1.rmp=PID_calc(&hang_pid_j, motor1.jiaodu,ROLLDISTANCE);
				motor1.pwm_out=PID_calc(&hang_pid_s, motor_chassis[5].speed_rpm,motor1.rmp)+GRAVITY;
				motor2.rmp=PID_calc(&motor2.pid_j, motor2.distance,0);
				motor2.pwm_out=PID_calc(&motor2.pid_s, motor_chassis[motor2_3508].speed_rpm,motor2.rmp);
				if(motor2.distance>=-20&&motor2.distance<=20)
					gimbal_state=STATE1_DOWN;
				
			}
			if(actionway==2)
			{
				motor1.rmp=PID_calc(&hang_pid_j, motor1.jiaodu,ECHODISTANCE);
				motor1.pwm_out=PID_calc(&hang_pid_s, motor_chassis[5].speed_rpm,motor1.rmp)+GRAVITY;
				motor2.rmp=PID_calc(&motor2.pid_j, motor2.distance,0);
				motor2.pwm_out=PID_calc(&motor2.pid_s, motor_chassis[motor2_3508].speed_rpm,motor2.rmp);
				if(catchok==1)
				{
					moveit=1;
					gimbal_state=STATE1_DOWN;
					
					break;
				}
				if(moveit==0&&catchok==0&&moving==0)
				{
					moveit=1;
					moving=1;
				}
					
					
				if(moveok==1)
				{
					moveok=0;
					moving=0;
					gimbal_state=STATE2_EXTEND;
				}
			}
			if(actionway==3)
			{																																																										//one or two times for success
				motor1.rmp=PID_calc(&hang_pid_j, motor1.jiaodu,hangdistance);
				motor1.pwm_out=PID_calc(&hang_pid_s, motor_chassis[5].speed_rpm,motor1.rmp)+GRAVITY;
				motor2.rmp=PID_calc(&motor2.pid_j, motor2.distance,0);
				motor2.pwm_out=PID_calc(&motor2.pid_s, motor_chassis[motor2_3508].speed_rpm,motor2.rmp);
				delaycount++;
				if(delaycount>1000)
				{
					if(hangdistance==ONEHIGH||hangdistance==TWOHIGH)
					{
						if(catchok==0&&catchcount==1&&moving==0)
						{
							moveit=1;
							moving=1;
							catchcount=0;
						}
						if(catchok==1)
						{									
							if(moving==0)
							{
								moveit=3-catchcount;
								catchcount=0;
								if(floorflag<3)
								floorflag++;
								delaycount=0;
								gimbal_state=STATE1_DOWN;
								catchok=0;
								break;
							}
							
									
						}
						if(moveok==1&&catchok==0&&catchcount==1)
						{
							delaycount=0;
							moveok=0;
							moving=0;
							gimbal_state=STATE2_EXTEND;
						}
					}
					if(hangdistance==THREEHIGH)
					{
						if(catchok!=2&&catchcount!=4&&moving==0)
						{										
							moveit=1;
							moving=1;
						}
						if(moveok==1)
						{
							moveok=0;
							moving=0;
							delaycount=0;
							gimbal_state=STATE2_EXTEND;
						}
						if(catchok==1&&downok==0)
						{
							downok=1;
							delaycount=0;
							moveit=1;
							gimbal_state=STATE1_DOWN;
						}
						if(catchok==2)
						{
							moveit=5-catchcount;
							floorflag++;
							delaycount=0;
							gimbal_state=STATE1_DOWN;
							catchok=0;
							catchcount=0;
							break;
						}
						
						
						
					}
				}
			}
			if(actionway==4)
			{
				
			}
		break;
		case STATE2_FLICK:																																								//FLICK
			if(actionway==1)
			{
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,500);
				motor1.rmp=PID_calc(&hang_pid_j, motor1.jiaodu,ROLLDISTANCE);
				motor1.pwm_out=PID_calc(&hang_pid_s, motor_chassis[5].speed_rpm,motor1.rmp)+GRAVITY;
				
				motor2.rmp=PID_calc(&motor2.pid_j, motor2.distance,EXTEND_DISTANCE);
				motor2.pwm_out=PID_calc(&motor2.pid_s, motor_chassis[motor2_3508].speed_rpm,motor2.rmp);
				if(flickit==1)
				{			
//					if(bodancount++<1)
//					{
//						roll_set_jiao+=36;
//					}
					delaycount++;
					if(delaycount<=200)
					{
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1200);
					}
					else if(delaycount>=200&&delaycount<=400)
					{
						__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1800);
					}
					else if(delaycount>=400&&delaycount<=600)
					{
						if(bodancount++<1)
						{
							roll_set_jiao+=36;
						}
					}
					else
					{
						bodancount=0;
						delaycount=0;
						flickcount++;
						flickit=0;
					}
				}
				
				
				if(flickcount>=6)
				{
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1500);
					flickcount=0;
					gimbal_state=STATE2_BACK;
				}
			}
			if(actionway==2)
			{
				
			}
			if(actionway==3)
			{
				
			}
			if(actionway==4)
			{
				
			}
		break;
		case STATE1_DOWN:																																			//DOWN
			if(actionway==1)
			{
				motor1.pwm_out=400;
				if(motor1.distance<=100)
				motor1.pwm_out=10*(motor1.distance-60);	
				if(motor1.distance<=20)
				{
					event=0;
					actionflag=0;
					gimbal_state=STATE_STOP;
				}
			}
			if(actionway==2)
			{
				
				motor1.pwm_out=400;
				if(motor1.distance<=100)
				motor1.pwm_out=10*(motor1.distance-60);	
				motor2.rmp=PID_calc(&motor2.pid_j, motor2.distance,0);
				motor2.pwm_out=PID_calc(&motor2.pid_s, motor_chassis[motor2_3508].speed_rpm,motor2.rmp);
				if(motor1.distance<=20)
				{
					if(catchok==1)
					{
						catchok=0;
						gimbal_state=STATE2_UNLOCK;
					}								
				}
			}
			if(actionway==3)
			{
				motor1.pwm_out=400;
				if(motor1.distance<=100)
				motor1.pwm_out=10*(motor1.distance-60);	
				motor2.rmp=PID_calc(&motor2.pid_j, motor2.distance,0);
				motor2.pwm_out=PID_calc(&motor2.pid_s, motor_chassis[motor2_3508].speed_rpm,motor2.rmp);
				if(motor1.distance<=20)
				{
					gimbal_state=STATE2_UNLOCK;
				}
			}								
			
			if(actionway==4)
			{
				motor1.pwm_out=400;
				if(motor1.distance<=100)
				motor1.pwm_out=10*(motor1.distance-60);	
				if(motor1.distance<=20)
				{
					event=0;
					actionflag=0;
					gimbal_state=STATE_STOP;
				}
			
				
				
				
			}
		break;
		case STATE2_UNLOCK:																																			//UNLOCK
			if(actionway==1)
			{
				
			}
			if(actionway==2)
			{
				motor1.pwm_out=PID_calc(&hang_pid_j, motor1.distance,0);
				motor2.rmp=PID_calc(&motor2.pid_j, motor2.distance,0);
				motor2.pwm_out=PID_calc(&motor2.pid_s, motor_chassis[motor2_3508].speed_rpm,motor2.rmp);
				delaycount++;
				if(delaycount<500)
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1500+delaycount);
		
				if(delaycount>500)
				{
					
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2000);
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,2500-delaycount);
				}	
				if(delaycount>1500&&delaycount<2000)
				{
					if(bodancount++<1)
					{
						roll_set_jiao+=36;
					}
				}
				if(delaycount>2000)
				{
					delaycount=0;
					bodancount=0;
					if(stopok==0)
					{
//						moveit=1;
						gimbal_state=STATE1_FLOOR;
					}
					if(stopok==1)
					{
						event=0;
						actionflag=0;
						stopok=0;
//						moveit=1;
						gimbal_state=STATE_STOP;
					}
				}
				
			}
			if(actionway==3)
			{
				motor1.pwm_out=PID_calc(&hang_pid_j, motor1.distance,0);
				motor2.rmp=PID_calc(&motor2.pid_j, motor2.distance,0);
				motor2.pwm_out=PID_calc(&motor2.pid_s, motor_chassis[motor2_3508].speed_rpm,motor2.rmp);
				delaycount++;
				if(delaycount<500)
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,500+delaycount);
		
				if(delaycount>500)
				{
					
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);
					__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1500-delaycount);
				}	
				if(delaycount>1000&&delaycount<1500)
				{
					if(bodancount++<1)
					{
						roll_set_jiao+=36;
					}
				}
				if(delaycount>1500)
				{
					delaycount=0;
					bodancount=0;
					if(floorflag<4)
						gimbal_state=STATE1_FLOOR;
					if(floorflag>=4)
					{
						event=0;
						actionflag=0;
						floorflag=0;
						gimbal_state=STATE_STOP;
					}
				}
			}
			if(actionway==4)
			{
				
				
				
				
			}
		break;
	}



}
	
	


























