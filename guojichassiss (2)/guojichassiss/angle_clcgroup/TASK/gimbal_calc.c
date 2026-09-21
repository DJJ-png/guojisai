#include "main.h"
#include "FreeRTos.h"
#include "cmsis_os.h"
#include "AHRS.h"
#include "gimbal_calc.h"
#include "CAN_receive.h"
#include "chassiss_calc.h"
#include "FSM_task.h"
#include "pid.h"
#define angle_relat 0
extern osThreadId GIMBAL_CALCHandle;
extern TIM_HandleTypeDef htim8;
extern state_chassiss_list chassiss_state;

extern fp32 INS_angle_deg[3];
fp32 set_angle,set_speed;
fp32 kp_6020_s=15.0f,ki_6020_s=0.0f,kd_6020_s=300.0f;
fp32 kp_6020_p=5.0f,ki_6020_p=0.0f,kd_6020_p=10.0f;
fp32 set_angle_calc;
uint16_t time_us=0;                                                                //hx1838ÏÂ½µÑØÊ±¼ä¼ä¸ô£¨µ¥Î»ms£©
uint8_t cnt_g=0,temp=0;                                                              //½ÓÊÕÊý¾Ý
uint8_t re_buf[4]={0,0,0x45,0};
pid_type_def motor_6020_s,motor_6020_p;
data_t data={
	.reality_angle=4

};
fp32 current_gimbal;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)      //hx1838ºìÍâÒ£¿Ø£¨¼ì²âÏÂ½µÑØ£©
{
    if(htim==&htim8)
    {
        time_us=TIM8->CCR2;
        if(time_us>12000&&time_us<15000)     //135000us
        {
            cnt_g=0;
            temp=0;
        }    
        else if(time_us>900&&time_us<1300)   //1120us
        {
            cnt_g++;
            temp>>=1;
        }    
        else if(time_us>2000&&time_us<2500)   //2240us
        {
            cnt_g++;
            temp>>=1;
            temp=temp|0x80;
        }    
        else
        {
            cnt_g=0;
            temp=0;
        }
        
        if(cnt_g==8)
        {
            re_buf[0]=temp;
        }
        else if(cnt_g==16)
        {
            re_buf[1]=temp;
        }
        else if(cnt_g==24)
        {
            re_buf[2]=temp;
        }
        else if(cnt_g==32)
        {
            re_buf[3]=temp;
        }
        
        TIM8->CNT=0;
        
    }
}

void gimbal_calc(void const * argument){
	HAL_TIM_IC_Start_IT(&htim8,TIM_CHANNEL_2);
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim8,TIM_CHANNEL_2,TIM_ICPOLARITY_FALLING);
    TIM3->CNT=0;
	while(1){
	if(cnt_g==32)                                                                   //°´¼ü¾ö¶¨ËÙ¶ÈÖµ£¨cnt==32Ö¤Ã÷Êý¾ÝÊÕÂú£©
        {
            cnt_g=0;
            if((re_buf[0]==(uint8_t)~re_buf[1])&&(re_buf[2]==(uint8_t)~re_buf[3]))      //¼ì²âÊý¾ÝÎÞÎó
            {
                if(re_buf[2]==0x45)
                {
                   chassiss_state=3;
                    
                }
                else if(re_buf[2]==0x46)
                {
                    
                    
                }
                else if(re_buf[2]==0x47)
                {
                    
                    
                }
                
            }
        }
	vTaskDelay(1);
	}
}