#ifndef CHASSISS_CALC__H
#define CHASSISS_CALC__H
#include "pid.h"
#define SPEED_RPM 100
    typedef struct{
            float speed_0;
            float speed_1;
            float speed_2;
            float speed_3;
						float speed_vechicle;
    }speed_f;
    
        typedef struct{
            float current_0;
            float current_1;
            float current_2;
            float current_3;
    }current_f;

        typedef struct{
            float motor_0;
            float motor_1;
            float motor_2;
            float motor_3;
    }motor_f;
				
				typedef struct{
					float cnt_current;
					float cnt_past;
					float cnt_delta;
					float times;
					float per_lenth;
					float road;
				}road_f;
		typedef struct{
			uint16_t meast;//measurement,测量值
			uint8_t is_whiteorblack;//0为白，1为黑
			float k;//权重系数
			float err;//偏离值,为各传感器值乘以权重系数之和（当八路灰度测得为黑的时候）
		}eight_road;		
				
		extern current_f current;
		extern fp32 set_yaw;
		extern fp32 set_v;
#endif