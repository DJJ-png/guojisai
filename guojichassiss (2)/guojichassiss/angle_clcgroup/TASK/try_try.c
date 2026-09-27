#include "main.h"
#include "INS_Task.h"
#include "try_try.h"
#include "FreeRTos.h"
#include "cmsis_os.h"
#include "AHRS.h"
#include "bsp_can.h"
#include "chassiss_calc.h"
#include "FSM_task.h"
#include "motor_3508_task.h"
extern osThreadId try_tryHandle;
extern state_chassiss_list chassiss_state;
uint8_t flag;
extern current_f current;
extern fp32 current_gimbal;
void Try_Try(void const * argument){

	while(1){
			if(chassiss_state!=3){
			CAN_CMD_BASE(&hcan1,0x200,current.current_0,current.current_1,current.current_2,current.current_3);
			vTaskDelay(1);	
			CAN_cmd_chassis(motorbo.pwm_out,motor1.pwm_out,motor2.pwm_out,0);
			}
		  vTaskDelay(1);
			 
	}
}