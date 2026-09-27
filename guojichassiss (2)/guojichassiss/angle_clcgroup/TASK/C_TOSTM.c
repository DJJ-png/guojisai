#include "main.h"
#include "FreeRTos.h"
#include "cmsis_os.h"
#include "AHRS.h"
#include "chassiss_calc.h"
#include "math.h"
#include "CAN_receive.h"
#include "FSM_task.h"
#include "gimbal_calc.h"
#include "string.h"

#define usart_dma_rx_len 256
#define usart_dma_rx 17
extern osThreadId C_TOSTMHandle;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint16_t USART1_Rx_BUF[usart_dma_rx_len];
uint16_t eight_data_front[8];
uint16_t eight_data_back[8];

void USART1_RX(void);

void USART1_RX(){
		if(huart1.Instance == USART1){
				if(USART1_Rx_BUF[0]==0xA5){
					for(int i=0;i<7;i++){
					eight_data_front[i] = USART1_Rx_BUF[i+1];
					}
					for(int i=0;i<7;i++){
					eight_data_back[i] = USART1_Rx_BUF[i+9];
					}
					HAL_UART_Receive_DMA(&huart1, USART1_Rx_BUF, usart_dma_rx);
				}
				else{
					__HAL_UART_CLEAR_PEFLAG(&huart1);//清标志位
					__HAL_DMA_DISABLE(huart1.hdmarx);//dma失能
					__HAL_DMA_SET_COUNTER(&hdma_usart1_rx, usart_dma_rx);
					memset(USART1_Rx_BUF, 0, sizeof(USART1_Rx_BUF));
					HAL_UART_Receive_DMA(&huart1, USART1_Rx_BUF, usart_dma_rx);
					__HAL_DMA_ENABLE(huart1.hdmarx);
				}
		}
}

void c_tostm(void const * argument){
//	HAL_UART_Receive_DMA(&huart1, USART1_Rx_BUF, usart_dma_rx);
	while(1){
		vTaskDelay(1);
	}
}