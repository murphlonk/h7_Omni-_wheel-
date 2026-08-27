#include "bsp_uart.h"
#include "remote_contrl.h"


extern  DMA_HandleTypeDef hdma_uart5_rx;

__attribute__((section(".ARM.__at_0x38000000"))) uint8_t data_temp_uart[BUFF_SIZE];
//extern  osSemaphoreId_t isdisconnect;



/*
void uart_init_bsp(UART_HandleTypeDef huart,DMA_HandleTypeDef hdma_uart_rx,uint8_t* data,uint32_t size)
{

    HAL_UARTEx_ReceiveToIdle_DMA(&huart,data,size);
	__HAL_DMA_DISABLE_IT(&hdma_uart_rx, DMA_IT_HT);
    //Remote_Contrl_Init();//self add f 
}*/

void uart_init_bsp(UART_HandleTypeDef* huart)
{

    HAL_UARTEx_ReceiveToIdle_DMA(huart,data_temp_uart,BUFF_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
    //Remote_Contrl_Init();//self add f 
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == UART5)
	{		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, data_temp_uart,50);
		
		
        dr16_update(data_temp_uart);
		__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
		//osSemaphoreRelease(isdisconnect);
       // lever_Status_();
   }

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == UART5)
	{
		__HAL_UNLOCK(huart);
		memset(data_temp_uart, 0, BUFF_SIZE);	//为什么以前版本并不会频繁进这个这个中断，我只好不让消除数据来保证数据存在？						   		
		HAL_UARTEx_ReceiveToIdle_DMA(huart, data_temp_uart,BUFF_SIZE);
	}
}


