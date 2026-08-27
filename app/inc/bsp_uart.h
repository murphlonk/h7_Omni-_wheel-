#ifndef BSP_UART_H
#define BSP_UART_H

#include "usart.h"
#include "string.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "timers.h"





//void uart_init_bsp(UART_HandleTypeDef huart,DMA_HandleTypeDef hdma_uart_rx,uint8_t* data,uint32_t size);

void uart_init_bsp(UART_HandleTypeDef* huart);

#endif
