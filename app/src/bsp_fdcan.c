#include "bsp_fdcan.h"

#include "motor.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"

uint8_t rx_candata[50];

extern DJMotor_hander PTZ_motor_pitch;
extern DJMotor_hander PTZ_motor_yaw;
extern DJMotor_hander chassiss_motor[4];

void Fdcan_FilterInit(FDCAN_HandleTypeDef *hfdcan)
{
 
FDCAN_FilterTypeDef FDCAN_FilterConfig;

FDCAN_FilterConfig.IdType = FDCAN_STANDARD_ID; 
FDCAN_FilterConfig.FilterIndex = 0;             
FDCAN_FilterConfig.FilterType = FDCAN_FILTER_MASK; 
FDCAN_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
FDCAN_FilterConfig.FilterID1 = 0x00000000; 
FDCAN_FilterConfig.FilterID2 = 0x00000000; 

HAL_FDCAN_ConfigFilter(hfdcan, &FDCAN_FilterConfig);
HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);	
HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
HAL_FDCAN_Start(hfdcan);
}



void Fdcan_Transmit_INIT(FDCAN_TxFrame_TypeDef* hfdcanTxFrame,FDCAN_HandleTypeDef *hfdcan)//暂时记不住先进行注释
{
      hfdcanTxFrame->hcan = hfdcan;                         
      hfdcanTxFrame->Header.IdType = FDCAN_STANDARD_ID;         
      hfdcanTxFrame->Header.TxFrameType = FDCAN_DATA_FRAME;  
      hfdcanTxFrame->Header.DataLength=FDCAN_DLC_BYTES_8	; 
      hfdcanTxFrame->Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE; 
      hfdcanTxFrame->Header.BitRateSwitch = FDCAN_BRS_OFF;
      hfdcanTxFrame->Header.FDFormat = FDCAN_CLASSIC_CAN;      
      hfdcanTxFrame->Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
      hfdcanTxFrame->Header.MessageMarker = 0;                  
}

void Fdcan_Transmit(FDCAN_TxFrame_TypeDef *TxFrame)
{
	HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan, &TxFrame->Header, TxFrame->Data);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ FDCAN_RxHeaderTypeDef rx_header_can1;
	FDCAN_RxHeaderTypeDef rx_header_can2;
  if(hfdcan==&hfdcan1)
{ 
	 HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header_can1, rx_candata);
    switch (rx_header_can1.Identifier)
    {
    case 0x201:
    FeedBackDataToReal_3508(&chassiss_motor[0].data,rx_candata);
      break;
    case 0x202:
    FeedBackDataToReal_3508(&chassiss_motor[1].data,rx_candata);
      break;
    case 0x203:
    FeedBackDataToReal_3508(&chassiss_motor[2].data,rx_candata);
      break;
    case 0x204:
    FeedBackDataToReal_3508(&chassiss_motor[3].data,rx_candata);
      break;
    case 0x205:
     FeedBackDataToReal_6020(&PTZ_motor_yaw.data   ,rx_candata);
      break;
		case 0x213:
			
      break;			
		default:
      break;
    }
	//HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, Rx_data_3508);
	HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    /*self interrupt slove logic*/  
}else if (hfdcan==&hfdcan2)
	{ HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header_can2, rx_candata);
    switch (rx_header_can2.Identifier)
    {
    
    case 0x206:
     FeedBackDataToReal_6020(&PTZ_motor_pitch.data   ,rx_candata);
      break;
    default:
      break;
    }
	//HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, Rx_data_3508);
	HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    /*self interrupt slove logic*/  
  }
}
  void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)//RxFifo0ITs是接收寄存器的值
{ 
	FDCAN_RxHeaderTypeDef rx_header;

 if(hfdcan==&hfdcan1)
   {
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_candata);
    switch (rx_header.Identifier)
    {
    case 0x201:
      /* code */
      break;
    case 0x202:
      /* code */
      break;
    case 0x203:
      /* code */
      break;
    case 0x204:
      /* code */
      break;
    case 0x205:
      /* code */
      break;
    default:
      break;
    }
	//HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, Rx_data_3508);
	HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    /*self interrupt slove logic*/  
   }else if(hfdcan==&hfdcan2)
   {
  //HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, Rx_data_6020);  
		 
	HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    /*self interrupt slove logic*/    
}

}



void CanTask04(void *argument)
{
  osThreadFlagsWait(0x00000002,osFlagsWaitAll,osWaitForever);
  for(;;)
  {
     static uint32_t orderflag;
	  orderflag=osThreadFlagsGet();
    osThreadFlagsClear(0xFFFFFFFF);
  }
}


