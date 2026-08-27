#include "motor.h"

///*
//DJMotor_hander chassiss_motor_0;
//DJMotor_hander chassiss_motor_1;
//DJMotor_hander chassiss_motor_2;
//DJMotor_hander chassiss_motor_3;
//*/


void DJMotor_Init(DJMotor_hander * motor,PID_H_POS *motor_contrl,uint32_t fbID,uint32_t ContrlID,uint8_t number)
{
   motor->motor_contrl = *motor_contrl;
   motor->fbID =fbID;
   motor->number=number;
   motor->ContrlID=ContrlID;
   
}


void Motor_Init(DJMotor_hander * motor,PID_H_POS *motor_contrl,uint32_t fbID,uint32_t ContrlID)
{
   motor->motor_contrl = *motor_contrl;
   motor->fbID =fbID;
   motor->ContrlID=ContrlID;
}


RealMotor_Data* FeedBackDataToReal_3508(RealMotor_Data* Motor,uint8_t* Rx_Data)
{
  Motor->Angle=(double)((int16_t)(Rx_Data[0]<<8|Rx_Data[1])/8191.0f)*6.28-3.14;
  Motor->Speed=(double)((int16_t)(Rx_Data[2]<<8|Rx_Data[3])*6.28f/60);
  Motor->RollCureet=(double)((int16_t)(Rx_Data[4]<<8|Rx_Data[5])/16384.0f)*3;
  Motor->TempareTure=Rx_Data[6];
  Motor->Protected=Rx_Data[7];
  return Motor;
}

RealMotor_Data* FeedBackDataToReal_6020(RealMotor_Data* Motor,uint8_t* Rx_Data)
{
  Motor->Angle=(double)((int16_t)(Rx_Data[0]<<8|Rx_Data[1])/8191.0f)*6.28-3.14;
  Motor->Speed=(double)((int16_t)(Rx_Data[2]<<8|Rx_Data[3])*6.28f/60);
  Motor->RollCureet=((int16_t)(Rx_Data[4]<<8|Rx_Data[5])/16384.0f)*3;
  Motor->TempareTure=Rx_Data[6];
  Motor->Protected=Rx_Data[7]; 
 return Motor;
}

float Motor_Speedcricle(float Expected_speed,DJMotor_hander *motorhandle )//
{
    return Positional_PID_Compute(&(motorhandle->motor_contrl),Expected_speed,(motorhandle->data.Speed/60*6.28)); 
}







#ifdef BSP_CAN_H

void Motor_Drive_Frame( CAN_TxFrame_TypeDef *TxFrame,uint32_t CAN_ID, int16_t* ContrlData)//contrldata is the arry of contrl value
{
    TxFrame->Header.StdId = CAN_ID;
    TxFrame->Data[0] = (ContrlData[0] >> 8);
    TxFrame->Data[1] = (ContrlData[0]);
    TxFrame->Data[2] = (ContrlData[1] >> 8);
    TxFrame->Data[3] = (ContrlData[1]);
    TxFrame->Data[4] = (ContrlData[2] >> 8);
    TxFrame->Data[5] = (ContrlData[2]);
    TxFrame->Data[6] = (ContrlData[3] >> 8);
    TxFrame->Data[7] = (ContrlData[3]);
	CanTransmit(TxFrame);
}


void Motor_Drive_Single(DJMotor_hander* singlemotor,CAN_TxFrame_TypeDef *TxFrame,int16_t singledata)
{
   for(uint8_t i=0;i<8;i++)
   {
    TxFrame->Data[i]=0;
   }
    TxFrame->Data[singlemotor->number  ]=(singledata>>8);
    TxFrame->Data[singlemotor->number+1]=singledata;
    CanTransmit(TxFrame);
}

#endif


#ifdef BSP_FDCAN_H

void Motor_Drive_Frame( FDCAN_TxFrame_TypeDef *TxFrame, uint32_t CAN_ID, int16_t* ContrlData)
{
    TxFrame->Header.Identifier = CAN_ID;
    TxFrame->Data[0] = (ContrlData[0] >> 8);
    TxFrame->Data[1] = (ContrlData[0]);
    TxFrame->Data[2] = (ContrlData[1] >> 8);
    TxFrame->Data[3] = (ContrlData[1]);
    TxFrame->Data[4] = (ContrlData[2] >> 8);
    TxFrame->Data[5] = (ContrlData[2]);
    TxFrame->Data[6] = (ContrlData[3] >> 8);
    TxFrame->Data[7] = (ContrlData[3]);
	Fdcan_Transmit(TxFrame);

}

void Motor_Drive_Single(DJMotor_hander* singlemotor,FDCAN_TxFrame_TypeDef *TxFrame,int16_t singledata)
{
   for(uint8_t i=0;i<8;i++)
   {
    TxFrame->Data[i]=0;
   }
    TxFrame->Data[singlemotor->number  ]=(singledata>>8);
    TxFrame->Data[singlemotor->number+1]=singledata;
    Fdcan_Transmit(TxFrame);
}

#endif



