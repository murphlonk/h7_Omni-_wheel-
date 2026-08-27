#ifndef MOTOR_H
#define MOTOR_H

#define    TORQUE_CONSTANT 0.3f//Torque_constant()

#include "stdint.h"
#include "bsp_fdcan.h"
#include "PID_T.h"

typedef struct {
    double  Angle;
    double  Speed;
    double  RollCureet;
    uint8_t TempareTure;
    uint8_t Protected;
}RealMotor_Data;

typedef struct {
    RealMotor_Data data;
    uint32_t fbID;
    uint32_t ContrlID;
    uint8_t number;
    PID_H_POS motor_contrl;
}DJMotor_hander;//because dj one pice could contrl 4 motor for max

typedef struct {
    RealMotor_Data data;
    uint32_t fbID;
    uint32_t ContrlID;
    PID_H_POS motor_contrl;
}Motor_hander;

/*
DJMotor_hander chassiss_motor_0;
DJMotor_hander chassiss_motor_1;
DJMotor_hander chassiss_motor_2;
DJMotor_hander chassiss_motor_3;
*/


void Motor_Init(DJMotor_hander * motor,PID_H_POS *motor_contrl,uint32_t fbID,uint32_t ContrlID);
void DJMotor_Init(DJMotor_hander * motor,PID_H_POS *motor_contrl,uint32_t fbID,uint32_t ContrlID,uint8_t number);
RealMotor_Data* FeedBackDataToReal_3508(RealMotor_Data* Motor,uint8_t* Rx_Data);
RealMotor_Data* FeedBackDataToReal_6020(RealMotor_Data* Motor,uint8_t* Rx_Data);
float Motor_Speedcricle(float Expected_speed,DJMotor_hander *motorhandle );
#ifdef BSP_CAN_H
void Motor_Drive_Frame( CAN_TxFrame_TypeDef *TxFrame, uint32_t CAN_ID, int16_t* ContrlData);//per frame to contrl 4 motor
void Motor_Drive_Single(DJMotor_hander* singlemotor,CAN_TxFrame_TypeDef *TxFrame ,int16_t singledata);//per frame to contrl single motor
#endif

#ifdef BSP_FDCAN_H
void Motor_Drive_Frame( FDCAN_TxFrame_TypeDef *TxFrame, uint32_t CAN_ID, int16_t* ContrlData);
void Motor_Drive_Single(DJMotor_hander* singlemotor,FDCAN_TxFrame_TypeDef *TxFrame ,int16_t singledata);

#endif

#endif
