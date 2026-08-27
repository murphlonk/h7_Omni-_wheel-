#ifndef CHASSISS_H
#define CHASSISS_H

#define CHASSISSCTRLID   0x200

#define WHEEL_S        0.1f
#define CAR_L          0.45f
#define MOTER_NUMBER   4
#define HEAD_CHASSANGLE 3.14f //the angle of the chassiss and head,waitinng to measure


#include "cmsis_os2.h"


#include "bsp_fdcan.h"

#include "motor.h"
#include "PID_T.h"
#include "power_meter.h"


typedef struct {
  float phase_difference;//the PTZ and chassiss angle value
  float VX;
  float VY;
  float W;
}chassiss_status;

typedef struct {
  DJMotor_hander motors[MOTER_NUMBER];
  chassiss_status status;
  Power_K chassisspower[MOTER_NUMBER];
}chassis_handler;

void   Chassiss_Init(chassis_handler * chassiss);
float* Chassiss_Slove(float VX,float VY,float WR,float* speed);

chassiss_status* Chassiss_Inverse_Slove(float * speeds,chassiss_status * status );
#ifdef BSP_CAN_H

void   Chassiss_Drive(float * speed,DJMotor_hander*motordata,CAN_TxFrame_TypeDef *Txframe);

#endif
#ifdef BSP_FDCAN_H
//void Chassiss_Drive(float * speed, DJMotor_hander*motordata,FDCAN_TxFrame_TypeDef *Txframe);
#endif

#endif
