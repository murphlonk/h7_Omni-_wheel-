#ifndef POWER_METER_H
#define POWER_METER_H

#define TORQUE_CONSTANT 0.3f
#define MAX_OFFLINETME   500
#define REMOTE_ONLINE  0x00000001
#define REMOTE_OFFLINE 0x00000000

#include "stdint.h"
#include "stdbool.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"

#include "timestamp.h"

typedef  struct
{
   float Ka;
   float Kb;
   float const_value;
   float torque_constant;// tor and current Kvalue
}Power_K;


void POWER_METER_INIT(Power_K* Kvalues,float Ka,float Kb,float const_value,float torque_constant);
float POWER_METER_COMPUTE_PER(float rollcrrent, float speed,Power_K* Kvalues);//calculte the peices
float Power_caculateall(float* ecost ,uint8_t numbers);// cost all calculater

#endif

