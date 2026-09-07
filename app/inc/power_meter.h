#ifndef POWER_METER_H
#define POWER_METER_H

#define TORQUE_CONSTANT 0.3f
#define MAX_OFFLINETME   500
#define REMOTE_ONLINE  0x00000001
#define REMOTE_OFFLINE 0x00000000

#include "stdint.h"
#include "stdbool.h"

#include "math.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"

#include "timestamp.h"
#include "motor.h"


typedef  struct
{
   float Ka;
   float Kb;
   float const_value;
   float torque_constant;// tor and current Kvalue
}Power_K;

typedef struct
{
 float upper_limit;
 float lower_limit;
}Power_limits;

void POWER_METER_INIT(Power_K* Kvalues,float Ka,float Kb,float const_value,float torque_constant);
float POWER_METER_COMPUTE_PER(float rollcrrent, float speed,Power_K* Kvalues);//calculte the peices
float Power_caculateall(float* ecost ,uint8_t numbers);// cost all calculater
float Power_Remap_ratio(float* powersloved ,float * nowdata);
float Power_to_rollcurrent(float power,float speed,Power_K* Kvalues);//slove the
float Power_All(float* speedinto,float* speednow,Power_K* Kvalues,uint8_t numbers);
float* Power_Remap_Bigp(float* speedslove,float* nowspeeddata,uint8_t numbers,Power_limits* limits,Power_K* Kvalues);//remap the power to the new power
void task_power_mode(uint32_t order,bool onoff);//true :gyro:0x00000000/0x00000020,false:with:0x00000010/0x00000010
#endif

