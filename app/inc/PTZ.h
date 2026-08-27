#ifndef PTZ_H
#define PTZ_H

#define PITCHMOTORID_CON 0x1FF
#define PITCHMOTORID_FB  0x205
#define YAWMOTORID_CON   0X1FF
#define YAWMOTORID_FB    0x206
#define QUALITY          2.0f//waitting to change

#define PITCH_FRIST 1024 //waitting to measure
#define ROLL_FRIST  1024 //waitting to measure

#include "stdint.h"//1.06,4.10
#include "stdbool.h"
#include "math.h"


#include "cmsis_os2.h"

#include "PID_T.h"
#include "motor.h"
#include "bsp_fdcan.h"




typedef struct 
{
    float PITCH;
    float ROLL;
    float YAW;
}Angles;


typedef struct {
     Angles imudata;
     Angles Relative_chassiss_slove;
     Angles target;
     PID_H_POS PTZ_PID;
}PTZ_handler;

void PTZ_Init(PTZ_handler * ptz);
void PTZ_Drive(float Targetpitch,float Targetroll);

#endif
