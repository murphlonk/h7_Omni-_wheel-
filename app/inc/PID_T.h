#ifndef PID_T_H
#define PID_T_H

#include "main.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float error_sum;
    float last_error;
    float OutputMAX;
    float ErrorSumMax;
}PID_H_POS;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float error_n;
    float error_n_1;
    float error_n_2;
}PID_H_ADD;

PID_H_POS* Positional_PID_Init(PID_H_POS* pid, float Kp, float Ki, float Kd,float OutputMAX,float ErrorSumMax);
float  Positional_PID_Compute(PID_H_POS* pid_t,float target,float nowdata);
PID_H_POS* Positional_PID_Reset(PID_H_POS* pid);
PID_H_POS* Positional_PID_Reset_nomal(PID_H_POS* pid);

PID_H_ADD * Incremental_PID_Init(PID_H_ADD * pid_t ,float Kp, float Ki, float Kd);
float Incremental_PID_Compute(PID_H_ADD* pid_t,float target,float nowdata);
PID_H_ADD* Incremental_PID_Reset_normal(PID_H_ADD *pid_t);
PID_H_ADD* Incremental_PID_Reset(PID_H_ADD *pid_t);

#endif
