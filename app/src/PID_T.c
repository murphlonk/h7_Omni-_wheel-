#include "PID_T.h"


PID_H_POS* Positional_PID_Init(PID_H_POS* pid, float Kp, float Ki, float Kd,float OutputMAX,float ErrorSumMax) 
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->error_sum  = 0.0f;
    pid->last_error = 0.0f;
    pid->OutputMAX =OutputMAX;
    pid->ErrorSumMax= ErrorSumMax;
    return pid;
}

//Positional PID
 
float Positional_PID_Compute(PID_H_POS* pid_t,float target,float nowdata)
{
 float error = target - nowdata;
 pid_t->error_sum += error ;
 //pid_t->last_error = error ;
 
 if(pid_t->error_sum > pid_t->ErrorSumMax)
 {pid_t->error_sum=pid_t->ErrorSumMax;}
 else if (pid_t->error_sum < -pid_t->ErrorSumMax)
 {pid_t->error_sum=-pid_t->ErrorSumMax;}
 float output =(pid_t->Kp * (target - nowdata) 
               + pid_t->Ki * pid_t->error_sum 
               + pid_t->Kd * (target - nowdata - pid_t->last_error));
 if(output >pid_t->OutputMAX)//负向对称限幅，这对吗？
 {output=pid_t->OutputMAX;}
 else if (output< -pid_t->OutputMAX||output< -pid_t->OutputMAX)
 {output=-pid_t->OutputMAX;}

 pid_t->last_error = error ;
 return output;
}

PID_H_POS* Positional_PID_Reset(PID_H_POS* pid) 
{ 
    pid->error_sum = 0.0f;
    pid->last_error = 0.0f;
    pid->Kp = 0.0f;
    pid->Ki = 0.0f;
    pid->Kd = 0.0f;
    return pid;
}

PID_H_POS* Positional_PID_Reset_nomal(PID_H_POS* pid)
{
    pid->error_sum = 0.0f;
    pid->last_error = 0.0f;
    return pid;
}

//Incremental PID
 
PID_H_ADD* Incremental_PID_Init(PID_H_ADD * pid_t ,float Kp, float Ki, float Kd)
{
    pid_t->Kp =Kp;
    pid_t->Ki =Ki;
    pid_t->Kd =Kd;
    pid_t->error_n =0.0f;
    pid_t->error_n_1 =0.0f;
    pid_t->error_n_2 =0.0f;
    return pid_t;
}

float Incremental_PID_Compute(PID_H_ADD* pid_t,float target,float nowdata)
{
   
    pid_t->error_n_1=pid_t->error_n;
    pid_t->error_n_2=pid_t->error_n_1;
    pid_t->error_n  = target-nowdata;
    return (pid_t->Kp*(pid_t->error_n-pid_t->error_n_1)
            +pid_t->Ki*pid_t->error_n
            +pid_t->Kd*(pid_t->error_n+pid_t->error_n_2-2*pid_t->error_n_1));
}

PID_H_ADD* Incremental_PID_Reset_normal(PID_H_ADD *pid_t)
{
    pid_t->error_n   =0.0f;
    pid_t->error_n_1 =0.0f;
    pid_t->error_n_2 =0.0f;
    return pid_t;
} 

PID_H_ADD* Incremental_PID_Reset(PID_H_ADD *pid_t)
{
    pid_t->Kp=0.0f;
    pid_t->Ki=0.0f;
    pid_t->Kd=0.0f;
    pid_t->error_n =0.0f;
    pid_t->error_n_1 =0.0f;
    pid_t->error_n_2 =0.0f;
    return pid_t;
}

