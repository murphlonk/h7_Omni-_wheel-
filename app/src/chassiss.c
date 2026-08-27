#include "chassiss.h"
#include "remote_contrl.h"
#include "PTZ.h"

extern Dr16_CH_NORMAILIZATION normal4chdata;
 
extern PTZ_handler Reverso_PTZ;
extern DJMotor_hander PTZ_motor_yaw;

DJMotor_hander chassiss_motor[4];
 /*DJMotor_hander chassiss_motor_1;
 DJMotor_hander chassiss_motor_2;
 DJMotor_hander chassiss_motor_3;
*/
#ifdef BSP_CAN_H

CAN_TxFrame_TypeDef Chassiss_CAN;

#endif

#ifdef BSP_FDCAN_H 

FDCAN_TxFrame_TypeDef chassis_fdcan;

#endif
 
PID_H_POS chassissmotor_contrl[4];
/*PID_H_POS chassissmotor_contrl_1;
PID_H_POS chassissmotor_contrl_2;
PID_H_POS chassissmotor_contrl_3;
*/
//extern DJMotor_hander PTZ_motor_yaw;

chassis_handler Reverso_Chassiss;

void Chassis_status_Init(chassiss_status* status)
{ 
    status->phase_difference=PTZ_motor_yaw.data.Angle-HEAD_CHASSANGLE;//waiting the PTZ yaw value
    status->VX=0.0f;
    status->VY=0.0f;
    status->W =0.0f;
} 

//void chassiss_DJMotors_Init(DJMotor_hander * motors)
//{
//    DJMotor_Init(&motors[0],&chassissmotor_contrl[0],0x201,0x200,1);//waiting to measure
//    DJMotor_Init(&motors[1],&chassissmotor_contrl[1],0x202,0x200,2);
//    DJMotor_Init(&motors[2],&chassissmotor_contrl[2],0x203,0x200,3);
//    DJMotor_Init(&motors[3],&chassissmotor_contrl[3],0x204,0x200,4);
//}

void chassiss_DJMotors_Init()
{
    DJMotor_Init(&chassiss_motor[0],&chassissmotor_contrl[0],0x201,0x200,1);//waiting to measure
    DJMotor_Init(&chassiss_motor[1],&chassissmotor_contrl[1],0x202,0x200,2);
    DJMotor_Init(&chassiss_motor[2],&chassissmotor_contrl[2],0x203,0x200,3);
    DJMotor_Init(&chassiss_motor[3],&chassissmotor_contrl[3],0x204,0x200,4);
}


void Chassiss_power_Init(Power_K* POWERS)
{
    POWER_METER_INIT(&POWERS[0],0.0f,0.0f,0.0f,0.0f);
    POWER_METER_INIT(&POWERS[1],0.0f,0.0f,0.0f,0.0f);
    POWER_METER_INIT(&POWERS[2],0.0f,0.0f,0.0f,0.0f);
    POWER_METER_INIT(&POWERS[3],0.0f,0.0f,0.0f,0.0f);
}

void chassiss_Motors_Init(Motor_hander* motors)//waiting to add
{
    
}

#ifdef BSP_FDCAN_H

void chassiscan_init(void)
{
	Fdcan_Transmit_INIT(&chassis_fdcan,&hfdcan1);
}

  void Chassiss_Drive(float * targetspeed,DJMotor_hander * motordata,FDCAN_TxFrame_TypeDef *Txframe)//speed0/1/2 is vx,vy,wr
{
    float speedsloved[4];
    int16_t speedinto[4];
    Chassiss_Slove(targetspeed[0],targetspeed[1],targetspeed[2],speedsloved);
    for(uint8_t i=0;i<4;i++)
      { 
        //speedinto[i] = (int16_t)(Positional_PID_Compute(&motordata[i].motor_contrl,speedsloved[i],motordata[i].data.Speed)/6.0f*16384);
        speedinto[i]=(int16_t)(speedsloved[i]/6*16384);
			}

     //POWER_METER_COMPUTE_PER(rollcrrent, float speed,Power_K* Kvalues);
	   
      //Motor_Drive_Frame( &chassis_fdcan,motordata[0].ContrlID,speedinto);//contrldata is the arry of contrl value
      Motor_Drive_Frame(Txframe,motordata[0].ContrlID,speedinto);//contrldata is the arry of contrl value
}

#endif


void Chassiss_Init(chassis_handler * chassiss)//the motor pid and POWER_K modefiy out of this function
{
    chassiscan_init();
    //chassiss_DJMotors_Init(chassiss->motors);
	   chassiss_DJMotors_Init();
    Chassis_status_Init(&chassiss->status);
    Chassiss_power_Init(chassiss->chassisspower);
}

float* Chassiss_Slove(float VX,float VY,float WR,float* speed)
{
   
   speed[3] = ( 0.7071*VX-0.7071*VY+WR*CAR_L)/WHEEL_S; 
   speed[1] = (-0.7071*VX-0.7071*VY+WR*CAR_L)/WHEEL_S; 
   speed[2] = ( 0.7071*VX+0.7071*VY+WR*CAR_L)/WHEEL_S; 
   speed[0] = (-0.7071*VX+0.7071*VY+WR*CAR_L)/WHEEL_S; 
   return speed;
}

chassiss_status* Chassiss_Inverse_Slove(float * speeds,chassiss_status * status )
{
    //status->phase_difference=
      status->VY = WHEEL_S*(speeds[0]-speeds[1])/0.7071;
      status->VX = WHEEL_S*(speeds[2]-speeds[1])/0.7071;
      status->W  = WHEEL_S*(speeds[1]+speeds[3])/CAR_L;
      return  status;
}

void chassis_relative_ptzupdate()
{

   Reverso_Chassiss.status.phase_difference=Reverso_PTZ.Relative_chassiss_slove.YAW;
}
void  speedptztochassis(float *ptzvx,float *ptzvy)
{

   
   float chavx = cos(Reverso_Chassiss.status.phase_difference)*(*ptzvx)
                -sin(Reverso_Chassiss.status.phase_difference)*(*ptzvy);
   float chavy = sin(Reverso_Chassiss.status.phase_difference)*(*ptzvx)
                +cos(Reverso_Chassiss.status.phase_difference)*(*ptzvy);
  *ptzvx=chavx;*ptzvy=chavy; 
}

void chassiss_disabled()
{
 
    int16_t speedinto[4]={0,0,0,0};
    //POWER_METER_COMPUTE_PER(rollcrrent, float speed,Power_K* Kvalues);
    Motor_Drive_Frame( &chassis_fdcan,chassiss_motor[0].ContrlID,speedinto);
}



#ifdef BSP_CAN_H
void chassiscan_init()
{
	can_Transmit_INIT(&Chassiss_CAN,&hcan2);
}


 void Chassiss_Drive(float * speed,DJMotor_hander * motordata,CAN_TxFrame_TypeDef *Txframe)//speed0/1/2 is vx,vy,wr
{
    float speedsloved[4];
    int16_t speedinto[4];
    Chassiss_Slove(speed[0],speed[1],speed[2],speedsloved);
    for(uint8_t i=0;i<4;i++)
      { 
        speedinto[i] = (int16_t)(Positional_PID_Compute(&motordata[i].motor_contrl,speedsloved[i],motordata[i].data.Speed)/6.0f*16384);
      }

     //POWER_METER_COMPUTE_PER(rollcrrent, float speed,Power_K* Kvalues);
      //Motor_Drive_Frame( &Chassiss_CAN,motordata[0].ContrlID,speedinto);//contrldata is the arry of contrl value
    Motor_Drive_Frame(Txframe,motordata[0].ContrlID,speedinto);//contrldata is the arry of contrl value
}

#endif


uint32_t debugflag=0;


void ChassisTask03(void *argument)
{
	  Fdcan_FilterInit(&hfdcan1);
    Positional_PID_Init(&chassissmotor_contrl[0],0,0.0f,0.0f,100,1.45);
    Positional_PID_Init(&chassissmotor_contrl[1],0,0.0f,0.0f,100,1.45);
    Positional_PID_Init(&chassissmotor_contrl[2],0,0.0f,0.0f,100,1.45);
    Positional_PID_Init(&chassissmotor_contrl[3],0.14,0.56f,0.0f,100000,145);
    Chassiss_Init(&Reverso_Chassiss);
    osThreadFlagsWait(0x00000002,osFlagsWaitAll,osWaitForever);
  for(;;)
  {
    static uint32_t orderflag;
	  orderflag=osThreadFlagsGet();
    debugflag= osThreadFlagsGet();
		
   if(orderflag&0x00000002)
   {
    chassis_relative_ptzupdate();
    if(orderflag&0x00000020)//gyro
    {
      float target[3];
      target[0]=normal4chdata.ch0;//vx
      target[1]=normal4chdata.ch1;//vy
      target[2]=normal4chdata.ch2*4;//wr
    speedptztochassis(&target[0],&target[1]);
    Chassiss_Drive(target,chassiss_motor,&chassis_fdcan);
    }
    else if (orderflag&0x00000010)//with
    {
     float target[3];
    target[0]=normal4chdata.ch0;
    target[1]=normal4chdata.ch1;
    target[2]=1.2f*Reverso_Chassiss.status.phase_difference;//
    speedptztochassis(&target[0],&target[1]);
    Chassiss_Drive(target,chassiss_motor,&chassis_fdcan);
    }

   }else
   {
    
    chassiss_disabled();
   }

    osThreadFlagsClear(0x7FFFFFFF);


    osDelay(1);
  }
  
}



