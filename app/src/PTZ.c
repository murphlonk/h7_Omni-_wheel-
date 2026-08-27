#include "PTZ.h"
#include "remote_contrl.h"

PID_H_POS PID_PITCH;
PID_H_POS PID_YAW;

extern float roll,pitch,yaw;
extern Dr16_CH_NORMAILIZATION normal4chdata;

#ifdef BSP_CAN_H

CAN_TxFrame_TypeDef CAN_PTZ_PITCH;
CAN_TxFrame_TypeDef CAN_PTZ_YAW;

#endif

#ifdef BSP_FDCAN_H
FDCAN_TxFrame_TypeDef FDCAN_PTZ_PITCH;
FDCAN_TxFrame_TypeDef FDCAN_PTZ_YAW;
#endif

DJMotor_hander PTZ_motor_pitch;
DJMotor_hander PTZ_motor_yaw;

PTZ_handler Reverso_PTZ;

#ifdef BSP_CAN_H
void PTZcanInit()
{
	  can_Transmit_INIT(&CAN_PTZ_PITCH,&hcan2);
    can_Transmit_INIT(&CAN_PTZ_YAW  ,&hcan1);
}

void PTZ_Drive_Angle(float Targetpitch,float Targetroll)
{    int16_t pitchspeed,yawspeed; 
    pitchspeed= (Positional_PID_Compute(&PID_PITCH,Targetpitch,(float)(PTZ_motor_pitch.data.Angle)))*60*25000/6.28/320;//wait to change
    yawspeed  = (Positional_PID_Compute(&PID_YAW,Targetroll,(float)(PTZ_motor_yaw.data.Angle)))*60*25000/6.28/320;
    
     Motor_Drive_Single(&PTZ_motor_pitch,&CAN_PTZ_PITCH,pitchspeed);
     Motor_Drive_Single(&PTZ_motor_yaw  ,&CAN_PTZ_YAW  ,yawspeed  );

}

void PTZ_Drive_WR(float targetspeeds)
{
    
   Motor_Drive_Single(&PTZ_motor_yaw  ,&CAN_PTZ_YAW,(int16_t)(targetspeeds*60*25000/6.28f/320));
}
#endif

#ifdef BSP_FDCAN_H

void PTZcanInit()
{
	  
	Fdcan_Transmit_INIT(&FDCAN_PTZ_PITCH,&hfdcan1);
	Fdcan_Transmit_INIT(&FDCAN_PTZ_YAW,&hfdcan2);
}

void PTZ_Drive_Angle(float Targetpitch,float Targetroll)
{    int16_t pitchspeed,yawspeed; 
    pitchspeed= (int16_t)(Positional_PID_Compute(&PID_PITCH,Targetpitch,(float)(PTZ_motor_pitch.data.Angle)))*60*25000/6.28/320;//wait to change
    yawspeed  = (int16_t)(Positional_PID_Compute(&PID_YAW,Targetroll,(float)(PTZ_motor_yaw.data.Angle)))*60*25000/6.28/320;
    
     Motor_Drive_Single(&PTZ_motor_pitch,&FDCAN_PTZ_PITCH,pitchspeed);
     Motor_Drive_Single(&PTZ_motor_yaw  ,&FDCAN_PTZ_YAW  ,yawspeed  );

}

void get_relativeangle(Angles*angle)
{

  angle->PITCH=PTZ_motor_pitch.data.Angle-PITCH_FRIST;
  angle->YAW  =PTZ_motor_yaw.data.Angle-ROLL_FRIST;
}

void PTZ_Drive_WR(float targetspeeds)
{
    
   Motor_Drive_Single(&PTZ_motor_yaw  ,&FDCAN_PTZ_YAW,(int16_t)(targetspeeds*60*25000/6.28f/320));
}

void PTZ_DISABLE()
{
      Motor_Drive_Single(&PTZ_motor_pitch,&FDCAN_PTZ_PITCH, 0);
      Motor_Drive_Single(&PTZ_motor_yaw  ,&FDCAN_PTZ_YAW  , 0);

}

#endif

void PTZ_Init(PTZ_handler * ptz)//the value of pid needed to init out of this one function 
{
    //get the imudata to init
    //get the of the motor feedback to fill the Relative_chassiss_slove;
    ptz->imudata.PITCH=pitch*3.14f/180;
    ptz->imudata.YAW =yaw*3.14f/180;
    PTZcanInit();
    get_relativeangle(&ptz->Relative_chassiss_slove);
    Positional_PID_Init(&PID_PITCH,0.45f,0.0,0.0,2.0,1.34);
    Positional_PID_Init(&PID_YAW,0.45f,0.0,0.0,2.0,1.34);
    DJMotor_Init(&PTZ_motor_pitch,&PID_PITCH,PITCHMOTORID_FB,PITCHMOTORID_CON,1);
    DJMotor_Init(&PTZ_motor_yaw  ,&PID_YAW  ,YAWMOTORID_FB  ,YAWMOTORID_CON  ,2);
    ptz->target.PITCH=PTZ_motor_pitch.data.Angle;//use the motor feedback ,but now data is waiting to build
    ptz->target.ROLL =0.0f;
    ptz->target.YAW  =PTZ_motor_yaw.data.Angle;

}

void PTZ_imuupdate()
{
   Reverso_PTZ.imudata.PITCH=pitch*3.14f/180;
   Reverso_PTZ.imudata.YAW=yaw*3.14f/180; 
}

float  Gravity_compensation(PTZ_handler *Gravityfix)
{
     return  QUALITY*cos(Gravityfix->imudata.PITCH);
}


void PTZ_UPDATE()
{
  PTZ_imuupdate();
  get_relativeangle(&Reverso_PTZ.Relative_chassiss_slove);
}

void PTZ_MIXdata_gyrodrive(float WR,float Targetpitch,float Targetroll)
{
     int16_t pitchspeed,yawspeed; 
    pitchspeed= (int16_t)(Positional_PID_Compute(&PID_PITCH,Targetpitch,(float)(PTZ_motor_pitch.data.Angle))
                 +Gravity_compensation(&Reverso_PTZ))*60*25000/6.28/320;//wait to change
    yawspeed  = (int16_t)(Positional_PID_Compute(&PID_YAW,Targetroll,(float)(PTZ_motor_yaw.data.Angle))+WR)*60*25000/6.28/320;
    
    Motor_Drive_Single(&PTZ_motor_pitch,&FDCAN_PTZ_PITCH,pitchspeed);
    Motor_Drive_Single(&PTZ_motor_yaw  ,&FDCAN_PTZ_YAW  ,yawspeed  );

}

void PTZ_static_drive(float Targetpitch,float Targetroll)
{
   int16_t pitchspeed,yawspeed; 
    pitchspeed= (int16_t)(Positional_PID_Compute(&PID_PITCH,Targetpitch,(float)(pitch))
                 +Gravity_compensation(&Reverso_PTZ))*60*25000/6.28/320;//wait to change
    yawspeed  = (int16_t)(Positional_PID_Compute(&PID_YAW,Targetroll,(float)(yaw)))*60*25000/6.28/320;
    
    Motor_Drive_Single(&PTZ_motor_pitch,&FDCAN_PTZ_PITCH,pitchspeed);
    Motor_Drive_Single(&PTZ_motor_yaw  ,&FDCAN_PTZ_YAW  ,yawspeed  );

}




void PTZTask02(void *argument)
{

  PTZ_Init(&Reverso_PTZ);
  osThreadFlagsWait(0x00000002,osFlagsWaitAll,osWaitForever);
  for(;;)
  { uint32_t orderflag;
    orderflag=osThreadFlagsGet();
	  if(orderflag&0x00000002)
   {

    PTZ_UPDATE();
    if(orderflag&0x00000020)
    {
       
      PTZ_MIXdata_gyrodrive(normal4chdata.ch3*4,normal4chdata.ch0,normal4chdata.ch1);
     
    }else if(orderflag&0x00000010)
    {
      PTZ_static_drive(normal4chdata.ch0,normal4chdata.ch1);
    }


   }else
   {
    PTZ_DISABLE();
   }
     
    osThreadFlagsClear(0x7FFFFFFF);
		
    osDelay(1);
  }
 

}


