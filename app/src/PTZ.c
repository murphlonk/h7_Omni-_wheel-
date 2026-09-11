#include "PTZ.h"
#include "remote_contrl.h"

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

void get_relativeangle(Angles*angle)
{

  angle->PITCH=PTZ_motor_pitch.data.Angle-PITCH_FRIST;
  angle->YAW  =PTZ_motor_yaw.data.Angle  -  YAW_FRIST;
	if(angle->YAW>3.14f)
  {
		angle->YAW-=6.28;
	}
//	  angle->YAW  =(yaw/360*6.28)-  YAW_FRIST;
//	if(angle->YAW>3.14f)
//  {
//		angle->YAW-=6.28;
//	}
}

float  Gravity_compensation(PTZ_handler *Gravityfix)
{
     return  QUALITY*cos(Gravityfix->imudata.PITCH);
}

#ifdef BSP_CAN_H
void PTZcanInit()
{
	  can_Transmit_INIT(&CAN_PTZ_PITCH,&hcan2);
    can_Transmit_INIT(&CAN_PTZ_YAW  ,&hcan1);
}

void PTZ_Drive_Angle(float Targetpitch,float Targetroll)
{    int16_t pitchspeed,yawspeed; 
    pitchspeed= (PID_calc(&PTZ_motor_pitch.motor_contrl,(float)(PTZ_motor_pitch.data.Angle),Targetpitch))*60*25000/6.28/320;//wait to change
    yawspeed  = (PID_calc(&PTZ_motor_yaw.motor_contrl,(float)(PTZ_motor_yaw.data.Angle),Targetroll))*60*25000/6.28/320;
    
     Motor_Drive_Single(&PTZ_motor_pitch,&CAN_PTZ_PITCH,pitchspeed);
     Motor_Drive_Single(&PTZ_motor_yaw  ,&CAN_PTZ_YAW  ,yawspeed  );

}



void PTZ_Drive_WR(float targetspeeds)
{
    
   Motor_Drive_Single(&PTZ_motor_yaw  ,&CAN_PTZ_YAW,(int16_t)(targetspeeds*60*25000/6.28f/320));
}

void PTZ_DISABLE()
{
      Motor_Drive_Single(&PTZ_motor_pitch,&CAN_PTZ_PITCH, 0);
      Motor_Drive_Single(&PTZ_motor_yaw  ,&CAN_PTZ_YAW  , 0);
      PID_clear(&PTZ_motor_pitch.motor_contrl);
      PID_clear(&PTZ_motor_yaw.motor_contrl  );
}

void PTZ_MIXdata_gyrodrive(float WR,float Targetpitch,float Targetroll)
{
     int16_t pitchspeed,yawspeed; 
    pitchspeed= (int16_t)(PID_calc(&PTZ_motor_pitch.motor_contrl,(float)(PTZ_motor_pitch.data.Angle),Targetpitch))
                 +Gravity_compensation(&Reverso_PTZ))*60*25000/6.28/320;//wait to change
    yawspeed  = (int16_t)(PID_calc(&PTZ_motor_yaw.motor_contrl,(float)(PTZ_motor_yaw.data.Angle),Targetroll)+WR)*60*25000/6.28/320;
    
    Motor_Drive_Single(&PTZ_motor_pitch,&CAN_PTZ_PITCH,pitchspeed);
    Motor_Drive_Single(&PTZ_motor_yaw  ,&CAN_PTZ_YAW  ,yawspeed  );

}

void PTZ_static_drive(float Targetpitch,float Targetroll)
{
   int16_t pitchspeed,yawspeed; 
    pitchspeed= (int16_t)(PID_calc(&PTZ_motor_pitch.motor_contrl,(float)(pitch),Targetpitch)
                 +Gravity_compensation(&Reverso_PTZ))*60*25000/6.28/320;//wait to change
    yawspeed  = (int16_t)(PID_calc(&PTZ_motor_yaw.motor_contrl,(float)(yaw),Targetroll))*60*25000/6.28/320;
    
    Motor_Drive_Single(&PTZ_motor_pitch,&CAN_PTZ_PITCH,pitchspeed);
    Motor_Drive_Single(&PTZ_motor_yaw  ,&CAN_PTZ_YAW  ,yawspeed  );

}
#endif

#ifdef BSP_FDCAN_H

void PTZcanInit()
{
	  
	Fdcan_Transmit_INIT(&FDCAN_PTZ_PITCH,&hfdcan2);
	Fdcan_Transmit_INIT(&FDCAN_PTZ_YAW,&hfdcan1);
}

void PTZ_Drive_Angle(float Targetpitch,float Targetroll)
{    int16_t pitchspeed,yawspeed; 
    pitchspeed= (int16_t)(PID_calc(&PTZ_motor_pitch.motor_contrl,(float)(PTZ_motor_pitch.data.Angle),Targetpitch)*60*25000/6.28/320);//wait to change
    yawspeed  = (int16_t)(PID_calc(&PTZ_motor_yaw.motor_contrl,(float)(PTZ_motor_yaw.data.Angle),Targetroll)*60*25000/6.28/320);
    
     Motor_Drive_Single(&PTZ_motor_pitch,&FDCAN_PTZ_PITCH,pitchspeed);
     Motor_Drive_Single(&PTZ_motor_yaw  ,&FDCAN_PTZ_YAW  ,yawspeed  );

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

void PTZ_MIXdata_gyrodrive(float WR,float Targetpitch,float Targetyaw)
{
     int16_t pitchspeed,yawspeed; 
    pitchspeed= (int16_t)(((PID_calc(&PTZ_motor_pitch.motor_contrl,(float)(PTZ_motor_pitch.data.Angle),Targetpitch))
                 +Gravity_compensation(&Reverso_PTZ))*60*25000/6.28/320);//wait to change
    //yawspeed  = (int16_t)(PID_calc(&PTZ_motor_yaw.motor_contrl,(float)(PTZ_motor_yaw.data.Angle),Targetroll)+WR)*60*25000/6.28/320;
    yawspeed  = (int16_t)((PID_calc(&PTZ_motor_yaw.motor_contrl,(float)(PTZ_motor_yaw.data.Angle  -  YAW_FRIST),Targetyaw)+WR)*60*25000/6.28/320);
    Motor_Drive_Single(&PTZ_motor_pitch,&FDCAN_PTZ_PITCH,pitchspeed);
    Motor_Drive_Single(&PTZ_motor_yaw  ,&FDCAN_PTZ_YAW  ,yawspeed  );

}


float motoranglenow=0;
float motoranglenowraw=0;
float targetyaw=0;
float deleta=0;

void PTZ_static_drive(float Targetpitch,float Targetyaw)
{
   int16_t pitchspeed=0,yawspeed=0;
	 targetyaw=Targetyaw;
  // float  motoranglenow=0;	
     pitchspeed= (int16_t)((PID_calc(&PTZ_motor_pitch.motor_contrl,(float)(pitch),(Targetpitch)))
                 +Gravity_compensation(&Reverso_PTZ))*60*25000/6.28/320;//wait to change
     //yawspeed  = (int16_t)(PID_calc(&PTZ_motor_yaw.motor_contrl,(float)(yaw),Targetroll))*60*25000/6.28/320;
		 motoranglenow=Reverso_PTZ.Relative_chassiss_slove.YAW ;
		 motoranglenowraw=Reverso_PTZ.Relative_chassiss_slove.YAW ;
     if(motoranglenow-Targetyaw >=3.14)
     {
      motoranglenow -=6.28;
     }else if(motoranglenow-Targetyaw<-3.14)
     {
			motoranglenow +=6.28;
		 }else
     {
			 
		 }
		 deleta=Targetyaw-motoranglenow;
     //yawspeed  = (int16_t)(PID_calc(&PTZ_motor_yaw.motor_contrl,(float)(PTZ_motor_yaw.data.Angle-YAW_FRIST),Targetyaw))/6.28*60*25000/320;
		 yawspeed  = (int16_t)((PID_calc(&PTZ_motor_yaw.motor_contrl,(float)(motoranglenow),Targetyaw))/6.28/320*60*25000);
     Motor_Drive_Single(&PTZ_motor_pitch,&FDCAN_PTZ_PITCH,pitchspeed);
     Motor_Drive_Single(&PTZ_motor_yaw  ,&FDCAN_PTZ_YAW  ,yawspeed  );

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
    //Positional_PID_Init(&PTZ_motor_pitch.motor_contrl,0.45f,0.0,0.0,2.0,1.34);
    //Positional_PID_Init(&PTZ_motor_yaw.motor_contrl  ,0.45f,0.0,0.0,2.0,1.34);
    PID_init(&PTZ_motor_pitch.motor_contrl,PID_POSITION,(fp32[]){0.45f,0.0f,0.0f},10000.0f,10000.34f);
    PID_init(&PTZ_motor_yaw.motor_contrl,PID_POSITION  ,(fp32[]){2.20f,0.03f,0.0f},10000.0f,1.34f);//good 
    DJMotor_Init(&PTZ_motor_pitch,PITCHMOTORID_FB,PITCHMOTORID_CON,1);
    DJMotor_Init(&PTZ_motor_yaw  ,YAWMOTORID_FB  ,YAWMOTORID_CON  ,0);
    ptz->target.PITCH=PTZ_motor_pitch.data.Angle;//use the motor feedback ,but now data is waiting to build
    ptz->target.ROLL =0.0f;
    ptz->target.YAW  =PTZ_motor_yaw.data.Angle;

}

void PTZ_imuupdate()
{
   Reverso_PTZ.imudata.PITCH=pitch*3.14f/180;
   Reverso_PTZ.imudata.YAW=yaw*3.14f/180; 
}




void PTZ_UPDATE()
{
  PTZ_imuupdate();
  get_relativeangle(&Reverso_PTZ.Relative_chassiss_slove);
}

uint32_t debugePTZcount=0;


void mustclearbits_PTZ()
{ if(osThreadFlagsClear(0x000000FF)==pdFALSE&&debugePTZcount%100!=0)
  { 
		debugePTZcount++;
		mustclearbits_PTZ();
	}
}

uint32_t debugorderflag=0;


void PTZTask02(void *argument)
{

  PTZ_Init(&Reverso_PTZ);
  osThreadFlagsWait(0x00000002,osFlagsWaitAll,osWaitForever);
  for(;;)
  { uint32_t orderflag;
    orderflag=osThreadFlagsGet();
		debugorderflag=orderflag;
	  if(orderflag&0x00000002)
   {

    PTZ_UPDATE();
    if(orderflag&0x00000020)
    {
       
      PTZ_MIXdata_gyrodrive(normal4chdata.ch2*6.5,normal4chdata.ch0,normal4chdata.ch1);//6.4,6.5,7
     
    }else if(orderflag&0x00000010)
    {  static bool isinit=false;
			for(uint8_t i=0;i<10&&isinit==false;i++)
			{
			PTZ_MIXdata_gyrodrive(0,normal4chdata.ch0,normal4chdata.ch1);//6.4,6.5,7
				if(i==9){isinit=true;}
			}
      PTZ_static_drive(normal4chdata.ch1,(normal4chdata.ch2*6.28));
    }


   }else
   {
    //PTZ_DISABLE();
   }
     
    //osThreadFlagsClear(0x7FFFFFFF);
		mustclearbits_PTZ();
	 
	 
    osDelay(1);
  }
 

}


