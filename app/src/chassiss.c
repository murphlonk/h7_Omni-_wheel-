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

Power_limits chassisspower_limits={-20.0f,20.0f};//wiating to measure the power limit of the chassis

#ifdef BSP_CAN_H

CAN_TxFrame_TypeDef Chassiss_CAN;

#endif

#ifdef BSP_FDCAN_H 

FDCAN_TxFrame_TypeDef chassis_fdcan;

#endif

pid_type_def WRoutcircle;

//PID_H_POS chassissmotor_contrl[4];
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
    DJMotor_Init(&chassiss_motor[0],0x201,0x200,0);//waiting to measure
    DJMotor_Init(&chassiss_motor[1],0x202,0x200,1);
    DJMotor_Init(&chassiss_motor[2],0x203,0x200,2);
    DJMotor_Init(&chassiss_motor[3],0x204,0x200,3);
}


void Chassiss_power_Init(Power_K* POWERS)
{
    POWER_METER_INIT(&POWERS[0],0.0007f,0.1165f,4.4148f,0.0f);
    POWER_METER_INIT(&POWERS[1],0.0f,0.0f,0.0f,0.0f);
    POWER_METER_INIT(&POWERS[2],0.0f,0.0f,0.0f,0.0f);
    POWER_METER_INIT(&POWERS[3],0.0f,0.0f,0.0f,0.0f);
}

void chassiss_Motors_Init(Motor_hander* motors)//waiting to add
{
    
}

float* Chassiss_Slove(float VX,float VY,float WR,float* speed)
{	/*
	 speed[3] = -( 0.7071*VX-0.7071*VY+WR*CAR_L)/WHEEL_S; 
   speed[2] = -(-0.7071*VX-0.7071*VY+WR*CAR_L)/WHEEL_S; 
   speed[1] = -( 0.7071*VX+0.7071*VY+WR*CAR_L)/WHEEL_S; 
   speed[0] = -(-0.7071*VX+0.7071*VY+WR*CAR_L)/WHEEL_S; */

   speed[3] = (-0.7071*VX-0.7071*VY+WR*CAR_L)/WHEEL_S; 
   speed[2] = (-0.7071*VX+0.7071*VY+WR*CAR_L)/WHEEL_S; 
   speed[1] = ( 0.7071*VX-0.7071*VY+WR*CAR_L)/WHEEL_S; 
   speed[0] = ( 0.7071*VX+0.7071*VY+WR*CAR_L)/WHEEL_S;

   return speed;
}

float * chassiss_backslove(float * speedfb,float * speeds_hallall)
{
  //speeds_hallall[0]=(float)(0.7071*(speedfb[0]-speedfb[1])*WHEEL_S);//vx
	//speeds_hallall[1]=(float)(0.7071*(speedfb[1]-speedfb[2])*WHEEL_S);//vy
  //speeds_hallall[2]=(float)(-(speedfb[0]+speedfb[2])*WHEEL_S/CAR_L/2);//wr
	speeds_hallall[0] =(float)( WHEEL_S* ((speedfb[0] + speedfb[1]) - (speedfb[2] + speedfb[3])) / (4.0f * 0.7071f));
	speeds_hallall[1] =(float)( WHEEL_S* ((speedfb[0] - speedfb[1]) + (speedfb[2] - speedfb[3])) / (4.0f * 0.7071f));
	speeds_hallall[2] =(float)( WHEEL_S* ( speedfb[0] + speedfb[1]  +  speedfb[2] + speedfb[3] ) / (4.0f * CAR_L  ));
	return speeds_hallall;
}




#ifdef BSP_FDCAN_H

void chassiscan_init(void)
{
	Fdcan_Transmit_INIT(&chassis_fdcan,&hfdcan1);
}

//for debug write it out of box

float debugwantedvx=0;
float debugnowvx=   0;


float speedchassisssimple[3]={0,0,0};
float speedfb[4]={0,0,0,0};
float acceleration[3]={0,0,0};
float accelerationinto[4]={0,0,0,0};
int16_t accelerationintoIdata[4]={0,0,0,0};
float debugpower=0;

void Chassiss_Drive_byforce(float * targetspeed,DJMotor_hander * motordata,FDCAN_TxFrame_TypeDef *Txframe)
{
	
//	for(uint8_t i=0;i<3;i++)
//  {
//		if(targetspeed[i]<0.001&&targetspeed[i]>-0.001)
//    { 
//			targetspeed[i]=0;
//		}
//	}
	debugwantedvx=targetspeed[0];
	
  for(uint8_t i=0;i<4;i++)
  {
    speedfb[i]=motordata[i].data.Speed;
  }
  chassiss_backslove(speedfb,speedchassisssimple);//get the speed(vx,vy,wr) of the chassis by the feedback speed of the four motors
	debugnowvx=speedchassisssimple[0];
	
  acceleration[0]=PID_calc(&motordata[0].motor_contrl,speedchassisssimple[0],targetspeed[0]);//acceleration direction X//use 3 pid values in all 4 values
  acceleration[1]=PID_calc(&motordata[1].motor_contrl,speedchassisssimple[1],targetspeed[1]);//acceleration direction Y
  acceleration[2]=PID_calc(&motordata[2].motor_contrl,speedchassisssimple[2],targetspeed[2]);//acceleration direction W
  Chassiss_Slove(acceleration[0],acceleration[1],acceleration[2],accelerationinto);//get the acceleration of the four motors by the acceleration of the chassis
  for(uint8_t i=0;i<4;i++) 
  {
    accelerationintoIdata[i]=(int16_t)(accelerationinto[i]/6.0f*16384);
    if (accelerationintoIdata[i] >  16384) accelerationintoIdata[i] =  16384;
    if (accelerationintoIdata[i] < -16384) accelerationintoIdata[i] = -16384;
  }
  debugpower=Power_All(accelerationinto,(float[]) {motordata[0].data.Speed,motordata[1].data.Speed,motordata[2].data.Speed,motordata[3].data.Speed}, Reverso_Chassiss.chassisspower, 4);
  Motor_Drive_Frame(Txframe,motordata[0].ContrlID,accelerationintoIdata);
}

float debugvalue=0;

void ChassisMove_withMode(float * targets,DJMotor_hander * motordata,FDCAN_TxFrame_TypeDef *Txframe)
{
	targets[2]=(PID_calc(&chassiss_motor[3].motor_contrl,targets[2],0)*30);
	debugvalue=targets[2];
	debugwantedvx=targets[0];
	
  for(uint8_t i=0;i<4;i++)
  {
    speedfb[i]=motordata[i].data.Speed;
  }
  chassiss_backslove(speedfb,speedchassisssimple);//get the speed(vx,vy,wr) of the chassis by the feedback speed of the four motors
	debugnowvx=speedchassisssimple[0];
	
  acceleration[0]=PID_calc(&motordata[0].motor_contrl,speedchassisssimple[0],targets[0]);//acceleration direction X//use 3 pid values in all 4 values
  acceleration[1]=PID_calc(&motordata[1].motor_contrl,speedchassisssimple[1],targets[1]);//acceleration direction Y
  acceleration[2]=PID_calc(&motordata[2].motor_contrl,speedchassisssimple[2],targets[2]);//acceleration direction W
  Chassiss_Slove(acceleration[0],acceleration[1],acceleration[2],accelerationinto);//get the acceleration of the four motors by the acceleration of the chassis
  for(uint8_t i=0;i<4;i++) 
  {
    accelerationintoIdata[i]=(int16_t)(accelerationinto[i]/6.0f*16384);
    if (accelerationintoIdata[i] >  16384) accelerationintoIdata[i] =  16384;
    if (accelerationintoIdata[i] < -16384) accelerationintoIdata[i] = -16384;
  }
  Motor_Drive_Frame(Txframe,motordata[0].ContrlID,accelerationintoIdata);

}

//float speedsloved[4]={0,0,0,0};
//int16_t speedinto[4]={0,0,0,0};
//int16_t speedintolast[4]={0,0,0,0};


//void Chassiss_Drive(float * targetspeed,DJMotor_hander * motordata,FDCAN_TxFrame_TypeDef *Txframe)//speed0/1/2 is vx,vy,wr
//{
//    //float speedsloved[4]={0,0,0,0};
//    //int16_t speedinto[4]={0,0,0,0};
//    float nowspeeddata[4]={motordata[0].data.Speed,motordata[1].data.Speed,motordata[2].data.Speed,motordata[3].data.Speed};
//    Chassiss_Slove(targetspeed[0],targetspeed[1],targetspeed[2],speedsloved);
//    for(uint8_t i=0;i<4;i++)
//      { 
//        //speedinto[i] = (int16_t)((Positional_PID_Compute(&motordata[i].motor_contrl,(speedsloved[i]),(motordata[i].data.Speed))/6.0f)*16384);
//				speedinto[i] = ((PID_calc(&(motordata[i].motor_contrl),(motordata[i].data.Speed),(speedsloved[i]))/6.0f)*16384); 
//        //speedinto[i]=(int16_t)(speedsloved[i]/6*16384);
//				speedinto[i]=(int16_t)(speedintolast[i]*0.9+speedinto[i]*0.1);
//				speedintolast[i]=speedinto[i];
//          if (speedinto[i] >  16384) speedinto[i] =  16384;
//          if (speedinto[i] < -16384) speedinto[i] = -16384;
//			}
//      debugpower=Power_All(speedsloved, nowspeeddata, Reverso_Chassiss.chassisspower, 4);
////       if(Power_All(speedsloved, nowspeeddata, Reverso_Chassiss.chassisspower, 4)>1000)  
////      {Power_Remap_Bigp(speedsloved,nowspeeddata,4,&chassisspower_limits,Reverso_Chassiss.chassisspower);}
//	   
//      //Motor_Drive_Frame( &chassis_fdcan,motordata[0].ContrlID,speedinto);//contrldata is the arry of contrl value
//      Motor_Drive_Frame(Txframe,motordata[0].ContrlID,speedinto);//contrldata is the arry of contrl value
//}


float fit_power_T=0;
float fit_power_W=0;
float fit_power_pre=0;


float speedwanted=0;
float speednow=0;
float speed_a_wanted=0;
  
  void chassissmotorsigletest(float speed,DJMotor_hander * motordata,FDCAN_TxFrame_TypeDef *Txframe,uint8_t motornumber)
  {static int16_t speedintolast=0;
		int16_t speedinto=0;
    speedinto = (int16_t)((PID_calc(&(motordata[motornumber].motor_contrl),(motordata[motornumber].data.Speed),(speed))/6.0f)*16384);
    //speedinto=(int16_t)((Positional_PID_Compute(&motordata[motornumber].motor_contrl,(speed),((motordata[motornumber].data).Speed))/6.0f)*16384);
	  speedinto=(int16_t)(speedinto*0.2+speedintolast*0.8); 
		speedwanted=speed;
		speed_a_wanted=(float)(speedinto/16384*6.0f);
		speednow=(motordata[motornumber].data.Speed);
    
if (speedinto >  16384) speedinto =  16384;
if (speedinto < -16384) speedinto = -16384;
		fit_power_T=((motordata[motornumber].data.RollCureet)*0.3);
		fit_power_W=(motordata[motornumber].data.Speed);
    fit_power_pre=POWER_METER_COMPUTE_PER(((speedinto)*6.0f/16384),motordata[motornumber].data.Speed,&Reverso_Chassiss.chassisspower[motornumber]);
    Motor_Drive_Single(&motordata[motornumber],Txframe,speedinto);
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
  /*  Positional_PID_Reset_nomal(&chassiss_motor[0].motor_contrl);
    Positional_PID_Reset_nomal(&chassiss_motor[1].motor_contrl);
    Positional_PID_Reset_nomal(&chassiss_motor[2].motor_contrl);
    Positional_PID_Reset_nomal(&chassiss_motor[3].motor_contrl);*/
    PID_clear(&chassiss_motor[0].motor_contrl);
    PID_clear(&chassiss_motor[1].motor_contrl);
    PID_clear(&chassiss_motor[2].motor_contrl);
    PID_clear(&chassiss_motor[3].motor_contrl);
}

float motorpower_1=0;



void chaisspower_debug()
{
	
}

void chasiss_gyrodrive()
{
  float target[3];
      target[0]=((normal4chdata.ch0)*30-0.09090);//vx
      target[1]=((normal4chdata.ch1)*30+0.02272);//vy
      target[2]=normal4chdata.ch2*50;//wr
      speedptztochassis(&target[0],&target[1]);
    //Chassiss_Drive(target,chassiss_motor,&chassis_fdcan);
      chassissmotorsigletest((target[0]),chassiss_motor,&chassis_fdcan,0);
			//Chassiss_Drive_byforce(target,chassiss_motor,&chassis_fdcan);
}

void chassiss_withdrive()
{
  chassis_relative_ptzupdate();
     float target[3];
    target[0]=((normal4chdata.ch0)*30-0.09090);//vx
    target[1]=((normal4chdata.ch1)*30+0.02272);//vy
    target[2]=Reverso_Chassiss.status.phase_difference;//
    speedptztochassis(&target[0],&target[1]);
    //Chassiss_Drive(target,chassiss_motor,&chassis_fdcan);
	  ChassisMove_withMode(target,chassiss_motor,&chassis_fdcan);
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
        //speedinto[i] = (int16_t)(Positional_PID_Compute(&motordata[i].motor_contrl,speedsloved[i],((motordata[i].data.Speed)*9.54))/6.0f*16384);
          speedinto[i] = (int16_t)(PID_calc(&motordata[i].motor_contrl, (motordata[i].data.Speed*6.28/60), speedsloved[i])/6.0f*16384);
      }

     //POWER_METER_COMPUTE_PER(rollcrrent, float speed,Power_K* Kvalues);
      //Motor_Drive_Frame( &Chassiss_CAN,motordata[0].ContrlID,speedinto);//contrldata is the arry of contrl value
    Motor_Drive_Frame(Txframe,motordata[0].ContrlID,speedinto);//contrldata is the arry of contrl value
}

#endif




uint32_t debugflag=0;
uint32_t debugerrcnt=0;

void mustclearbits_chassiss()
{ 
  for(uint8_t count=0;count<100&&osThreadFlagsClear(0x000000FF)==pdFALSE;count++){}
}

uint32_t edebug=0;

void ChassisTask03(void *argument)
{
	  Fdcan_FilterInit(&hfdcan1);
//    PID_init(&chassiss_motor[0].motor_contrl,PID_POSITION,(fp32[]){0.17f,0.07f,0.00f},10000,800);//450/440//when force it just use the heah 3 values
//    PID_init(&chassiss_motor[1].motor_contrl,PID_POSITION,(fp32[]){0.10f,0.05f,0.00f},10000,800);//450/448
//    PID_init(&chassiss_motor[2].motor_contrl,PID_POSITION,(fp32[]){0.12f,0.00f,0.00f},10000,800);//450/442
//    PID_init(&chassiss_motor[3].motor_contrl,PID_POSITION,(fp32[]){0.90f,0.25f,0.00f},10000,800);//450/442//when with use it as the WR pid
//		PID_init(&WRoutcircle                   ,PID_POSITION,(fp32[]){1.22f,0.00f,0.00f},10000,800);//when with use it as inside pid
      PID_init(&chassiss_motor[0].motor_contrl,PID_POSITION,(fp32[]){1.53f,0.00f,0.00f},10000,800);//450/440//when force it just use the heah 3 values
      PID_init(&chassiss_motor[1].motor_contrl,PID_POSITION,(fp32[]){0.10f,0.05f,0.00f},10000,800);//450/448
      PID_init(&chassiss_motor[2].motor_contrl,PID_POSITION,(fp32[]){0.12f,0.00f,0.00f},10000,800);//450/442
      PID_init(&chassiss_motor[3].motor_contrl,PID_POSITION,(fp32[]){0.90f,0.25f,0.00f},10000,800);//450/442//when with use it as the WR pid
    	PID_init(&WRoutcircle                   ,PID_POSITION,(fp32[]){1.22f,0.00f,0.00f},10000,800);//when with use it as inside pid
    Chassiss_Init(&Reverso_Chassiss);
    osThreadFlagsWait(0x00000002,osFlagsWaitAll,osWaitForever);

  static uint32_t lastEffectiveorderflag_cha=0;
  static uint32_t lastEffectivecnt=0;  
   
  for(;;)
  {
    static uint32_t orderflag;
	  orderflag=osThreadFlagsGet();
    debugflag= osThreadFlagsGet();

    if(orderflag==0x00000000&&(lastEffectivecnt>=10))//cleartoofasterrorhandle
    { lastEffectivecnt++;
      if(lastEffectivecnt<100000000)
      {
        orderflag=lastEffectiveorderflag_cha;
				edebug=lastEffectiveorderflag_cha;
				
      }else{//orderflag=0x00000001;
				lastEffectivecnt=0;}
    }

		
   if(orderflag&0x00000002)
   {
    chassis_relative_ptzupdate();
    if(orderflag&0x00000020)//gyro
    {
      lastEffectiveorderflag_cha=orderflag;
			edebug=lastEffectiveorderflag_cha;
      //chasiss_gyrodrive();
    }
    else if (orderflag&0x00000010)//with
    {
      lastEffectiveorderflag_cha=orderflag;
			edebug=lastEffectiveorderflag_cha;
	 	  //chassiss_withdrive();
    }
   }
	 else if(orderflag==0x00000001)
   {
    lastEffectiveorderflag_cha=orderflag;
		edebug=lastEffectiveorderflag_cha;
    chassiss_disabled();
   }else
   {
    lastEffectivecnt++;
   }

    mustclearbits_chassiss();

    osDelay(1);
  }
  
}



