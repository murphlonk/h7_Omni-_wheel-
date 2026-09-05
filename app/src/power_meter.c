#include "power_meter.h"

#include "chassiss.h"

extern osThreadId_t TaskPTZ02Handle;
extern osThreadId_t TaskChassis03Handle;
extern osThreadId_t TaskCan04Handle;
extern osThreadId_t TaskPowermeter0Handle;
//extern osThreadId_t TaskIMU07Handle;
extern chassis_handler Reverso_Chassiss;

void POWER_METER_INIT(Power_K* Kvalues,float Ka,float Kb,float const_value,float torque_constant)
{
     Kvalues->Ka=Ka;
     Kvalues->Kb=Kb;
     Kvalues->const_value=const_value;
     Kvalues->torque_constant=torque_constant;
}

float POWER_METER_COMPUTE_PER(float rollcrrent, float speed,Power_K* Kvalues)//calculte the peices
{
    return rollcrrent*Kvalues->torque_constant*speed
    +Kvalues->Ka*rollcrrent*rollcrrent*Kvalues->torque_constant*Kvalues->torque_constant
    +Kvalues->Kb*speed*speed
    +Kvalues->const_value;
}



float Power_caculateall(float* ecost ,uint8_t numbers)// cost all calculater
{ 
    float allcost;
   for(uint8_t i=0;i<numbers;i++)
   {
    allcost+=ecost[i];
   }
   return allcost;
}


float Power_Remap_ratio(float* powersloved ,float * nowdata)
{
float ratio[4];


}

float Power_to_rollcurrent(float power,float speed,Power_K* Kvalues)//slove the rollcurrent by power and speed
{
  float KA=Kvalues->Ka,KB=speed,KC=Kvalues->Kb*speed*speed+Kvalues->const_value-power;
  if(KB*KB-4*KA*KC>0)
  {
    return (float)((-KB+sqrt(KB*KB-4*KA*KC))/(2*KA));
  }else if(KB*KB-4*KA*KC==0||KB*KB-4*KA*KC<0)
  {
    return (float)(-KB/(2*KA));
  }

}



float* Power_Remap_Bigp(float* powerslove,float* nowpowerdata,uint8_t numbers,Power_limits* limits,RealMotor_Data* data)//remap the power to the new power
{
  float Kcoe=0,errorsum=0,Powersum=0;
  float powerchanged[4];
  for(uint8_t i=0;i<numbers;i++)
  {
    Kcoe+=(powerslove[i]-nowpowerdata[i]-limits->lower_limit)/(limits->upper_limit-limits->lower_limit);
    errorsum+=nowpowerdata[i]-powerslove[i];
    Powersum+=powerslove[i];
  }

  for(uint8_t i=0;i<numbers;i++)
  {
    powerchanged[i]=Kcoe*(nowpowerdata[i]-powerslove[i])/errorsum+(1-Kcoe)*powerslove[i]/Powersum;
    powerslove[i]=Power_to_rollcurrent(powerchanged[i],data[i].Speed,&Reverso_Chassiss.chassisspower[i]);
  }
  
}






void taskpower_onoff(bool onoff) //ture :poweron:0x00000002 false :poweroff:0x00000000;
{
  if (onoff)
  {
      osThreadFlagsSet(TaskPTZ02Handle,        0x00000002);//openpower:0x00000002
      osThreadFlagsSet(TaskChassis03Handle,    0x00000002);
      osThreadFlagsSet(TaskCan04Handle,        0x00000002);
      osThreadFlagsSet( TaskPowermeter0Handle, 0x00000002);
      //osThreadFlagsSet(TaskIMU07Handle,        0x00000002);   
  }else 
  {/*
      osThreadFlagsSet(TaskPTZ02Handle,        0x00000004);//offpower:0x00000002
      osThreadFlagsSet(TaskChassis03Handle,    0x00000004);
      osThreadFlagsSet(TaskCan04Handle,        0x00000004);
      osThreadFlagsSet(TaskPowermeter0Handle,  0x00000004);
      //osThreadFlagsSet(TaskIMU07Handle,      0x00000004);
      */ 
   }

}

void task_carmode(uint32_t order)//true :gyro:0x00000000/0x00000020,false:with:0x00000010/0x00000010
{
   if(order&0x00000010)
   {
      osThreadFlagsSet(TaskChassis03Handle,0x00000010);
      osThreadFlagsSet(TaskPTZ02Handle,    0x00000010);
   }else
   {
      osThreadFlagsSet(TaskChassis03Handle,0x00000020);
      osThreadFlagsSet(TaskPTZ02Handle,    0x00000020);

   }

} 

 uint32_t orderflag;

void PowermeterTask05(void *argument)
{
   osThreadFlagsWait(0x00000002,osFlagsWaitAll,osWaitForever);
  for(;;)
  { //uint32_t orderflag;
     orderflag=osThreadFlagsGet();
     
       if(orderflag&0x00000002)
       {

       }else{}

    osThreadFlagsClear(0x7FFFFFFF);
  }
  
}

uint8_t count=0;

uint32_t flagswtich=0;
extern uint32_t high_stamp;

void SwtichTask06(void *argument)
{ uint32_t flagsInit=0;
  TIM5_Init();
  static uint32_t timstamp_last=0,high_stamp_last=0;
  static uint8_t onoff=0;
  flagsInit=osThreadFlagsWait(0x00000001,osFlagsWaitAll,osWaitForever);
  for (;;)
  {
		count++;
    if(flagsInit&0x00000001)
    {  //uint32_t flagswtich=0;
      flagswtich=osThreadFlagsGet();
      if(flagswtich&0x00000001)//online
      {
        onoff=1;
        if(flagswtich&0x00000002)//open_power
        {
      taskpower_onoff(true);
      task_carmode(flagswtich);
      osThreadFlagsClear(0X7FFFFFFF);
					//break;
        }
        else if((flagswtich&0x00000003)==0x00000001)
        {

        taskpower_onoff(false);
        osThreadFlagsClear(0X7FFFFFFF);
					//break;
        }
      }
      else if(flagswtich==0x0000000)//offline
      {
				
        if(onoff==1)
        {
          timstamp_last=TIM5_GetCounter();
          high_stamp_last=high_stamp;
          onoff=0;
        }
       
          if(high_stamp_last==high_stamp)
          {
            if((TIM5_GetCounter()-timstamp_last)>500000)
            {
              taskpower_onoff(false);
              osThreadFlagsClear(0X7FFFFFFF);
            }
          }
          else if(high_stamp_last==high_stamp-1)
          {
            if((TIM5_GetCounter()+1000000-timstamp_last)<500000)
            {
              taskpower_onoff(true);
              task_carmode(flagswtich);
              osThreadFlagsClear(0X7FFFFFFF);
            }else
            {
              taskpower_onoff(false);
              osThreadFlagsClear(0X7FFFFFFF);
            }
          }else
          {
            taskpower_onoff(false);
            osThreadFlagsClear(0X7FFFFFFF);
          }
      } 
    }
    else
    {
       uint32_t flagstran;
     taskpower_onoff(false);
      flagstran=osThreadFlagsGet();
       if(flagstran&0x00000001)
       {
          flagsInit=0x00000001;
          taskpower_onoff(true);
          task_carmode(flagswtich);
          osThreadFlagsClear(0x7FFFFFFF);
				 //break;
       }
    }

    osThreadFlagsClear(0x7FFFFFFF);
		osDelay(1);
  }

}



