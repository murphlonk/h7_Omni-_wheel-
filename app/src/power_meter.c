#include "power_meter.h"

#include "chassiss.h"

extern osThreadId_t TaskPTZ02Handle;
extern osThreadId_t TaskChassis03Handle;
extern osThreadId_t TaskCan04Handle;
extern osThreadId_t TaskPowermeter0Handle;
//extern osThreadId_t TaskIMU07Handle;
//extern chassis_handler Reverso_Chassiss;

void POWER_METER_INIT(Power_K* Kvalues,float Ka,float Kb,float const_value,float torque_constant)
{
     Kvalues->Ka=Ka;
     Kvalues->Kb=Kb;
     Kvalues->const_value=const_value;
     Kvalues->torque_constant=torque_constant;
}


float POWER_METER_COMPUTE_PER(float rollcrrent, float speed,Power_K* Kvalues)//calculte the peices
{
//    return rollcrrent*Kvalues->torque_constant*speed
//    +Kvalues->Ka*rollcrrent*rollcrrent*Kvalues->torque_constant*Kvalues->torque_constant
//    +Kvalues->Kb*speed*speed
//    +Kvalues->const_value;
		return rollcrrent*Kvalues->torque_constant*speed
    +Kvalues->Ka*rollcrrent*rollcrrent*Kvalues->torque_constant*Kvalues->torque_constant
    +Kvalues->Kb*speed
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

float Power_All(float* speedinto,float* speednow,Power_K* Kvalues,uint8_t numbers)
{
  float allpower=0;
  for(uint8_t i=0;i<numbers;i++)
  {
    allpower+=POWER_METER_COMPUTE_PER(speedinto[i],speednow[i],&Kvalues[i]);
  }
  return allpower;
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



float* Power_Remap_Bigp(float* speedslove,float* nowspeeddata,uint8_t numbers,Power_limits* limits,Power_K* Kvalues)//remap the power to the new power
{
  float Kcoe=0,errorsum=0,Powersum=0;
  float powerchanged[4];
  for(uint8_t i=0;i<numbers;i++)
  {
    Kcoe+=(speedslove[i]-nowspeeddata[i]-limits->lower_limit)/(limits->upper_limit-limits->lower_limit);
    errorsum+=nowspeeddata[i]-speedslove[i];
    Powersum+=speedslove[i];
  }

  for(uint8_t i=0;i<numbers;i++)
  {
    powerchanged[i]=Kcoe*(nowspeeddata[i]-speedslove[i])/errorsum+(1-Kcoe)*speedslove[i]/Powersum;
    speedslove[i]=Power_to_rollcurrent(powerchanged[i],nowspeeddata[i],&Kvalues[i]);
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


void task_power_mode(uint32_t order,bool onoff)//true :gyro:0x00000000/0x00000020,false:with:0x00000010/0x00000010
{
 if(onoff&&order!=0)
   {
        if(order&0x00000010)
      {
      osThreadFlagsSet(TaskPTZ02Handle,        0x00000012);//openpower:0x00000002
      osThreadFlagsSet(TaskChassis03Handle,    0x00000012);
      osThreadFlagsSet(TaskCan04Handle,        0x00000002);
      osThreadFlagsSet(TaskPowermeter0Handle,  0x00000002);
      }else
      {
      osThreadFlagsSet(TaskPTZ02Handle,        0x00000022);//openpower:0x00000002
      osThreadFlagsSet(TaskChassis03Handle,    0x00000022);
      osThreadFlagsSet(TaskCan04Handle,        0x00000002);
      osThreadFlagsSet(TaskPowermeter0Handle,  0x00000002);
      } 
   }else if(onoff&&order==0)
   {
	   
	 
	 }
	  else 
   {
		  osThreadFlagsSet(TaskPTZ02Handle,        0x00000001);//openpower:0x00000002
      osThreadFlagsSet(TaskChassis03Handle,    0x00000001);//offpower:0x00000001
      osThreadFlagsSet(TaskCan04Handle,        0x00000001);
      osThreadFlagsSet(TaskPowermeter0Handle,  0x00000001);
   }


}


void mustclearbits_swtich()
{
 for(uint8_t count=0;count<100&&osThreadFlagsClear(0x000000FF)==pdFALSE;count++){}
}

extern uint32_t high_stamp;

void if_outlineouttime_detected(uint8_t* onoff,uint32_t flagswtich)
{
      uint32_t timestamp=TIM5_GetCounter();
      static uint32_t high_stamp_last=0,timstamp_last=0;
        if(*onoff==1)
        {
          timstamp_last=TIM5_GetCounter();
          high_stamp_last=high_stamp;
          *onoff=0;
        }
       
          if(high_stamp_last==high_stamp)
          {
            if((timestamp-timstamp_last)>500000)
            {
              //taskpower_onoff(false);
              task_power_mode(0,false);//?
              //osThreadFlagsClear(0X7FFFFFFF);
              mustclearbits_swtich();
            }
          }
          else if(high_stamp_last==high_stamp-1)
          {
            if((timestamp+1000000-timstamp_last)<500000)
            {
              //taskpower_onoff(true);
              //task_carmode(flagswtich);
							flagswtich=osThreadFlagsGet();
              task_power_mode(flagswtich,true);
              //osThreadFlagsClear(0X7FFFFFFF);
              mustclearbits_swtich();
            }else
            {
              task_power_mode(0,false);//?
              //osThreadFlagsClear(0X7FFFFFFF);
              mustclearbits_swtich();
            }
          }else
          {
            //taskpower_onoff(false);
            task_power_mode(0,false);
            //osThreadFlagsClear(0X7FFFFFFF);
            mustclearbits_swtich();
          }
       
}


void online_powercomtrl(uint32_t flagswtich,uint8_t* onoff)
{
         *onoff=1;
        if(flagswtich&0x00000002)//open_power
        {
      task_power_mode(flagswtich,true);
      //osThreadFlagsClear(0X7FFFFFFF);
      mustclearbits_swtich();
					//break;
        }
        else if((flagswtich&0x00000003)==0x00000001)
        {
        task_power_mode(0,false);
        mustclearbits_swtich();
					//break;
        }
}


void init_errorhandle(uint32_t flagsInit,uint32_t flagswtich)
{
   uint32_t flagstran;
     //taskpower_onoff(false);
     task_power_mode(0,false);
      flagstran=osThreadFlagsGet();
       if(flagstran&0x00000001)
       {
          flagsInit=0x00000001;
          //taskpower_onoff(true)
          //task_carmode(flagswtich);
          task_power_mode(flagswtich,true);
          //osThreadFlagsClear(0x7FFFFFFF);
          mustclearbits_swtich();
				 //break;
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

extern Dr16_Data Dr16_Data_Receive;

//uint32_t flagswtich=0;

//void SwtichTask06(void *argument)
//{ uint32_t flagsInit=0;
//  TIM5_Init();
//  static uint8_t onoff=0;
//  static uint32_t Dr16_high_last=0;
//  static uint32_t Dr16_low_last=0;
//  static uint8_t compare=0;
//  flagsInit=osThreadFlagsWait(0x00000001,osFlagsWaitAll,osWaitForever);
//  for (;;)
//  {
//    if(flagsInit&0x00000001)
//    {  //uint32_t flagswtich=0;
//      flagswtich=osThreadFlagsGet();
//      if(flagswtich&0x00000001)//online
//      { 
//      compare=0;
//      Dr16_high_last=Dr16_Data_Receive.get_data_cnt_high;
//      Dr16_low_last=Dr16_Data_Receive.get_data_cnt_low;
//       online_powercomtrl(flagswtich,&onoff); 
//      }
//      else if(flagswtich==0x0000000)//offline
//      {
//      if(Dr16_high_last==Dr16_Data_Receive.get_data_cnt_high&&Dr16_low_last==Dr16_Data_Receive.get_data_cnt_low) 
//       {if_outlineouttime_detected(&onoff,flagswtich);}
//       else{}
//      }
//    }
//    else
//    {
//      init_errorhandle(flagsInit,flagswtich);
//    }

//    mustclearbits_swtich();
//		osDelay(1);
//  }

//}





