#include "power_meter.h"


extern osThreadId_t TaskPTZ02Handle;
extern osThreadId_t TaskChassis03Handle;
extern osThreadId_t TaskCan04Handle;
extern osThreadId_t TaskPowermeter0Handle;
//extern osThreadId_t TaskIMU07Handle;

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

void SwtichTask06(void *argument)
{ uint32_t flagsInit=0;
  flagsInit=osThreadFlagsWait(0x00000001,osFlagsWaitAll,osWaitForever);
  for (;;)
  {
		count++;
		
    if(flagsInit&0x00000001)
    {  //uint32_t flagswtich=0;
      flagswtich=osThreadFlagsGet();
      if(flagswtich&0x00000001)//online
      {
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
        
//        osDelay(MAX_OFFLINETME);
//        flagswtich=osThreadFlagsGet();
				flagswtich=osThreadFlagsWait(0x00000001,(osFlagsWaitAll|osFlagsNoClear),500);
        if(flagswtich&0x00000001)
        {
         taskpower_onoff(true);
         task_carmode(flagswtich);
        // osThreadFlagsClear(0X7FFFFFFF);
					//break;
        }
        else if(flagswtich==osErrorTimeout)
        {
          flagsInit=0x00000000;
          taskpower_onoff(false);
        }
				osThreadFlagsClear(0X7FFFFFFF);
      }
        //break;
    }
    else if(flagsInit==0x0000000)
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



//uint32_t frq=0;


//void SwtichTask06(void *argument)
//{
//	uint32_t flagsInit=0;
//  flagsInit=osThreadFlagsWait(0x00000001,osFlagsWaitAll,osWaitForever);
//	static uint32_t tickcountdisconnect=0x00000000;
//  for(;;)
//  {
//		count++;
//		
//    if(flagsInit&0x00000001)
//    {  uint32_t flagswtich;
//      flagswtich=osThreadFlagsGet();
//      if(flagswtich&0x00000001)//online
//      {
//        if(flagswtich&0x00000002)
//        {
//      taskpower_onoff(true);
//      osThreadFlagsClear(0X7FFFFFFF);
//					//break;
//        }
//        else if(flagswtich==0x00000001)
//        {

//      taskpower_onoff(false);
//      osThreadFlagsClear(0X7FFFFFFF);
//					
//        }
//      }
//      else if(flagswtich==0x0000000)//offline
//      {
////       static uint8_t init=0;
////				if(init==0)
////        {
////				 tickcountdisconnect=osKernelGetTickCount();
////					init=1;
////				}else if(init==1)
////        {
////					if(tickcountdisconnect+500<=osKernelGetTickCount())
////            {
////						  taskpower_onoff(false);
////						}else if(tickcountdisconnect)
////            {
////							
////						}
////				}
//       
//				
//				
//      }
//    else if(flagsInit==0x0000000)
//    {
//       uint32_t flagstran;
//     taskpower_onoff(false);
//      flagstran=osThreadFlagsGet();
//       if(flagstran&0x00000001)
//       {
//          flagsInit=0x00000001;
//          taskpower_onoff(true);
//          osThreadFlagsClear(0x7FFFFFFF);
//				 
//       }
//    }

//	
//		
//  }
//}
//}


