#include "remote_contrl.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include  "car_mode.h"


Dr16_Data Dr16_Data_Receive;
Dr16_CH_NORMAILIZATION normal4chdata;
//extern uint8_t data_temp_uart[BUFF_SIZE];

void Remote_Contrl_Init()
{
  
    
}

void DR16data_normal()
{
	normal4chdata.ch0= (float)((Dr16_Data_Receive.Channel_0-1024)/1320.0f);
	normal4chdata.ch1= (float)((Dr16_Data_Receive.Channel_1-1024)/1320.0f);
	normal4chdata.ch2= (float)((Dr16_Data_Receive.Channel_2-1024)/1320.0f);
	normal4chdata.ch3= (float)((Dr16_Data_Receive.Channel_3-1024)/1320.0f);
}


void dr16_Dataslove(uint8_t* Data_Temp)
{
	/*数据解析位运算采用ai生成,我脑子想这个完成时间太长,但已测试数据符合通讯协议*/
Dr16_Data_Receive.Channel_0 = (uint16_t)(Data_Temp[0] | ((Data_Temp[1] & 0x07) << 8));
Dr16_Data_Receive.Channel_1 = (uint16_t)(((Data_Temp[1] >> 3) & 0x1F) | ((Data_Temp[2] & 0x3F) << 5));
Dr16_Data_Receive.Channel_2 = (uint16_t)(((Data_Temp[2] >> 6) & 0x03) | (Data_Temp[3] << 2) | ((Data_Temp[4] & 0x01) << 10));
Dr16_Data_Receive.Channel_3 = (uint16_t)(((Data_Temp[4] >> 1) & 0x7F) | ((Data_Temp[5] & 0x0F) << 7));
Dr16_Data_Receive.S_1 = (Data_Temp[5] >> 4) & 0x03;
Dr16_Data_Receive.S_2 = (Data_Temp[5] >> 6) & 0x03;
Dr16_Data_Receive.Mouse_X = (uint16_t)(Data_Temp[6] | (Data_Temp[7] << 8));
Dr16_Data_Receive.Mouse_Y = (uint16_t)(Data_Temp[8] | (Data_Temp[9] << 8));
Dr16_Data_Receive.Mouse_Z = (uint16_t)(Data_Temp[10] | (Data_Temp[11] << 8));
Dr16_Data_Receive.Key_1 = Data_Temp[12];
Dr16_Data_Receive.wheel = (uint16_t)(Data_Temp[16] | (Data_Temp[17] << 8));
}

volatile uint32_t debugcount=0;
//UBaseType_t Markdefault;
//UBaseType_t MarkPTZ;
//UBaseType_t Markchassis;
//UBaseType_t Marktaskcan;
//UBaseType_t Markpowrmeter;
//UBaseType_t Markswitch;
//UBaseType_t MarkIMU;





extern osThreadId_t  defaultTaskHandle;
extern osThreadId_t  TaskPTZ02Handle;
extern osThreadId_t  TaskChassis03Handle;
extern osThreadId_t  TaskCan04Handle;
extern osThreadId_t  TaskPowermeter0Handle;
extern osThreadId_t  TaskIMU07Handle;
UBaseType_t Mark[7];



void lever_Status_()//quick stop contrl
{
	extern osThreadId_t TaskSwitch06Handle;
	static bool enable =false;
    if(Dr16_Data_Receive.S_1==2&&Dr16_Data_Receive.S_2==2)
	{
		enable=false;
		debugcount=osThreadFlagsSet(TaskSwitch06Handle, 0x00000001);
	}else 
	{

		if(enable==true)
		{
           debugcount=osThreadFlagsSet(TaskSwitch06Handle, (0x00000002|0x00000001));//open power:0x00000002 |online :0x00000001
            
		}
		else if(enable==false&&(Dr16_Data_Receive.S_1!=2||Dr16_Data_Receive.S_2!=2))
		{
          enable=true;
        debugcount=osThreadFlagsSet(TaskSwitch06Handle, (0x00000001|0x00000002));
		}
	}

	Mark[0]=uxTaskGetStackHighWaterMark( defaultTaskHandle );
	 Mark[1]=uxTaskGetStackHighWaterMark( TaskPTZ02Handle   );
	 Mark[2]=uxTaskGetStackHighWaterMark( TaskChassis03Handle);
	 Mark[3]=uxTaskGetStackHighWaterMark( TaskCan04Handle );
	 Mark[4]=uxTaskGetStackHighWaterMark( TaskPowermeter0Handle );
	 Mark[5]=uxTaskGetStackHighWaterMark( TaskSwitch06Handle );
	 Mark[6]=uxTaskGetStackHighWaterMark( TaskIMU07Handle  );
}

void rule_status()
{ 
   
    
}

void chassis_modetran()
{
	//extern osThreadId_t TaskChassis03Handle;
	extern osThreadId_t TaskSwitch06Handle;
  if(Dr16_Data_Receive.S_2==2)
  {
    //osThreadFlagsSet(TaskChassis03Handle, 0x00000010);
	osThreadFlagsSet(TaskSwitch06Handle, 0x00000010);
  }
  else if(Dr16_Data_Receive.S_2==3)
  {
     //osThreadFlagsSet(TaskChassis03Handle, 0x00000020);
	 //osThreadFlagsSet(TaskChassis03Handle, 0x00000000);
  } else{}
}

void ptz_modetran()
{
	//extern osThreadId_t TaskPTZ02Handle;
	extern osThreadId_t TaskSwitch06Handle;
	if(Dr16_Data_Receive.S_1==2)
	{
     //osThreadFlagsSet(TaskPTZ02Handle, 0x00000010);
	 osThreadFlagsSet(TaskSwitch06Handle, 0x00000010);
	}
	else if(Dr16_Data_Receive.S_1==3)
	{
     //osThreadFlagsSet(TaskPTZ02Handle, 0x00000020); //gyro0x0
	 //osThreadFlagsSet(TaskChassis03Handle, 0x00000000);
	}
}

 

void dr16_update(uint8_t* Data_Temp)
{
  
	dr16_Dataslove(Data_Temp);
	lever_Status_();
	rule_status();
	chassis_modetran();
	//ptz_modetran();
	DR16data_normal();
}

 