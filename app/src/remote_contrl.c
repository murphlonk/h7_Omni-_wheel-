#include "remote_contrl.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include  "car_mode.h"


Dr16_Data Dr16_Data_Receive;
Dr16_CH_NORMAILIZATION normal4chdata;

//extern uint8_t data_temp_uart[BUFF_SIZE];
SemaphoreHandle_t Keyborad_Semaphore;

uint32_t thebitwanted=0;
uint32_t thebitlast=0;

void Remote_Contrl_Init()
{
    Keyborad_Semaphore = xSemaphoreCreateMutex();
    if (Keyborad_Semaphore == NULL)
    {
        // Handle semaphore creation failure
		Remote_Contrl_Init();
    }

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
if(Dr16_Data_Receive.get_data_cnt_low==0xFFFFFFFF)
{
	Dr16_Data_Receive.get_data_cnt_high++;
	Dr16_Data_Receive.get_data_cnt_low=0;
}
Dr16_Data_Receive.get_data_cnt_low++;
if(Dr16_Data_Receive.get_data_cnt_low==0xFFFFFFFF&&Dr16_Data_Receive.get_data_cnt_high==0xFFFFFFFF)
{
	Dr16_Data_Receive.get_data_cnt_high=0;
	Dr16_Data_Receive.get_data_cnt_low=0;
}
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

bool must_give_semaphore(SemaphoreHandle_t Keyborad_Semaphore)
{
	static uint32_t count=0;
	if(xSemaphoreGive(Keyborad_Semaphore) == pdFALSE&&count<10)
	{
		count++;
		must_give_semaphore(Keyborad_Semaphore);
	}else if(count>=10){count = 0;return false;}else{return true;}
}

bool must_take_semaphore(SemaphoreHandle_t Keyborad_Semaphore)
{
	static uint32_t count=0;
	if(xSemaphoreTake(Keyborad_Semaphore, (TickType_t)10) == pdTRUE&&count<10)
	{
		count++;
		must_take_semaphore(Keyborad_Semaphore);
	}
	else if(count>=10){count = 0;return false;}else{return true;}
}

void Keyborad_Contrl()
{
	
	
		if(Dr16_Data_Receive.Key_1==39)
		{
			
		}else if(Dr16_Data_Receive.Key_1==40)
		{

		}
   
}



void lever_Status_()//quick stop contrl
{
	extern osThreadId_t TaskSwitch06Handle;
	static bool enable =false;
	thebitlast=thebitwanted;
	thebitwanted=0x00000000;
   	if(Dr16_Data_Receive.S_1==2&&Dr16_Data_Receive.S_2==2)
		{
			enable=false;
			thebitwanted |=0x00000001;
		}else 
		{

			if(enable==true)
			{
			thebitwanted |=(0x00000002|0x00000001);
			}
			else if(enable==false&&(Dr16_Data_Receive.S_1!=2||Dr16_Data_Receive.S_2!=2))
			{
			enable=true;
			thebitwanted |=(0x00000002|0x00000001);
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
	thebitwanted |=0x00000010;
  }
  else if(Dr16_Data_Receive.S_2==3)
  {
  } else{}
}

void ptz_modetran()
{
	//extern osThreadId_t TaskPTZ02Handle;
	extern osThreadId_t TaskSwitch06Handle;
	if(Dr16_Data_Receive.S_1==2)
	{
	 thebitwanted |=0x00000010;
	}
	else if(Dr16_Data_Receive.S_1==3)
	{
	}
}

 void bitmustset_remote()
{
	extern osThreadId_t TaskSwitch06Handle;//to keep the only be updated in one go
	 for(uint8_t count=0;count<100&&osThreadFlagsSet(TaskSwitch06Handle,thebitwanted)==pdFALSE;count++){}
}

void dr16_update(uint8_t* Data_Temp)
{
  extern osThreadId_t TaskSwitch06Handle;//to keep the only be updated in one go
	dr16_Dataslove(Data_Temp);
	lever_Status_();
	rule_status();
	chassis_modetran();
	//ptz_modetran();
	DR16data_normal();
	bitmustset_remote();
}

 