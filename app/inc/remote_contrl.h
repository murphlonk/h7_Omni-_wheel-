#ifndef REMOTE_CONTRL_H
#define REMOTE_CONTRL_H

#define BUFF_SIZE 50

#include "stdint.h"
#include "stdbool.h"

typedef struct 
{
	uint16_t Channel_0;
    uint16_t Channel_1;
	uint16_t Channel_2;
	uint16_t Channel_3;
	uint8_t S_1;
	uint8_t S_2;
    uint16_t Mouse_X;
    uint16_t Mouse_Y;
    uint16_t Mouse_Z;
    uint8_t Mouse_L;
    uint8_t Mouse_R;
    uint16_t Key_1;
    uint16_t wheel;
} Dr16_Data;

typedef struct 
{
    float ch0;
    float ch1;
    float ch2;
    float ch3;
}Dr16_CH_NORMAILIZATION;


void Remote_Contrl_Init();
void dr16_Dataslove(uint8_t* Data_Temp);
void dr16_update(uint8_t* Data_Temp);
#endif
