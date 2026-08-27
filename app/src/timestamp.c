#include "timestamp.h"


#include "stm32h7xx.h"

void TIM5_Init(void)
{
    RCC->APB1LENR |= RCC_APB1LENR_TIM5EN; // enable the apb line,
    TIM5->CR1 &= ~TIM_CR1_CEN;//disable the tim1
    TIM5->PSC =APBFRE - 1; 
    //TIM5->ARR =0xFFFFFFFF-1;//set the max conut space,tim2,tim5 are the 32 bit timer,else are 16 bit timer
    TIM5->ARR =1000000-1;
	  TIM5->CR1 |= TIM_CR1_ARPE; 
    TIM5->EGR |= TIM_EGR_UG;//sei the interrupt event
	  TIM5->DIER |= TIM_DIER_UIE;  //  enable refreash interrupt  
    #if USEUIFRENAP
  	TIM5->CR1 |= TIM_CR1_UIFREMAP;
    #endif
	  TIM5->SR &= ~TIM_SR_UIF;   //reset the SR 
    NVIC_SetPriority(TIM5_IRQn, 5);   //use the cmsis api to set the prority of tim1_up_irqn
    NVIC_EnableIRQ(TIM5_IRQn);            //enable nvic
    TIM5->CR1 |= TIM_CR1_CEN;//enable the tim1
}

#if USEUIFRENAP


uint32_t TIM5_GetCounterSafe(void)
{
    
    uint32_t cnt = TIM2->CNT;
    if (cnt & 0x80000000) {
        
    }
    return (cnt & 0x7FFFFFFF);
}

#else

uint32_t TIM5_GetCounter(void)
{
	 
    return TIM5->CNT; //straight get the count value ,just use this fuction when the uifremap did not be used
}



#endif

uint32_t high_stamp=0;
uint32_t debug=0;

void TIM5_IRQHandler(void)//tim1 is highlevel timer ,use TIM1_UP_IRQHandler(overstack interrupt) etc
{ debug=TIM5->CNT;
    if (TIM5->SR & TIM_SR_UIF) {//SR is the updatebit
        TIM5->SR &= ~TIM_SR_UIF;   //reset the SR 
			  high_stamp++;
    }
}


