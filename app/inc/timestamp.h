//#ifndef TIMESTAMP_H
//#define TIMESTAMP_H

//#include <stdint.h>

//#define MCUFRE 550

//void TIM1_Init(void);
//uint32_t TIM1_GetCounter(void);


//#endif 

//#ifndef TIMESTAMP_H
//#define TIMESTAMP_H

///*user could use this text with out enable the timer in the cube */

//#define APBFRE 275
//#define USEUIFRENAP 0  // 0 for disable , not 0 for  enable 

//#include <stdint.h>




//void TIM1_Init(void);
//uint32_t TIM1_GetCounter(void);


//#endif 

#ifndef TIMESTAMP_H
#define TIMESTAMP_H

/*user could use this text with out enable the timer in the cube */

#define APBFRE 275
#define USEUIFRENAP FLASE

#include <stdint.h>




void TIM5_Init(void);
uint32_t TIM5_GetCounter(void);


#endif 
