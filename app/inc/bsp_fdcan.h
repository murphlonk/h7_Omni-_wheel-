#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#include "fdcan.h"
#include "stdint.h"

typedef struct {
    FDCAN_HandleTypeDef *hcan;
    FDCAN_TxHeaderTypeDef Header;
    uint8_t Data[8];
} FDCAN_TxFrame_TypeDef;


typedef struct {
    FDCAN_HandleTypeDef *hcan;
    FDCAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
} FDCAN_RxFrame_TypeDef;

void Fdcan_FilterInit(FDCAN_HandleTypeDef *hfdcan);
void Fdcan_Transmit_INIT(FDCAN_TxFrame_TypeDef* hfdcanTxFrame,FDCAN_HandleTypeDef *hfdcan);
void Fdcan_Transmit(FDCAN_TxFrame_TypeDef *TxFrame);

#endif

