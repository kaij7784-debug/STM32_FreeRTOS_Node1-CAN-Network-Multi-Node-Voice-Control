#ifndef __MYCAN_H
#define __MYCAN_H

#include "stm32f10x.h"

// 接收相关全局变量声明
extern uint32_t MyCAN_RxID;
extern uint8_t MyCAN_RxLen;
extern uint8_t MyCAN_RxData[8];
extern uint8_t MyCAN_RxFlag;

void MyCAN_Init(void);
void MyCAN_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data);
uint8_t MyCAN_HasReceived(void);
void MyCAN_GetRxMessage(uint32_t *ID, uint8_t *Length, uint8_t *Data);

#endif
