#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"
#include <stdio.h>

/* 外部声明数据包数组，方便其他任务读取解析后的指令 */
extern uint8_t Serial_RxPacket[4];

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_Printf(char *format, ...);
void Serial_SendPacket(uint8_t *Msg4Byte);

#endif
