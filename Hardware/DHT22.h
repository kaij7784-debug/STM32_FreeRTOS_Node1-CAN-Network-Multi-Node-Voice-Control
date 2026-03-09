#ifndef __DHT22_H
#define __DHT22_H

#include "stm32f10x.h"

/* 引脚定义：你可以根据实际连接修改 PB10 或 PA1 等 */
#define DHT22_RCC          RCC_APB2Periph_GPIOA
#define DHT22_GPIO_PORT    GPIOA
#define DHT22_GPIO_PIN     GPIO_Pin_4

/* 函数声明 */
void DHT22_Init(void);
uint8_t DHT22_Read_Data(float *temp, float *hum);

#endif
