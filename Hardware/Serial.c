#include "Serial.h"
#include <stdarg.h>
#include "FreeRTOS.h"
#include "semphr.h"

uint8_t Serial_RxPacket[4];				// 接收数据包数组
extern SemaphoreHandle_t xSemVoiceReady; // 声明在main.c中创建的信号量

/**
  * 函    数：串口初始化
  * 说    明：适配FreeRTOS，波特率115200，优先级分组4
  */
void Serial_Init(void)
{
	/* 1. 开启时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	/* 2. GPIO初始化 */
	GPIO_InitTypeDef GPIO_InitStructure;
	// PA9 - TX (复用推挽输出)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// PA10 - RX (上拉输入)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* 3. USART初始化 */
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600; // SU-03T 默认波特率
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStructure);
	
	/* 4. 中断配置 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	// FreeRTOS工程必须强制使用分组4
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	// 优先级必须在 5-15 之间 (受configMAX_SYSCALL_INTERRUPT_PRIORITY限制)
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	
	/* 5. 使能串口 */
	USART_Cmd(USART1, ENABLE);
}

/**
  * 函    数：发送相关功能 (保持不变)
  */
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++) { Serial_SendByte(Array[i]); }
}

void Serial_SendString(char *String)
{
	for (uint8_t i = 0; String[i] != '\0'; i ++) { Serial_SendByte(String[i]); }
}

// printf 重定向
int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);
	return ch;
}

void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(String);
}

/**
  * 函    数：USART1中断函数 (状态机 + RTOS同步)
  */
void USART1_IRQHandler(void)
{
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;
	
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(USART1);
		
		/* 状态机接收数据包：FF [4字节数据] FE */
		if (RxState == 0) // 找包头
		{
			if (RxData == 0xFF) { RxState = 1; pRxPacket = 0; }
		}
		else if (RxState == 1) // 存数据
		{
			Serial_RxPacket[pRxPacket++] = RxData;
			if (pRxPacket >= 2) { RxState = 2; }
		}
		else if (RxState == 2) // 找包尾
		{
			if (RxData == 0xFE)
			{
				RxState = 0;
				
				/* --- RTOS 关键操作：释放信号量唤醒任务 --- */
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				if (xSemVoiceReady != NULL)
				{
					xSemaphoreGiveFromISR(xSemVoiceReady, &xHigherPriorityTaskWoken);
					// 强制进行一次上下文切换，确保高优先级的语音任务立即执行
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
				}
			}
			else { RxState = 0; } // 包尾不对，丢弃此包
		}
		
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}
