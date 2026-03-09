#include "MyCAN.h"

// 定义接收变量
uint32_t MyCAN_RxID;
uint8_t MyCAN_RxLen;
uint8_t MyCAN_RxData[8];
uint8_t MyCAN_RxFlag = 0;

/**
  * 函    数：CAN初始化
  * 说    明：波特率125k，开启接收中断
  */
void MyCAN_Init(void)
{
    /* 1. 开启时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    
    /* 2. GPIO配置 */
    GPIO_InitTypeDef GPIO_InitStructure;
    // PA12 - CAN_TX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // PA11 - CAN_RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* 3. CAN初始化 */
    CAN_InitTypeDef CAN_InitStructure;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    // 波特率计算 = 36M / 48 / (1 + 2 + 3) = 125K
    CAN_InitStructure.CAN_Prescaler = 48;        
    CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
    CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;
    
    CAN_InitStructure.CAN_NART = DISABLE; // 允许自动重传
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_Init(CAN1, &CAN_InitStructure);
    
    /* 4. 过滤器配置 (接收所有ID) */
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
    
    /* 5. 中断配置 (Node B 必须使用) */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE); // 开启FIFO0消息挂号中断
    
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10; // 确保在SysCall优先级范围内
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * 函    数：CAN发送报文 (优化版)
  */
void MyCAN_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = ID;
    TxMessage.IDE = CAN_Id_Standard;
    TxMessage.RTR = CAN_RTR_Data;
    TxMessage.DLC = Length;
    for (uint8_t i = 0; i < Length; i++)
    {
        TxMessage.Data[i] = Data[i];
    }
    
    // 检查是否有空的发送邮箱
    uint8_t mbox = CAN_Transmit(CAN1, &TxMessage);
    
    // 在 RTOS 中，我们不需要在这里 while 忙等直到发送成功
    // 硬件会自动处理重传。如果邮箱满了，CAN_Transmit 会返回 CAN_TxStatus_NoMailBox
    // 我们可以加一个极小的超时防止万一
    uint16_t timeout = 0;
    while (CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Pending && timeout < 500) 
    {
        timeout++;
    }
}

/**
  * 函    数：CAN接收中断服务函数
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CanRxMsg RxMessage;
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
        
        MyCAN_RxID = RxMessage.StdId;
        MyCAN_RxLen = RxMessage.DLC;
        for (uint8_t i = 0; i < MyCAN_RxLen; i++)
        {
            MyCAN_RxData[i] = RxMessage.Data[i];
        }
        
        MyCAN_RxFlag = 1; // 设置标志位通知主循环
        
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}

/**
  * 函    数：检查是否收到报文
  * 返 回 值：1表示收到，0表示未收到
  */
uint8_t MyCAN_HasReceived(void)
{
    if (MyCAN_RxFlag == 1)
    {
        return 1;
    }
    return 0;
}

/**
  * 函    数：获取接收到的数据并复位标志位
  * 参数ID, Length, Data 均为输出参数
  */
void MyCAN_GetRxMessage(uint32_t *ID, uint8_t *Length, uint8_t *Data)
{
    *ID = MyCAN_RxID;
    *Length = MyCAN_RxLen;
    for (uint8_t i = 0; i < MyCAN_RxLen; i++)
    {
        Data[i] = MyCAN_RxData[i];
    }
    
    // 读取后自动清除标志位，准备接收下一帧
    MyCAN_RxFlag = 0;
}
