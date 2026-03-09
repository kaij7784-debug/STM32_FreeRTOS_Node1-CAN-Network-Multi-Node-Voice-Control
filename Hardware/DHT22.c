#include "DHT22.h"
#include "FreeRTOS.h"
#include "task.h"

/**
  * @brief  微秒级延时 (针对72MHz频率)
  */
static void DHT22_Delay_us(uint32_t us) {
    uint32_t i = us * 8; 
    while(i--);
}

/**
  * @brief  初始化GPIO
  */
void DHT22_Init(void) {
    RCC_APB2PeriphClockCmd(DHT22_RCC, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = DHT22_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // 初始设为输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DHT22_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_SetBits(DHT22_GPIO_PORT, DHT22_GPIO_PIN); // 总线空闲为高
}

/**
  * @brief  切换引脚为输入模式
  */
static void DHT22_Mode_In(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = DHT22_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
    GPIO_Init(DHT22_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  切换引脚为输出模式
  */
static void DHT22_Mode_Out(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = DHT22_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(DHT22_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  读取温湿度数据
  * @param  temp: 温度存储地址, hum: 湿度存储地址
  * @retval 1: 成功, 0: 失败
  */
uint8_t DHT22_Read_Data(float *temp, float *hum) {
    uint8_t i, j, data[5] = {0};
    uint16_t timeout;

    // 1. 主机发送起始信号
    DHT22_Mode_Out();
    GPIO_ResetBits(DHT22_GPIO_PORT, DHT22_GPIO_PIN);
    DHT22_Delay_us(1500); // 拉低1ms以上
    GPIO_SetBits(DHT22_GPIO_PORT, DHT22_GPIO_PIN);
    DHT22_Delay_us(35);   // 等待20-40us

    // 2. 检测从机响应
    DHT22_Mode_In();
    timeout = 0;
    while(GPIO_ReadInputDataBit(DHT22_GPIO_PORT, DHT22_GPIO_PIN) == Bit_SET) {
        if(++timeout > 200) return 0; // 响应超时
        DHT22_Delay_us(1);
    }
    
    while(GPIO_ReadInputDataBit(DHT22_GPIO_PORT, DHT22_GPIO_PIN) == Bit_RESET); // 等待响应低电平结束
    while(GPIO_ReadInputDataBit(DHT22_GPIO_PORT, DHT22_GPIO_PIN) == Bit_SET);   // 等待响应高电平结束

    // 3. 接收数据（进入临界区，防止时序被RTOS打断）
    taskENTER_CRITICAL();
    for (i = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            while(GPIO_ReadInputDataBit(DHT22_GPIO_PORT, DHT22_GPIO_PIN) == Bit_RESET);
            DHT22_Delay_us(40); // 延时判断：0为26us，1为70us
            if (GPIO_ReadInputDataBit(DHT22_GPIO_PORT, DHT22_GPIO_PIN) == Bit_SET) {
                data[i] |= (1 << (7 - j));
                while(GPIO_ReadInputDataBit(DHT22_GPIO_PORT, DHT22_GPIO_PIN) == Bit_SET);
            }
        }
    }
    taskEXIT_CRITICAL();

    // 4. 校验与换算
    if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        *hum = (float)((data[0] << 8) | data[1]) / 10.0f;
        *temp = (float)(((data[2] & 0x7F) << 8) | data[3]) / 10.0f;
        if (data[2] & 0x80) *temp *= -1.0f;
        return 1;
    }
    return 0;
}
