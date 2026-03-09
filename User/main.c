#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "OLED.h"
#include "Serial.h"
#include "DHT22.h"
#include "MyCAN.h"
#include <stdio.h>
#include "DS1302.h"

/* ===================== 宏定义与数据结构 ===================== */
// UI 显示 ID 定义
#define UI_WELCOME      0x00
#define UI_TEMP_REPORT  0x01
#define UI_FAN_ON       0x02
#define UI_FAN_OFF      0x03
#define UI_WINDOW_ON    0x04
#define UI_WINDOW_OFF   0x05
#define UI_SCREEN_ON    0x06
#define UI_LIGHT_ON     0x07
#define UI_LIGHT_OFF    0x08
#define UI_FAN_HIGH     0x09
#define UI_ALARM        0x0A  // 新增：车内高温报警显示
#define UI_ALL_OFF      0x0B  // 新增：全系统关闭显示

#define UI_OTA_MODE       0x11  // 显示 "OTA MODE..."
#define UI_OTA_EXIT       0x12  // 显示 "SYSTEM START"

#define UI_OFFLINE      0x0C  // 新增：从机离线显示状态
#define UI_EXECUTING    0x0D  // 正在执行中
#define UI_EXEC_SUCCESS 0x0E  // 执行成功
#define CAN_ID_A_TO_B   0x110
#define CAN_ID_B_TO_A   0x220

#define FILTER_SIZE 5

// 1. 温度计图标 (TempIcon) - 16x16
uint8_t BMP_TempIcon[] = {
   0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x03,0x03,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x18,0x7C,0xC7,0x80,0x80,0xC7,0x7C,0x18,0x00,0x00,0x00,0x00,
};/*"C:\Users\86178\Desktop\test_biyesheji\image\temp.bmp",0*/

// 2. 风扇图标 (FanIcon) - 16x16
uint8_t BMP_FanIcon[] = {
    0x00,0x00,0x00,0x00,0x04,0x1A,0xE2,0x42,0x42,0xE2,0x1E,0x00,0x00,0x00,0x00,0x00,
0x00,0x07,0x18,0x21,0x21,0x18,0x07,0x02,0x02,0x07,0x18,0x21,0x31,0x18,0x07,0x00,
};/*"C:\Users\86178\Desktop\test_biyesheji\image\fan.bmp",0*/

// 3. 信号正常图标 (SignalOn) - 16x16
uint8_t BMP_SignalOn[] = {
0x30,0x18,0x8C,0xC4,0x66,0x76,0x36,0x36,0x36,0x36,0x76,0xE6,0xCC,0x9C,0x38,0x20,
0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x1E,0x1E,0x0C,0x00,0x00,0x00,0x00,0x00,0x00,
};/*"C:\Users\86178\Desktop\test_biyesheji\image\SignalOn.bmp",0*/
// 4. 信号断开图标 (SignalOff) - 16x16

uint8_t BMP_SignalOff[] = {0x30,0x18,0x8C,0xC4,0x66,0x76,0x36,0x36,0x36,0x36,0x76,0xE6,0xCC,0x9C,0x38,0x20,
0x00,0x00,0x00,0x00,0x00,0x21,0x12,0x0C,0x0C,0x12,0x21,0x00,0x00,0x00,0x00,0x00,
};/*"C:\Users\86178\Desktop\test_biyesheji\image\SignalOFF.bmp",0*/

uint8_t BMP_windowOff[] = {0x00,0x00,0xC0,0xA0,0x90,0x88,0x84,0x84,0x84,0x84,0x84,0x84,0x84,0xFC,0x00,0x00,
0x00,0x00,0x3F,0x20,0x20,0x20,0x20,0x20,0x20,0x22,0x22,0x22,0x20,0x3F,0x00,0x00,
};/*"C:\Users\86178\Desktop\test_biyesheji\image\window_open.bmp",0*/

uint8_t BMP_lightOn[] = {0x00,0xF0,0x18,0x24,0x1A,0x0B,0x05,0x01,0x01,0x01,0x03,0x02,0x04,0x18,0xF0,0x00,
0x00,0x03,0x0C,0x18,0xF0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xF0,0x18,0x0C,0x03,0x00,
/*"C:\Users\86178\Desktop\test_biyesheji\image\light.bmp",0*/
};

uint8_t BMP_warning[] = {0x00,0x00,0x00,0x80,0xE0,0xF8,0xFC,0x0E,0x0E,0xFC,0xF8,0xE0,0x80,0x00,0x00,0x00,
0x70,0x7C,0x7E,0x7F,0x7F,0x7F,0x7F,0x6C,0x6C,0x7F,0x7F,0x7F,0x7F,0x7E,0x7C,0x70,
/*"C:\Users\86178\Desktop\test_biyesheji\image\warning.bmp",0*/
};

uint8_t BMP_time[] = {0x00,0xE0,0x10,0x08,0x04,0x02,0x02,0xFA,0x02,0x02,0x04,0x04,0x08,0x10,0xE0,0x00,
0x00,0x07,0x08,0x10,0x20,0x40,0x40,0x41,0x41,0x41,0x41,0x21,0x11,0x08,0x07,0x00,
};/*"C:\Users\86178\Desktop\test_biyesheji\image\time.bmp",0*/

uint32_t LastHeartbeatTick = 0; // 记录最后一次收到心跳的时间
uint8_t Status_Fan = 0;    // 0:关, 1:开/高速
uint8_t Status_Window = 0; // 0:关, 1:开 (对应 UI_WINDOW_OFF 显示图标)
uint8_t Status_Light = 0;  // 0:关, 1:开
uint8_t NodeB_Online = 1;      // 节点 B 在线状态标志
// 增加一个标志位防止重复触发报警
uint8_t TempAlarmFlag = 0;

// 环境数据结构体
typedef struct {
    float temp;
    float hum;
} EnvData_t;

// 控制指令结构体
typedef struct {
    uint8_t device; // 对应 Serial_RxPacket[0]
    uint8_t action; // 对应 Serial_RxPacket[1]
} ControlCmd_t;

/* ===================== 内核对象句柄 ===================== */
TaskHandle_t xTaskVoiceHandle   = NULL;
TaskHandle_t xTaskCanCommHandle = NULL;
TaskHandle_t xTaskSensorHandle  = NULL;
TaskHandle_t xTaskDisplayHandle = NULL;

SemaphoreHandle_t xSemVoiceReady = NULL; 
QueueHandle_t     xQueueVoiceCmd = NULL; // 存放 ControlCmd_t
QueueHandle_t     xQueueEnvData  = NULL; // 存放 EnvData_t
QueueHandle_t     xQueueDisplay  = NULL; // 存放 UI ID (uint8_t)

/* ===================== 函数声明 ===================== */
void Hardware_Init(void);
void vTaskVoiceParse(void *pvParameters);
void vTaskCanComm(void *pvParameters);
void vTaskSensor(void *pvParameters);
void vTaskDisplay(void *pvParameters);

/* ===================== 主函数 ===================== */

int main(void)
{
    // 1. 中断优先级分组 (FreeRTOS 必须为 Group 4)
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    // 2. 硬件初始化
   	DHT22_Init(); 
    Hardware_Init();
    OLED_Init();
    Serial_Init();   // 串口驱动 (SU-03T)
       // 温湿度驱动
    MyCAN_Init();    // CAN 驱动 (125Kbps + 中断配置)

    DS1302_GPIO_Init();
	//  DS1302_Init();// 如果 DS1302 没掉电，这句初始化在 main 中只做一次，防止每次重启都重置时间
    // 3. 创建内核对象
    xSemVoiceReady = xSemaphoreCreateBinary();
    xQueueVoiceCmd = xQueueCreate(5, sizeof(ControlCmd_t)); // 结构体队列
    xQueueEnvData  = xQueueCreate(1, sizeof(EnvData_t));
    xQueueDisplay  = xQueueCreate(5, sizeof(uint8_t));
    
    // 4. 创建任务
    xTaskCreate(vTaskVoiceParse, "Voice", 128, NULL, 4, &xTaskVoiceHandle);
    xTaskCreate(vTaskCanComm,    "CAN",   128, NULL, 3, &xTaskCanCommHandle);
    xTaskCreate(vTaskSensor,     "Snsr",  256, NULL, 2, &xTaskSensorHandle);
    xTaskCreate(vTaskDisplay,    "Disp",  256, NULL, 1, &xTaskDisplayHandle);

    // 5. 启动调度器
    vTaskStartScheduler();

    while (1); 
}

/* ===================== 任务实现 ===================== */

/**
  * @brief 语音解析任务 (优先级 4)
  */
void vTaskVoiceParse(void *pvParameters) {
    uint8_t d_cmd;
    EnvData_t env_buf;
    ControlCmd_t can_cmd;
    uint8_t tx_buf[5] = {0xFF, 0x01, 0x00, 0x00, 0xFE};
    uint8_t rtc_read_div = 0;
    while(1) {    
			  /* A. 读取 DS1302 实时时间 */
			  if (++rtc_read_div >= 5) {
            DS1302_read_realTime(); 
            rtc_read_div = 0;
        }
        if (xSemaphoreTake(xSemVoiceReady, portMAX_DELAY) == pdPASS) {
            uint8_t data0 = Serial_RxPacket[0];
            uint8_t data1 = Serial_RxPacket[1];

            // UI 逻辑判断
					 if (data0 == 0x06) { // 离家模式/全部关闭
                d_cmd = UI_ALL_OFF;
                xQueueSend(xQueueDisplay, &d_cmd, 0);

                // 批量发送关机指令给 CAN 队列
                can_cmd.action = 0x01; // 1为关闭
                
                can_cmd.device = 0x02; xQueueSend(xQueueVoiceCmd, &can_cmd, 0); // 关风扇
                vTaskDelay(pdMS_TO_TICKS(50)); // 微调延时避免队列拥塞
                can_cmd.device = 0x03; xQueueSend(xQueueVoiceCmd, &can_cmd, 0); // 关窗
                vTaskDelay(pdMS_TO_TICKS(50));
                can_cmd.device = 0x05; xQueueSend(xQueueVoiceCmd, &can_cmd, 0); // 关灯
						    Status_Fan = 0;
						    Status_Window = 0;
						    Status_Light = 0;
            }
            else if (data0 == 0x00 && data1 == 0x00) d_cmd = UI_WELCOME;
            else if (data0 == 0x01 && data1 == 0x02) {
                if (xQueuePeek(xQueueEnvData, &env_buf, 0) == pdPASS) {
									  tx_buf[1] = 0x01;
                    tx_buf[2] = (uint8_t)env_buf.temp;
                    Serial_SendArray(tx_buf, 5); // 回传温度给语音模块播报
                }
                d_cmd = UI_TEMP_REPORT;
            }
						 else if (data0 == 0x01 && data1 == 0x03) {
                if (xQueuePeek(xQueueEnvData, &env_buf, 0) == pdPASS) {
                    tx_buf[1] = 0x04;          // 固定功能码 0x04
                    tx_buf[2] = TimeData.hour;   // xx: 小时
                    tx_buf[3] = TimeData.minute; // yy: 分钟
                    Serial_SendArray(tx_buf, 5); // 回传时间给语音模块播报
                }
                d_cmd = UI_TEMP_REPORT;
            }
            else if (data0 == 0x02 && data1 == 0x00) { d_cmd = UI_FAN_ON; Status_Fan = 1; }
            else if (data0 == 0x02 && data1 == 0x01) { d_cmd = UI_FAN_OFF; Status_Fan = 0; }
            else if (data0 == 0x02 && data1 == 0x02) { d_cmd = UI_FAN_HIGH; Status_Fan = 1; }
            else if (data0 == 0x03 && data1 == 0x00) { d_cmd = UI_WINDOW_ON; Status_Window = 1; }
            else if (data0 == 0x03 && data1 == 0x01) { d_cmd = UI_WINDOW_OFF; Status_Window = 0; }
            else if (data0 == 0x05 && data1 == 0x00) { d_cmd = UI_LIGHT_ON; Status_Light = 1; }
            else if (data0 == 0x05 && data1 == 0x01) { d_cmd = UI_LIGHT_OFF; Status_Light = 0; }
						// --- 场景 1：进入升级模式 ---
            else if (data0 == 0x01 && data1 == 0xEE) { // 语音识别到“准备升级”
                can_cmd.device = 0x11; // 触发拦截广播
                can_cmd.action = 0x00;
                xQueueSend(xQueueVoiceCmd, &can_cmd, 0);
    
                   // OLED 显示 OTA 状态
                d_cmd = UI_OTA_MODE;
                xQueueSend(xQueueDisplay, &d_cmd, 0);
               }
						else if (data0 == 0x01 && data1 == 0xEF) { // 假设语音识别到“退出升级/系统重启”
                 can_cmd.device = 0x33; // 重启设备码
                 can_cmd.action = 0x00;
                 xQueueSend(xQueueVoiceCmd, &can_cmd, 0);
    
                 d_cmd = UI_OTA_EXIT;
                 xQueueSend(xQueueDisplay, &d_cmd, 0);
                 }

            // 发送给显示任务
            xQueueSend(xQueueDisplay, &d_cmd, 0);
            
            // 【关键】如果是控制类指令，封装结构体发送给 CAN 任务
            if (data0 >= 0x02) {
                can_cmd.device = data0;
                can_cmd.action = data1;
                xQueueSend(xQueueVoiceCmd, &can_cmd, 0);
            }
        }
    }
}

/**
  * @brief CAN 通信任务 (优先级 3)
  * 功能：处理语音指令发送、失败重传、从机反馈校验及总线健康监测
  */
void vTaskCanComm(void *pvParameters) {
    /* 1. 变量定义 */
    ControlCmd_t recv_cmd;    // 存储从队列收到的原始指令
    ControlCmd_t pending_cmd; // 存储当前正在等待确认的备份指令
    uint8_t tx_payload[2];    // CAN 发送缓冲区
    
    uint32_t rxID;            // 接收 ID
    uint8_t rxLen;            // 接收长度
    uint8_t rxData[8];        // 接收缓冲区
    uint8_t d_cmd;            // OLED 显示指令缓冲区
    
    // 闭环控制逻辑变量
    uint8_t wait_ack = 0;           // 标志位：是否正在等待确认回执
    uint8_t retry_count = 0;        // 当前重试计数器
    TickType_t last_send_tick = 0;  // 记录上一次发送的时间戳

	
    /* 2. 初始化心跳戳 */
    // 初始状态假设节点在线，并记录当前系统滴答数
    LastHeartbeatTick = xTaskGetTickCount();
    NodeB_Online = 1;
    while(1) {
        // --- 第一部分：发送逻辑 (语音指令下发) ---
        // 只有当前没有正在处理的重传任务时，才从语音队列获取新任务
        if (wait_ack == 0) {
            // 使用 10ms 超时接收，确保即使没有语音，下方接收逻辑也能运行
            if (xQueueReceive(xQueueVoiceCmd, &recv_cmd, pdMS_TO_TICKS(10)) == pdPASS) {
                // 1. 备份当前指令，以备重发
                pending_cmd = recv_cmd;
                
                // 2. 构造载荷并发送
                tx_payload[0] = recv_cmd.device;
                tx_payload[1] = recv_cmd.action;
							if (recv_cmd.device == 0x11) {
                    // 1. 发第一帧触发复位
                    MyCAN_Transmit(CAN_ID_A_TO_B, 2, tx_payload);
                    
										vTaskDelay(pdMS_TO_TICKS(500));
										uint8_t voice_tx[5] = {0xFF, 0x05, 0x00, 0x00, 0xFE};
                    Serial_SendArray(voice_tx, 5);
                    // 拦截完成后，由于 Node B 已经进入 Bootloader 的 C 循环，
                    // Node A 这边直接结束此任务的本次循环，不进入重传逻辑。
                    wait_ack = 0; 
                }
							else if (recv_cmd.device == 0x33) { // 退出/重启
                     MyCAN_Transmit(CAN_ID_A_TO_B, 2, tx_payload);
    
                      // 给语音模块发指令，提示系统重启
                     uint8_t voice_tx[5] = {0xFF, 0x06, 0x00, 0x00, 0xFE};
                     Serial_SendArray(voice_tx, 5);
    
                     wait_ack = 0;
                }
          else{
						   MyCAN_Transmit(CAN_ID_A_TO_B, 2, tx_payload);
                
                // 3. 进入“等待确认”模式
                wait_ack = 1;
                retry_count = 0;
                last_send_tick = xTaskGetTickCount();
                
                // 4. OLED 物理反馈：显示“正在执行”
                d_cmd = UI_EXECUTING;
                xQueueSend(xQueueDisplay, &d_cmd, 0);
                }
                // 5. 翻转 LED 指示
                GPIO_WriteBit(GPIOC, GPIO_Pin_13, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13)));
            }
        }

        // --- 第二部分：接收逻辑 (从机反馈与心跳处理) ---
        if (MyCAN_HasReceived()) {
            MyCAN_GetRxMessage(&rxID, &rxLen, rxData);
            
            // 收到节点 B 的报文 (ID: 0x220)
            if (rxID == CAN_ID_B_TO_A) {
                // 只要收到节点 B 的信号，就更新心跳时间戳，防止报离线
                LastHeartbeatTick = xTaskGetTickCount(); 
                if (rxData[0] == 0xFC && rxData[1] == 0xBB) {
                 // 1. OLED 显示升级成功
                uint8_t d_cmd = UI_OTA_EXIT; 
                xQueueSend(xQueueDisplay, &d_cmd, 0);

                // 发送语音播报指令给 SU-03T
                // 固件更新成功，系统正在重新启动
                uint8_t voice_success[5] = {0xFF, 0x07, 0x00, 0x00, 0xFE};
                Serial_SendArray(voice_success, 5);
                 }
                // 如果之前标记为离线，现在收到消息，则恢复在线状态
                if (NodeB_Online == 0) {
                    NodeB_Online = 1;
                    d_cmd = UI_WELCOME; // 离线恢复后显示欢迎界面
                    xQueueSend(xQueueDisplay, &d_cmd, 0);
                }

                // 【闭环确认判断】
                // 检查节点 B 回传的设备 ID (Data[0]) 是否匹配我们正在等待的设备
                if (wait_ack == 1 && rxData[0] == pending_cmd.device) {
                    wait_ack = 0; // 收到确认，关闭重传机制
                    retry_count = 0;
                    
                    // OLED 物理反馈：显示“执行成功”
                    d_cmd = UI_EXEC_SUCCESS;
                    xQueueSend(xQueueDisplay, &d_cmd, 0);
                }
            }
        }

        // --- 第三部分：重传逻辑 (超时检测) ---
        // 如果当前正在等待回执，且距离上次发送超过 200ms
        if (wait_ack == 1 && (xTaskGetTickCount() - last_send_tick) > pdMS_TO_TICKS(200)) {
            if (retry_count < 3) { // 最大允许重发 3 次
                // 重新下发之前备份的指令
                tx_payload[0] = pending_cmd.device;
                tx_payload[1] = pending_cmd.action;
                MyCAN_Transmit(CAN_ID_A_TO_B, 2, tx_payload);
                
                retry_count++;
                last_send_tick = xTaskGetTickCount(); // 更新发送时间
            } else {
                // 重发 3 次（共 4 次尝试）全部失败
                wait_ack = 0; 
                
                // OLED 物理反馈：显示“通信异常/通信失败”
                d_cmd = UI_OFFLINE;
                xQueueSend(xQueueDisplay, &d_cmd, 0);
            }
        }

        // --- 第四部分：总线健康监测---
        // 如果节点当前在线，但已经超过 10 秒没收到节点 B 的任何消息
     if (NodeB_Online == 1 && (xTaskGetTickCount() - LastHeartbeatTick) > pdMS_TO_TICKS(15000)) {
            NodeB_Online = 0;
            d_cmd = UI_OFFLINE;
            xQueueSend(xQueueDisplay, &d_cmd, 0);
        }
        
        vTaskDelay(pdMS_TO_TICKS(20)); // 给总线留出处理空隙
    }
}
/**
  * @brief 传感器采样任务 (优先级 2)
  */
void vTaskSensor(void *pvParameters) {
    EnvData_t raw_data;        // 存储传感器原始值
    EnvData_t filtered_data;   // 存储滤波后的平滑值
    ControlCmd_t auto_cmd;
    uint8_t alarm_buf[4] = {0xFF, 0x03, 0x00, 0xFE}; 
    
    // 滤波缓冲区
    float temp_history[FILTER_SIZE] = {0};
    float hum_history[FILTER_SIZE] = {0};
    uint8_t filter_idx = 0;
    uint8_t filter_count = 0; // 记录缓冲区是否填满
    while(1) {
        // 1. 读取 DHT22 原始数据
        if (DHT22_Read_Data(&raw_data.temp, &raw_data.hum)) {
            
            // 2. 滑动平均滤波处理
            temp_history[filter_idx] = raw_data.temp;
            hum_history[filter_idx] = raw_data.hum;
            filter_idx = (filter_idx + 1) % FILTER_SIZE;
            if (filter_count < FILTER_SIZE) filter_count++;

            float temp_sum = 0, hum_sum = 0;
            for (int i = 0; i < filter_count; i++) {
                temp_sum += temp_history[i];
                hum_sum += hum_history[i];
            }
            filtered_data.temp = temp_sum / filter_count;
            filtered_data.hum = hum_sum / filter_count;

            // 3. 将平滑后的数据存入队列，供显示任务和语音任务查询
            xQueueOverwrite(xQueueEnvData, &filtered_data);

            // 4. 翻转 LED 表示采样正常工作
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13)));

            // 5. 核心逻辑：基于滤波后的平滑温度进行报警判断
            // 使用平滑值可以防止由于一次采样误差导致的误报
            if (filtered_data.temp >= 30.0f && TempAlarmFlag == 0) {
                // (1) 立即通知语音模块播报
                Serial_SendArray(alarm_buf, 4);
                
                // (2) 更新显示状态为报警
                uint8_t d_cmd = UI_ALARM;
                xQueueSend(xQueueDisplay, &d_cmd, 0);

                // (3) 5秒后增速
                vTaskDelay(pdMS_TO_TICKS(5000));
                
                auto_cmd.device = 0x02; // 风扇设备
                auto_cmd.action = 0x03; // 增速
                xQueueSend(xQueueVoiceCmd, &auto_cmd, 0);
                
                TempAlarmFlag = 1; // 锁定报警，防止每2秒播报一次
            }
            // 降到 28°C 以下才允许下次报警，防止在 30°C 临界点反复震荡
            else if (filtered_data.temp < 28.0f) {
                TempAlarmFlag = 0; 
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000)); // DHT22 建议采样间隔
    }
}

/**
  * @brief OLED 显示任务 (优先级 1)
  */
/**
  * @brief OLED 显示任务
  * 功能：分区显示系统状态，提供手机式状态栏及动态反馈
  */
void vTaskDisplay(void *pvParameters) {
    uint8_t d_cmd;
    EnvData_t env_buf;
    char str[20];
    uint8_t last_cmd = UI_WELCOME; 
    uint8_t exec_dots = 0;
    uint8_t clear_flag = 0;
    OLED_Clear();
    uint8_t rtc_read_div = 0;
	  NodeB_Online = 1;
    while(1) {
			  /* A. 读取 DS1302 实时时间 */
			  if (++rtc_read_div >= 5) {
            DS1302_read_realTime(); 
            rtc_read_div = 0;
        }
	//		printf("Display Task Heartbeat...\r\n");
        /* 1. 接收显示指令 */
        if (xQueueReceive(xQueueDisplay, &d_cmd, pdMS_TO_TICKS(100)) == pdPASS) {
        if (d_cmd != last_cmd) {
                last_cmd = d_cmd;
                // 如果是重要的状态切换，才清屏，否则只局部刷新
                if(d_cmd == UI_OFFLINE || d_cmd == UI_WELCOME || d_cmd == UI_ALARM) {
                     OLED_ShowString(8, 0, "                     ");
                }
            }
        }

        /* 2. 顶部状态栏图标绘制 (Page 0-1, 固定位逻辑) */
        
        // [0位]: 节点A信号 (常亮)
        OLED_ShowIcon16x16(0, 0, BMP_SignalOn);
        
        // [17位]: 节点B信号
        if (NodeB_Online) {
            OLED_ShowIcon16x16(17, 0, BMP_SignalOn);
        } else {
            OLED_ShowIcon16x16(17, 0, BMP_SignalOff);
        }

        // [34位]: 风扇状态
        if (Status_Fan) OLED_ShowIcon16x16(34, 0, BMP_FanIcon);
          else OLED_DrawBlank16x16(34, 0);

        // [51位]: 车窗状态
        if (Status_Window) OLED_ShowIcon16x16(51, 0, BMP_windowOff); 
        else OLED_DrawBlank16x16(51, 0);

        // [68位]: 灯光状态
        if (Status_Light) OLED_ShowIcon16x16(68, 0, BMP_lightOn); 
        else OLED_DrawBlank16x16(68, 0);

        // [112位]: 报警图标 (最右侧)
        if (last_cmd == UI_ALARM) 
            OLED_ShowIcon16x16(112, 0, BMP_warning); 
        else 
            OLED_DrawBlank16x16(112, 0);


        /* 3. 中部环境数据区 (Line 4-5) */
        if (xQueuePeek(xQueueEnvData, &env_buf, 0) == pdPASS) {
            // 温度图标在 Line 4 (Page 3) 
            OLED_ShowIcon16x16(0, 2, BMP_TempIcon);
					if(clear_flag==1){
            OLED_ShowString(3, 4, "                  ");
					}
            // 6x8字体：Line 4 对应 Page 3，Column 4对应像素 24
            sprintf(str, "Temp: %.1f C", env_buf.temp);
            OLED_ShowString(3, 4, str); 
            clear_flag=0;
            sprintf(str, "Humi: %.1f %%", env_buf.hum);
            OLED_ShowString(4, 4, str);
        }
				else{
					  OLED_ShowIcon16x16(0, 2, BMP_TempIcon);
            OLED_ShowString(3, 4, "waiting the sensor"); 
					  clear_flag=1;
				}

        /* 4. 底部反馈区 (Line 7-8) */
				OLED_ShowIcon16x16(1, 4, BMP_time);
				sprintf(str, "%02d:%02d:%02d  %02d.%02d",TimeData.hour, TimeData.minute, TimeData.second,TimeData.month,TimeData.day);
        OLED_ShowString(6, 4, str); // 6x8字体，Line 1, Column 14
        switch(last_cmd) {
            case UI_WELCOME:
                OLED_ShowString(8, 2, "< System Ready >");
                break;
                
            case UI_EXECUTING:
                OLED_ShowString(8, 1, "Executing");
                exec_dots = (exec_dots + 1) % 4;
                // 在字符位置进行打点动画
                if(exec_dots == 0)      OLED_ShowString(7, 11, "    ");
                else if(exec_dots == 1) OLED_ShowString(7, 11, ".   ");
                else if(exec_dots == 2) OLED_ShowString(7, 11, "..  ");
                else if(exec_dots == 3) OLED_ShowString(7, 11, "... ");
                break;
                
            case UI_EXEC_SUCCESS:
                OLED_ShowString(8, 1, "Status: SUCCESS ");
                break;
                
            case UI_OFFLINE:
                OLED_ShowString(8, 1, "Command OK      ");
                break;
                
            case UI_ALARM:
                OLED_ShowString(8, 1, "!! TEMP HIGH !! ");
                break;

            case UI_ALL_OFF:
                OLED_ShowString(8, 1, "All Systems OFF ");
                break;
						case UI_OTA_MODE:
                   OLED_ShowString(8, 1, ">> OTA MODE <<  ");
                 OLED_ShowIcon16x16(112, 0, BMP_warning); // 闪烁警告图标提醒正在更新
                  break;

            case UI_OTA_EXIT:
                   OLED_ShowString(8, 1, " SYSTEM START  ");
                 break;

            default:      OLED_ShowString(8, 1, "System Ready >");
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ===================== 基础硬件初始化 ===================== */
void Hardware_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_SetBits(GPIOC, GPIO_Pin_13); // 默认灭
}
