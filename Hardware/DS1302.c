#include "ds1302.h"
#include "sys.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

 
struct TIMEData TimeData;
struct TIMERAM TimeRAM;
u8 read_time[7];
 
static void DS1302_SimpleDelay(u32 us) {
    u32 i = us * 10; 
    while(i--);
}
void DS1302_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(DS1302_CLK, ENABLE); // 假设 CLK 是 RCC_APB2Periph_GPIOA
    
    // SCLK 和 CE 始终是推挽输出
    GPIO_InitStructure.GPIO_Pin = DS1302_SCLK_PIN | DS1302_CE_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(DS1302_SCLK_PORT, &GPIO_InitStructure);
    
    GPIO_ResetBits(DS1302_SCLK_PORT, DS1302_SCLK_PIN);
    GPIO_ResetBits(DS1302_CE_PORT, DS1302_CE_PIN);
}
 
void DS1302_DATAOUT_init()//配置双向I/O端口为输出态
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(DS1302_CLK, ENABLE);
	 
	GPIO_InitStructure.GPIO_Pin = DS1302_DATA_PIN; // DATA
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(DS1302_DATA_PORT, &GPIO_InitStructure);//初始化
	GPIO_ResetBits(DS1302_DATA_PORT,DS1302_DATA_PIN);
}
 
void DS1302_DATAINPUT_init()//配置双向I/O端口为输入态
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(DS1302_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = DS1302_DATA_PIN; //DATA
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DS1302_DATA_PORT, &GPIO_InitStructure);//初始化
}
 
 
void DS1302_write_onebyte(u8 data)//向DS1302发送一字节数据
{
		u8 count=0;
		SCLK_L;
		DS1302_DATAOUT_init();
 
 
	for(count=0;count<8;count++)
		{	SCLK_L;
			if(data&0x01)
			{DATA_H;}
			else{DATA_L;}//先准备好数据再发送
			SCLK_H;//拉高时钟线，发送数据
			data>>=1;
		}
}
 
void DS1302_wirte_rig(u8 address,u8 data)//向指定寄存器地址发送数据
{
	u8 temp1=address;
	u8 temp2=data;
	CE_L;SCLK_L;Delay_us(1);
	CE_H;Delay_us(3);
	DS1302_write_onebyte(temp1);
	DS1302_write_onebyte(temp2);
	CE_L;SCLK_L;Delay_us(3);
}
 
u8 DS1302_read_rig(u8 address)
{
    u8 count, return_data = 0x00;
    
    CE_L; SCLK_L; DS1302_SimpleDelay(2);
    CE_H; DS1302_SimpleDelay(2);
    
    // 1. 先写地址（切换为输出模式）
    DS1302_DATAOUT_init();
    DS1302_write_onebyte(address);
    
    // 2. 准备读数据（切换为输入模式）
    DS1302_DATAINPUT_init();
    DS1302_SimpleDelay(2);
    
    for(count=0; count<8; count++)
    {
        return_data >>= 1;
        SCLK_H; DS1302_SimpleDelay(2); // 高电平持续
        SCLK_L; DS1302_SimpleDelay(2); // 下降沿后 DS1302 输出数据
        if(GPIO_ReadInputDataBit(DS1302_DATA_PORT, DS1302_DATA_PIN))
        {
            return_data |= 0x80;
        }
    }
    CE_L; 
    DS1302_SimpleDelay(2);
    return return_data;
}
 
void DS1302_Init(void)
{
	DS1302_wirte_rig(0x8e,0x00);//关闭写保护
	DS1302_wirte_rig(0x80,0x00);//seconds00秒
	DS1302_wirte_rig(0x82,0x25);//minutes30分
	DS1302_wirte_rig(0x84,0x20);//hours20时
	DS1302_wirte_rig(0x86,0x17);//date3日
	DS1302_wirte_rig(0x88,0x2);//months月
	DS1302_wirte_rig(0x8a,0x02);//days星期
	DS1302_wirte_rig(0x8c,0x26);//year年
	DS1302_wirte_rig(0x8e,0x80);//开启写保护
}
 
void DS1302_read_time(void)
{
	read_time[0]=DS1302_read_rig(0x81);//读秒
	read_time[1]=DS1302_read_rig(0x83);//读分
	read_time[2]=DS1302_read_rig(0x85);//读时
	read_time[3]=DS1302_read_rig(0x87);//读日
	read_time[4]=DS1302_read_rig(0x89);//读月
	read_time[5]=DS1302_read_rig(0x8B);//读星期
	read_time[6]=DS1302_read_rig(0x8D);//读年
}
 
void DS1302_read_realTime(void)
{
    // 进入临界区，防止读取时被其他任务打断导致时序混乱卡死
    vTaskSuspendAll(); 
    
    DS1302_read_time(); 
    
    xTaskResumeAll(); 

    // BCD 转换逻辑保持不变
    TimeData.second = (read_time[0]>>4)*10 + (read_time[0]&0x0f);
    TimeData.minute = (read_time[1]>>4)*10 + (read_time[1]&0x0f);
    TimeData.hour   = (read_time[2]>>4)*10 + (read_time[2]&0x0f);
    TimeData.day    = (read_time[3]>>4)*10 + (read_time[3]&0x0f);
    TimeData.month  = (read_time[4]>>4)*10 + (read_time[4]&0x0f);
    TimeData.week   = read_time[5];
    TimeData.year   = (read_time[6]>>4)*10 + (read_time[6]&0x0f) + 2000;    
}
 
 
void DS1302_wirteRAM(void)
{
	DS1302_wirte_rig(0x8e,0x00);//关闭写保护
	DS1302_wirte_rig(0xC0,TimeRAM.hour_kai);//开时
	DS1302_wirte_rig(0xC2,TimeRAM.minute_kai);//开分
	DS1302_wirte_rig(0xC4,TimeRAM.hour_guan);//关时
	DS1302_wirte_rig(0xC6,TimeRAM.minute_guan);//关分
	DS1302_wirte_rig(0xC8,TimeRAM.kai);//关分
	DS1302_wirte_rig(0xCA,TimeRAM.guan);//关分
	DS1302_wirte_rig(0x8e,0x80);//关闭写保护
}
void DS1302_readRAM(void)
{
	TimeRAM.hour_kai=DS1302_read_rig(0xC1);//读秒
	TimeRAM.minute_kai=DS1302_read_rig(0xC3);//读分
	TimeRAM.hour_guan=DS1302_read_rig(0xC5);//读时
	TimeRAM.minute_guan=DS1302_read_rig(0xC7);//读日	
	TimeRAM.kai=DS1302_read_rig(0xC9);//读日
	TimeRAM.guan=DS1302_read_rig(0xCB);//读日
}
