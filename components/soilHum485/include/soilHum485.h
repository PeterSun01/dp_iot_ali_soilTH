/*
风速传感器读取程序

创建日期：2018年10月29日
作者：孙浩

soilHum485_Init(void);
初始化函数，主要为UART初始化和GPIO初始化
本例使用UART2，9600，8N1
UART2_TXD = (GPIO_NUM_17)
UART2_RXD = (GPIO_NUM_16)
RS485RD   =  21

*/
#ifndef _soilHum485_H_
#define _soilHum485_H_

int soilHum485;
int soilTemp485;

extern void soilHum485_Init(void);
extern void soilHum485_Read(void);
extern void soilHum485_pwroff(void);

#endif

