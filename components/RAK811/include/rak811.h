
#ifndef _RAK811_H_
#define _RAK811_H_
#include "freertos/FreeRTOS.h"

uint16_t PM2_5;
uint16_t PM10;


extern void RAK811_Init(void);
extern void RAK811_pwroff(void);

#endif

