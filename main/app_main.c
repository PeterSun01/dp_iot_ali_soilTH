#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "Nvs.h"
#include "Uart0.h"
#include "driver/gpio.h"

#include "Led.h"
#include "E2prom.h"
#include "RtcUsr.h"
#include "sht31.h"
#include "rak811.h"
#include "driver/adc.h"
#include "esp_system.h"
#include "esp_adc_cal.h"
#include "soilHum485.h"


char Send811_data[100]={0};

void timer_periodic_cb(void *arg); 
esp_timer_handle_t timer_periodic_handle = 0; //定时器句柄


esp_timer_create_args_t timer_periodic_arg = {
    .callback =
        &timer_periodic_cb, 
    .arg = NULL,            
    .name = "PeriodicTimer" 
};



void timer_periodic_cb(void *arg) //1ms中断一次
{
  static int64_t timer_count = 0;
  static uint8_t send_conut=0;//发送次数
  timer_count++;
  if (timer_count >= 10000) //第一次30S等待土壤传感器稳定，第二次10s
  {
    timer_count = 5000;
    printf("Free memory: %d bytes\n", esp_get_free_heap_size());
  
    soilHum485_Read();
    bzero(Send811_data,sizeof(Send811_data));
    //soilHumidity=0899
    //soilTemperature=0830
    //tpye=8001
    uint8_t data_len=sprintf(Send811_data,"AT+DTX=24,0830%04X0899%04X80010005\r\n",soilTemp485,soilHum485);
    ESP_LOGI("811DATA=", "%s", Send811_data);
    uart_write_bytes(UART_NUM_1, Send811_data, data_len);
    if(send_conut>=2)//休眠1min
    {
      send_conut=0;
      vTaskDelay(3000 / portTICK_RATE_MS);
      //关闭外设电源
      soilHum485_pwroff();
      RAK811_pwroff();
      const int wakeup_time_sec = 600;
      printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
      esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);
      printf("Entering deep sleep\n");
      esp_deep_sleep_start();
    }
    send_conut++;

  }
}

static void Uart0_Task(void* arg)
{
    while(1)
    {
        Uart0_read();
        vTaskDelay(1000 / portTICK_RATE_MS);
    }  
}




void app_main(void)
{
  Led_Init();
  Uart0_Init();
  RAK811_Init();
  //i2c_init();
  soilHum485_Init();

  xTaskCreate(Uart0_Task, "Uart0_Task", 4096, NULL, 10, NULL);

  /*******************************timer 1s init**********************************************/
  esp_err_t err = esp_timer_create(&timer_periodic_arg, &timer_periodic_handle);
  err = esp_timer_start_periodic(timer_periodic_handle, 1000); //创建定时器，单位us，定时1ms
  if (err != ESP_OK)
  {
    printf("timer periodic create err code:%d\n", err);
  }


}
