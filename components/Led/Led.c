#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "Led.h"

#define GPIO_LED_STA_R        18
#define GPIO_LED_STA_G        5

static void Led_Task(void* arg)
{
    while(1)
    {
        vTaskDelay(10 / portTICK_RATE_MS);
        switch(Led_Status)
        {
            // case LED_STA_INIT:   //初始化
            //     Led_CTL_B_On();
            //     vTaskDelay(10 / portTICK_RATE_MS);
            //     break;
            
            // case LED_STA_TOUCH:   //等待配网
            //     Led_CTL_R_On();
            //     vTaskDelay(300 / portTICK_RATE_MS);
            //     Led_CTL_B_On();
            //     vTaskDelay(300 / portTICK_RATE_MS);
            //     break;
            
            // case LED_STA_NOSER:  //无序列号
            //     Led_CTL_R_On();
            //     vTaskDelay(10 / portTICK_RATE_MS);
            //     break;
            
            // case LED_STA_WIFIERR:  //wifi连接错误
            //     Led_CTL_R_On();
            //     vTaskDelay(300 / portTICK_RATE_MS);
            //     Led_CTL_Off();
            //     vTaskDelay(300 / portTICK_RATE_MS);
            //     break;

            // case LED_STA_SENDDATA://发送数据
            //     Led_CTL_B_On();
            //     vTaskDelay(200 / portTICK_RATE_MS);
            //     Led_CTL_Off();
            //     Led_Status=LED_STA_SENDDATAOVER;
            //     break;

            // case LED_STA_SENDDATAOVER:  //发送数据结束
            //     Led_CTL_Off();
            //     vTaskDelay(10 / portTICK_RATE_MS);
            //     break;

            // case LED_STA_SENDDATAERR: //发送数据失败，如用户名错误
            //     Led_CTL_R_On();
            //     vTaskDelay(200 / portTICK_RATE_MS);
            //     Led_CTL_Off();
            //     Led_Status=LED_STA_SENDDATAOVER;
            //     break;               

        }
    }
}

void Led_Init(void)
{
    gpio_config_t io_conf;

    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO16
    io_conf.pin_bit_mask = (1ULL<<GPIO_LED_STA_R);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);  

    io_conf.pin_bit_mask = (1ULL<<GPIO_LED_STA_G);
    gpio_config(&io_conf); 

    Led_STA_G_On();

    Led_Status=LED_STA_INIT;

    xTaskCreate(Led_Task, "Led_Task", 4096, NULL, 5, NULL);

}



void Led_STA_R_On(void)  
{
    gpio_set_level(GPIO_LED_STA_R, 0);
    gpio_set_level(GPIO_LED_STA_G, 1);
}

void Led_STA_G_On(void)
{
    gpio_set_level(GPIO_LED_STA_R, 1);
    gpio_set_level(GPIO_LED_STA_G, 0);
}





