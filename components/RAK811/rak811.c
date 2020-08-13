#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "Led.h"
#include "rak811.h"


#define UART1_TXD           (GPIO_NUM_22)
#define UART1_RXD           (GPIO_NUM_23)
#define UART1_RTS           (UART_PIN_NO_CHANGE)
#define UART1_CTS           (UART_PIN_NO_CHANGE)
#define GPIO_POWER811       (GPIO_NUM_32)

#define BUF_SIZE    1000


static const char *TAG = "RAK811";

void RAK811_Read_Task(void* arg);

void RAK811_pwroff(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    io_conf.pin_bit_mask = (1ULL<<GPIO_POWER811);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);  

    gpio_set_level(GPIO_POWER811, 0);

}

void POWER811_CTL_INIT()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<GPIO_POWER811);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);  

    gpio_set_level(GPIO_POWER811, 0);
    vTaskDelay(300 / portTICK_RATE_MS);
    gpio_set_level(GPIO_POWER811, 1);
}

void RAK811_Init(void)
{
  
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART1_TXD, UART1_RXD, UART1_RTS, UART1_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
    
    vTaskDelay(100 / portTICK_RATE_MS);
    xTaskCreate(&RAK811_Read_Task, "RAK811_Read_Task", 10240, NULL, 10, NULL);
    POWER811_CTL_INIT();
    
    vTaskDelay(1000 / portTICK_RATE_MS);
    const char Send811_join[]="AT+CJOIN=1,1\r\n";
    uart_write_bytes(UART_NUM_1, Send811_join, sizeof(Send811_join));
}


void RAK811_Read_Task(void* arg)
{
    uint8_t data_u1[BUF_SIZE];
    while(1)
    {
        int len1 = uart_read_bytes(UART_NUM_1, data_u1, BUF_SIZE, 20 / portTICK_RATE_MS);
        if(len1!=0)  //读取到传感器数据
        {
            
            printf("len=%d,811=%s\n",len1,data_u1);
            len1=0;
            bzero(data_u1,sizeof(data_u1));                 
        }  
        vTaskDelay(5 / portTICK_RATE_MS); 
    }   
}


