#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "soilHum485.h"

#define UART2_TXD  (GPIO_NUM_17)
#define UART2_RXD  (GPIO_NUM_16)
#define UART2_RTS  (UART_PIN_NO_CHANGE)
#define UART2_CTS  (UART_PIN_NO_CHANGE)

#define RS485RD    (GPIO_NUM_4)

#define PWR_CTL    (GPIO_NUM_26)

#define BUF_SIZE    100

char soilHum485_modbus_send_data[]={0x01,0x03,0x00,0x12,0x00,0x02,0x64,0x0e};//发送查询指令


/*******************************************************************************
// CRC16_Modbus Check
*******************************************************************************/
static uint16_t CRC16_ModBus(uint8_t *buf,uint8_t buf_len)  
{  
  uint8_t i,j;
  uint8_t c_val;
  uint16_t crc16_val = 0xffff;
  uint16_t crc16_poly = 0xa001;  //0x8005
  
  for (i = 0; i < buf_len; i++)
  {
    crc16_val = *buf^crc16_val;
    
    for(j=0;j<8;j++)
    {
      c_val = crc16_val&0x01;
      
      crc16_val = crc16_val >> 1;
      
      if(c_val)
      {
        crc16_val = crc16_val^crc16_poly;
      }
    }
    buf++;
  }
  //printf("crc16_val=%x\n",crc16_val);
  return ((crc16_val&0x00ff)<<8)|((crc16_val&0xff00)>>8);
}

void soilHum485_Read(void)
{

    uint8_t data_u2[BUF_SIZE];
    gpio_set_level(RS485RD, 1); //RS485输出模式
    uart_write_bytes(UART_NUM_2, soilHum485_modbus_send_data, 8);
    vTaskDelay(15 / portTICK_PERIOD_MS);
    gpio_set_level(RS485RD, 0); //RS485输入模式
    int len2 = uart_read_bytes(UART_NUM_2, data_u2, BUF_SIZE, 20 / portTICK_RATE_MS);
    if(len2!=0)
    {
        printf("UART2 recv:");
        for(int i=0;i<len2;i++)
        {
            printf("%02x ",data_u2[i]);
        }
        printf("\r\n");

        // uint16_t crc_check=CRC16_ModBus(data_u2,7);
        // printf("crc-check=%x\n",crc_check);
        // printf("crc-check2=%x\n",(data_u2[7]*256+data_u2[8]));
        // printf("u25=%x,u26=%x\n",data_u2[7],data_u2[8]);
        if((data_u2[7]*256+data_u2[8])==CRC16_ModBus(data_u2,7))
        {
            soilHum485=data_u2[3]*256+data_u2[4];
            printf("soilHum485=%f\n",(float)(soilHum485/10.0));
            soilTemp485=data_u2[5]*256+data_u2[6];
            if(soilTemp485>2000)
            {
              soilTemp485=0;
            }
            printf("soilTemp485=%f\n",(float)(soilTemp485/10.0));
        }
        else
        {
            soilHum485=-1;
            printf("soilHum485 crc error\n");
        }

        len2=0;
    }
    else//未接
    {
        soilHum485=-1;
    }
}

void soilHum485_pwroff(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    io_conf.pin_bit_mask = (1<<PWR_CTL);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1<<RS485RD);
    gpio_config(&io_conf);

    gpio_set_level(PWR_CTL, 0);//1=on 0=off
}

void soilHum485_Init(void)
{
    
    /**********************uart init**********************************************/
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };


    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, UART2_TXD, UART2_RXD, UART2_RTS, UART2_CTS);
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);
    
    /******************************gpio init*******************************************/
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO16
    io_conf.pin_bit_mask = (1<<RS485RD);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1<<PWR_CTL);
    gpio_config(&io_conf);

    gpio_set_level(PWR_CTL, 1);//1=on 0=off
    vTaskDelay(300 / portTICK_RATE_MS);
    for(int i=0;i<10;i++)
    {
      soilHum485_Read();
      vTaskDelay(500 / portTICK_RATE_MS);
    }


}