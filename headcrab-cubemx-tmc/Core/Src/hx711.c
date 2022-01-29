/**
  ******************************************************************************
  * @file           : xh711.c
  * @brief          : XH711 library source file
  ******************************************************************************
  * @attention
  *
  * <h2><center> by leonek
  * have fun</center></h2>
  *
  * gnu gpl v3.0 licence forever!~
  *
  ******************************************************************************
  */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "hx711.h"
#include "gpio.h"
#include "main.h"

typedef struct
{
    GPIO_TypeDef* clk_port;
    uint16_t clk_pin;
    GPIO_TypeDef* data_port;
    uint16_t data_pin;
} HX_ADC;

static const HX_ADC HXes[] =
{
  {HX0_CLOCK_GPIO_Port, HX0_CLOCK_Pin, HX0_DATA_GPIO_Port, HX0_DATA_Pin},
  {HX1_CLOCK_GPIO_Port, HX1_CLOCK_Pin, HX1_DATA_GPIO_Port, HX1_DATA_Pin},
  {HX2_CLOCK_GPIO_Port, HX2_CLOCK_Pin, HX2_DATA_GPIO_Port, HX2_DATA_Pin},
  {HX3_CLOCK_GPIO_Port, HX3_CLOCK_Pin, HX3_DATA_GPIO_Port, HX3_DATA_Pin}
};

#define BITS_A_128   25
#define BITS_B_32    26
#define BITS_A_64    27

#define CLOCK_BITS BITS_A_128

long offset = 0;	// used for tare weight
float scale = 0;	// used to return weight in grams, kg, ounces, whatever

void HX_begin(HX_nr module)
{
    while(HAL_GPIO_ReadPin(HXes[module].data_port, HXes[module].data_pin)); //wait til ready
    for(int x = 0; x < CLOCK_BITS; x++)
    {
        HAL_GPIO_WritePin(HXes[module].data_port, HXes[module].data_pin, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(HXes[module].data_port, HXes[module].data_pin, GPIO_PIN_SET);
        HAL_Delay(0);
    }
}

//read
uint32_t HX_read(HX_nr module)
{
    uint32_t result = 0;
    while(HX_check(module)); //wait til ready
    for(int x = 0; x < CLOCK_BITS; x++)
    {
        HAL_GPIO_WritePin(HXes[module].clk_port, HXes[module].clk_pin, GPIO_PIN_SET);
        if(x < 24) result |= HAL_GPIO_ReadPin(HXes[module].data_port, HXes[module].data_pin) << (24 - x);
        HAL_GPIO_WritePin(HXes[module].clk_port, HXes[module].clk_pin, GPIO_PIN_RESET);
    }
    return result;
}

//check the state 
bool HX_check(HX_nr module)
{
    if(HAL_GPIO_ReadPin(HXes[module].data_port, HXes[module].data_pin) == GPIO_PIN_SET)
        return true;
    else
        return false;
}

int HX_readAverage(HX_nr module, int times)
{
    float mean = 0;
    for(int x = 0; x < times; x++)
    {
        mean += HX_read(module);
    }
    return mean / times;
}

void HX_tare(HX_nr module, int times)
{
    offset = HX_readAverage(module, times);
    char msg[32] = "OFFSET: ";
    char buf[10];
    itoa(offset, buf, 10);
    strcat(msg, buf);
    debug_serialWrite(msg, 1);
}

void HX_setScale(HX_nr module, float s)
{
    scale = s;
    char msg[32] = "SCALE: ";
    char buf[10];
    itoa(scale, buf, 10);
    strcat(msg, buf);
    debug_serialWrite(msg, 1);
}

int HX_readValue(HX_nr module, int times)
{
    return HX_readAverage(module, times) - offset;
}

float HX_readUnits(HX_nr module, int times)
{
    return HX_readValue(module, times) / scale;
}
