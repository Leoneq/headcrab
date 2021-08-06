/**
  ******************************************************************************
  * @file           : hx711.h
  * @brief          : HX711 library header file
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

#ifndef HX711_H //we want to include library only once
#define HX711_H

#include <stdint.h>
#include <stdbool.h>

// all hxes
typedef enum
{
    HX_A  = 0, //top right
    HX_B  = 1, //top left
    HX_C  = 2, //bottom left
    HX_D  = 3  //bottom right
} HX_nr;

void HX_begin(HX_nr module);
uint32_t HX_read(HX_nr module);
bool HX_check(HX_nr module);

#endif