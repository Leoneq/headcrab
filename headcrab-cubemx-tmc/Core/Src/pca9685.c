/**
  ******************************************************************************
  * @file           : pca9685.c
  * @brief          : PCA9685 library source file
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
#include "pca9685.h"
#include "i2c.h"

#define PCA9685_ADDRESS 0x80
//registers
#define PCA9685_MODE1 0x00      
#define PCA9685_MODE2 0x01      
#define PCA9685_SUBADR1 0x02    
#define PCA9685_SUBADR2 0x03    
#define PCA9685_SUBADR3 0x04    
#define PCA9685_ALLCALLADR 0x05 
#define PCA9685_LED0_ON_L 0x06  
#define PCA9685_LED0_ON_H 0x07  
#define PCA9685_LED0_OFF_L 0x08 
#define PCA9685_LED0_OFF_H 0x09 
// etc all 16:  LED15_OFF_H 0x45

#define PCA9685_ALLLED_ON_L 0xFA  
#define PCA9685_ALLLED_ON_H 0xFB  
#define PCA9685_ALLLED_OFF_L 0xFC 
#define PCA9685_ALLLED_OFF_H 0xFD 
#define PCA9685_PRESCALE 0xFE

// MODE1 bits
#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02    /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04    /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08    /**< respond to I2C-bus subaddress 1 */
#define PCA9685_BIT_SLEEP 0x04   /**< Low power mode. Oscillator off */
#define PCA9685_BIT_AI 0x05      /**< Auto-Increment enabled */
#define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock */
#define PCA9685_BIT_RESTART 0x07 /**< Restart enabled */
// MODE2 bits
#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1 0x02 /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 0x10  /**< Output logic state inverted */

const int servoMinMaxValues[16][2] = 
{
  {115, 430},
  {160, 510},
  {100, 545},
  {0, 0},
  {150, 500},
  {150, 505},
  {100, 530},
  {0, 0},
  {180, 500},
  {100, 505},
  {120, 550}, 
  {0, 0},
  {130, 450},
  {130, 485},
  {110, 495},
  {0, 0}
};

uint8_t PCA_read8(uint8_t reg)
{
	  uint8_t value = 0;
	  HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, reg, 1, &value, 1, HAL_MAX_DELAY);
	  return value;
}

void PCA_write8(uint8_t reg, uint8_t val)
{
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, reg, 1, &val, 1, HAL_MAX_DELAY);
}

void PCA_reset()
{
    uint8_t rst = 0x06;
    HAL_I2C_Master_Transmit(&hi2c1, 0x00, &rst, 1, HAL_MAX_DELAY);
    HAL_Delay(10);
}

//register, bit place, the bit (lol)
void PCA_setBit(uint8_t reg, uint8_t bit, bool val)
{
    uint8_t value = PCA_read8(reg);

	  if(val) value |= val << bit;
    else value &= ~(!val << bit);

    /*
    char b[10];
    itoa(value, b, 16);
    char msg[32] = "VALUE: ";
    strcat(msg, b);
    debug_serialWrite(msg, 1);
    */

    PCA_write8(reg, value);
}

void PCA_sleepMode(bool s)
{
    PCA_setBit(PCA9685_MODE1, PCA9685_BIT_SLEEP, s);
}

void PCA_restartMode(bool s)
{
    PCA_setBit(PCA9685_MODE1, PCA9685_BIT_RESTART, s);
}
void PCA_autoIncrement(bool s)
{
  	PCA_setBit(PCA9685_MODE1, PCA9685_BIT_AI, s);
}

void PCA_setFreq(int f)
{
    uint8_t prescale = 0;
    if(f >= 1526)
        prescale = 0x03;
    else if(f <= 24)
        prescale = 0xFF;
    else 
    {
        float val = (25000000 / (4096 * (float)f)) - 1;
        prescale = round(val);
    }

    PCA_sleepMode(1);
    PCA_write8(PCA9685_PRESCALE, prescale);
    PCA_sleepMode(0);
    PCA_restartMode(1);
}

void PCA_setPWM(uint8_t channel, int ontime, int offtime)
{
    uint8_t reg;
    uint8_t buffer[4];

    reg = PCA9685_LED0_ON_L + (4 * channel);
    buffer[0] = ontime;
    buffer[1] = ontime >> 8;
    buffer[2] = offtime;
    buffer[3] = offtime >> 8;

    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, reg, 1, buffer, 4, HAL_MAX_DELAY);
}

void PCA_setPin(uint8_t channel, int value)
{
  if(value == 4095)
      PCA_setPWM(channel, 4096, 0);
  else if(value == 0)
      PCA_setPWM(channel, 0, 4096);
  else
      PCA_setPWM(channel, 0, value);
}

//straight from arduino ide :3
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void PCA_setServoAngle(uint8_t channel, float angle)
{
    int a;
    if(angle < 0) a = 0;
    else a = map((uint16_t)angle, 0, 180, servoMinMaxValues[channel][0], servoMinMaxValues[channel][1]);
    PCA_setPin(channel, a);
    //HAL_Delay(1);
    //PCA_setPin(channel, servoMinMaxValues[channel][0]);
    //HAL_Delay(1);
    //PCA_setPin(channel, a);
}

void PCA_begin()
{
    PCA_reset();
    PCA_setFreq(50);
    PCA_autoIncrement(true);
}