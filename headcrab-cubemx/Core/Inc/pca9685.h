/**
  ******************************************************************************
  * @file           : pca9685.h
  * @brief          : PCA9685 PWM library header file
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

#ifndef PCA9685_H //we want to include library only once
#define PCA9685_H

#include <stdint.h>
#include <stdbool.h>

uint8_t PCA_read8(uint8_t reg);
void PCA_write8(uint8_t reg, uint8_t val);
void PCA_reset();
void PCA_setBit(uint8_t reg, uint8_t bit, bool val);
void PCA_sleepMode(bool s);
void PCA_restartMode(bool s);
void PCA_autoIncrement(bool s);
void PCA_setFreq(int f);
void PCA_setPWM(uint8_t channel, int ontime, int offtime);
void PCA_setPin(uint8_t channel, int value);
long map(long x, long in_min, long in_max, long out_min, long out_max);
void PCA_setServoAngle(uint8_t channel, float angle);
void PCA_begin();

#endif