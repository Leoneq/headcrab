/**
  ******************************************************************************
  * @file           : dfplayermini.h
  * @brief          : DFPlayer Mini library
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

#ifndef DFPLAYER_MINI_H //we want to include library only once
#define DFPLAYER_MINI_H

#include <stdint.h>
#include <stdbool.h>
#include <usart.h>

// equalizer settings
typedef enum
{
    DF_NORMAL  = 0,
    DF_POP     = 1,
    DF_ROCK    = 2,
    DF_JAZZ    = 3,
    DF_CLASSIC = 4,
    DF_BASS    = 5 
} DFPlayer_EQ;

// initialization state
typedef enum
{
    DF_IERROR   = 0,
    DF_USB_DISK = 1,
    DF_SD       = 2,
    DF_USB_PC   = 4,
} DFPlayer_INIT;

typedef enum
{
    DF_BUSY     = 1,
    DF_SLEEP    = 2,
    DF_RXERROR  = 3,
    DF_CHECKSUM = 4,
    DF_OUTOFTR  = 5,
    DF_NOTFOUND = 6,
    DF_INTERCUT = 7,
    DF_SDREAD   = 8,
    DF_SLEEPENT = 0x0A
} DFPlayer_ERROR;

typedef enum
{
    DF_ONLINE   = 0x3F,
    DF_USB_DONE = 0x3C,
    DF_SD_DONE  = 0x3D,
    DF_ACK      = 0x41,
    DF_ERROR    = 0x40,
    DF_PLUGIN   = 0x3A,
    DF_PLUGOUT  = 0x3B,
    DF_STATUS   = 0x42,
} DFPlayer_COMMAND;

typedef enum
{
    DF_UDISK    = 1,
    DF_SDCARD   = 2,
    DF_USBPC    = 4,
    DF_USB_SD   = 3
} DFPlayer_DEVICE_PLUG;

// send/receive uart buffer
char DFPlayer_buffer[10];
int DFPlayer_pointer;

// send additional information
bool DFPlayer_debug;
//disable if you want to only send data
uint8_t DFPlayer_ACK;

unsigned long DFPlayer_timeOutTimer;
unsigned long DFPlayer_timeOutDuration;

void DFPlayer_enableACK();
void DFPlayer_disableACK();
DFPlayer_INIT DFPlayer_Init(UART_HandleTypeDef *huart);
void DFPlayer_reset();
void DFPlayer_handleSerial();
void DFPlayer_play(int nr);
void DFPlayer_sleep();
void DFPlayer_setEqualizer(int eq);
void DFPlayer_setVolume(int vol);

#endif