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
    DF_USB_DISK = 1,
    DF_SD       = 2,
    DF_USB_PC   = 4,
    DF_USB_SD   = 3
} DFPlayer_INIT;

typedef enum
{
    DF_USB_DISK = 1,
    DF_SD       = 2,
    DF_USB_PC   = 4,
    DF_USB_SD   = 3
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
    DF_UDISK    = 1,
    DF_SD_CARD  = 2,
    DF_USBPC    = 4
} DFPlayer_DEVICE_PLUG;
// send/receive uart buffer
char dfplayer_buffer[11];
int dfplayer_pointer = 0;

// send additional information
bool dfplayer_debug = true;
//disable if you want to only send data
uint8_t DFPlayer_ACK = 0;

class DFPlayerMini
{
    unsigned long _timeOutTimer;
    unsigned long _timeOutDuration = 500;
    
}

#endif