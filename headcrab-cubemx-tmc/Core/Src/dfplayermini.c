/**
  ******************************************************************************
  * @file           : dfplayermini.c
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
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "dfplayermini.h"
#include "usart.h"

void DFPlayer_enableACK()
{
    DFPlayer_ACK = 0x01;
}

void DFPlayer_disableACK()
{
    DFPlayer_ACK = 0x00;
}


void DFPlayer_sendCommand(int cmd, int arg)
{
  uint8_t command[10] = {0x7E, 0xFF, 6, cmd, DFPlayer_ACK, arg >> 8, arg, 0, 0, 0xEF};

  //calculate checksum
  int checksum = 0;
  for(int x = 1; x <= 6; x++)
      checksum -= command[x];
  command[7] = checksum >> 8;
  command[8] = checksum;

  HAL_UART_Transmit(&huart2, (uint8_t*)&command, 10, HAL_MAX_DELAY);
}

bool DFPlayer_checkPacket()
{
    bool s = true;
    if(DFPlayer_buffer[0] != 0x7E) s = false;
    if(DFPlayer_buffer[1] != 0xFF) s = false;
    if(DFPlayer_buffer[2] != 0x06) s = false;
    //should count bytes and checksum but eh
    if(DFPlayer_buffer[9] != 0xEF) s = false;
    return s;
}

void DFPlayer_executeSerial()
{
    char msg[64] = "DFPlayer: ";
    switch(DFPlayer_buffer[3])
    {
        case DF_ONLINE:
            switch(DFPlayer_buffer[6])
            {
                case DF_UDISK:
                    strcat(msg, "USB Disk online");
                    break;

                case DF_SDCARD:
                    strcat(msg, "SD Card online");
                    break;

                case DF_USBPC:
                    strcat(msg, "PC connection online");
                    break;
                
                case DF_USB_SD:
                    strcat(msg, "USB Disk and SD Card online");
                    break;
            }
            break;

        case DF_SD_DONE:
            strcat(msg, "successfully played file nr. ");
            char b[8];
            itoa(DFPlayer_buffer[6]-1, b, 10);
            strcat(msg, b);
            break;

        case DF_ACK:
            strcat(msg, "ok");
            break;

        case DF_ERROR:
            strcat(msg, "error, ");
            switch(DFPlayer_buffer[6])
            {
                case DF_BUSY:
                    strcat(msg, "module is busy");
                    break;

                case DF_SLEEP:
                    strcat(msg, "module is sleeping");
                    break;

                case DF_RXERROR:
                    strcat(msg, "received bad frame");
                    break;

                case DF_CHECKSUM:
                    strcat(msg, "bad checksum");
                    break;

                case DF_OUTOFTR:
                    strcat(msg, "track is out of current track scope");
                    break;

                case DF_NOTFOUND:
                    strcat(msg, "file not found");
                    break;

                case DF_INTERCUT:
                    strcat(msg, "intercut error");
                    break;

                case DF_SDREAD:
                    strcat(msg, "SD Card failure");
                    break;

                case DF_SLEEPENT:
                    strcat(msg, "entered into sleep mode");
                    break;
            }
            break;

        case DF_PLUGIN:
            switch(DFPlayer_buffer[6])
            {
                case DF_UDISK:
                    strcat(msg, "USB Disk plugged in");
                    break;

                case DF_SDCARD:
                    strcat(msg, "SD Card plugged in");
                    break;
                
                case DF_USBPC:
                    strcat(msg, "PC connection estabilished");
                    break;
            }
            break;
        
        case DF_PLUGOUT:
            switch(DFPlayer_buffer[6])
            {
                case DF_UDISK:
                    strcat(msg, "USB Disk plugged out");
                    break;

                case DF_SDCARD:
                    strcat(msg, "SD Card plugged out");
                    break;
                
                case DF_USBPC:
                    strcat(msg, "PC connection has been lost");
                    break;
            }
            break;
    }
    debug_serialWrite(msg, 1);
}

DFPlayer_INIT DFPlayer_Init(UART_HandleTypeDef *uart)
{
    DFPlayer_disableACK();
    DFPlayer_pointer = 0;
    DFPlayer_reset();
    HAL_UART_Receive(&huart2, (uint8_t *)&DFPlayer_buffer[DFPlayer_pointer], 1, 2000);
    while(DFPlayer_pointer < 10) //block everything until receive data
    {
        if(HAL_UART_Receive(&huart2, (uint8_t *)&DFPlayer_buffer[DFPlayer_pointer], 1, 2000) == HAL_OK)
        {
            DFPlayer_pointer++;
        }
        else
        {
            return DF_IERROR;
        }
    }
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&DFPlayer_buffer[DFPlayer_pointer], 1);
    if(!DFPlayer_checkPacket()) return DF_IERROR;
    else DFPlayer_executeSerial();
    DFPlayer_pointer = 0;

    return DFPlayer_buffer[6];
}

void DFPlayer_handleSerial()
{
    if(DFPlayer_buffer[DFPlayer_pointer] == 0xEF)
    {
        if(DFPlayer_checkPacket()) DFPlayer_executeSerial();
        for(int x = 0; x < sizeof(DFPlayer_buffer); x++)
            DFPlayer_buffer[x] = 0;
        DFPlayer_pointer = 0;
    }
    else
    {
        DFPlayer_pointer++;
    }
}

void DFPlayer_play(int nr)
{
    DFPlayer_sendCommand(3, nr+1);
}

void DFPlayer_setVolume(int vol)
{
    DFPlayer_sendCommand(6, vol);
}

void DFPlayer_reset()
{
    DFPlayer_sendCommand(0x0C, 0);
}

void DFPlayer_setEqualizer(int eq)
{
    DFPlayer_sendCommand(0x07, eq);
}

void DFPlayer_sleep()
{
    DFPlayer_sendCommand(0x0A, 0);
}

