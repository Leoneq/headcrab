/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    GPIO_TypeDef* port;
    uint16_t pin;
} pinPort;

typedef struct
{
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;
} dateTime;




/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG
#define CRLF 1
#define NOCRLF 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const pinPort LEDs[] =
{
  {LD3_GPIO_Port, LD3_Pin},
  {LD5_GPIO_Port, LD5_Pin},
  {LD7_GPIO_Port, LD7_Pin},
  {LD9_GPIO_Port, LD9_Pin},
  {LD10_GPIO_Port, LD10_Pin},
  {LD8_GPIO_Port, LD8_Pin},
  {LD6_GPIO_Port, LD6_Pin},
  {LD4_GPIO_Port, LD4_Pin},
};

char huart2_buffer[10];
int huart2_pointer = 0;

char huart1_buffer[128];
int huart1_pointer = 0;

bool debug_echo = true;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//turn on/off builtin leds
void led_set(int led, bool turn_on)
{ 
	GPIO_PinState state = (turn_on) ? GPIO_PIN_SET : GPIO_PIN_RESET;
 
	if (led >= 0 && led < 8)
		HAL_GPIO_WritePin(LEDs[led].port, LEDs[led].pin, state);
}

void led_toggle(int led)
{
    if (led >= 0 && led < 8)
		    HAL_GPIO_TogglePin(LEDs[led].port, LEDs[led].pin);
}

// on stm32f303 board theres only one user button, and we probably wont need more.
bool is_button_pressed()
{
  return (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_SET) ? true : false;
}

// get time
dateTime get_date_time()
{
    dateTime t;
    HAL_RTC_GetTime(&hrtc, &t.time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &t.date, RTC_FORMAT_BIN);
    return t;
}

//send debug message with additional info
void debug_serialWrite(char* message, bool nl)
{
  //please help me i hate strings this is the only case where java is better than c plus plus
  char msg[64] = "";
  char hours[] = "";
  char minutes[] = "";
  char seconds[] = "";

  itoa(get_date_time().time.Hours, hours, 10);
  itoa(get_date_time().time.Minutes, minutes, 10);
  itoa(get_date_time().time.Seconds, seconds, 10);

  if(nl) 
  {
      strcat(msg, hours);
      strcat(msg, ":");
      strcat(msg, minutes);
      strcat(msg, ":");
      strcat(msg, seconds);
      strcat(msg, "\t>> ");
  }
  strcat(msg, message);
  if(nl) strcat(msg, "\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)&msg, strlen(msg), HAL_MAX_DELAY);
}

//send a command to dfplayer
void wav_sendcommand(int cmd, int arg)
{
  // the command consists of severals bytes. respectively:
  //start byte, version, length, command, feedback, parameter, checksum, end byte
  uint8_t command[10] = {0x7E, 0xFF, 6, cmd, 0, arg >> 8, arg, 0, 0, 0xEF};

  //calculate checksum
  int checksum = 0;
  for(int x = 1; x <= 6; x++)
      checksum -= command[x];
  command[7] = checksum >> 8;
  command[8] = checksum;

  //send debug information
  #ifdef DEBUG
      char msg[64] = "DFPlayer send: ";
      for(int x = 0; x < 10; x++)
      {
          char buffer[3] = "";
          itoa(command[x], buffer, 16);
          strcat(buffer, " ");
          strcat(msg, buffer);
      }
      debug_serialWrite(msg, CRLF);
  #endif

  HAL_UART_Transmit(&huart2, (uint8_t*)&command, 10, HAL_MAX_DELAY);
}

// play a song
void wav_play(int nr)
{
    wav_sendcommand(3, nr);
}

void wav_setvolume(int vol)
{
    wav_sendcommand(6, vol);
}

// split received data and execute a command
void debug_executeSerial()
{
    if(debug_echo)
    {
        debug_serialWrite("echo: ", NOCRLF);
        debug_serialWrite(huart1_buffer, CRLF);
    }

    char *ptr = strtok(huart1_buffer, "_");
    if(strcmp(ptr, "dfplayer") == 0)
    {
        ptr = strtok(NULL, "_");
        if(strcmp(ptr, "play") == 0)
        {
            ptr = strtok(NULL, "_");
            wav_play(atoi(ptr));
        }
        else if(strcmp(ptr, "setvolume") == 0)
        {
            ptr = strtok(NULL, "_");
            wav_setvolume(atoi(ptr));
        }
        else
            goto error;
    }
    else if(strcmp(ptr, "headcrab") == 0)
    {
        ptr = strtok(NULL, "_");
        if(strcmp(ptr, "echo") == 0)
        {
            ptr = strtok(NULL, "_");
            debug_echo = atoi(ptr);
        }
        else if(strcmp(ptr, "ping!") == 0)
        {
            debug_serialWrite("pong!", CRLF);
        }
        else
            goto error;
    }
    else if(strcmp(ptr, "led") == 0)
    {
        ptr = strtok(NULL, "_");
        if(strcmp(ptr, "toggle") == 0)
        {
            ptr = strtok(NULL, "_");
            led_toggle(atoi(ptr));
        }
        else if(strcmp(ptr, "set") == 0)
        {
            ptr = strtok(NULL, "_");
            led_set(atoi(ptr), 1);
        }
        else if(strcmp(ptr, "reset") == 0)
        {
            ptr = strtok(NULL, "_");
            led_set(atoi(ptr), 0);
        }
        else
          goto error;
    }
    else
    {
        error:
        debug_serialWrite("error: invalid command", CRLF);
    }
    
}

// read a character from serial and add to the buffer
void debug_handleSerial()
{
    uint8_t rec;
    if(HAL_UART_Receive(&huart1, &rec, 1, 0) == HAL_OK)
        led_set(0, 1);
    else
    {
        led_set(0, 0);
        return;
    }

    if(rec == '\n')
    {
      huart1_buffer[huart1_pointer] = '\0';
      debug_executeSerial();
      for(int x = 0; x < sizeof(huart1_buffer); x++)
          huart1_buffer[x] = 0;
      huart1_pointer = 0;
    }
    else
    {
        huart1_buffer[huart1_pointer] = rec;
        huart1_pointer++;
    }
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  const char message[] = "hai";
  debug_serialWrite((char*)message, CRLF);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      debug_handleSerial();
      


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
