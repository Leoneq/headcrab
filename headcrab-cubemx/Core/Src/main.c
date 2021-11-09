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
#include "iwdg.h"
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
#include "dfplayermini.h"
#include <math.h>
#include "pca9685.h"
#include "hx711.h"
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
#define RAD_TO_DEG 180/M_PI

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

char huart1_buffer[128];
int huart1_pointer = 0;

bool debug_echo = false;

// legs
typedef enum
{
    LEG_A  = 0, //bottom left
    LEG_B  = 1, //bottom right
    LEG_C  = 2, //top right
    LEG_D  = 3  //top left
} LEGS;

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

// toggle yay
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
HAL_StatusTypeDef debug_serialWrite(char* message, bool nl)
{
  //please help me i hate strings this is the only case where java is better than c
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
  return HAL_UART_Transmit(&huart1, (uint8_t*)&msg, strlen(msg), HAL_MAX_DELAY);
}

void moveLeg(LEGS leg, float x, float y, float z)
{
    // units in angles and mm.
    float thigh = 70;
    float hip_angle_offset = -67;
    switch(leg)
    {
      case LEG_A:
        
        break;
      case LEG_B:
        thigh = 70;
        break;
      case LEG_C:
        thigh = 60;
        break;
      case LEG_D:
        thigh = 60;
        break;
      default:
        debug_serialWrite("error: wrong leggie lol", CRLF);
        return;
    }

    const float tibia = 80; // TODO: replace with define or sth
    const float hip_offset = 35;
    const float hip_servo_offset = 49; //the center is at 0,0,0 (middle of the robot)
    const float tibia_offset = 40; //in degrees
    
    x -= hip_servo_offset;
    y -= hip_servo_offset;
    float xy = sqrt(pow(x, 2) + pow(y, 2));
    float hip_angle = (180 - acos(x/xy) * RAD_TO_DEG) + hip_angle_offset;
    //xy -= hip_offset;
    float xyz = sqrt(pow(xy, 2) + pow(z, 2));
    float flat_xyz_angle = acos(xy/xyz) * RAD_TO_DEG;
    float thigh_angle =  acos((pow(tibia, 2)+pow(thigh, 2)-pow(xyz, 2))/(2*tibia*thigh)) * RAD_TO_DEG - flat_xyz_angle + 90;
    float tibia_angle = acos((pow(xyz, 2)+pow(thigh, 2)-pow(tibia, 2))/(2*xyz*thigh)) * RAD_TO_DEG - tibia_offset;

    char msg[90];
    sprintf(msg, "MOVELEG:\ta: %d,\tb: %d,\tc: %d", (int)xy, (int)hip_angle, (int) xyz);
    debug_serialWrite(msg, CRLF);
    HAL_IWDG_Refresh(&hiwdg);

    PCA_setServoAngle(leg*4, hip_angle);
    PCA_setServoAngle(leg*4+1, thigh_angle);
    PCA_setServoAngle(leg*4+2, tibia_angle);
}

// split received data and execute a command
void debug_executeSerial()
{
    if(debug_echo)
    {
        debug_serialWrite("echo: ", NOCRLF);
        debug_serialWrite(huart1_buffer, NOCRLF);
    }

    char *ptr = strtok(huart1_buffer, "_");
    if(strcmp(ptr, "dfp") == 0)
    {
        ptr = strtok(NULL, "_");
        if(strcmp(ptr, "play") == 0)
        {
            ptr = strtok(NULL, "_");
            DFPlayer_play(atoi(ptr));
        }
        else if(strcmp(ptr, "setvolume") == 0)
        {
            ptr = strtok(NULL, "_");
            DFPlayer_setVolume(atoi(ptr));
        }
        else if(strcmp(ptr, "setack") == 0)
        {
            ptr = strtok(NULL, "_");
            if(atoi(ptr)) DFPlayer_enableACK();
            else DFPlayer_disableACK();
        }
        else if(strcmp(ptr, "seteq") == 0)
        {
            ptr = strtok(NULL, "_");
            DFPlayer_setEqualizer(atoi(ptr));
        }
        else if(strcmp(ptr, "reset") == 0)
        {
            DFPlayer_reset();
        }
        else if(strcmp(ptr, "sleep") == 0)
        {
            DFPlayer_sleep();
        }
        else
            goto error;
    }
    else if(strcmp(ptr, "headcrab") == 0)
    {
        ptr = strtok(NULL, "_");
        if(strcmp(ptr, "ping!") == 0)
        {
            debug_serialWrite("pong!", CRLF);
        }
        else if(strcmp(ptr, "echo") == 0)
        {
            ptr = strtok(NULL, "_");
            debug_echo = atoi(ptr);
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
    else if(strcmp(ptr, "pca") == 0)
    {
        ptr = strtok(NULL, "_");
        if(strcmp(ptr, "set") == 0)
        {
            ptr = strtok(NULL, "_");
            if(ptr == NULL) goto error;
            uint8_t channel = atoi(ptr);

            ptr = strtok(NULL, "_");
            if(ptr == NULL) goto error;
            int value = atoi(ptr);

            PCA_setServoAngle(channel, value);
            //PCA_setPin(channel, value);
        }
        else
          goto error;
    }
    else if(strcmp(ptr, "move") == 0)
    {
        ptr = strtok(NULL, "_");
        if(strcmp(ptr, "leg") == 0)
        {
            ptr = strtok(NULL, "_");
            if(ptr == NULL) goto error;
            uint8_t leg = atoi(ptr);

            ptr = strtok(NULL, "_");
            if(ptr == NULL) goto error;
            int x = atoi(ptr);

            ptr = strtok(NULL, "_");
            if(ptr == NULL) goto error;
            int y = atoi(ptr);

            ptr = strtok(NULL, "_");
            if(ptr == NULL) goto error;
            int z = atoi(ptr);

            moveLeg(leg, x, y, z);
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
    if(huart1_buffer[huart1_pointer] == '\n')
    {
      debug_executeSerial();
      for(int x = 0; x < sizeof(huart1_buffer); x++)
          huart1_buffer[x] = 0;
      huart1_pointer = 0;
    }
    else
    {
        huart1_pointer++;
    }
}

// interrupt on receive
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        debug_handleSerial();
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&huart1_buffer[huart1_pointer], 1);
    } 
    else if(huart == &huart2)
    {
        DFPlayer_handleSerial();
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&DFPlayer_buffer[DFPlayer_pointer], 1);
    }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return 1;
}

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
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  debug_serialWrite("headcrab says hi :3", CRLF);

  HAL_UART_Receive_IT(&huart1, (uint8_t *)&huart1_buffer[huart1_pointer], 1);

  DFPlayer_disableACK();
  if(DFPlayer_Init(&huart2) == DF_IERROR)
      debug_serialWrite("DFPlayer: initialize error.", CRLF);
  else
      debug_serialWrite("DFPlayer: initialized successfully", CRLF);
  DFPlayer_play(19);

  PCA_begin();

  HX_begin(0);
  /*
  HX_setScale(0, 1);
  HX_tare(0, 10);
  HX_setScale(0, 1);
  HAL_IWDG_Refresh(&hiwdg);
  HX_setScale(0, HX_readUnits(0, 10));
  */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      led_toggle(6);
      HAL_Delay(100);
      HAL_IWDG_Refresh(&hiwdg);
      if(is_button_pressed())
      {
          //moveLeg(LEG_A, 129,119, 50);
          while(1)
          {
              int delay = 500;
              HAL_IWDG_Refresh(&hiwdg);

              PCA_setServoAngle(0, 70);
              PCA_setServoAngle(4, 110);
              PCA_setServoAngle(8, 70);
              PCA_setServoAngle(12, 110);

              PCA_setServoAngle(1, 120);
              PCA_setServoAngle(5, 120);
              PCA_setServoAngle(9, 120);
              PCA_setServoAngle(13, 120);

              PCA_setServoAngle(2, 50);
              PCA_setServoAngle(6, 50);
              PCA_setServoAngle(10, 50);
              PCA_setServoAngle(14, 50); //default pos

              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);

              PCA_setServoAngle(0, 110);
              PCA_setServoAngle(4, 110);
              PCA_setServoAngle(8, 70);
              PCA_setServoAngle(12, 50);

              PCA_setServoAngle(1, 130);
              PCA_setServoAngle(5, 120);
              PCA_setServoAngle(9, 120);
              PCA_setServoAngle(13, 130);

              PCA_setServoAngle(2, 25);
              PCA_setServoAngle(6, 50);
              PCA_setServoAngle(10, 50);
              PCA_setServoAngle(14, 25); //pos e

              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);
              PCA_setServoAngle(1, 160); //raise the leg
              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);

              PCA_setServoAngle(0, 50);
              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);
              PCA_setServoAngle(2, 90);
              
              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);

              PCA_setServoAngle(1, 100); //pos f

              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);

              PCA_setServoAngle(0, 50);
              PCA_setServoAngle(4, 50);
              PCA_setServoAngle(8, 70);
              PCA_setServoAngle(12, 100);

              PCA_setServoAngle(1, 120);
              PCA_setServoAngle(5, 120);
              PCA_setServoAngle(9, 100);
              PCA_setServoAngle(13, 130);

              PCA_setServoAngle(2, 50);
              PCA_setServoAngle(6, 50);
              PCA_setServoAngle(10, 90);
              PCA_setServoAngle(14, 25); //pos g

              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);
              PCA_setServoAngle(9, 130); //raise the leg
              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);

              PCA_setServoAngle(8, 120);
              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);
              PCA_setServoAngle(10, 50); 
              PCA_setServoAngle(9, 130);
              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);

              PCA_setServoAngle(9, 130);
              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);//pos h

              PCA_setServoAngle(5, 130); //raise the leg
              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);

              PCA_setServoAngle(4, 130);
              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);
              PCA_setServoAngle(6, 40);
              
              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);

              PCA_setServoAngle(5, 100); //pos f
              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);

              PCA_setServoAngle(0, 70);
              PCA_setServoAngle(4, 100);
              PCA_setServoAngle(8, 50);
              PCA_setServoAngle(12, 50);

              PCA_setServoAngle(1, 120);
              PCA_setServoAngle(5, 120);
              PCA_setServoAngle(9, 130);
              PCA_setServoAngle(13, 120);

              PCA_setServoAngle(2, 90);
              PCA_setServoAngle(6, 25);
              PCA_setServoAngle(10, 50);
              PCA_setServoAngle(14, 50); //pos j

              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);
              PCA_setServoAngle(13, 130); //raise the leg
              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);

              PCA_setServoAngle(12, 70);
              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);
              PCA_setServoAngle(14, 50); 
              PCA_setServoAngle(13, 130);
              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);

              PCA_setServoAngle(13, 70);
              HAL_Delay(delay);
              HAL_IWDG_Refresh(&hiwdg);//pos k
              
          }
      }
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
