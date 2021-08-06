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
  {140, 410},
  {100, 480},
  {80, 470},
  {0, 0},
  {150, 440},
  {100, 480},
  {80, 470},
  {0, 0},
  {150, 450},
  {110, 480},
  {160, 510}, //fix
  {0, 0},
  {170, 470},
  {100, 470},
  {150, 520},
  {0, 0}
};

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

    char b[10];
    itoa(value, b, 16);
    char msg[32] = "VALUE: ";
    strcat(msg, b);
    debug_serialWrite(msg, CRLF);

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
}

void PCA_begin()
{
    PCA_reset();
    PCA_setFreq(50);
    PCA_autoIncrement(true);
}

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
HAL_StatusTypeDef debug_serialWrite(char* message, bool nl)
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
  return HAL_UART_Transmit(&huart1, (uint8_t*)&msg, strlen(msg), HAL_MAX_DELAY);
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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      led_toggle(6);
      HAL_Delay(100);
      HAL_IWDG_Refresh(&hiwdg);
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
