/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

HAL_StatusTypeDef debug_serialWrite(char* message, bool nl);
void led_set(int led, bool turn_on);

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DRDY_Pin GPIO_PIN_2
#define DRDY_GPIO_Port GPIOE
#define SERVO12_A_Pin GPIO_PIN_3
#define SERVO12_A_GPIO_Port GPIOE
#define MEMS_INT3_Pin GPIO_PIN_4
#define MEMS_INT3_GPIO_Port GPIOE
#define MEMS_INT4_Pin GPIO_PIN_5
#define MEMS_INT4_GPIO_Port GPIOE
#define SERVO12_B_Pin GPIO_PIN_6
#define SERVO12_B_GPIO_Port GPIOE
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define SERVO4_A_Pin GPIO_PIN_10
#define SERVO4_A_GPIO_Port GPIOF
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOF
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOF
#define SERVO4_B_Pin GPIO_PIN_0
#define SERVO4_B_GPIO_Port GPIOC
#define LED_A_Pin GPIO_PIN_1
#define LED_A_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_2
#define LED_B_GPIO_Port GPIOC
#define LED_C_Pin GPIO_PIN_3
#define LED_C_GPIO_Port GPIOC
#define LED_D_Pin GPIO_PIN_2
#define LED_D_GPIO_Port GPIOF
#define BTN_Pin GPIO_PIN_0
#define BTN_GPIO_Port GPIOA
#define SERVO10_PWM_Pin GPIO_PIN_1
#define SERVO10_PWM_GPIO_Port GPIOA
#define SERVO1_A_Pin GPIO_PIN_4
#define SERVO1_A_GPIO_Port GPIOF
#define SERVO2_PWM_Pin GPIO_PIN_4
#define SERVO2_PWM_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MISOA7_Pin GPIO_PIN_7
#define SPI1_MISOA7_GPIO_Port GPIOA
#define SERVO9_ADC_Pin GPIO_PIN_0
#define SERVO9_ADC_GPIO_Port GPIOB
#define SERVO1_ADC_Pin GPIO_PIN_1
#define SERVO1_ADC_GPIO_Port GPIOB
#define SERVO1_B_Pin GPIO_PIN_2
#define SERVO1_B_GPIO_Port GPIOB
#define SERVO10_ADC_Pin GPIO_PIN_7
#define SERVO10_ADC_GPIO_Port GPIOE
#define SERVO5_ADC_Pin GPIO_PIN_8
#define SERVO5_ADC_GPIO_Port GPIOE
#define SERVO2_ADC_Pin GPIO_PIN_9
#define SERVO2_ADC_GPIO_Port GPIOE
#define SERVO11_ADC_Pin GPIO_PIN_10
#define SERVO11_ADC_GPIO_Port GPIOE
#define SERVO12_ADC_Pin GPIO_PIN_11
#define SERVO12_ADC_GPIO_Port GPIOE
#define SERVO2_A_Pin GPIO_PIN_12
#define SERVO2_A_GPIO_Port GPIOE
#define SERVO3_ADC_Pin GPIO_PIN_13
#define SERVO3_ADC_GPIO_Port GPIOE
#define SERVO2_B_Pin GPIO_PIN_14
#define SERVO2_B_GPIO_Port GPIOE
#define SERVO3_A_Pin GPIO_PIN_15
#define SERVO3_A_GPIO_Port GPIOE
#define SERVO11_PWM_Pin GPIO_PIN_10
#define SERVO11_PWM_GPIO_Port GPIOB
#define SERVO12_PWM_Pin GPIO_PIN_11
#define SERVO12_PWM_GPIO_Port GPIOB
#define SERVO3_B_Pin GPIO_PIN_12
#define SERVO3_B_GPIO_Port GPIOB
#define SERVO4_ADC_Pin GPIO_PIN_13
#define SERVO4_ADC_GPIO_Port GPIOB
#define SERVO5_A_Pin GPIO_PIN_8
#define SERVO5_A_GPIO_Port GPIOD
#define SERVO5_B_Pin GPIO_PIN_9
#define SERVO5_B_GPIO_Port GPIOD
#define SERVO6_ADC_Pin GPIO_PIN_10
#define SERVO6_ADC_GPIO_Port GPIOD
#define SERVO7_ADC_Pin GPIO_PIN_11
#define SERVO7_ADC_GPIO_Port GPIOD
#define SERVO5_PWM_Pin GPIO_PIN_12
#define SERVO5_PWM_GPIO_Port GPIOD
#define SERVO6_PWM_Pin GPIO_PIN_13
#define SERVO6_PWM_GPIO_Port GPIOD
#define SERVO8_ADC_Pin GPIO_PIN_14
#define SERVO8_ADC_GPIO_Port GPIOD
#define SERVO8_PWM_Pin GPIO_PIN_15
#define SERVO8_PWM_GPIO_Port GPIOD
#define SERVO1_PWM_Pin GPIO_PIN_6
#define SERVO1_PWM_GPIO_Port GPIOC
#define SERVO6_A_Pin GPIO_PIN_7
#define SERVO6_A_GPIO_Port GPIOC
#define SERVO3_PWM_Pin GPIO_PIN_8
#define SERVO3_PWM_GPIO_Port GPIOC
#define SERVO4_PWM_Pin GPIO_PIN_9
#define SERVO4_PWM_GPIO_Port GPIOC
#define SERVO6_B_Pin GPIO_PIN_8
#define SERVO6_B_GPIO_Port GPIOA
#define SERVO7_A_Pin GPIO_PIN_9
#define SERVO7_A_GPIO_Port GPIOA
#define SERVO7_B_Pin GPIO_PIN_10
#define SERVO7_B_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SERVO8_A_Pin GPIO_PIN_6
#define SERVO8_A_GPIO_Port GPIOF
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SERVO9_PWM_Pin GPIO_PIN_15
#define SERVO9_PWM_GPIO_Port GPIOA
#define SERVO8_B_Pin GPIO_PIN_10
#define SERVO8_B_GPIO_Port GPIOC
#define SERVO9_A_Pin GPIO_PIN_11
#define SERVO9_A_GPIO_Port GPIOC
#define SERVO9_B_Pin GPIO_PIN_12
#define SERVO9_B_GPIO_Port GPIOC
#define SERVO10_A_Pin GPIO_PIN_0
#define SERVO10_A_GPIO_Port GPIOD
#define HX0_DATA_Pin GPIO_PIN_1
#define HX0_DATA_GPIO_Port GPIOD
#define HX0_CLOCK_Pin GPIO_PIN_2
#define HX0_CLOCK_GPIO_Port GPIOD
#define HX1_DATA_Pin GPIO_PIN_3
#define HX1_DATA_GPIO_Port GPIOD
#define HX1_CLOCK_Pin GPIO_PIN_4
#define HX1_CLOCK_GPIO_Port GPIOD
#define HX2_DATA_Pin GPIO_PIN_5
#define HX2_DATA_GPIO_Port GPIOD
#define HX2_CLOCK_Pin GPIO_PIN_6
#define HX2_CLOCK_GPIO_Port GPIOD
#define HX3_DATA_Pin GPIO_PIN_7
#define HX3_DATA_GPIO_Port GPIOD
#define HX3_CLOCK_Pin GPIO_PIN_3
#define HX3_CLOCK_GPIO_Port GPIOB
#define SERVO10_B_Pin GPIO_PIN_4
#define SERVO10_B_GPIO_Port GPIOB
#define SERVO11_A_Pin GPIO_PIN_5
#define SERVO11_A_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define SERVO7_PWM_Pin GPIO_PIN_8
#define SERVO7_PWM_GPIO_Port GPIOB
#define SERVO11_B_Pin GPIO_PIN_9
#define SERVO11_B_GPIO_Port GPIOB
#define MEMS_INT1_Pin GPIO_PIN_0
#define MEMS_INT1_GPIO_Port GPIOE
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
