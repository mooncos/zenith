/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32wbxx_hal.h"

#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* Definitions of environment analog values */
/* Value of analog reference voltage (Vref+), connected to analog voltage   */
/* supply Vdda (unit: mV).                                                  */
#define VDDA_APPLI                       (3300UL)
#define USE_DOUBLE_BUFFER

/* Definition of ADCx conversions data table size */
#define ADC_CONVERTED_DATA_BUFFER_SIZE   (1UL*158U) // uses double buffer to not stop the DMA

/* Init variable out of expected ADC conversion data range */
#define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)
#define QUEUE_BLE_PACKETS_NO 22
#define SEND_BIN_NO 20
#define BIN_SENT_OFFSET 3
#define SEND_BIN_INITIAL_IDX 0
#define EXTREMA_SAMPLES_DISCARD_NO 16
#define TOTAL_SAMPLES_DISCARD_NO 48

#define MICRODOPPLER_MODE

#define MD_BIN_SUM_START 10
#define MD_BIN_SUM_NUMBER 60
#define MD_SUMS_PER_PACKET 30 /* max 30 ramps */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define OSCILLO_Pin GPIO_PIN_8
#define OSCILLO_GPIO_Port GPIOB
#define TRIG_Pin GPIO_PIN_9
#define TRIG_GPIO_Port GPIOB
#define TRIG_EXTI_IRQn EXTI9_5_IRQn
#define I_ADC_Pin GPIO_PIN_1
#define I_ADC_GPIO_Port GPIOA
#define Q_ADC_Pin GPIO_PIN_2
#define Q_ADC_GPIO_Port GPIOA
#define BTN1_Pin GPIO_PIN_6
#define BTN1_GPIO_Port GPIOA
#define BTN1_EXTI_IRQn EXTI9_5_IRQn
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
