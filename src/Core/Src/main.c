/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "ipcc.h"
#include "rf.h"
#include "rng.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "arm_const_structs.h"
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include "fifo_queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Variables for ADC conversion data */
__IO uint16_t uhADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE] = { 0 }; /* ADC group regular conversion data (array of data) */

/* Variables for ADC conversion data computation to physical values */
__IO int16_t uhADCxConvertedData_q15[512] = { 0 }; /* Value of converted data in mV */

/* Variable to report status of DMA transfer of ADC group regular conversions */
/*  0: DMA transfer is not completed                                          */
/*  1: DMA transfer is completed                                              */
/*  2: DMA transfer has not yet been started yet (initial state)              */
__IO uint8_t ignoreTrig = 0x0;
#ifdef MICRODOPPLER_MODE
__IO uint16_t sumNo = 0x0;
#else
__IO uint16_t rampNo = 0x0;
#endif
__IO uint8_t packetStorageArray[QUEUE_BLE_PACKETS_NO * 247] = { 0 }; // this holds 16 ble packets of 247 bytes == 16 ramps, change this below
__IO struct Queue bleQueue;
__IO uint8_t *packetArr[QUEUE_BLE_PACKETS_NO];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void DMA_Callback(DMA_HandleTypeDef *hdma);
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
	initAllocatedQueueWithArrayCapacity(&bleQueue, (uint8_t**) &packetArr,
			QUEUE_BLE_PACKETS_NO);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* IPCC initialisation */
   MX_IPCC_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_RF_Init();
  MX_RNG_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	// FOR DEBUG
	//DWT->CTRL = DWT_CTRL_CYCEVTENA_Msk | DWT_CTRL_CYCCNTENA_Msk;
	//DWT->CYCCNT = 0;

	/* Perform ADC calibration */
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
		/* Calibration Error */
		Error_Handler();
	}

	/* Start time base */
	if (HAL_TIM_Base_Start(&htim2) != HAL_OK) {
		/* Starting Error */
		Error_Handler();
	}

	HAL_DMA_RegisterCallback(&hdma_memtomem_dma1_channel2,HAL_DMA_XFER_CPLT_CB_ID, DMA_Callback);

  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  MX_APPE_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */
    MX_APPE_Process();

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI1
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  DMA transfer complete callback
 * @note   This function is executed when the transfer complete interrupt
 *         is generated
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	// Stop ADC and DMA
	LL_ADC_Disable(ADC1);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	// Convert DMA buffer to mV buffer
	//uint32_t c1 = DWT->CYCCNT;
	for (uint16_t i = 0;
			i < ADC_CONVERTED_DATA_BUFFER_SIZE - TOTAL_SAMPLES_DISCARD_NO;
			i++) {
		//uhADCxConvertedData_mV[conv_idx] = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, uhADCxConvertedData[conv_idx], LL_ADC_RESOLUTION_12B);
		uhADCxConvertedData_q15[i] = ((int32_t) (uhADCxConvertedData[i
				+ EXTREMA_SAMPLES_DISCARD_NO]) * 65535U + 2047U) / 4095U
				- 32768U;
	}

	/// OLD ALGO
//	arm_offset_f32(uhADCxConvertedData_mV + EXTREMA_CUT, -(4096>>1),
//			uhADCxConvertedData_mV, ADC_CONVERTED_DATA_BUFFER_SIZE - TOTAL_CUT);
//	arm_scale_f32(uhADCxConvertedData_mV, 0.00048828125,
//			uhADCxConvertedData_mV, ADC_CONVERTED_DATA_BUFFER_SIZE - TOTAL_CUT);
//	arm_float_to_q15(uhADCxConvertedData_mV, qADC1Data,
//			ADC_CONVERTED_DATA_BUFFER_SIZE - TOTAL_CUT);

	arm_cfft_q15(&arm_cfft_sR_q15_len256, (q15_t*) uhADCxConvertedData_q15, 0,
			1); // bit reversal is needed

#ifdef MICRODOPPLER_MODE
	uint16_t samp_cnt;

	int32_t cmplx_result[2];

	// loop unrolling: MD_BIN_SUM_NUMBER must be divisible by 4
	samp_cnt = MD_BIN_SUM_NUMBER >> 2u;

	q15_t * sample_ptr = uhADCxConvertedData_q15 + (MD_BIN_SUM_START << 1u);

	while (samp_cnt > 0u) {
		cmplx_result[0] += *sample_ptr++;
		cmplx_result[1] -= *sample_ptr++;
		cmplx_result[0] += *sample_ptr++;
		cmplx_result[1] -= *sample_ptr++;
		cmplx_result[0] += *sample_ptr++;
		cmplx_result[1] -= *sample_ptr++;
		cmplx_result[0] += *sample_ptr++;
		cmplx_result[1] -= *sample_ptr++;
		samp_cnt--;
	}

	// sum results S.log2N.15 format
	HAL_DMA_Start_IT(&hdma_memtomem_dma1_channel2, (uint32_t) &cmplx_result, (uint32_t) &packetStorageArray + (sumNo << 3u) % 240 + (247 * (sumNo << 3u)/240) % QUEUE_BLE_PACKETS_NO, 8);
#else
	*(uhADCxConvertedData_q15 + ((SEND_BIN_INITIAL_IDX + SEND_BIN_NO) << 1)) =
			rampNo; // store ramp number

	//*(uhADCxConvertedData_q15 + 121) = 0xADDE;
	//*(uhADCxConvertedData_q15 + 122) = 0xEFBE;
	//uint32_t c2 = DWT->CYCCNT;
	HAL_DMA_Start_IT(&hdma_memtomem_dma1_channel2,
			(uint32_t) &uhADCxConvertedData_q15 + (SEND_BIN_INITIAL_IDX << 1),
			(uint32_t) &packetStorageArray + (((rampNo / BIN_SENT_OFFSET) % QUEUE_BLE_PACKETS_NO)*247) + (SEND_BIN_NO << 2) * (rampNo % BIN_SENT_OFFSET),
			(SEND_BIN_NO << 2) + 2);
//	char msg[20];
//	//HAL_UART_Transmit(&huart1, (uint8_t*) uhADCxConvertedData_mV + 20, 128*32/8, 5);
//	HAL_UART_Transmit(&huart1, (uint8_t*) msg, sprintf(msg, "%ld\n", c2-c1), 10);
#endif
}

/**
 * @brief ADC Error callback
 * @note This function is called when an error with the ADC sampling and/or conversion has occurred
 * @retval None
 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
	/* Note: Disable ADC interruption that caused this error before entering in
	 infinite loop below. */

	/* In case of error due to overrun: Disable ADC group regular overrun interruption */
	LL_ADC_DisableIT_OVR(ADC1);

	/* Error reporting */
	Error_Handler();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if ((GPIO_Pin == BTN1_Pin) || (GPIO_Pin == TRIG_Pin)) {
		if (!ignoreTrig) {
			LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*) uhADCxConvertedData,
			ADC_CONVERTED_DATA_BUFFER_SIZE - (TOTAL_SAMPLES_DISCARD_NO- EXTREMA_SAMPLES_DISCARD_NO));
		}
		//ignoreTrig ^= 1;
	}
}

void DMA_Callback(DMA_HandleTypeDef *hdma) {
#ifdef MICRODOPPLER_MODE
	if(sumNo % 240 == 0) {
		enq(&bleQueue, (uint8_t*) &packetStorageArray + (247 * (sumNo << 3u)/240) % QUEUE_BLE_PACKETS_NO);
	}
	sumNo++;
#else
	if ((rampNo+1) % BIN_SENT_OFFSET == 0) {
		enq(&bleQueue, (uint8_t*) &packetStorageArray + ((rampNo / BIN_SENT_OFFSET) % QUEUE_BLE_PACKETS_NO)*247);
	}
	rampNo++;
#endif
}

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
	while (1) {
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
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
