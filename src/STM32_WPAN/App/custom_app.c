/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    App/custom_app.c
 * @author  MCD Application Team
 * @brief   Custom Example Application (Server)
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
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fifo_queue.h"
#include "app_ble.h"
#include "app_conf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct {
	/* SiliconRemoteSvc */
	uint8_t Siliconremoteramp0_Notification_Status;
	/* USER CODE BEGIN CUSTOM_APP_Context_t */

	/* USER CODE END CUSTOM_APP_Context_t */

	uint16_t ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

PLACE_IN_SECTION("BLE_APP_CONTEXT") static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

/* USER CODE BEGIN PV */
uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

uint8_t SecureReadData;
extern __IO struct Queue bleQueue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* SiliconRemoteSvc */
static void Custom_Siliconremoteramp0_Update_Char(void);
static void Custom_Siliconremoteramp0_Send_Notification(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(
		Custom_STM_App_Notification_evt_t *pNotification) {
	/* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

	/* USER CODE END CUSTOM_STM_App_Notification_1 */
	switch (pNotification->Custom_Evt_Opcode) {
	/* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */
	/* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

	/* SiliconRemoteSvc */
	case CUSTOM_STM_SILICONREMOTERAMP0_NOTIFY_ENABLED_EVT:
		/* USER CODE BEGIN CUSTOM_STM_SILICONREMOTERAMP0_NOTIFY_ENABLED_EVT */
		BLE_SVC_GAP_UpdateConnTo2M();
		BLE_SVC_L2CAP_MinInterval();
		Custom_App_Context.Siliconremoteramp0_Notification_Status = 1;
		UTIL_SEQ_SetTask(1 << CFG_TASK_DO_SEND, CFG_SCH_PRIO_0);
		/* USER CODE END CUSTOM_STM_SILICONREMOTERAMP0_NOTIFY_ENABLED_EVT */
		break;

	case CUSTOM_STM_SILICONREMOTERAMP0_NOTIFY_DISABLED_EVT:
		/* USER CODE BEGIN CUSTOM_STM_SILICONREMOTERAMP0_NOTIFY_DISABLED_EVT */
		Custom_App_Context.Siliconremoteramp0_Notification_Status = 0;
		/* USER CODE END CUSTOM_STM_SILICONREMOTERAMP0_NOTIFY_DISABLED_EVT */
		break;

	default:
		/* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

		/* USER CODE END CUSTOM_STM_App_Notification_default */
		break;
	}
	/* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

	/* USER CODE END CUSTOM_STM_App_Notification_2 */
	return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification) {
	/* USER CODE BEGIN CUSTOM_APP_Notification_1 */

	/* USER CODE END CUSTOM_APP_Notification_1 */

	switch (pNotification->Custom_Evt_Opcode) {
	/* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

	/* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
	case CUSTOM_CONN_HANDLE_EVT:
		/* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */
		/* USER CODE END CUSTOM_CONN_HANDLE_EVT */
		break;

	case CUSTOM_DISCON_HANDLE_EVT:
		/* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

		/* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
		break;

	default:
		/* USER CODE BEGIN CUSTOM_APP_Notification_default */

		/* USER CODE END CUSTOM_APP_Notification_default */
		break;
	}

	/* USER CODE BEGIN CUSTOM_APP_Notification_2 */

	/* USER CODE END CUSTOM_APP_Notification_2 */

	return;
}

void Custom_APP_Init(void) {
	/* USER CODE BEGIN CUSTOM_APP_Init */

	/* USER CODE END CUSTOM_APP_Init */
	return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* SiliconRemoteSvc */
void Custom_Siliconremoteramp0_Update_Char(void) /* Property Read */
{
	Custom_STM_App_Update_Char(CUSTOM_STM_SILICONREMOTERAMP0,
			(uint8_t*) UpdateCharData);
	/* USER CODE BEGIN Siliconremoteramp0_UC*/

	/* USER CODE END Siliconremoteramp0_UC*/
	return;
}

void Custom_Siliconremoteramp0_Send_Notification(void) /* Property Notification */
{
	if (Custom_App_Context.Siliconremoteramp0_Notification_Status) {
		Custom_STM_App_Update_Char(CUSTOM_STM_SILICONREMOTERAMP0,
				(uint8_t*) NotifyCharData);
		/* USER CODE BEGIN Siliconremoteramp0_NS*/

		/* USER CODE END Siliconremoteramp0_NS*/
	} else {
		APP_DBG_MSG(
				"-- CUSTOM APPLICATION : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
	}
	return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
//void doSend(void) {
//	tBleStatus status = BLE_STATUS_INVALID_PARAMS;
//	if (Custom_App_Context.Siliconremoteramp0_Notification_Status) {
//		status = Custom_STM_App_Update_Char(CUSTOM_STM_SILICONREMOTERAMP0,
//				front(&bleQueue));
//		if (status == BLE_STATUS_SUCCESS) {
//			move_front(&bleQueue);
//		} else {
//			Custom_App_Context.Siliconremoteramp0_Notification_Status = 0;
//		}
//	}
//	UTIL_SEQ_SetTask(1 << CFG_TASK_DO_SEND, CFG_SCH_PRIO_0);
//}
void doSend(void) {
	tBleStatus status = BLE_STATUS_INVALID_PARAMS;
	if (!isEmpty(&bleQueue))
		status = Custom_STM_App_Update_Char(CUSTOM_STM_SILICONREMOTERAMP0,
				front(&bleQueue));
	if (status == BLE_STATUS_SUCCESS)
		move_front(&bleQueue);

	UTIL_SEQ_SetTask(1 << CFG_TASK_DO_SEND, CFG_SCH_PRIO_0);
}

void doResumeNotification(void) {
	Custom_App_Context.Siliconremoteramp0_Notification_Status = 1;
	UTIL_SEQ_SetTask(1 << CFG_TASK_DO_SEND, CFG_SCH_PRIO_0);
}
/* USER CODE END FD_LOCAL_FUNCTIONS*/
