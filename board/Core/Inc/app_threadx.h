/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.h
  * @author  MCD Application Team
  * @brief   ThreadX applicative header file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#ifndef __APP_THREADX_H__
#define __APP_THREADX_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "tx_api.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* Watch g_bmi088_debug in the debugger to trace ThreadX and IMU progress. */
typedef struct
{
  uint32_t kernel_define_count; /* tx_application_define was entered */
  uint32_t byte_pool_status;    /* TX_SUCCESS (0) or ThreadX error code */
  uint32_t app_init_status;     /* App_ThreadX_Init result */
  uint32_t thread_create_status;
  uint32_t thread_entry_count;
  uint32_t loop_count;
  uint32_t init_attempts;
  uint32_t init_successes;
  uint32_t sample_attempts;
  uint32_t sample_successes;
  uint32_t error_flags;        /* bit0 ACC ID, bit1 GYRO ID, bit2 init,
                                * bit3 ACC data, bit4 GYRO data */
  uint8_t acc_chip_id;         /* expected 0x1E */
  uint8_t gyro_chip_id;        /* expected 0x0F */
  uint8_t last_sample_ok;
} BMI088_DebugInfo;

extern volatile BMI088_DebugInfo g_bmi088_debug;

/* Live Watch: zero means false; heartbeat fields report the latest monitor window. */
typedef struct
{
  uint32_t thread_create_status;
  uint32_t thread_entries;
  uint32_t checks;
  uint8_t startup_ready;
  uint8_t can_ready;
  uint8_t imu_ready;
  uint8_t motor_ready;
  uint8_t imu_alive;
  uint8_t motor_alive;
  uint8_t imu_operation_ok;
  uint8_t motor_operation_ok;
  uint8_t motor_feedback_online;
  uint8_t fault;
} Alive_DebugInfo;

extern volatile Alive_DebugInfo g_alive_debug;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
UINT App_ThreadX_Init(VOID *memory_ptr);
void MX_ThreadX_Init(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * Sensor-only bring-up mode: initialize only the BMI088 acquisition thread
 * and its GPIO/SPI dependencies. Set to 0 to restore the full application.
 */
#define BMI088_ONLY_DEBUG 0U
/* Motor-only bring-up: CAN1/CAN2 and one ThreadX motor thread. */
#define M2006_ONLY_DEBUG 0U
/* Bring up BMI088 and M2006 in two separate ThreadX threads. */
#define IMU_M2006_DEBUG 1U
#if (BMI088_ONLY_DEBUG + M2006_ONLY_DEBUG + IMU_M2006_DEBUG) > 1U
#error "Choose only one debug mode"
#endif
#define APP_FULL_MODE (!(BMI088_ONLY_DEBUG || M2006_ONLY_DEBUG || IMU_M2006_DEBUG))
#define APP_ALIVE_MONITOR (APP_FULL_MODE || IMU_M2006_DEBUG)
/* USER CODE END PD */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /* __APP_THREADX_H__ */
