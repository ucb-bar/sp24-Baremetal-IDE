/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include <string.h>
#include "hardware.h"

// 15M version
// #include "weights_15Mq.h"
// #include "weights_TS_small_q.h"
// #include "tokenizer_TS.h"

// 260K version
// #include "weights_260Kq.h"
// #include "tokenizer_512.h"

// newer tok32000 stories260 version
// #include "weights_15Mq.h"
// #include "weights_260Kq_32000.h"
// #include "tokenizer_32000.h"


// Bora Datasets
#include "bora_tok8096.h"
#include "bora_260K8096.h"
//#include "bora_3M8096.h"
//#include "bora_15M8096.h"
//#include "bora_42M8096.h"


/**
 * This section controls which peripheral device is included in the application program.
 * To save the memory space, the unused peripheral device can be commented out.
 */
// #include "hal_core.h"
// #include "hal_clint.h"
// #include "hal_gpio.h"
// #include "hal_i2c.h"
// #include "hal_plic.h"
// #include "hal_uart.h"

/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define MODEL_MAGIC_NUMBER 0x616b3432
#define MODEL_VERSION_INT8 2
#define MODEL_V2_HEADER_SIZE 256

/* USER CODE END Private defines */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
    GENERATE,
    CHAT
} GenMode;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
int main(int argc, char** argv);
void __main();
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
