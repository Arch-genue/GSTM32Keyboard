/**
  ******************************************************************************
  * @file    leds.h
  * @brief   This file contains leds functions
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LEDS_H__
#define __LEDS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */
void ledView(uint8_t key, uint8_t led, uint8_t st);
void ledsAll(uint8_t st);
void ledsView();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__LEDS_H__ */