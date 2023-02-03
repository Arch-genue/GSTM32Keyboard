/**
  ******************************************************************************
  * @file    buttons.h
  * @brief   This file contains scan buttons functions
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUTTONS_H__
#define __BUTTONS_H__

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
uint8_t getKeyCounter();
uint16_t getEKeysMatrix(uint8_t par, uint8_t key);
uint16_t getKeysMatrix(uint8_t key, uint8_t par, uint8_t key2);

void setKeyCounter(uint8_t cnt);
void setKeysMatrix(uint16_t key1, uint8_t par, uint16_t key2, uint8_t st);
void setEKeysMatrix(uint8_t par, uint16_t key, uint8_t st);

void keyboardStart();
void keyboardEnable();
void keyboardDisable();
void keyboardScan();

uint8_t buttonPressed(pint *keys, uint8_t type, uint8_t b);
uint8_t buttonReleased(pint *keys, uint8_t type, uint8_t b);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__BUTTONS_H__ */