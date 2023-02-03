/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "buttons.h"

pint ekeys[3] = {
    {GPIOC, GPIO_PIN_1},
    {GPIOC, GPIO_PIN_2},
    {GPIOC, GPIO_PIN_3}
};

pint keyout[KEYOUTS] = {
    {GPIOD, GPIO_PIN_13}, // 15
    {GPIOD, GPIO_PIN_14}, // 14
    {GPIOD, GPIO_PIN_15}, // 13
    {GPIOC, GPIO_PIN_6},  // 12

    {GPIOC, GPIO_PIN_7}, // 11
    {GPIOC, GPIO_PIN_8}, // 10
    {GPIOC, GPIO_PIN_9}, // 9
    {GPIOA, GPIO_PIN_8}, // 8

    {GPIOA, GPIO_PIN_11}, // 7
    {GPIOA, GPIO_PIN_12}, // 6
    {GPIOA, GPIO_PIN_15}, // 5
    {GPIOC, GPIO_PIN_10}, // 4

    {GPIOC, GPIO_PIN_11}, // 3
    {GPIOC, GPIO_PIN_12}, // 2
    {GPIOD, GPIO_PIN_0},  // 1
    {GPIOD, GPIO_PIN_1},  // 0
};

pint keyin[KEYINS] = {
    {GPIOD, GPIO_PIN_12},
    {GPIOD, GPIO_PIN_11},
    {GPIOD, GPIO_PIN_10},
    {GPIOD, GPIO_PIN_9},

    {GPIOD, GPIO_PIN_8},
    {GPIOB, GPIO_PIN_15},
    {GPIOE, GPIO_PIN_6},
    {GPIOC, GPIO_PIN_13},

    {GPIOC, GPIO_PIN_0},
};

uint16_t btnmatrix[KEYOUTS][2];  // Optimized
uint16_t ebtnmatrix[2];          // Optimized

uint8_t keycnt = 0;

uint16_t *matrix;
uint16_t *keys = keyin;
uint8_t keyd = 0;
uint8_t type = 0;

uint8_t getKeyCounter() {
    return keycnt;
}

void setKeyCounter(uint8_t cnt) {
    keycnt = cnt;
}

uint16_t getEKeysMatrix(uint8_t par, uint8_t key) {
    return bitRead(ebtnmatrix[par], key);
}

uint16_t getKeysMatrix(uint8_t key, uint8_t par, uint8_t key2) {
    return bitRead(btnmatrix[key][par], key2);
}

void setKeysMatrix(uint16_t key1, uint8_t par, uint16_t key2, uint8_t st) {
    bitWrite(btnmatrix[key1][par], key2, st);
}

void setEKeysMatrix(uint8_t par, uint16_t key, uint8_t st) {
    bitWrite(ebtnmatrix[par], key, st);
}

void keyboardStart() {
    for (int d = 0; d < KEYOUTS; d++)
    {
        writePin(keyout[d].port, keyout[d].pin, 1);
    }
}
void keyboardEnable() {
    writePin(keyout[keycnt].port, keyout[keycnt].pin, 0);
}

void keyboardDisable() {
    writePin(keyout[keycnt].port, keyout[keycnt].pin, 1);
}

void keyboardScan()
{
    //! Scanning

    usDelay(50);
    
    // Main keyboard
    keys = keyin;
    type = 0;
    for (uint8_t key = 0; key < KEYINS+3; key++)
    {
        keyd = key;
        
        if (key >= KEYINS) { 
            type = 1; 
            keys = ekeys;
            keyd -= 9; 
        }

        buttonScan(ekeys, type, keyd);
    }
    usDelay(100);
}

void buttonScan(pint *keys, uint8_t type, uint8_t key) {
    
    if (type == 0) { keys = keyin; matrix = btnmatrix[keycnt]; }
    else {  matrix = ebtnmatrix; }

    if (readPin(keys[key].port, keys[key].pin) == 0 && bitRead(matrix[0], key) == 0) {
        usDelay(300);
        if (readPin(keys[key].port, keys[key].pin) == 0) {
            bitWrite(matrix[0], key, 1);
            bitWrite(matrix[1], key, 1); //Change
        }
    } else if (readPin(keys[key].port, keys[key].pin) == 1 && bitRead(matrix[0], key) == 1) {
        usDelay(300);
        if (readPin(keys[key].port, keys[key].pin) == 1) {
            bitWrite(matrix[0], key, 0);
            bitWrite(matrix[1], key, 1); //Change
        }
    }
}