/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "leds.h"

pint leds[LEDS] = {
    {GPIOD, GPIO_PIN_2}, // 0
    {GPIOD, GPIO_PIN_3}, // 1
    {GPIOD, GPIO_PIN_4}, // 2
    {GPIOD, GPIO_PIN_5}, // 3

    {GPIOD, GPIO_PIN_6}, // 4
    {GPIOD, GPIO_PIN_7}, // 5
    {GPIOB, GPIO_PIN_3}, // 6
    {GPIOB, GPIO_PIN_5}, // 7

    {GPIOB, GPIO_PIN_6}, // 8
    {GPIOB, GPIO_PIN_7}  // 9
};

pint ekeysleds[4] = {
    {GPIOA, GPIO_PIN_0},
    {GPIOA, GPIO_PIN_1},
    {GPIOA, GPIO_PIN_2},
    {GPIOA, GPIO_PIN_3},
};

const uint8_t banledscols[7] = {0x07,0x08,0x09,0x0a,0x0b,0x0d,0x0e}; // *Disabled Cols for leds

uint8_t ledmatrix[LEDS_KEYOUTS]; // Optimized

/*--LED FUNCTIONS--*/
void ledView(uint8_t key, uint8_t led, uint8_t st) {
    ledmatrix[key] = bitWrite(ledmatrix[key], led, st);
}

void ledsAll(uint8_t st)
{
    st = st ? 1 : 0;
    for (uint8_t i = 0; i < KEYOUTS; i++) {
        for (uint8_t j = 0; j < LEDS; j++) {
            ledmatrix[i] = bitWrite(ledmatrix[i], j, st);
        }
    }
}

void ledsView()
{
    if ( find(&banledscols, 7, getKeyCounter()) ) return;

    for (uint8_t led = 0; led < LEDS+4; led++) {
        uint8_t ldst = (led == 8) ? 4 : ( (led == 9) ? 5 : led );
        uint8_t keycol = (led == 8 || led == 9) ? 7 : getKeyCounter();

        if (led < LEDS) writePin(leds[led].port, leds[led].pin, !bitRead(ledmatrix[keycol], ldst));
        else writePin(ekeysleds[led-10].port, ekeysleds[led-10].pin, !bitRead(ledmatrix[7], led-10));
    }
    
    usDelay(600);

    for (int led = 0; led < LEDS; led++) {
        writePin(leds[led].port, leds[led].pin, 1);
    }

    usDelay(200);
}