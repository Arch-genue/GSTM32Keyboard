/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

pint ekeys[3] = {
    {GPIOC, GPIO_PIN_1},
    {GPIOC, GPIO_PIN_2},
    {GPIOC, GPIO_PIN_3}};

pint ekeysleds[4] = {
    {GPIOA, GPIO_PIN_0},
    {GPIOA, GPIO_PIN_1},
    {GPIOA, GPIO_PIN_2},
    {GPIOA, GPIO_PIN_3},
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint8_t banledscols[7] = {0x07,0x08,0x09,0x0a,0x0b,0x0d,0x0e};

//Flags
uint8_t ledmatrix[LEDS_KEYOUTS]; // Optimized
uint16_t btnmatrix[KEYOUTS][2];  // Optimized
uint8_t ebtnmatrix[2];           // Optimized

//Buffers
uint8_t txbuff[5];
uint8_t rxbuff[5];

//tmp
int32_t prevCounter = 0;
uint8_t keycols = 0;
uint8_t mode = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t find(uint8_t *array, const uint8_t count, const uint8_t value) {
    for (int i = 0; i < count; i++) {
        if ( array[i] == value ) return 1;
    }
    return 0;
}

/*--LED FUNCTIONS--*/
void ledView(uint8_t key, uint8_t led, uint8_t st)
{
    ledmatrix[key] = bitWrite(ledmatrix[key], led, st);
}

void ledsAll(uint8_t st)
{
    st = st ? 1 : 0;
    for (uint8_t i = 0; i < KEYOUTS; i++)
    {
        for (uint8_t j = 0; j < LEDS; j++)
        {
            ledmatrix[i] = bitWrite(ledmatrix[i], j, st);
        }
    }
}

/*--UART FUNCTIONS--*/
void uart_rx()
{
    if (HAL_UART_Receive_IT(&huart1, rxbuff, 3) != HAL_BUSY)
    {
        if (rxbuff[0] == 0xaa && rxbuff[1] == 0xbb)
            (rxbuff[2] == 0xff) ? ledsAll(1) : ledsAll(0);
        else if (rxbuff[0] < KEYOUTS && rxbuff[1] < LEDS)
            ledView(rxbuff[0], rxbuff[1], (rxbuff[2] == 0xff) ? 1 : 0);
    }
}

void uart_tx()
{
    //Main buttons
    for (uint8_t y = 0; y < KEYINS; y++)
    {
        if (bitRead(btnmatrix[keycols][1], y) == 1)
        {
            txbuff[0] = keycols;
            txbuff[1] = y;
            txbuff[2] = (bitRead(btnmatrix[keycols][0], y) == 1) ? 0xFF : 0x00;

            bitWrite(btnmatrix[keycols][1], y, 0);
            HAL_UART_Transmit_IT(&huart1, &txbuff, 3);
        }
    }

    //Special buttons
    for (uint8_t u = 0; u < 3; u++)
    {
        if (bitRead(ebtnmatrix[1], u) == 1)
        {
            txbuff[0] = 0xAA;
            txbuff[1] = (u == 0) ? 0xAB : (u == 1) ? 0xAC : 0xAD;
            txbuff[2] = (bitRead(ebtnmatrix[0], u) == 1) ? 0xFF : 0x00;

            bitWrite(ebtnmatrix[1], u, 0);
            HAL_UART_Transmit_IT(&huart1, &txbuff, 3);
        }
    }

    //Encoder
#ifdef USE_ENCODER
    scanEncoder();
#endif
}

void scanKeyboard()
{
    usDelay(50);

    // Special buttons
    for (uint8_t u = 0; u < 3; u++)
    {
        if (u == 2) {
            if (readPin(ekeys[2].port, ekeys[2].pin) == GPIO_PIN_RESET && bitRead(ebtnmatrix[0], 2) == 1) {
                bitWrite(ebtnmatrix[0], 2, 0);
                bitWrite(ebtnmatrix[1], 2, 1);
            } else if (readPin(ekeys[2].port, ekeys[2].pin) == GPIO_PIN_SET && bitRead(ebtnmatrix[0], 2) == 0) {
                bitWrite(ebtnmatrix[0], 2, 1);
                bitWrite(ebtnmatrix[1], 2, 1);
            }
            break;
        }
        if (readPin(ekeys[u].port, ekeys[u].pin) == GPIO_PIN_RESET && bitRead(ebtnmatrix[0], u) == 0)
        {
            bitWrite(ebtnmatrix[0], u, 1);
            bitWrite(ebtnmatrix[1], u, 1);
        }
        else if (readPin(ekeys[u].port, ekeys[u].pin) == GPIO_PIN_SET && bitRead(ebtnmatrix[0], u) == 1)
        {
            bitWrite(ebtnmatrix[0], u, 0);
            bitWrite(ebtnmatrix[1], u, 1);
        }
    }
    // Main keyboard
    for (uint8_t b = 0; b < KEYINS; b++)
    {
        if (readPin(keyin[b].port, keyin[b].pin) == 0 && bitRead(btnmatrix[keycols][0], b) == 0)
        {
            usDelay(300);
            if (readPin(keyin[b].port, keyin[b].pin) == 0)
            {
                bitWrite(btnmatrix[keycols][0], b, 1);
                bitWrite(btnmatrix[keycols][1], b, 1);
            }
        }
        else if (readPin(keyin[b].port, keyin[b].pin) == 1 && bitRead(btnmatrix[keycols][0], b) == 1)
        {
            usDelay(300);
            if (readPin(keyin[b].port, keyin[b].pin) == 1)
            {
                bitWrite(btnmatrix[keycols][0], b, 0);
                bitWrite(btnmatrix[keycols][1], b, 1);
            }
        }
    }
    usDelay(100);
}

void ledsView()
{
    if ( find(&banledscols, 7, keycols) ) return;

    for (uint8_t led = 0; led < LEDS; led++)
    {
        uint8_t ldst = (led == 8) ? 4 : ( (led == 9) ? 5 : led );
        uint8_t keycol = (led == 8 || led == 9) ? 7 : keycols;

        writePin(leds[led].port, leds[led].pin, !bitRead(ledmatrix[keycol], ldst));
    }

    for (int l = 0; l < 4; l++)
    {
        writePin(ekeysleds[l].port, ekeysleds[l].pin, !bitRead(ledmatrix[7], l));
    }
    usDelay(600);
    for (int led = 0; led < LEDS; led++)
    {
        writePin(leds[led].port, leds[led].pin, 1);
    }
    usDelay(200);
}

#ifdef USE_ENCODER
void scanEncoder()
{
    int currCounter = __HAL_TIM_GET_COUNTER(&htim1);
    currCounter = 32767 - ((currCounter - 1) & 0xFFFF) / 2;
    if (currCounter != prevCounter)
    {
        txbuff[0] = 0xCC;
        txbuff[1] = 0x25;
        txbuff[2] = 0x00;
        if (currCounter > prevCounter)
            txbuff[2] = 0xFF;

        HAL_UART_Transmit_IT(&huart1, &txbuff, 3);

        prevCounter = currCounter;
    }
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  MX_TIM1_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start(&htim10);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    /* --DISABLE KEYOUTS-- */
    for (int d = 0; d < KEYOUTS; d++)
    {
        writePin(keyout[d].port, keyout[d].pin, 1);
    }

    /* --DISABLE LEDS-- */
    for (int d = 0; d < LEDS; d++)
    {
        writePin(leds[d].port, leds[d].pin, 1);
    }

    ledsAll(0);
    writePin(GPIOA, GPIO_PIN_6, 1);
    writePin(GPIOA, GPIO_PIN_7, 1);

    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        if (keycols == KEYOUTS)
        {
            keycols = 0;
            mode = !mode;
        }

        uart_rx();
        
        // ENABLE KEYBOARD
        writePin(keyout[keycols].port, keyout[keycols].pin, 0);

        if (mode)
            scanKeyboard();
        else {
            ledsView();
            uart_tx();
        }

        // DISABLE KEYBOARD
        writePin(keyout[keycols].port, keyout[keycols].pin, 1);
        if (!mode) usDelay(500);

        keycols++;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
    while (1)
    {
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
