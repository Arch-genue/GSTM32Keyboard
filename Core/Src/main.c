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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEYOUTS 16
#define KEYINS 9
#define LEDS 10

#define REPEATCOUNT 15

// #define USE_ENCODER

pint ekeys[3] = {
    {GPIOC, GPIO_PIN_1},
    {GPIOC, GPIO_PIN_2},
    {GPIOC, GPIO_PIN_3}
};

pint ekeysleds[4] = {
    {GPIOA, GPIO_PIN_0},
    {GPIOA, GPIO_PIN_1},
    {GPIOA, GPIO_PIN_2},
    {GPIOA, GPIO_PIN_3},
};
pint keyout[KEYOUTS] = {
    {GPIOD, GPIO_PIN_13},  // 15
    {GPIOD, GPIO_PIN_14}, // 14
    {GPIOD, GPIO_PIN_15}, // 13
    {GPIOC, GPIO_PIN_6}, // 12
    
    {GPIOC, GPIO_PIN_7}, // 11
    {GPIOC, GPIO_PIN_8},  // 10
    {GPIOC, GPIO_PIN_9}, // 9
    {GPIOA, GPIO_PIN_8}, // 8
    
    {GPIOA, GPIO_PIN_11},  // 7
    {GPIOA, GPIO_PIN_12}, // 6
    {GPIOA, GPIO_PIN_15},  // 5
    {GPIOC, GPIO_PIN_10}, // 4
    
    {GPIOC, GPIO_PIN_11}, // 3
    {GPIOC, GPIO_PIN_12}, // 2
    {GPIOD, GPIO_PIN_0}, // 1
    {GPIOD, GPIO_PIN_1}, // 0
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t keycols = 0;
uint8_t ledsaddr[3] = {0, 0, 0};
uint8_t ledmatrix[KEYOUTS][LEDS];

// TODO Optimize
uint8_t flag[KEYOUTS][KEYINS];    //[KEYINS] = {};
uint8_t flagChg[KEYOUTS][KEYINS]; //[KEYINS] = {};

uint8_t flagEKeys[3] = {0, 0, 0};
uint8_t flagChgEKeys[3] = {0, 0, 0};
uint8_t rptcnt = 0;
uint32_t mkr = 0;

uint8_t txbuff[5];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int prevCounter = 0;
void usDelay(uint16_t useconds)
{
  __HAL_TIM_SET_COUNTER(&htim10, 0);
  while(__HAL_TIM_GET_COUNTER(&htim10) < useconds);
}

uint8_t Enc_Counter = 0;

/*--LED FUNCTIONS--*/
void ledView(uint8_t key, uint8_t led, uint8_t status)
{
    ledmatrix[key][led] = status;
}

void ledsAll(uint8_t st)
{
    for (uint8_t i = 0; i < KEYOUTS; i++) { for (uint8_t j = 0; j < LEDS; j++) { ledmatrix[i][j] = st; } }
}

void ledsView()
{
    for (int led = 0; led < LEDS; led++) { HAL_GPIO_WritePin(leds[led].port, leds[led].pin, ledmatrix[keycols][led]); }
    
    for (int l = 0; l < 4; l++ ) { HAL_GPIO_WritePin(ekeysleds[l].port, ekeysleds[l].pin, ledmatrix[7][l]); }
    usDelay(800);
    for (int led = 0; led < LEDS; led++) { HAL_GPIO_WritePin(leds[led].port, leds[led].pin, 1); }
    usDelay(200);
}

/*--UART FUNCTIONS--*/
void receiveData()
{
    if (HAL_UART_Receive_IT(&huart1, ledsaddr, 3) != HAL_BUSY)
    {
        if (ledsaddr[0] == 0xaa && ledsaddr[1] == 0xbb)
            (ledsaddr[2] == 0xff) ? ledsAll(0) : ledsAll(1);
        else if (ledsaddr[0] < KEYOUTS && ledsaddr[1] < LEDS)
            ledView(ledsaddr[0], ledsaddr[1], (ledsaddr[2] == 0xff) ? 0 : 1); // ledmatrix[ ledsaddr[0] ][ ledsaddr[1] ] = (ledsaddr[2] == 0xff) ? 0 : 1;
    }
}

void sendKeyboardData()
{
    int currCounter = __HAL_TIM_GET_COUNTER(&htim1);
    
    currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;
    if(currCounter != prevCounter) {
        char buff[16];
        snprintf(buff, sizeof(buff), "%06d", currCounter);

        txbuff[0] = currCounter;
        HAL_UART_Transmit_IT(&huart1, &txbuff, 2);

        prevCounter = currCounter;
    }
    
    for (uint8_t x = 0; x < KEYOUTS; x++)
    {
        for (uint8_t y = 0; y < KEYINS; y++)
        {
            if (flagChg[x][y] == 1)
            {
                txbuff[0] = x; //(y + x * KEYINS);
                txbuff[1] = y;
                txbuff[2] = (flag[x][y] == 1) ? 0xFF : 0x00; // status;

                flagChg[x][y] = 0;
                HAL_UART_Transmit_IT(&huart1, &txbuff, 3);
            }
        }
    }
    for (uint8_t u = 0; u < 3; u++)
    {
        if (flagChgEKeys[u] == 1)
        {

            txbuff[0] = 0xAA;
            txbuff[1] = (u == 0) ? 0xAB : (u == 1) ? 0xAC : 0xAD;
            txbuff[2] = (flagEKeys[u] == 1) ? 0xFF : 0x00;

            flagChgEKeys[u] = 0;
            HAL_UART_Transmit_IT(&huart1, &txbuff, 3);
        }
    }
}

/*--INPUT FUNCTIONS--*/
void scanKeyboard()
{
    usDelay(50);
    // Special buttons
    for (int u = 0; u < 3; u++)
    {
        if (HAL_GPIO_ReadPin(ekeys[u].port, ekeys[u].pin) == GPIO_PIN_RESET && flagEKeys[u] == 0)
        {
            flagEKeys[u] = 1;
            flagChgEKeys[u] = 1;
        } else if(HAL_GPIO_ReadPin(ekeys[u].port, ekeys[u].pin) == GPIO_PIN_SET && flagEKeys[u] == 1) { 
            flagEKeys[u] = 0; 
            flagChgEKeys[u] = 1; 
        }
    }
    // Main keyboard
    for (int b = 0; b < KEYINS; b++)
    {
        if (HAL_GPIO_ReadPin(keyin[b].port, keyin[b].pin) == GPIO_PIN_RESET && flag[keycols][b] == 0)
        {
            usDelay(300);
            if (HAL_GPIO_ReadPin(keyin[b].port, keyin[b].pin) == GPIO_PIN_RESET)
            {
                flag[keycols][b] = 1;
                flagChg[keycols][b] = 1;
            }
        }
        else if (HAL_GPIO_ReadPin(keyin[b].port, keyin[b].pin) == GPIO_PIN_SET && flag[keycols][b] == 1)
        {
            usDelay(300);
            if (HAL_GPIO_ReadPin(keyin[b].port, keyin[b].pin) == GPIO_PIN_SET)
            {
                flag[keycols][b] = 0;
                flagChg[keycols][b] = 1;
            }
        }
    }
    usDelay(100);
}

#ifdef USE_ENCODER
void scanEncoder()
{
    int currCounter = __HAL_TIM_GET_COUNTER(&htim2);
    currCounter = 32767 - ((currCounter - 1) & 0xFFFF) / 2;
    if (currCounter != prevCounter)
    {
        char buff[16];
        HAL_GPIO_TogglePin(keyout[0].port, keyout[0].pin);
        HAL_GPIO_TogglePin(keyout[1].port, keyout[1].pin);
        HAL_GPIO_TogglePin(keyout[2].port, keyout[2].pin);
        sprintf(buff, "%d", currCounter);
        HAL_UART_Transmit(&huart3, &buff, sizeof(buff), 100);
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
    for (int d = 0; d < KEYOUTS; d++) { HAL_GPIO_WritePin(keyout[d].port, keyout[d].pin, 1); }

    /* --DISABLE LEDS-- */
    for (int d = 0; d < LEDS; d++) { HAL_GPIO_WritePin(leds[d].port, leds[d].pin, 1); }

    /* --NULL FLAGS-- */
    for (int d = 0; d < KEYOUTS; d++)
    {
        for (int g = 0; g < KEYINS; g++)
        {
            flag[d][g] = 0;
            flagChg[d][g] = 0;
        }
    }

    ledsAll(1);

    uint8_t mode = 1;
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifdef USE_ENCODER
        scanEncoder();
#endif
        receiveData();

        sendKeyboardData();
        if (keycols == KEYOUTS) { keycols = 0; mode = !mode; }

        // ENABLE KEYBOARD
        HAL_GPIO_WritePin(keyout[keycols].port, keyout[keycols].pin, 0);
        
        if (mode) scanKeyboard();
        else ledsView();
 
        //DISABLE KEYBOARD
        HAL_GPIO_WritePin(keyout[keycols].port, keyout[keycols].pin, 1);
        if(!mode) usDelay(500);
        
        //HAL_Delay(200);
        //txbuff[0] = keycols;
        //HAL_UART_Transmit_IT(&huart1, &txbuff, 1);
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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 30;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 6;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 6;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 167;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC0 PC1 PC2
                           PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA6 PA7 PA8 PA11
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD13 PD14 PD15 PD0
                           PD1 PD2 PD3 PD4
                           PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9
                           PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
