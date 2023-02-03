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
#include "buttons.h"
#include "leds.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void usDelay(uint16_t usec) {
    __HAL_TIM_SET_COUNTER(&htim10, 0); 
    while (__HAL_TIM_GET_COUNTER(&htim10) < usec);
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t ENABLED = 0;

//Buffers
uint8_t txbuff[5];
uint8_t rxbuff[5];

//Tmp
int32_t prevCounter = 0;
uint8_t mode = 1;
uint8_t keycols = 0;
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

/*--UART FUNCTIONS--*/
void uart_rx()
{
    if (HAL_UART_Receive_IT(&huart1, rxbuff, 3) != HAL_BUSY)
    {
        if (rxbuff[0] == 0xdd && rxbuff[1] == 0xee){
            ENABLED = (rxbuff[2] == 0xff) ? 1 : 0;
        } else if (ENABLED) {
            if (rxbuff[0] == 0xaa && rxbuff[1] == 0xbb)
                (rxbuff[2] == 0xff) ? ledsAll(1) : ledsAll(0);
            else if (rxbuff[0] < KEYOUTS && rxbuff[1] < LEDS)
                ledView(rxbuff[0], rxbuff[1], (rxbuff[2] == 0xff) ? 1 : 0);
        }
    }
}

void uart_tx()
{

    //Main buttons
    for (uint8_t key = 0; key < KEYINS + 3; key++) {
        if (getKeysMatrix(keycols, 1, key) == 1 || ( key >= KEYINS && getEKeysMatrix(1, key-9) == 1 )) {
            if (key < KEYINS) {
                txbuff[0] = keycols;
                txbuff[1] = key;
                txbuff[2] = getKeysMatrix(keycols, 0, key) == 1 ? 0xFF : 0x00;

                setKeysMatrix(keycols, 1, key, 0);
            } else {
                txbuff[0] = 0xAA;
                txbuff[1] = (key-9 == 0) ? 0xAB : (key-9 == 1) ? 0xAC : 0xAD;
                txbuff[2] = getEKeysMatrix(0, key-9) == 1 ? 0xFF : 0x00;

                setEKeysMatrix(1, key-9, 0);
            }
            HAL_UART_Transmit_IT(&huart1, txbuff, 3);
        }
    }

    //Encoder
#ifdef USE_ENCODER
    scanEncoder();
#endif
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

    keyboardStart();

    ledsAll(0);
    ledsView();
    
    writePin(GPIOA, GPIO_PIN_6, 1);
    writePin(GPIOA, GPIO_PIN_7, 1);

    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        uart_rx();
        if (!ENABLED) {
            ledsAll(0);
            ledsView();
            keyboardStart();
            keycols = KEYOUTS;
            mode = 0;
            continue;
        }

        if (keycols == KEYOUTS)
        {
            keycols = 0;
            setKeyCounter(keycols);
            mode = !mode;
        }

        setKeyCounter(keycols);

        keyboardEnable(); //! ENABLE KEYBOARD
        
        if (mode)
            keyboardScan();
        else {
            ledsView();
            uart_tx();
        }

        keyboardDisable(); //! DISABLE KEYBOARD
        
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
