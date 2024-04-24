/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "ssd1306.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
	
	int red = 6;
	int green = 9;
	int blue = 7;
	int orange = 8;
	int gateStatus;
	int parkingStatus;
	int ticks;
	int timer;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_Init();

	SystemClock_Config();

	//Setup Interupt on specific pin to interupt when gate is detected
	
	//enable the RCC for specific GPIO 
	RCC->APB2ENR|= RCC_APB2ENR_SYSCFGCOMPEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	RCC->AHBENR   |= RCC_AHBENR_GPIOCEN;

	//unmask exti4 
	EXTI->IMR  |= (1 << 4);
	
	//Figure out what the trigger is rising or fall edge 
	EXTI->FTSR |= (1 << 4); 
	
	//configure exti4 to go to pb4
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;
	//set up interrupt and give it priority 
	NVIC_EnableIRQ(EXTI4_15_IRQn);
	
	//set priority of extio
	NVIC_SetPriority(EXTI4_15_IRQn, 1);

	/////////LED/////////////////////////////////////////
	//set pins to general purpose output mode in the moder register
	GPIOC->MODER |= (1<<14) | (1<<12);
	GPIOC->MODER &= ~(1<<15);
	GPIOC->MODER &= ~(1<<13);
	GPIOC->MODER |= (1<<18) | (1<<16);
	GPIOC->MODER &= ~(1<<19);
	GPIOC->MODER &= ~(1<<17);
	
	//set to push pull output in OTYPER reg
	GPIOC->OTYPER &= ~(1<<12);
	GPIOC->OTYPER &= ~(1<<13);
	GPIOC->OTYPER &= ~(1<<14);
	GPIOC->OTYPER &= ~(1<<15);
	GPIOC->OTYPER &= ~(1<<8);
	GPIOC->OTYPER &= ~(1<<9);
	
	//set pins low speed in OSPEEDR reg
	GPIOC->OSPEEDR &= ~(1<<12);
	GPIOC->OSPEEDR &= ~(1<<14);
	GPIOC->OSPEEDR &= ~(1<<18);
	GPIOC->OSPEEDR &= ~(1<<16);

	//set to no pullup/down resistors in PUPDR reg
	GPIOC->PUPDR &= ~(1<<12);
	GPIOC->PUPDR &= ~(1<<13);
	GPIOC->PUPDR &= ~(1<<14);
	GPIOC->PUPDR &= ~(1<<15);
	GPIOC->PUPDR &= ~(1<<16);
	GPIOC->PUPDR &= ~(1<<17);
	GPIOC->PUPDR &= ~(1<<18);
	GPIOC->PUPDR &= ~(1<<19);
	
	////////////////////////////////////////////////////



	//Setup Timer to check parking spot at certain interval 
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->PSC = 7999;
	TIM2->ARR = 250;
	TIM2->DIER |= (1 << 0);
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 2);
	

	//Configure gate and parking pins. Gate = PB4, Parking = PB5
	//input
	GPIOB -> MODER &= ~(1<<8);
	GPIOB -> MODER &= ~(1<<9);
	GPIOB -> MODER &= ~(1<<10);
	GPIOB -> MODER &= ~(1<<11);
	
	//start timer
	TIM2->CR1 |= TIM_CR1_CEN;

	//initalize screen and fill it with white
	ssd1306_Init();
	ssd1306_Fill(White);
	ssd1306_UpdateScreen();

	//set inital conditions
	gateStatus = 0;
	parkingStatus = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(ticks >= 4)
		{
			gateStatus = 0;
		}
		//update screen based on gateStatus 
		char myText[] = "Park: Taken";
		char myText2[] = "Park: Empty";
		//Make it all white
		ssd1306_Fill(White);
		ssd1306_SetCursor(20,5);

		//Write according to parking status
		if(parkingStatus == 1)
		{
			ssd1306_FillCircle(7,7,5,Black);
			ssd1306_WriteString(myText, Font_6x8, Black);
		}
		else
		{
			ssd1306_DrawCircle(7,7,5,Black);
			ssd1306_WriteString(myText2, Font_6x8, Black);
		}
		
		char myText3[] = "Gate: Detected";
		char myText4[] = "Gate: Empty";
		ssd1306_SetCursor(20,17);

		//Write according to gate status 
		if(gateStatus == 1)
		{
			ssd1306_FillCircle(7,20,5,Black);
			ssd1306_WriteString(myText3, Font_6x8, Black);
		}
		else
		{
			ssd1306_DrawCircle(7,20,5,Black);
			ssd1306_WriteString(myText4, Font_6x8, Black);
		}


		//Update ever 0.25 seconds based on the timer 
		if(timer == 1)
		{
			ssd1306_UpdateScreen();
			timer = 0;
		}
    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}
void TIM2_IRQHandler(void)
{

	//if PB5 is taken 
	if((GPIOB->IDR & 0x0020) != 0)
	{
		parkingStatus = 0;
	}
	else
	{
		
		parkingStatus = 1;
	}
	//increment ticks 
	ticks++;
	//set timer to 1 
	timer = 1;
	//send signal saying interupt is over 
	TIM2 -> SR &= ~(1<<0);
}
void EXTI4_15_IRQHandler(void)
{
	//interupt is trigged, so gate is detected 
	gateStatus = 1;

	//reset ticks 
	ticks = 0;
	//Finish the interupt
	EXTI->PR |= (1 << 0);
}
/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
