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
#include "stm32f0xx.h"

#define SSD1306_I2C_ADDR 0x3C

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

// Function to initialize I2C
void I2C_Init() {
    // Enable GPIOB and I2C2 clock
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

    // Configure PB13 (I2C2_SCL) and PB11 (I2C2_SDA) as alternate function mode
    GPIOB -> MODER |= (1<<23);
		GPIOB -> MODER &= ~(1<<22);
		GPIOB -> OTYPER |= (1<<11);
		GPIOB -> AFR[1] &= ~(1<<15);
		GPIOB -> AFR[1] &= ~(1<<14);
		GPIOB -> AFR[1] &= ~(1<<13);
		GPIOB -> AFR[1] |= (1<<12);

    GPIOB -> MODER |= (1<<27);
		GPIOB -> MODER &= ~(1<<26);
		GPIOB -> OTYPER |= (1<<13);
		GPIOB -> AFR[1] &= ~(1<<23);
		GPIOB -> AFR[1] |= (1<<22);
		GPIOB -> AFR[1] &= ~(1<<21);
		GPIOB -> AFR[1] |= (1<<20);

    // Select alternate function AF1 for PB13 and PB11
    GPIOB->AFR[1] |= (1 << 12) | (1 << 20);

    // Configure I2C2
    I2C2->CR1 &= ~I2C_CR1_PE; // Disable I2C2
    I2C2->TIMINGR = 0x00201D2B; // Standard mode timing settings
    I2C2->CR1 |= I2C_CR1_PE; // Enable I2C2
}


// Function to send data via I2C
void I2C_Write(uint8_t addr, uint8_t* data, uint16_t len) {
    // Ensure start condition
    I2C2->CR2 &= ~I2C_CR2_AUTOEND;
    I2C2->CR2 |= I2C_CR2_START;

    // Wait until START condition is set
    while (!(I2C2->ISR & I2C_ISR_TXIS));

    // Send slave address
    I2C2->TXDR = (addr << 1) | 0; // Write mode

    // Wait until address is sent
    while (!(I2C2->ISR & I2C_ISR_TC));

    // Send data
    for (uint16_t i = 0; i < len; i++) {
        // Wait until data register is empty
        while (!(I2C2->ISR & I2C_ISR_TXE));
        I2C2->TXDR = data[i];
    }

    // Wait until stop condition is sent
    while (!(I2C2->ISR & I2C_ISR_STOPF));
    I2C2->ICR |= I2C_ICR_STOPCF; // Clear stop flag
}

// Function to send a command to SSD1306
void ssd1306_WriteCommand(uint8_t cmd) {
    uint8_t data[2];
    data[0] = 0x00; // Command mode
    data[1] = cmd;
    I2C_Write(SSD1306_I2C_ADDR, data, 2);
}

// Function to initialize SSD1306
void ssd1306_Init() {
    // Initialization commands for SSD1306 (for 128x32 resolution)
    ssd1306_WriteCommand(0xAE); // Display off

    // Set Display Clock Divide Ratio / Oscillator Frequency
    ssd1306_WriteCommand(0xD5);
    ssd1306_WriteCommand(0x80); // Default ratio

    // Set Multiplex Ratio
    ssd1306_WriteCommand(0xA8);
    ssd1306_WriteCommand(0x1F); // 0x1F for 32 pixels height, 0x3F for 64 pixels height

    // Set Display Offset
    ssd1306_WriteCommand(0xD3);
    ssd1306_WriteCommand(0x00); // No offset

    // Set Display Start Line
    ssd1306_WriteCommand(0x40); // Start line 0

    // Charge Pump Setting
    ssd1306_WriteCommand(0x8D);
    ssd1306_WriteCommand(0x14); // Enable charge pump

    // Memory Addressing Mode
    ssd1306_WriteCommand(0x20);
    ssd1306_WriteCommand(0x00); // Horizontal addressing mode

    // Set Segment Re-map
    ssd1306_WriteCommand(0xA1); // Set segment re-map 0 to 127

    // Set COM Output Scan Direction
    ssd1306_WriteCommand(0xC8); // Set COM Output Scan Direction (remapped)

    // Set COM Pins Hardware Configuration
    ssd1306_WriteCommand(0xDA);
    ssd1306_WriteCommand(0x02); // Alternative COM pin configuration, disable COM Left/Right remap

    // Set Contrast Control
    ssd1306_WriteCommand(0x81);
    ssd1306_WriteCommand(0xCF); // Default contrast value

    // Disable Entire Display On
    ssd1306_WriteCommand(0xA4);

    // Set Normal Display
    ssd1306_WriteCommand(0xA6);

    // Set Display On
    ssd1306_WriteCommand(0xAF);
}

// Function to display a character
void ssd1306_WriteChar(uint8_t ch) {
    uint32_t i = 0;
    uint32_t j = 0;

    // Send character data
    for(i = 0; i < 8; i++) {
        ssd1306_WriteCommand(0x40 + i);
        for(j = 0; j < 128; j++) {
            ssd1306_WriteCommand(ch);
        }
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	//initialize everything
	HAL_Init();
	I2C_Init();
	SystemClock_Config();
	ssd1306_Init();
	
	//unmask exti0 
	EXTI->IMR  |= (1 << 0);
	
	//Figure out what the trigger is rising or fall edge 
	EXTI->RTSR |= (1 << 0); 
	
	//configure the register, FIGURE out the pin 
	SYSCFG->EXTICR[0] &= ~( 1<<0);
	SYSCFG->EXTICR[0] &= ~( 1<<1);
	SYSCFG->EXTICR[0] &= ~( 1<<2);
	SYSCFG->EXTICR[0] &= ~( 1<<3);
	
	//set up interrupt and give it priority 
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	
	//set priority of extio
	NVIC_SetPriority(EXTI0_1_IRQn, 1);

	//Setup Timer to check parking spot at certain interval 
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->PSC = 7999;
	TIM2->ARR = 250;
	TIM2->DIER |= (1 << 0);
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 2);
	
	//Configure gate and parking pins. Gate = PB0, Parking = PB1
	//input
	GPIOB -> MODER &= (1<<0);
	GPIOB -> MODER &= (1<<1);
	GPIOB -> MODER &= (1<<2);
	GPIOB -> MODER &= (1<<3);

	
	//Delay 100 ms 
	HAL_Delay(100);
	ssd1306_WriteChar(0xFF); // For demonstration, sending all pixels on
	
	
  while (1)
  {
		//Just loop to keep checking with timer and interupt interval 
  }
}


void TIM2_IRQHandler(void)
{
	//Check the pin of the Parking spot 

	//WRite to oled if detected 

}
void EXTI0_1_IRQHandler(void)
{
	//Check the pin of the Gate Detection

	//write to OLED if detected 
	
	//Finish the interupt
	EXTI->PR |= (1 << 0);
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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