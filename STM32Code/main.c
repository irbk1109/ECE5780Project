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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void Write_Byte(char data);
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
	HAL_Init();

	SystemClock_Config();

	//Setup Interupt on specific pin to interupt when gate is detected
	//TODO: enable the RCC for specific GPIO 
	RCC->APB2ENR|= RCC_APB2ENR_SYSCFGCOMPEN;

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
	
	///////////////////////////////////////////
	//Setup I2C2 
	//PB11 to AF, open drain, and I2C2_SDA for AF
	//Internal Pull up? 
	GPIOB -> MODER |= (1<<23);
	GPIOB -> MODER &= ~(1<<22);
	
	GPIOB -> OTYPER |= (1<<11);
	
	GPIOB -> AFR[1] &= ~(1<<15);
	GPIOB -> AFR[1] &= ~(1<<14);
	GPIOB -> AFR[1] &= ~(1<<13);
	GPIOB -> AFR[1] |= (1<<12);
	//-------------------------------------------


	//PB13 to AF, open drain, and I2C2_SCL for AF
	//Internal Pull up? 
	GPIOB -> MODER |= (1<<27);
	GPIOB -> MODER &= ~(1<<26);
	
	GPIOB -> OTYPER |= (1<<13);
	
	GPIOB -> AFR[1] &= ~(1<<23);
	GPIOB -> AFR[1] |= (1<<22);
	GPIOB -> AFR[1] &= ~(1<<21);
	GPIOB -> AFR[1] |= (1<<20);
	//------------------------------------------



	//set to 100kHz
	I2C2->TIMINGR |= 0x13;
	I2C2->TIMINGR |= (0xF<<8);
	I2C2->TIMINGR |= (0x2<<16);
	I2C2->TIMINGR |= (0x4<<20);
	I2C2->TIMINGR |= (0x1<<28);

	//i2c enable
	I2C2-> CR1 |=(1<<0);
	
	//set slave address
	I2C2->CR2 |= (0x3C<<1); //address of OLED
		
	//Setup Display conditions using I2C
	Write_Byte(0xAE);    /*display off*/    
	Write_Byte(0x00);    /*set lower column addres*/
	Write_Byte(0x00);    /*set display start line*/               
	Write_Byte(0xB0);    /*set page address*/
	Write_Byte(0x81);    /*contract control*/        
	Write_Byte(0xCF);    /*128*/ 
	Write_Byte(0xA1);    /*set segment remap*/              
	Write_Byte(0xA6);    /*normal / reverse*/                
	Write_Byte(0xA8);    /*multiplex ratio*/       
	Write_Byte(0x1F);    /*duty = 1/32*/               
	Write_Byte(0xC8);    /*Com scan direction*/            
	Write_Byte(0xD3);    /*set display offset*/      
	Write_Byte(0x00);              
	Write_Byte(0xD5);    /*set osc division*/      
	Write_Byte(0x80);               
	Write_Byte(0xD9);    /*set pre-charge period*/       
	Write_Byte(0x1f);            
	Write_Byte(0xDA);    /*set COM pins*/      
	Write_Byte(0x00);               
	Write_Byte(0xdb);    /*set vcomh*/      
	Write_Byte(0x40);            
	Write_Byte(0x8d);    /*set charge pump enable*/      
	Write_Byte(0x10);               
	Write_Byte(0xAF);    /*display ON*/
	
	//Delay 100 ms 
	
	HAL_Delay(100);
	Write_Byte(0x41);
  while (1)
  {
		//Just loop to keep checking with timer and interupt interval 
  }
}

//TODO: Make Write 1 Byte to I2c
void Write_Byte(char data)
{
	//Set conditions, 1 byte, address, write, no auto end, 
	//set 1 bytes
	I2C2->CR2 |= (1<<16); 
	//RD WRN
	I2C2->CR2 &=~(1<<10); 
	//START
	I2C2->CR2 |=(1<<13); 
		
	//wait for txis or nackf
	while(1)
	{
		if(I2C2 -> ISR & I2C_ISR_TXIS) break;
	}
	
	//address of who am i reg
	I2C2->TXDR |= (data<<0);
	
	//transfer complete wait
	while(1) 
		{
			if(I2C2->ISR & I2C_ISR_TC) break;
		}
	
	//Stop
	I2C2->CR2 |= (1 << 14);
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
