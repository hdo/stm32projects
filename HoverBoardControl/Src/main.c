
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#define ACTIVE 0x055
#define NONE_ACTIVE 0x0AA
#define MAX_SPEED 1500
#define DEFAULT_SPEED 1500
#define MAX_ADC_SPEED 1500
#define MAX_BUTTON_SPEED 500

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

volatile uint32_t ticks=0;
uint16_t wheelData1[9] = {0x100, 0x000, 0x000, 0x000, 0x000, NONE_ACTIVE, 0x058, 0x058, 0x0FE};
uint16_t wheelData2[9] = {0x100, 0x000, 0x000, 0x000, 0x000, NONE_ACTIVE, 0x058, 0x058, 0x0FE};

uint32_t adc_val=0;
uint16_t selected_speed = 0;
uint8_t button_1, button_2;
int16_t speed = 0;
uint8_t reverse_mode = 0;

/* USER CODE BEGIN PV */

/*****************************************
 * SERIAL OUTPUT BEGIN
 *****************************************/

int _write(int file, char *data, int len)
{

   // arbitrary timeout 1000
   HAL_StatusTypeDef status =
      HAL_UART_Transmit(&huart3, (uint8_t*)data, len, 1000);

   // return # of bytes written - as best we can tell
   return (status == HAL_OK ? len : 0);
}
/*****************************************
 * SERIAL OUTPUT END
 *****************************************/





void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adc_val = HAL_ADC_GetValue (&hadc1);
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);



void HAL_SYSTICK_Callback(void)
{
	ticks++;
}


void set_wheel_state(uint16_t state) {
	  wheelData1[5] = state;
	  wheelData2[5] = state;
}

void set_speed(uint16_t speed_value, uint8_t reverse)
{
	// max speed is 1500
	uint16_t value = speed_value;
	if (speed_value > 1500) {
		value = 1500;
	}

	uint16_t value1 = value;
	// 2 complement
	uint16_t value2 =  (~value)+1;

	if (reverse == 1) {
		value2 = value;
		value1 =  (~value)+1;

	}

	uint16_t low_a = value1 & 0xFF;
	uint16_t high_a = (value1 >> 8) & 0xFF;
	wheelData1[1] = low_a;
	wheelData1[2] = high_a;
	wheelData1[3] = low_a;
	wheelData1[4] = high_a;


	//uint16_t low_b = value2 & 0xFF;
	//uint16_t high_b= (value2 >> 8) & 0xFF;
	uint16_t low_b = value1 & 0xFF;
	uint16_t high_b= (value1 >> 8) & 0xFF;
	wheelData2[1] = low_b;
	wheelData2[2] = high_b;
	wheelData2[3] = low_b;
	wheelData2[4] = high_b;

}


uint16_t read_speed() {
	// ADC 1000 => 0
	// ADC 3000 => MAX
	if (adc_val < 1000) {
		return  0;
	}
	if (adc_val > 1000) {
		uint16_t temp = ((adc_val-1000)*(MAX_ADC_SPEED/2000.0));
		if (temp > MAX_ADC_SPEED) {
			temp = MAX_ADC_SPEED;
		}
		return temp;
	}
	return 0;
}

void wheel_task() {
  HAL_UART_Transmit(&huart1, (uint8_t *) wheelData1, 9, 0xFFFF);
  HAL_UART_Transmit(&huart2, (uint8_t *) wheelData2, 9, 0xFFFF);
}

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t last_ticks = ticks;
  uint32_t last_button = 0;


  set_speed(DEFAULT_SPEED, 0);

  HAL_ADC_Start_IT(&hadc1);

  printf("Starting ...\r\n");

  set_wheel_state(NONE_ACTIVE);
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


	  /*
	  if (speed > 0) {
		  set_speed(speed, 0);
	  } else if (speed < 0) {
		  set_speed(-speed, 1);
	  }
	  */

	  //set_speed(1000,0);

	  set_speed(read_speed(), reverse_mode);

	  wheel_task();

	  HAL_Delay(1); // 1 ms



	  // set intitial state
	  //set_wheel_state(NONE_ACTIVE);
	  reverse_mode = 0;

	  if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) {
		  button_1 = 1;
	  }
	  else {
		  button_1 = 0;
	  }

	  if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) {
		  button_2 = 1;
	  }
	  else {
		  button_2 = 0;
	  }

	  if (button_1 && button_2) {
		  // invalid state
		  //set_wheel_state(NONE_ACTIVE);
	  }
	  else {
		  if (button_1) {
			  last_button = ticks;
			  reverse_mode = 0;
			  set_wheel_state(ACTIVE);

			  if (speed < MAX_BUTTON_SPEED) {
				  speed += 2;
				  if (speed > MAX_BUTTON_SPEED) {
					  speed = MAX_BUTTON_SPEED;
				  }
			  }
		  }
		  if (button_2) {
			  last_button = ticks;
			  reverse_mode = 1;
			  set_wheel_state(ACTIVE);

			  if (speed > -MAX_BUTTON_SPEED) {
				  speed -= 2;
				  if (speed < -MAX_BUTTON_SPEED) {
					  speed = -MAX_BUTTON_SPEED;
				  }
			  }
		  }
	  }

	  // each x ms
	  if ((ticks - last_ticks) > 100 ) {
		  last_ticks = ticks;
		  // heart beat
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		  //printf("ping\r\n");
		  printf("ADC: %d\r\n", (uint16_t) adc_val);
		  printf("adc speed: %d\r\n", read_speed());
		  printf("btn speed: %d\r\n", speed);
		  printf("reverse: %d\r\n", reverse_mode);


		  if ((ticks - last_button) > 5000) {
			  speed = 0;
		  }
	  }
	  if ((ticks - last_button) > 100) {
		  set_wheel_state(NONE_ACTIVE);
		  printf("release\r\n");
	  }

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  //HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/100000); // us

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
