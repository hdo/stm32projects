
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define BUTTON_COUNT 16 // 4x4 matrix
#define BUTTON_MATRIX_WIDTH 4 // A: INPUT PB12-14 // B: OUTPUT PB6-PB9


uint8_t keypad_button_pressed[BUTTON_COUNT];

volatile uint32_t ticks=0;
uint32_t last_ticks = 0;

uint32_t ticks_last_rotary_check = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void HAL_SYSTICK_Callback(void)
{
	ticks++;
}

uint32_t math_calc_diff(uint32_t value1, uint32_t value2) {
	if (value1 == value2) {
		return 0;
	}
	if (value1 > value2) {
		return (value1 - value2);
	}
	else {
		// check for overflow
		return (0xffffffff - value2 + value1);
	}
}

void clear_buttons() {
	for(int i=0; i < BUTTON_COUNT; i++) {
		keypad_button_pressed[i] = 0;
	}
}

void keypad_reset_output() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // B0
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // B1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); // B2
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); // B3
}

void keypad_set_output(uint8_t b_index) {
	keypad_reset_output();
	switch(b_index) {
		case 0 : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); break;
		case 1 : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); break;
		case 2 : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); break;
		case 3 : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); break;
	}
}

uint8_t keypad_read_input(uint8_t a_index) {
	// note buttons are inverted (0 = pushed)
	uint8_t test = 0;
	switch(a_index) {
		case 0: test = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12); break;
		case 1: test = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13); break;
		case 2: test = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14); break;
		case 3: test = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15); break;
	}
	// 1 means pushed due to invert
	return test;
}

/**
 * NOTE: Simultaneous reading of vertical buttons is not possible currently
 * This is due to how horizontal ouptut is set
 */
void keypad_read_buttons() {
	clear_buttons();
	for(int i=0; i < BUTTON_MATRIX_WIDTH; i++) {
		for(int j=0; j < BUTTON_MATRIX_WIDTH; j++) {
			keypad_set_output(j);
			uint8_t index = i + BUTTON_MATRIX_WIDTH*j;
			if (keypad_read_input(i)) {
				keypad_button_pressed[index] = 1;
			}
		}
	}
}

void print_buttons() {
	for(int i=0; i < BUTTON_COUNT;i++) {
		printf("%d ", keypad_button_pressed[i]);
	}
	printf("\r\n");
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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


void send_stop_jog() {
	uint8_t data[1] = {0x85};
	HAL_UART_Transmit(&huart3, (uint8_t*) data, 1, 1000);
}

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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //printf("Starting ...\r\n");

  TIM3->CNT = 10000;
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2 );
  /* USER CODE END 2 */

  uint16_t counter_last_value = TIM3->CNT >> 1;
  int16_t counter_distance = 0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  keypad_reset_output();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  uint8_t is_jog = 0;
  uint32_t last_jog = 0;
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


	  if (is_jog && ((ticks - last_jog) > 200)) {
		  is_jog = 0;
		  //printf("\x85");
		  //printf("stop\r\n");
		  send_stop_jog();
	  }

	  /*
	  if (math_calc_diff(ticks, ticks_last_rotary_check) > 300) {
		  ticks_last_rotary_check = ticks;
		  uint16_t counter_current_value = (TIM3->CNT) >> 1;
		  int16_t counter_distance = (counter_current_value - counter_last_value);
		  //printf("%d\r\n", counter_distance);
		  counter_last_value = counter_current_value;
		  if (counter_distance != 0) {
			  //is_jog = 1;
			  //last_jog = last_ticks;
			  //printf("counter:%d \r\n", (int) counter_distance);
			  int step = counter_distance * 1;
			  int test = step;
			  if (test < 0) {
				  test *= -1;
			  }

			  int stepZ = test / 10;
			  int stepE = test % 10;

			  if (step > 0) {
				  printf("$J=G91X%d.%dF250\r\n", stepZ, stepE);
			  }
			  else {
				  printf("$J=G91X-%d.%dF250\r\n", stepZ, stepE);
			  }

		  }
	  }
	  */


	  // each x ms
	  if ((ticks - last_ticks) > 100) {
		  last_ticks = ticks;
		  // blink LED
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		  uint16_t counter_current_value = (TIM3->CNT) >> 1;
		  int16_t counter_distance = (counter_current_value - counter_last_value);
		  //printf("%d\r\n", counter_distance);
		  counter_last_value = counter_current_value;

		  if (counter_distance != 0) {
			  is_jog = 1;
			  last_jog = last_ticks;
			  if (counter_distance > 0) {
				  printf("$J=G91X10F3000\r\n");
			  }
			  else {
				  printf("$J=G91X-10F3000\r\n");
			  }
		  }


		  keypad_read_buttons();
		  //print_buttons();
		  if (keypad_button_pressed[3]) {
			  printf("$X\r\n");
		  }
		  // 2
		  if (keypad_button_pressed[1]) {
			  is_jog = 1;
			  last_jog = last_ticks;
			  printf("$J=G91Y5F2000\r\n");
		  }
		  // 8
		  if (keypad_button_pressed[9]) {
			  is_jog = 1;
			  last_jog = last_ticks;
			  printf("$J=G91Y-5F2000\r\n");
		  }
		  // 4
		  if (keypad_button_pressed[4]) {
			  is_jog = 1;
			  last_jog = last_ticks;
			  printf("$J=G91X-10F3000\r\n");
		  }
		  // 6
		  if (keypad_button_pressed[6]) {
			  is_jog = 1;
			  last_jog = last_ticks;
			  printf("$J=G91X20F3000\r\n");
		  }
		  // 3
		  if (keypad_button_pressed[2]) {
			  is_jog = 1;
			  last_jog = last_ticks;
			  printf("$J=G91Z1F500\r\n");
		  }
		  // 9
		  if (keypad_button_pressed[10]) {
			  is_jog = 1;
			  last_jog = last_ticks;
			  printf("$J=G91Z-1F500\r\n");
		  }

		  // 9
		  if (keypad_button_pressed[12]) {
			  is_jog = 1;
			  last_jog = last_ticks;
			  //printf("\x85");
			  send_stop_jog();
		  }


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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

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
