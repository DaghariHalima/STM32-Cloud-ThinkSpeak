/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "string.h"
char BUFFER_TO_SEND[100];
uint16_t ADC_VALUE[3];

uint8_t  RECEIVE_CHAR;
uint8_t BUFFER[100];
uint8_t BUFFER2[100]="HELLO\n";
int i=-1;
uint8_t CMD_AT0[10]="AT\r\n";
uint8_t CMD_AT1[10]="AT+RST\r\n";
uint8_t CMD_AT2[15]="AT+CWMODE=1\r\n";
//uint8_t CMD_AT3[53]="AT+CWJAP=\"Galaxy S5 6815\",\"5346680853466808\"\r\n";
uint8_t CMD_AT3[53]="AT+CWJAP=\"No one needs u\",\"halima98\"\r\n";
//SendTxt_USART3("AT+CWJAP=\"NetBox-4A87\",\"G9B2EADJAY1\"\r\n");
uint8_t CMD_AT4[12]="AT+CIFSR\r\n";
uint8_t CMD_AT5[15]="AT+CIPMUX=0\r\n";
uint8_t CMD_AT6[46]="AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n";
uint8_t CMD_AT7[18]="AT+CIPSEND=100\r\n";
uint8_t CMD_AT8[13]="AT+CIPCLOSE\r\n";

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
void CONFIGURATION_ESP8266();
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void delay(int ncount)
{
	while(ncount--);
}

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
	CONFIGURATION_ESP8266();
 
  while (1)
  {
    //ADC1->SQR3=0x4;
		 HAL_ADC_Start(&hadc1);
		 HAL_ADC_PollForConversion(&hadc1, 100);
		 ADC_VALUE[0]=HAL_ADC_GetValue(&hadc1);;
		 HAL_ADC_Stop(&hadc1);
	
		
	  
	
	   HAL_UART_Transmit(&huart3,&CMD_AT6[0],46, 10);
	   delay(1000000);
	   HAL_UART_Transmit(&huart3,&CMD_AT7[0],18, 10);
	   delay(500000);
	   sprintf(BUFFER_TO_SEND,"GET /update?key=10M8JRO1Z6PFKR0Q&field1=%d\r\n",ADC_VALUE[0]);   
	   HAL_UART_Transmit(&huart3,&BUFFER_TO_SEND[0],strlen(BUFFER_TO_SEND),5);
     delay(200000);
	   HAL_UART_Transmit(&huart3,&CMD_AT8[0],13, 10);
	   delay(1300000);

  }
  /* USER CODE END 3 */

}

void CONFIGURATION_ESP8266()
{
	HAL_UART_Transmit(&huart3,&CMD_AT0[0],6, 10);
	HAL_Delay(3000);
	HAL_UART_Transmit(&huart3,&CMD_AT1[0],10, 10);
	HAL_Delay(3000);
	HAL_UART_Transmit(&huart3,&CMD_AT2[0],15, 10);
	HAL_Delay(3000);
	HAL_UART_Transmit(&huart3,&CMD_AT3[0],53, 10);
	HAL_Delay(3000);
	HAL_UART_Transmit(&huart3,&CMD_AT4[0],12, 10);
	HAL_Delay(3000);
	HAL_UART_Transmit(&huart3,&CMD_AT5[0],12, 10);
	HAL_Delay(3000);

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);

}

/** Pinout Configuration
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
