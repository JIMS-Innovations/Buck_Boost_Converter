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
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "hrtim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "controllers.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_SIZE 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint32_t ADC_DMA[ADC_BUF_SIZE];
uint32_t adcVal;
uint8_t adcFlag  = 0;
float realTemp(uint32_t dig);
//This function convert digital value by adc in current
float realCurrent(uint32_t dig);
//This function convert digital value by adc in voltage
float realVoltage(uint32_t dig);
//This function make direct DAC without consideration of acq chain conditionning
float rawVoltage(uint32_t dig);
uint32_t adcValue ;
uint32_t Get_ADC_Value(ADC_HandleTypeDef *hadc, uint32_t channel,uint32_t rank);
void PowerPWMSet(uint32_t frequency, float dutycycle);
float temp1,temp2,Vin,Iin,Vout,Iout;
float ovRef = 9.0 ;
float ovMeasure ;
float DutyCycle ;
PIController PI_voltage;
int cpt ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void updateADCVal(void)
//{
//
//	/* Moving ADC values from DMA buffer to variables */
//	vInVal = adcVals[0];
//	iInVal = adcVals[1];
//	vOutVal = adcVals[2];
//	iOutVal = adcVals[3];
//}
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
	//ConfigPIController(&PI_voltage,Kp_i,Ti_i,Up_limit_i,Low_limit_i,F_samp);
	ConfigPIController(&PI_voltage,0.0017f,(0.0017f/0.5f),0.8f,0.1f,1000.0);
	//Ki=Kp/Ti

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
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_FDCAN2_Init();
  MX_HRTIM1_Init();
  MX_LPUART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();
  MX_ADC4_Init();
  /* USER CODE BEGIN 2 */

  /* Calibrating ADCs for better accuracy */
  if(HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
	  Error_Handler();

  if(HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED) != HAL_OK)
	  Error_Handler();

  if(HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED) != HAL_OK)
	  Error_Handler();

  /* Starting ADC in DMA mode */
  /*if(HAL_ADC_Start_DMA(&hadc2, ADC_DMA, ADC_BUF_SIZE) != HAL_OK)
	  Error_Handler();*/


  /* Starting ADCs in normal mode */
  if(HAL_ADC_Start(&hadc2) != HAL_OK)
    	  Error_Handler();

  if(HAL_ADC_Start(&hadc3) != HAL_OK)
  	  Error_Handler();

  if(HAL_ADC_Start(&hadc4) != HAL_OK)
  	  Error_Handler();

  /* Turn on PWM output */
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);

  /* Start counter */
  HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);

  //HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 + HRTIM_OUTPUT_TA2);
  PowerPWMSet(100000,(1-0.33));//Function created to ease dutty cycle modification

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	  for(int i=0;i<10;i++)
	  	  {
	  	  	adcValue = adcValue + Get_ADC_Value(&hadc4, ADC_CHANNEL_5,ADC_REGULAR_RANK_1);
	  	  }
	  	  temp1 = realTemp((float)adcValue/10.0f);
	  	  adcValue = 0;


	  	  for(int i=0;i<10;i++)
	  	  {
	  	  	 adcValue = adcValue + Get_ADC_Value(&hadc3, ADC_CHANNEL_12,ADC_REGULAR_RANK_1);
	  	  }
	  	  temp2 = realTemp((float)adcValue/10.0f);
	  	  adcValue = 0;

	  	  for(int i=0;i<100;i++)
	  	  {
	  		 adcValue = adcValue + Get_ADC_Value(&hadc2, ADC_CHANNEL_9,ADC_REGULAR_RANK_1);
	  	  }
	  	  Vout = realVoltage((float)adcValue/100.0f);
	  	  adcValue = 0;




	  	if(temp1<100 && temp2<100 && Vout <50)
	  		  {
	  			  //Close loop :
	  			  	//1 -  Set output voltage

	  			  	//2 - Measure output voltage
	  			       ovMeasure = fabs(Vout);
	  			  	// 3 - Calculte Error = Set - Meas & new dc
	  			       DutyCycle = RunPIController(&PI_voltage, (ovRef-ovMeasure));
	  			  	//4 -update pwm
	  			     //HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
	  			     PowerPWMSet(100000,(1-DutyCycle));

	  		  }
	  		  else
	  		  {
	  			  //HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 + HRTIM_OUTPUT_TA2);
	  			  DutyCycle = 1 - 0.05;
	  			PowerPWMSet(100000, DutyCycle);
	  		  }


	  	  /*for(int i=0;i<10;i++)
	  	  {
	  	  	adcValue = adcValue + Get_ADC_Value(&hadc2, ADC_CHANNEL_2);
	  	  }
	  	  Iin = realCurrent((float)adcValue/10.0f);
	  	  adcValue = 0;

	  	  for(int i=0;i<10;i++)
	  	  {
	  		adcValue = adcValue + Get_ADC_Value(&hadc2, ADC_CHANNEL_9);
	  	  }
	  	  Vout = realVoltage((float)adcValue/10.0f);
	  	  adcValue = 0;

	  	  for(int i=0;i<10;i++)
	  	  {
	  		adcValue = adcValue + Get_ADC_Value(&hadc2, ADC_CHANNEL_8);
	  	  }
	  	  Iout= realCurrent((float)adcValue/10.0f);
	  	  adcValue = 0;*/
	  	  uint8_t buffer[64];
	  	  snprintf(buffer, 64, "Vin:%.2f, Vout:%.2f, Duty Cycle:%.2f\r\n", Vin, ovMeasure, DutyCycle);

	  	  HAL_UART_Transmit(&huart2,buffer, 64, 500);
		  HAL_Delay(1); // Delay for 10 milllisecond
		  //cpt=cpt+1;
	  //}
    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ADC DMA callback function */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	adcFlag = 1;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
