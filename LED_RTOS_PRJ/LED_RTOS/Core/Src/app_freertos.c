/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "controllers.h"
#include "math.h"
#include "stdio.h"
#include "stdint.h"

  /*
   * ### Observations ###
   * Output voltage not responding to PWM control
   *
   * ###Possibility of solution
   * 1- osdelay(100) should be osdelay(1)
   *
   * Increase buffer size and use halfcplt callback
   *No offset & oversampling done on hardware / Offset compensation will  be done now after full  conver
   *Conversion on software to prevent from interaction noises btw channels
   *hard oversampling is done  to ensure stability of the measurement and offset is removed  on software in the functions readVoltage
   * */


  /*
   * ### Change log ###
   * Added delay to Get_ADC_Value function
   * Configured ADC to reference voltage of 2.048
   * Disabled ADC internal reference
   * Configured ADC internal reference to 2.5V
   * Configured ADC to use DMA
   * Increased ADC sample cycles to increase accuracy
   * ADC value converted
   * Closed loop control integrated
   * */


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
/* USER CODE BEGIN Variables */
extern ADC_HandleTypeDef hadc2;
extern UART_HandleTypeDef huart2;
extern HRTIM_HandleTypeDef hhrtim1;
ADC_ChannelConfTypeDef adcConfig = {0};
char buffer[64];
uint8_t adcFlag = 0;
uint32_t adcValue[4];
uint32_t adcValueTemp = 0;
float temp1,temp2,Vin,Iin,Vout,Iout;
float ovRef = 9.0 ;
float ovMeasure ;
float DutyCycle = 0.33;
float OvSampler;
PIController PI_voltage;
int cpt ;


/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId SerialTaskHandle;
osMessageQId VinQueueHandle;
osMessageQId VoutQueueHandle;
osMessageQId dutyCycleHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint32_t Get_ADC_Value(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t rank);
void PowerPWMSet(uint32_t frequency, float dutycycle);
float realTemp(uint32_t dig);
float realVoltageOUT(uint32_t dig);
float realVoltageIN(uint32_t dig);
float realCurrent(uint32_t dig);


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void serial_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

	/* Calibrating ADCs for better accuracy */
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

	HAL_ADC_Start_DMA(&hadc2, adcValue, 4);
	//HAL_ADC_Start(&hadc2);

	//HAL_ADC_StartSampling(&hadc2);

	/* Turn on PWM output */
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);

	/* Start counter */
	HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);

	  //HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 + HRTIM_OUTPUT_TA2);
	PowerPWMSet(100000,(1-0.05));//Function created to ease duty cycle modification

	ConfigPIController(&PI_voltage,0.0017f,(0.0017f/0.5f),0.8f,0.1f,1000.0);

	 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);


  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of VinQueue */
  osMessageQDef(VinQueue, 16, float);
  VinQueueHandle = osMessageCreate(osMessageQ(VinQueue), NULL);

  /* definition and creation of VoutQueue */
  osMessageQDef(VoutQueue, 16, float);
  VoutQueueHandle = osMessageCreate(osMessageQ(VoutQueue), NULL);

  /* definition and creation of dutyCycle */
  osMessageQDef(dutyCycle, 16, float);
  dutyCycleHandle = osMessageCreate(osMessageQ(dutyCycle), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of SerialTask */
  osThreadDef(SerialTask, serial_task, osPriorityNormal, 0, 256);
  SerialTaskHandle = osThreadCreate(osThread(SerialTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  /* Infinite loop */
  for(;;)
  {
//    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	//HAL_ADC_StartSampling(&hadc2);

	//HAL_ADC_StartSampling(&hadc2);

	/* Get the ADC values */
	//taskENTER_CRITICAL();

	/*// ADC Channel 1
	//for(int i=0;i<10;i++)
	  //{
	adcValue[0] = Get_ADC_Value(&hadc2, ADC_CHANNEL_1, ADC_REGULAR_RANK_1);

	// ADC Channel 2
	adcValue[1] = Get_ADC_Value(&hadc2, ADC_CHANNEL_2, ADC_REGULAR_RANK_1);

	// ADC Channel 8
	adcValue[2] = Get_ADC_Value(&hadc2, ADC_CHANNEL_8, ADC_REGULAR_RANK_1);

	// ADC Channel 9
	adcValue[3] = Get_ADC_Value(&hadc2, ADC_CHANNEL_9, ADC_REGULAR_RANK_1);

	HAL_ADC_Stop(&hadc2);*/
	//osDelay(100);

	// HAL_ADC_Start(&hadc1);
	//HAL_ADC_PollForConversion(&hadc1, 10);
	//HAL_ADC_StopSampling(&hadc2);

	//taskENTER_CRITICAL();


	  if(adcFlag)
	  {
		  adcFlag = 0;
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	  }
	Vin = realVoltageIN((float)adcValue[0]);
	Vout =realVoltageOUT((float)adcValue[1]);

	//taskEXIT_CRITICAL();

	if(Vout<50.0)
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
	    DutyCycle = 0.05;
		PowerPWMSet(100000, 1-DutyCycle);
	  }

	//snprintf(buffer, 64, "CH1:%d, CH2:%d, CH3:%d, CH4:%d \r\n", adcValue[0], adcValue[1], adcValue[2], adcValue[3]);


	//taskEXIT_CRITICAL();

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_serial_task */
/**
* @brief Function implementing the SerialTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_serial_task */
void serial_task(void const * argument)
{
  /* USER CODE BEGIN serial_task */
  /* Infinite loop */
  for(;;)
  {

	 snprintf(buffer, 64, "Vin:%.2f, Vout:%.2f, Duty Cycle:%.2f\r\n", Vin, Vout, DutyCycle);

	 HAL_UART_Transmit(&huart2,buffer, 64, 500);


    osDelay(1000);
  }
  /* USER CODE END serial_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

uint32_t Get_ADC_Value(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t rank)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    // Configure the selected ADC channel
    sConfig.Channel = channel;
    sConfig.Rank = rank;
    sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5; // Adjust sampling time as needed
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_1  ;  // No offset applied
    sConfig.OffsetSign = ADC_OFFSET_NONE;
    sConfig.Offset = 0;                      // No offset applied

    /*if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    {
        // Configuration error
        Error_Handler();
    }*/

    // Start the ADC conversion
    if (HAL_ADC_Start(hadc) != HAL_OK)
    {
        // Start error
        Error_Handler();
    }

    // Wait for the conversion to complete
    if (HAL_ADC_PollForConversion(hadc, 1) != HAL_OK)
    {
        // Polling error
        Error_Handler();
    }

    // Get the ADC value
    uint32_t adcValue = HAL_ADC_GetValue(hadc);

    osDelay(100);

    // Stop the ADC conversion
    // HAL_ADC_Stop(hadc);

    // Return the ADC value
    return adcValue;
}

//This function convert the digital value converted by adc in temperature
float realTemp(uint32_t dig)
{
	float raw = ((float)dig/4095.0f)*2.048f;//Digital to analog reconversion
	raw = raw*1000.0f;
	float R , T ;
	float T0 = (273.15f + 25.0f);
	float R0 = 10000.0f;
	float B = 3450.0f;
	R = ( (raw*20000.0f)/(3300.0f - raw) );
	T =  B/( log(R) - log(R0) + (B/T0));
	return (T - 273.15f);
}

//This function convert digital value by adc in voltage
float realVoltageIN(uint32_t dig)
{
    float raw = ((float)dig / 4095.0f) * 2.5f;
    return ((raw - 1.020f) * 89.7606f) + 10.3f;
}

float realVoltageOUT(uint32_t dig)
{
    float raw = ((float)dig / 4095.0f) * 2.5f;
    return ((raw - 1.017f) * 89.7606f) + 7.8f;
}


//This function convert digital value by adc in current
float realCurrent(uint32_t dig)
{
	float raw = ((float)dig / 4095.0f) * 2.048f; // Digital to analog reconversion
	float real = (float)(raw - 1000.0f) / 50.0f;
	return (real/1000.0f);
}


//This function makes a direct DAC without consideration of acq chain conditioning
float rawVoltage(uint32_t dig)
{
	float raw = ((float)dig / 4095.0f) * 2.5f; // Digital to analog reconversion
	return raw;
}

/* USER CODE END Application */

