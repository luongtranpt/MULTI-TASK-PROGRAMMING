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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include <stdio.h>
#include "stdlib.h"
#include "string.h"

#include "Humd.h"
#include "Temp.h"
#include "LCD_I2C.h"
#include "timers.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum user
{
  Mode_Normal,
  Mode_Humd,
  Mode_Temp
} user_t;

typedef enum
{
  Mes_UART_Temp  = 1,
  Mes_UART_Humd  = 2,
	Mes_Humd       = 3,
  Mes_Temp       = 4,
}Message_t;

typedef struct 
{
  Message_t ID;
  uint16_t Data_Int;
  double Data_double;
}Mes_Queue;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BIT_Humd     (1 << 0)
#define BIT_Temp     (1 << 1)
#define BIT_Trans    (1 << 2)
#define BIT_RECV     (1 << 3)
#define UART_BUFFER_LEN 6
#define START_BYTE 1
#define END_BYTE 2
#define ERROR_FRAME 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */

/* USER CODE BEGIN PV */
user_t Mode = Mode_Normal;
char tg[10];
volatile int indx = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
int a = 0;
int b = 0;
int c = 0;
uint8_t period;
uint32_t time = 0;
uint16_t turn = 0;
volatile uint32_t frequency;
uint8_t ucRxData;
uint8_t ucRxBuffer[UART_BUFFER_LEN] = {0};
uint8_t ucRxFlag = 0;
uint8_t ucRxCnt = 0;
uint8_t flag_Humd_UART = 0;
uint8_t flag_Temp_UART = 0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

xTaskHandle HumdHandle;
xTaskHandle WaitHandle;
xTaskHandle TempHandle;
xTaskHandle LCDHandle;
xTaskHandle UARTHandle;

void HumdTask(void *argument);
void WaitTask(void *argument);
void TempTask(void *argument);
void LCDTask(void *argument);
void UARTTask(void *argument);
void vTimerCallback(TimerHandle_t);
void trans(char *);

// void myTimerCallback(void *argument);

TimerHandle_t xTimers;
SemaphoreHandle_t HumdWait;
SemaphoreHandle_t TempWait;
QueueHandle_t xQueue_data;
SemaphoreHandle_t UARTSemaphore;
EventGroupHandle_t DISPLAY_GROUP;

// TimerHandle_t xTimer;

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
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /*============= Init peripheral ===============*/

  HAL_TIM_Base_Start(&htim1); // Bat timer
  HAL_TIM_Base_Start_IT(&htim2);
  LCD_I2C_Init();                             // Khoi dong LCD_I2C
  HAL_ADC_Start_IT(&hadc1);                   // Bat ADC
  HAL_UART_Receive_IT(&huart2, &ucRxData, 1); // HAL_UART_Receive_IT(&huart2,&data_rx,1); // khoi dong ngat nhan uart

  /*============ CREATE TASK ==============*/
  // xTimers = xTimerCreate("Timer1",pdMS_TO_TICKS( 1000 ),pdTRUE,( void * ) 0,vTimerCallback);

  xTaskCreate(HumdTask, "Humd", 128, NULL, 2, &HumdHandle);
  xTaskCreate(WaitTask, "Wait", 64, NULL, 1, &WaitHandle);
  xTaskCreate(TempTask, "Temp", 64, NULL, 2, &TempHandle);
  xTaskCreate(LCDTask, "LCD", 128, NULL, 3, &LCDHandle);
  xTaskCreate(UARTTask, "UART", 256, NULL, 4, &UARTHandle);
  // xTimerStart( xTimers,0);
  xTimers = xTimerCreate("Timer1", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, vTimerCallback);
  xTimerStart(xTimers, 0);
  HumdWait = xSemaphoreCreateBinary();
  TempWait = xSemaphoreCreateBinary();
  DISPLAY_GROUP = xEventGroupCreate();
  UARTSemaphore = xSemaphoreCreateBinary();
  xQueue_data = xQueueCreate(10, sizeof(Mes_Queue));
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 35;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
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
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void trans(char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), HAL_MAX_DELAY);
}
void vTimerCallback(TimerHandle_t xTimer)
{
  flag_Humd_UART++;
  flag_Temp_UART++;
}
void Get_Time()
{
  double t;
  char str[40];
  t = ((double)(turn * 65536 + __HAL_TIM_GET_COUNTER(&htim2))) / 36000000 * 1000;
  sprintf(str, "Current T: %.2lf:\n", t);
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

void HumdTask(void *argument)
{
  uint32_t Humd;
  while (1)
  {
    xSemaphoreTake(HumdWait, portMAX_DELAY);
    Mes_Queue Message;
    Message.ID = Mes_Humd;
    Humd = Get_Humd(frequency - 900);
    Message.Data_Int = Humd;
    xQueueSendToFront(xQueue_data, (void *)&Message, (TickType_t)10);
    if (flag_Humd_UART > 1)
    {
      Message.ID = Mes_UART_Humd;
      xQueueSendToFront(xQueue_data, (void *)&Message, (TickType_t)10);
      xEventGroupSetBits(DISPLAY_GROUP, BIT_Trans);
      flag_Humd_UART = 0;
    }
    a = 0;
    xEventGroupSetBits(DISPLAY_GROUP, BIT_Humd);
  }
}

void WaitTask(void *argument)
{

  while (1)
  {
    if (a > (100 + 5 * period) )
    {
      xSemaphoreGive(HumdWait);
    }
    if (b > (850+ 42 * period) )
    {
      xSemaphoreGive(TempWait);
    }
  }
}

void TempTask(void *argument)
{
  while (1)
  {
    char str[100];
    uint16_t adcv;
    double Temp;
    xSemaphoreTake(TempWait, portMAX_DELAY);
    adcv = (uint16_t)HAL_ADC_GetValue(&hadc1); // Lay gia tri cua ADC
    Temp = temp_convert(adcv);
    Mes_Queue Message;
    Message.ID = Mes_Temp;
    Message.Data_double = Temp;
    xQueueSendToFront(xQueue_data, (void *)&Message, (TickType_t)10);
    if (flag_Temp_UART > 1)
    {
      Message.ID = Mes_UART_Temp;
      xQueueSendToFront(xQueue_data, (void *)&Message, (TickType_t)10);
      xEventGroupSetBits(DISPLAY_GROUP, BIT_Trans);
      flag_Temp_UART = 0;
    }

    b = 0;
    xEventGroupSetBits(DISPLAY_GROUP, BIT_Temp);
  }
}
void LCDTask(void *argument)
{
  EventBits_t uxBits;
  while (1)
  {
    uxBits = xEventGroupWaitBits(DISPLAY_GROUP, BIT_Humd | BIT_Temp, pdTRUE, pdFALSE, 100 / portTICK_PERIOD_MS);
    if ((uxBits & BIT_Humd) != 0)
    {
      Mes_Queue Message;
      xQueueReceive(xQueue_data, &Message, 10);
      if (Message.ID == Mes_Humd)
      {
        LCD_I2C_Set_Cursor(0, 0);                    // Dat vi tri hang 0, cot 0 cho con tro LCD
        LCD_I2C_Write_String("Humd:");               //
        LCD_I2C_WRITE_NUMBER((int)Message.Data_Int); // IN RA GIA TRI TAN SO
        LCD_I2C_Write_String("%");
      }
    }
    else if ((uxBits & BIT_Temp) != 1)
    {
      Mes_Queue Message;
      xQueueReceive(xQueue_data, &Message, 10);
      if (Message.ID == Mes_Temp)
      {
        LCD_I2C_Set_Cursor(1, 0); // Vi tri hang 0 cot 1
        LCD_I2C_Write_String("Temp:");
        LCD_I2C_WRITE_Num_f(Message.Data_double); // IN RA GIA TRI NHIET DO
        LCD_I2C_WRITE_DATA(0xDF);
        LCD_I2C_WRITE_DATA('C');
      }
    }
  }
}

void UARTTask(void *argument)
{
  EventBits_t uxBits;
  while (1)
  {
    uxBits = xEventGroupWaitBits(DISPLAY_GROUP, BIT_Trans | BIT_RECV, pdTRUE, pdFALSE, portMAX_DELAY);
    if ((uxBits & BIT_RECV) != 0)
    {
      if (strcmp((char *)ucRxBuffer, "&norm*") == 0)
        Mode = Mode_Normal;
      else if (strcmp((char *)ucRxBuffer, "&humd*") == 0)
        Mode = Mode_Humd;
      else if (strcmp((char *)ucRxBuffer, "&temp*") == 0)
        Mode = Mode_Temp;
      else if (ucRxBuffer[1] == 'T' && ucRxBuffer[2] == '_')
      {
        // check++;
        uint8_t per[2] = {ucRxBuffer[3], ucRxBuffer[4]};
        period = atoi((char *)per);
        xTimerChangePeriod( xTimers, pdMS_TO_TICKS( 1000 + period*100 ) , 100 );
      }
      memset((char *)ucRxBuffer, 0, strlen((char *)ucRxBuffer));
      ucRxCnt = 0;
      ucRxFlag = 0;
      HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
    }
    else if ((uxBits & BIT_Trans) != 0)
    {
      char str[100];
      Mes_Queue Message;
      xQueueReceive(xQueue_data, &Message, 10);
      switch (Mode)
      {
      case Mode_Temp:

        if (Message.ID == Mes_UART_Temp)
        {
          Get_Time();
          sprintf(str, "Period = %d\n",period);
          HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
          sprintf(str, "Temp = %.1lf\n", Message.Data_double);
          HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
        }
        break;
      case Mode_Humd:
        if (Message.ID == Mes_UART_Humd)
        {
          Get_Time();
          sprintf(str, "Period = %d\n",period);
          HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
          sprintf(str, "Humd = %d\n", Message.Data_Int);
          HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
        }
        break;
      case Mode_Normal:
        static int flag1 = 0;
        static int flag2 = 0;
        static Mes_Queue Data_Temp, Data_Humd;
        if (Message.ID == Mes_UART_Temp)
        {
          Data_Temp.ID = Mes_Temp;
          Data_Temp.Data_double = Message.Data_double;
          flag1 = 1;
        }
        else if (Message.ID == Mes_UART_Humd)
        {
          Data_Humd.ID = Mes_Humd;
          Data_Humd.Data_Int = Message.Data_Int;
          flag2 = 1;
        }
        if ((flag1 >= 1) && (flag2 >= 1))
        {
          Get_Time();
          sprintf(str, "Period = %d\n",period);
          HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
          sprintf(str, "Humd = %d\n", Data_Humd.Data_Int);
          HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
          vTaskDelay(100);
          sprintf(str, "Temp = %.1lf\n", Data_Temp.Data_double);
          HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
          flag1 = 0;
          flag2 = 0;
        }
        break;
      default:
        trans("Nhap sai Mode vui long nhap lai\n");
      }
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_15)
  {
    uint32_t timer1_count = __HAL_TIM_GET_COUNTER(&htim1); // lay gia tri moi cua timer
    static uint32_t lastTime = 0;
    uint32_t period = (timer1_count - lastTime);

    // tinh tan so
    frequency = 1000000 / period; // Chuyen doi ra HZ

    lastTime = timer1_count; // gia tri cua cua timer
    a++;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == hadc1.Instance)
  {
    b++;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (ucRxData == '&')
  {
    memset((char *)ucRxBuffer, 0, strlen((char *)ucRxBuffer));
    ucRxBuffer[0] = ucRxData;
    ucRxFlag = START_BYTE;
    HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
  }
  else if (ucRxFlag == START_BYTE && ucRxCnt < UART_BUFFER_LEN)
  {
    ucRxBuffer[++ucRxCnt] = ucRxData;
    if (ucRxData == '*')
      xEventGroupSetBitsFromISR(DISPLAY_GROUP, BIT_RECV, &xHigherPriorityTaskWoken);
    else
      HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
  }
  else if (ucRxCnt == UART_BUFFER_LEN)
  {
    ucRxFlag = ERROR_FRAME;
    memset((char *)ucRxBuffer, 0, strlen((char *)ucRxBuffer));
    ucRxCnt = 0;
    HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
  }
  HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if( htim->Instance == htim2.Instance )
  turn++;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */

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

#ifdef USE_FULL_ASSERT
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
