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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define uart_odrive1 huart2
#define uart_odrive2 huart6
#define timer_Trajectory htim1
#define AXIS_STATE_UNDEFINED (uint8_t)(0 + 48)
#define AXIS_STATE_IDLE (uint8_t)(1 + 48)
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE (uint8_t)(3 + 48)
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION (uint8_t)(7 + 48)
#define AXIS_STATE_CLOSED_LOOP_CONTROL (uint8_t)(8 + 48)

#define MOTOR_0 0
#define MOTOR_1 1
#define MOTOR_2 0
#define MOTOR_3 1

#define MAX_TRAJECTORY_INDEX 10

#define USB_RX_ITEM_CNT 1//ya da 4, düşün
#define USB_TX_ITEM_CNT 2
#define TIM_USB_SIMULINK htim3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t TX_BUF_ODRV[50];
uint8_t RX_BUF_ODRV[50];

uint8_t testVariableMotor = 1;
float testVariablePos = 0.9876;
float testVariableTrajectory = 5;
float testVariableVelocity_ff = 1;
uint8_t testVariableState;
uint8_t vbus_voltage[10];
uint8_t isCalibrated;

//float trajectoryOfM0[] = {0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1,0.1, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18, 0.19,0.2, 0.21, 0.22, 0.23, 0.24, 0.25, 0.26, 0.27, 0.28, 0.29,0.3, 0.31, 0.32, 0.33, 0.34, 0.35, 0.36, 0.37, 0.38, 0.39,0.4, 0.41, 0.42, 0.43, 0.44, 0.45, 0.46, 0.47, 0.48, 0.49,0.5, 0.51, 0.52, 0.53, 0.54, 0.55, 0.56, 0.57, 0.58, 0.59,0.6, 0.61, 0.62, 0.63, 0.64, 0.65, 0.66, 0.67, 0.68, 0.69,0.7, 0.71, 0.72, 0.73, 0.74, 0.75, 0.76, 0.77, 0.78, 0.79,0.8, 0.81, 0.82, 0.83, 0.84, 0.85, 0.86, 0.87, 0.88, 0.89,0.9, 0.91, 0.92, 0.93, 0.94, 0.95, 0.96, 0.97, 0.98, 0.99};
float trajectory2[] = {0 ,0.25, 0.5, 0.75 , 1, 1, 0.75, 0.5, 0.25, 0};
uint32_t trajectoryIndex = 0;
uint8_t setPointFlag = 0;

float positionM0, velocityM0, positionM1, velocityM1, positionM2, velocityM2, positionM3, velocityM3;


extern uint8_t USB_RX_Buf[4*USB_RX_ITEM_CNT + 2];
uint8_t USB_TX_Buf[4*USB_TX_ITEM_CNT+2];// Napmışım emin olamadım// Hee float sayıyı 4bit gönderiyoruz ya o bu + 2 de # ve \n

typedef union{
	float fltnum;
	uint8_t fbyte[4];
}NUMUNION_t;

NUMUNION_t positionFloatNum;
NUMUNION_t velocityFloatNum;

uint8_t motorCount = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void setPosition(uint8_t motor, float position, float velocity_ff, float torque_ff, UART_HandleTypeDef* huart);
void setVelocity(uint8_t motor, float velocity, float torque_ff, UART_HandleTypeDef* huart);
void setTorque(uint8_t motor, float torque, UART_HandleTypeDef* huart);
void setTrajectory(uint8_t motor, float position, UART_HandleTypeDef* huart);

void setState(uint8_t motor, uint8_t state, UART_HandleTypeDef* huart);
void setParameter(uint8_t motor, char* property, char* value, UART_HandleTypeDef* huart);
void clearErrors(UART_HandleTypeDef* huart);

uint8_t getState(uint8_t motor, UART_HandleTypeDef* huart);
void getParameter(uint8_t motor, char* property, uint8_t* value, UART_HandleTypeDef* huart);
void getFeedback(uint8_t motor, float* position, float* velocity, UART_HandleTypeDef* huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void setTrajectory(uint8_t motor, float position, UART_HandleTypeDef* huart)
{

	uint8_t TX_BUF[50];

	if(motor == 0 || motor == 1)
	{
		sprintf((char*)TX_BUF, "t %u %.3f\n", motor, position);
		strcpy((char*)TX_BUF_ODRV,(char*)TX_BUF);
//		HAL_UART_Transmit(huart, TX_BUF, 10, 1000);
		HAL_UART_Transmit_IT(huart, TX_BUF, strlen((char*)TX_BUF));
	}

}
void setPosition(uint8_t motor, float position, float velocity_ff, float torque_ff, UART_HandleTypeDef* huart)
{

	uint8_t TX_BUF[50];

	if(motor == 0 || motor == 1)
	{
		sprintf((char*)TX_BUF, "p %u %.3f %.3f %.3f\nf %u\n", motor, position, velocity_ff, torque_ff, motor);
		strcpy((char*)TX_BUF_ODRV,(char*)TX_BUF);
//		HAL_UART_Transmit(huart, TX_BUF, 18, 1000);
		HAL_UART_Transmit(huart, TX_BUF, strlen((char*)TX_BUF),100);
	}

}
void getFeedback(uint8_t motor, float* position, float* velocity, UART_HandleTypeDef* huart)
{
	uint8_t TX_BUF[50];
	uint8_t RX_BUF[50];
	if(motor == 0 || motor == 1)
	{
		sprintf((char*)TX_BUF, "f %u\n", motor);
		strcpy((char*)TX_BUF_ODRV,(char*)TX_BUF);
		HAL_UART_Transmit(huart, TX_BUF, strlen((char*)TX_BUF), 100);
		HAL_UART_Receive(huart, RX_BUF, strlen((char*)RX_BUF), 100);
		sscanf((char*)RX_BUF, "%f %f", position, velocity);
//		HAL_UART_Transmit_IT(huart, TX_BUF, strlen((char*)TX_BUF));
	}
}

void setVelocity(uint8_t motor, float velocity, float torque_ff, UART_HandleTypeDef* huart)
{
	uint8_t TX_BUF[50];

		if(motor == 0 || motor == 1)
		{
			sprintf((char*)TX_BUF, "v %u %.3f %.3f\nf %u\n", motor, velocity, torque_ff, motor);
			strcpy((char*)TX_BUF_ODRV,(char*)TX_BUF);
	//		HAL_UART_Transmit(huart, TX_BUF, 18, 1000);
			HAL_UART_Transmit_IT(huart, TX_BUF, strlen((char*)TX_BUF));
		}
}

void setTorque(uint8_t motor, float torque, UART_HandleTypeDef* huart)
{
	uint8_t TX_BUF[50];

		if(motor == 0 || motor == 1)
		{
			sprintf((char*)TX_BUF, "c %u %.3f\nf %u\n", motor, torque, motor);
			strcpy((char*)TX_BUF_ODRV,(char*)TX_BUF);
	//		HAL_UART_Transmit(huart, TX_BUF, 18, 1000);
			HAL_UART_Transmit_IT(huart, TX_BUF, strlen((char*)TX_BUF));
		}
}

void setState(uint8_t motor, uint8_t state, UART_HandleTypeDef* huart)
{

	uint8_t TX_BUF[30] = "w axis0.requested_state 1\n";

	TX_BUF[24] = state;

	if(motor == 0 || motor == 1)
	{
		TX_BUF[6] = motor + 48;
		strcpy((char*)TX_BUF_ODRV,(char*)TX_BUF);
		HAL_UART_Transmit(huart, TX_BUF, 26, 1000);
//		HAL_UART_Transmit_IT(huart, TX_BUF, 26);
	}

}

uint8_t getState(uint8_t motor, UART_HandleTypeDef* huart)
{
	uint8_t TX_BUF[30] = "r axis0.current_state\n";
//	uint8_t AxisState[24] = "axis0.current_state\n";
	uint8_t stateValue;
//	strcat((char*)TX_BUF,(char*)AxisState);

//	if(motor == 0 || motor == 1)
//	{
		TX_BUF[6] = motor + 48;
		strcpy((char*)TX_BUF_ODRV,(char*)TX_BUF);
		HAL_UART_Transmit(&uart_odrive1, TX_BUF, 22, 1000);
//		HAL_UART_Transmit_IT(huart, TX_BUF, 22);
//		HAL_Delay(100);
		HAL_UART_Receive(&uart_odrive1, RX_BUF_ODRV, sizeof(RX_BUF_ODRV), 1000);
//		HAL_UART_Receive_IT(huart, RX_BUF_ODRV, sizeof(RX_BUF_ODRV));
//		RX_BUF_ODRV[0] = stateValue;
		stateValue = RX_BUF_ODRV[0];
		return stateValue;
//	}


//	return AXIS_STATE_IDLE;//if an error occurs change state to idle

}

void setParameter(uint8_t motor, char* property, char* value, UART_HandleTypeDef* huart)
{
	uint8_t TX_BUF[50] = "w ";

	strcat(strcat(strcat(strcat((char*)TX_BUF, property)," "), value),"\n");

	if(motor == 0 || motor == 1)
	{
		strcpy((char*)TX_BUF_ODRV,(char*)TX_BUF);
//		TX_BUF_ODRV[6] = motor + 48;
		HAL_UART_Transmit(huart, TX_BUF, sizeof(TX_BUF), 1000);
//		HAL_UART_Transmit_IT(huart, TX_BUF, sizeof(TX_BUF));
	}
}

void getParameter(uint8_t motor, char* property, uint8_t* value, UART_HandleTypeDef* huart)
{
	uint8_t TX_BUF[50] = "r ";

	strcat(strcat((char*)TX_BUF, property),"\n");

	if(motor == 0 || motor == 1)
	{
		strcpy((char*)TX_BUF_ODRV,(char*)TX_BUF);
//		TX_BUF_ODRV[6] = motor + 48;
		HAL_UART_Transmit(huart, TX_BUF, 28, 1000);
//		HAL_UART_Transmit_IT(huart, TX_BUF, 28);
	}
	HAL_Delay(10);
	HAL_UART_Receive(huart, value, sizeof(value), 1000);
//	HAL_UART_Receive_IT(huart, value, sizeof(value));
}

void clearErrors(UART_HandleTypeDef* huart)
{
	uint8_t TX_BUF[50] = "sc\n";
	strcpy((char*)TX_BUF_ODRV,(char*)TX_BUF);
	HAL_UART_Transmit(huart, TX_BUF, 3, 1000);
//	HAL_UART_Transmit_IT(huart, TX_BUF, 3);
}
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
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&timer_Trajectory);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  setPosition(MOTOR_0, 5*testVariablePos, 0, 0, &uart_odrive1);
	  setPosition(MOTOR_1, testVariablePos, 0, 0, &uart_odrive2);
	  setPosition(MOTOR_2, 5*testVariablePos, 0, 0, &uart_odrive1);
	  setPosition(MOTOR_3, testVariablePos, 0, 0, &uart_odrive2);
	  getFeedback(MOTOR_0, &positionM0, &velocityM0, &uart_odrive1);
	  getFeedback(MOTOR_1, &positionM1, &velocityM1, &uart_odrive1);
	  getFeedback(MOTOR_2, &positionM2, &velocityM2, &uart_odrive2);
	  getFeedback(MOTOR_3, &positionM3, &velocityM3, &uart_odrive2);


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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 8400-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	switch(setPointFlag)
//	{
//		case 0:
//			setPointFlag++;
//			setPosition(MOTOR_1, trajectory2[trajectoryIndex], 0, 0, &uart_odrive1);
//			break;
//		case 1:
//			setPointFlag++;
//			setPosition(MOTOR_2, trajectory2[trajectoryIndex], 0, 0, &uart_odrive2);
//			break;
//		case 2:
//			setPointFlag++;
//			setPosition(MOTOR_3, trajectory2[trajectoryIndex], 0, 0, &uart_odrive2);
//			break;
//
//		case 3:
//			trajectoryIndex++;
//
//			if(trajectoryIndex == MAX_TRAJECTORY_INDEX)
//			{
//				trajectoryIndex = 0;
//			}
//			break;
//	}
//
//}
////
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &timer_Trajectory)
	{
//		setPointFlag = 0;
//		setPosition(MOTOR_0, -3*trajectory2[trajectoryIndex], 0, 0, &uart_odrive1);
	}
	if(htim == &TIM_USB_SIMULINK)
	{
		if(motorCount == 0)
		{
			positionFloatNum.fltnum = positionM0;
			velocityFloatNum.fltnum = velocityM0;
		}
		positionFloatNum.fltnum = positionM0;
		sprintf((char*)USB_TX_Buf,"#%u%u%u%u%u%u%u%u\n",positionFloatNum.fbyte[0],positionFloatNum.fbyte[1],positionFloatNum.fbyte[2],positionFloatNum.fbyte[3],velocityFloatNum.fbyte[0],velocityFloatNum.fbyte[1],velocityFloatNum.fbyte[2],velocityFloatNum.fbyte[3]);
		CDC_Transmit_FS(USB_TX_Buf, 10);
	}
}
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
