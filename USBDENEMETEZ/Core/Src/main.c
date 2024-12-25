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
#include "string.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TIM_USB_SIMULINK htim3

//ODRIVE
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
#define MOTOR_3 1//

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2004c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2004c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2004c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2004c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
extern uint8_t USB_RX_Buf[18];
uint8_t USB_TX_BUF[34];

typedef union{
	float number;
	uint8_t bytes[4];
}NUMUNION_t;

NUMUNION_t positionFloatNum0;
NUMUNION_t positionFloatNum1;
NUMUNION_t positionFloatNum2;
NUMUNION_t positionFloatNum3;

NUMUNION_t velocityFloatNum0;
NUMUNION_t velocityFloatNum1;
NUMUNION_t velocityFloatNum2;
NUMUNION_t velocityFloatNum3;

extern NUMUNION_t desiredPositionFloatNum0;
extern NUMUNION_t desiredPositionFloatNum1;
extern NUMUNION_t desiredPositionFloatNum2;
extern NUMUNION_t desiredPositionFloatNum3;

uint8_t TX_BUF_ODRV[50];
uint8_t RX_BUF_ODRV[50];
uint8_t motor_count = 0;

uint8_t motor_count1 = 0;
uint8_t motor_count2 = 0;

uint8_t TX_BUF1[50];
uint8_t TX_BUF2[50];
uint8_t RX_BUF1[50];
uint8_t RX_BUF2[50];

uint8_t g_dataReceivedFlag = 0;

uint8_t TX_BUF[30];
uint8_t buflen=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
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
		sprintf((char*)TX_BUF, "p %u %.3f %.3f %.3f\r\n", motor, position, velocity_ff, torque_ff);//f %u\n , motor
//		strcpy((char*)TX_BUF_ODRV,(char*)TX_BUF);
		HAL_UART_Transmit(huart, TX_BUF, 24, 1000);
//		HAL_UART_Transmit_IT(huart, TX_BUF, 16);
	}

}
void getFeedback(uint8_t motor, float* position, float* velocity, UART_HandleTypeDef* huart)
{
//	uint8_t TX_BUF[50];
	uint8_t RX_BUF[50];
	if(motor == 0 || motor == 1)
	{
//		sprintf((char*)TX_BUF, "f %u\n", motor);
//		strcpy((char*)TX_BUF_ODRV,(char*)TX_BUF);
//		HAL_UART_Transmit(huart, TX_BUF, strlen((char*)TX_BUF), 100);
		HAL_UART_Receive(huart, RX_BUF,24,1000);
//		strcpy((char*)RX_BUF_ODRV,(char*)RX_BUF);
		sscanf((char*)RX_BUF,"%f %f\r\n", position, velocity);
//		memset (RX_BUF_ODRV, '\0', 20);  // clear the buffer
//		memset (RX_BUF, '\0', 20);  // clear the buffer
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_Base_Start_IT(&TIM_USB_SIMULINK);

//HAL_UART_Receive_IT(&huart2, (uint8_t*)RX_BUF1, 24);/////////////////////////
//HAL_UART_Receive_IT(&huart6, (uint8_t*)RX_BUF2, 24);
//
HAL_UART_Transmit_IT(&huart2, (uint8_t*)TX_BUF1, 16);
HAL_UART_Transmit_IT(&huart6, (uint8_t*)TX_BUF2, 16);

//positionFloatNum0.number=3.14;
//positionFloatNum1.number=1.612;
//positionFloatNum2.number=2.2;
//positionFloatNum3.number=3.7;
//velocityFloatNum0.number=0.322;
//velocityFloatNum1.number=-0.456;
//velocityFloatNum2.number=1.032;
//velocityFloatNum3.number=0.992;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


//	  if(g_dataReceivedFlag == 1)
//	  {
//		  g_dataReceivedFlag = 0;
//
//	  }


//	  clearErrors(&uart_odrive1);
//	  HAL_Delay(10);
//	  clearErrors(&uart_odrive2);
////	  HAL_Delay(10);

	  uint8_t motor=1;
	  float position = desiredPositionFloatNum0.number;
	  sprintf((char*)TX_BUF, "p %u %.3f 0 0\r\n", motor, position);
	  if(position >=0 )
	  {
		  buflen=14;
		  //		  HAL_UART_Transmit(&uart_odrive2,TX_BUF ,14, 2000);
	  }
	  else
	  {
//		  HAL_UART_Transmit(&uart_odrive2,TX_BUF ,15, 2000);
		  buflen=15;
	  }

	  HAL_Delay(100);
//	  setPosition(MOTOR_0, desiredPositionFloatNum0.number, 0, 0, );
//	  setPosition(MOTOR_1, desiredPositionFloatNum1.number, 0, 0, &uart_odrive1);
//	  setPosition(MOTOR_2, desiredPositionFloatNum2.number, 0, 0, &uart_odrive2);
//	  setPosition(MOTOR_3, desiredPositionFloatNum3.number, 0, 0, &uart_odrive2);
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
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  htim3.Init.Period = 1000-1;
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

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &TIM_USB_SIMULINK)
	{
		  USB_TX_BUF[0] = '#';
		  USB_TX_BUF[1] = positionFloatNum0.bytes[0];
		  USB_TX_BUF[2] = positionFloatNum0.bytes[1];
		  USB_TX_BUF[3] = positionFloatNum0.bytes[2];
		  USB_TX_BUF[4] = positionFloatNum0.bytes[3];

		  USB_TX_BUF[5] = velocityFloatNum0.bytes[0];
		  USB_TX_BUF[6] = velocityFloatNum0.bytes[1];
		  USB_TX_BUF[7] = velocityFloatNum0.bytes[2];
		  USB_TX_BUF[8] = velocityFloatNum0.bytes[3];

		  USB_TX_BUF[9] = positionFloatNum1.bytes[0];
		  USB_TX_BUF[10] = positionFloatNum1.bytes[1];
		  USB_TX_BUF[11] = positionFloatNum1.bytes[2];
		  USB_TX_BUF[12] = positionFloatNum1.bytes[3];

		  USB_TX_BUF[13] = velocityFloatNum1.bytes[0];
		  USB_TX_BUF[14] = velocityFloatNum1.bytes[1];
		  USB_TX_BUF[15] = velocityFloatNum1.bytes[2];
		  USB_TX_BUF[16] = velocityFloatNum1.bytes[3];

		  USB_TX_BUF[17] = positionFloatNum2.bytes[0];
		  USB_TX_BUF[18] = positionFloatNum2.bytes[1];
		  USB_TX_BUF[19] = positionFloatNum2.bytes[2];
		  USB_TX_BUF[20] = positionFloatNum2.bytes[3];

		  USB_TX_BUF[21] = velocityFloatNum2.bytes[0];
		  USB_TX_BUF[22] = velocityFloatNum2.bytes[1];
		  USB_TX_BUF[23] = velocityFloatNum2.bytes[2];
		  USB_TX_BUF[24] = velocityFloatNum2.bytes[3];

		  USB_TX_BUF[25] = positionFloatNum3.bytes[0];
		  USB_TX_BUF[26] = positionFloatNum3.bytes[1];
		  USB_TX_BUF[27] = positionFloatNum3.bytes[2];
		  USB_TX_BUF[28] = positionFloatNum3.bytes[3];

		  USB_TX_BUF[29] = velocityFloatNum3.bytes[0];
		  USB_TX_BUF[30] = velocityFloatNum3.bytes[1];
		  USB_TX_BUF[31] = velocityFloatNum3.bytes[2];
		  USB_TX_BUF[32] = velocityFloatNum3.bytes[3];

		  USB_TX_BUF[33] = '\n';

		  CDC_Transmit_FS(USB_TX_BUF, 34);//Simulink is getting feedback values at this point
		  // Maybe i can split every feedback and send them in order. It could be the problem

		 desiredPositionFloatNum0.bytes[0] = USB_RX_Buf[1];
		 desiredPositionFloatNum0.bytes[1] = USB_RX_Buf[2];
		 desiredPositionFloatNum0.bytes[2] = USB_RX_Buf[3];
		 desiredPositionFloatNum0.bytes[3] = USB_RX_Buf[4];
		 desiredPositionFloatNum1.bytes[0] = USB_RX_Buf[5];
		 desiredPositionFloatNum1.bytes[1] = USB_RX_Buf[6];
		 desiredPositionFloatNum1.bytes[2] = USB_RX_Buf[7];
		 desiredPositionFloatNum1.bytes[3] = USB_RX_Buf[8];
		 desiredPositionFloatNum2.bytes[0] = USB_RX_Buf[9];
		 desiredPositionFloatNum2.bytes[1] = USB_RX_Buf[10];
		 desiredPositionFloatNum2.bytes[2] = USB_RX_Buf[11];
		 desiredPositionFloatNum2.bytes[3] = USB_RX_Buf[12];
		 desiredPositionFloatNum3.bytes[0] = USB_RX_Buf[13];
		 desiredPositionFloatNum3.bytes[1] = USB_RX_Buf[14];
		 desiredPositionFloatNum3.bytes[2] = USB_RX_Buf[15];
		 desiredPositionFloatNum3.bytes[3] = USB_RX_Buf[16];
	}
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if((huart == &uart_odrive1)||(huart == &uart_odrive2))
//	{
//		switch(motor_count)
//			{
//			case 0:
//				motor_count++;
//				getFeedback(MOTOR_0, &positionFloatNum0.number, &velocityFloatNum0.number, &uart_odrive1);
//				setPosition(MOTOR_2, desiredPositionFloatNum2.number, 0, 0, &uart_odrive2);
//				break;
//			case 1:
//				motor_count++;
//
//				setPosition(MOTOR_1, desiredPositionFloatNum1.number, 0, 0, &uart_odrive1);
//				getFeedback(MOTOR_1, &positionFloatNum1.number, &velocityFloatNum1.number, &uart_odrive1);
//
//				break;
//			case 2:
//				motor_count++;
//				getFeedback(MOTOR_2, &positionFloatNum2.number, &velocityFloatNum2.number, &uart_odrive2);
//				setPosition(MOTOR_3, desiredPositionFloatNum3.number, 0, 0, &uart_odrive2);
//				break;
//			case 3:
//				motor_count++;
//				motor_count = 0;
//				getFeedback(MOTOR_3, &positionFloatNum3.number, &velocityFloatNum3.number, &uart_odrive2);
//				setPosition(MOTOR_0, desiredPositionFloatNum0.number, 0, 0, &uart_odrive1);
//				break;
//			}
//	}
//
//
//}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if((huart == &uart_odrive1)||(huart == &uart_odrive2))
//		{
//			switch(motor_count)
//				{
//				case 0:
//					motor_count++;
////					getFeedback(MOTOR_0, &positionFloatNum0.number, &velocityFloatNum0.number, &uart_odrive1);
//					setPosition(MOTOR_1, desiredPositionFloatNum1.number, 0, 0, &uart_odrive1);
//					break;
//				case 1:
//					motor_count++;
////					getFeedback(MOTOR_1, &positionFloatNum1.number, &velocityFloatNum1.number, &uart_odrive1);
//					setPosition(MOTOR_2, desiredPositionFloatNum2.number, 0, 0, &uart_odrive2);
//					break;
//				case 2:
//					motor_count++;
////					getFeedback(MOTOR_2, &positionFloatNum2.number, &velocityFloatNum2.number, &uart_odrive2);
//					setPosition(MOTOR_3, desiredPositionFloatNum3.number, 0, 0, &uart_odrive2);
//					break;
//				case 3:
////					motor_count++;
//					motor_count = 0;
////					getFeedback(MOTOR_3, &positionFloatNum3.number, &velocityFloatNum3.number, &uart_odrive2);
//					setPosition(MOTOR_0, desiredPositionFloatNum0.number, 0, 0, &uart_odrive1);
//					break;
//				}
//		}
//
//}///////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		if(motor_count1 == 0)
		{
			sscanf((char*)RX_BUF1,"%f %f\r\n", &positionFloatNum0.number, &velocityFloatNum0.number);
			//interpretting feedback request answer for motor 1
		}
		else if(motor_count1 == 1)
		{
			sscanf((char*)RX_BUF1,"%f %f\r\n", &positionFloatNum1.number, &velocityFloatNum1.number);
			//interpretting feedback request answer for motor 2
		}

		HAL_UART_Receive_IT(&huart2, (uint8_t*)RX_BUF1, 24);/////////////////
	}
	else if(huart->Instance == USART6)
	{
		if(motor_count2 == 0)
		{
			sscanf((char*)RX_BUF2,"%f %f\r\n", &positionFloatNum2.number, &velocityFloatNum2.number);
			//interpretting feedback request answer for motor 3
			// maybe a space will be enough for sscanf, if there is a problem
		}
		else if(motor_count2 == 1)
		{
			sscanf((char*)RX_BUF2,"%f %f\r\n", &positionFloatNum3.number, &velocityFloatNum3.number);
			//interpretting feedback request answer for motor 4
		}
		HAL_UART_Receive_IT(&huart6, (uint8_t*)RX_BUF2, 24);
		//Actually making an endless loop will be better. Store values at Buff and then interpret them
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2) // GPS Transmit
	{
//		float num;
//		if(motor_count1 == 0)
//			num = desiredPositionFloatNum0.number;
//		else
//			num = desiredPositionFloatNum1.number;
//		sprintf((char*)TX_BUF1, "p %u %.3f\r\nf %u\r\n", motor_count1, num, motor_count1);//preparing buff for transmit
		HAL_UART_Transmit_IT(&huart2, (uint8_t*)TX_BUF1, buflen);// at the same time, feedback request has been made
//		motor_count1++;
//		if(motor_count1 == 2)
//			motor_count1 = 0;

	}
	else if(huart->Instance == USART6)
	{
//		float num;
//		if(motor_count2 == 0)
//			num = desiredPositionFloatNum2.number;
//		else
//			num = desiredPositionFloatNum3.number;
//		sprintf((char*)TX_BUF2, "p %u %.3f\r\nf %u\r\n", motor_count2, num, motor_count2);
		HAL_UART_Transmit_IT(&huart6, (uint8_t*)TX_BUF, buflen);

//		motor_count2++;
//		if(motor_count2 == 2)
//			motor_count2 = 0;
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
