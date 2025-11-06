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
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_VBATT 255 //valeur max sur ADC1 resolution de 12bits
#define SEUIL_VBATT 232 //3V

#define PWM 20661 //10 cm/s  //80000 * 0.25

#define PWM_MOTEUR_DROIT  TIM2->CCR4
#define PWM_MOTEUR_GAUCHE TIM2->CCR1

#define SENS_MOTEUR_DROIT_AVANCE 	HAL_GPIO_WritePin(Cmde_DirD_GPIO_Port, Cmde_DirD_Pin, 1)
#define SENS_MOTEUR_GAUCHE_AVANCE 	HAL_GPIO_WritePin(Cmde_DirG_GPIO_Port, Cmde_DirG_Pin, 1)

#define SENS_MOTEUR_DROIT_RECULE 	HAL_GPIO_WritePin(Cmde_DirD_GPIO_Port, Cmde_DirD_Pin, 0)
#define SENS_MOTEUR_GAUCHE_RECULE 	HAL_GPIO_WritePin(Cmde_DirG_GPIO_Port, Cmde_DirG_Pin, 0)

#define DEMARRAGE_MOTEUR_DROIT 	TIM2->CCR4 = PWM
#define DEMARRAGE_MOTEUR_GAUCHE	TIM2->CCR1 = PWM

#define ARRET_MOTEUR_DROIT 		TIM2->CCR4 = 0
#define ARRET_MOTEUR_GAUCHE 	TIM2->CCR1 = 0

// Coefficient asservissement

#define KP 1.5f
#define KI 0.4f
#define KD 0.2f
#define CONSIGNE_D 300
#define CONSIGNE_G 300
#define PWM_MAX 20661
#define PWM_MIN 0



#define SEUIL_IR 200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t mesures_IR = 0; //contient les quatres flags de detection des leds IR
volatile uint8_t nb_conv = 0; //nombre de conversions realisees dans le cycle de conversion de l'ADC
volatile uint8_t flag_blanc = 0;
volatile uint8_t mesure = 0;
volatile uint8_t init = 1;
volatile int blancs[] = {0, 0};
volatile int tbl_detection[] = {0, 0};
volatile int seuils_detection[] = {40, -30};
volatile uint8_t start = 0;
volatile unsigned int Vbatt = MAX_VBATT;

typedef enum {ETEINDRE_LEDS, BLANC, ALLUMER_LEDS, MESURES} etat;
volatile etat etat_courant = BLANC;


extern TIM_HandleTypeDef htim1;   // Timer de commande moteur (PWM)

// asservissement

//static float somme_erreurs = 0;
//static float erreur_precedente = 0;

// DMA
#define NB_ADC_CHANNELS 3
#define CH_IR1 0 // led gauche
#define CH_IR2 1 // led droite
#define CH_VBATT 2 // batterie

uint32_t adc_buffer[NB_ADC_CHANNELS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void correction_trajectoire(void);
//void read_speed(void);
//void asservissement_PID(void);

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6); //demarrage scan des leds toutes les 10ms

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //demarrage pwm moteur Gauche
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); //demarrage pwm moteur Droit

  HAL_GPIO_WritePin(Cmde_DirD_GPIO_Port, Cmde_DirD_Pin, 1); // avancer ou reculer ?
  HAL_GPIO_WritePin(Cmde_DirG_GPIO_Port, Cmde_DirG_Pin, 1);

  SENS_MOTEUR_DROIT_AVANCE;
  SENS_MOTEUR_GAUCHE_AVANCE;
  ARRET_MOTEUR_DROIT;
  ARRET_MOTEUR_GAUCHE;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_GPIO_WritePin(Cmde_led_IR2_GPIO_Port, Cmde_led_IR2_Pin, 1);
//	  while(1);
//	  read_speed();
//	  HAL_Delay(100);

	  while(!start) {
		  // tant que l'on n'appuie pas sur le bouton de demarrage
		  ARRET_MOTEUR_DROIT;
		  ARRET_MOTEUR_GAUCHE;

	  }


	  //affichage batterie en console
//	  char message[30];
//	  sprintf(message, "Tension de la batterie %d\n", Vbatt);
//	  HAL_UART_Transmit(&huart2, (const uint8_t*)message, sizeof(message), HAL_MAX_DELAY);

	  //avancer tout droit
	  SENS_MOTEUR_DROIT_AVANCE;
	  SENS_MOTEUR_GAUCHE_AVANCE;
	  DEMARRAGE_MOTEUR_DROIT;
	  DEMARRAGE_MOTEUR_GAUCHE;


	  if (mesures_IR) {
		  //arret moteurs

		  //correction de trajectoire
		  correction_trajectoire();
		  //HAL_Delay(100);

		  //remise a zéro des flags leds IR
		  mesures_IR = 0;

	  }


//	  HAL_Delay(100);
//	  mesures_IR = 0;

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analog WatchDog 1
  */
  AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_3;
  AnalogWDGConfig.ITMode = ENABLE;
  AnalogWDGConfig.HighThreshold = 255;
  AnalogWDGConfig.LowThreshold = 232;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 80000-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 13-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 61538;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Alert_batt_Pin|Cmde_led_IR3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Cmde_DirG_Pin|Cmde_led_IR1_Pin|Cmde_led_IR4_Pin|Cmde_led_IR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Cmde_DirD_GPIO_Port, Cmde_DirD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Start_btn_Pin */
  GPIO_InitStruct.Pin = Start_btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Start_btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Alert_batt_Pin Cmde_led_IR3_Pin */
  GPIO_InitStruct.Pin = Alert_batt_Pin|Cmde_led_IR3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Cmde_DirG_Pin Cmde_led_IR1_Pin Cmde_led_IR4_Pin Cmde_led_IR2_Pin */
  GPIO_InitStruct.Pin = Cmde_DirG_Pin|Cmde_led_IR1_Pin|Cmde_led_IR4_Pin|Cmde_led_IR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Cmde_DirD_Pin */
  GPIO_InitStruct.Pin = Cmde_DirD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Cmde_DirD_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Fonction boutton anti-rebond
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t dernier_appui = 0;

    if (GPIO_Pin == Start_btn_Pin)
    {
        // Anti-rebond logiciel : 200 ms de délai minimum
        if (HAL_GetTick() - dernier_appui > 200)
        {
            start ^= 1;  // inverse le flag start
            dernier_appui = HAL_GetTick(); // met à jour le moment de l'appui
        }
    }
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim6) {
		//callback machine d'etat
		switch (etat_courant) {

			case ETEINDRE_LEDS:
				HAL_GPIO_WritePin(Cmde_led_IR1_GPIO_Port, Cmde_led_IR1_Pin, 0);
				HAL_GPIO_WritePin(Cmde_led_IR2_GPIO_Port, Cmde_led_IR2_Pin, 0);

				etat_courant = BLANC;
				break;


			case BLANC:
			    flag_blanc = 1;
			    HAL_ADC_Start_DMA(&hadc1, adc_buffer, NB_ADC_CHANNELS);
			    etat_courant = ALLUMER_LEDS;
			    break;



			case ALLUMER_LEDS:
				HAL_GPIO_WritePin(Cmde_led_IR1_GPIO_Port, Cmde_led_IR1_Pin, 1);
				HAL_GPIO_WritePin(Cmde_led_IR2_GPIO_Port, Cmde_led_IR2_Pin, 1);
				// laisser le temps de s'allumer
				etat_courant = MESURES;
				break;



			case MESURES:
			    flag_blanc = 0;
			    HAL_ADC_Start_DMA(&hadc1, adc_buffer, NB_ADC_CHANNELS);
			    etat_courant = ETEINDRE_LEDS;
			    break;



			default:
				etat_courant = ETEINDRE_LEDS;
				break;
		}
	}
}



//gestion des mesures
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
    	for (int i = 0; i < 2; i++)
    	{
    	    uint8_t adc_val = (uint8_t)adc_buffer[i]; // cast clair à 8 bits

    	    if (flag_blanc) {
    	        blancs[i] = adc_val;
    	    } else {
    	        int diff = (int)adc_val - blancs[i];
    	        tbl_detection[i] = diff;

    	        char buffer[64];
    	        int len = sprintf(buffer, "LED num %d, Blanc : %d, Mesure : %d, Detection %d\n",
    	                          i,blancs[i], adc_val, diff);
    	        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

    	        if (diff > seuils_detection[i]) {
    	            mesures_IR |= 1 << i;
    	        }
    	    }
    	}

    }
}





// fonction de correction de trajectoire
void correction_trajectoire(void) {
	switch (mesures_IR) {
		case 0b00000010:

			ARRET_MOTEUR_DROIT;
			ARRET_MOTEUR_GAUCHE;

			// tourne a droite

			SENS_MOTEUR_DROIT_RECULE;
			SENS_MOTEUR_GAUCHE_AVANCE;

			DEMARRAGE_MOTEUR_DROIT;
			DEMARRAGE_MOTEUR_GAUCHE;

			HAL_Delay(200);

			ARRET_MOTEUR_DROIT;
			ARRET_MOTEUR_GAUCHE;



			break;

		case 0b00000001:
			ARRET_MOTEUR_DROIT;
			ARRET_MOTEUR_GAUCHE;

			// tourne a gauche
			SENS_MOTEUR_DROIT_AVANCE;
			SENS_MOTEUR_GAUCHE_RECULE;

			DEMARRAGE_MOTEUR_DROIT;
			DEMARRAGE_MOTEUR_GAUCHE;

			HAL_Delay(200);

			ARRET_MOTEUR_DROIT;
			ARRET_MOTEUR_GAUCHE;


			break;



		default:
			//HAL_Delay(1000);
			break;
	}
}

//void read_speed(void) {
//    static int32_t last_encoder_count_d = 0;
//    static int32_t last_encoder_count_g = 0;
//    char buffer[80];
//    int32_t current_d = __HAL_TIM_GET_COUNTER(&htim3);
//    int32_t current_g = __HAL_TIM_GET_COUNTER(&htim4);
//
//    *speed_d = current_d - last_encoder_count_d;
//    *speed_g = current_g - last_encoder_count_g;
//
//
//    // overflow
//
//    if (*speed_d > 32767) *speed_d -= 65536;
//    if (*speed_d < -32768) *speed_d += 65536;
//    if (*speed_g > 32767) *speed_g -= 65536;
//    if (*speed_g < -32768) *speed_g += 65536;
//
//    last_encoder_count_d = current_d;
//    last_encoder_count_g = current_g;
//
//    // Transmission UART
//    sprintf(buffer, "Speed moteur droit : %ld\r\n, Speed moteur gauche : %ld\r\n", speed_d, speed_g);
//    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//}
//
//
//void asservissement_double_PID(void) {
//    int32_t speed_d, speed_g;
//    read_speed(&speed_d, &speed_g);  // récupère les vitesses des 2 moteurs
//
//    // --- Moteur droit ---
//    float erreur_d = CONSIGNE_D - speed_d;
//    erreur_i_d += erreur_d;
//    float delta_e_d = erreur_d - erreur_precedente_d;
//    float cmd_d = KP * erreur_d + KI * erreur_i_d + KD * delta_e_d;
//    if (cmd_d > PWM_MAX) cmd_d = PWM_MAX;
//    if (cmd_d < PWM_MIN) cmd_d = PWM_MIN;
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)cmd_d);
//    erreur_precedente_d = erreur_d;
//
//    // --- Moteur gauche ---
//    float erreur_g = CONSIGNE_G - speed_g;
//    erreur_i_g += erreur_g;
//    float delta_e_g = erreur_g - erreur_precedente_g;
//    float cmd_g = KP * erreur_g + KI * erreur_i_g + KD * delta_e_g;
//    if (cmd_g > PWM_MAX) cmd_g = PWM_MAX;
//    if (cmd_g < PWM_MIN) cmd_g = PWM_MIN;
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)cmd_g);
//    erreur_precedente_g = erreur_g;
//}

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
