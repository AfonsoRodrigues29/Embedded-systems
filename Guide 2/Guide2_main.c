X/* USER CODE BEGIN Header */
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
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BUFFER_SIZE 32
#define MAX_PERIOD 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
unsigned int counter_period = 0;
unsigned int period = 0;
uint32_t time_period_elapsed = 0;
unsigned int divison = 0;
double period1 = 0;
double period2 = 0;
double helper = 0;
double pos = 0;
double rpm = 0;
double rpm_sampled = 0;
double pos_sampled = 0;
unsigned int flag_read_pos_vel = 0;
unsigned int flag_cr = 0;
unsigned int period_elapsed = 0;
unsigned int samples = 0;
unsigned int flag_enable = 1;
uint8_t rx_index = 0;
uint8_t rx_index1 = 0;
uint8_t rx_index2 = 0;
uint8_t state_input = 0;
uint32_t time_between_interrupts = 0;
int pulse = 0;
uint32_t elapsedtime = 0;
char rxbuff[MAX_BUFFER_SIZE];
char rxbuff1[5];
char rxbuff2[3];
char rxbuff3[5];
char rxbuff4[4];
typedef enum {
	STATE_RESET, STATE_CONFIG, STATE_MANUALCTR, STATE_OPENLOOP, STATE_AUTOMATIC
} State;

State lastState = STATE_RESET;
State currentState = STATE_RESET;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 5000);
	return ch;
}
int __io_getchar(void) {
	uint8_t ch;
	HAL_UART_Receive(&huart3, &ch, 1, 5000);
	return ch;
}
void error() {
	printf("Invalid\n");
}
void PWM() {

}
void print() {
	if (flag_cr == 1) {
		counter_period = period;
		divison = MAX_PERIOD / counter_period;
		printf("my counter period is %d\n", divison);
		if (time_period_elapsed == divison) {
			if (flag_read_pos_vel == 0) {
				printf("position is %f\n", pos);
			} else if (flag_read_pos_vel == 1) {
				printf("speed in rpm is %f\n", rpm);
			} else if (flag_read_pos_vel == 2) {
				printf(" position is %f and speed in rpm is %f\n", pos, rpm);
			}
		}
	}
}

void configuer_timer_period(){
	period1 = period / 1000;
	counter_period = (period1 / 0.001) - 1;
	htim17.Instance->ARR = counter_period - 1;
	HAL_TIM_Base_Start_IT(&htim17);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM16) {
		time_between_interrupts++;
	}
	if (htim->Instance == TIM17) {
		pos = (float) pulse / 960;
		period2 = period * 1000;
		helper = (60 * pulse) / period2;
		rpm = helper / 960;
		printf("entered\n");
		time_period_elapsed++;
		HAL_TIM_Base_Stop_IT(&htim17);
		HAL_TIM_Base_Start_IT(&htim17);

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	static uint8_t clickflag = 0;
	if (GPIO_Pin == GPIO_PIN_13) {
		if (clickflag == 0) {
			HAL_TIM_Base_Start_IT(&htim16);
			clickflag = 1;
		} else {
			HAL_TIM_Base_Stop_IT(&htim16);
			clickflag = 0;
			//printf(" interrupt time lasted %lu\n", time_between_interrupts);
			time_between_interrupts = 0;
		}
		if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_5) != GPIO_PIN_SET) { // Check pin G5
			pulse++;
			//printf("sentido horário\n");
		} else {
			pulse--;
			//printf("sentido anti horário\n");
		}

		// Clear the interrupt flag
		//__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_Pin);
	}
}
void read_enable() {
	memset(rxbuff1, 0, sizeof(rxbuff1));
	HAL_UART_Receive(&huart3, (uint8_t*) rxbuff1, 5, 10000);
	rxbuff1[5] = '\0';
	while (rxbuff1[0] == '\n') {
		// Move the data one position to the left to remove the newline character
		memmove(rxbuff1, rxbuff1 + 1, 5);
	}
}
void config_pwm() {
	memset(rxbuff2, 0, sizeof(rxbuff2));
	HAL_UART_Receive(&huart3, (uint8_t*) rxbuff2, 3, 10000);
	rxbuff2[3] = '\0';
	while (rxbuff2[0] == '\n') {
		// Move the data one position to the left to remove the newline character
		memmove(rxbuff2, rxbuff2 + 1, 3);
	}
	if (strcmp(rxbuff2, "HW") == 0) {
		printf("we entererd\n");
		scanf("%d", &period);
		period1 = period;
		printf("your period is %d\n", period);
	} else
		error();
}
void config_refresh() {
	memset(rxbuff3, 0, sizeof(rxbuff3));
	HAL_UART_Receive(&huart3, (uint8_t*) rxbuff3, 5, 10000);
	rxbuff3[5] = '\0';
	while (rxbuff3[0] == '\n') {
		// Move the data one position to the left to remove the newline character
		memmove(rxbuff3, rxbuff3 + 1, 5);
	}
	if (strcmp(rxbuff3, "CR 0") == 0) {
		printf("we entererd CR 0\n");
	} else if (strcmp(rxbuff3, "CR 1") == 0) {
		printf("we entererd CR 1\n");
		flag_cr = 1;
	} else if (strcmp(rxbuff3, "CR 2") == 0) {
		printf("we entererd CR 2\n");
	} else
		error();

}
void config_print() {
	memset(rxbuff4, 0, sizeof(rxbuff4));
	HAL_UART_Receive(&huart3, (uint8_t*) rxbuff4, 4, 10000);
	rxbuff4[4] = '\0';
	while (rxbuff4[0] == '\n') {
		// Move the data one position to the left to remove the newline character
		memmove(rxbuff4, rxbuff4 + 1, 4);
	}
	if (strcmp(rxbuff4, "L 0") == 0) {
		printf("we entererd L 0\n");
		flag_read_pos_vel = 0;
	} else if (strcmp(rxbuff4, "L 1") == 0) {
		printf("we entererd L 1\n");
		flag_read_pos_vel = 1;
	} else if (strcmp(rxbuff4, "L 2") == 0) {
		printf("we entererd L 2\n");
		flag_read_pos_vel = 2;
	} else
		error();

}
void reset_force() {

}
void force() {

}
void statemachine(State currentState, State lastState) {
	switch (currentState) {
	case STATE_RESET:
		printf("we are in state reset\n");
		break;
	case STATE_CONFIG:
		switch (lastState) {
		case STATE_RESET:
			printf("last state was reset state\n");
			printf("we are in state config\n");
			config_pwm();
			config_refresh();
			config_print();
			break;
		case STATE_CONFIG:
			printf("last state was config state\n");
			printf("we are in state config\n");
			config_pwm();
			config_refresh();
			config_print();
			break;
		default:
			error();
			break;
		}
		break;
	case STATE_MANUALCTR:
		switch (lastState) {
		case STATE_CONFIG:
			printf("last state was config state\n");
			printf("we are in state manual control\n");
			configuer_timer_period();
			read_enable();
			if (strcmp(rxbuff1, "EN 0") == 0) {
				flag_enable = 0;
				printf("no enable\n");
			} else if (strcmp(rxbuff1, "EN 1") == 0) {
				flag_enable = 1;
				printf("we have enable\n");
				PWM();
				print();
			} else {
				flag_enable = 1;
				printf("we have enable\n");
				PWM();
				print();
			}
			break;
		case STATE_OPENLOOP:
			printf("last state was openloop state\n");
			printf("we are in state manual control\n");
			read_enable();
			if (strcmp(rxbuff1, "EN 0") == 0) {
				printf("no enable\n");
			} else if (strcmp(rxbuff1, "EN 1") == 0) {
				printf("we have enable\n");
				PWM();
				configuer_timer_period();
				print();
			}
		case STATE_MANUALCTR:
			printf("last state was manual control state\n");
			printf("we are in state manual control\n");
			read_enable();
			if (strcmp(rxbuff1, "EN 0") == 0) {
				printf("no enable\n");
			} else if (strcmp(rxbuff1, "EN 1") == 0) {
				printf("we have enable\n");
				PWM();
				configuer_timer_period();
				print();
			}
			break;
		default:
			error();
			break;
		}
		break;
	case STATE_OPENLOOP:
		switch (lastState) {
		case STATE_CONFIG:
			printf("last state was config state\n");
			printf("we are in state open loop\n");
			read_enable();
			if (strcmp(rxbuff1, "EN 0") == 0) {
				printf("no enable\n");
				reset_force();
			} else if (strcmp(rxbuff1, "EN 1") == 0) {
				printf("we have enable\n");
				force();
				configuer_timer_period();
				print();
			}
			break;
		case STATE_MANUALCTR:
			printf("last state was manual control\n");
			printf("we are in state open loop\n");
			read_enable();
			if (strcmp(rxbuff1, "EN 0") == 0) {
				printf("no enable\n");
				reset_force();
			} else if (strcmp(rxbuff1, "EN 1") == 0) {
				printf("we have enable\n");
				force();
				configuer_timer_period();
				print();
			}
			break;
		case STATE_OPENLOOP:
			printf("last state was open loop\n");
			printf("we are in state open loop\n");
			read_enable();
			if (strcmp(rxbuff1, "EN 0") == 0) {
				printf("no enable\n");
				reset_force();
			} else if (strcmp(rxbuff1, "EN 1") == 0) {
				printf("we have enable\n");
				force();
				configuer_timer_period();
				print();
			}
			break;
		default:
			error();
			break;
		}
		break;
	case STATE_AUTOMATIC:
		switch (lastState) {
		case STATE_CONFIG:
			printf("last state was config state\n");
			printf("we are in automatic state\n");
			read_enable();
			if (strcmp(rxbuff1, "EN 0") == 0) {
				printf("no enable\n");
				reset_force();
			} else if (strcmp(rxbuff1, "EN 1") == 0) {
				printf("we have enable\n");
				force();
				configuer_timer_period();
				print();
			}
			break;
		case STATE_AUTOMATIC:
			printf("last state was automatic state\n");
			printf("we are in automatic state\n");
			read_enable();
			if (strcmp(rxbuff1, "EN 0") == 0) {
				printf("no enable\n");
				reset_force();
			} else if (strcmp(rxbuff1, "EN 1") == 0) {
				printf("we have enable\n");
				force();
				configuer_timer_period();
				print();
			}
			break;

		default:
			error();
			break;
		}
		break;
	}
}
void start() {
	stdin = stdout = stderr;
	int ch;
	printf(">");
//Commands command1;
	while ((ch = getchar()) != '\n' && rx_index < MAX_BUFFER_SIZE - 1) {
		if (ch != EOF) {
			rxbuff[rx_index++] = (uint8_t) ch;
		}
	}
// Process the input only if it's not empty
	if (rx_index > 0) {
		state_input = rxbuff[rx_index - 1] - '0';
		printf("CS %d\n", state_input);
		lastState = currentState;
		//command1.mode=state_input;
		// Check if the input is valid and set the current state accordingly
		if (state_input >= 0 && state_input <= 4) {
			switch (state_input) {
			case 0:
				currentState = STATE_RESET;
				break;
			case 1:
				currentState = STATE_CONFIG;
				break;
			case 2:
				currentState = STATE_MANUALCTR;
				break;
			case 3:
				currentState = STATE_OPENLOOP;
				break;
			case 4:
				currentState = STATE_AUTOMATIC;
				break;
			}
			statemachine(currentState, lastState);
		} else {
			printf("Invalid input. No state change.\n");
		}
		// Reset the buffer and index for the next input
		rx_index = 0;
		memset(rxbuff, 0, sizeof(rxbuff));
		HAL_UART_AbortReceive(&huart3);
	}
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
  MX_ADC3_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 10, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		start();
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 64-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 64000-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PG5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
	while (1) {
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
