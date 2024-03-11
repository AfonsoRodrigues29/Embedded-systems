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
#define MEM_SIZE 0x100
#define RX_BUFFER_SIZE 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
char *tok;
char *cmd[10];
char *comando;
char rxbuff[32];
int contador = 0;
uint8_t memory[MEM_SIZE]={0};
uint16_t gpio_pins[]={GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6,GPIO_PIN_7,GPIO_PIN_8,GPIO_PIN_9,GPIO_PIN_10,GPIO_PIN_11,GPIO_PIN_12,GPIO_PIN_13,GPIO_PIN_14,GPIO_PIN_15};
uint32_t gpio_mode[]={GPIO_MODE_INPUT,GPIO_MODE_OUTPUT_PP,GPIO_MODE_ANALOG};
uint32_t channels[] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4,ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_9};
uint16_t adc_gpio_pins[]={GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_9, GPIO_PIN_7,GPIO_PIN_5 ,GPIO_PIN_3,GPIO_PIN_10,GPIO_PIN_8,GPIO_PIN_6,GPIO_PIN_4,};
uint8_t interrupt_flag=0;
uint8_t tx_complete_flag=0;
uint8_t rx_complete_flag=0;
GPIO_TypeDef* adc_gpio_ports[]={GPIOC, GPIOC, GPIOF, GPIOF,GPIOF,GPIOF,GPIOF,GPIOF,GPIOF,GPIOF};
GPIO_TypeDef* gpio_ports[]={GPIOA,GPIOB,GPIOC,GPIOD};
ADC_HandleTypeDef *ADC=&hadc3;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_BDMA_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
   interrupt_flag=1;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        rx_complete_flag = 1;
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        tx_complete_flag = 1;
    }
}

int __io_putchar(int ch) {
    if (tx_complete_flag) {
        tx_complete_flag = 0;
        HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&ch, 1);
        return ch;
    }
    return EOF;
}
int __io_getchar(void) {
    uint8_t ch;
    if(rx_complete_flag){
    	rx_complete_flag=0;
    	HAL_UART_Receive_DMA(&huart3, &ch, 1); // Start new reception using DMA
    	return ch;
    }
    return EOF;
}

int  memory_read(unsigned int  address , unsigned int lenght ){
	uint8_t value;
	if(address+lenght >= MEM_SIZE){
		return -1;
	}
	for(unsigned int i=0;i<lenght;i++){
			value=memory[address+i];
			printf("address :%x content :%x\n",(address+i),value);

	}
	return 0;
}
int  memory_write(unsigned int  address , unsigned int lenght, unsigned int byte ){
	if(address+lenght >= MEM_SIZE){
		return -1;
	}
	for(unsigned int i=0;i<lenght;i++){
			memory[address+i]=byte;
			printf("address :%x content :%x\n ",(address+i),memory[address+i]);

	}
	return 0;
}
int read_data(unsigned int portAddr, unsigned int pin){
	GPIO_PinState value;
	if ((portAddr-10)>=4 || pin>=16){
		return -1;
	}
	value = HAL_GPIO_ReadPin(gpio_ports[portAddr-10],gpio_pins[pin]);
	printf("%d",value);
	return 0;
}
int write_data(unsigned int portAddr, unsigned int pin, unsigned int pinValue){


	if ((portAddr-10)>=4 || pin>=16 || pinValue >=2){
		return -1;
	}
	HAL_GPIO_WritePin(gpio_ports[portAddr-10],gpio_pins[pin],pinValue);

	return 0;
}
int analog_read(){
	if(interrupt_flag == 0){
		return -1;
		}
	uint32_t Data = HAL_ADC_GetValue(&hadc3);
	float voltage_mV = ((float)Data * 3.3) / 65535.0f; // Assuming 12-bit ADC
	printf("Voltage Value is %.2f\n", voltage_mV); // Assuming floating-point output is desired
	interrupt_flag=0;
	return 0;
}
int gpio_define(unsigned int mode,unsigned int portAddr, unsigned int pin){

	if ((portAddr-10)>=4 || pin>=16 || mode >=2){
			return -1;
		}
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin=gpio_pins[pin];
	GPIO_InitStruct.Mode=gpio_mode[mode];
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(gpio_ports[portAddr-10], &GPIO_InitStruct);
	return 0;
}
/*int gpio_adc_define(unsigned int mode,unsigned int portAddr, unsigned int pin){

	if (portAddr>=10 || pin>=10 || mode != 3){
			return -1;
		}
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin=adc_gpio_pins[pin];
	GPIO_InitStruct.Mode=gpio_mode[mode];
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(adc_gpio_ports[portAddr], &GPIO_InitStruct);
	return 0;
}*/


int adc_define(unsigned int addr3){
	if(addr3>=10){
		return -1;
	}
	/*gpio_adc_define(3,addr3,addr3);*/
	ADC_ChannelConfTypeDef ADCConfig={0};
	ADCConfig.Channel = channels[addr3];
	ADCConfig.Rank = ADC_REGULAR_RANK_1;
	ADCConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
	ADCConfig.SingleDiff = ADC_SINGLE_ENDED;
	ADCConfig.OffsetNumber = ADC_OFFSET_NONE;
	ADCConfig.Offset = 0;
	//ADCConfig.OffsetSignedSaturation = 0;
	HAL_ADC_ConfigChannel(&hadc3, &ADCConfig);
	HAL_ADC_Start_IT(&hadc3);
	HAL_Delay(10);
	return 0;
}
void getstring()
{
	stdin = stdout = stderr;
	fflush(stdout);
	int ch;
	uint8_t rx_index=0;
	printf(">");
	while ((ch = getchar()) != '\n' && rx_index < 32)
	{
		if (ch != EOF){
			rxbuff[rx_index]=(uint8_t)ch;
			rx_index++;
		}
	}
	if(rx_index == 32){
		rxbuff[rx_index-1] = '\0';
	}
	rxbuff[rx_index] = '\0';
	printf("%s\n",rxbuff);
}

void clrphr(){
	for (int j = 0; j < sizeof(rxbuff); j++) {
	    rxbuff[j] = '\0';
	}
	contador=0;
}

void error(char *comando)
{
    printf("The command %s does not have that parameters", comando);
}
void lexico()
{
    tok = strtok(rxbuff, " ");
    while (tok != NULL)
    {
        cmd[contador] = tok;
        contador++;
        tok = strtok(NULL, " ");
    }
    contador--;
}
void sintatico()
{
    unsigned int addr3;
    unsigned int address;
    unsigned int length;
    unsigned int byte;
    unsigned int portAddr;
    unsigned int pin;
    unsigned int pinValue;

    switch(contador){

    case (0):
        comando = cmd[0];
        printf("\n command: %s", comando);
    break;

    case(1):
        comando = cmd[0];
        if (strcmp(comando, "RA") == 0)
        {
        	addr3 = strtoul(cmd[1], NULL, 16);
            if(adc_define(addr3)== -1){
            	error(comando);
            }
            if(analog_read()== -1){
                        	error(comando);
            }

        }
        else
        {
            error(comando);
        }
    break;

    case(2):
        comando = cmd[0];
        if (strcmp(comando, "MR") == 0)
        {
            address = strtoul(cmd[1], NULL, 16);
            length = strtoul(cmd[2], NULL, 16);
            if(memory_read(address,length)==-1){
            	error(comando);
            }
        }
        else if (strcmp(comando, "PI") == 0)
        {
            portAddr = strtoul(cmd[1], NULL, 16);
            pin = strtoul(cmd[2], NULL, 16);
            if(gpio_define(0, portAddr, pin)==-1){
                       	error(comando);
                       }
           printf("done with sucess\n");
        }

        else if (strcmp(comando, "PO") == 0)
        {
            portAddr = strtoul(cmd[1], NULL, 16);
            pin = strtoul(cmd[2], NULL, 16);
            if(gpio_define(1, portAddr, pin)==-1){
                                   	error(comando);
                                   }
            printf("done with sucess\n");
        }

        else if (strcmp(comando, "RD") == 0)
        {
            portAddr = strtoul(cmd[1], NULL, 16);
            pin = strtoul(cmd[2], NULL, 16);
            if(read_data(portAddr,pin)==-1){
                        	error(comando);
                        }
        }
        else
        {
            error(comando);
        }
    break;

    case(3):
        comando = cmd[0];
        if (strcmp(comando, "MW") == 0)
        {
        	address = strtoul(cmd[1], NULL, 16);
        	length = strtoul(cmd[2], NULL, 16);
        	byte = strtoul(cmd[3], NULL, 16);
        	if(memory_write(address,length,byte)==-1){
        	            	error(comando);
        	            }

        }

        else if (strcmp(comando, "WD") == 0)
        {
            portAddr = strtoul(cmd[1], NULL, 16);
            pin = strtoul(cmd[2], NULL, 16);
            pinValue = strtoul(cmd[3], NULL, 16);
            if(write_data(portAddr,pin,pinValue)==-1) {
            	error(comando);
            	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
            }
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
        }
        else
        {
            error(comando);
        }
    break;

    case(4):
    {
       error(comando);
    }
    break;
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
  MX_BDMA_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  getstring();
	  lexico();
	  sintatico();
	  clrphr();
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
  * Enable DMA controller clock
  */
static void MX_BDMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_BDMA_CLK_ENABLE();

  /* DMA interrupt init */
  /* BDMA_Channel0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BDMA_Channel0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(BDMA_Channel0_IRQn);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX1_OVR_IRQn);



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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

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
