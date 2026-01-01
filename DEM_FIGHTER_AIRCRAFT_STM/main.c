/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "lis2dw12_reg.h"  // IMU Driver
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * @brief Flight Control Data Structure
 * Encapsulates the state of all aircraft controls to be transmitted via telemetry.
 */
typedef struct {
	int8_t pitch_cmd; /*!< 0: Neutral, 1: Nose Down (W), 2: Nose Up (S) */
	int8_t roll_cmd; /*!< 0: Neutral, 1: Bank Left (A), 2: Bank Right (D) */
	int8_t yaw_cmd; /*!< 0: Neutral, 1: Rudder Left (Q), 2: Rudder Right (E) */
	int8_t throttle_cmd; /*!< 0: Hold, 1: Increase (Shift), 2: Decrease (Ctrl) */
	uint8_t btn_space; /*!< 1: Active, 0: Inactive */
	uint8_t btn_esc; /*!< 1: Active, 0: Inactive */
	uint8_t btn_respawn; /*!< 1: Active, 0: Inactive */
	uint8_t btn_enter; /*!< 1: Active, 0: Inactive */
} FlightControlState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* * I2C Address: 0x19 (Standard) shifted left by 1 for LL API.
 * If sensor doesn't work, try (0x18U << 1).
 */
#define SENSOR_I2C_ADDR  (0x19U << 1)
#define I2C_TIMEOUT      10000        // Timeout counter to prevent freezing
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* Definitions for UARTTask */
osThreadId_t UARTTaskHandle;
const osThreadAttr_t UARTTask_attributes = {
  .name = "UARTTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
FlightControlState_t flightControlState; /* Global State to be sent to ESP32 */
stmdev_ctx_t imu_ctx; /* IMU Driver Context */
static int16_t imu_raw_data[3]; /* Raw Acceleration Data */
static uint8_t sensor_id; /* Chip ID for verification */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void StartUARTTask(void *argument);
void StartSensorTask(void *argument);

/* USER CODE BEGIN PFP */
/* Safe Low-Layer Driver Wrappers */
int32_t Platform_I2C_Write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len);
int32_t Platform_I2C_Read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);
uint16_t Platform_ADC_Read(ADC_TypeDef *ADCx, uint32_t Channel);

/* Helper */
int Fast_IntToStr(char *buf, int value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Robust I2C Write with Timeout.
 * Returns -1 if hardware freezes, preventing system lockup.
 */
int32_t Platform_I2C_Write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len) {
	I2C_TypeDef *I2Cx = (I2C_TypeDef*) handle;
	uint32_t timeout = I2C_TIMEOUT;

	LL_I2C_HandleTransfer(I2Cx, SENSOR_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, len + 1,
			LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

	while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {
		if (timeout-- == 0)
			return -1;
	}
	LL_I2C_TransmitData8(I2Cx, reg);

	for (uint16_t i = 0; i < len; i++) {
		timeout = I2C_TIMEOUT;
		while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {
			if (timeout-- == 0)
				return -1;
		}
		LL_I2C_TransmitData8(I2Cx, bufp[i]);
	}

	timeout = I2C_TIMEOUT;
	while (!LL_I2C_IsActiveFlag_STOP(I2Cx)) {
		if (timeout-- == 0)
			return -1;
	}
	LL_I2C_ClearFlag_STOP(I2Cx);
	return 0;
}

/**
 * @brief Robust I2C Read with Timeout.
 */
int32_t Platform_I2C_Read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	I2C_TypeDef *I2Cx = (I2C_TypeDef*) handle;
	uint32_t timeout = I2C_TIMEOUT;

	LL_I2C_HandleTransfer(I2Cx, SENSOR_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, 1,
			LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);

	while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {
		if (timeout-- == 0)
			return -1;
	}
	LL_I2C_TransmitData8(I2Cx, reg);

	while (!LL_I2C_IsActiveFlag_TC(I2Cx)) {
		if (timeout-- == 0)
			return -1;
	}

	LL_I2C_HandleTransfer(I2Cx, SENSOR_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, len,
			LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

	for (uint16_t i = 0; i < len; i++) {
		timeout = I2C_TIMEOUT;
		while (!LL_I2C_IsActiveFlag_RXNE(I2Cx)) {
			if (timeout-- == 0)
				return -1;
		}
		bufp[i] = LL_I2C_ReceiveData8(I2Cx);
	}

	timeout = I2C_TIMEOUT;
	while (!LL_I2C_IsActiveFlag_STOP(I2Cx)) {
		if (timeout-- == 0)
			return -1;
	}
	LL_I2C_ClearFlag_STOP(I2Cx);
	return 0;
}

/**
 * @brief Simple Polling ADC Read (Replaces Complex DMA).
 */
uint16_t Platform_ADC_Read(ADC_TypeDef *ADCx, uint32_t Channel) {
	/* Select Channel */
	ADCx->CHSELR = Channel;

	/* Start & Wait */
	LL_ADC_REG_StartConversion(ADCx);
	uint32_t timeout = 5000;
	while (!LL_ADC_IsActiveFlag_EOC(ADCx)) {
		if (timeout-- == 0)
			return 0;
	}
	return LL_ADC_REG_ReadConversionData12(ADCx);
}

/* Fast String Converter */
int Fast_IntToStr(char *buf, int value) {
	char *p = buf;
	if (value < 0) {
		*p++ = '-';
		value = -value;
	}
	char *first = p;
	do {
		int digit = value % 10;
		*p++ = (char) (digit + '0');
		value /= 10;
	} while (value > 0);
	char *last = p - 1;
	while (first < last) {
		char temp = *first;
		*first++ = *last;
		*last-- = temp;
	}
	return (int) (p - buf);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UARTTask */
  UARTTaskHandle = osThreadNew(StartUARTTask, NULL, &UARTTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
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

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  /**ADC1 GPIO Configuration
  PA7   ------> ADC1_IN7
  PB0   ------> ADC1_IN8
  */
  GPIO_InitStruct.Pin = YAW_ADC_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(YAW_ADC_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = THRUST_ADC_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(THRUST_ADC_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */

   #define ADC_CHANNEL_CONF_RDY_TIMEOUT_MS ( 1U)
   #if (USE_TIMEOUT == 1)
   uint32_t Timeout ; /* Variable used for Timeout management */
   #endif /* USE_TIMEOUT */

  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  LL_ADC_REG_SetSequencerConfigurable(ADC1, LL_ADC_REG_SEQ_CONFIGURABLE);

   /* Poll for ADC channel configuration ready */
   #if (USE_TIMEOUT == 1)
   Timeout = ADC_CHANNEL_CONF_RDY_TIMEOUT_MS;
   #endif /* USE_TIMEOUT */
   while (LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0)
     {
   #if (USE_TIMEOUT == 1)
   /* Check Systick counter flag to decrement the time-out value */
   if (LL_SYSTICK_IsActiveCounterFlag())
     {
   if(Timeout-- == 0)
         {
   Error_Handler();
         }
     }
   #endif /* USE_TIMEOUT */
     }
   /* Clear flag ADC channel configuration ready */
   LL_ADC_ClearFlag_CCRDY(ADC1);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  LL_ADC_SetTriggerFrequencyMode(ADC1, LL_ADC_CLOCK_FREQ_MODE_HIGH);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1, LL_ADC_SAMPLINGTIME_19CYCLES_5);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_2, LL_ADC_SAMPLINGTIME_39CYCLES_5);
  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);

   /* Enable ADC internal voltage regulator */
   LL_ADC_EnableInternalRegulator(ADC1);
   /* Delay for ADC internal voltage regulator stabilization. */
   /* Compute number of CPU cycles to wait for, from delay in us. */
   /* Note: Variable divided by 2 to compensate partially */
   /* CPU processing cycles (depends on compilation optimization). */
   /* Note: If system core clock frequency is below 200kHz, wait time */
   /* is only a few CPU processing cycles. */
   __IO uint32_t wait_loop_index;
   wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
   while(wait_loop_index != 0)
     {
   wait_loop_index--;
     }

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_7);

   /* Poll for ADC channel configuration ready */
   #if (USE_TIMEOUT == 1)
   Timeout = ADC_CHANNEL_CONF_RDY_TIMEOUT_MS;
   #endif /* USE_TIMEOUT */
   while (LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0)
     {
   #if (USE_TIMEOUT == 1)
   /* Check Systick counter flag to decrement the time-out value */
   if (LL_SYSTICK_IsActiveCounterFlag())
     {
   if(Timeout-- == 0)
         {
   Error_Handler();
         }
     }
   #endif /* USE_TIMEOUT */
     }
   /* Clear flag ADC channel configuration ready */
   LL_ADC_ClearFlag_CCRDY(ADC1);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_COMMON_1);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_8);

   /* Poll for ADC channel configuration ready */
   #if (USE_TIMEOUT == 1)
   Timeout = ADC_CHANNEL_CONF_RDY_TIMEOUT_MS;
   #endif /* USE_TIMEOUT */
   while (LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0)
     {
   #if (USE_TIMEOUT == 1)
   /* Check Systick counter flag to decrement the time-out value */
   if (LL_SYSTICK_IsActiveCounterFlag())
     {
   if(Timeout-- == 0)
         {
   Error_Handler();
         }
     }
   #endif /* USE_TIMEOUT */
     }
   /* Clear flag ADC channel configuration ready */
   LL_ADC_ClearFlag_CCRDY(ADC1);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_COMMON_1);
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

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = IMU_SCL_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(IMU_SCL_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = IMU_SDA_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(IMU_SDA_GPIO_Port, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x00C12166;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = ESP_TX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(ESP_TX_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ESP_RC_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(ESP_RC_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);

  /* USER CODE BEGIN WKUPType USART2 */

  /* USER CODE END WKUPType USART2 */

  LL_USART_Enable(USART2);

  /* Polling USART2 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
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
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOF);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin);

  /**/
  GPIO_InitStruct.Pin = MISSILE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(MISSILE_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = ESC_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(ESC_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = ENTER_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(ENTER_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = RESPAWN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(RESPAWN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUARTTask */
/**
 * @brief  Telemetry Transmission Task.
 * - Priority: Normal
 * - Frequency: 20Hz (50ms)
 * - Mode: Continuous Stream (Snapshot)
 * - Responsibilities: Serialize state data and transmit via UART.
 */
/* USER CODE END Header_StartUARTTask */
void StartUARTTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	char tx_buffer[64];
	char *ptr;

	/* Infinite loop */
	for (;;) {
		/* 1. Build Packet: $P,R,Y,T,M,E,ENT,R# */
		ptr = tx_buffer;
		*ptr++ = '$';
		ptr += Fast_IntToStr(ptr, flightControlState.pitch_cmd);
		*ptr++ = ',';
		ptr += Fast_IntToStr(ptr, flightControlState.roll_cmd);
		*ptr++ = ',';
		ptr += Fast_IntToStr(ptr, flightControlState.yaw_cmd);
		*ptr++ = ',';
		ptr += Fast_IntToStr(ptr, flightControlState.throttle_cmd);
		*ptr++ = ',';
		ptr += Fast_IntToStr(ptr, flightControlState.btn_space);
		*ptr++ = ',';
		ptr += Fast_IntToStr(ptr, flightControlState.btn_esc);
		*ptr++ = ',';
		ptr += Fast_IntToStr(ptr, flightControlState.btn_enter);
		*ptr++ = ',';
		ptr += Fast_IntToStr(ptr, flightControlState.btn_respawn);
		*ptr++ = '#';
		*ptr++ = '\n';
		*ptr = 0;

		/* 2. Transmit to ESP32 (USART2) */
		char *send_ptr = tx_buffer;
		while (*send_ptr) {
			while (!LL_USART_IsActiveFlag_TXE_TXFNF(USART2))
				;
			LL_USART_TransmitData8(USART2, *send_ptr++);
		}

		/* 3. Maintain Rate (20Hz) */
		osDelay(50);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
 * @brief  Primary Sensor Acquisition Task.
 * - Priority: High (AboveNormal)
 * - Frequency: 50Hz (20ms)
 * - Responsibilities:
 * 1. Trigger ADC via DMA (Normal Mode).
 * 2. Poll IMU Data via I2C.
 * 3. Poll GPIO Buttons (Software Debounce).
 * 4. Update Global Flight Control State.
 */
void StartSensorTask(void *argument) {
	/* --- 1. Driver Initialization --- */
	imu_ctx.write_reg = Platform_I2C_Write;
	imu_ctx.read_reg = Platform_I2C_Read;
	imu_ctx.handle = I2C1;

	/* --- B. IMU Setup (Robust) --- */
	// Check ID to confirm connection
	lis2dw12_device_id_get(&imu_ctx, &sensor_id);

	// Settings from Reference Project (2g, 25Hz, HighPerformance)
	lis2dw12_reset_set(&imu_ctx, PROPERTY_ENABLE);
	osDelay(20);
	lis2dw12_block_data_update_set(&imu_ctx, PROPERTY_ENABLE);
	lis2dw12_full_scale_set(&imu_ctx, LIS2DW12_2g);
	lis2dw12_filter_path_set(&imu_ctx, LIS2DW12_LPF_ON_OUT);
	lis2dw12_filter_bandwidth_set(&imu_ctx, LIS2DW12_ODR_DIV_4);
	lis2dw12_power_mode_set(&imu_ctx, LIS2DW12_HIGH_PERFORMANCE);
	lis2dw12_data_rate_set(&imu_ctx, LIS2DW12_XL_ODR_25Hz);

	/* --- C. ADC Calibration --- */
	if (!LL_ADC_IsEnabled(ADC1)) {
		LL_ADC_EnableInternalRegulator(ADC1);
		osDelay(25);
		LL_ADC_StartCalibration(ADC1);
		while (LL_ADC_IsCalibrationOnGoing(ADC1))
			;
		LL_ADC_Enable(ADC1);
		while (!LL_ADC_IsActiveFlag_ADRDY(ADC1))
			;
	}

	/* --- D. Main Loop --- */
	for (;;) {
		/* 1. Read IMU (Polling) */
		uint8_t drdy = 0;
		lis2dw12_flag_data_ready_get(&imu_ctx, &drdy);

		if (drdy) {
			memset(imu_raw_data, 0, sizeof(imu_raw_data));
			if (lis2dw12_acceleration_raw_get(&imu_ctx, imu_raw_data) == 0) {

				// Convert to mg
				float acc_x = lis2dw12_from_fs2_to_mg(imu_raw_data[0]);
				float acc_y = lis2dw12_from_fs2_to_mg(imu_raw_data[1]);

				// Logic: Threshold 300mg
				// Pitch (Y)
				if (acc_y > 300.0f)
					flightControlState.roll_cmd = 1;
				else if (acc_y < -300.0f)
					flightControlState.roll_cmd = 2;
				else
					flightControlState.roll_cmd = 0;

				// Roll (X)
				if (acc_x > 300.0f)
					flightControlState.pitch_cmd = 2;
				else if (acc_x < -300.0f)
					flightControlState.pitch_cmd = 1;
				else
					flightControlState.pitch_cmd = 0;
			}
		}

		/* 2. Read ADC (Yaw & Throttle) */
		// Yaw -> Ch7, Throttle -> Ch8
		uint16_t val_yaw = Platform_ADC_Read(ADC1, LL_ADC_CHANNEL_7);
		if (val_yaw < 1000)
			flightControlState.yaw_cmd = 1;
		else if (val_yaw > 3000)
			flightControlState.yaw_cmd = 2;
		else
			flightControlState.yaw_cmd = 0;

		uint16_t val_thr = Platform_ADC_Read(ADC1, LL_ADC_CHANNEL_8);
		if (val_thr > 3000)
			flightControlState.throttle_cmd = 1;
		else if (val_thr < 1000)
			flightControlState.throttle_cmd = 2;
		else
			flightControlState.throttle_cmd = 0;

		/* 3. Read Buttons (Inverted Logic) */
		flightControlState.btn_space = !LL_GPIO_IsInputPinSet(GPIOB,
				LL_GPIO_PIN_1);
		flightControlState.btn_esc = !LL_GPIO_IsInputPinSet(GPIOB,
				LL_GPIO_PIN_2);
		flightControlState.btn_respawn = !LL_GPIO_IsInputPinSet(GPIOB,
				LL_GPIO_PIN_10);
		flightControlState.btn_enter = !LL_GPIO_IsInputPinSet(GPIOB,
				LL_GPIO_PIN_11);

		/* 4. Yield (Important!) */
		osDelay(40); // 25Hz loop
	}
  /* USER CODE END StartSensorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM16)
  {
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
	while (1) {
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
