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
#include "tusb.h"
#include "event_groups.h"
#include <limits.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USBD_STACK_SIZE    (3*configMINIMAL_STACK_SIZE/2) * (CFG_TUSB_DEBUG ? 2 : 1)
#define ARPEGGIATOR_CONTROL_FLAG (1 << 1)

#define JOY_UP_BIT    0x01
#define JOY_DOWN_BIT  0x02
#define JOY_LEFT_BIT  0x04
#define JOY_RIGHT_BIT 0x08

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
EventGroupHandle_t xArpEventGroup;

/* Definitions for USB Device Task */
StackType_t  usb_device_stack[USBD_STACK_SIZE];
StaticTask_t usb_device_taskdef;

/* This doesn't work */
osThreadId_t usbDeviceTaskHandle;
const osThreadAttr_t usbDeviceTask_attributes = {
    .name = "usbd",
		.stack_size = USBD_STACK_SIZE,
    .priority = (osPriority_t) configMAX_PRIORITIES-1,  // Adjust the priority as needed
};

/* maybe i should just use FreeRTOS */
osThreadId_t arpTaskHandle;
const osThreadAttr_t arpTask_attributes = {
    .name = "arp",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,  // Adjust the priority as needed
};

osThreadId_t testTaskHandle;
const osThreadAttr_t testTask_attributes = {
    .name = "test",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,  // Adjust the priority as needed
};

//typedef struct {
//	uint32_t speeds[3];
//	uint8_t current_speed;
//	uint8_t *sequences[3];
//	uint8_t current_sequence;
//
//} ArpState;

typedef struct {
	uint32_t speeds[3];
	uint8_t current_speed;
	uint8_t *sequences[3];
	uint8_t current_sequence;
	uint32_t note_pos;
	uint8_t running;

} ArpState;

static uint8_t note_sequence_1[] =
{
  74,78,81,86,90,93,98,102,57,61,66,69,73,78,81,85,88,92,97,100,97,92,88,85,81,78,
  74,69,66,62,57,62,66,69,74,78,81,86,90,93,97,102,97,93,90,85,81,78,73,68,64,61,
  56,61,64,68,74,78,81,86,90,93,98,102
};
static uint8_t note_sequence_2[] =
{
  44,45,46,47,46,45,48,49,50,46,47,48,55,54,53,52,51,50,53,52,50
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void UsbDeviceTask(void *param);
void MidiTask(void *param);
void vTimerMidiCallback(TimerHandle_t xTimer);

void initArpState(ArpState *state);
void stepArpSpeed(ArpState *state);
void stepArpSequence(ArpState *state);
void stepArpNote(ArpState *state);
uint32_t getArpSpeed(ArpState* state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void initArpState(ArpState *state) {
    state->speeds[0] = 400;
    state->speeds[1] = 200;
    state->speeds[2] = 100;

    state->current_speed = 0;

    state->sequences[0] = note_sequence_1;
    state->sequences[1] = note_sequence_2;
    state->sequences[2] = note_sequence_1;

    state->current_sequence = 0;

    state->note_pos = 0;

    state->running = 0;
}

void stepArpSpeed(ArpState *state) {
	state->current_speed = (state->current_speed + 1) % 3;
}

void stepArpSequence(ArpState *state) {
	state->current_sequence = (state->current_sequence + 1) % 3;
	state->note_pos = 0;
}

void stepArpNote(ArpState *state) {
	//TODO sequences must also store their size!
	state->note_pos = (state->note_pos + 1) % 8;
}

uint32_t getArpSpeed(ArpState* state) {
	return state->speeds[state->current_speed];
}

uint8_t *getArpSequence(ArpState* state) {
	return state->sequences[state->current_sequence];
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
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  tusb_init();
  // turn off leds except 4
  HAL_GPIO_WritePin(GPIOE, LED3_Pin|LED1_Pin|LED2_Pin, 1);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
//  usbDeviceTaskHandle = osThreadNew(UsbDeviceTask, NULL, &usbDeviceTask_attributes);
  xTaskCreateStatic(UsbDeviceTask, "usbd", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES-1, usb_device_stack, &usb_device_taskdef);
  arpTaskHandle = osThreadNew(MidiTask, NULL, &arpTask_attributes);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
//  xArpEventGroup = xEventGroupCreate();
//  if( xArpEventGroup == NULL ) return 0;

  /* USER CODE END RTOS_EVENTS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED3_Pin|LED4_Pin|LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LCD_BLCTRL_Pin|EXT_RESET_Pin|CTP_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTGFS_PPWR_EN_GPIO_Port, USB_OTGFS_PPWR_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED3_Pin LED4_Pin LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED4_Pin|LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : DFSDM_DATIN3_Pin */
  GPIO_InitStruct.Pin = DFSDM_DATIN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_DFSDM1;
  HAL_GPIO_Init(DFSDM_DATIN3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : A0_Pin */
  GPIO_InitStruct.Pin = A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
  HAL_GPIO_Init(A0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_BLCTRL_Pin EXT_RESET_Pin CTP_RST_Pin */
  GPIO_InitStruct.Pin = LCD_BLCTRL_Pin|EXT_RESET_Pin|CTP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_BK1_IO3_Pin QSPI_BK1_IO2_Pin */
  GPIO_InitStruct.Pin = QSPI_BK1_IO3_Pin|QSPI_BK1_IO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QSPI;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_BK1_IO0_Pin QSPI_BK1_IO1_Pin */
  GPIO_InitStruct.Pin = QSPI_BK1_IO0_Pin|QSPI_BK1_IO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : DFSDM_CKOUT_Pin */
  GPIO_InitStruct.Pin = DFSDM_CKOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_DFSDM1;
  HAL_GPIO_Init(DFSDM_CKOUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLINK_RX_Pin STLINK_TX_Pin */
  GPIO_InitStruct.Pin = STLINK_RX_Pin|STLINK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CODEC_I2S3_WS_Pin */
  GPIO_InitStruct.Pin = CODEC_I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(CODEC_I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DFSDM_DATIN0_Pin */
  GPIO_InitStruct.Pin = DFSDM_DATIN0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_DFSDM1;
  HAL_GPIO_Init(DFSDM_DATIN0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_CLK_Pin */
  GPIO_InitStruct.Pin = QSPI_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QSPI;
  HAL_GPIO_Init(QSPI_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : JOY_RIGHT_Pin JOY_LEFT_Pin */
  GPIO_InitStruct.Pin = JOY_RIGHT_Pin|JOY_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : JOY_UP_Pin JOY_DOWN_Pin CODEC_INT_Pin CTP_INT_Pin */
  GPIO_InitStruct.Pin = JOY_UP_Pin|JOY_DOWN_Pin|CODEC_INT_Pin|CTP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin
                           D8_Pin D9_Pin D10_Pin D11_Pin
                           D12_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin
                          |D8_Pin|D9_Pin|D10_Pin|D11_Pin
                          |D12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C2_SCL_Pin */
  GPIO_InitStruct.Pin = I2C2_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(I2C2_SCL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : M2_CKIN_Pin */
  GPIO_InitStruct.Pin = M2_CKIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(M2_CKIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CODEC_I2S3_SCK_Pin CODEC_I2S3ext_SD_Pin */
  GPIO_InitStruct.Pin = CODEC_I2S3_SCK_Pin|CODEC_I2S3ext_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D13_Pin D14_Pin D15_Pin D0_Pin
                           D1_Pin D2_Pin D3_Pin FMC_NOE_Pin
                           FMC_NWE_Pin FMC_NE1_Pin */
  GPIO_InitStruct.Pin = D13_Pin|D14_Pin|D15_Pin|D0_Pin
                          |D1_Pin|D2_Pin|D3_Pin|FMC_NOE_Pin
                          |FMC_NWE_Pin|FMC_NE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RESET_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_TE_Pin USB_OTGFS_OVRCR_Pin */
  GPIO_InitStruct.Pin = LCD_TE_Pin|USB_OTGFS_OVRCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_BK1_NCS_Pin */
  GPIO_InitStruct.Pin = QSPI_BK1_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(QSPI_BK1_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTGFS_PPWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTGFS_PPWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTGFS_PPWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CODEC_I2S3_MCK_Pin */
  GPIO_InitStruct.Pin = CODEC_I2S3_MCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(CODEC_I2S3_MCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : uSD_D0_Pin uSD_D1_Pin uSD_D2_Pin uSD_D3_Pin
                           uSD_CLK_Pin */
  GPIO_InitStruct.Pin = uSD_D0_Pin|uSD_D1_Pin|uSD_D2_Pin|uSD_D3_Pin
                          |uSD_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : M2_CKINA8_Pin */
  GPIO_InitStruct.Pin = M2_CKINA8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(M2_CKINA8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_CMD_Pin */
  GPIO_InitStruct.Pin = uSD_CMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
  HAL_GPIO_Init(uSD_CMD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_DETECT_Pin */
  GPIO_InitStruct.Pin = uSD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CODEC_I2S3_SD_Pin */
  GPIO_InitStruct.Pin = CODEC_I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(CODEC_I2S3_SD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C1_SCL_Pin I2C1_SDA_Pin */
  GPIO_InitStruct.Pin = I2C1_SCL_Pin|I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C2_SDA_Pin */
  GPIO_InitStruct.Pin = I2C2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_I2C2;
  HAL_GPIO_Init(I2C2_SDA_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// DOCUMENTATION: EXTI Callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(GPIO_Pin == JOY_UP_Pin)
	{
		HAL_GPIO_WritePin(GPIOE, LED1_Pin, 0);
		xTaskNotifyFromISR( arpTaskHandle,
		                       JOY_UP_BIT,
		                       eSetBits,
		                       &xHigherPriorityTaskWoken );
	}
	if(GPIO_Pin == JOY_DOWN_Pin)
	{
		HAL_GPIO_WritePin(GPIOE, LED1_Pin, 1);
		xTaskNotifyFromISR( arpTaskHandle,
		                       JOY_DOWN_BIT,
		                       eSetBits,
		                       &xHigherPriorityTaskWoken );
	}
	if(GPIO_Pin == JOY_LEFT_Pin)
	{
		HAL_GPIO_TogglePin(GPIOE, LED3_Pin);
		xTaskNotifyFromISR( arpTaskHandle,
				                       JOY_LEFT_BIT,
				                       eSetBits,
				                       &xHigherPriorityTaskWoken );
	}
	if(GPIO_Pin == JOY_RIGHT_Pin)
	{
		HAL_GPIO_TogglePin(GPIOE, LED4_Pin);
		xTaskNotifyFromISR( arpTaskHandle,
				                       JOY_RIGHT_BIT,
				                       eSetBits,
				                       &xHigherPriorityTaskWoken );
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


//--------------------------------------------------------------------+
// USB DEVICE
//--------------------------------------------------------------------+
static void UsbDeviceTask(void *param) {
  (void) param;

  // init device stack on configured roothub port
  // This should be called after scheduler/kernel is started.
  // Otherwise it could cause kernel issue since USB IRQ handler does use RTOS queue API.
  tud_init(BOARD_TUD_RHPORT);

  // RTOS forever loop
  for (;;) {
    // put this thread to waiting state until there is new events
    tud_task();
    osDelay(1);

    // following code only run if tud_task() process at least 1 event
    // tud_cdc_write_flush();
  }
}

//--------------------------------------------------------------------+
// USB MIDI
//--------------------------------------------------------------------+

void MidiTask(void *params) {
	(void) params;

	BaseType_t xResult;
	uint32_t ulNotifiedValue;

	static ArpState state;
	initArpState(&state);

	TimerHandle_t timer = xTimerCreate
      (
        "Timer",
        getArpSpeed(&state),
//				3, //TODO remove, for debugging only
        pdTRUE,
        ( void * ) &state, // initialize counter
        vTimerMidiCallback
      );
//	static uint32_t period_ms = 200;


	// RTOS forever loop
	for (;;) {

		xResult = xTaskNotifyWait( pdFALSE, /* Don't clear bits on entry. */
				ULONG_MAX, /* bits to clear on exit */
				&ulNotifiedValue, /* Stores the notified value. */
				portMAX_DELAY);

		if( xResult == pdPASS )
		{
			/* A notification was received.  See which bits were set. */
			if( ( ulNotifiedValue & JOY_UP_BIT ) != 0 )
			{
				 xTimerStart( timer, 0 );
			}

			if( ( ulNotifiedValue & JOY_DOWN_BIT ) != 0 )
			{
				if (timer != NULL) {
					xTimerStop( timer, 0 );
				}
				state.note_pos = 0;

				uint8_t all_notes_off[3] = { 0xB0, 0x7B, 0 };
				tud_midi_stream_write(0, all_notes_off, 3);
			}

			if( ( ulNotifiedValue & JOY_LEFT_BIT ) != 0 )
			{
				stepArpSpeed(&state);
			}

			if( ( ulNotifiedValue & JOY_RIGHT_BIT ) != 0 )
			{
				stepArpSequence(&state);
			}
		}

		// not sure if this is still needed
		// osDelay(1);
	}
}

void vTimerMidiCallback(TimerHandle_t xTimer)
{
		uint8_t const channel   = 0;
		uint8_t const cable_num = 0;

    ArpState *state = (ArpState *)pvTimerGetTimerID(xTimer);

    xTimerChangePeriod( xTimer, state->speeds[state->current_speed], 0);

		// Previous positions in the note sequence.
		int previous = (int) (state->note_pos - 1);

		// If we currently are at position 0, set the
		// previous position to the last note in the sequence.
		// TODO fix this! hardcoded magic number
		if (previous < 0) previous = 7;

    uint8_t note_on[3] = { 0x90 | channel, state->sequences[state->current_sequence][state->note_pos], 127 };
    tud_midi_stream_write(cable_num, note_on, 3);

    uint8_t note_off[3] = { 0x90 | channel, state->sequences[state->current_sequence][previous], 0 };
    tud_midi_stream_write(cable_num, note_off, 3);

    stepArpNote(state);
}

//uint32_t note_pos = 0; // for testing only will go away

// for testing only will go away
//uint8_t note_sequence[] =
//{
//  74,78,81,86,90,93,98,102,57,61,66,69,73,78,81,85,88,92,97,100,97,92,88,85,81,78,
//  74,69,66,62,57,62,66,69,74,78,81,86,90,93,97,102,97,93,90,85,81,78,73,68,64,61,
//  56,61,64,68,74,78,81,86,90,93,98,102
//};

// working MIDI Task
//void MidiTaskWorking(void *params) {
//	(void) params;
//
//	ArpState state;
//
//	initArpState(&state);
//
//	BaseType_t xResult;
//	uint32_t ulNotifiedValue;
//
//	uint8_t task_state = 0;
////	static uint32_t period_ms = 200;
//
//	TickType_t lastWake = 0;
//
//	// RTOS forever loop
//	for (;;) {
//
//		xResult = xTaskNotifyWait( pdFALSE, /* Don't clear bits on entry. */
//				ULONG_MAX, /* bits to clear on exit */
//				&ulNotifiedValue, /* Stores the notified value. */
//				0);
//
//		if( xResult == pdPASS )
//		{
//			/* A notification was received.  See which bits were set. */
//			if( ( ulNotifiedValue & JOY_UP_BIT ) != 0 )
//			{
//				task_state = 1;
////				lastWake = xTaskGetTickCount();
//			}
//
//			if( ( ulNotifiedValue & JOY_DOWN_BIT ) != 0 )
//			{
//				task_state = 0;
//			}
//
//			if( ( ulNotifiedValue & JOY_LEFT_BIT ) != 0 )
//			{
//				stepArpSpeed(&state);
//			}
//
//			if( ( ulNotifiedValue & JOY_RIGHT_BIT ) != 0 )
//			{
//				stepArpSequence(&state);
//			}
//		}
//
//		if (task_state){
//			uint8_t should_play = 1;
//
//			static uint32_t start_ms = 0;
//			uint32_t period_ms = getArpSpeed(&state);
//			uint8_t *note_sequence = getArpSequence(&state);
//
//			uint8_t const cable_num = 0; // MIDI jack associated with USB endpoint
//			uint8_t const channel   = 0; // 0 for channel 1
//
//			// The MIDI interface always creates input and output port/jack descriptors
//			// regardless of these being used or not. Therefore incoming traffic should be read
//			// (possibly just discarded) to avoid the sender blocking in IO
//			// TODO extract reading into it's own task
//			uint8_t packet[4];
//			while ( tud_midi_available() ) tud_midi_packet_read(packet);
//
//			uint32_t testTick = xTaskGetTickCount() - lastWake;
//
//			// send note periodically
//			if (xTaskGetTickCount() - start_ms < period_ms) should_play = 0; // not enough time
//			//    if (xTaskGetTickCount() - start_ms < 286) return;
//			if (should_play) {
//				start_ms += period_ms;
//
//				// Previous positions in the note sequence.
//				int previous = (int) (note_pos - 1);
//
//				// If we currently are at position 0, set the
//				// previous position to the last note in the sequence.
//				if (previous < 0) previous = sizeof(note_sequence) - 1;
//
//				// Send Note On for current position at full velocity (127) on channel 1.
//				uint8_t note_on[3] = { 0x90 | channel, note_sequence[note_pos], 127 };
//				tud_midi_stream_write(cable_num, note_on, 3);
//
//				// Send Note Off for previous note.
//				uint8_t note_off[3] = { 0x80 | channel, note_sequence[previous], 0};
//				tud_midi_stream_write(cable_num, note_off, 3);
//
//				// Increment position
//				note_pos++;
//
//				// If we are at the end of the sequence, start over.
//				if (note_pos >= sizeof(note_sequence)) note_pos = 0;
//			}
//		}
//
//		// For ESP32-Sx this delay is essential to allow idle how to run and reset watchdog
//		osDelay(1);
//	}
//}

// Resumable Task

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

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
