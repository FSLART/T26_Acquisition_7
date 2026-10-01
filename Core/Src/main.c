/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "../DBC/data_t26.h"


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

/* USER CODE BEGIN PV */
#define PRESSURE_LEVEL_FRONT 7   // brake light threshold (bar)
#define PRESSURE_LEVEL_REAR 7    // brake light threshold (bar)

volatile uint32_t time_ms = 0;

CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

// Front brake pressure, received on CAN1 (0x710)
float BRK_PRESS_FRONT;         // bar, 0 while 0x710 is not arriving
uint16_t coded_front_pressure; // 0.1 bar/bit
volatile uint32_t front_last_rx_ms = 0;  // time_ms of the last 0x710 frame
uint8_t front_alive;                     // 1 while 0x710 arrives within FRONT_PRESSURE_TIMEOUT_MS

// 0x710 is only trusted while it keeps arriving (set to >= 3x the 0x710 period)
#define FRONT_PRESSURE_TIMEOUT_MS 500

// Analog sensors
float BRK_PRESS_REAR;   // Rear brake pressure (bar)
float SUSP_LEFT;        // Left suspension position (mm)
float SUSP_RIGHT;       // Right suspension position (mm)

// Coolant NTCs: Vishay NTCAIMM66H, 10k @ 25 C, B25/85 = 3984 K.
// Board: 5 V -> NTC -> node A; node A has 100k to GND and 24.3k -> ADC pin (4.9k from the ADC pin to GND)
#define NTC_COUNT 1             // PC4 (ADC1_IN14); more NTCs = more ranks in the .ioc, then raise this
#define NTC_VSUPPLY 5.0f        // V, tune to the real 5 V rail
#define NTC_R_PULLDOWN 100000.0f
#define NTC_R_SERIES 24300.0f
#define NTC_R_BOTTOM 4900.0f
#define NTC_R25 10000.0f
#define NTC_B 3984.0f
#define NTC_FAULT_C (-99.0f)    // open (ADC = 0) or shorted sensor
float NTC_TEMP[NTC_COUNT];      // degC, from ADC_VALUE[3 + i]

#define ADC_CHANNELS (3 + NTC_COUNT)

// DATA bus (CAN2, 1 Mbit/s): AQT7 (0x770) packed with the cantools code in data_t26.c/.h (T26_DBC repo)
_Static_assert(NTC_COUNT == 1, "AQT7 in the DBC only has NTC_1: add the other NTCs to the DBC and to Data_SendSensors()");
uint16_t ADC_VALUE[ADC_CHANNELS];  // DMA target: [0] PA7 brake pressure, [1] PB0 susp left, [2] PB1 susp right, [3..] NTCs (PC4)
uint16_t brk_press_adc;
uint16_t susp_left_adc;
uint16_t susp_right_adc;

// ADC moving average
#define ADC_BUFFER_SIZE 10
uint16_t adc_buffers[ADC_CHANNELS][ADC_BUFFER_SIZE];
uint8_t adc_buffer_index = 0;
uint16_t adc_filtered[ADC_CHANNELS];

// Debug snapshot of the whole board: add "acq7" to STM32CubeIDE Live Expressions
typedef struct {
	uint32_t time_ms;           // uptime (ms)

	struct {
		uint16_t raw[ADC_CHANNELS];       // DMA samples: [0] PA7 brake pressure, [1] PB0 susp left, [2] PB1 susp right, [3..] NTCs
		uint16_t filtered[ADC_CHANNELS];  // moving average of ADC_BUFFER_SIZE samples
		float volts[ADC_CHANNELS];        // filtered value at the MCU pin (V)
	} adc;

	struct {
		float brake_rear_bar;
		float brake_front_bar;  // 0 while 0x710 is not arriving
		float susp_left_mm;
		float susp_right_mm;
		float ntc_c[NTC_COUNT]; // coolant temperature (degC), NTC_FAULT_C if open / shorted
	} sensors;

	struct {
		uint8_t brake_light;    // PC10 output level, 0 while braking (pressure >= PRESSURE_LEVEL)
		uint8_t heartbeat;      // PC11 output level
	} io;

	struct {
		uint16_t coded;         // 0x710 bytes 0-1, 0.1 bar/bit
		uint8_t alive;          // 1 while 0x710 arrives within FRONT_PRESSURE_TIMEOUT_MS
		uint32_t age_ms;        // time since the last 0x710
		uint32_t rx_count;      // 0x710 frames received
	} front;

	struct {
		uint8_t tx_770_autonomous[2]; // last payload queued on the Autonomous bus (CAN1, 0x770): rear brake pressure
		uint8_t tx_aqt7_data[DATA_T26_AQT7_LENGTH]; // last payload queued on the DATA bus (CAN2, AQT7): SUSP_L, SUSP_R, NTC_1
		CAN_BusStatus autonomous;     // CAN1
		CAN_BusStatus data;           // CAN2
	} can;
} Acq7_Debug;

Acq7_Debug acq7;

// Tasks Prototypes
void execute_immediate_tasks(void);
void execute_10ms_tasks(void);
void execute_50ms_tasks(void);
void execute_100ms_tasks(void);
void Acq7_DebugUpdate(void);
void Data_SendSensors(void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ADC_UpdateMovingAverage(void);
float MeasureSuspensionPosition(uint16_t bits);
float MeasureBrakePressure(uint16_t bits);
float MeasureNtcTemperature(uint16_t bits);

// printf -> USART1
int _write(int file, char *data, int len) {
	HAL_UART_Transmit(&huart1, (uint8_t*) data, len, HAL_MAX_DELAY);
	return len;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM7) {
		time_ms++;
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_VALUE, ADC_CHANNELS);

	for (int i = 0; i < ADC_CHANNELS; i++) {
		for (int j = 0; j < ADC_BUFFER_SIZE; j++) {
			adc_buffers[i][j] = 0;
		}
		adc_filtered[i] = 0;
	}

	// CAN filters, start and RX notification are done in MX_CANx_Init (CAN_Config)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		// Execute immediate tasks
		execute_immediate_tasks();

		static uint32_t previus_tick_10ms = 0;
		static uint32_t previus_tick_50ms = 0;
		static uint32_t previus_tick_100ms = 0;

		// Execute 10ms Tasks
		if (time_ms - previus_tick_10ms >= 10) {
			execute_10ms_tasks();
			previus_tick_10ms = time_ms;
		}

		// Execute 50ms Tasks
		if (time_ms - previus_tick_50ms >= 50) {
			execute_50ms_tasks();
			previus_tick_50ms = time_ms;
		}

		// Execute 100ms Tasks
		if (time_ms - previus_tick_100ms >= 100) {
			execute_100ms_tasks();
			previus_tick_100ms = time_ms;
		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void execute_immediate_tasks() {

}
void execute_10ms_tasks() {
	HAL_IWDG_Refresh(&hiwdg);

	brk_press_adc = adc_filtered[0];
	susp_left_adc = adc_filtered[1];
	susp_right_adc = adc_filtered[2];

	// CAN health check every 10 ms, restart max every 100 ms (see canX_status / acq7.can)
	CAN_Service(CAN_AUTONOMOUS);
	CAN_Service(CAN_DATA);

	Acq7_DebugUpdate();
}
void execute_50ms_tasks() {

	ADC_UpdateMovingAverage();

	BRK_PRESS_REAR = MeasureBrakePressure(brk_press_adc);
	SUSP_LEFT = MeasureSuspensionPosition(susp_left_adc);
	SUSP_RIGHT = MeasureSuspensionPosition(susp_right_adc);

	for (int i = 0; i < NTC_COUNT; i++) {
		NTC_TEMP[i] = MeasureNtcTemperature(adc_filtered[3 + i]);
	}

	uint16_t brake = (uint16_t) (BRK_PRESS_REAR * 10.0f);      // 0.1 bar/bit

	// Autonomous bus (CAN1) - 0x770: rear brake pressure
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = 0x770;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 2;

	TxData[0] = brake & 0xFF;
	TxData[1] = (brake >> 8) & 0xFF;

	// A failed send is counted in can1_status and handled by CAN_Service, never fatal
	memcpy(acq7.can.tx_770_autonomous, TxData, sizeof(acq7.can.tx_770_autonomous));
	CAN_Send(CAN_AUTONOMOUS, &TxHeader, TxData);

	// DATA bus (CAN2): suspension and NTCs
	Data_SendSensors();

	// Front brake pressure (Autonomous bus, CAN1 0x710, 0.1 bar/bit). Forced to 0 if 0x710 stopped arriving,
	// so the brake light then follows the rear pressure only.
	front_alive = (time_ms - front_last_rx_ms) <= FRONT_PRESSURE_TIMEOUT_MS;
	if (front_alive) {
		BRK_PRESS_FRONT = coded_front_pressure * 0.1;
	} else {
		BRK_PRESS_FRONT = 0.0f;
	}

	// Brake light: output low while either circuit is at PRESSURE_LEVEL or more

	if ((BRK_PRESS_REAR >= PRESSURE_LEVEL_REAR)
			|| (BRK_PRESS_FRONT >= PRESSURE_LEVEL_FRONT)) {
		HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin, GPIO_PIN_SET);
	}
}
void execute_100ms_tasks() {
	HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin); // HEARTBEAT

	printf("A pressao traseira é %.2f bar e adc é %d\n", BRK_PRESS_REAR, brk_press_adc);

	printf("A pressao da frente é %.2f bar e %d\n", BRK_PRESS_FRONT, coded_front_pressure);

	printf("A suspensao esquerda é %.2f mm e adc é %d\n", SUSP_LEFT, susp_left_adc);

	printf("A suspensao direita é %.2f mm e adc é %d\n", SUSP_RIGHT, susp_right_adc);

	for (int i = 0; i < NTC_COUNT; i++) {
		printf("A NTC%d está a %.1f C e adc é %d\n", i + 1, NTC_TEMP[i], adc_filtered[3 + i]);
	}

}

// DATA bus (CAN2) transmit: a failed send is counted in can2_status and handled by CAN_Service
void Data_SendSensors(void) {
	CAN_TxHeaderTypeDef header = { .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA,
			.StdId = DATA_T26_AQT7_FRAME_ID, .DLC = DATA_T26_AQT7_LENGTH };
	struct data_t26_aqt7_t aqt7;
	uint8_t data[DATA_T26_AQT7_LENGTH];

	aqt7.susp_l = data_t26_aqt7_susp_l_encode(SUSP_LEFT);
	aqt7.susp_r = data_t26_aqt7_susp_r_encode(SUSP_RIGHT);
	// NTC_1 is uint8, 1 degC/bit: rounded and clamped to 0..254, 255 = open / shorted sensor
	if (NTC_TEMP[0] <= NTC_FAULT_C) {
		aqt7.ntc_1 = 255;
	} else {
		float t = NTC_TEMP[0];
		t = (t < 0.0f) ? 0.0f : ((t > 254.0f) ? 254.0f : t);
		aqt7.ntc_1 = data_t26_aqt7_ntc_1_encode(t + 0.5f);
	}

	if (data_t26_aqt7_pack(data, &aqt7, sizeof(data)) < 0) {
		return;
	}

	memcpy(acq7.can.tx_aqt7_data, data, sizeof(data));
	CAN_Send(CAN_DATA, &header, data);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	// Never fatal: a frame that cannot be read is skipped
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
		return;
	}
	if ((RxHeader.StdId == 0x710) && (RxHeader.IDE == CAN_ID_STD)) {
		coded_front_pressure = RxData[1] << 8 | RxData[0];
		front_last_rx_ms = time_ms;
		acq7.front.rx_count++;
	}
}

// Copies everything into acq7 for Live Expressions (called every 10 ms)
void Acq7_DebugUpdate(void) {
	uint32_t now = time_ms;

	acq7.time_ms = now;

	for (int i = 0; i < ADC_CHANNELS; i++) {
		acq7.adc.raw[i] = ADC_VALUE[i];
		acq7.adc.filtered[i] = adc_filtered[i];
		acq7.adc.volts[i] = adc_filtered[i] * 3.3f / 4095.0f;
	}

	acq7.sensors.brake_rear_bar = BRK_PRESS_REAR;
	acq7.sensors.brake_front_bar = BRK_PRESS_FRONT;
	acq7.sensors.susp_left_mm = SUSP_LEFT;
	acq7.sensors.susp_right_mm = SUSP_RIGHT;
	for (int i = 0; i < NTC_COUNT; i++) {
		acq7.sensors.ntc_c[i] = NTC_TEMP[i];
	}

	acq7.io.brake_light = HAL_GPIO_ReadPin(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin);
	acq7.io.heartbeat = HAL_GPIO_ReadPin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);

	acq7.front.coded = coded_front_pressure;
	acq7.front.alive = front_alive;
	acq7.front.age_ms = now - front_last_rx_ms;

	acq7.can.autonomous = can1_status;
	acq7.can.data = can2_status;
}

void ADC_UpdateMovingAverage(void) {

	for (int channel = 0; channel < ADC_CHANNELS; channel++) {
		// Latest sample into the circular buffer
		adc_buffers[channel][adc_buffer_index] = ADC_VALUE[channel];

		// Average of the last ADC_BUFFER_SIZE samples
		uint32_t sum = 0;
		for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
			sum += adc_buffers[channel][i];
		}


		adc_filtered[channel] = (uint16_t) (sum / ADC_BUFFER_SIZE);
	}

	// Next buffer position
	adc_buffer_index = (adc_buffer_index + 1) % ADC_BUFFER_SIZE;
}
float MeasureBrakePressure(uint16_t bits) {
	// Pressure sensor: 0.5-4.5 V for 0-140 bar, 5 V -> 3.3 V divider on the board
	const float ADC_MAX = 4095.0f;
	const float MCU_VREF = 3.3f;
	const float SENSOR_VREF = 5.0f;
	const float CONVERSION_FACTOR = 0.667f;
	const float OFFSET_VOLTAGE = 0.5f;
	const float SENSITIVITY = 0.02857f;    // V/bar

	// Voltage at the MCU pin
	float volts = (float) bits * MCU_VREF / ADC_MAX;

	// Voltage at the sensor output
	volts = volts / CONVERSION_FACTOR;

	// Clamp to the sensor supply range
	if (volts < 0.0f) {
		volts = 0.0f;
	} else if (volts > SENSOR_VREF) {
		volts = SENSOR_VREF;
	}

	// Below the 0.5 V offset the pressure is 0

	float pressure = 0.0f;
	if (volts <= OFFSET_VOLTAGE) {
		pressure = 0.0f;
	} else {
		pressure = (volts - OFFSET_VOLTAGE) / SENSITIVITY;
	}

	// Sensor full scale
	const float MAX_PRESSURE = 140.0f;
	if (pressure > MAX_PRESSURE) {
		pressure = MAX_PRESSURE;
	}

	return pressure;
}
float MeasureSuspensionPosition(uint16_t bits) {
	// Linear potentiometer: 75 mm electrical stroke, 5 V supply, 5 V -> 3.3 V divider
	float V_SUSP;
	float sensor_voltage = 5.0f;
	float MCU_voltage = 3.3f;
	float Electrical_stroke = 75.0f;
	float SUSPENSION_POSITION;
	float Conversion_Factor = MCU_voltage / sensor_voltage;
	float volts;

	V_SUSP = (float)(bits * MCU_voltage / 4095.0f);
	volts = V_SUSP / Conversion_Factor;

	SUSPENSION_POSITION = (Electrical_stroke * volts) / sensor_voltage;

	return SUSPENSION_POSITION;
}
float MeasureNtcTemperature(uint16_t bits) {
	// Voltage at the MCU pin, then back up through the 24.3k / 4.9k divider to node A
	float v_pin = (float) bits * 3.3f / 4095.0f;
	float v_a = v_pin * (NTC_R_SERIES + NTC_R_BOTTOM) / NTC_R_BOTTOM;

	// Open sensor (pulled to 0 V) or shorted sensor (node A at the full supply)
	if (bits == 0 || v_a >= NTC_VSUPPLY) {
		return NTC_FAULT_C;
	}

	// Node A sees the 100k pull-down in parallel with the 24.3k + 4.9k branch
	float r_branch = NTC_R_SERIES + NTC_R_BOTTOM;
	float r_low = (NTC_R_PULLDOWN * r_branch) / (NTC_R_PULLDOWN + r_branch);

	// v_a = VSUPPLY * r_low / (r_ntc + r_low)
	float r_ntc = r_low * (NTC_VSUPPLY - v_a) / v_a;

	// Beta equation around 25 C
	const float T25_K = 298.15f;
	float inv_t = 1.0f / T25_K + logf(r_ntc / NTC_R25) / NTC_B;

	return 1.0f / inv_t - 273.15f;
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
