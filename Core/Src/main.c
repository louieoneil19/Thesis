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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define ADC_BUF_LEN 64
#define ADC_MAX_COUNTS      4095.0f
#define ADC_VREF            3.3f      // STM32 VREF+ — used for ALL ADC conversions

/* Soil moisture sensor (DFRobot SEN0308 on PC3 / ADC3_IN11) */
#define MOISTURE_AIR_VALUE      3665   // CALIBRATE: raw ADC value in dry air
#define MOISTURE_WATER_VALUE    540   // CALIBRATE: raw ADC value submerged in water
#define MOISTURE_INTERVALS      ((MOISTURE_AIR_VALUE - MOISTURE_WATER_VALUE) / 3)

/* Pump speed control (M3 = TIM1 CH3) */
typedef enum {
    PUMP_OFF  = 0,
    PUMP_SLOW = 50,    // ~6V at 12V battery
    PUMP_FAST = 75,    // ~9V at 12V battery
    PUMP_FULL = 100    // 12V (demo only)
} PumpSpeed_t;

// Voltage sensor signal chain:
//   Vbus -> divider (R1=98.8k top, R2=10k bottom) -> ACPL-C87X (gain=1)
//        -> OPA237 diff amp (gain=1) -> STM32 ADC pin
// So: V_adc_pin = Vbus * R2 / (R1+R2) = Vbus / 10.88
#define VDIV_RTOP           98800.0f
#define VDIV_RBOT           10000.0f
#define VDIV_SCALE          ((VDIV_RTOP + VDIV_RBOT) / VDIV_RBOT)  // = 10.88
#define VOLTAGE_OFFSET      0.0002f   //

// Current sensor (ACS722, bidirectional, 3.3V supply):
//   VIOUT_Q = VCC/2 = 1.65V at zero current

#define ACS_ZERO_V          1.6328f   //
#define ACS_SENS_V_PER_A    0.2640f   // adjust if you have a different variant

static uint32_t override_until_tick = 0;

uint32_t adc_buffer[ADC_BUF_LEN];

volatile uint64_t v_sum = 0;
volatile uint64_t i_sum = 0;
volatile uint32_t sample_count = 0;

uint32_t last_print = 0;

/* =========================
 * Control mode selection
 * ========================= */

#define CONTROL_MANUAL             0
#define CONTROL_IV_SWEEP           1
#define CONTROL_MPPT               2
#define CONTROL_SOIL_ONLY          3
#define CONTROL_MPPT_AND_SOIL      4

/*
 * Select your current operating mode here.
 *
 * CONTROL_MANUAL:
 *      M1/M2/M3 use manual duty values.
 *
 * CONTROL_IV_SWEEP:
 *      M1/M2 sweep through iv_duty_list.
 *      M3 is pump off for safety.
 *
 * CONTROL_MPPT:
 *      MPPT will control M1/M2.
 *      M3 uses manual duty unless ENABLE_SOIL_MOISTURE is 1.
 *
 * CONTROL_SOIL_ONLY:
 *      M1/M2 manual.
 *      M3 controlled by soil moisture.
 *
 * CONTROL_MPPT_AND_SOIL:
 *      MPPT controls M1/M2.
 *      Soil moisture controls M3.
 */
#define CONTROL_MODE CONTROL_MPPT

/*
 * Independent enable for soil moisture pump control.
 * This is automatically used in CONTROL_SOIL_ONLY and CONTROL_MPPT_AND_SOIL.
 * You can also enable it with CONTROL_MPPT if desired.
 */
#define ENABLE_SOIL_MOISTURE      0

/* Manual duty settings */
#define MANUAL_M1_DUTY            30u
#define MANUAL_M2_DUTY            30u
#define MANUAL_M3_DUTY            70u

/* Live sensor print timing */
#define SENSOR_PRINT_TIME_MS      500u

/* =========================
 * Automatic IV sweep settings
 * ========================= */

#define IV_SETTLE_TIME_MS         500u
#define IV_MEASURE_TIME_MS        1000u

static const uint8_t iv_duty_list[] = {
	93, 81, 72, 65, 59,
    55, 52, 50, 48, 46,
    45, 44, 43, 42, 41,
    40, 39, 38, 37, 36,
    35, 34, 33, 32, 30
};

#define IV_NUM_POINTS   (sizeof(iv_duty_list) / sizeof(iv_duty_list[0]))

typedef enum {
    IV_SWEEP_IDLE = 0,
    IV_SWEEP_SET_DUTY,
    IV_SWEEP_SETTLE,
    IV_SWEEP_MEASURE,
    IV_SWEEP_PRINT,
    IV_SWEEP_DONE
} IV_SweepState_t;

static IV_SweepState_t iv_state = IV_SWEEP_SET_DUTY;
static uint32_t iv_state_tick = 0;
static uint8_t iv_index = 0;

/* Sums used only for one sweep operating point */
static uint64_t iv_v_sum = 0;
static uint64_t iv_i_sum = 0;
static uint32_t iv_count = 0;

/* Latest soil moisture / pump status */
static uint16_t latest_moisture_raw = 0;
static const char* latest_moisture_label = "N/A";
static PumpSpeed_t latest_pump_speed = PUMP_OFF;
static uint8_t soil_data_valid = 0;

/* =========================
 * MPPT placeholder variables
 * ========================= */

/* =========================
 * MPPT Perturb & Observe settings
 * ========================= */

#define MPPT_UPDATE_TIME_MS       500u

#define MPPT_DUTY_MIN             30u
#define MPPT_DUTY_MAX             93u
#define MPPT_DUTY_START           45u
#define MPPT_DUTY_STEP            1u

#define MPPT_TRACK_DURATION_MS        20000u
#define MPPT_FINAL_AVG_WINDOW_MS      5000u
#define MPPT_TRACK_SETTLE_MS          500u

/*
 * Small threshold to avoid reacting to sensor noise.
 * If power change is smaller than this, duty is not changed.
 */
#define MPPT_POWER_DEADBAND_W     0.03f

static uint8_t mppt_duty = MPPT_DUTY_START;
static int8_t mppt_direction = 1;   // +1 means increase duty, -1 means decrease duty
static float mppt_prev_power = 0.0f;
static float mppt_prev_voltage = 0.0f;
static float mppt_prev_current = 0.0f;
static uint8_t mppt_initialized = 0;
static uint32_t last_mppt_tick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */

void find_calibration_values(void);
void find_moisture_calibration(void);

void Set_Converter_Duty(uint8_t duty_percent);
void Set_Manual_Duties(void);
void Read_Panel_Sensors(float *panel_voltage,
                        float *panel_current,
                        float *panel_power,
                        float *v_pin_out,
                        uint32_t *sample_n);
void Print_Panel_Sensors(void);
void Run_IV_Sweep(void);
void Run_MPPT(void);
void Run_Soil_Moisture_Control(void);
void track_mpp_20_seconds(uint8_t start_duty);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Process_ADC_Buffer(uint32_t *buffer, uint32_t length)
{
    for (uint32_t n = 0; n < length; n++)
    {
        uint32_t combined = buffer[n];

        uint16_t voltage_raw = combined & 0xFFFF;
        uint16_t current_raw = (combined >> 16) & 0xFFFF;

        v_sum += voltage_raw;
        i_sum += current_raw;
        sample_count++;
    }
}

uint16_t Read_Moisture(void)
{
    // Clear any leftover OVR/EOC flags from the previous conversion
    ADC3->SR = 0;

    // Manually trigger a single conversion
    ADC3->CR2 |= ADC_CR2_SWSTART;

    // Wait for conversion to complete (with timeout)
    uint32_t timeout = HAL_GetTick() + 10;
    while ((ADC3->SR & ADC_SR_EOC) == 0)
    {
        if (HAL_GetTick() > timeout) {
            return 0xFFFF;  // timeout sentinel
        }
    }

    // Read the value (this also clears EOC automatically)
    return (uint16_t)ADC3->DR;
}

const char* Classify_Moisture(uint16_t value)
{
    if (value > MOISTURE_AIR_VALUE)
        return "Very Dry / Air";
    if (value > (MOISTURE_AIR_VALUE - MOISTURE_INTERVALS))
        return "Dry";
    if (value > (MOISTURE_WATER_VALUE + MOISTURE_INTERVALS))
        return "Wet";
    if (value > MOISTURE_WATER_VALUE)
        return "Very Wet";
    return "Submerged";
}

PumpSpeed_t Decide_Pump_Speed(uint16_t moisture_raw)
{
    const uint16_t AIR_MARGIN = 100;  // counts of margin below air calibration

    // Sensor in dry air (or above) -> pump off (safety)
    if (moisture_raw >= (MOISTURE_AIR_VALUE - AIR_MARGIN))   return PUMP_OFF;

    // Below water calibration -> already submerged, pump off (safety)
    if (moisture_raw < MOISTURE_WATER_VALUE) return PUMP_OFF;

    // Within calibrated range -> decide based on dryness
    if (moisture_raw > (MOISTURE_AIR_VALUE - MOISTURE_INTERVALS)) {
        return PUMP_FAST;     // Dry soil
    }
    if (moisture_raw > (MOISTURE_WATER_VALUE + MOISTURE_INTERVALS)) {
        return PUMP_SLOW;     // Moist soil
    }
    return PUMP_OFF;          // Already wet enough
}

void Set_Pump_Speed(PumpSpeed_t speed)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ((uint32_t)speed * arr) / 100);
}

void Set_Converter_Duty(uint8_t duty_percent)
{
    if (duty_percent > 100) {
        duty_percent = 100;
    }

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

    /*
     * MPPT and IV sweep both control M1 and M2 together.
     */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ((uint32_t)duty_percent * arr) / 100);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ((uint32_t)duty_percent * arr) / 100);
}

void Set_Manual_Duties(void)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ((uint32_t)MANUAL_M1_DUTY * arr) / 100);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ((uint32_t)MANUAL_M2_DUTY * arr) / 100);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ((uint32_t)MANUAL_M3_DUTY * arr) / 100);
}

void Read_Panel_Sensors(float *panel_voltage,
                        float *panel_current,
                        float *panel_power,
                        float *v_pin_out,
                        uint32_t *sample_n)
{
    __disable_irq();
    uint64_t v_sum_copy = v_sum;
    uint64_t i_sum_copy = i_sum;
    uint32_t count_copy = sample_count;

    v_sum = 0;
    i_sum = 0;
    sample_count = 0;
    __enable_irq();

    if (sample_n != NULL) {
        *sample_n = count_copy;
    }

    if (count_copy == 0)
    {
        if (panel_voltage != NULL) *panel_voltage = 0.0f;
        if (panel_current != NULL) *panel_current = 0.0f;
        if (panel_power   != NULL) *panel_power   = 0.0f;
        if (v_pin_out     != NULL) *v_pin_out     = 0.0f;
        return;
    }

    float v_adc_avg = (float)v_sum_copy / (float)count_copy;
    float i_adc_avg = (float)i_sum_copy / (float)count_copy;

    float v_pin = (v_adc_avg / ADC_MAX_COUNTS) * ADC_VREF;
    float i_pin = (i_adc_avg / ADC_MAX_COUNTS) * ADC_VREF;

    float v_compensated = v_pin - VOLTAGE_OFFSET;
    float v_panel = v_compensated * VDIV_SCALE;

    float i_panel = (i_pin - ACS_ZERO_V) / ACS_SENS_V_PER_A;

    /*
     * If current polarity is reversed, use:
     * float i_panel = (ACS_ZERO_V - i_pin) / ACS_SENS_V_PER_A;
     */

    float p_panel = v_panel * i_panel;

    if (panel_voltage != NULL) *panel_voltage = v_panel;
    if (panel_current != NULL) *panel_current = i_panel;
    if (panel_power   != NULL) *panel_power   = p_panel;
    if (v_pin_out     != NULL) *v_pin_out     = v_pin;
}

void Print_Panel_Sensors(void)
{
    if ((HAL_GetTick() - last_print) >= SENSOR_PRINT_TIME_MS)
    {
        float panel_voltage = 0.0f;
        float panel_current = 0.0f;
        float panel_power = 0.0f;
        uint32_t sample_n = 0;

        Read_Panel_Sensors(&panel_voltage,
                           &panel_current,
                           &panel_power,
                           NULL,
                           &sample_n);

        if (sample_n > 0)
        {
            char msg[240];

#if (CONTROL_MODE == CONTROL_SOIL_ONLY) || (CONTROL_MODE == CONTROL_MPPT_AND_SOIL) || ((CONTROL_MODE == CONTROL_MPPT) && ENABLE_SOIL_MOISTURE)

            if (soil_data_valid)
            {
                snprintf(msg, sizeof(msg),
                         "Vpanel: %.2f V, I: %.3f A, P: %.2f W | Moisture: %u (%s) | Pump: %u%% (n=%lu)\r\n",
                         panel_voltage,
                         panel_current,
                         panel_power,
                         latest_moisture_raw,
                         latest_moisture_label,
                         (unsigned int)latest_pump_speed,
                         (unsigned long)sample_n);
            }
            else
            {
                snprintf(msg, sizeof(msg),
                         "Vpanel: %.2f V, I: %.3f A, P: %.2f W | Moisture: waiting... (n=%lu)\r\n",
                         panel_voltage,
                         panel_current,
                         panel_power,
                         (unsigned long)sample_n);
            }

#else

            snprintf(msg, sizeof(msg),
                     "D: %u%%, Vpanel: %.2f V, I: %.3f A, P: %.2f W (n=%lu)\r\n",
                     (unsigned int)mppt_duty,
                     panel_voltage,
                     panel_current,
                     panel_power,
                     (unsigned long)sample_n);

#endif

            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
        }

        last_print = HAL_GetTick();
    }
}

void Run_Soil_Moisture_Control(void)
{
    static uint32_t last_soil_tick = 0;

    /*
     * Soil moisture does not need to run very fast.
     * 500 ms is enough for pump control.
     */
    if ((HAL_GetTick() - last_soil_tick) >= 500u)
    {
    	uint16_t moisture_raw = Read_Moisture();
    	const char* moisture_label = Classify_Moisture(moisture_raw);

    	PumpSpeed_t pump_speed;

    	if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
    	{
    	    override_until_tick = HAL_GetTick() + 5000;
    	}

    	if (HAL_GetTick() < override_until_tick)
    	{
    	    pump_speed = PUMP_FULL;
    	    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    	}
    	else
    	{
    	    pump_speed = Decide_Pump_Speed(moisture_raw);
    	    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    	}

    	Set_Pump_Speed(pump_speed);

    	/* Save latest soil/pump values for printing */
    	latest_moisture_raw = moisture_raw;
    	latest_moisture_label = moisture_label;
    	latest_pump_speed = pump_speed;
    	soil_data_valid = 1;

    	last_soil_tick = HAL_GetTick();
    }
}

void Run_MPPT(void)
{
    if ((HAL_GetTick() - last_mppt_tick) >= MPPT_UPDATE_TIME_MS)
    {
        float panel_voltage = 0.0f;
        float panel_current = 0.0f;
        float panel_power = 0.0f;
        uint32_t sample_n = 0;

        /*
         * Read latest averaged panel sensors.
         * v_pin is not needed here, so pass NULL.
         */
        Read_Panel_Sensors(&panel_voltage,
                           &panel_current,
                           &panel_power,
                           NULL,
                           &sample_n);

        if (sample_n > 0)
        {
            if (!mppt_initialized)
            {
                /*
                 * First MPPT sample: store initial values and apply starting duty.
                 */
                mppt_prev_power = panel_power;
                mppt_prev_voltage = panel_voltage;
                mppt_prev_current = panel_current;
                mppt_initialized = 1;

                Set_Converter_Duty(mppt_duty);
            }
            else
            {
                float delta_p = panel_power - mppt_prev_power;

                /*
                 * Perturb & Observe:
                 * If power increased, keep perturbing in the same duty direction.
                 * If power decreased, reverse perturbation direction.
                 */
                if (delta_p > MPPT_POWER_DEADBAND_W)
                {
                    /* Power increased: keep same direction */
                }
                else if (delta_p < -MPPT_POWER_DEADBAND_W)
                {
                    /* Power decreased: reverse direction */
                    mppt_direction = -mppt_direction;
                }
                else
                {
                    /*
                     * Power change is tiny, likely sensor noise.
                     * Do not change direction.
                     */
                }

                int16_t next_duty = (int16_t)mppt_duty +
                                    ((int16_t)mppt_direction * (int16_t)MPPT_DUTY_STEP);

                if (next_duty > MPPT_DUTY_MAX)
                {
                    next_duty = MPPT_DUTY_MAX;
                    mppt_direction = -1;
                }
                else if (next_duty < MPPT_DUTY_MIN)
                {
                    next_duty = MPPT_DUTY_MIN;
                    mppt_direction = 1;
                }

                mppt_duty = (uint8_t)next_duty;
                Set_Converter_Duty(mppt_duty);

                mppt_prev_power = panel_power;
                mppt_prev_voltage = panel_voltage;
                mppt_prev_current = panel_current;
            }

            char msg[180];
            snprintf(msg, sizeof(msg),
                     "MPPT: D=%u%%, V=%.2f V, I=%.3f A, P=%.2f W, dir=%d, n=%lu\r\n",
                     (unsigned int)mppt_duty,
                     panel_voltage,
                     panel_current,
                     panel_power,
                     (int)mppt_direction,
                     (unsigned long)sample_n);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
        }

        last_mppt_tick = HAL_GetTick();
    }
}

void Run_IV_Sweep(void)
{
    switch (iv_state)
    {
        case IV_SWEEP_SET_DUTY:
        {
            if (iv_index >= IV_NUM_POINTS)
            {
                iv_state = IV_SWEEP_DONE;
                break;
            }

            uint8_t duty = iv_duty_list[iv_index];
            Set_Converter_Duty(duty);

            /*
             * Keep pump off during IV sweep for cleaner measurements.
             */
            Set_Pump_Speed(PUMP_OFF);

            char msg[120];
            snprintf(msg, sizeof(msg),
                     "\r\nSetting point %u/%u, duty = %u%%\r\n",
                     (unsigned int)(iv_index + 1),
                     (unsigned int)IV_NUM_POINTS,
                     (unsigned int)duty);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

            iv_state_tick = HAL_GetTick();
            iv_state = IV_SWEEP_SETTLE;
            break;
        }

        case IV_SWEEP_SETTLE:
        {
            if ((HAL_GetTick() - iv_state_tick) >= IV_SETTLE_TIME_MS)
            {
                __disable_irq();
                v_sum = 0;
                i_sum = 0;
                sample_count = 0;
                __enable_irq();

                iv_state_tick = HAL_GetTick();
                iv_state = IV_SWEEP_MEASURE;
            }
            break;
        }

        case IV_SWEEP_MEASURE:
        {
            if ((HAL_GetTick() - iv_state_tick) >= IV_MEASURE_TIME_MS)
            {
                __disable_irq();
                iv_v_sum = v_sum;
                iv_i_sum = i_sum;
                iv_count = sample_count;

                v_sum = 0;
                i_sum = 0;
                sample_count = 0;
                __enable_irq();

                iv_state = IV_SWEEP_PRINT;
            }
            break;
        }

        case IV_SWEEP_PRINT:
        {
            uint8_t duty = iv_duty_list[iv_index];

            if (iv_count > 0)
            {
                float v_adc_avg = (float)iv_v_sum / (float)iv_count;
                float i_adc_avg = (float)iv_i_sum / (float)iv_count;

                float v_pin = (v_adc_avg / ADC_MAX_COUNTS) * ADC_VREF;
                float i_pin = (i_adc_avg / ADC_MAX_COUNTS) * ADC_VREF;

                float v_compensated = v_pin - VOLTAGE_OFFSET;
                float panel_voltage = v_compensated * VDIV_SCALE;

                float panel_current = (i_pin - ACS_ZERO_V) / ACS_SENS_V_PER_A;
                float panel_power = panel_voltage * panel_current;

                char msg[220];
                snprintf(msg, sizeof(msg),
                         "%u,%u,%.3f,%.4f,%.4f,%lu\r\n",
                         (unsigned int)(iv_index + 1),
                         (unsigned int)duty,
                         panel_voltage,
                         panel_current,
                         panel_power,
                         (unsigned long)iv_count);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
            }
            else
            {
                char msg[120];
                snprintf(msg, sizeof(msg),
                         "%u,%u,NO_SAMPLES,NO_SAMPLES,NO_SAMPLES,0\r\n",
                         (unsigned int)(iv_index + 1),
                         (unsigned int)duty);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
            }

            iv_index++;

            if (iv_index < IV_NUM_POINTS)
            {
                iv_state = IV_SWEEP_SET_DUTY;
            }
            else
            {
                iv_state = IV_SWEEP_DONE;

                char msg[] = "\r\nIV sweep complete.\r\n";
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
            }

            break;
        }

        case IV_SWEEP_DONE:
        {
            /*
             * Leave final duty applied, or uncomment to turn converter off.
             */
            // Set_Converter_Duty(0);
            break;
        }

        default:
        {
            iv_state = IV_SWEEP_DONE;
            break;
        }
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */

  char msg[] = "UART WORKING\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

  // Start timer base
  HAL_TIM_Base_Start(&htim1);

  // Start PWM on all 3 channels
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  Set_Pump_Speed(PUMP_OFF);  //
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);

  HAL_ADC_Start(&hadc1);  // Start ADC1 first
  HAL_ADC_Start(&hadc2);  // Start ADC2

  if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, adc_buffer, ADC_BUF_LEN) != HAL_OK)
  {
      Error_Handler();
  }
  HAL_ADC_Start(&hadc3); // Start ADC3

  // Initial duty setup

  Set_Manual_Duties();

  #if CONTROL_MODE == CONTROL_IV_SWEEP
  char csv_header[] = "\r\nPoint,Duty_percent,Vpanel_V,Ipanel_A,Ppanel_W,Samples\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)csv_header, strlen(csv_header), 100);
  #endif

  // find_calibration_values();
  // find_moisture_calibration();
  track_mpp_20_seconds(35);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE BEGIN 3 */

  #if CONTROL_MODE == CONTROL_MANUAL

      /*
       * Manual mode:
       * M1, M2, and M3 are set by MANUAL_M1_DUTY,
       * MANUAL_M2_DUTY, and MANUAL_M3_DUTY.
       */
      Set_Manual_Duties();
      Print_Panel_Sensors();

  #elif CONTROL_MODE == CONTROL_IV_SWEEP

      /*
       * IV sweep mode:
       * M1/M2 are swept through iv_duty_list.
       * M3 is kept off inside Run_IV_Sweep().
       */
      Run_IV_Sweep();

  #elif CONTROL_MODE == CONTROL_MPPT

      /*
       * MPPT mode:
       * MPPT controls M1/M2.
       * M3 stays manual unless ENABLE_SOIL_MOISTURE is set.
       */
      Run_MPPT();

      #if ENABLE_SOIL_MOISTURE
          Run_Soil_Moisture_Control();
      #else
          Set_Pump_Speed((PumpSpeed_t)MANUAL_M3_DUTY);
      #endif

  #elif CONTROL_MODE == CONTROL_SOIL_ONLY

      /*
       * Soil-only mode:
       * M1/M2 are manual.
       * M3 is controlled by soil moisture.
       */
      Set_Converter_Duty(MANUAL_M1_DUTY);
      Run_Soil_Moisture_Control();
      Print_Panel_Sensors();

  #elif CONTROL_MODE == CONTROL_MPPT_AND_SOIL

      /*
       * Combined final mode:
       * MPPT controls M1/M2.
       * Soil moisture controls M3.
       */
      Run_MPPT();
      Run_Soil_Moisture_Control();
      Print_Panel_Sensors();

  #else

      /*
       * Fallback safety mode.
       */
      Set_Converter_Duty(0);
      Set_Pump_Speed(PUMP_OFF);

  #endif

    /* USER CODE END 3 */
  }
    /* USER CODE END WHILE */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_2;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 839;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 420;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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
void find_calibration_values(void)
{
    const uint32_t calibration_time_ms = 30000;  // 30 seconds
    uint32_t start_time = HAL_GetTick();

    char msg[160];

    snprintf(msg, sizeof(msg),
             "\r\nStarting calibration. Ensure NO voltage and NO current input.\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    /* Reset accumulators */
    __disable_irq();
    v_sum = 0;
    i_sum = 0;
    sample_count = 0;
    __enable_irq();

    /* Wait while ADC DMA callbacks collect samples */
    while ((HAL_GetTick() - start_time) < calibration_time_ms)
    {
        HAL_Delay(1000);

        snprintf(msg, sizeof(msg),
                 "Calibrating... %lu / 30 seconds\r\n",
                 (unsigned long)((HAL_GetTick() - start_time) / 1000));
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    }

    // STOP THE TIMER AFTER CALIBRATION IF NEEDED
    // HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1);

    /* Copy final accumulated values safely */
    __disable_irq();
    uint64_t v_sum_copy = v_sum;
    uint64_t i_sum_copy = i_sum;
    uint32_t count_copy = sample_count;
    __enable_irq();

    if (count_copy == 0)
    {
        snprintf(msg, sizeof(msg),
                 "Calibration failed: no ADC samples collected.\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
        return;
    }

    float v_raw_avg = (float)v_sum_copy / (float)count_copy;
    float i_raw_avg = (float)i_sum_copy / (float)count_copy;

    float voltage_offset = (v_raw_avg / ADC_MAX_COUNTS) * ADC_VREF;
    float current_zero_v = (i_raw_avg / ADC_MAX_COUNTS) * ADC_VREF;

    snprintf(msg, sizeof(msg),
             "\r\nCalibration complete\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    snprintf(msg, sizeof(msg),
             "Voltage raw avg: %.2f counts\r\n", v_raw_avg);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    snprintf(msg, sizeof(msg),
             "Current raw avg: %.2f counts\r\n", i_raw_avg);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    snprintf(msg, sizeof(msg),
             "Use this VOLTAGE_OFFSET: %.4f\r\n", voltage_offset);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    snprintf(msg, sizeof(msg),
             "Use this ACS_ZERO_V: %.4f\r\n\r\n", current_zero_v);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

void find_moisture_calibration(void)
{
    char msg[200];

    snprintf(msg, sizeof(msg),
             "\r\n=== Moisture Sensor Calibration ===\r\n"
             "Step 1: Hold sensor in DRY AIR for 10 seconds (starts in 3s)...\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    HAL_Delay(3000);

    uint32_t air_sum = 0, air_n = 0;
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 10000) {
        air_sum += Read_Moisture();
        air_n++;
        HAL_Delay(50);
    }
    uint16_t air_avg = (uint16_t)(air_sum / air_n);

    snprintf(msg, sizeof(msg),
             "AIR average: %u counts\r\n\r\n"
             "Step 2: Submerge sensor in WATER. Starting in 10 seconds...\r\n",
             air_avg);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    HAL_Delay(10000);

    snprintf(msg, sizeof(msg),
             "Recording WATER values for 10 seconds...\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    uint32_t water_sum = 0, water_n = 0;
    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 10000) {
        water_sum += Read_Moisture();
        water_n++;
        HAL_Delay(50);
    }
    uint16_t water_avg = (uint16_t)(water_sum / water_n);

    snprintf(msg, sizeof(msg),
             "WATER average: %u counts\r\n\r\n"
             "=== Calibration Results ===\r\n"
             "MOISTURE_AIR_VALUE = %u\r\n"
             "MOISTURE_WATER_VALUE = %u\r\n"
             "Update these defines and rebuild.\r\n\r\n",
             water_avg, air_avg, water_avg);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

void track_mpp_20_seconds(uint8_t start_duty)
{
    char msg[240];

    if (start_duty > MPPT_DUTY_MAX) {
        start_duty = MPPT_DUTY_MAX;
    }
    if (start_duty < MPPT_DUTY_MIN) {
        start_duty = MPPT_DUTY_MIN;
    }

    /*
     * Reset MPPT state.
     */
    mppt_duty = start_duty;
    mppt_direction = 1;
    mppt_prev_power = 0.0f;
    mppt_prev_voltage = 0.0f;
    mppt_prev_current = 0.0f;
    mppt_initialized = 0;

    /*
     * Keep pump off during MPPT tracking test unless you specifically want
     * to test MPPT + pump operation together.
     */
    Set_Pump_Speed(PUMP_OFF);

    /*
     * Apply initial duty.
     */
    Set_Converter_Duty(mppt_duty);

    snprintf(msg, sizeof(msg),
             "\r\n=== Starting 20s MPPT Tracking Test ===\r\n"
             "Start duty: %u%%\r\n"
             "Duration: %lu ms\r\n\r\n",
             (unsigned int)mppt_duty,
             (unsigned long)MPPT_TRACK_DURATION_MS);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    snprintf(msg, sizeof(msg),
             "Time_ms,Duty_percent,Vpanel_V,Ipanel_A,Ppanel_W,Direction,NextDuty_percent,Samples\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    /*
     * Allow converter/panel to settle at starting duty.
     */
    HAL_Delay(MPPT_TRACK_SETTLE_MS);

    /*
     * Clear ADC accumulators before test starts.
     */
    __disable_irq();
    v_sum = 0;
    i_sum = 0;
    sample_count = 0;
    __enable_irq();

    uint32_t test_start = HAL_GetTick();

    float best_power = -1000000.0f;
    float best_voltage = 0.0f;
    float best_current = 0.0f;
    uint8_t best_duty = mppt_duty;
    uint32_t best_time_ms = 0;

    float final_v_sum = 0.0f;
    float final_i_sum = 0.0f;
    float final_p_sum = 0.0f;
    float final_d_sum = 0.0f;
    uint32_t final_count = 0;

    while ((HAL_GetTick() - test_start) < MPPT_TRACK_DURATION_MS)
    {
        HAL_Delay(MPPT_UPDATE_TIME_MS);

        float panel_voltage = 0.0f;
        float panel_current = 0.0f;
        float panel_power = 0.0f;
        uint32_t sample_n = 0;

        uint8_t duty_measured = mppt_duty;

        Read_Panel_Sensors(&panel_voltage,
                           &panel_current,
                           &panel_power,
                           NULL,
                           &sample_n);

        if (sample_n == 0)
        {
            continue;
        }

        uint32_t elapsed_ms = HAL_GetTick() - test_start;

        /*
         * Track best instantaneous power point during the test.
         */
        if (panel_power > best_power)
        {
            best_power = panel_power;
            best_voltage = panel_voltage;
            best_current = panel_current;
            best_duty = duty_measured;
            best_time_ms = elapsed_ms;
        }

        /*
         * P&O MPPT update.
         */
        if (!mppt_initialized)
        {
            mppt_prev_power = panel_power;
            mppt_prev_voltage = panel_voltage;
            mppt_prev_current = panel_current;
            mppt_initialized = 1;
        }
        else
        {
            float delta_p = panel_power - mppt_prev_power;

            if (delta_p > MPPT_POWER_DEADBAND_W)
            {
                /*
                 * Power increased: keep same perturb direction.
                 */
            }
            else if (delta_p < -MPPT_POWER_DEADBAND_W)
            {
                /*
                 * Power decreased: reverse perturb direction.
                 */
                mppt_direction = -mppt_direction;
            }
            else
            {
                /*
                 * Power change is small: likely noise. Keep direction unchanged.
                 */
            }

            mppt_prev_power = panel_power;
            mppt_prev_voltage = panel_voltage;
            mppt_prev_current = panel_current;
        }

        int16_t next_duty = (int16_t)mppt_duty +
                            ((int16_t)mppt_direction * (int16_t)MPPT_DUTY_STEP);

        if (next_duty > MPPT_DUTY_MAX)
        {
            next_duty = MPPT_DUTY_MAX;
            mppt_direction = -1;
        }
        else if (next_duty < MPPT_DUTY_MIN)
        {
            next_duty = MPPT_DUTY_MIN;
            mppt_direction = 1;
        }

        /*
         * Print measured operating point before applying next duty.
         */
        snprintf(msg, sizeof(msg),
                 "%lu,%u,%.3f,%.4f,%.4f,%d,%u,%lu\r\n",
                 (unsigned long)elapsed_ms,
                 (unsigned int)duty_measured,
                 panel_voltage,
                 panel_current,
                 panel_power,
                 (int)mppt_direction,
                 (unsigned int)next_duty,
                 (unsigned long)sample_n);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

        /*
         * Average final window, usually the final 5 seconds.
         * This is the best way to report the converged MPPT value.
         */
        if (elapsed_ms >= (MPPT_TRACK_DURATION_MS - MPPT_FINAL_AVG_WINDOW_MS))
        {
            final_v_sum += panel_voltage;
            final_i_sum += panel_current;
            final_p_sum += panel_power;
            final_d_sum += (float)duty_measured;
            final_count++;
        }

        /*
         * Apply next duty for the next MPPT sample.
         */
        mppt_duty = (uint8_t)next_duty;
        Set_Converter_Duty(mppt_duty);
    }

    snprintf(msg, sizeof(msg),
             "\r\n=== MPPT Tracking Test Complete ===\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    snprintf(msg, sizeof(msg),
             "Best point: t=%lu ms, D=%u%%, V=%.3f V, I=%.4f A, P=%.4f W\r\n",
             (unsigned long)best_time_ms,
             (unsigned int)best_duty,
             best_voltage,
             best_current,
             best_power);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

    if (final_count > 0)
    {
        float avg_v = final_v_sum / (float)final_count;
        float avg_i = final_i_sum / (float)final_count;
        float avg_p = final_p_sum / (float)final_count;
        float avg_d = final_d_sum / (float)final_count;

        snprintf(msg, sizeof(msg),
                 "Final %lu ms average: D=%.2f%%, V=%.3f V, I=%.4f A, P=%.4f W, points=%lu\r\n\r\n",
                 (unsigned long)MPPT_FINAL_AVG_WINDOW_MS,
                 avg_d,
                 avg_v,
                 avg_i,
                 avg_p,
                 (unsigned long)final_count);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    }
    else
    {
        snprintf(msg, sizeof(msg),
                 "Final average unavailable: no samples collected in final window.\r\n\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        Process_ADC_Buffer(&adc_buffer[0], ADC_BUF_LEN / 2);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        Process_ADC_Buffer(&adc_buffer[ADC_BUF_LEN / 2], ADC_BUF_LEN / 2);
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
