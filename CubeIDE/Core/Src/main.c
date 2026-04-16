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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/*
 * INSTRUMENTACIÓN 2025-2
 * 24/NOV/25
 * Built on CubeIDE 2.0.0 and CubeMX 6.16.0
 *
 * CARLOS DAVID CANCINO BEJARANO
 * NICOLÁS FORERO PORTELA
 */

#include "u8g2_custom_dma.h"
#include "bitmaps.h"
#include "EEPROM.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "BH1750.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
RTC_TimeTypeDef RTC_Time;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_BUZZER

#define NUM_CH 4

#define ADC_DEFAULT_MIN 200
#define ADC_DEFAULT_MAX 2500

// AC dimmer with TIM4
// CubeMX doesn't expose PWM mode 2. Instead of changing the mode manually, just
// undefine it (janky) and define it again with the needed PWM2 mode
#undef TIM_OCMODE_TIMING
#define TIM_OCMODE_TIMING TIM_OCMODE_PWM2

#define short_press_threshold 50
#define long_press_threshold 500

#define OUTPUT_RAMP_STEP_UP     5     // Speed of ramp up per cycle (0–255)
#define OUTPUT_RAMP_STEP_DOWN   5     // Speed of ramp down per cycle

#define ENCODER_DEFAULT_ACCEL 10

// Used for EncoderTarget_t variables
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0])) - 1

#define CH_OFF 0
#define CH_ON 1
#define CH_RAMP 2
#define CH_FULL 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
u8g2_t disp;

FATFS FatFs;
FRESULT FR_Status;
FIL file;
UINT bw;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Buzzer Tone
static uint32_t tone_end_time = 0;
static bool tone_active = 0;

RTC_HandleTypeDef hrtc;

uint16_t ADC1_BUFFER[NUM_CH];

char LCD_BUF[32];

// INPUT CHANNELS STRUCT
typedef struct {
    uint16_t value;       		// Latest averaged ADC value
    uint16_t percent;     		// Scaled value: 0–100
    uint16_t percent_ADC;		// Percentage to 0-4095
} ADC_Channel_t;

ADC_Channel_t ADC_IN[4] = {
    {0, 0, 0}, // IN01
    {0, 0, 0}, // IN02
    {0, 0, 0}, // IN03
    {0, 0, 0}  // IN04
};

#define CCR_MIN 0
#define CCR_MAX 8400

// OUTPUT CHANNELS STRUCT
typedef struct {
	uint8_t OUT;
	uint16_t CCR_OUT;
} Dimmer_Channel_t;

Dimmer_Channel_t CH_OUT[4] = {
		{0, 0}, // CH01
		{0, 0}, // CH02
		{0, 0}, // CH03
		{0, 0}, // CH04
};

uint32_t system_reset_count = 0;
uint32_t system_EEPROM_write_count = 0;

uint32_t time_pressed = 0;
uint8_t menu_active = 0;
// Selects current active row
uint8_t menu_config_selected_row = 0;
// Selects current configuration item
uint16_t menu_config_selected_item = 0;
// Selects current active channel
uint16_t menu_config_selected_channel = 0;

bool blink_cursor = false;
uint8_t blink_cursor_counter = 0;
uint8_t slow_text_alternate_counter = 0;
bool slow_text_alternate = true;

bool short_press_done = false;
bool long_press_done = false;
bool TANK_SW_INVERT = false;
bool TANK_HAS_WATER = false;
bool IsDataSaved = true;
bool WasDataSaved = false;

const uint16_t Watering_Period_time_array[] = {
    3, 5, 15, 30,
    60, 120, 180, 240, 300, 360, 420, 480, 540, 600,
    660, 720, 780, 840, 900, 960, 1020, 1080, 1140, 1200,
    1260, 1320, 1380, 1440, 1500, 1560, 1620, 1680, 1740, 1800,
    1860, 1920, 1980, 2040, 2100, 2160, 2220, 2280, 2340, 2400,
    2460, 2520, 2580, 2640, 2700, 2760, 2820, 2880, 2940, 3000,
    3060, 3120, 3180, 3240, 3300, 3360, 3420, 3480, 3540, 3600
};

const uint32_t Cooldown_Period_time_array[] = {
    5,          // 5 s
    15,         // 15 s
    30,         // 30 s
    60,         // 1 min
    300, 600, 900, 1200, 1500, 1800, 2100, 2400, 2700, 3000, 3300, 3600, // 5–60 min (5-min steps)
    7200, 10800, 14400, 18000, 21600, 25200, 28800, 32400, 36000, 39600, 43200 // 1–12 h (1-h steps)
};

const uint16_t Watering_Period_time_len = ARRAY_SIZE(Watering_Period_time_array);
const uint16_t Cooldown_Period_time_len = ARRAY_SIZE(Cooldown_Period_time_array);

// MENU CONFIG VARIABLES
// ------------------------------------ PAGE 1 ------------------------------------
// Enable Channels (0)
uint16_t Last_Enable_Channels_value[] = {0, 0, 0, 0};
const uint8_t Menu_Enable_Channels_idx = 0;
uint16_t Enable_Channels[] = {0, 0, 0, 0};
// Lower ADC Limit (1)
uint16_t Last_Lower_ADC_Limit_value[] = {0, 0, 0, 0};
const uint8_t Menu_Lower_ADC_Limit_idx = 1;
uint16_t Lower_ADC_Limit[] = {ADC_DEFAULT_MIN, ADC_DEFAULT_MIN, ADC_DEFAULT_MIN, ADC_DEFAULT_MIN};
// Upper ADC Limit (2)
uint16_t Last_Upper_ADC_Limit_value[] = {0, 0, 0, 0};
const uint8_t Menu_Upper_ADC_Limit_idx = 2;
uint16_t Upper_ADC_Limit[] = {ADC_DEFAULT_MAX, ADC_DEFAULT_MAX, ADC_DEFAULT_MAX, ADC_DEFAULT_MAX};
// Target Humidity (3)
uint16_t Last_Target_Humidity_value[] = {0, 0, 0, 0};
const uint8_t Menu_Target_Humidity_idx = 3;
uint16_t Target_Humidity[] = {0, 0, 0, 0};
// Target Radiance (4)
uint16_t Last_Target_Radiance_value[] = {0, 0, 0, 0};
const uint8_t Menu_Target_Radiance_idx = 4;
uint16_t Target_Radiance[] = {500, 500, 500, 500};
// ------------------------------------ PAGE 2 ------------------------------------
// Watering Period (5)
uint16_t Last_Watering_Period_value[] = {0, 0, 0, 0};
const uint8_t Menu_Watering_Period_idx = 5;
uint16_t Watering_Period[] = {0, 0, 0, 0};
uint32_t current_watering_time[NUM_CH];
// Cooldown Period (6)
uint16_t Last_Cooldown_Period_value[] = {0, 0, 0, 0};
const uint8_t Menu_Cooldown_Period_idx = 6;
uint16_t Cooldown_Period[] = {0, 0, 0, 0};
uint32_t current_cooldown_time[NUM_CH];
// ADC  Hysteresis (7)
uint16_t Last_ADC__Hysteresis_value[] = {0, 0, 0, 0};
const uint8_t Menu_ADC__Hysteresis_idx = 7;
uint16_t ADC__Hysteresis[] = {1, 1, 1, 1};
uint16_t ADC_Lower_Hysteresis_value[] = {0, 0, 0, 0};
uint16_t ADC_Upper_Hysteresis_value[] = {0, 0, 0, 0};
// LUX  Hysteresis (8)
uint16_t Last_LUX__Hysteresis_value[] = {0, 0, 0, 0};
const uint8_t Menu_LUX__Hysteresis_idx = 8;
uint16_t LUX__Hysteresis[] = {1, 1, 1, 1};
uint16_t LUX_Lower_Hysteresis_value[] = {0, 0, 0, 0};
uint16_t LUX_Upper_Hysteresis_value[] = {0, 0, 0, 0};
// Max Power Level (9)
uint16_t Last_Max_Power_Level_value[] = {0, 0, 0, 0};
const uint8_t Menu_Max_Power_Level_idx = 9;
uint16_t Max_Power_Level[] = {255, 255, 255, 255};
// ------------------------------------ PAGE 3 ------------------------------------
// Backlight Timer (10)
const uint8_t Menu_Backlight_Timer_idx = 10;
// Adjust RTC Time (11)
const uint8_t Menu_Adjust_RTC_Time_idx = 11;
// N/A (12)
// N/A (13)
// System Counters (14)
const uint8_t Menu_System_Counters_idx = 14;

typedef struct {
    uint16_t *current;  // base pointer to uint16_t[4]
    uint16_t *last; // base pointer to uint16_t[4]
    uint16_t min;
    uint16_t max;
    uint8_t  accel;
} MenuArrayInfo;

MenuArrayInfo menu_info[] = {
		// ------------------------------------ PAGE 1 ------------------------------------
		{Enable_Channels, Last_Enable_Channels_value, 0, 3, 0},
		{Lower_ADC_Limit, Last_Lower_ADC_Limit_value, 0, 4095, ENCODER_DEFAULT_ACCEL},
		{Upper_ADC_Limit, Last_Upper_ADC_Limit_value, 0, 4095, ENCODER_DEFAULT_ACCEL},
		{Target_Humidity, Last_Target_Humidity_value, 0, 100, 1},
		{Target_Radiance, Last_Target_Radiance_value, 0, 65535, 20},
		// ------------------------------------ PAGE 2 ------------------------------------
		{Watering_Period, Last_Watering_Period_value, 0, Watering_Period_time_len, 1},
		{Cooldown_Period, Last_Cooldown_Period_value, 0, Cooldown_Period_time_len, 1},
		{ADC__Hysteresis, Last_ADC__Hysteresis_value, 1, 25, 1},
		{LUX__Hysteresis, Last_LUX__Hysteresis_value, 1, 25, 1},
		{Max_Power_Level, Last_Max_Power_Level_value, 0, 255, 1}
};

#define MENU_INFO_COUNT   (sizeof(menu_info)/sizeof(menu_info[0]))
#define CHANNELS_PER_ITEM 4

// Encoder loop
typedef struct {
	uint16_t *value;     // Pointer to the variable being adjusted
	uint16_t min;        // Minimum allowed value
	uint16_t max;        // Maximum allowed value
	uint8_t accel;       // Acceleration multiplier
} EncoderTarget_t;

EncoderTarget_t encoder_targets[3 + MENU_INFO_COUNT * CHANNELS_PER_ITEM];

const uint8_t EncoderTarget_menu_config_selected_item_idx = 1;
const uint8_t EncoderTarget_menu_config_selected_channel_idx = 2;

uint8_t selected_target = 0;

// Screen menu
const char *PLANT_names[] = {"PLANT 1", "PLANT 2", "PLANT 3", "PLANT 4"};
const char *CHxx_names[] = {"CH 01", "CH 02", "CH 03", "CH 04"};
const char *config_menu_iten_names[] = {
		"ENABLE CHANNELS", "LOWER ADC LIMIT", "UPPER ADC LIMIT", "TARGET HUMIDITY", "TARGET RADIANCE", 	// 1st menu page
		"WATERING PERIOD", "COOLDOWN PERIOD", "ADC  HYSTERESIS", "LUX  HYSTERESIS", "MAX POWER LEVEL", 	 // 2nd menu page
		"BACKLIGHT TIMER", "ADJUST RTC TIME", "               ", "               ", "SYSTEM COUNTERS",	// 3rd menu page
};
const char *config_menu_Enable_Channels_names[] = {"OFF", "ON", "RAMP", "FULL"};

// BH1750
float BH1750_lux = 0.0f;

#define MENU_ENCODER_BASE_INDEX 3
#define MENU_ENCODER_INDEX(item, channel) \
    (MENU_ENCODER_BASE_INDEX + (item)*CHANNELS_PER_ITEM + (channel))

volatile bool watering_loop_flag = false;
uint8_t Watering_State[] = {0,0,0,0};
enum { CH_IDLE = 0, CH_WATERING = 1, CH_COOLDOWN = 2 };

#define SD_BUFLEN 64
char SD_BUF[SD_BUFLEN];
bool SD_Present = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void BuildEncoderTargetTable(void)
{
    uint16_t idx = 0;

    encoder_targets[idx++] = (EncoderTarget_t){0, 0, 0, 0}; // Dummy entry
    encoder_targets[idx++] = (EncoderTarget_t){&menu_config_selected_item,    0, 9, 0};
    encoder_targets[idx++] = (EncoderTarget_t){&menu_config_selected_channel, 0, 3,  0};

    // Auto-generate all 4-per-item entries
    for (uint16_t m = 0; m < MENU_INFO_COUNT; m++) {
        for (uint8_t ch = 0; ch < CHANNELS_PER_ITEM; ch++) {

            encoder_targets[idx].value = &menu_info[m].current[ch];
            encoder_targets[idx].min   = menu_info[m].min;
            encoder_targets[idx].max   = menu_info[m].max;
            encoder_targets[idx].accel = menu_info[m].accel;

            idx++;
        }
    }
}

void Beep(uint32_t Duration, bool Block) {
	TIM3->CCR3 = (TIM3->ARR >> 1);   // 50% duty

	if (Block)
	{
		// Blocking mode
		HAL_Delay(Duration);
		TIM3->CCR3 = 0;   // stop tone
	}
	else
	{
		// Non-blocking mode
		tone_end_time = HAL_GetTick() + Duration;
		tone_active = true;
	}
}

void Buzzer_loop(void) {
	if (tone_active && HAL_GetTick() >= tone_end_time) {
		TIM3->CCR3 = 0;   // stop tone
		tone_active = false;
	}
}

void Counters_Load(void) {
	uint8_t buf[8];
	if (EEPROM_Read(0, 0, buf, sizeof(buf)) == HAL_OK) {
		system_reset_count = ((uint32_t)buf[0] << 24) |
				((uint32_t)buf[1] << 16) |
				((uint32_t)buf[2] << 8)  |
				((uint32_t)buf[3]);

		system_EEPROM_write_count = ((uint32_t)buf[4] << 24) |
				((uint32_t)buf[5] << 16) |
				((uint32_t)buf[6] << 8)  |
				((uint32_t)buf[7]);
        system_reset_count++;
    }
}

void WriteCounter_Save(void) {
	system_EEPROM_write_count +=2;
    uint8_t buf[4] = {
        (uint8_t)(system_EEPROM_write_count >> 24),
        (uint8_t)(system_EEPROM_write_count >> 16),
        (uint8_t)(system_EEPROM_write_count >> 8),
        (uint8_t)(system_EEPROM_write_count)
    };
    EEPROM_Write(0, 4, buf, sizeof(buf)); // Offset 4 = after reset counter
}

void ResetCounter_Save(void) {
	uint8_t buf[4] = {
			(uint8_t)(system_reset_count >> 24),
			(uint8_t)(system_reset_count >> 16),
			(uint8_t)(system_reset_count >> 8),
			(uint8_t)(system_reset_count)
	};
	EEPROM_Write(0, 0, buf, sizeof(buf));
	WriteCounter_Save();
}

void Unpack4xU16_Bounded(uint16_t *dst, uint8_t *src, uint16_t minv, uint16_t maxv)
{
    for (int i = 0; i < NUM_CH; i++) {
        uint16_t val = (src[i*2] << 8) | src[i*2 + 1];

        if (val > maxv) val = maxv;
        if (val < minv) val = minv;

        dst[i] = val;
    }
}

void Pack4xU16(uint16_t *src, uint8_t *dst)
{
    for (int i = 0; i < NUM_CH; i++) {
        dst[i*2]   = (src[i] >> 8) & 0xFF;
        dst[i*2+1] = (src[i] & 0xFF);
    }
}

void EEPROM_load_all_menu_data(void)
{
    uint8_t buf[8];

    for (int page = 0; page < MENU_INFO_COUNT; page++) {

        if (EEPROM_Read(page + 1, 0, buf, sizeof(buf)) == HAL_OK) {

            Unpack4xU16_Bounded(
                menu_info[page].current,
                buf,
                menu_info[page].min,
                menu_info[page].max
            );
        }

        HAL_Delay(5);
    }
}

void EEPROM_save_all_menu_data(void)
{
    uint8_t buf[8];

    for (int page = 0; page < MENU_INFO_COUNT; page++) {

        Pack4xU16(menu_info[page].current, buf);

        EEPROM_Write(page + 1, 0, buf, sizeof(buf));
        HAL_Delay(5);
    }
    system_EEPROM_write_count += MENU_INFO_COUNT;
    WriteCounter_Save();
    HAL_Delay(5);
}

void EEPROM_save_current_menu_item(void)
{
	if (menu_config_selected_item < MENU_INFO_COUNT) {
		uint8_t buf[8];
		Pack4xU16(menu_info[menu_config_selected_item].current, buf);
		EEPROM_Write(menu_config_selected_item + 1, 0, buf, sizeof(buf));
		WriteCounter_Save();
	}
}

void compare_current_to_new_data() {
	IsDataSaved = true;
	// Check if last data is different from current, enabling EEPROM saving if data has been modified
	for (int i = 0; i < MENU_INFO_COUNT; i++) {
		for (int j = 0; j < CHANNELS_PER_ITEM; j++) {
			if (menu_info[i].last[j] != menu_info[i].current[j]) {
				IsDataSaved = false;
				// Return early, no more checks needed
				return;
			}
		}
	}
}

void update_last_data() {
	// Update last values with the current ones
	for (int i = 0; i < MENU_INFO_COUNT; i++) {
		for (int j = 0; j < CHANNELS_PER_ITEM; j++) {
			menu_info[i].last[j] = menu_info[i].current[j];
		}
	}
}

void reset_slow_text_alternate() {
	slow_text_alternate_counter = 0;
	slow_text_alternate = true;
}

void map_ADC_input_to_percentage() {
	for (int ch = 0; ch < NUM_CH; ch++) {
		if (Upper_ADC_Limit[ch] == Lower_ADC_Limit[ch]) {
			ADC_IN[ch].percent = 0; // avoid division by zero
			continue;
		}
		if (ADC_IN[ch].value <= Lower_ADC_Limit[ch]) {
			ADC_IN[ch].percent = 0;
			continue;
		} else if (ADC_IN[ch].value >= Upper_ADC_Limit[ch]) {
			ADC_IN[ch].percent = 100;
			continue;
		}
		uint64_t temp = (uint64_t)(ADC_IN[ch].value - Lower_ADC_Limit[ch]) * 100;
		temp /= (Upper_ADC_Limit[ch] - Lower_ADC_Limit[ch]);
		ADC_IN[ch].percent = (uint16_t)(temp + 0);
	}
}

void map_percentage_to_ADC_value(uint8_t ch)
{
    if (Upper_ADC_Limit[ch] == Lower_ADC_Limit[ch]) {
        ADC_IN[ch].percent_ADC = Lower_ADC_Limit[ch];
        return;
    }
    if (Target_Humidity[ch] <= 0) {
        ADC_IN[ch].percent_ADC = Lower_ADC_Limit[ch];
        return;
    }
    if (Target_Humidity[ch] >= 100) {
        ADC_IN[ch].percent_ADC = Upper_ADC_Limit[ch];
        return;
    }
    uint64_t temp = (uint64_t)(Target_Humidity[ch]) * (uint64_t)(Upper_ADC_Limit[ch] - Lower_ADC_Limit[ch]);
    temp /= 100;

    ADC_IN[ch].percent_ADC = (uint16_t)(Lower_ADC_Limit[ch] + temp);
}

void Inputs_loop() {
	static uint32_t last_inputs_reading = 0;
	if (HAL_GetTick() - last_inputs_reading >= 10) {
		last_inputs_reading = HAL_GetTick();

		if (HAL_GPIO_ReadPin(TANK_SW_GPIO_Port, TANK_SW_Pin) == TANK_SW_INVERT) {
			TANK_HAS_WATER = true;
		} else {
			TANK_HAS_WATER = false;
		}

		if (!HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin)) {
			time_pressed += 10;

			if (time_pressed > long_press_threshold && !long_press_done) {
				// Long press action

				if (menu_config_selected_row == 0) {
					menu_active += 1;

					if (menu_active > 1) {
						menu_active = 0;
					}

					if (menu_active) {
						update_last_data();

						selected_target = EncoderTarget_menu_config_selected_item_idx;
						menu_config_selected_row = 0;
					} else {
						selected_target = 0;
					}

					menu_config_selected_channel = 0;

				} else if (menu_config_selected_row == 1) {
					// Save data

					selected_target = EncoderTarget_menu_config_selected_item_idx;
					menu_config_selected_row = 0;

					if (!IsDataSaved) {
						EEPROM_save_current_menu_item();
						WasDataSaved = true;
						reset_slow_text_alternate();

						update_last_data();
					}
				} else if (menu_config_selected_row == 2) {
					// Value changing menu row, long click to reset to previous value
					if (menu_config_selected_item < MENU_INFO_COUNT) {
						menu_info[menu_config_selected_item].current[menu_config_selected_channel] = menu_info[menu_config_selected_item].last[menu_config_selected_channel];
					}
				}

				Beep(50, 0);

				long_press_done = true;
			}
			short_press_done = false;
		} else {
			if (time_pressed > short_press_threshold && time_pressed < long_press_threshold && !short_press_done) {
				// Short press action
				if (menu_active) {
					menu_config_selected_row +=1;

					if (menu_config_selected_row > 2) {
						menu_config_selected_row = 1;
					}

					reset_slow_text_alternate();

					if (menu_config_selected_row == 1) {
						selected_target = EncoderTarget_menu_config_selected_channel_idx;
					} else if (menu_config_selected_row == 2) {
						if (menu_config_selected_item < MENU_INFO_COUNT) {
							// Select correct encoder target variable with a macro to avoid hard-coding indexes
							selected_target = MENU_ENCODER_INDEX(menu_config_selected_item, menu_config_selected_channel);
						}
					}

					Beep(10, 0);

				}
				short_press_done = true;
			}
			time_pressed = 0;
			long_press_done = false;
		}
	}
}

void OUTPUT_MAP_TO_DIMMER(Dimmer_Channel_t *OUT) {
	for (int i = 0; i < NUM_CH; i++) {
		// Map 0–255 → CCR_MAX–CCR_MIN (inverted)
		uint32_t temp = (uint32_t)(OUT[i].OUT) * (CCR_MAX - CCR_MIN);
		temp = temp / 255U;
		// Invert mapping: 0 → CCR_MAX, 255 → CCR_MIN
		uint16_t mapped = CCR_MAX - (uint16_t)temp;

		// Store and write to timer
		OUT[i].CCR_OUT = mapped;
	}

	// Assign mapped CCRs to timer channels
	TIM4->CCR1 = OUT[0].CCR_OUT;
	TIM4->CCR2 = OUT[1].CCR_OUT;
	TIM4->CCR3 = OUT[2].CCR_OUT;
	TIM4->CCR4 = OUT[3].CCR_OUT;
}

void Outputs_loop(void) {
    static uint32_t last_outputs_loop = 0;
    static uint16_t ramp_value[4] = {0, 0, 0, 0};

    #define RAMP_STEP 2

    if (HAL_GetTick() - last_outputs_loop < 10) return;
    last_outputs_loop = HAL_GetTick();

    for (int ch = 0; ch < NUM_CH; ch++) {

        uint16_t target = 0;

        if (TANK_HAS_WATER && !menu_active) {
            switch (Enable_Channels[ch]) {

            case CH_OFF:
                target = 0;
                break;
            case CH_ON:
                target = (Watering_State[ch] == CH_WATERING) ? Max_Power_Level[ch] : 0;
                break;
            case CH_RAMP:
                if (Watering_State[ch] == CH_WATERING)
                    target = Max_Power_Level[ch];
                else
                    target = 0;
                break;
            case CH_FULL:
            	target = Max_Power_Level[ch];
            	break;
            }
        } else {
        	target = 0;
        }

        if (Enable_Channels[ch] == CH_RAMP) {
            if (ramp_value[ch] < target) {
                uint16_t newv = ramp_value[ch] + RAMP_STEP;
                ramp_value[ch] = (newv > target ? target : newv);
            } else if (ramp_value[ch] > target) {
                uint16_t newv = ramp_value[ch] - RAMP_STEP;
                ramp_value[ch] = (newv < target ? target : newv);
            }
            CH_OUT[ch].OUT = ramp_value[ch];
        } else {
            CH_OUT[ch].OUT = target;
            ramp_value[ch] = target;
        }
    }

    // Apply outputs to dimmers
    OUTPUT_MAP_TO_DIMMER(CH_OUT);
}

void calculate_ADC_hysteresis_thresholds() {
	for (int ch = 0; ch < NUM_CH; ch++) {
        float h = ADC__Hysteresis[ch] / 100.0f;
        float v = (float)ADC_IN[ch].value;
        float v_low  = v * (1.0f - h);
        if (v_low > 4095.0f) v_low = 4095;
        float v_high = v * (1.0f + h);
        if (v_high > 4095.0f) v_high = 4095;

        ADC_Lower_Hysteresis_value[ch] = v_low;
        ADC_Upper_Hysteresis_value[ch] = v_high;
	}
}

void calculate_LUX_hysteresis_thresholds() {
    for (int ch = 0; ch < NUM_CH; ch++) {

        float h = LUX__Hysteresis[ch] / 100.0f;
        float v_low  = BH1750_lux * (1.0f - h);
        if (v_low > 65535.0f) v_low = 65535;
        float v_high = BH1750_lux * (1.0f + h);
        if (v_high > 65535.0f) v_high = 65535;

        LUX_Lower_Hysteresis_value[ch] = v_low;
        LUX_Upper_Hysteresis_value[ch] = v_high;
    }
}

void Read_BH1750() {
	static uint32_t last_BH1750_reading = 0;
	if (HAL_GetTick() - last_BH1750_reading >= 500) { // Must be higher than 160ms
		last_BH1750_reading = HAL_GetTick();

		BH1750_ReadLight(&BH1750_lux);

		calculate_LUX_hysteresis_thresholds();
	}
}

void Display_loop() {
	static uint32_t last_disp_refresh = 0;

	// Display must update at 24Hz or less, to avoid breaking the SPI DMA transfer
	if (HAL_GetTick() - last_disp_refresh >= 60) {
		last_disp_refresh = HAL_GetTick();

		blink_cursor_counter++;
		if (blink_cursor_counter >= 10) {
			blink_cursor = !blink_cursor;
			blink_cursor_counter = 0;
		}

		slow_text_alternate_counter++;
		if (slow_text_alternate_counter >= 40) {
			slow_text_alternate = !slow_text_alternate;
			slow_text_alternate_counter = 0;
		}

		if (!slow_text_alternate) {
			WasDataSaved = false;
		}

		// Clear display buffer
		u8g2_ClearBuffer(&disp);

		// Draw frame
		u8g2_DrawFrame(&disp, 0, 0, 128, 64);

		u8g2_DrawHLine(&disp, 0, 8, 128);

		HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BIN);

		if (menu_active == 0) {
			// Top bar data
			u8g2_DrawVLine(&disp, 93, 1, 7);
			sprintf(LCD_BUF, "%02u:%02u:%02u", RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
			u8g2_DrawStr(&disp, 95, 7, LCD_BUF);
			u8g2_DrawVLine(&disp, 59, 1, 7);
			sprintf(LCD_BUF, "%5iLUX", (uint16_t)BH1750_lux);
			u8g2_DrawStr(&disp, 61, 7, LCD_BUF);

			// Base parameters
			const uint8_t y0 = 9;
			const uint8_t h  = 54;
			const uint8_t plant_y0 = 17;
			const uint8_t info_x0[] = {3, 34, 66, 97};

			// Segment boundaries
			u8g2_DrawVLine(&disp, 1, y0, h);
			u8g2_DrawVLine(&disp, 32, y0, h);
			u8g2_DrawBox(&disp, 63, y0, 2, h);
			u8g2_DrawVLine(&disp, 95, y0, h);
			u8g2_DrawVLine(&disp, 126, y0, h);

			for (int ch = 0; ch < NUM_CH; ch++) {
				u8g2_DrawXBM(&disp, info_x0[ch], plant_y0, plant_width, plant_height, plant_bits);

				if (Enable_Channels[ch] == CH_OFF) {
					u8g2_DrawStr(&disp, info_x0[ch] + 9, 15, "OFF");
				} else if(Enable_Channels[ch] == CH_FULL) {
					u8g2_DrawStr(&disp, info_x0[ch], 15, "FULL ON");
				} else {
					u8g2_SetFont(&disp, u8g2_font_5x8_tf);
					sprintf(LCD_BUF, "%03i", ADC_IN[ch].percent);
					u8g2_DrawStr(&disp, info_x0[ch]+6, plant_y0+31, LCD_BUF);
				}

				if (Watering_State[ch] == CH_WATERING && TANK_HAS_WATER) {
					if (blink_cursor) {
						u8g2_DrawCircle(&disp, info_x0[ch]+5, plant_y0+17, 2, U8G2_DRAW_ALL);
						u8g2_DrawCircle(&disp, info_x0[ch]+22, plant_y0+17, 2, U8G2_DRAW_ALL);
					}
				}

				uint32_t t = 0;

				u8g2_SetFont(&disp, u8g2_font_4x6_tr);
				if (Watering_State[ch] == CH_IDLE) {
					u8g2_DrawStr(&disp, info_x0[ch], 62, PLANT_names[ch]);
				} else if (Watering_State[ch] == CH_WATERING) {
					u8g2_DrawStr(&disp, info_x0[ch], 62, "DOUSING");
					t = current_watering_time[ch];
				} else if (Watering_State[ch] == CH_COOLDOWN) {
					u8g2_DrawStr(&disp, info_x0[ch], 62, "WAITING");
					t = current_cooldown_time[ch];
				}

				if (t >= 3600) {
				    // HH:MM format
				    uint32_t hours = t / 3600;
				    uint32_t minutes = (t % 3600) / 60;

				    sprintf(LCD_BUF, "%02luH:%02luM", hours, minutes);
				}
				else {
				    // MM:SS format
					uint32_t minutes = t / 60;
					uint32_t seconds = t % 60;

					sprintf(LCD_BUF, "%02luM:%02luS", minutes, seconds);
				}

				if (Watering_State[ch] != CH_IDLE && (blink_cursor || TANK_HAS_WATER)) {
					u8g2_DrawStr(&disp, info_x0[ch], 15, LCD_BUF);
				}
			}

			if (blink_cursor && !TANK_HAS_WATER) {
				u8g2_DrawStr(&disp, 2, 7, "LOW WATER LVL");
			}

		} else if (menu_active == 1) {
			compare_current_to_new_data();

			if (menu_config_selected_row != 0 && !IsDataSaved) {
				u8g2_DrawBox(&disp, 73, 1, 54, 7);
				u8g2_SetDrawColor(&disp, 0);
				u8g2_DrawStr(&disp, 79, 7, " NOT SAVED");
				u8g2_SetDrawColor(&disp, 1);
			}

			if (menu_config_selected_item < MENU_INFO_COUNT) {
				u8g2_SetFont(&disp, u8g2_font_4x6_tr);

				u8g2_DrawStr(&disp, 2, 7, "IRRIGATION CONFIG");

				u8g2_DrawStr(&disp, 75, 16, "CHANNEL");
				u8g2_DrawVLine(&disp, 103, 9, 46);
				u8g2_DrawStr(&disp, 106, 16, "VALUE");

				u8g2_DrawHLine(&disp, 74, 18, 53);

				for (int i = 0; i < NUM_CH; i++) {
					u8g2_SetFont(&disp, u8g2_font_5x8_tf);

					if ((i == menu_config_selected_channel && menu_config_selected_row == 1 && blink_cursor) || (i == menu_config_selected_channel && menu_config_selected_row == 2)) {
						u8g2_SetDrawColor(&disp, 1);
						u8g2_DrawBox(&disp, 74, 19+i*9, 29, 8);
						u8g2_SetDrawColor(&disp, 0);
					}
					u8g2_DrawStr(&disp, 76, 26+i*9, CHxx_names[i]);
					u8g2_SetDrawColor(&disp, 1);

					if (i == menu_config_selected_channel && menu_config_selected_row == 2 && blink_cursor) {
						u8g2_SetDrawColor(&disp, 1);
						u8g2_DrawBox(&disp, 104, 19+i*9, 27, 8);
						u8g2_SetDrawColor(&disp, 0);
					}

					// Draw per channel current data
					// ------------------------------------ PAGE 1 ------------------------------------
					if (menu_config_selected_item == Menu_Enable_Channels_idx) {
						sprintf(LCD_BUF, "%s", config_menu_Enable_Channels_names[Enable_Channels[i]]);
					} else if (menu_config_selected_item == Menu_Lower_ADC_Limit_idx) {
						sprintf(LCD_BUF, "%4i", Lower_ADC_Limit[i]);
					} else if (menu_config_selected_item == Menu_Upper_ADC_Limit_idx) {
						sprintf(LCD_BUF, "%4i", Upper_ADC_Limit[i]);
					} else if (menu_config_selected_item == Menu_Target_Humidity_idx) {
						sprintf(LCD_BUF, "%3u%%", Target_Humidity[i]);
						map_percentage_to_ADC_value(i);
					} else if (menu_config_selected_item == Menu_Target_Radiance_idx) {
						if (Target_Radiance[i] > 9999) {
							u8g2_SetFont(&disp, u8g2_font_4x6_tr);
						}
						sprintf(LCD_BUF, "%4u", Target_Radiance[i]);
					}
					// ------------------------------------ PAGE 2 ------------------------------------
					else if (menu_config_selected_item == Menu_Watering_Period_idx) {
						uint16_t t = Watering_Period_time_array[Watering_Period[i]];
						if (t < 60) {
							sprintf(LCD_BUF, "%3is", t);
						} else if (t < 3600) {
							sprintf(LCD_BUF, "%3im", (t / 60));
						} else {
							sprintf(LCD_BUF, "%3ih", (t / 3600));
						}
					} else if (menu_config_selected_item == Menu_Cooldown_Period_idx) {
						uint32_t t = Cooldown_Period_time_array[Cooldown_Period[i]];
						if (t < 60) {
							sprintf(LCD_BUF, "%3lus", t);
						} else if (t < 3600) {
							sprintf(LCD_BUF, "%3lum", (t / 60));
						} else {
							sprintf(LCD_BUF, "%3luh", (t / 3600));
						}
					} else if (menu_config_selected_item == Menu_ADC__Hysteresis_idx) {
						u8g2_DrawGlyph(&disp, 106, 26 + i*9, 0x00b1);
						sprintf(LCD_BUF, " %2u%%", ADC__Hysteresis[i]);
					} else if (menu_config_selected_item == Menu_LUX__Hysteresis_idx) {
						u8g2_DrawGlyph(&disp, 106, 26 + i*9, 0x00b1);
						sprintf(LCD_BUF, " %2u%%", LUX__Hysteresis[i]);
					} else if (menu_config_selected_item == Menu_Max_Power_Level_idx) {
						sprintf(LCD_BUF, "%4u", Max_Power_Level[i]);
					}

					u8g2_DrawStr(&disp, 106, 26+i*9, LCD_BUF);

					u8g2_SetDrawColor(&disp, 1);

					u8g2_DrawHLine(&disp, 74, 27+i*9, 53);
				}

			} else {
				u8g2_DrawStr(&disp, 2, 7, "SYSTEM CONFIG");

				// Backlight Timer (10)

				// Adjust RTC Time (11)}
				if (menu_config_selected_item == Menu_Adjust_RTC_Time_idx) {
					u8g2_SetFont(&disp, u8g2_font_6x12_tr);
					sprintf(LCD_BUF, "%02u:%02u:%02u", RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
					u8g2_DrawStr(&disp, 77, 20, LCD_BUF);
				}
				// N/A (12)
				// N/A (13)
				// System Counters (14)
				else if (menu_config_selected_item == Menu_System_Counters_idx) {
					u8g2_SetFont(&disp, u8g2_font_5x8_tf);
					sprintf(LCD_BUF, "RESET:%lu", system_reset_count);
					u8g2_DrawStr(&disp, 75, 20, LCD_BUF);
					sprintf(LCD_BUF, "WRITE:%lu", system_EEPROM_write_count);
					u8g2_DrawStr(&disp, 75, 30, LCD_BUF);
				}

			}

			// Separator between options and item

			u8g2_DrawVLine(&disp, 73, 9, 46);

			u8g2_SetFont(&disp, u8g2_font_4x6_tr);

			if (!WasDataSaved) {
				bool DrawText = true;
				uint8_t offset = 0;

				if (menu_config_selected_row == 0) {
					if (slow_text_alternate) {
						sprintf(LCD_BUF, "Short click to select an option");
					} else {
						sprintf(LCD_BUF, "Long click to go back");
					}
				} else if (menu_config_selected_row == 1) {
					if (slow_text_alternate) {
						sprintf(LCD_BUF, "Short click to select an input");
					} else {
						sprintf(LCD_BUF, "Long click to save current data");
					}

				} else if (menu_config_selected_row == 2) {
					// Draw bottom helper info
					// ------------------------------------ PAGE 1 ------------------------------------
					if (menu_config_selected_item == Menu_Enable_Channels_idx) {
						sprintf(LCD_BUF, "Selects OFF, ON, RAMP or FULL");
					} else if (menu_config_selected_item == Menu_Lower_ADC_Limit_idx) {
						offset = 1;
						u8g2_SetFont(&disp, u8g2_font_5x8_tf);
						sprintf(LCD_BUF, "VAL:%4i  LAST MIN:%4i", ADC_IN[menu_config_selected_channel].value, Last_Lower_ADC_Limit_value[menu_config_selected_channel]);
					} else if (menu_config_selected_item == Menu_Upper_ADC_Limit_idx) {
						offset = 1;
						u8g2_SetFont(&disp, u8g2_font_5x8_tf);
						sprintf(LCD_BUF, "VAL:%4i  LAST MAX:%4i", ADC_IN[menu_config_selected_channel].value, Last_Upper_ADC_Limit_value[menu_config_selected_channel]);
					}
					// ------------------------------------ PAGE 2 ------------------------------------
					else if (menu_config_selected_item == Menu_ADC__Hysteresis_idx) {
						offset = 1;
						u8g2_SetFont(&disp, u8g2_font_5x8_tf);
						sprintf(LCD_BUF, "CURR:%4i LO:%4i HI:%4i", ADC_IN[menu_config_selected_channel].value, ADC_Lower_Hysteresis_value[menu_config_selected_channel], ADC_Upper_Hysteresis_value[menu_config_selected_channel]);
					} else if (menu_config_selected_item == Menu_LUX__Hysteresis_idx) {
						offset = 1;
						sprintf(LCD_BUF, "CURR:%5i LO:%5i HI:%5i", (uint16_t) BH1750_lux, LUX_Lower_Hysteresis_value[menu_config_selected_channel], LUX_Upper_Hysteresis_value[menu_config_selected_channel]);
					}
					// ------------------------------------ PAGE 3 ------------------------------------
					else {
						DrawText = false;
					}
				}
				if (DrawText) {
					u8g2_DrawStr(&disp, 2, 61 + offset, LCD_BUF);
				}

			} else {
				u8g2_SetFont(&disp, u8g2_font_4x6_tr);
				u8g2_DrawBox(&disp, 1, 55, 126, 8);
				u8g2_SetDrawColor(&disp, 0);
				u8g2_DrawStr(&disp, 2, 61, "Saving configuration to EEPROM");
				u8g2_SetDrawColor(&disp, 1);
			}

			int start_index = 0;

			u8g2_SetFont(&disp, u8g2_font_4x6_tr);

			// Determine which page of 5 items to show
			if (menu_config_selected_item < 5)
				start_index = 0;
			else if (menu_config_selected_item < 10)
				start_index = 5;
			else start_index = 10;

			for (int i = 0; i < 5; i++) {
				int item_index = start_index + i;

				// Highlight active option
				if ((item_index == menu_config_selected_item && menu_config_selected_row != 0) ||
						(i == menu_config_selected_item - start_index && menu_config_selected_row == 0 && blink_cursor)) {
					u8g2_SetDrawColor(&disp, 1);
					u8g2_DrawBox(&disp, 3, 12 + i * 8, 61, 7);
					u8g2_SetDrawColor(&disp, 0);
				}

				// Draw item text
				u8g2_DrawStr(&disp, 4, 18 + i * 8, config_menu_iten_names[item_index]);
				u8g2_SetDrawColor(&disp, 1);
			}

			// Draw scroll indicators
			// This font increases code size by ~8kB lol
			u8g2_SetFont(&disp, u8g2_font_6x12_t_symbols);
			if (menu_config_selected_item < 5) {
				// Down arrow (more items below)
				u8g2_DrawGlyph(&disp, 67, 52, 0x2193);
			} else if (menu_config_selected_item >= 5 && menu_config_selected_item <= 9) {
				// Up arrow (more items above)
				u8g2_DrawGlyph(&disp, 67, 18, 0x2191);
				// Down arrow (more items below)
				u8g2_DrawGlyph(&disp, 67, 52, 0x2193);
			} else {
				// Up arrow (more items above)
				u8g2_DrawGlyph(&disp, 67, 18, 0x2191);
			}

			u8g2_SetFont(&disp, u8g2_font_4x6_tr);
		}
		// Bottom line
		u8g2_DrawHLine(&disp, 1, 54, 126);

		// Send display buffer
		u8g2_SendBuffer(&disp);
	}
}

volatile int16_t last_count = 0;

void Encoder_ISR(void) {
	if (selected_target == 0) {
		// No action needed
		return;
	}

	static uint32_t last_tick = 0;
	uint32_t now = HAL_GetTick();
	uint32_t delta = now - last_tick;
	last_tick = now;

	// Make display blinking cursor turn on, reseting the blink counter run inside display_loop
	blink_cursor = true;
	blink_cursor_counter = 0;

	uint16_t raw = TIM2->CNT >> 1;
	int16_t new_count = (int16_t)raw;
	int16_t diff = new_count - last_count;

	if (diff > 32767 / 2)
		diff -= 32768;
	else if (diff < -(32767 / 2))
		diff += 32768;

	last_count = new_count;

	if (diff != 0) {
		EncoderTarget_t *t = &encoder_targets[selected_target];
		int32_t current_val = *t->value;

		// Acceleration based on time between pulses
		int step = 1;
		if (delta < 10) {
			step = t->accel*delta;
		}

		if (diff > 0)
			current_val += step;
		else
			current_val -= step;

		if (current_val > t->max) current_val = t->max;
		else if (current_val < t->min) current_val = t->min;

		*t->value = (uint16_t)current_val;
	}
}

void SD_LOG_Init() {
	if (!HAL_GPIO_ReadPin(SD_CD_GPIO_Port, SD_CD_Pin)) {
		// SD card slot's Card Detect pin active

		// Mount SD card
		if (f_mount(&FatFs, "", 1) == FR_OK) {
			Beep(50, 1);
			static char filename[50];
			HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BIN);
			sprintf(filename, "INSTRUF103_LOG_R%lu_%02d%02d%02d.csv",
					system_reset_count,
					RTC_Time.Hours,
					RTC_Time.Minutes,
					RTC_Time.Seconds
					);

			HAL_Delay(100);

			if(f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
				Beep(50, 1);
				uint8_t len = snprintf(SD_BUF, SD_BUFLEN, "TIME,HUM01,HUM02,HUM03,HUM04,LUX\r\n");
				f_write(&file, SD_BUF, len, &bw);
				f_sync(&file);

				SD_Present = true;
			}
		}
	}
}

void watering_loop(void) {
	static volatile bool toggle_led = false;
	// To mantain accurate timing, a flag inside stm32f1xx_it becomes true each second.
	if (watering_loop_flag) {
		watering_loop_flag = false;
		toggle_led = !toggle_led;
		TIM3->CCR4=toggle_led ? 255 : 32;

		if (TANK_HAS_WATER && !menu_active) {
			for (int ch = 0; ch < NUM_CH; ch++) {

				if (Enable_Channels[ch] == CH_OFF || Enable_Channels[ch] == CH_FULL) {
					Watering_State[ch] = CH_IDLE;
					current_watering_time[ch] = Watering_Period_time_array[Watering_Period[ch]];
					current_cooldown_time[ch] = Cooldown_Period_time_array[Cooldown_Period[ch]];
				} else {

					switch (Watering_State[ch]) {

					case CH_IDLE:
						// Condition to start watering
						if ((ADC_IN[ch].percent_ADC > ADC_Upper_Hysteresis_value[ch]) && (BH1750_lux >= Target_Radiance[ch])) {
							Watering_State[ch] = CH_WATERING;
							current_watering_time[ch] = Watering_Period_time_array[Watering_Period[ch]];   // start timing now
						}
						break;

					case CH_WATERING:
						current_watering_time[ch]--;

						// Stop watering if ADC falls low enough
						if (ADC_IN[ch].percent_ADC < ADC_Lower_Hysteresis_value[ch]) {
							Watering_State[ch] = CH_COOLDOWN;
							current_cooldown_time[ch] = Cooldown_Period_time_array[Cooldown_Period[ch]];
						}
						// Or stop if watering time is exceeded
						else if (current_watering_time[ch] == 0) {
							Watering_State[ch] = CH_COOLDOWN;
							current_cooldown_time[ch] = Cooldown_Period_time_array[Cooldown_Period[ch]];
						}
						break;

					case CH_COOLDOWN:
						current_cooldown_time[ch]--;

						// End cooldown when time is exceeded
						if (current_cooldown_time[ch] == 0) {
							Watering_State[ch] = CH_IDLE;
						}
						break;
					}
				}
			}
		}

		// Write data to SD card
		if (SD_Present) {
			uint8_t len = snprintf(SD_BUF, SD_BUFLEN, "%lu,%i,%i,%i,%i,%i\r\n", HAL_GetTick(), ADC_IN[0].percent, ADC_IN[1].percent, ADC_IN[2].percent, ADC_IN[3].percent, (uint16_t)BH1750_lux);
			f_write(&file, SD_BUF, len, &bw);
			f_sync(&file);
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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  BuildEncoderTargetTable();

  if (EEPROM_Init() == HAL_OK) {
	  // Load, update and store reset counter in EEPROM
	  Counters_Load();
	  ResetCounter_Save();

	  if (system_reset_count == 0) {
		  // Save default data to EEPROM on first power up
		  EEPROM_save_all_menu_data();
	  }

	  // Load all data from EEPROM
	  EEPROM_load_all_menu_data();
  }

  for (int ch = 0; ch < NUM_CH; ch++)
	  map_percentage_to_ADC_value(ch);

  // ADC1 initialisation
  //HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_Delay(500);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_BUFFER, NUM_CH);
  // SPI (ST7920) display initialisation
  setup_u8g2_st7920_SPI(&disp, &hspi1);
  // TIM1 initialisation for AC dimmer zero cross sync
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  // TIM4 initialisation for AC dimmer control
  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4);

  // HB LED
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  // Buzzer
#ifdef USE_BUZZER
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
#endif
  Beep(50, 1);

  // Turn LCD backlight on
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);

  // Start BH1750 lux sensor in High Resolution mode
  BH1750_Init(&hi2c2);
  BH1750_SetMode(CONTINUOUS_HIGH_RES_MODE);

  SD_LOG_Init();

  // TIM2 initialisation for encoder
  // Must be the last fcn called
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Inputs_loop();
	  Outputs_loop();
	  Read_BH1750();
	  Display_loop();

	  watering_loop();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 105;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  HAL_GPIO_WritePin(SD_NSS_GPIO_Port, SD_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENC_SW_Pin TANK_SW_Pin */
  GPIO_InitStruct.Pin = ENC_SW_Pin|TANK_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CD_Pin */
  GPIO_InitStruct.Pin = SD_CD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_CD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_NSS_Pin */
  GPIO_InitStruct.Pin = SD_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t ADC1_avg_counter = 0;
uint8_t ADC1_avg_until = 20;
uint64_t TMP_IN01 = 0;
uint64_t TMP_IN02 = 0;
uint64_t TMP_IN03 = 0;
uint64_t TMP_IN04 = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {

        // Accumulate (inverted ADC)
        TMP_IN01 += 4095 - ADC1_BUFFER[0];
        TMP_IN02 += 4095 - ADC1_BUFFER[1];
        TMP_IN03 += 4095 - ADC1_BUFFER[2];
        TMP_IN04 += 4095 - ADC1_BUFFER[3];

        // When enough samples gathered
        if (++ADC1_avg_counter >= ADC1_avg_until) {
            // Compute averaged values
            ADC_IN[0].value = (uint16_t)(TMP_IN01 / ADC1_avg_until);
            ADC_IN[1].value = (uint16_t)(TMP_IN02 / ADC1_avg_until);
            ADC_IN[2].value = (uint16_t)(TMP_IN03 / ADC1_avg_until);
            ADC_IN[3].value = (uint16_t)(TMP_IN04 / ADC1_avg_until);
            // Reset vars
            ADC1_avg_counter = 0;
            TMP_IN01 = TMP_IN02 = TMP_IN03 = TMP_IN04 = 0;
            // Map all channels to 0–100%
            map_ADC_input_to_percentage();
            // Calculate ADC Hysteresis
            calculate_ADC_hysteresis_thresholds();
        }
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		// Reset the TIM4 counter, making the PWM aligned with the AC zero crossing signal
		TIM4->CNT=0;
	} else if (htim->Instance == TIM2) {
		// New encoder event, handle it
		Encoder_ISR();
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
