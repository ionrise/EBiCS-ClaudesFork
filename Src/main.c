
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2019 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"




/* USER CODE BEGIN Includes */

// Lishui BLCD FOC Open Source Firmware
// Board uses IRS2003 half bridge drivers, this need inverted pulses for low-side Mosfets, deadtime is generated in driver
// This firmware bases on the ST user manual UM1052
// It uses the OpenSTM32 workbench (SW4STM32 toolchain)
// Basic peripheral setup was generated with CubeMX

#include "FOC.h"
#include "config.h"
#include "eeprom.h"


#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER)
#include "display_kingmeter.h"
#endif

#if (DISPLAY_TYPE & DISPLAY_TYPE_BAFANG)
#include "display_bafang.h"
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KUNTENG)
#include "display_kunteng.h"
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_EBiCS)
#include "display_ebics.h"
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_NO2)
#include "display_No_2.h"
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_TONGSHENG)
#include "display_tongsheng.h"
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_VESC)
#include "display_vesc.h"
#endif

#ifdef SENSORLESS
#include "sensorless.h"
#endif

#include <arm_math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


uint32_t ui32_tim1_counter=0;
uint32_t ui32_tim3_counter=0;
uint8_t ui8_hall_state=0;
uint8_t ui8_hall_state_old=0;
uint8_t ui8_hall_case =0;
uint16_t ui16_tim2_recent=0;
uint16_t ui16_timertics=5000; 					//timertics between two hall events for 60Â° interpolation
uint16_t ui16_throttle;
uint16_t ui16_torque;
uint16_t ui16_throttle_offset = 0; //default from RS.throttle_offset, measured at power-up
uint16_t ui16_torque_offset = TORQUE_OFFSET;
uint16_t ui16_brake_adc;
uint32_t ui32_throttle_cumulated;
uint32_t ui32_torque_cumulated_raw;
uint32_t ui32_brake_adc_cumulated;
uint32_t ui32_int_Temp_cumulated = (INT_TEMP_25 ? INT_TEMP_25 : INT_TEMP_V25_TYPICAL)<<5;
int16_t i16_int_Temp_V25;
uint16_t ui16_ph1_offset=0;
uint16_t ui16_ph2_offset=0;
uint16_t ui16_ph3_offset=0;
int16_t i16_ph1_current=0;

int16_t i16_ph2_current=0;
int16_t i16_ph2_current_filter=0;
int16_t i16_ph3_current=0;
uint16_t i=0;
uint16_t j=0;
uint16_t k=0;
uint16_t y=0;
uint8_t brake_flag=0;
volatile uint8_t ui8_overflow_flag=0;
uint8_t ui8_slowloop_counter=0;
volatile uint8_t ui8_adc_inj_flag=0;
volatile uint8_t ui8_adc_regular_flag=0;
uint8_t ui8_speedcase=0;
uint8_t ui8_speedfactor=0;
int8_t i8_direction= 1; //for permanent reverse direction, set from RS.reverse at boot
int8_t i8_reverse_flag = 1; //for temporaribly reverse direction
uint8_t ui8_KV_detect_flag = 0; //for getting the KV of the motor after auto angle detect
uint16_t ui16_KV_detect_counter = 0; //for getting timing of the KV detect
int16_t ui32_KV = 0;


volatile uint8_t ui8_adc_offset_done_flag=0;
volatile uint8_t ui8_print_flag=0;
volatile uint8_t ui8_UART_flag=0;
volatile uint8_t ui8_Push_Assist_flag=0;
volatile uint8_t ui8_UART_TxCplt_flag=1;
volatile uint8_t ui8_PAS_flag=0;
volatile uint8_t ui8_SPEED_flag=0;
volatile uint8_t ui8_SPEED_control_flag=0;
volatile uint8_t ui8_BC_limit_flag=0;  //flag for Battery current limitation
volatile uint8_t ui8_6step_flag=0;
int16_t i16_60deg_Hall_flag=0;
uint32_t uint32_PAS_counter= 32000; //set to RS.pas_timeout+1 at boot
uint32_t uint32_PAS_HIGH_counter= 0;
uint32_t uint32_PAS_HIGH_accumulated= 32000;
uint32_t uint32_PAS_fraction= 100;
uint32_t uint32_SPEED_counter=32000;
uint32_t uint32_SPEEDx100_cumulated=0;
uint32_t uint32_PAS=32000;
uint32_t PAS_IMP_PER_TURN_RECIP_MULTIPLIER = 8;  // recproce multiplier to avoid division in the main loop, (1<<8)/RS.pas_imp_per_turn set at boot

q31_t q31_rotorposition_PLL = 0;
q31_t q31_angle_per_tic = 0;

uint8_t ui8_UART_Counter=0;
int8_t i8_recent_rotor_direction=1;
int16_t i16_hall_order=1;
uint16_t ui16_erps=0;

uint32_t uint32_torque_cumulated=0;
uint32_t uint32_PAS_cumulated=32000;
uint16_t uint16_mapped_throttle=0;
uint16_t uint16_mapped_PAS=0;
uint16_t uint16_mapped_BRAKE=0;
uint16_t uint16_half_rotation_counter=0;
uint16_t uint16_full_rotation_counter=0;
int32_t int32_temp_current_target=0;
q31_t q31_PLL_error=0;
q31_t q31_t_Battery_Current_accumulated=0;

q31_t q31_rotorposition_absolute;
q31_t q31_rotorposition_hall;
//q31_t q31_rotorposition_motor_specific = SPEC_ANGLE;
q31_t q31_u_d_temp=0;
q31_t q31_u_q_temp=0;
int16_t i16_sinus=0;
int16_t i16_cosinus=0;
char char_dyn_adc_state_old=1;
const uint8_t assist_profile[2][6]= {	{0,10,20,30,45,48},
		{64,64,128,200,255,0}};

uint16_t switchtime[3];
volatile uint16_t adcData[9]; //Buffer for ADC1 Input
q31_t tic_array[6];

//Rotor angle scaled from degree to q31 for arm_math. -180Â°-->-2^31, 0Â°-->0, +180Â°-->+2^31
const q31_t deg_30 = 357913941;
// angles for 120� setup
q31_t Hall_13 = 0;
// autodetect: edge angles of the forward and backward sweep (order 45, 51, 13, 32, 26, 64), their average,
// and the largest forward/backward difference in degrees (read by the setup apps)
q31_t q31_hall_fwd[6];
q31_t q31_hall_bwd[6];
q31_t q31_hall_avg[6];
volatile uint8_t ui8_setup_halls_found_bwd = 0;
volatile uint8_t ui8_setup_hall_spread = 0;
// field weakening (runPIcontrol): negative Id setpoint (internal current units, live data), its ramp state,
// the ramp step per FOC cycle, the voltage the current controllers ask for (before the limit, low pass x16)
volatile int32_t i32_fw_id = 0;
// voltage and hall interval when the PWM was switched off for coasting (0 A for 100 ms: u_q = back-EMF at that speed)
static int32_t i32_uq_coast = 0;
static uint32_t ui32_tics_coast = 0;
static int32_t i32_fw_id16 = 0;
static int32_t i32_fw_step16 = 0;
static int32_t i32_fw_u_start = 0;
static int32_t i32_u_demand16 = 0;
static int8_t i8_direction_fwd = 1;
q31_t Hall_32 = 0;
q31_t Hall_26 = 0;
q31_t Hall_64 = 0;
q31_t Hall_51 = 0;
q31_t Hall_45 = 0;
// angles for 60� setup
q31_t Hall_46 = 0;
q31_t Hall_67 = 0;
q31_t Hall_73 = 0;
q31_t Hall_31 = 0;
q31_t Hall_10 = 0;
q31_t Hall_4 = 0;

q31_t tics_lower_limit = 0; //set at boot
q31_t tics_higher_limit = 0; //set at boot
uint32_t uint32_tics_filtered=1000000;

uint16_t VirtAddVarTab[NB_OF_VAR] = { 	EEPROM_POS_HALL_ORDER,
		EEPROM_POS_HALL_45,
		EEPROM_POS_HALL_51,
		EEPROM_POS_HALL_13,
		EEPROM_POS_HALL_32,
		EEPROM_POS_HALL_26,
		EEPROM_POS_HALL_64,
		EEPROM_POS_KV,
		EEPROM_KT_B0_B3,
		EEPROM_KT_B2_B4,
		EEPROM_KT_B6_B7,
		EEPROM_KT_B8_B9,
		EEPROM_KT_B1_B10,
		EEPROM_INT_TEMP_V25,
		EEPROM_HALL_60
}; //all NB_OF_VAR (15) entries, otherwise variables are dropped when the EEPROM page is full

// Setup mailbox: the EBiCS Setup app writes/reads these over the ST-Link (SWD) while the controller runs.
// No UART or debug display is needed for hall/KV calibration.
volatile uint8_t ui8_setup_cmd = 0;          // app writes 1 = start hall + KV autodetect (only accepted when motor is off)
volatile uint8_t ui8_setup_state = 0;        // 0 idle, 1 hall scan, 2 KV spin-up, 3 done, 4 failed (hall missing)
volatile uint8_t ui8_setup_progress = 0;     // 0..100 for the running step
volatile uint8_t ui8_setup_halls_found = 0;  // bitmask of the 6 hall transitions seen during the scan, 0x3F = all
volatile uint8_t ui8_hall_calibrated = 0;    // 1 if hall angles were loaded from EEPROM or detected
volatile uint8_t ui8_watchdog_reset = 0;     // 1 if the last reset was caused by the watchdog
volatile uint32_t ui32_hall_events = 0;      // counts every hall sensor change (gear ratio measurement by hand)
volatile uint32_t ui32_pas_events = 0;       // counts PAS sensor pulses (sensor check)
volatile uint16_t ui16_ticks_since_hall = 65535; // 8 kHz ticks since the last hall edge (TIM2 wraps after 132 ms)
volatile uint8_t ui8_hall_edges = 0;          // hall edges since the motor started turning (reset after 125 ms standstill)
volatile uint8_t ui8_overcurrent = 0;         // 1 = software overcurrent trip, PWM off until the request is released
volatile uint8_t ui8_oc_trips = 0;            // number of trips since power on (3 = stays off until restart)
volatile uint8_t ui8_stall_limited = 0;       // 1 = current limited because the motor does not turn
int16_t i16_oc_limit = 1950;                  // phase current trip level in ADC counts
static uint8_t ui8_oc_count = 0;
// overcurrent trip capture for live data: u_q, u_d, phase 1, phase 2, CCR1..3, PWM cycles since switch-on, setup state
volatile int16_t i16_oc_capture[9];
static uint16_t ui16_moe_cycles = 0;
// startup melody (ionrise): the FOC interrupt puts an audio frequency voltage on one phase axis instead of the FOC
volatile uint8_t ui8_tone_on = 0;
static uint32_t ui32_tone_step, ui32_tone_phase;
static int16_t i16_tone_amp, i16_tone_ca, i16_tone_sa;
static volatile uint16_t ui16_tone_peak[3];
volatile uint8_t ui8_phase_fault = 0;         // phase test of the startup melody: bit 0..2 = axis A, B, C failed (motor blocked)
int32_t i32_ramp_step16 = 0;                  // current ramp: setpoint step per FOC cycle <<16, 0 = no ramp
int32_t i32_iq_ramped16 = 0;                  // ramped phase current setpoint <<16
volatile uint8_t ui8_kv_valid = 0;            // 1 = motor constant KV measured (EEPROM or KV detect), needed for coasting
volatile uint8_t ui8_motor_temp_sensor_ok = 0; // 1 = NTC reading plausible, 0 = open/shorted/not connected (temperature ignored)
volatile uint16_t ui16_motor_temp_adc = 0;   // filtered NTC reading of the speed/temperature input (only samples above the threshold)
volatile uint32_t ui32_motor_temp_adc_cumulated = 2048UL<<4;
const volatile uint32_t ui32_setup_magic = 0xEB1C5002; // lets the app check the firmware supports the mailbox

enum state {Stop, SixStep, Regen, Running, BatteryCurrentLimit, Interpolation, PLL, IdleRun};
enum state SystemState;

#define iabs(x) (((x) >= 0)?(x):-(x))
#define sign(x) (((x) >= 0)?(1):(-1))


//variables for display communication
#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER)
KINGMETER_t KM;
#endif

//variables for display communication
#if (DISPLAY_TYPE & DISPLAY_TYPE_BAFANG)
BAFANG_t BF;
#endif

#if (DISPLAY_TYPE & DISPLAY_TYPE_EBiCS)
uint8_t ui8_main_LEV_Page_counter=0;
uint8_t ui8_additional_LEV_Page_counter=0;
uint8_t ui8_LEV_Page_to_send=1;
#endif

//variables for display communication
#if (DISPLAY_TYPE == DISPLAY_TYPE_NO2)
No2_t No2;
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_TONGSHENG)
TONGSHENG_t TS;
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_VESC)
VESC_t VESC;
#endif

MotorState_t MS;
MotorParams_t MP;

//structs for PI_control
PI_control_t PI_iq;
PI_control_t PI_id;
PI_control_t PI_speed;


int32_t battery_percent_fromcapacity = 50; 			//Calculation of used watthours not implemented yet
int16_t wheel_time = 1000;							//duration of one wheel rotation for speed calculation
int16_t current_display;							//pepared battery current for display

int16_t power;										//recent power output

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
int16_t T_NTC(uint16_t ADC);
void init_watchdog(void);
void MX_IWDG_Init(void);
void get_internal_temp_offset(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Set_Hall_Logic(void);
void Set_Hall_Angle60(void);
void Set_Hall_Angle120(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER)
void kingmeter_update(void);
#endif

#if (DISPLAY_TYPE & DISPLAY_TYPE_BAFANG)
void bafang_update(void);
#endif

static void dyn_adc_state(q31_t angle);
static void set_inj_channel(char state);
void get_standstill_position();
void svpwm(q31_t q31_u_alpha, q31_t q31_u_beta);
static void play_melody(void);
#if defined(SENSORLESS) && defined(PARAM_ID)
static void pid_measure(void);
extern volatile uint8_t ui8_pid_rotate;
extern volatile int32_t i32_pid_step;
#endif
q31_t speed_PLL (q31_t ist, q31_t soll, uint8_t speedadapt);
int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
int32_t speed_to_tics (uint8_t speed);
int8_t tics_to_speed (uint32_t tics);
int16_t internal_tics_to_speedx100 (uint32_t tics);
int16_t external_tics_to_speedx100 (uint32_t tics);





/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
	/* USER CODE BEGIN 1 */



	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_USART1_UART_Init();


	//runtime settings: block written by the Android setup app, or the config.h values
	settings_load();
	// software overcurrent trip at 1.5x the max phase current (at least the hall autodetect current, at most the ADC range)
	i16_oc_limit = (RS.ph_current_max * 3) / 2;
	if (i16_oc_limit < 500) i16_oc_limit = 500;
	if (i16_oc_limit > 1950) i16_oc_limit = 1950;
	// current ramp: full phase current in current_ramp_ms (FOC runs at 15779 Hz, ~16 cycles per ms; 32 bit math:
	// a 64 bit division here pulled in ~1 KB of library code)
	i32_ramp_step16 = (RS.current_ramp_ms > 0) ? (int32_t)(((uint32_t)RS.ph_current_max << 16) / ((uint32_t)RS.current_ramp_ms * 16)) : 0;
	if (RS.current_ramp_ms > 0 && i32_ramp_step16 < 1) i32_ramp_step16 = 1;
	// field weakening: fw_current_max in fw_ramp_ms (16 FOC cycles per ms), voltage use to keep
	if (RS.fw_current_max > RS.ph_current_max) RS.fw_current_max = RS.ph_current_max; //Id alone within the phase current
	i32_fw_step16 = (RS.fw_current_max > 0) ? (int32_t)(((uint32_t)RS.fw_current_max << 16) / ((uint32_t)(RS.fw_ramp_ms ? RS.fw_ramp_ms : 500) * 16)) : 0;
	if (RS.fw_current_max > 0 && i32_fw_step16 < 1) i32_fw_step16 = 1;
	i32_fw_u_start = (_U_MAX * (RS.fw_voltage_pct ? RS.fw_voltage_pct : 90)) / 100;
	ui16_throttle_offset = RS.throttle_offset;
	i8_direction = RS.reverse;
	uint32_PAS_counter = RS.pas_timeout+1;
	PAS_IMP_PER_TURN_RECIP_MULTIPLIER = (1 << 8) / RS.pas_imp_per_turn;
	tics_lower_limit = RS.wheel_circumference*5*3600/(6*RS.gear_ratio*RS.speed_limit*10); //tics=wheelcirc*timerfrequency/(no. of hallevents per rev*gear-ratio*speedlimit)*3600/1000000
	tics_higher_limit = RS.wheel_circumference*5*3600/(6*RS.gear_ratio*(RS.speed_limit+2)*10);

	//initialize MS struct.
	MS.hall_angle_detect_flag=1;
	MS.Speed=128000;
	MS.assist_level=127;
	MS.regen_level=7;
	MS.i_q_setpoint = 0;
	MS.i_d_setpoint = 0;
	MS.angle_est=SPEED_PLL;


	MP.pulses_per_revolution = RS.pulses_per_revolution;
	MP.wheel_cirumference = RS.wheel_circumference;
	MP.speedLimit=RS.speed_limit;
	MP.battery_current_max = RS.battery_current_max;


	//init PI structs
	PI_id.gain_i=I_FACTOR_I_D;
	PI_id.gain_p=P_FACTOR_I_D;
	PI_id.setpoint = 0;
	PI_id.limit_output = _U_MAX;
	PI_id.max_step=5000;
	PI_id.shift=10;
	PI_id.limit_i=1800;

	PI_iq.gain_i=I_FACTOR_I_Q;
	PI_iq.gain_p=P_FACTOR_I_Q;
	PI_iq.setpoint = 0;
	PI_iq.limit_output = _U_MAX;
	PI_iq.max_step=5000;
	PI_iq.shift=10;
	PI_iq.limit_i=_U_MAX;

#ifdef SPEEDTHROTTLE

	PI_speed.gain_i=I_FACTOR_SPEED;
	PI_speed.gain_p=P_FACTOR_SPEED;
	PI_speed.setpoint = 0;
	PI_speed.limit_output = RS.ph_current_max;
	PI_speed.max_step=50;
	PI_speed.shift=5;
	PI_speed.limit_i=RS.ph_current_max;

#endif

	//Virtual EEPROM init
	HAL_FLASH_Unlock();
	EE_Init();
	HAL_FLASH_Lock();

	MX_ADC1_Init();
	/* Run the ADC calibration */
	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
	{
		/* Calibration Error */
		Error_Handler();
	}
	MX_ADC2_Init();
	/* Run the ADC calibration */
	if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK)
	{
		/* Calibration Error */
		Error_Handler();
	}

	/* USER CODE BEGIN 2 */
	SET_BIT(ADC1->CR2, ADC_CR2_JEXTTRIG);//external trigger enable
	__HAL_ADC_ENABLE_IT(&hadc1,ADC_IT_JEOC);
	SET_BIT(ADC2->CR2, ADC_CR2_JEXTTRIG);//external trigger enable
	__HAL_ADC_ENABLE_IT(&hadc2,ADC_IT_JEOC);


	//HAL_ADC_Start_IT(&hadc1);
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adcData, 8);
	HAL_ADC_Start_IT(&hadc2);
	MX_TIM1_Init(); //Hier die Reihenfolge getauscht!
	MX_TIM2_Init();
	MX_TIM3_Init();

	// Start Timer 1
	if(HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
	{
		/* Counter Enable Error */
		Error_Handler();
	}

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); // turn on complementary channel
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);




	TIM1->CCR4 = TRIGGER_DEFAULT; //ADC sampling just before timer overflow (just before middle of PWM-Cycle)
	//PWM Mode 1: Interrupt at counting down.

	//TIM1->BDTR |= 1L<<15;
	// TIM1->BDTR &= ~(1L<<15); //reset MOE (Main Output Enable) bit to disable PWM output
	// Start Timer 2
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	//HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	//HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);

	// Start Timer 3

	if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
	{
		/* Counter Enable Error */
		Error_Handler();
	}


#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER)
	KingMeter_Init (&KM);
#endif

#if (DISPLAY_TYPE & DISPLAY_TYPE_BAFANG)
	Bafang_Init (&BF);
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KUNTENG)
	kunteng_init();
	check_message(&MS, &MP);
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_EBiCS)
	//  ebics_init();
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_NO2)
	No2_Init(&No2);
#endif
#if (DISPLAY_TYPE == DISPLAY_TYPE_TONGSHENG)
	Tongsheng_Init(&TS);
#endif
#if (DISPLAY_TYPE == DISPLAY_TYPE_VESC)
	Vesc_Init(&VESC);
	MS.assist_level = 153; //no assist level from VESC devices: PAS at level 3 of 5
#endif
	TIM1->CCR1 = 1023; //set initial PWM values
	TIM1->CCR2 = 1023;
	TIM1->CCR3 = 1023;


	CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);//Disable PWM

	HAL_Delay(500); //wait for stable conditions

	for(i=0;i<32;i++){
		while(!ui8_adc_regular_flag){}
		temp1+=adcData[2]; //Phasecurrent 1 offset
		temp2+=adcData[3]; //Phasecurrent 2 offset
		temp3+=adcData[4]; //Phasecurrent 3 offset
		temp4+=adcData[1]; //Throttle Offset
		temp5+=adcData[6]; //Torque Offset on AD1

		ui8_adc_regular_flag=0;
	}
	ui16_ph1_offset=temp1>>5;
	ui16_ph2_offset=temp2>>5;
	ui16_ph3_offset=temp3>>5;
	ui16_throttle_offset=(temp4>>5)+5;

	if(RS_FLAG(SF_TORQUE_ON_AD1)) ui16_torque_offset=(temp5>>5)+5;
	else ui16_torque_offset=ui16_throttle_offset;

#ifdef DISABLE_DYNAMIC_ADC // set  injected channel with offsets
	ADC1->JSQR=0b00100000000000000000; //ADC1 injected reads phase A JL = 0b00, JSQ4 = 0b00100 (decimal 4 = channel 4)
	ADC1->JOFR1 = ui16_ph1_offset;
	ADC2->JSQR=0b00101000000000000000; //ADC2 injected reads phase B, JSQ4 = 0b00101, decimal 5
	ADC2->JOFR1 = ui16_ph2_offset;
#endif

	//phase current reading state 1 (A and B directly, as the injected channels are set up). It was 0 until the first FOC
	//run: state 0 doesn't update the phase currents, they kept the raw ADC values (~2048) of the offset measurement and
	//the first switch-on after power up tripped the overcurrent protection at once (real motor)
	MS.char_dyn_adc_state = 1;
#ifndef DISABLE_DYNAMIC_ADC
	//the offsets go into the ADC only with set_inj_channel(), which the interrupt calls when the state changes. State 1
	//at power up is no change, so the first PWM cycles read raw ADC values (~1024): the startup melody switched the PWM
	//on right away and tripped the overcurrent protection in the 3rd cycle (real motor)
	set_inj_channel(1);
	char_dyn_adc_state_old = 1;
#endif
	i16_ph1_current = 0;
	i16_ph2_current = 0;
	ui8_adc_offset_done_flag=1;
	init_watchdog();

	if(RS_FLAG(SF_ADC_BRAKE)){
		while ((adcData[5]>ui16_throttle_offset)&&(adcData[1]>(RS.throttle_max-ui16_throttle_offset))){HAL_Delay(200);
		HAL_IWDG_Refresh(&hiwdg);
		y++;
		if(y==35) autodetect();
		}
	}

	//run autodect, whenn brake is pulled an throttle is pulled for 10 at startup
#ifndef NCTE

	while ((!HAL_GPIO_ReadPin(Brake_GPIO_Port, Brake_Pin))&&(adcData[1]>(ui16_throttle_offset+20))){
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(200);
		y++;
		if(y==35) autodetect();
	}
#else
	ui32_throttle_cumulated=ui16_throttle_offset<<4;
#endif


	{
		HAL_IWDG_Refresh(&hiwdg);//do nothing (For Safety at switching on)
	}
	//read internal temp calibration from emulated EEPROM
	EE_ReadVariable(EEPROM_INT_TEMP_V25, &i16_int_Temp_V25);
	if(!i16_int_Temp_V25)i16_int_Temp_V25=INT_TEMP_25 ? INT_TEMP_25 : INT_TEMP_V25_TYPICAL; //not calibrated: main.h value, else the datasheet typical (was 0 = about -300 deg C shown)

#if (!USE_FIX_POSITIONS)
	uint16_t ui16_hall_order_status = EE_ReadVariable(EEPROM_POS_HALL_ORDER, (uint16_t*)&i16_hall_order);
	// set varaiables to value from emulated EEPROM only if valid
	if(ui16_hall_order_status==0) { //0 = variable found in EEPROM
		ui8_hall_calibrated=1;
		int16_t temp;
		//hall edge angles (a loop over a table: 6 copies of the same code were ~60 bytes more, the sensorless
		//build with the Kunteng display was full)
#ifndef SENSORLESS //not used without hall sensors
		static const uint16_t ui16_hall_ee[6] = {EEPROM_POS_HALL_45, EEPROM_POS_HALL_51, EEPROM_POS_HALL_13,
				EEPROM_POS_HALL_32, EEPROM_POS_HALL_26, EEPROM_POS_HALL_64};
		q31_t * const q31_hall_dst[6] = {&Hall_45, &Hall_51, &Hall_13, &Hall_32, &Hall_26, &Hall_64};
		for (i = 0; i < 6; i++) {
			EE_ReadVariable(ui16_hall_ee[i], (uint16_t *) &temp);
			*q31_hall_dst[i] = temp<<16;
			HAL_IWDG_Refresh(&hiwdg);
		}
#endif

		if(EE_ReadVariable(EEPROM_POS_KV, (uint16_t*)&ui32_KV)==0 && ui32_KV>0) ui8_kv_valid=1;
		if(ui32_KV<=0)ui32_KV=111;
		HAL_IWDG_Refresh(&hiwdg);

	}

#else
	EE_ReadVariable(EEPROM_HALL_60, &i16_60deg_Hall_flag);

	i16_hall_order = HALL_ORDER;
	ui32_KV = KV;
	Hall_45 = HALL_45;
	Hall_51 = HALL_51;
	Hall_13 = HALL_13;
	Hall_32 = HALL_32;
	Hall_26 = HALL_26;
	Hall_64 = HALL_64;

	Hall_46 = HALL_60_46;
	Hall_67 = HALL_60_67;
	Hall_73 = HALL_60_73;
	Hall_31 = HALL_60_31;
	Hall_10 = HALL_60_10;
	Hall_4 = HALL_60_4;

#endif


	// set absolute position to corresponding hall pattern.



	CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);//Disable PWM

	get_standstill_position();

	play_melody();

	//	init_watchdog();


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		HAL_IWDG_Refresh(&hiwdg);

		//setup app requested hall + KV autodetect over SWD (only while the motor is switched off)
		//The command stays pending until the motor stands still (no hall edge for ~1 s) and no current is asked for: an
		//autodetect started while the wheel still turned (a geared hub coasts ~10 s without load) let the current
		//controller wind up against the spinning rotor and tripped the overcurrent protection. Before, the command was
		//also dropped when the PWM was on (0 A after a failed calibration): every retry was ignored until power off.
		if(ui8_setup_cmd==1 && SystemState == Stop && !MS.i_q_setpoint && !ui8_overcurrent){
			ui8_setup_cmd=0;
			CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
			HAL_Delay(2); //a few PWM cycles: the interrupt resets the controllers and the voltage while the PWM is off
			autodetect();
		}
#if defined(SENSORLESS) && defined(PARAM_ID)
		if(ui8_setup_cmd==2 && SystemState == Stop && !MS.i_q_setpoint && !ui8_overcurrent){
			ui8_setup_cmd=0;
			CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
			HAL_Delay(2);
			pid_measure();
		}
#endif


		/* if(PI_flag){
	  runPIcontrol();
	  PI_flag=0;
	  }*/
		//display message processing
		if(ui8_UART_flag){
#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_901U)

			KingMeter_Service(&KM);
#endif
#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_618U)
			kingmeter_update();

#endif

#if (DISPLAY_TYPE & DISPLAY_TYPE_BAFANG)
			bafang_update();
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KUNTENG)
			check_message(&MS, &MP);
			if(MS.assist_level==6)ui8_Push_Assist_flag=1;
			else ui8_Push_Assist_flag=0;
#endif

#if (DISPLAY_TYPE & DISPLAY_TYPE_EBiCS)
			//  process_ant_page(&MS, &MP);
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_NO2)
			No2_Service(&No2);
#endif
#if (DISPLAY_TYPE == DISPLAY_TYPE_TONGSHENG)
			Tongsheng_Service(&TS);
#endif
#if (DISPLAY_TYPE == DISPLAY_TYPE_VESC)
			Vesc_Service(&VESC);
#endif
			ui8_UART_flag=0;
		}


		//process regualr ADC
		if(ui8_adc_regular_flag){
			ui32_throttle_cumulated -= ui32_throttle_cumulated>>4;
			ui32_throttle_cumulated += adcData[1]; //get value from SP
			ui32_torque_cumulated_raw -= ui32_torque_cumulated_raw>>4;
			ui32_torque_cumulated_raw += adcData[6]; //get value from AD1
			if(!RS_FLAG(SF_TORQUE_ON_AD1)) ui32_torque_cumulated_raw=ui32_throttle_cumulated;
			ui32_brake_adc_cumulated -= ui32_brake_adc_cumulated>>4;
			ui32_brake_adc_cumulated+=adcData[5];//get value for analog brake from AD2 = PB0
			ui16_brake_adc=ui32_brake_adc_cumulated>>4;
			ui16_throttle = ui32_throttle_cumulated>>4;
			ui16_torque =  ui32_torque_cumulated_raw>>4;
			ui8_adc_regular_flag=0;
		}

		//PAS signal processing
		if(ui8_PAS_flag){

			if(uint32_PAS_counter>100){ //debounce
				uint32_PAS_cumulated -= uint32_PAS_cumulated>>2;
				uint32_PAS_cumulated += uint32_PAS_counter;
				uint32_PAS = uint32_PAS_cumulated>>2;

				uint32_PAS_HIGH_accumulated-=uint32_PAS_HIGH_accumulated>>2;
				uint32_PAS_HIGH_accumulated+=uint32_PAS_HIGH_counter;

				uint32_PAS_fraction=(uint32_PAS_HIGH_accumulated>>2)*100/uint32_PAS;
				uint32_PAS_HIGH_counter=0;
				uint32_PAS_counter =0;
				ui8_PAS_flag=0;
				//read in and sum up torque-signal within one crank revolution (for sempu sensor 32 PAS pulses/revolution, 2^5=32)
				uint32_torque_cumulated -= (uint32_torque_cumulated*PAS_IMP_PER_TURN_RECIP_MULTIPLIER) >> 8 ;
#ifdef NCTE
				if(ui16_torque<ui16_torque_offset)uint32_torque_cumulated += (ui16_torque_offset-ui16_torque);
#else
				if(ui16_torque>ui16_torque_offset)uint32_torque_cumulated += (ui16_torque-ui16_torque_offset);
#endif


			}
		}

		//SPEED signal processing
		if(!RS_FLAG(SF_SPEED_EXTERNAL)) MS.Speed = uint32_tics_filtered>>3;
		else if(ui8_SPEED_flag){
			if(uint32_SPEED_counter>200){ //debounce
				MS.Speed = uint32_SPEED_counter;
				uint32_SPEED_counter =0;
				ui8_SPEED_flag=0;
				uint32_SPEEDx100_cumulated -=uint32_SPEEDx100_cumulated>>SPEEDFILTER;
				uint32_SPEEDx100_cumulated +=external_tics_to_speedx100(MS.Speed);
			}
		}
		if(ui8_SPEED_control_flag){
			if(!RS_FLAG(SF_SPEED_EXTERNAL)){
				uint32_SPEEDx100_cumulated -=uint32_SPEEDx100_cumulated>>SPEEDFILTER;
				uint32_SPEEDx100_cumulated +=internal_tics_to_speedx100(uint32_tics_filtered>>3);
			}
			ui16_erps=500000/((uint32_tics_filtered>>3)*6);
			ui8_SPEED_control_flag=0;
		}


		//--------------------------------------------------------------------------------------------------------------------------------------------------

		//current target calculation
		//highest priority: regen by brake lever


		if(RS_FLAG(SF_ADC_BRAKE)){ //proportional regen with an analog brake sensor on AD2
			uint16_mapped_BRAKE = map(ui16_brake_adc, ui16_throttle_offset , RS.throttle_max, 0, RS.regen_current);
			brake_flag = (uint16_mapped_BRAKE>0);
		}
		else brake_flag = !HAL_GPIO_ReadPin(Brake_GPIO_Port, Brake_Pin);

		if(brake_flag){
			if(tics_to_speed(uint32_tics_filtered>>3)>6){ //only apply regen, if motor is turning fast enough
				int32_temp_current_target = RS_FLAG(SF_ADC_BRAKE) ? uint16_mapped_BRAKE : RS.regen_current;
			}
			else int32_temp_current_target=0;
			int32_temp_current_target= -map(MS.Voltage*CAL_V,RS.battery_voltage_max-RS.voltage_ramp_mv,RS.battery_voltage_max,int32_temp_current_target,0); //soft ramp: no regen above the full charge voltage
		}

		//next priority: undervoltage protection
		else if(MS.Voltage*CAL_V<RS.battery_voltage_min)int32_temp_current_target=0; //below cutoff: off (soft ramp above, see below)
		//next priority: push assist
#if (DISPLAY_TYPE == DISPLAY_TYPE_KUNTENG)
		else if(ui8_Push_Assist_flag){int32_temp_current_target=(RS.pushassist_current);} //Kunteng: walk assist = level 6, sets ui8_Push_Assist_flag
#else
		else if(ui8_Push_Assist_flag)int32_temp_current_target=(MS.assist_level*RS.pushassist_current)>>8; //does not work for BAFANG
#endif
		// last priority normal ride conditiones
		else {
			if(RS_FLAG(SF_TORQUE_SENSOR)){ //torque-sensor mode
				//calculate current target form torque, cadence and assist level
				int32_temp_current_target = (RS.ts_coef*(int32_t)(MS.assist_level)* ((uint32_torque_cumulated*PAS_IMP_PER_TURN_RECIP_MULTIPLIER)>>8)/uint32_PAS)>>8; // >>8 aus KM5S-Protokoll Assistlevel 0..255
				//limit current target to max value
				if(int32_temp_current_target>RS.ph_current_max) int32_temp_current_target = RS.ph_current_max;
				//set target to zero, if pedals are not turning
				if(uint32_PAS_counter > RS.pas_timeout){
					int32_temp_current_target = 0;
					if(uint32_torque_cumulated>0)uint32_torque_cumulated--; //ramp down cumulated torque value
				}
			}
			else { // torque-simulation mode: assist from the cadence (PAS) and the assist level
#if (DISPLAY_TYPE & DISPLAY_TYPE_BAFANG)
				uint16_mapped_PAS = map(uint32_PAS, RS.ramp_end, RS.pas_timeout, (RS.ph_current_max*(int32_t)assist_from_level(MS.assist_level))>>8, 0); // level 0...9, % per level from the settings
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KUNTENG)
				uint16_mapped_PAS = map(uint32_PAS, RS.ramp_end, RS.pas_timeout, (RS.ph_current_max*(int32_t)assist_from_level(MS.assist_level))>>8, 0); // level 0...5, % per level from the settings
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_618U)
				uint16_mapped_PAS = map(uint32_PAS, RS.ramp_end, RS.pas_timeout, (RS.ph_current_max*(int32_t)(MS.assist_level-1))>>2, 0); // level in range 1...5
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_901U||DISPLAY_TYPE == DISPLAY_TYPE_NO2||DISPLAY_TYPE == DISPLAY_TYPE_TONGSHENG||DISPLAY_TYPE == DISPLAY_TYPE_VESC)
				uint16_mapped_PAS = map(uint32_PAS, RS.ramp_end, RS.pas_timeout, ((RS.ph_current_max*(int32_t)(MS.assist_level)))>>8, 0); // level in range 0...255
#endif


				if(RS_FLAG(SF_DIRDET)){ //direction detection
					if (uint32_PAS_counter< RS.pas_timeout){
						if ((uint32_PAS_fraction < RS.frac_low ||uint32_PAS_fraction > RS.frac_high)){
							uint16_mapped_PAS= 0;//pedals are turning backwards, stop motor
						}
					}
					else uint32_PAS_HIGH_accumulated=uint32_PAS_cumulated;
				}
#ifdef INDIVIDUAL_MODES
				uint16_mapped_PAS = (uint16_mapped_PAS * ui8_speedfactor)>>8;
#endif
				//set current target in torque-simulation-mode, if pedals are turning
				//(before the runtime settings this was only done with THROTTLE_OVERRIDE: without a throttle PAS gave no assist)
				if (uint32_PAS_counter < RS.pas_timeout) int32_temp_current_target = uint16_mapped_PAS;
				else  {
					int32_temp_current_target= 0;//pedals are not turning, stop motor
					uint32_PAS_cumulated=32000;
					uint32_PAS=32000;
				}
			}

			if(RS_FLAG(SF_THROTTLE)){ //throttle override
#ifdef NCTE
				uint16_mapped_throttle = map(ui16_throttle, RS.throttle_max, ui16_throttle_offset,RS.ph_current_max,0);
#else
				uint16_mapped_throttle = map(ui16_throttle, ui16_throttle_offset, RS.throttle_max, 0,RS.ph_current_max);
#endif
				//check for throttle override
				if(uint16_mapped_throttle>int32_temp_current_target){
#ifdef SPEEDTHROTTLE


					uint16_mapped_throttle = uint16_mapped_throttle*RS.speed_limit/RS.ph_current_max;//throttle override: calulate speed target from thottle




					PI_speed.setpoint = uint16_mapped_throttle*100;
					PI_speed.recent_value = internal_tics_to_speedx100(uint32_tics_filtered>>3);
					if( PI_speed.setpoint)SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
					if (internal_tics_to_speedx100(uint32_tics_filtered>>3)<300){//control current slower than 3 km/h
						PI_speed.limit_i=100;
						PI_speed.limit_output=100;
						int32_temp_current_target = PI_control(&PI_speed);

						if(int32_temp_current_target>100)int32_temp_current_target=100;
						if(int32_temp_current_target*i8_direction*i8_reverse_flag<0){
							int32_temp_current_target=0;
						}

					}
					else{


						if(ui8_SPEED_control_flag){//update current target only, if new hall event was detected
							PI_speed.limit_i=RS.ph_current_max;
							PI_speed.limit_output=RS.ph_current_max;
							int32_temp_current_target = PI_control(&PI_speed);
							ui8_SPEED_control_flag=0;
						}
						if(int32_temp_current_target*i8_direction*i8_reverse_flag<0)int32_temp_current_target=0;

					}
#else // else speedthrottle
					int32_temp_current_target=uint16_mapped_throttle;
#endif  //end speedthrottle
				}
			} //end throttle override
		} //end else for normal riding

		//ramp down setpoint at speed limit (legal mode), only if no regen active
		if(RS_FLAG(SF_LEGAL) && !brake_flag){
			if(uint32_PAS_counter<RS.pas_timeout){
				int32_temp_current_target=map(uint32_SPEEDx100_cumulated>>SPEEDFILTER, MP.speedLimit*100,(MP.speedLimit+2)*100,int32_temp_current_target,0);
			}
			else{ //limit to 6km/h if pedals are not turning
				int32_temp_current_target=map(uint32_SPEEDx100_cumulated>>SPEEDFILTER, 500,700,int32_temp_current_target,0);
			}
		}

#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER)
			if(KM.DirectSetpoint!=-1)int32_temp_current_target=(KM.DirectSetpoint*RS.ph_current_max)>>7;
#endif
#if (DISPLAY_TYPE == DISPLAY_TYPE_VESC)
			//current requested by a VESC device, released 1 s after the last command, never overrides braking
			if(VESC.cmd_active && (HAL_GetTick()-VESC.last_cmd_tick) < VESC_CMD_TIMEOUT_MS){
				if(!brake_flag) int32_temp_current_target=VESC.current_target;
			}
			else VESC.cmd_active=0;
#endif
			//soft ramp down of the motor current when the battery voltage approaches the cutoff (all modes: PAS, throttle, push, VESC, display)
			if(!brake_flag && int32_temp_current_target>0){
				int32_temp_current_target=map(MS.Voltage*CAL_V, RS.battery_voltage_min, RS.battery_voltage_min+RS.voltage_ramp_mv, 0, int32_temp_current_target);
			}
#ifndef SENSORLESS
			//KV detect: its current replaces throttle / PAS here, before MS.i_q_setpoint is written. It used to be written
			//later in the loop, so MS.i_q_setpoint was 0 for a moment in every loop pass; the current ramp in runPIcontrol
			//drops to 0 at once on a 0 setpoint, so the KV current never built up and the motor only crept (real motor).
			if(ui8_KV_detect_flag) int32_temp_current_target = ui8_KV_detect_flag;
#endif
			MS.i_q_setpoint=map(MS.Temperature, RS.motor_temp_threshold,RS.motor_temp_max,int32_temp_current_target,0); //ramp down power with temperature to avoid overheating the motor
#if(INT_TEMP_25)
			MS.i_q_setpoint=map(MS.int_Temperature, CONTROLLER_TEMPERATURE_THRESHOLD,CONTROLLER_TEMPERATURE_MAX,MS.i_q_setpoint,0); //ramp down power with processor temperatur to avoid overheating the controller
#endif

			//stall protection: more than half the max current for 3 s without the motor turning -> limit to a quarter,
			//until the motor turns again or the request is released (protects motor and MOSFETs, e.g. blocked wheel)
			{
				static uint32_t ui32_stall_since = 0;
#ifdef SENSORLESS
				uint8_t ui8_turning = ui8_sensorless_running;
#else
				uint8_t ui8_turning = ui16_ticks_since_hall < 4000;   //hall edge within the last 0.5 s
#endif
				int32_t i32_sp_abs = MS.i_q_setpoint < 0 ? -MS.i_q_setpoint : MS.i_q_setpoint;
				if (!ui8_turning && i32_sp_abs > (RS.ph_current_max >> 1)) {
					if (!ui32_stall_since) ui32_stall_since = HAL_GetTick() | 1;
					else if (HAL_GetTick() - ui32_stall_since > 3000) ui8_stall_limited = 1;
				}
				else {
					ui32_stall_since = 0;
					if (ui8_turning || !i32_sp_abs) ui8_stall_limited = 0;
				}
				if (ui8_stall_limited) {
					int32_t i32_lim = RS.ph_current_max >> 2;
					if (MS.i_q_setpoint > i32_lim) MS.i_q_setpoint = i32_lim;
					if (MS.i_q_setpoint < -i32_lim) MS.i_q_setpoint = -i32_lim;
				}
			}

			//overcurrent trip recovery: after the request has been released for 1 s (3 trips: off until restart)
			if (ui8_overcurrent) {
				static uint32_t ui32_oc_released_since = 0;
				if (MS.i_q_setpoint || ui8_oc_trips >= 3) ui32_oc_released_since = 0;
				else if (!ui32_oc_released_since) ui32_oc_released_since = HAL_GetTick() | 1;
				else if (HAL_GetTick() - ui32_oc_released_since > 1000) { ui8_overcurrent = 0; ui32_oc_released_since = 0; }
			}

#ifndef SENSORLESS //KV detect is started by the hall calibration only
			//auto KV detect
			if(ui8_KV_detect_flag){
				if(ui16_KV_detect_counter>32){
					ui8_KV_detect_flag++;
					ui16_KV_detect_counter=0;
					ui8_setup_progress = (MS.u_abs>=1900) ? 100 : (uint8_t)((MS.u_abs*100)/1900);
					if(MS.u_abs>1900 || ui8_KV_detect_flag>=250){ //250: stop before the uint8 counter wraps to 0
						ui8_setup_state = 3;
						ui8_setup_progress = 100;
						ui8_KV_detect_flag=0;
						HAL_FLASH_Unlock();
						EE_WriteVariable(EEPROM_POS_KV, (int16_t) ui32_KV);
						if(ui32_KV>0) ui8_kv_valid=1;
						HAL_FLASH_Lock();
					}
				}
				ui32_KV -=ui32_KV>>4;
				ui32_KV += ((uint32_SPEEDx100_cumulated*_T))/(MS.Voltage*MS.u_q);


			}//end KV detect
#endif



			//------------------------------------------------------------------------------------------------------------
#ifndef SENSORLESS
			//coast with the switches open when no current is wanted for 100 ms (no braking / uncontrolled charging
			//of a fast spinning direct drive motor; switched on again below with the speed based PI start value)
			{
				static uint32_t ui32_no_current_since = 0;
				//only with a measured KV: switching on again at speed pre-loads the voltage from KV; with the default KV
				//the simulation showed up to 45 A at switch-on, so without a calibration the PWM stays on (old behaviour)
				if(MS.i_q_setpoint || brake_flag || ui8_KV_detect_flag || !MS.hall_angle_detect_flag || !ui8_kv_valid || i32_fw_id) ui32_no_current_since = HAL_GetTick(); //not while field weakening: PWM off above the base speed = uncontrolled diode regen
				else if(READ_BIT(TIM1->BDTR, TIM_BDTR_MOE) && (HAL_GetTick() - ui32_no_current_since) > 100){
					CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
					i32_uq_coast = MS.u_q;                   //back-EMF voltage at this speed, for switching on again
					ui32_tics_coast = uint32_tics_filtered>>3;
					TIM1->CCR1 = 1023;
					TIM1->CCR2 = 1023;
					TIM1->CCR3 = 1023;
					q31_t_Battery_Current_accumulated = 0; //runPIcontrol doesn't run while coasting: don't show a stale current
					MS.Battery_Current = 0;
				}
			}
#endif
			//enable PWM if power is wanted
			if (MS.i_q_setpoint&&!READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)&&!ui8_overcurrent){
				i32_iq_ramped16 = 0;   //current ramp starts from zero
				PI_id.integral_part = 0; //no stale d voltage (e.g. from field weakening before the PWM went off)
				PI_id.out = 0;

				uint16_half_rotation_counter=0;
				uint16_full_rotation_counter=0;
				TIM1->CCR1 = 1023; //set initial PWM values
				TIM1->CCR2 = 1023;
				TIM1->CCR3 = 1023;
				//the PWM is switched on only after the controllers have their start values (below), with the interrupt held
				//off: it resets the controllers while the PWM is off and would wipe the start values in between
				__disable_irq();
				if(SystemState == Stop){
					speed_PLL(0,0,0);//reset the speed PLL
					//start the current controllers from 0. While the PWM is off PI_control() doesn't run, so its own reset
					//never happened: after coasting off at full speed the integral still held the full speed voltage and the
					//next start from standstill put ~1700 of 2000 on a stopped motor -> overcurrent trip, 3 trips = off until
					//power cycle (found on the real motor: release at speed, stop, throttle again)
					PI_iq.integral_part = 0; PI_iq.out = 0;
					MS.u_q = 0; MS.u_d = 0;
				}
				else if (ui32_tics_coast && ui16_timertics) {
					//the hall interval now is at least the time since the last hall edge (the motor slows down while
					//coasting, the last measured interval is too short until the next edge comes)
					uint32_t ui32_tics_now = __HAL_TIM_GET_COUNTER(&htim2);
					if (ui32_tics_now < ui16_timertics) ui32_tics_now = ui16_timertics;
					//switching on while the motor still turns after coasting: start at the voltage it had at 0 A when the
					//PWM went off, scaled with the speed (hall interval then / now). Needs no KV. The KV based start value
					//was ~20 % off and tripped the overcurrent protection about every second time on the real motor.
					int32_t i32_u = (i32_uq_coast * (int32_t)(ui32_tics_coast > 65535 ? 65535 : ui32_tics_coast)) / (int32_t)ui32_tics_now; //|u_q| <= 2000: fits 32 bit (64 bit division costs ~1 KB)
					if (i32_u > _U_MAX) i32_u = _U_MAX;
					if (i32_u < -_U_MAX) i32_u = -_U_MAX;
					PI_iq.integral_part = i32_u << PI_iq.shift;
					PI_iq.out = PI_iq.integral_part;
					MS.u_q = i32_u; MS.u_d = 0;
				}
				else {
					PI_iq.integral_part = ((((uint32_SPEEDx100_cumulated*_T))/(MS.Voltage*ui32_KV))<<4)<<PI_iq.shift;
					PI_iq.out=PI_iq.integral_part;
				}
				ui32_tics_coast = 0;
				ui8_oc_count = 0;
				SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
				__enable_irq();
				i8_recent_rotor_direction=i8_direction*i8_reverse_flag*sign(MS.i_q_setpoint);
				if(SystemState == Stop){
					__HAL_TIM_SET_COUNTER(&htim2,0); //reset tim2 counter
					ui16_timertics=20000; //set interval between two hallevents to a large value
					get_standstill_position();
				}
				//still turning: keep the measured hall timing and angle. Resetting them (timer 0, interval 20000, sector
				//angle after a 100 ms wait) threw the angle up to 60 deg off right when the voltage is switched on at speed
#ifdef SENSORLESS
				sensorless_reset();
#endif
			}


			//----------------------------------------------------------------------------------------------------------------------------------------------------------
			//slow loop procedere @16Hz, for LEV standard every 4th loop run, send page,
			if(ui32_tim3_counter>500){
#ifdef SENSORLESS
				sensorless_slow();
#endif
				if(ui8_watchdog_reset) HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				else HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);



				if(ui8_KV_detect_flag){ui16_KV_detect_counter++;}
				//NTC on the speed/temperature wire, filtered without speed pulses. An open input (no NTC, ~3.3 V) or a short
				//reads as an impossible temperature: ignore it instead of derating to zero / reporting overheat.
				if(!RS_FLAG(SF_TEMP_FROM_WIRE)){
					ui8_motor_temp_sensor_ok = 0;
					MS.Temperature = 25;
				}
				else if(ui16_motor_temp_adc > 4000 || ui16_motor_temp_adc <= RS.speed_temp_threshold){
					ui8_motor_temp_sensor_ok = 0;
					MS.Temperature = 25;
				}
				else {
					ui8_motor_temp_sensor_ok = 1;
					MS.Temperature = T_NTC(ui16_motor_temp_adc); //Thank you Hendrik ;-)
				}
				//filter internal temperature reading
				ui32_int_Temp_cumulated-=ui32_int_Temp_cumulated>>5;
				ui32_int_Temp_cumulated+=adcData[7];
				MS.int_Temperature=(((i16_int_Temp_V25-(ui32_int_Temp_cumulated>>5))*24)>>7)+25;

				MS.Voltage=adcData[0];
				if(uint32_SPEED_counter>32000){
					MS.Speed = 32000;
					if(RS_FLAG(SF_SPEED_EXTERNAL)) uint32_SPEEDx100_cumulated=0;
				}

#ifdef INDIVIDUAL_MODES
				// GET recent speedcase for assist profile
				if (uint32_tics_filtered>>3 > speed_to_tics(assist_profile[0][1]))ui8_speedcase=0;
				else if (uint32_tics_filtered>>3 < speed_to_tics(assist_profile[0][1]) && uint32_tics_filtered>>3 > speed_to_tics(assist_profile[0][2]))ui8_speedcase=1;
				else if (uint32_tics_filtered>>3 < speed_to_tics(assist_profile[0][2]) && uint32_tics_filtered>>3 > speed_to_tics(assist_profile[0][3]))ui8_speedcase=2;
				else if (uint32_tics_filtered>>3 < speed_to_tics(assist_profile[0][3]) && uint32_tics_filtered>>3 > speed_to_tics(assist_profile[0][4]))ui8_speedcase=3;
				else if (uint32_tics_filtered>>3 < speed_to_tics(assist_profile[0][4]))ui8_speedcase=4;

				ui8_speedfactor = map(uint32_tics_filtered>>3,speed_to_tics(assist_profile[0][ui8_speedcase+1]),speed_to_tics(assist_profile[0][ui8_speedcase]),assist_profile[1][ui8_speedcase+1],assist_profile[1][ui8_speedcase]);


#endif
				//check if rotor is turning

				if((uint16_full_rotation_counter>7999||uint16_half_rotation_counter>7999)){
					SystemState = Stop;
					//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
					if(READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)){
						CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); //Disable PWM if motor is not turning
						uint32_tics_filtered=1000000;
						get_standstill_position();
					}

				}
				else if(ui8_6step_flag) SystemState = SixStep;
				else SystemState = Running;


#if (DISPLAY_TYPE == DISPLAY_TYPE_EBiCS)
				ui8_slowloop_counter++;
				if(ui8_slowloop_counter>3){
					ui8_slowloop_counter = 0;

					switch (ui8_main_LEV_Page_counter){
					case 1: {
						ui8_LEV_Page_to_send = 1;
					}
					break;
					case 2: {
						ui8_LEV_Page_to_send = 2;
					}
					break;
					case 3: {
						ui8_LEV_Page_to_send = 3;
					}
					break;
					case 4: {
						//to do, define other pages
						ui8_LEV_Page_to_send = 4;
					}
					break;
					}//end switch

					//  send_ant_page(ui8_LEV_Page_to_send, &MS, &MP);

					ui8_main_LEV_Page_counter++;
					if(ui8_main_LEV_Page_counter>4)ui8_main_LEV_Page_counter=1;
				}

#endif
				ui32_tim3_counter=0;
			}// end of slow loop

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

		RCC_OscInitTypeDef RCC_OscInitStruct;
		RCC_ClkInitTypeDef RCC_ClkInitStruct;
		RCC_PeriphCLKInitTypeDef PeriphClkInit;

		/**Initializes the CPU, AHB and APB busses clocks
		 */
		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
		RCC_OscInitStruct.HSIState = RCC_HSI_ON;
		RCC_OscInitStruct.HSICalibrationValue = 16;
		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
		RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
		RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
		if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Initializes the CPU, AHB and APB busses clocks
		 */
		RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
				|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
		RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
		RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
		PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure the Systick interrupt time
		 */
		HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

		/**Configure the Systick
		 */
		HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

		/* SysTick_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	}


	/* ADC1 init function */
	static void MX_ADC1_Init(void)
	{

		ADC_MultiModeTypeDef multimode;
		ADC_InjectionConfTypeDef sConfigInjected;
		ADC_ChannelConfTypeDef sConfig;

		/**Common config
		 */
		hadc1.Instance = ADC1;
		hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE; //Scan muÃ fÃ¼r getriggerte Wandlung gesetzt sein
		hadc1.Init.ContinuousConvMode = DISABLE;
		hadc1.Init.DiscontinuousConvMode = DISABLE;
		hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;// Trigger regular ADC with timer 3 ADC_EXTERNALTRIGCONV_T1_CC1;// // ADC_SOFTWARE_START; //
		hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
		hadc1.Init.NbrOfConversion = 8;
		hadc1.Init.NbrOfDiscConversion = 0;


		if (HAL_ADC_Init(&hadc1) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure the ADC multi-mode
		 */
		multimode.Mode = ADC_DUALMODE_REGSIMULT_INJECSIMULT;
		if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure Injected Channel
		 */
		sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
		sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
		sConfigInjected.InjectedNbrOfConversion = 1;
		sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
		sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4; // Hier bin ich nicht sicher ob Trigger out oder direkt CC4
		sConfigInjected.AutoInjectedConv = DISABLE; //muÃ aus sein
		sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
		sConfigInjected.InjectedOffset = ui16_ph1_offset;//1900;
		HAL_ADC_Stop(&hadc1); //ADC muÃ gestoppt sein, damit Triggerquelle gesetzt werden kann.
		if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure Regular Channel
		 */
		sConfig.Channel = ADC_CHANNEL_7; //battery voltage
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}


		/**Configure Regular Channel
		 */
		sConfig.Channel = ADC_CHANNEL_3; //Connector SP: throttle input
		sConfig.Rank = ADC_REGULAR_RANK_2;
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}
		/**Configure Regular Channel
		 */
		sConfig.Channel = ADC_CHANNEL_4; //Phase current 1
		sConfig.Rank = ADC_REGULAR_RANK_3;
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}
		/**Configure Regular Channel
		 */
		sConfig.Channel = ADC_CHANNEL_5; //Phase current 2
		sConfig.Rank = ADC_REGULAR_RANK_4;
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}
		/**Configure Regular Channel
		 */
		sConfig.Channel = ADC_CHANNEL_6; //Phase current 3
		sConfig.Rank = ADC_REGULAR_RANK_5;
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure Regular Channel
		 */
		sConfig.Channel = ADC_CHANNEL_8;
		sConfig.Rank = ADC_REGULAR_RANK_6; // connector AD2
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure Regular Channel
		 */
		sConfig.Channel = ADC_CHANNEL_9; // connector AD1, temperature or torque input for Controller from PhoebeLiu @ aliexpress
		sConfig.Rank = ADC_REGULAR_RANK_7;
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure Regular Channel
		 */
		sConfig.Channel = ADC_CHANNEL_TEMPSENSOR; // internal STM32 temperature sensor
		sConfig.Rank = ADC_REGULAR_RANK_8;
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

	}

	/* ADC2 init function */
	static void MX_ADC2_Init(void)
	{

		ADC_InjectionConfTypeDef sConfigInjected;

		/**Common config
		 */
		hadc2.Instance = ADC2;
		hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE; //hier auch Scan enable?!
		hadc2.Init.ContinuousConvMode = DISABLE;
		hadc2.Init.DiscontinuousConvMode = DISABLE;
		hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
		hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
		hadc2.Init.NbrOfConversion = 1;
		if (HAL_ADC_Init(&hadc2) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure Injected Channel
		 */
		sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
		sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
		sConfigInjected.InjectedNbrOfConversion = 1;
		sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
		sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
		sConfigInjected.AutoInjectedConv = DISABLE;
		sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
		sConfigInjected.InjectedOffset = ui16_ph2_offset;//	1860;
		if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

	}
	/* TIM1 init function */
	static void MX_TIM1_Init(void)
	{

		TIM_ClockConfigTypeDef sClockSourceConfig;
		TIM_MasterConfigTypeDef sMasterConfig;
		TIM_OC_InitTypeDef sConfigOC;
		TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

		htim1.Instance = TIM1;
		htim1.Init.Prescaler = 0;
		htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
		htim1.Init.Period = _T;
		htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim1.Init.RepetitionCounter = 0;
		htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 1;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		//sConfigOC.OCMode = TIM_OCMODE_ACTIVE; // war hier ein Bock?!
		sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
		if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
		sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
		sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
		sBreakDeadTimeConfig.DeadTime = 32;
		sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
		sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
		sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
		if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		HAL_TIM_MspPostInit(&htim1);

	}

	/* TIM2 init function */
	static void MX_TIM2_Init(void)
	{
		TIM_ClockConfigTypeDef sClockSourceConfig;
		TIM_SlaveConfigTypeDef sSlaveConfig;
		TIM_MasterConfigTypeDef sMasterConfig;
		TIM_IC_InitTypeDef sConfigIC;

		htim2.Instance = TIM2;
		htim2.Init.Prescaler = 128;
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.Period = 65535;
		htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
		sSlaveConfig.InputTrigger = TIM_TS_TI1F_ED;
		sSlaveConfig.TriggerFilter = 8;
		if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		sConfigIC.ICSelection = TIM_ICSELECTION_TRC;
		sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		sConfigIC.ICFilter = 15;
		if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		if (HAL_TIM_ConfigTI1Input(&htim2, TIM_TI1SELECTION_XORCOMBINATION) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

	}

	/* TIM3 init function 8kHz interrupt frequency for regular adc triggering */
	static void MX_TIM3_Init(void)
	{

		TIM_ClockConfigTypeDef sClockSourceConfig;
		TIM_MasterConfigTypeDef sMasterConfig;

		htim3.Instance = TIM3;
		htim3.Init.Prescaler = 0;
		htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim3.Init.Period = 7813;
		htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		HAL_TIM_MspPostInit(&htim3);

	}

	/* USART1 init function */
	static void MX_USART1_UART_Init(void)
	{

		huart1.Instance = USART1;

#if ((DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER) ||DISPLAY_TYPE==DISPLAY_TYPE_KUNTENG||DISPLAY_TYPE==DISPLAY_TYPE_EBiCS||DISPLAY_TYPE==DISPLAY_TYPE_NO2 || DISPLAY_TYPE==DISPLAY_TYPE_BAFANG_850_860 || DISPLAY_TYPE==DISPLAY_TYPE_TONGSHENG)
		huart1.Init.BaudRate = 9600;
#elif (DISPLAY_TYPE == DISPLAY_TYPE_VESC)
		huart1.Init.BaudRate = 115200;
#elif (DISPLAY_TYPE == DISPLAY_TYPE_BAFANG_LCD)
		huart1.Init.BaudRate = 1200; 
#else
		huart1.Init.BaudRate = 56000;
#endif


		huart1.Init.WordLength = UART_WORDLENGTH_8B;
		huart1.Init.StopBits = UART_STOPBITS_1;
		huart1.Init.Parity = UART_PARITY_NONE;
		huart1.Init.Mode = UART_MODE_TX_RX;
		huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart1.Init.OverSampling = UART_OVERSAMPLING_16;
		if (HAL_UART_Init(&huart1) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
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
		HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
		//  /* DMA1_Channel4_IRQn interrupt configuration */
		//  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 3, 0);
		//  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
		//  /* DMA1_Channel5_IRQn interrupt configuration */
		//  //HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
		//  //HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

	}

	/** Configure pins as
	 * Analog
	 * Input
	 * Output
	 * EVENT_OUT
	 * EXTI
	 */
	static void MX_GPIO_Init(void)
	{

		GPIO_InitTypeDef GPIO_InitStruct;

		/* GPIO Ports Clock Enable */
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

		/*Configure GPIO pins : Hall_1_Pin Hall_2_Pin Hall_3_Pin */
		GPIO_InitStruct.Pin = Hall_1_Pin|Hall_2_Pin|Hall_3_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/*Configure GPIO pin : LED_Pin */
		GPIO_InitStruct.Pin = LED_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

		/*Configure GPIO pin : LIGHT_Pin */
		GPIO_InitStruct.Pin = LIGHT_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(LIGHT_GPIO_Port, &GPIO_InitStruct);

		/*Configure GPIO pin : BRAKE_LIGHT_Pin */
		GPIO_InitStruct.Pin = BRAKE_LIGHT_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(BRAKE_LIGHT_GPIO_Port, &GPIO_InitStruct);

		/*Configure GPIO pin : Brake_Pin */
		GPIO_InitStruct.Pin = Brake_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(Brake_GPIO_Port, &GPIO_InitStruct);


		/*Configure GPIO pins : Speed_EXTI5_Pin PAS_EXTI8_Pin */
		GPIO_InitStruct.Pin = Speed_EXTI5_Pin|PAS_EXTI8_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


		/* EXTI interrupt init*/


		HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	}

	/* USER CODE BEGIN 4 */

	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
		//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		if (htim == &htim3) {

#if SPEED_PLL
			if(!READ_BIT(TIM1->BDTR, TIM_BDTR_MOE))q31_rotorposition_PLL += (q31_angle_per_tic<<1);
#endif

			if(ui32_tim3_counter<32000)ui32_tim3_counter++;
			if(ui16_ticks_since_hall < 65535) ui16_ticks_since_hall++;
			if(ui16_ticks_since_hall > 1000) ui8_hall_edges = 0;   //125 ms without hall edge: standing still
			if (uint32_PAS_counter < RS.pas_timeout+1){
				uint32_PAS_counter++;
				if(HAL_GPIO_ReadPin(PAS_GPIO_Port, PAS_Pin))uint32_PAS_HIGH_counter++;
			}
			if (uint32_SPEED_counter<128000)uint32_SPEED_counter++;					//counter for external Speedsensor
			if(uint16_full_rotation_counter<8000)uint16_full_rotation_counter++;	//full rotation counter for motor standstill detection
			if(uint16_half_rotation_counter<8000)uint16_half_rotation_counter++;	//half rotation counter for motor standstill detection

		}
		//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}



	// regular ADC callback
	void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
	{
		ui8_adc_regular_flag=1;
		if(RS.flags & (SF_SPEED_FROM_WIRE | SF_TEMP_FROM_WIRE))
		{
			// motor speed sensor and NTC on one wire: the NTC sits between signal and GND, the speed sensor
			// pulls the signal to ~0 V when the magnet passes. Low = speed pulse, everything else = temperature.
			static uint8_t ui8_speed_input_low = 0;
			uint16_t v = adcData[RS.speed_temp_adc];
			if(v < RS.speed_temp_threshold){
				if(!ui8_speed_input_low && RS_FLAG(SF_SPEED_FROM_WIRE)) ui8_SPEED_flag = 1; //falling edge = one speed sensor pulse
				ui8_speed_input_low = 1;
			}
			else if(v > RS.speed_temp_threshold + (RS.speed_temp_threshold>>1) + 20){ //hysteresis
				ui8_speed_input_low = 0;
				ui32_motor_temp_adc_cumulated -= ui32_motor_temp_adc_cumulated>>4;
				ui32_motor_temp_adc_cumulated += v;
				ui16_motor_temp_adc = ui32_motor_temp_adc_cumulated>>4;
			}
		}

	}

	//injected ADC

	void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
	{
		//for oszi-check of used time in FOC procedere
		//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		ui32_tim1_counter++;

		/*  else {
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  uint32_SPEED_counter=0;
	  }*/

		if(!ui8_adc_offset_done_flag)
		{
			i16_ph1_current = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
			i16_ph2_current = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

			ui8_adc_inj_flag=1;
		}
		else{

#ifdef DISABLE_DYNAMIC_ADC

			i16_ph1_current = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
			i16_ph2_current = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);


#else
			switch (MS.char_dyn_adc_state) //read in according to state
			{
			case 1: //Phase C at high dutycycles, read from A+B directly
			{
				temp1=(q31_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
				i16_ph1_current = temp1 ;

				temp2=(q31_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
				i16_ph2_current = temp2;
			}
			break;
			case 2: //Phase A at high dutycycles, read from B+C (A = -B -C)
			{

				temp2=(q31_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
				i16_ph2_current = temp2;

				temp1=(q31_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
				i16_ph1_current = -i16_ph2_current-temp1;

			}
			break;
			case 3: //Phase B at high dutycycles, read from A+C (B=-A-C)
			{
				temp1=(q31_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
				i16_ph1_current = temp1 ;
				temp2=(q31_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
				i16_ph2_current = -i16_ph1_current-temp2;
			}
			break;

			case 0: //timeslot too small for ADC
			{
				//do nothing
			}
			break;




			} // end case
#endif

			//software overcurrent trip: a phase current above the limit for 4 FOC cycles in a row switches the PWM off
			//(the hardware break input is not used on this board)
			{
				int16_t i16_a = i16_ph1_current < 0 ? -i16_ph1_current : i16_ph1_current;
				int16_t i16_b = i16_ph2_current < 0 ? -i16_ph2_current : i16_ph2_current;
				int16_t i16_c = i16_ph1_current + i16_ph2_current;
				if (i16_c < 0) i16_c = -i16_c;
				if (i16_b > i16_a) i16_a = i16_b;
				if (i16_c > i16_a) i16_a = i16_c;
				if (i16_a > i16_oc_limit && READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)) {
					if (++ui8_oc_count >= 4) {
						CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
						i16_oc_capture[0] = MS.u_q; i16_oc_capture[1] = MS.u_d;
						i16_oc_capture[2] = i16_ph1_current; i16_oc_capture[3] = i16_ph2_current;
						i16_oc_capture[4] = TIM1->CCR1; i16_oc_capture[5] = TIM1->CCR2; i16_oc_capture[6] = TIM1->CCR3;
						i16_oc_capture[7] = ui16_moe_cycles; i16_oc_capture[8] = ui8_setup_state;
						ui8_overcurrent = 1;
						if (ui8_oc_trips < 255) ui8_oc_trips++;
						ui8_oc_count = 0;
					}
				}
				else ui8_oc_count = 0;
			}

			__disable_irq(); //ENTER CRITICAL SECTION!!!!!!!!!!!!!

			//extrapolate recent rotor position
			ui16_tim2_recent = __HAL_TIM_GET_COUNTER(&htim2); // read in timertics since last event
#ifndef SENSORLESS
			if (MS.hall_angle_detect_flag) {
				if(ui16_timertics<SIXSTEPTHRESHOLD && ui16_tim2_recent<200)ui8_6step_flag=0;
				if(ui16_timertics>(SIXSTEPTHRESHOLD*6)>>2)ui8_6step_flag=1;


				if(MS.angle_est){
					q31_rotorposition_PLL += q31_angle_per_tic;
				}
				//interpolate from the second hall edge on at any speed (was: only above the 6-step threshold, the 6-step
				//region caused +-30 deg angle jumps = the noise at start-up). 6-step only before two edges or after a stop.
				if (ui16_tim2_recent < ui16_timertics+(ui16_timertics>>2) && !ui8_overflow_flag
						&& ui8_hall_edges >= 2 && ui16_ticks_since_hall < 1000 && ui16_timertics < 65535) { //prevent angle running away at standstill
					if(MS.angle_est&&iabs(q31_PLL_error)<deg_30&&!ui8_6step_flag){
						q31_rotorposition_absolute=q31_rotorposition_PLL;
						MS.system_state=PLL;
					}
					else{
						q31_rotorposition_absolute = q31_rotorposition_hall
								+ (q31_t) (i8_recent_rotor_direction
										* ((10923 * ui16_tim2_recent) / ui16_timertics)
										<< 16); //interpolate angle between two hallevents by scaling timer2 tics, 10923<<16 is 715827883 = 60deg
						MS.system_state=Interpolation;
					}
				} else {
					ui8_overflow_flag = 1;
					if(MS.KV_detect_flag)q31_rotorposition_absolute = q31_rotorposition_hall;
					else q31_rotorposition_absolute = q31_rotorposition_hall+i8_direction*sign(MS.i_q_setpoint)*deg_30;//offset of 30 degree to get the middle of the sector
					MS.system_state=SixStep;
					//	}

				}
			} //end if hall angle detect
#endif
			//temp2=(((q31_rotorposition_absolute >> 23) * 180) >> 8);
			__enable_irq(); //EXIT CRITICAL SECTION!!!!!!!!!!!!!!

#ifndef DISABLE_DYNAMIC_ADC

			//get the Phase with highest duty cycle for dynamic phase current reading
#ifdef SENSORLESS
			dyn_adc_state(q31_teta_obs); //was the hall angle (constant without halls): wrong phases sampled at high duty
#else
			dyn_adc_state(q31_rotorposition_absolute);
#endif
			//set the according injected channels to read current at Low-Side active time

			if (MS.char_dyn_adc_state!=char_dyn_adc_state_old){
				set_inj_channel(MS.char_dyn_adc_state);
				char_dyn_adc_state_old = MS.char_dyn_adc_state;
			}
#endif

			//int16_current_target=0;
			// call FOC procedure if PWM is enabled

#if defined(SENSORLESS) && defined(PARAM_ID)
			if (ui8_pid_rotate) q31_teta_obs += i32_pid_step;   //parameter identification: open loop field
#endif
			if (ui8_tone_on) {
				//startup melody: voltage u = amp * sin(tone) along the phase axis (ca, sa), no FOC
				q31_t q31_s, q31_c;
				uint8_t k;
				int16_t i16_i[3] = {i16_ph1_current, i16_ph2_current, -i16_ph1_current - i16_ph2_current};
				ui32_tone_phase += ui32_tone_step;
				arm_sin_cos_q31((q31_t) ui32_tone_phase, &q31_s, &q31_c);
				int32_t i32_u = ((int32_t) i16_tone_amp * (q31_s >> 16)) >> 15;
				svpwm((i32_u * i16_tone_ca) >> 10, (i32_u * i16_tone_sa) >> 10);
				for (k = 0; k < 3; k++) {
					uint16_t ui16_a = i16_i[k] < 0 ? -i16_i[k] : i16_i[k];
					if (ui16_a > ui16_tone_peak[k]) ui16_tone_peak[k] = ui16_a;
				}
			}
			else if (READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)){
#ifdef SENSORLESS
				FOC_calculation(i16_ph1_current, i16_ph2_current, q31_teta_obs, (((int16_t)i8_direction*i8_reverse_flag)*MS.i_q_setpoint), &MS);
#else
				FOC_calculation(i16_ph1_current, i16_ph2_current, q31_rotorposition_absolute, (((int16_t)i8_direction*i8_reverse_flag)*MS.i_q_setpoint), &MS);
#endif
			}
			if (READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)) { if (ui16_moe_cycles < 65535) ui16_moe_cycles++; }
			else {
				ui16_moe_cycles = 0;
				//PWM off: nothing stale at the next switch-on (field weakening, current controllers). PI_control() resets
				//itself only when it runs, and it doesn't run while the PWM is off: after coasting off at full speed the
				//next start (throttle or hall autodetect) put the old full speed voltage on a stopped motor -> overcurrent
				i32_fw_id16 = 0; i32_fw_id = 0; i32_u_demand16 = 0;
				PI_iq.integral_part = 0; PI_iq.out = 0;
				PI_id.integral_part = 0; PI_id.out = 0;
				//zero voltage vector: the compare registers below are written every cycle, they kept the last pattern
				//(full speed voltage after coasting off) and the next switch-on put it on the stopped motor
				MS.u_q = 0; MS.u_d = 0;
				switchtime[0] = 1023; switchtime[1] = 1023; switchtime[2] = 1023;
			}
			//temp5=__HAL_TIM_GET_COUNTER(&htim1);
			//set PWM

			TIM1->CCR1 =  (uint16_t) switchtime[0];
			TIM1->CCR2 =  (uint16_t) switchtime[1];
			TIM1->CCR3 =  (uint16_t) switchtime[2];
			//__enable_irq();


			//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

		} // end else

	}

	void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
	{
		//__HAL_TIM_SET_COUNTER(&htim2,0); //reset tim2 counter
		//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

		temp5=TIM3->CNT;

		//TIM2 is 16 bit at 496 kHz and wraps after 132 ms: after a long pause CCR1 holds a nonsense (wrapped) interval
		if(ui16_ticks_since_hall > 1000) ui16_timertics = 65535;
		else if(TIM2->CCR1>20)ui16_timertics = TIM2->CCR1; //debounce hall signals
		ui16_ticks_since_hall = 0;
		if(ui8_hall_edges < 255) ui8_hall_edges++;


		//Hall sensor event processing

		ui8_hall_state = GPIOA->IDR & 0b111; //Mask input register with Hall 1 - 3 bits
		ui32_hall_events++;


		ui8_hall_case=ui8_hall_state_old*10+ui8_hall_state;
		if(MS.hall_angle_detect_flag){ //only process, if autodetect procedere is fininshed
			ui8_hall_state_old=ui8_hall_state;
		}
#ifndef SENSORLESS
#if (USE_FIX_POSITIONS)
//Check for 60� hall configuration
		if(ui8_hall_state==0)i16_60deg_Hall_flag |= 0b1;
		if(ui8_hall_state==7)i16_60deg_Hall_flag |= 0b10;

		if((i16_60deg_Hall_flag&0b111) == 0b11){
			i16_60deg_Hall_flag = 0b111;

			Set_Hall_Logic();

		}
//Check for 120� hall configuration
		if(ui8_hall_state==2)i16_60deg_Hall_flag |= 0b1000;
		if(ui8_hall_state==5)i16_60deg_Hall_flag |= 0b10000;
		if(i16_60deg_Hall_flag>>3==0b11){
			i16_60deg_Hall_flag=0b111000;
			Set_Hall_Logic();

		}

		if(i16_60deg_Hall_flag>7)Set_Hall_Angle120();
		else Set_Hall_Angle60();
#else
		Set_Hall_Angle120();
#endif
#endif //SENSORLESS

		uint32_tics_filtered-=uint32_tics_filtered>>3;
		uint32_tics_filtered+=ui16_timertics;

		ui8_overflow_flag=0;
		ui8_SPEED_control_flag=1;





		if(MS.angle_est){
			q31_PLL_error=q31_rotorposition_PLL-q31_rotorposition_hall;
			q31_angle_per_tic = speed_PLL(q31_rotorposition_PLL,q31_rotorposition_hall,0);
		}

#if SPEED_PLL
		if(ui16_erps>30){   //360 interpolation at higher erps
			if(ui8_hall_case==32||ui8_hall_case==23){
				q31_angle_per_tic = speed_PLL(q31_rotorposition_PLL,q31_rotorposition_hall, SPDSHFT*tics_higher_limit/(uint32_tics_filtered>>3));

			}
		}
		else{

			q31_angle_per_tic = speed_PLL(q31_rotorposition_PLL,q31_rotorposition_hall, SPDSHFT*tics_higher_limit/(uint32_tics_filtered>>3));
		}

#endif

		//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);


	}

	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
		//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		//PAS processing
		if(GPIO_Pin == PAS_EXTI8_Pin)
		{
			ui8_PAS_flag = 1;
			ui32_pas_events++;
		}

		//Speed processing
		if(GPIO_Pin == Speed_EXTI5_Pin)
		{

			ui8_SPEED_flag = 1; //with debounce

		}
		//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}

	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
	{
		//ui8_UART_flag=1;

	}

	void UART_IdleItCallback(void)
	{
		ui8_UART_flag=1;

	}

	void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
	{
		ui8_UART_TxCplt_flag=1;
	}

	void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER)
		KingMeter_Init (&KM);
#endif

#if (DISPLAY_TYPE & DISPLAY_TYPE_BAFANG)
		Bafang_Init (&BF);
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KUNTENG)
		kunteng_init();
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_EBiCS)
		//       ebics_init();
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_NO2)
	No2_Init(&No2);
#endif
#if (DISPLAY_TYPE == DISPLAY_TYPE_TONGSHENG)
	Tongsheng_Init(&TS);
#endif
#if (DISPLAY_TYPE == DISPLAY_TYPE_VESC)
	Vesc_Init(&VESC);
	MS.assist_level = 153; //no assist level from VESC devices: PAS at level 3 of 5
#endif
	}

	void get_internal_temp_offset(void){
		int32_t temp=0;
		for(i=0;i<32;i++){
			while(!ui8_adc_regular_flag){}
			temp+=adcData[7];
			ui8_adc_regular_flag=0;
		}
		HAL_FLASH_Unlock();
		EE_WriteVariable(EEPROM_INT_TEMP_V25,temp>>5);
		HAL_FLASH_Lock();
	}
#if (DISPLAY_TYPE == DISPLAY_TYPE_NO2)
	void No2_update(void)
	{
		/* Prepare Tx parameters */

		No2.Tx.Wheeltime_ms = get_wheeltime_ms(); //clamped, works for hall, external sensor and sensorless
		if(MS.Temperature>RS.motor_temp_max) No2.Tx.Error = 7;  //motor failure
		else if(MS.int_Temperature>CONTROLLER_TEMPERATURE_MAX)No2.Tx.Error = 9; //controller failure
		else No2.Tx.Error = 0; //no failure


		No2.Tx.Current_x10 = (uint16_t) (MS.Battery_Current/100); //MS.Battery_Current is in mA
		No2.Tx.BrakeActive=brake_flag;

		/* Apply Rx parameters */

		MS.assist_level = assist_from_level(No2.Rx.AssistLevel >> 4); //display level 0 ... 15 (<<4), % per level from the settings

		if(!No2.Rx.Headlight)
		{
			HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);

		}
		else // KM_HEADLIGHT_ON, KM_HEADLIGHT_LOW, KM_HEADLIGHT_HIGH
		{
			HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);

		}


		if(No2.Rx.PushAssist)
		{
			ui8_Push_Assist_flag=1;
		}
		else
		{
			ui8_Push_Assist_flag=0;
		}

	}

#endif

#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER)
	void kingmeter_update(void)
	{
		/* Prepare Tx parameters */

		if(battery_percent_fromcapacity > 10)
		{
			KM.Tx.Battery = KM_BATTERY_NORMAL;
		}
		else
		{
			KM.Tx.Battery = KM_BATTERY_LOW;
		}


		KM.Tx.Wheeltime_ms = get_wheeltime_ms(); //clamped, works for hall, external sensor and sensorless
		if(MS.Temperature>130) KM.Tx.Error = KM_ERROR_OVHT;
		else if(MS.int_Temperature>80)KM.Tx.Error = KM_ERROR_IOVHT;
		else KM.Tx.Error = KM_ERROR_NONE;


		KM.Tx.Current_x10 = (uint16_t) (MS.Battery_Current/100); //MS.Battery_Current is in mA


		/* Receive Rx parameters/settings and send Tx parameters */
#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_618U)
		KingMeter_Service(&KM);
#endif


		/* Apply Rx parameters */

		MS.assist_level = KM.Rx.AssistLevel;

		if(KM.Rx.Headlight == KM_HEADLIGHT_OFF)
		{
			HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);

		}
		else // KM_HEADLIGHT_ON, KM_HEADLIGHT_LOW, KM_HEADLIGHT_HIGH
		{
			HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);

		}


		if(KM.Rx.PushAssist == KM_PUSHASSIST_ON)
		{
			ui8_Push_Assist_flag=1;
		}
		else
		{
			ui8_Push_Assist_flag=0;
		}
//	    if( KM.Settings.Reverse)i8_direction = -1;
//	    else i8_direction = 1;
		//    MP.speedLimit=KM.Rx.SPEEDMAX_Limit;
		//    MP.battery_current_max = KM.Rx.CUR_Limit_mA;



	}

#endif

#if (DISPLAY_TYPE & DISPLAY_TYPE_BAFANG)
	void bafang_update(void)
	{
		/* Prepare Tx parameters */

		/* if(MS.Voltage*CAL_BAT_V>RS.battery_level[4])battery_percent_fromcapacity=95; */
		/* else if(MS.Voltage*CAL_BAT_V>RS.battery_level[3])battery_percent_fromcapacity=80; */
		/* else if(MS.Voltage*CAL_BAT_V>RS.battery_level[2])battery_percent_fromcapacity=50; */
		/* else if(MS.Voltage*CAL_BAT_V>RS.battery_level[1])battery_percent_fromcapacity=30; */
		/* else if(MS.Voltage*CAL_BAT_V>RS.battery_level[0])battery_percent_fromcapacity=20; */
		/* else battery_percent_fromcapacity=5; */

        battery_percent_fromcapacity = ((((MS.Voltage * CAL_BAT_V) - (RS.battery_level[0])) * 100) / ((RS.battery_level[4] - RS.battery_level[0])));
        if (battery_percent_fromcapacity < 0) battery_percent_fromcapacity = 0;
        if (battery_percent_fromcapacity > 100) battery_percent_fromcapacity = 100;

		BF.Tx.Battery = battery_percent_fromcapacity;



		if(RS_FLAG(SF_SPEED_EXTERNAL)){ // Adapt wheeltime to match displayed speedo value
		if(uint32_SPEED_counter<16000) BF.Tx.Speed = (external_tics_to_speedx100(MS.Speed)*20)>>8;// Geschwindigkeit ist Weg pro Zeit Radumfang durch Dauer einer Radumdrehung --> Umfang * 8000*3600/(n*1000000) * Skalierung Bafang Display 200/26,6
		else BF.Tx.Speed = 0;
		}
		else {
		if(motor_is_turning() && MS.system_state != Stop)
		{
			BF.Tx.Speed =(internal_tics_to_speedx100(MS.Speed)*20)>>8; //factor is *20/256, found empiric

		}
		else
		{
			BF.Tx.Speed = 0;
		}
		}

        if (MS.Battery_Current < 0) BF.Tx.Power = MS.Battery_Current*-1 > 0 && MS.Battery_Current*-1 < 500 ? 1 : ((MS.Battery_Current*-1)/500);
        else BF.Tx.Power = MS.Battery_Current > 0 && MS.Battery_Current < 500 ? 1 : (MS.Battery_Current/500); // Unit: 1 digit --> 0.5 A, MS.Battery_Current is in mA


		/* Receive Rx parameters/settings and send Tx parameters */
		Bafang_Service(&BF,1, &MS);



		/* Apply Rx parameters */

		//No headlight supported on my controller hardware.
		if(BF.Rx.Headlight)
		{
			HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);

		}
		else
		{
			HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);

		}


		if(BF.Rx.PushAssist) ui8_Push_Assist_flag=1;
		else ui8_Push_Assist_flag=0;

		MS.assist_level=BF.Rx.AssistLevel;
	}

#endif

	//1 if the motor turns: hall event within the last 24 ms, or the sensorless observer is running
	uint8_t motor_is_turning(void)
	{
#ifdef SENSORLESS
		return ui8_sensorless_running;
#else
		return __HAL_TIM_GET_COUNTER(&htim2) < 12000;
#endif
	}

	//duration of one wheel revolution in ms for the displays, 64000 = standing still
	uint32_t get_wheeltime_ms(void)
	{
		uint32_t t;
		if(RS_FLAG(SF_SPEED_EXTERNAL)) t = ((MS.Speed>>3)*RS.pulses_per_revolution); //8 kHz counter, 8 tics per ms
		else {
			if(!motor_is_turning()) return 64000;
			t = ((uint32_t)MS.Speed*RS.gear_ratio*6)>>9; //500 kHz timer2, 6 hall events per electric revolution
		}
		return (t > 64000) ? 64000 : t;
	}

	int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
	{
		if (in_max <= in_min) return (x <= in_min) ? out_min : out_max; //avoid division by zero -> HardFault
		// if input is smaller/bigger than expected return the min/max out ranges value
		if (x < in_min)
			return out_min;
		else if (x > in_max)
			return out_max;

		// map the input to the output range.
		// round up if mapping bigger ranges to smaller ranges
		else  if ((in_max - in_min) > (out_max - out_min))
			return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
		// round down if mapping smaller ranges to bigger ranges
		else
			return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	//assuming, a proper AD conversion takes 350 timer tics, to be confirmed. DT+TR+TS deadtime + noise subsiding + sample time
	void dyn_adc_state(q31_t angle){
		if (switchtime[2]>switchtime[0] && switchtime[2]>switchtime[1]){
			MS.char_dyn_adc_state = 1; // -90Â° .. +30Â°: Phase C at high dutycycles
			if(switchtime[2]>1500)TIM1->CCR4 =  switchtime[2]-TRIGGER_OFFSET_ADC;
			else TIM1->CCR4 = TRIGGER_DEFAULT;
		}

		if (switchtime[0]>switchtime[1] && switchtime[0]>switchtime[2]) {
			MS.char_dyn_adc_state = 2; // +30Â° .. 150Â° Phase A at high dutycycles
			if(switchtime[0]>1500)TIM1->CCR4 =  switchtime[0]-TRIGGER_OFFSET_ADC;
			else TIM1->CCR4 = TRIGGER_DEFAULT;
		}

		if (switchtime[1]>switchtime[0] && switchtime[1]>switchtime[2]){
			MS.char_dyn_adc_state = 3; // +150 .. -90Â° Phase B at high dutycycles
			if(switchtime[1]>1500)TIM1->CCR4 =  switchtime[1]-TRIGGER_OFFSET_ADC;
			else TIM1->CCR4 = TRIGGER_DEFAULT;
		}
	}

	static void set_inj_channel(char state){
		switch (state)
		{
		case 1: //Phase C at high dutycycles, read current from phase A + B
		{
			ADC1->JSQR=0b00100000000000000000; //ADC1 injected reads phase A JL = 0b00, JSQ4 = 0b00100 (decimal 4 = channel 4)
			ADC1->JOFR1 = ui16_ph1_offset;
			ADC2->JSQR=0b00101000000000000000; //ADC2 injected reads phase B, JSQ4 = 0b00101, decimal 5
			ADC2->JOFR1 = ui16_ph2_offset;


		}
		break;
		case 2: //Phase A at high dutycycles, read current from phase C + B
		{
			ADC1->JSQR=0b00110000000000000000; //ADC1 injected reads phase C, JSQ4 = 0b00110, decimal 6
			ADC1->JOFR1 = ui16_ph3_offset;
			ADC2->JSQR=0b00101000000000000000; //ADC2 injected reads phase B, JSQ4 = 0b00101, decimal 5
			ADC2->JOFR1 = ui16_ph2_offset;


		}
		break;

		case 3: //Phase B at high dutycycles, read current from phase A + C
		{
			ADC1->JSQR=0b00100000000000000000; //ADC1 injected reads phase A JL = 0b00, JSQ4 = 0b00100 (decimal 4 = channel 4)
			ADC1->JOFR1 = ui16_ph1_offset;
			ADC2->JSQR=0b00110000000000000000; //ADC2 injected reads phase C, JSQ4 = 0b00110, decimal 6
			ADC2->JOFR1 = ui16_ph3_offset;


		}
		break;


		}


	}

    uint8_t brake_is_set(void) {
        return !HAL_GPIO_ReadPin(Brake_GPIO_Port, Brake_Pin) || (uint16_mapped_BRAKE > 0);
    }
	uint8_t throttle_is_set(void){
		if(uint16_mapped_throttle > 0)
		{
			return 1;
		}
		else return 0;
	}
	// one open loop sweep of 3 electrical turns (1 deg per 5 ms, d current), dir = 1 forward, -1 backward.
	// Every hall edge: field angle into dst[] (index order 45, 51, 13, 32, 26, 64), found bits per direction.
	static void autodetect_sweep(int8_t i8_dir, q31_t *dst, uint8_t ui8_progress_start) {
		uint8_t ui8_idx;
		for (i = 0; i < 1080; i++) {
			HAL_IWDG_Refresh(&hiwdg);
			q31_rotorposition_absolute += i8_dir * 11930465; //drive motor in open loop with steps of 1 deg
			ui8_setup_progress = ui8_progress_start + (uint8_t)((i*50UL)/1080);
			HAL_Delay(5);

			if (ui8_hall_state_old != ui8_hall_state) {

				switch (ui8_hall_case) //12 cases for each transition from one stage to the next. 6x forward, 6x reverse
				{
				case 45: case 54: ui8_idx = 0; break;
				case 51: case 15: ui8_idx = 1; break;
				case 13: case 31: ui8_idx = 2; break;
				case 32: case 23: ui8_idx = 3; break;
				case 26: case 62: ui8_idx = 4; break;
				case 64: case 46: ui8_idx = 5; break;
				default: ui8_idx = 6; break;
				}
				if (ui8_idx < 6) {
					dst[ui8_idx] = q31_rotorposition_absolute;
					if (i8_dir > 0) ui8_setup_halls_found |= (1 << ui8_idx);
					else ui8_setup_halls_found_bwd |= (1 << ui8_idx);
				}
				ui8_hall_state_old = ui8_hall_state;
			}
		}
	}

	void autodetect() {
#ifdef SENSORLESS
		ui8_setup_state = 4; //no hall sensors to calibrate (saves ~600 bytes, the sensorless builds are close to full)
		return;
#endif
		SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
		ui8_setup_state = 1;
		ui8_setup_progress = 0;
		ui8_setup_halls_found = 0;
		ui8_setup_halls_found_bwd = 0;
		ui8_setup_hall_spread = 0;
		MS.hall_angle_detect_flag = 0; //set uq to contstant value in FOC.c for open loop control
		q31_rotorposition_absolute = 1 << 31;
		i16_hall_order = 1;//reset hall order
		MS.i_d_setpoint= 300; //set MS.id to appr. 2000mA
		MS.i_q_setpoint= 0;
		//	uint8_t zerocrossing = 0;
		//	q31_t diffangle = 0;
		HAL_Delay(5);
		// 3 electrical turns each way. Each hall edge is seen in both directions: the rotor lags the field
		// (friction, cogging) and the hall sensors switch late in the direction of travel, both with the opposite
		// sign the other way. The average is the true edge, the difference shows how good the measurement is
		// (like the Fardriver autodetect). Backwards (as set by 'reverse') first: a geared hub's freewheel lets the
		// motor turn freely without the wheel, and the apps ask "first backwards, then forwards?" to check the
		// direction setting. fwd/bwd below are the electrical directions (+1 / -1).
		{
			int8_t i8_first = -i8_direction;
			autodetect_sweep(i8_first, i8_first > 0 ? q31_hall_fwd : q31_hall_bwd, 0);
			if (i8_first > 0) i8_direction_fwd = i8_recent_rotor_direction;
			autodetect_sweep(-i8_first, i8_first > 0 ? q31_hall_bwd : q31_hall_fwd, 50);
			if (i8_first < 0) i8_direction_fwd = i8_recent_rotor_direction;
		}

		CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); //Disable PWM if motor is not turning
		TIM1->CCR1 = 1023; //set initial PWM values
		TIM1->CCR2 = 1023;
		TIM1->CCR3 = 1023;
		MS.hall_angle_detect_flag=1;
		MS.i_d = 0;
		MS.i_q = 0;
		MS.u_d=0;
		MS.u_q=0;
		MS.i_d_setpoint= 0;
		uint32_tics_filtered=1000000;

		if(ui8_setup_halls_found != 0x3F || ui8_setup_halls_found_bwd != 0x3F){ //not all 6 hall positions seen in both directions: wiring/hall problem, don't save garbage
			ui8_setup_state = 4;
			return;
		}
		{
			// average of both directions (unsigned arithmetic: the q31 angles wrap around at 360 deg)
			uint32_t ui32_spread_max = 0;
			for (i = 0; i < 6; i++) {
				int32_t i32_diff = (int32_t) ((uint32_t) q31_hall_bwd[i] - (uint32_t) q31_hall_fwd[i]);
				uint32_t ui32_abs = (i32_diff < 0) ? (uint32_t) (-(i32_diff + 1)) + 1 : (uint32_t) i32_diff;
				if (ui32_abs > ui32_spread_max) ui32_spread_max = ui32_abs;
				q31_hall_avg[i] = (q31_t) ((uint32_t) q31_hall_fwd[i] + (uint32_t) (i32_diff / 2));
			}
			ui8_setup_hall_spread = (uint8_t) ((((uint64_t) ui32_spread_max) * 360) >> 32); // degrees, max of the 6 edges
			Hall_45 = q31_hall_avg[0];
			Hall_51 = q31_hall_avg[1];
			Hall_13 = q31_hall_avg[2];
			Hall_32 = q31_hall_avg[3];
			Hall_26 = q31_hall_avg[4];
			Hall_64 = q31_hall_avg[5];
		}

		HAL_FLASH_Unlock();

		if (i8_direction_fwd == 1) {
			EE_WriteVariable(EEPROM_POS_HALL_ORDER, 1);
			i16_hall_order = 1;
		} else {
			EE_WriteVariable(EEPROM_POS_HALL_ORDER, -1);
			i16_hall_order = -1;
		}
		EE_WriteVariable(EEPROM_POS_HALL_45, Hall_45 >> 16);
		EE_WriteVariable(EEPROM_POS_HALL_51, Hall_51 >> 16);
		EE_WriteVariable(EEPROM_POS_HALL_13, Hall_13 >> 16);
		EE_WriteVariable(EEPROM_POS_HALL_32, Hall_32 >> 16);
		EE_WriteVariable(EEPROM_POS_HALL_26, Hall_26 >> 16);
		EE_WriteVariable(EEPROM_POS_HALL_64, Hall_64 >> 16);

		HAL_FLASH_Lock();

		MS.hall_angle_detect_flag = 1;

		HAL_Delay(5);
		ui8_hall_calibrated = 1;
		ui8_setup_state = 2;
		ui8_setup_progress = 0;
		ui8_KV_detect_flag = 30;


	}

	// ionrise startup melody, played by the motor windings: an audio frequency voltage along one phase axis gives
	// no torque on average, the wheel doesn't move and no hall calibration is needed. Every note uses the next
	// phase axis (A, B, C) and checks the phase currents: the axis phase carries the current, the other two half
	// of it each. An open phase wire or a dead MOSFET is found before the ride: ui8_phase_fault, motor blocked.
	// The voltage is adjusted so the peak current is melody_pct of the phase current (the coil impedance rises
	// with the frequency). Brake, wheel movement or an overcurrent trip end it.
	static const uint8_t ui8_melody[][3] = { // note Hz/16 (0 = pause), length in 5 ms, volume in 1/8
		{49, 20, 8}, {0, 6, 0}, {65, 20, 8}, {0, 6, 0}, {82, 68, 8}, {0, 14, 0}, {98, 14, 5}, {0, 5, 0}, {131, 60, 6}};

	static void play_melody(void) {
		static const int16_t i16_axis[3][2] = {{1024, 0}, {-512, 887}, {-512, -887}};
		int32_t i32_target = (RS.ph_current_max * RS.melody_pct) / 100;
		int32_t i32_amp = 150; // voltage (of _U_MAX), found by the loop below
		uint32_t ui32_halls = ui32_hall_events;
		uint16_t ui16_f_old = 0;
		uint8_t n, k, ui8_axis = 0, ui8_faults = 0;
		if (!RS.melody_pct) return;
		if (i32_target < 40) i32_target = 40; // ~1.5 A: enough for the phase test
		i16_tone_amp = 0;
		ui8_tone_on = 1; // also in the pauses (amplitude 0): the FOC must not run
		SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
		for (n = 0; n < sizeof(ui8_melody) / 3; n++) {
			uint16_t ui16_f = ui8_melody[n][0] * 16, ui16_len = ui8_melody[n][1] * 5, t;
			uint16_t ui16_max[3] = {0, 0, 0};
			int32_t i32_tgt = (i32_target * ui8_melody[n][2]) >> 3;
			if (ui16_f) {
				if (ui16_f_old) i32_amp = (i32_amp * ui16_f) / ui16_f_old; // inductive: voltage ~ frequency
				ui16_f_old = ui16_f;
				ui32_tone_step = ui16_f * 272196UL; // 2^32 / 15779 Hz FOC rate
				i16_tone_ca = i16_axis[ui8_axis][0];
				i16_tone_sa = i16_axis[ui8_axis][1];
			}
			for (t = 0; t < ui16_len; t++) {
				int32_t i32_env = 256; // 5 ms up, 15 ms down: no clicks
				if (t < 5) i32_env = t * 51;
				if (ui16_len - t < 15) i32_env = (ui16_len - t) * 17;
				if (i32_amp > 1800) i32_amp = 1800;
				i16_tone_amp = (i32_amp * i32_env * ui8_melody[n][2]) >> 11;
				HAL_Delay(1);
				HAL_IWDG_Refresh(&hiwdg);
				if (ui16_f && (t & 3) == 3 && i32_env == 256) {
					uint16_t ui16_p = 0;
					for (k = 0; k < 3; k++) {
						if (ui16_tone_peak[k] > ui16_p) ui16_p = ui16_tone_peak[k];
						if (ui16_tone_peak[k] > ui16_max[k]) ui16_max[k] = ui16_tone_peak[k];
						ui16_tone_peak[k] = 0;
					}
					if (ui16_p < i32_tgt) i32_amp += (i32_amp >> 3) + 1;
					else i32_amp -= i32_amp >> 3;
				}
				if (!HAL_GPIO_ReadPin(Brake_GPIO_Port, Brake_Pin) || ui32_hall_events != ui32_halls || ui8_overcurrent) {
					ui8_faults = 0;
					goto end;
				}
			}
			if (ui16_f) {
				// phase test: largest ~ the axis phase, smallest ~ half of it. Open phase: nothing flows (axis on
				// that phase) or one of the others stays at 0.
				uint16_t ui16_hi = 0, ui16_lo = 65535;
				for (k = 0; k < 3; k++) {
					if (ui16_max[k] > ui16_hi) ui16_hi = ui16_max[k];
					if (ui16_max[k] < ui16_lo) ui16_lo = ui16_max[k];
				}
				if (ui16_hi < 20 || ui16_lo * 4 < ui16_hi) ui8_faults |= 1 << ui8_axis;
				if (++ui8_axis > 2) ui8_axis = 0;
			}
		}
	end:
		ui8_tone_on = 0;
		CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
		HAL_Delay(2); // the FOC interrupt resets the PWM pattern and the controllers while the PWM is off
		ui8_phase_fault = ui8_faults;
		if (ui8_faults) {
			ui8_overcurrent = 1; // motor off until restart
			ui8_oc_trips = 255;
		}
	}

#if defined(SENSORLESS) && defined(PARAM_ID)
	// ---- motor parameter identification: a temporary firmware of the setup apps (sensorless motors) ----
	// The app flashes this build, writes ui8_setup_cmd = 2, reads the results, saves them in the settings and
	// installs the normal firmware again. Wheel lifted, the motor turns for a few seconds in step 3.
	// 1. resistance: d current at a fixed angle (the rotor aligns, does not turn) at two levels, R = dU / dI
	//    (the dead time voltage error is the same at both levels and cancels)
	// 2. inductance: 1 kHz sine voltage on the d axis (the tone generator of the startup melody) at two current
	//    levels, |Z| = dU / dI, L = sqrt(Z^2 - R^2) / w
	// 3. flux linkage: the field is turned open loop (I/f) up to 20 electrical rev/s, the rotor follows; then the
	//    current is controlled to 0 and the voltage is the back-EMF alone: flux = U / w
	// Phase volts per voltage unit = Vbat / 3511 (svpwm), amps per current unit = CAL_I mA.
	volatile uint8_t ui8_pid_state = 0;      // 0 idle, 1 resistance, 2 inductance, 3 flux, 4 done, 5 failed
	volatile uint8_t ui8_pid_progress = 0;   // %
	volatile int32_t i32_pid_r_mohm = 0;     // results: mOhm, uH, uWb
	volatile int32_t i32_pid_l_uh = 0;
	volatile int32_t i32_pid_flux_uwb = 0;
	volatile int32_t i32_pid_obs_r = 0;      // the same in the observer units of the settings (obs_resistance ...)
	volatile int32_t i32_pid_obs_l = 0;
	volatile int32_t i32_pid_obs_flux = 0;
	volatile uint8_t ui8_pid_rotate = 0;     // FOC interrupt: turn q31_teta_obs by i32_pid_step per cycle
	volatile int32_t i32_pid_step = 0;

	static uint32_t pid_isqrt(uint32_t x) {
		uint32_t r = 0, b = 1UL << 30;
		while (b > x) b >>= 2;
		while (b) {
			if (x >= r + b) { x -= r + b; r = (r >> 1) + b; }
			else r >>= 1;
			b >>= 2;
		}
		return r;
	}

	static uint8_t pid_wait(uint16_t ms) { // 1 = aborted (brake, over current)
		while (ms--) {
			HAL_Delay(1);
			HAL_IWDG_Refresh(&hiwdg);
			if (ui8_overcurrent || !HAL_GPIO_ReadPin(Brake_GPIO_Port, Brake_Pin)) return 1;
		}
		return 0;
	}

	// mean u_d and i_d over ms milliseconds
	static uint8_t pid_mean_d(uint16_t ms, int32_t *u, int32_t *i) {
		int32_t su = 0, si = 0;
		uint16_t k;
		for (k = 0; k < ms; k++) {
			if (pid_wait(1)) return 1;
			su += MS.u_d;
			si += MS.i_d;
		}
		*u = su / ms;
		*i = si / ms;
		return 0;
	}

	// tone on the d (= alpha at angle 0) axis, amplitude regulated to the peak current target; mean amplitude and
	// phase A peak current of the last 100 ms
	static uint8_t pid_tone(int32_t i32_target, int32_t *a, int32_t *p) {
		int32_t i32_amp = 60, sa = 0, sp = 0, n = 0;
		uint16_t t;
		for (t = 0; t < 500; t++) {
			i16_tone_amp = i32_amp;
			if (pid_wait(1)) return 1;
			if ((t & 3) == 3) {
				int32_t i32_peak = ui16_tone_peak[0];
				ui16_tone_peak[0] = ui16_tone_peak[1] = ui16_tone_peak[2] = 0;
				if (t >= 400) { sa += i32_amp; sp += i32_peak; n++; }
				else if (i32_peak < i32_target) i32_amp += (i32_amp >> 4) + 1;
				else i32_amp -= i32_amp >> 4;
				if (i32_amp > 1800) i32_amp = 1800;
			}
		}
		*a = sa / n;
		*p = sp / n;
		return 0;
	}

	static void pid_measure(void) {
		int32_t i32_vbat = adcData[0] * CAL_V;          // mV
		int32_t i32_cal = CAL_I >> 8;                    // mA per current unit
		int32_t i32_div = (3511 * i32_cal) / 100;        // x*10 / i32_div: (units x mV / units) -> mOhm
		int32_t i1 = RS.ph_current_max >> 2, i2 = RS.ph_current_max >> 1;
		int32_t u1, a1, u2, a2, d;
		uint32_t z, x;
		uint16_t k;
		if (i1 < 40) i1 = 40;                            // at least ~1.5 / 3 A
		if (i2 < 80) i2 = 80;
		ui8_pid_state = 1; ui8_pid_progress = 0;

		// 1. resistance
		q31_teta_obs = 0;
		MS.i_q_setpoint = 0;
		MS.i_d_setpoint = i1;
		SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
		if (pid_wait(600) || pid_mean_d(200, &u1, &a1)) goto fail;
		MS.i_d_setpoint = i2;
		if (pid_wait(300) || pid_mean_d(200, &u2, &a2)) goto fail;
		d = a2 - a1; if (d < 0) d = -d;
		if (d < 10) goto fail;                           // the current did not follow: motor not connected?
		u2 -= u1; if (u2 < 0) u2 = -u2;
		i32_pid_r_mohm = ((u2 * i32_vbat / d) * 10) / i32_div;
		MS.i_d_setpoint = 0;
		ui8_pid_progress = 30;

		// 2. inductance
		ui8_pid_state = 2;
		CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
		pid_wait(50);
		i16_tone_ca = 1024; i16_tone_sa = 0;
		ui32_tone_step = 1000UL * 272196UL;               // 1 kHz
		i16_tone_amp = 0;
		ui8_tone_on = 1;
		SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
		if (pid_tone(i1, &u1, &a1) || pid_tone(i2, &u2, &a2)) goto fail;
		ui8_tone_on = 0;
		CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
		d = a2 - a1;
		if (d < 10 || u2 <= u1) goto fail;
		z = (uint32_t) ((((u2 - u1) * i32_vbat / d) * 10) / i32_div);        // |Z| in mOhm
		x = (z > (uint32_t) i32_pid_r_mohm) ? pid_isqrt(z * z - (uint32_t) (i32_pid_r_mohm * i32_pid_r_mohm)) : 0;
		i32_pid_l_uh = (int32_t) ((x * 1000UL) / 6283UL);                    // X / (2 pi 1 kHz)
		ui8_pid_progress = 50;
		pid_wait(100);

		// 3. flux linkage
		ui8_pid_state = 3;
		q31_teta_obs = 0;
		i32_pid_step = 0;
		ui8_pid_rotate = 1;
		MS.i_d_setpoint = i2;
		SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
		if (pid_wait(400)) goto fail;                    // align
		for (k = 0; k < 3000; k++) {                     // ramp to 20 erev/s in 3 s
			i32_pid_step = (int32_t) ((5443000UL / 3000UL) * k);
			if (pid_wait(1)) goto fail;
			if ((k & 63) == 0) ui8_pid_progress = 50 + (uint8_t) ((k * 40UL) / 3000UL);
		}
		i32_pid_step = 5443000L;                         // 20 * 2^32 / 15779
		if (pid_wait(500)) goto fail;
		MS.i_d_setpoint = 0;                             // current 0: the voltage is the back-EMF
		if (pid_wait(20)) goto fail;                     // short: without current the rotor slows down
		{
			int32_t su = 0;
			for (k = 0; k < 20; k++) {
				if (pid_wait(1)) goto fail;
				su += (int32_t) pid_isqrt((uint32_t) (MS.u_d * MS.u_d + MS.u_q * MS.u_q));
			}
			u1 = su / 20;
		}
		ui8_pid_rotate = 0;
		CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
		u1 = u1 * i32_vbat / 3511;                       // back-EMF peak in mV
		if (u1 < 100) goto fail;                         // the rotor did not follow
		{
			int32_t f_x100 = (int32_t) (((5443000UL >> 8) * 15779UL) / 167772UL);  // frequency x100 (step*f_FOC/2^32)
			i32_pid_flux_uwb = (u1 * 15915L) / f_x100;   // U / (2 pi f)
		}
		// observer units (same conversion as tools/motor_sim.py: L*2^16/1.5, R*2^9/1.5, flux*256000)
		i32_pid_obs_r = (i32_pid_r_mohm * 512 + 750) / 1500;
		i32_pid_obs_l = (i32_pid_l_uh * 437 + 5000) / 10000;
		i32_pid_obs_flux = (i32_pid_flux_uwb * 256 + 500) / 1000;
		ui8_pid_progress = 100;
		ui8_pid_state = 4;
		goto end;
	fail:
		ui8_pid_state = 5;
	end:
		ui8_tone_on = 0;
		ui8_pid_rotate = 0;
		i32_pid_step = 0;
		MS.i_d_setpoint = 0;
		CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
		HAL_Delay(2);
	}
#endif

	void get_standstill_position(){
		HAL_Delay(100);
		HAL_TIM_IC_CaptureCallback(&htim2); //read in initial rotor position

		switch (ui8_hall_state) {
		//6 cases for forward direction
		case 2:
			q31_rotorposition_hall = Hall_32;
			break;
		case 6:
			q31_rotorposition_hall = Hall_26;
			break;
		case 4:
			q31_rotorposition_hall = Hall_64;
			break;
		case 5:
			q31_rotorposition_hall = Hall_45;
			break;
		case 1:
			q31_rotorposition_hall = Hall_51;

			break;
		case 3:
			q31_rotorposition_hall = Hall_13;
			break;

		}

		q31_rotorposition_absolute = q31_rotorposition_hall;
	}

	int32_t speed_to_tics (uint8_t speed){
		return RS.wheel_circumference*5*3600/(6*RS.gear_ratio*speed*10);
	}

	int8_t tics_to_speed (uint32_t tics){
		return RS.wheel_circumference*5*3600/(6*RS.gear_ratio*tics*10);
	}

	int16_t internal_tics_to_speedx100 (uint32_t tics){
		return RS.wheel_circumference*50*3600/(6*RS.gear_ratio*tics);
	}

	int16_t external_tics_to_speedx100 (uint32_t tics){
		return RS.wheel_circumference*8*360/(RS.pulses_per_revolution*tics);
	}

	void runPIcontrol(){

		// battery current from the active power P ~ u_q*i_q - u_d*i_d (firmware frame).
		// 1 voltage unit * 1 current unit = 1/2341 A battery current (u = Vbat/3511 phase volts, P = 1.5*u*i).
		// The old estimate used u_abs (includes the reactive voltage) and >>11: 15..32 % too high in the simulation,
		// so the battery current limit cut in at about 11-12 A for a 15 A setting.
		q31_t q31_power = MS.u_q*MS.i_q - MS.u_d*MS.i_d;
		q31_t_Battery_Current_accumulated -= q31_t_Battery_Current_accumulated>>8;
		q31_t_Battery_Current_accumulated += ((q31_power*7)>>14)*(uint16_t)(CAL_I>>8);
		MS.Battery_Current = (q31_t_Battery_Current_accumulated>>8)*i8_direction*i8_reverse_flag; //Battery current in mA

		// phase current setpoint in the controller frame
		int32_t i32_iq_setpoint = i8_direction*i8_reverse_flag*MS.i_q_setpoint;
#ifdef SENSORLESS
		i32_iq_setpoint = sensorless_q_setpoint(i32_iq_setpoint); //0 during the open loop start, faded in at the hand-over
#endif
		// current ramp: the setpoint rises at most by ph_current_max per current_ramp_ms (smooth take-off), it drops
		// immediately (releasing the throttle / braking acts at once). 0 ms = no ramp.
		if (i32_ramp_step16 && !brake_flag) {
			int32_t i32_target16 = i32_iq_setpoint << 16;
			if (i32_target16 >= 0) {
				if (i32_iq_ramped16 < 0) i32_iq_ramped16 = 0;
				if (i32_iq_ramped16 < i32_target16) { i32_iq_ramped16 += i32_ramp_step16; if (i32_iq_ramped16 > i32_target16) i32_iq_ramped16 = i32_target16; }
				else i32_iq_ramped16 = i32_target16;
			}
			else {
				if (i32_iq_ramped16 > 0) i32_iq_ramped16 = 0;
				if (i32_iq_ramped16 > i32_target16) { i32_iq_ramped16 -= i32_ramp_step16; if (i32_iq_ramped16 < i32_target16) i32_iq_ramped16 = i32_target16; }
				else i32_iq_ramped16 = i32_target16;
			}
			i32_iq_setpoint = i32_iq_ramped16 >> 16;
		}
		else i32_iq_ramped16 = i32_iq_setpoint << 16;
		// battery / regen current limit as a clamp of the phase current setpoint: i_q_max = I_bat_max / (k*u_q).
		// The old code switched the current PI to the (slowly filtered) battery current: it oscillated between
		// 0 and 2x the limit in the simulation. u_q is low pass filtered (16 ms), unfiltered it oscillates in regen.
		static q31_t q31_uq_abs_filtered = 0;
		q31_uq_abs_filtered -= q31_uq_abs_filtered>>8;
		q31_uq_abs_filtered += (MS.u_q >= 0) ? MS.u_q : -MS.u_q;
		int32_t i32_uq_abs = q31_uq_abs_filtered>>8;
		ui8_BC_limit_flag=0;
		if(i32_uq_abs > 20){
			//regen limit only while braking (like the old code): otherwise forward torque against a backwards
			//rolling wheel (negative power) would be clamped to the regen limit, 0 A by default = no torque
			int32_t i32_bat_limit = brake_flag ? RS.regen_current_max : MP.battery_current_max;
			if(i32_bat_limit > 131000) i32_bat_limit = 131000; //keeps *16384 inside int32: 32 bit hardware division in the ISR
			if(i32_bat_limit < 0) i32_bat_limit = 0;
			int32_t i32_iq_limit = (i32_bat_limit * 16384) / (7 * (int32_t)(CAL_I>>8) * i32_uq_abs);
			if(i32_iq_setpoint > i32_iq_limit){ i32_iq_setpoint = i32_iq_limit; ui8_BC_limit_flag=1; }
			if(i32_iq_setpoint < -i32_iq_limit){ i32_iq_setpoint = -i32_iq_limit; ui8_BC_limit_flag=1; }
		}

		// Field weakening (idea from VESC: negative Id above a duty cycle threshold, ramped, limited). Here it is closed
		// loop: the voltage the two current controllers ask for (before the voltage limit) is kept at fw_voltage_pct of
		// the maximum. Asking for more -> more negative Id (lower back-EMF), asking for less -> Id back to 0, both at
		// most fw_current_max per fw_ramp_ms. So only as much weakening as the current control needs, whatever the
		// load, while motoring and while braking (above the base speed the regen current stays controlled because
		// the weakening stays as long as the back-EMF needs it). VESC maps the duty cycle to an Id open loop.
		// Limits: Id >= -fw_current_max; Id and Iq together within ph_current_max (Iq is reduced, the phase current
		// does not grow); while motoring the battery current may not exceed fw_battery_current_max because of the
		// weakening: above it the weakening goes back (the speed drops until the battery current fits). Clamping Iq
		// instead switched the limit on and off with the weakening and chattered in the simulation.
		// Off (Id 0) during the hall autodetect, sensorless start up and whenever another Id setpoint is active.
		if (i32_fw_step16 && MS.i_d_setpoint == 0 && MS.hall_angle_detect_flag && !ui8_KV_detect_flag
#ifdef SENSORLESS
				&& ui8_sensorless_running
#endif
				) {
			int32_t i32_e = (i32_u_demand16 >> 4) - i32_fw_u_start; //>0: voltage nearly used up
			if (i32_e > 100) i32_e = 100;                              //full ramp speed at 5 % of _U_MAX off
			if (i32_e < -100) i32_e = -100;
			//battery current (mA, motoring > 0 in both directions; MS.Battery_Current carries the direction sign)
			if (!brake_flag && (q31_t_Battery_Current_accumulated >> 8) > (RS.fw_battery_current_max ? RS.fw_battery_current_max : MP.battery_current_max)) i32_e = -100;
			i32_fw_id16 -= (i32_fw_step16 * i32_e) / 100;
			if (i32_fw_id16 > 0) i32_fw_id16 = 0;
			if (i32_fw_id16 < -(RS.fw_current_max << 16)) i32_fw_id16 = -(RS.fw_current_max << 16);
		}
		else i32_fw_id16 = 0;
		i32_fw_id = i32_fw_id16 >> 16;
		if (i32_fw_id) {
			// Iq limit on the phase current circle: Iq^2 + Id^2 <= ph_current_max^2 (integer square root)
			int32_t i32_r2 = RS.ph_current_max * RS.ph_current_max - i32_fw_id * i32_fw_id;
			int32_t i32_iq_max = 0;
			if (i32_r2 > 0) {
				uint32_t ui32_bit = 1UL << 30, ui32_x = (uint32_t)i32_r2, ui32_root = 0;
				while (ui32_bit > ui32_x) ui32_bit >>= 2;
				while (ui32_bit) {
					if (ui32_x >= ui32_root + ui32_bit) { ui32_x -= ui32_root + ui32_bit; ui32_root = (ui32_root >> 1) + ui32_bit; }
					else ui32_root >>= 1;
					ui32_bit >>= 2;
				}
				i32_iq_max = (int32_t)ui32_root;
			}
			if (i32_iq_setpoint > i32_iq_max) i32_iq_setpoint = i32_iq_max;
			if (i32_iq_setpoint < -i32_iq_max) i32_iq_setpoint = -i32_iq_max;
		}

		//control iq
		PI_iq.recent_value = MS.i_q;
		PI_iq.setpoint = i32_iq_setpoint;

		q31_u_q_temp =  PI_control(&PI_iq);

		//Control id
		PI_id.recent_value = MS.i_d;
		PI_id.setpoint = MS.i_d_setpoint + i32_fw_id;
		q31_u_d_temp = -PI_control(&PI_id); //control direct current to zero (or the field weakening current)


		//limit voltage in rotating frame, refer chapter 4.10.1 of UM1052
		//MS.u_abs = (q31_t)hypot((double)q31_u_d_temp, (double)q31_u_q_temp); //absolute value of U in static frame
		arm_sqrt_q31((q31_u_d_temp*q31_u_d_temp+q31_u_q_temp*q31_u_q_temp)<<1,&MS.u_abs);
		MS.u_abs = (MS.u_abs>>16)+1;
		i32_u_demand16 += MS.u_abs - (i32_u_demand16 >> 4); //voltage asked for, before the limit (~1 ms low pass)


		if (MS.u_abs > _U_MAX){
			MS.u_q = (q31_u_q_temp*_U_MAX)/MS.u_abs; //division!
			MS.u_d = (q31_u_d_temp*_U_MAX)/MS.u_abs; //division!
			MS.u_abs = _U_MAX;
		}
		else{
			MS.u_q=q31_u_q_temp;
			MS.u_d=q31_u_d_temp;
		}

		PI_flag=0;
	}

	q31_t speed_PLL (q31_t ist, q31_t soll, uint8_t speedadapt)
	{
		q31_t q31_p;
		static q31_t q31_d_i = 0;
		static q31_t q31_d_dc = 0;
		// temp6 = soll-ist;
		// temp5 = speedadapt;
		if (speedadapt > P_FACTOR_PLL) speedadapt = P_FACTOR_PLL; //negative shift is undefined
		if (speedadapt > I_FACTOR_PLL) speedadapt = I_FACTOR_PLL;
		q31_p=(soll - ist)>>(P_FACTOR_PLL-speedadapt);   				//7 for Shengyi middrive, 10 for BionX IGH3
		q31_d_i+=(soll - ist)>>(I_FACTOR_PLL-speedadapt);				//11 for Shengyi middrive, 10 for BionX IGH3

		//clamp i part to twice the theoretical value from hall interrupts
		if (q31_d_i>((deg_30>>18)*500/ui16_timertics)<<16) q31_d_i = ((deg_30>>18)*500/ui16_timertics)<<16;
		if (q31_d_i<-((deg_30>>18)*500/ui16_timertics)<<16) q31_d_i =- ((deg_30>>18)*500/ui16_timertics)<<16;


		if (!ist&&!soll)q31_d_i=0;

		q31_d_dc=q31_p+q31_d_i;
		return (q31_d_dc);
	}

void Set_Hall_Logic(void){
	HAL_FLASH_Unlock();
	EE_WriteVariable(EEPROM_HALL_60, i16_60deg_Hall_flag);
	HAL_FLASH_Lock();
	}

void Set_Hall_Angle120(void){

	switch (ui8_hall_case) //12 cases for each transition from one stage to the next. 6x forward, 6x reverse
		{
		//6 cases for forward direction
		//6 cases for forward direction
		case 64:
			q31_rotorposition_hall = Hall_64;

			i8_recent_rotor_direction = -i16_hall_order;
			uint16_full_rotation_counter = 0;
			break;
		case 45:
			q31_rotorposition_hall = Hall_45;

			i8_recent_rotor_direction = -i16_hall_order;
			break;
		case 51:
			q31_rotorposition_hall = Hall_51;

			i8_recent_rotor_direction = -i16_hall_order;
			break;
		case 13:
			q31_rotorposition_hall = Hall_13;

			i8_recent_rotor_direction = -i16_hall_order;
			uint16_half_rotation_counter = 0;
			break;
		case 32:
			q31_rotorposition_hall = Hall_32;

			i8_recent_rotor_direction = -i16_hall_order;
			break;
		case 26:
			q31_rotorposition_hall = Hall_26;

			i8_recent_rotor_direction = -i16_hall_order;
			break;

			//6 cases for reverse direction
		case 46:
			q31_rotorposition_hall = Hall_64;

			i8_recent_rotor_direction = i16_hall_order;
			break;
		case 62:
			q31_rotorposition_hall = Hall_26;

			i8_recent_rotor_direction = i16_hall_order;
			break;
		case 23:
			q31_rotorposition_hall = Hall_32;

			i8_recent_rotor_direction = i16_hall_order;
			uint16_half_rotation_counter = 0;
			break;
		case 31:
			q31_rotorposition_hall = Hall_13;

			i8_recent_rotor_direction = i16_hall_order;
			break;
		case 15:
			q31_rotorposition_hall = Hall_51;

			i8_recent_rotor_direction = i16_hall_order;
			break;
		case 54:
			q31_rotorposition_hall = Hall_45;

			i8_recent_rotor_direction = i16_hall_order;
			uint16_full_rotation_counter = 0;
			break;

		} // end case

	}

void Set_Hall_Angle60(void){

	switch (ui8_hall_case) //12 cases for each transition from one stage to the next. 6x forward, 6x reverse
		{
		//6 cases for forward direction
		//6 cases for forward direction
		case 10:
			q31_rotorposition_hall = Hall_10;

			i8_recent_rotor_direction = -i16_hall_order;
			uint16_full_rotation_counter = 0;
			break;
		case 4:
			q31_rotorposition_hall = Hall_4;

			i8_recent_rotor_direction = -i16_hall_order;
			break;
		case 46:
			q31_rotorposition_hall = Hall_46;

			i8_recent_rotor_direction = -i16_hall_order;
			break;
		case 67:
			q31_rotorposition_hall = Hall_67;

			i8_recent_rotor_direction = -i16_hall_order;
			uint16_half_rotation_counter = 0;
			break;
		case 73:
			q31_rotorposition_hall = Hall_73;

			i8_recent_rotor_direction = -i16_hall_order;
			break;
		case 31:
			q31_rotorposition_hall = Hall_31;

			i8_recent_rotor_direction = -i16_hall_order;
			break;

			//6 cases for reverse direction
		case 01:
			q31_rotorposition_hall = Hall_10;

			i8_recent_rotor_direction = i16_hall_order;
			break;
		case 40:
			q31_rotorposition_hall = Hall_4;

			i8_recent_rotor_direction = i16_hall_order;
			break;
		case 64:
			q31_rotorposition_hall = Hall_46;

			i8_recent_rotor_direction = i16_hall_order;
			uint16_half_rotation_counter = 0;
			break;
		case 76:
			q31_rotorposition_hall = Hall_67;

			i8_recent_rotor_direction = i16_hall_order;
			break;
		case 37:
			q31_rotorposition_hall = Hall_73;

			i8_recent_rotor_direction = i16_hall_order;
			break;
		case 13:
			q31_rotorposition_hall = Hall_31;

			i8_recent_rotor_direction = i16_hall_order;
			uint16_full_rotation_counter = 0;
			break;

		} // end case

	}




	int16_t T_NTC(uint16_t ADC) // ADC 12 Bit, 10k Pullup, Rückgabewert in °C

	{
		uint16_t Ux1000 = 3300;
		uint16_t U2x1000 = ADC*Ux1000/4095;
		uint32_t R1 = RS.ntc_pullup_ohm;
		uint32_t R = U2x1000*R1/(Ux1000-U2x1000);
		// 	printf("R= %d\r\n",R);
		//  printf("u2= %d\r\n",U2x1000);
		if(R >> 19) return -44;
		uint16_t n = 0;
		while(R >> n > 1) n++;
		R <<= 13;
		for(n <<= 6; R >> (n >> 6) >> 13; n++) R -= (R >> 10)*11; // Annäherung 1-11/1024 für 2^(-1/64)
		int16_t T6 = 2160580/(n+357)-1639; // Berechnung für 10 kOhm-NTC (bei 25 °C) mit beta=3900 K
		return (T6 > 0 ? T6+3 : T6-2)/6; // Rundung

	}
	void init_watchdog(void)
	{

		if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
		{
			ui8_watchdog_reset=1;

			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			// do not continue here if reset from watchdog
			//while(1){}
			//__HAL_RCC_CLEAR_RESET_FLAGS();
		}
		else
		{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

		}
		__HAL_RCC_CLEAR_RESET_FLAGS();
		// start independent watchdog
		MX_IWDG_Init();


	}

	/* IWDG init function */
	void MX_IWDG_Init(void)
	{
		// RM0008 - Table 96 IWDG timout period in seconds:
		// (IWDG_PRESCALER) * (Period + 1) / f_LSI
		// datasheet STM32F103x4 -> f_LSI = 40'000 Hz ?!
		//
		// 32 * 1000 / 32000 = 1s
		hiwdg.Instance = IWDG;
		hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
		hiwdg.Init.Reload = 1000;
		// start the watchdog timer
		if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

	}
	/* USER CODE END 4 */

	/**
	 * @brief  This function is executed in case of error occurrence.
	 * @param  file: The file name as string.
	 * @param  line: The line in file as a number.
	 * @retval None
	 */
	void _Error_Handler(char *file, int line)
	{
		/* USER CODE BEGIN Error_Handler_Debug */
		/* User can add his own implementation to report the HAL error return state */
		while(1)
		{
			CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); // Disable PWM in case of an error.
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
	void assert_failed(uint8_t* file, uint32_t line)
	{
		/* USER CODE BEGIN 6 */
		/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
		/* USER CODE END 6 */
	}
#endif /* USE_FULL_ASSERT */

	/**
	 * @}
	 */

	/**
	 * @}
	 */

	/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
