/*
 * display_ionrise.c - IONRISE display protocol (iondisp), EBiCS Lishui port. See display_ionrise.h and
 * ionrise/IONRISE_PROTOCOL.md (section 9.1). The protocol logic is in ionrise/ionctrl.c; here are its hooks.
 *
 * Settings: the display owns the profile. It reads / writes a RAM copy of the 256 byte block (staging);
 * APPLY checks it with the boot check of settings.c, then LIVE copies it into RS, REBOOT hands it over in RAM
 * through a software reset. There is no PERSIST: no free flash page on the 32 KB chip.
 *
 * This file is part of EBiCS, GPL v3 or later.
 */

#include "config.h"
#include "main.h"
#include "stm32f1xx_hal.h"

#if (DISPLAY_TYPE == DISPLAY_TYPE_IONRISE)

#include <string.h>
#include "FOC.h"
#include "settings.h"
#include "ionctrl.h"
#include "display_ionrise.h"
// the shared protocol code (ionrise/, also used by iondisp and the KT firmware) is compiled here, so only
// the IONRISE build contains it (the other display builds keep the RAM of its buffers)
#include "ionproto.c"
#include "ionctrl.c"

extern UART_HandleTypeDef huart1;
extern MotorState_t MS;
extern MotorParams_t MP;
extern uint8_t brake_flag;
extern volatile uint8_t ui8_Push_Assist_flag;
extern uint32_t uint32_SPEEDx100_cumulated;
extern uint32_t uint32_PAS_counter;
extern uint32_t uint32_PAS;
extern uint32_t uint32_torque_cumulated;
extern uint16_t uint16_mapped_throttle;
extern volatile uint8_t ui8_motor_temp_sensor_ok;
extern volatile uint8_t ui8_overcurrent;
extern volatile uint8_t ui8_setup_cmd;
extern volatile uint8_t ui8_phase_fault;
extern volatile int32_t i32_fw_id;

ION_t ION;
ion_handoff_t ion_handoff __attribute__((section(".noinit")));

static uint8_t rx_ring[ION_RX_BUFF];
static uint8_t rx_tail;
static uint8_t staging[EBICS_SETTINGS_SIZE];

const ionc_info_t ionc_info = {
	ION_FW_EBICS_LISHUI, 0, 0x0100, ION_LAYOUT_EBICS_V1, 1, EBICS_SETTINGS_SIZE,
	ION_CAP_LIVE_APPLY | ION_CAP_HANDOFF | ION_CAP_HALL_DETECT
#ifdef SENSORLESS
	| ION_CAP_SENSORLESS
#endif
	, (uint8_t *) &RS, staging
};

void ionc_port_send(const uint8_t *frame, uint8_t len)
{
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *) frame, len);   // the frame buffer stays until the next request
}

uint32_t ionc_port_random(void)
{
	return SysTick->VAL ^ (TIM1->CNT << 16) ^ MS.i_q ^ (MS.Voltage << 8);
}

uint8_t ionc_port_source(void)
{
	return ui8_settings_source;   // 0 config.h, 1 flash block, 2 flash block rejected, 3 handed over by the display
}

uint8_t ionc_port_motor_stopped(void)
{
	return !READ_BIT(TIM1->BDTR, TIM_BDTR_MOE) && (uint32_SPEEDx100_cumulated >> SPEEDFILTER) < 100;
}

void ionc_port_control(const uint8_t *req)
{
	uint16_t limit = ion_get16(req + ION_TQ_SPEED_LIMIT);
	if (!req[ION_TQ_ASSIST]) ION.stop = 0;
	MS.assist_level = ION.stop ? 0 : req[ION_TQ_ASSIST];       // 0..255, same path as KM5S
	ION.walk = (req[ION_TQ_FLAGS] & ION_CF_WALK) ? 1 : 0;
	ui8_Push_Assist_flag = ION.walk;
	HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, (req[ION_TQ_FLAGS] & ION_CF_LIGHT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	if (!limit) limit = RS.speed_limit;
	MP.speedLimit = limit > 255 ? 255 : limit;
	ION.power_pct = req[ION_TQ_POWER_PCT] > 100 ? 100 : req[ION_TQ_POWER_PCT];
}

void ionc_port_link_lost(void)
{
	MS.assist_level = 0;
	ION.walk = 0;
	ui8_Push_Assist_flag = 0;
}

// the TELEMETRY answer as it is on the wire (little endian like the Cortex-M3: filled in place, no packing code)
typedef struct __attribute__((packed)) {
	uint16_t speed, voltage;
	int16_t batt_current, phase_current;
	int8_t motor_temp, ctrl_temp;
	uint8_t cadence, throttle;
	uint16_t torque;
	uint8_t assist, duty;
	uint16_t status, errors;
	uint8_t code, reserved;
} ion_tr_t;
typedef char ion_tr_size_check[(sizeof(ion_tr_t) == ION_TR_LEN) ? 1 : -1];

void ionc_port_telemetry(uint8_t *r)
{
	ion_tr_t *t = (ion_tr_t *) r;
	uint32_t v = uint32_SPEEDx100_cumulated >> SPEEDFILTER;
	uint32_t pas = uint32_PAS_counter < RS.pas_timeout;
	uint32_t mv = MS.Voltage * CAL_V;
	uint32_t st = 0, er = 0;
	t->speed = v > 65534 ? 65534 : v;
	t->voltage = mv / 10;
	t->batt_current = MS.Battery_Current / 100;
	t->phase_current = (MS.i_q * (int32_t) (CAL_I >> 8)) / 100;
	t->motor_temp = ui8_motor_temp_sensor_ok ? MS.Temperature : -128;
	t->ctrl_temp = MS.int_Temperature;
	v = pas ? 491400UL / (uint32_PAS * RS.pas_imp_per_turn + 1) : 0;  // 8190 ticks/s x 60 / (period x pulses)
	t->cadence = v > 254 ? 254 : v;
	t->throttle = RS_FLAG(SF_THROTTLE) ? (uint16_mapped_throttle * 100) / RS.ph_current_max : 255;
	t->torque = RS_FLAG(SF_TORQUE_SENSOR) ? (uint16_t) uint32_torque_cumulated : 0xFFFF;
	t->assist = MS.assist_level;
	t->duty = (MS.u_abs * 100) / _U_MAX;
	t->reserved = 0;

	if (READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)) st |= ION_ST_MOTOR_ON;
	if (brake_flag) st |= ION_ST_BRAKE;
	if (uint16_mapped_throttle) st |= ION_ST_THROTTLE;
	if (pas) st |= ION_ST_PAS;
	if (ION.walk) st |= ION_ST_WALK;
	if (i32_fw_id) st |= ION_ST_FIELD_WEAK;
	if (ionc_linked()) st |= ION_ST_LINKED;
	if (ui8_settings_source == 3) st |= ION_ST_DISPLAY_SET;
#ifdef SENSORLESS
	st |= ION_ST_SENSORLESS;
#endif
	if (ui8_setup_cmd) st |= ION_ST_BUSY;
	t->status = st;

	if (ui8_overcurrent) er |= ION_ER_OVERCURRENT;
	if (mv < (uint32_t) RS.battery_voltage_min) er |= ION_ER_UNDERVOLTAGE;
	if (ui8_motor_temp_sensor_ok && MS.Temperature >= RS.motor_temp_max) er |= ION_ER_MOTOR_TEMP;
	if (MS.int_Temperature > CONTROLLER_TEMPERATURE_MAX) er |= ION_ER_CTRL_TEMP;
	if (ui8_phase_fault) er |= ION_ER_PHASE;
	if (ui8_settings_source == 2) er |= ION_ER_SETTINGS;
	t->errors = er;
	t->code = ui8_overcurrent ? 0x21 : ui8_phase_fault ? 0x23 : 0;
}

uint8_t ionc_port_apply(uint8_t mode)
{
	if (mode & ION_APPLY_PERSIST) return ION_E_UNSUPPORTED;    // no free flash page
	if (!settings_check((const ebics_settings_t *) staging)) return ION_E_INVALID;
	if (mode & ION_APPLY_LIVE) {
		__disable_irq();
		memcpy(&RS, staging, EBICS_SETTINGS_SIZE);
		__enable_irq();
		ui8_settings_source = 3;
	}
	if (mode & ION_APPLY_REBOOT) {
		memcpy(ion_handoff.block, staging, EBICS_SETTINGS_SIZE);
		ion_handoff.crc = settings_crc32(staging, EBICS_SETTINGS_SIZE);
		ion_handoff.magic = ION_HANDOFF_MAGIC;
		ION.reset = 1;
	}
	return ION_OK;
}

uint8_t ionc_port_action(uint8_t action, const uint8_t *arg)
{
	(void) arg;
	switch (action) {
	case ION_ACT_REBOOT:
		if (!ionc_port_motor_stopped()) return ION_E_BUSY;
		ION.reset = 1;
		return ION_OK;
	case ION_ACT_HALL_DETECT:
		if (!ionc_port_motor_stopped() || ui8_setup_cmd) return ION_E_BUSY;
		ui8_setup_cmd = 1;                                  // hall + KV autodetect of the main loop
		return ION_OK;
	case ION_ACT_CLEAR_ERRORS:
		ui8_overcurrent = 0;
		return ION_OK;
	case ION_ACT_MOTOR_STOP:
		ION.stop = 1;
		MS.assist_level = 0;
		return ION_OK;
	}
	return ION_E_RANGE;
}

// at start up, and again after a UART error (noise on the line): then only the receiver restarts, the session
// with the display stays
void Ionrise_Init(void)
{
	static uint8_t started;
	if (!started) {
		started = 1;
		ION.power_pct = 100;
		ION.walk = ION.stop = ION.reset = 0;
		MS.assist_level = 0;
		ionc_init();
	}
	rx_tail = 0;
	if (HAL_UART_Receive_DMA(&huart1, rx_ring, ION_RX_BUFF) != HAL_OK) Error_Handler();
}

void Ionrise_Service(void)
{
	uint8_t head = (uint8_t) (ION_RX_BUFF - DMA1_Channel5->CNDTR) & (ION_RX_BUFF - 1);
	while (rx_tail != head) {
		ionc_rx_byte(rx_ring[rx_tail]);
		rx_tail = (rx_tail + 1) & (ION_RX_BUFF - 1);
		ionc_service((uint16_t) HAL_GetTick());              // a complete frame is answered at once
	}
	ionc_service((uint16_t) HAL_GetTick());                  // link supervision
	if (ION.reset && huart1.gState == HAL_UART_STATE_READY) NVIC_SystemReset();   // the answer is out
}

#endif
