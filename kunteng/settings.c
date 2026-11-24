/*
 * Runtime settings for the KT firmware: read the settings block from the data EEPROM, fall back to the
 * compile time defaults of main.h. See settings.h for the block layout.
 *
 * This file is part of the KT firmware, GPL v3.
 */

#include <stdint.h>
#include "stm8s.h"
#include "main.h"
#include "settings.h"
#include "display.h"

struct_kt_settings kt_settings;

#ifndef PHASE_CURRENT_FACTOR_X10
#define PHASE_CURRENT_FACTOR_X10 25 // phase current may be 2.5 x the battery current limit
#endif

// the block is read from the data EEPROM, or from a RAM copy when the IONRISE display checks its staging block
static const volatile uint8_t *p_settings_src = (const volatile uint8_t *) SETTINGS_BLOCK_ADDRESS;

static uint8_t read_eeprom (uint8_t ui8_offset)
{
  return p_settings_src[ui8_offset];
}

static void load_defaults (void)
{
  kt_settings.ui8_cells = BATTERY_LI_ION_CELLS_NUMBER;
#if defined (MOTOR_CURRENT_MAX_A)
  kt_settings.ui8_motor_current_a = MOTOR_CURRENT_MAX_A;
#else
  kt_settings.ui8_motor_current_a = ADC_MOTOR_CURRENT_MAX / 2;
#endif
  kt_settings.ui8_regen_current_a = ADC_MOTOR_REGEN_CURRENT_MAX / 2;
  kt_settings.ui8_throttle_type = EBIKE_THROTTLE_TYPE;
  kt_settings.ui8_flags = 0;
#if defined (MOTOR_TYPE_DIRECT_DRIVE)
  kt_settings.ui8_flags |= KT_FLAG_DIRECT_DRIVE;
#endif
#if (PAS_DIRECTION == PAS_DIRECTION_LEFT)
  kt_settings.ui8_flags |= KT_FLAG_PAS_LEFT;
#endif
#if defined (BATTERY_OVER_VOLTAGE_PROTECTION)
  kt_settings.ui8_flags |= KT_FLAG_OVER_VOLTAGE_PROTECTION;
#endif
#if defined (DISABLE_PHASE_CURRENT_LIMIT)
  kt_settings.ui8_flags |= KT_FLAG_NO_PHASE_CURRENT_LIMIT;
#endif
#if defined (DISABLE_HALL_LEARNING)
  kt_settings.ui8_flags |= KT_FLAG_NO_HALL_LEARNING;
#endif
  kt_settings.ui8_pas_magnets = PAS_NUMBER_MAGNETS;
  kt_settings.ui8_pas_max_cadence = PAS_MAX_CADENCE_RPM;
  kt_settings.ui8_assist_percent[0] = (uint8_t) (PAS_ASSIST_LEVEL_1 * 100.0 + 0.5);
  kt_settings.ui8_assist_percent[1] = (uint8_t) (PAS_ASSIST_LEVEL_2 * 100.0 + 0.5);
  kt_settings.ui8_assist_percent[2] = (uint8_t) (PAS_ASSIST_LEVEL_3 * 100.0 + 0.5);
  kt_settings.ui8_assist_percent[3] = (uint8_t) (PAS_ASSIST_LEVEL_4 * 100.0 + 0.5);
  kt_settings.ui8_assist_percent[4] = (uint8_t) (PAS_ASSIST_LEVEL_5 * 100.0 + 0.5);
  kt_settings.ui8_rotor_offset_angle = MOTOR_ROTOR_OFFSET_ANGLE;
  kt_settings.ui8_foc_angle_adjust = FOC_READ_ID_CURRENT_ANGLE_ADJUST;
  kt_settings.ui8_erps_interpolation = MOTOR_ROTOR_ERPS_START_INTERPOLATION_60_DEGREES;
  kt_settings.ui8_default_max_speed = DEFAULT_VALUE_MAX_SPEED;
  kt_settings.ui8_default_wheel_size = DEFAULT_VALUE_WHEEL_SIZE;
  kt_settings.ui8_controller_type = CONTROLLER_TYPE;
  kt_settings.ui8_phase_current_factor_x10 = PHASE_CURRENT_FACTOR_X10;
  kt_settings.ui8_display = 0;
  kt_settings.ui8_ramp_x10 = 0;
  kt_settings.i8_foc_delay_x4us = 0;
  kt_settings.ui8_motor_characteristic = DEFAULT_VALUE_MOTOR_CHARACTARISTIC;
  kt_settings.ui8_fw_angle_deg = 0;
  kt_settings.ui8_fw_battery_a = 0;
  kt_settings.ui8_fw_phase_a = 0;
  kt_settings.ui8_fw_ramp_dps = 0;
}

// 1 if the block is valid and in range
static uint8_t block_ok (void)
{
  uint8_t ui8_i;
  uint8_t ui8_sum = 0;

  if ((read_eeprom (0) != 'K') || (read_eeprom (1) != 'T') ||
      (read_eeprom (2) != SETTINGS_BLOCK_VERSION) || (read_eeprom (3) != SETTINGS_BLOCK_LENGTH))
  {
    return 0;
  }
  for (ui8_i = 0; ui8_i < (SETTINGS_BLOCK_LENGTH - 1); ui8_i++) { ui8_sum += read_eeprom (ui8_i); }
  if ((uint8_t) ~ui8_sum != read_eeprom (SETTINGS_BLOCK_LENGTH - 1)) { return 0; }

  // range checks: a value out of range rejects the whole block (better the known defaults than a guess)
  if ((read_eeprom (4) < 7) || (read_eeprom (4) > 16)) { return 0; }
  if ((read_eeprom (5) < 1) || (read_eeprom (5) > 30)) { return 0; }
  if (read_eeprom (6) > 15) { return 0; }
  if ((read_eeprom (7) != EBIKE_THROTTLE_TYPE_THROTTLE_PAS) && (read_eeprom (7) != EBIKE_THROTTLE_TYPE_TORQUE_SENSOR)) { return 0; }
  if ((read_eeprom (9) < 1) || (read_eeprom (9) > 64)) { return 0; }
  if ((read_eeprom (10) < 30) || (read_eeprom (10) > 150)) { return 0; }
  for (ui8_i = 11; ui8_i <= 15; ui8_i++) { if (read_eeprom (ui8_i) > 100) { return 0; } }
  if ((read_eeprom (18) < 5) || (read_eeprom (18) > 100)) { return 0; }
  if ((read_eeprom (22) < 10) || (read_eeprom (22) > 40)) { return 0; }
  return 1;
}

// returns 1 if the EEPROM block is valid and in range
static uint8_t load_block (void)
{
  uint8_t ui8_i;
  uint8_t ui8_p;

  if (!block_ok ()) { return 0; }
  kt_settings.ui8_cells = read_eeprom (4);
  kt_settings.ui8_motor_current_a = read_eeprom (5);
  kt_settings.ui8_regen_current_a = read_eeprom (6);
  kt_settings.ui8_throttle_type = read_eeprom (7);
  kt_settings.ui8_flags = read_eeprom (8);
  kt_settings.ui8_pas_magnets = read_eeprom (9);
  kt_settings.ui8_pas_max_cadence = read_eeprom (10);
  for (ui8_i = 0; ui8_i < 5; ui8_i++) { kt_settings.ui8_assist_percent[ui8_i] = read_eeprom (11 + ui8_i); }
  kt_settings.ui8_rotor_offset_angle = read_eeprom (16);
  kt_settings.ui8_foc_angle_adjust = read_eeprom (17);
  kt_settings.ui8_erps_interpolation = read_eeprom (18);
  ui8_p = read_eeprom (19);
  if ((ui8_p >= 5) && (ui8_p <= 70)) { kt_settings.ui8_default_max_speed = ui8_p; }
  ui8_p = read_eeprom (20);
  if (ui8_p <= 0x1e) { kt_settings.ui8_default_wheel_size = ui8_p; }
  kt_settings.ui8_controller_type = read_eeprom (21);
  kt_settings.ui8_phase_current_factor_x10 = read_eeprom (22);
  ui8_p = read_eeprom (23);
  if (ui8_p <= DISPLAY_LAST) { kt_settings.ui8_display = ui8_p; }
  ui8_p = read_eeprom (24);
  if (ui8_p <= 50) { kt_settings.ui8_ramp_x10 = ui8_p; }
  kt_settings.i8_foc_delay_x4us = (int8_t) read_eeprom (25);
  ui8_p = read_eeprom (26);
  if (ui8_p) { kt_settings.ui8_motor_characteristic = ui8_p; }
  ui8_p = read_eeprom (27);
  if (ui8_p <= 60) { kt_settings.ui8_fw_angle_deg = ui8_p; }
  ui8_p = read_eeprom (28);
  if (ui8_p <= 30) { kt_settings.ui8_fw_battery_a = ui8_p; }
  ui8_p = read_eeprom (29);
  if (ui8_p <= 15) { kt_settings.ui8_fw_phase_a = ui8_p; }
  ui8_p = read_eeprom (30);
  kt_settings.ui8_fw_ramp_dps = (ui8_p > 120) ? 120 : ui8_p; // faster oscillates with the 160 ms current filter
  return 1;
}

static void calc_derived (void)
{
  uint16_t ui16_temp;
  uint8_t ui8_cells = kt_settings.ui8_cells;

  // battery: 4.2 V and 3.0 V per cell, 0.272 V per ADC step
  kt_settings.ui8_adc_battery_voltage_max = (uint8_t) ((((uint16_t) ui8_cells) * 420) / 27);
  kt_settings.ui8_adc_battery_voltage_min = (uint8_t) ((((uint16_t) ui8_cells) * 300) / 27);
  if (ui8_cells <= 8) { kt_settings.ui8_communications_battery_voltage = 24; }
  else if (ui8_cells <= 11) { kt_settings.ui8_communications_battery_voltage = 36; }
  else { kt_settings.ui8_communications_battery_voltage = 48; }
  // display bars: volts * 256 (same scale as ADC * ADC_BATTERY_VOLTAGE_K)
  kt_settings.ui16_battery_pack_volts_x256[0] = (uint16_t) (LI_ION_CELL_VOLTS_100 * 256.0) * ui8_cells;
  kt_settings.ui16_battery_pack_volts_x256[1] = (uint16_t) (LI_ION_CELL_VOLTS_80 * 256.0) * ui8_cells;
  kt_settings.ui16_battery_pack_volts_x256[2] = (uint16_t) (LI_ION_CELL_VOLTS_40 * 256.0) * ui8_cells;
  kt_settings.ui16_battery_pack_volts_x256[3] = (uint16_t) (LI_ION_CELL_VOLTS_20 * 256.0) * ui8_cells;

  // PAS: (1 / (rpm / 60)) / (magnets * 64 us)
  kt_settings.ui16_pas_max_cadence_ticks = 6250 / kt_settings.ui8_pas_magnets;      // 150 rpm
  kt_settings.ui16_pas_min_cadence_ticks = (uint16_t) (156250L / kt_settings.ui8_pas_magnets); // 6 rpm
  if (kt_settings.ui8_pas_magnets < 3) { kt_settings.ui16_pas_min_cadence_ticks = 63000; } // keep below the 16 bit counter limit

  // phase current limit in ADC steps of 0.5 A (the 8 bit total current reading)
  ui16_temp = (((uint16_t) kt_settings.ui8_motor_current_a) * 2 * kt_settings.ui8_phase_current_factor_x10) / 10;
  kt_settings.ui8_phase_current_max = (ui16_temp > 250) ? 250 : (uint8_t) ui16_temp;

  // field weakening
  kt_settings.ui8_fw_angle_max = (uint8_t) ((((uint16_t) kt_settings.ui8_fw_angle_deg) * 256 + 180) / 360);
  ui16_temp = kt_settings.ui8_fw_battery_a ? kt_settings.ui8_fw_battery_a : kt_settings.ui8_motor_current_a;
  kt_settings.ui8_fw_battery_x2 = (uint8_t) (ui16_temp * 2);
  ui16_temp = kt_settings.ui8_fw_phase_a ? kt_settings.ui8_fw_phase_a : 15;
  kt_settings.ui8_fw_phase_peak = (uint8_t) ((ui16_temp * 1000 + 60) / 121); // 15 A -> 124 of 126 steps
  ui16_temp = kt_settings.ui8_fw_ramp_dps ? kt_settings.ui8_fw_ramp_dps : 30;
  kt_settings.ui16_fw_step_x256 = (uint16_t) ((ui16_temp * 182) / 100); // deg/s -> 1/256 x 256 per 10 ms
}

void settings_init (void)
{
  load_defaults ();
  kt_settings.ui8_from_eeprom = load_block ();
  calc_derived ();
}

uint8_t settings_check (const uint8_t *p_block)
{
  uint8_t ui8_ok;
  p_settings_src = p_block;
  ui8_ok = block_ok ();
  p_settings_src = (const volatile uint8_t *) SETTINGS_BLOCK_ADDRESS;
  return ui8_ok;
}

uint8_t settings_state (void)
{
  if (kt_settings.ui8_from_eeprom) { return 1; }
  return ((read_eeprom (0) == 'K') && (read_eeprom (1) == 'T')) ? 2 : 0;
}

void settings_pack (uint8_t *p)
{
  uint8_t ui8_i;
  uint8_t ui8_sum = 0;
  if (kt_settings.ui8_from_eeprom)
  {
    for (ui8_i = 0; ui8_i < SETTINGS_BLOCK_LENGTH; ui8_i++) { p[ui8_i] = read_eeprom (ui8_i); }
    return;
  }
  p[0] = 'K'; p[1] = 'T'; p[2] = SETTINGS_BLOCK_VERSION; p[3] = SETTINGS_BLOCK_LENGTH;
  p[4] = kt_settings.ui8_cells;
  p[5] = kt_settings.ui8_motor_current_a;
  p[6] = kt_settings.ui8_regen_current_a;
  p[7] = kt_settings.ui8_throttle_type;
  p[8] = kt_settings.ui8_flags;
  p[9] = kt_settings.ui8_pas_magnets;
  p[10] = kt_settings.ui8_pas_max_cadence;
  for (ui8_i = 0; ui8_i < 5; ui8_i++) { p[11 + ui8_i] = kt_settings.ui8_assist_percent[ui8_i]; }
  p[16] = kt_settings.ui8_rotor_offset_angle;
  p[17] = kt_settings.ui8_foc_angle_adjust;
  p[18] = kt_settings.ui8_erps_interpolation;
  p[19] = kt_settings.ui8_default_max_speed;
  p[20] = kt_settings.ui8_default_wheel_size;
  p[21] = kt_settings.ui8_controller_type;
  p[22] = kt_settings.ui8_phase_current_factor_x10;
  p[23] = kt_settings.ui8_display;
  p[24] = kt_settings.ui8_ramp_x10;
  p[25] = (uint8_t) kt_settings.i8_foc_delay_x4us;
  p[26] = kt_settings.ui8_motor_characteristic;
  p[27] = kt_settings.ui8_fw_angle_deg;
  p[28] = kt_settings.ui8_fw_battery_a;
  p[29] = kt_settings.ui8_fw_phase_a;
  p[30] = kt_settings.ui8_fw_ramp_dps;
  for (ui8_i = 0; ui8_i < (SETTINGS_BLOCK_LENGTH - 1); ui8_i++) { ui8_sum += p[ui8_i]; }
  p[SETTINGS_BLOCK_LENGTH - 1] = (uint8_t) ~ui8_sum;
}

uint8_t settings_flag (uint8_t ui8_flag)
{
  return (kt_settings.ui8_flags & ui8_flag) ? 1 : 0;
}
