/*
 * Display protocols of the KT firmware, chosen at runtime (kt_settings.ui8_display):
 *   - Kunteng KT-LCD3 / LCD5: controller sends a 12 byte status every 150 ms, the display a 13 byte packet
 *     (casainho's code, moved here from ebike_app.c)
 *   - KingMeter 901U / KM5S: request / answer, 0x52 operation and 0x53 settings messages with the 901U
 *     handshake (ported from EBiCS display_kingmeter.c, Michael Fabry's library)
 *   - Bafang LCD (1200 baud) and 850C / 860C (9600 baud): request / answer (ported from EBiCS display_bafang.c)
 *   - KingMeter J-LCD / 618U, UART No.2 (S866, EN06 ...), Tongsheng (VLCD5 / VLCD6 / XH18): the display sends,
 *     the controller answers each valid frame (ported from EBiCS display_kingmeter.c, display_No_2.c,
 *     display_tongsheng.c)
 *   - VESC UART (115200 baud): live values for VESC displays / apps, current requests (display_vesc.c)
 *   - EBiCS app / BLE module: ANT+ LEV pages 1 / 4 out, page 16 in (display_ebics.c)
 *   - IONRISE (iondisp display): handshake, full telemetry, the display reads / writes the settings block
 *     (protocol code shared with iondisp and EBiCS Lishui in ../ionrise, spec ionrise/IONRISE_PROTOCOL.md)
 * Receiving and sending are interrupt driven (ring buffers), so the main loop never waits for the UART.
 *
 * This file is part of the KT firmware, GPL v3.
 */

#include <stdint.h>
#include "stm8s.h"
#include "stm8s_uart2.h"
#include "main.h"
#include "interrupts.h"
#include "display.h"
#include "settings.h"
#include "ebike_app.h"
#include "motor.h"
#include "brake.h"
#include "eeprom.h"
#include "ionctrl.h"

extern volatile struc_lcd_configuration_variables lcd_configuration_variables;
extern float f_wheel_speed;
extern float f_wheel_perimeter;
uint8_t ebike_app_cruise_control_is_set (void);
uint8_t throttle_is_set (void);

uint16_t ui16_display_wheel_perimeter_mm = 0;
uint8_t ui8_display_power_pct = 100;  // power limit of the display (IONRISE), % of the current limits
uint8_t ui8_display_no_throttle = 0;  // the display asks to ignore the throttle (IONRISE)
extern uint8_t ui8_pas_cadence_rpm;
extern volatile uint8_t ui8_ad_cmd;
extern volatile uint8_t ui8_ad_active;
extern volatile uint8_t ui8_phase_fault;
extern volatile uint8_t ui8_fw_angle;

// ------------------------------------------------------------------------------------------- ring buffers
#define RX_SIZE 64      // power of 2
#define TX_SIZE 128     // power of 2 (a VESC values answer is 79 bytes)
// not static: the simulator test (kunteng/sim/run_sim.py) writes display messages straight into the buffer
volatile uint8_t ui8_rx_buf[RX_SIZE];
volatile uint8_t ui8_rx_head = 0;          // written by the interrupt
uint8_t ui8_rx_tail = 0;                   // read by display_poll
static volatile uint8_t ui8_tx_buf[TX_SIZE];
static uint8_t ui8_tx_head = 0;
static volatile uint8_t ui8_tx_tail = 0;

void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER)
{
  uint8_t ui8_next;
  uint8_t ui8_byte;
  if (UART2->SR & (UART2_SR_RXNE | UART2_SR_OR))
  {
    ui8_byte = UART2->DR; // reading DR also clears an overrun
    ui8_next = (ui8_rx_head + 1) & (RX_SIZE - 1);
    if (ui8_next != ui8_rx_tail) // full: drop the byte
    {
      ui8_rx_buf[ui8_rx_head] = ui8_byte;
      ui8_rx_head = ui8_next;
    }
  }
}

// "transmission complete" interrupt: one byte at a time (robust, the gap between bytes is negligible)
static volatile uint8_t ui8_tx_busy = 0;

void UART2_TX_IRQHandler(void) __interrupt(UART2_TX_IRQHANDLER)
{
  (void) UART2->SR; // TC is cleared by reading SR and then writing DR
  if (ui8_tx_tail != ui8_tx_head)
  {
    UART2->DR = ui8_tx_buf[ui8_tx_tail];
    ui8_tx_tail = (ui8_tx_tail + 1) & (TX_SIZE - 1);
  }
  else
  {
    UART2->CR2 &= (uint8_t) ~UART2_CR2_TCIEN;
    UART2->SR &= (uint8_t) ~UART2_SR_TC;
    ui8_tx_busy = 0;
  }
}

// KT error -> KingMeter error: under voltage is the blinking battery symbol (0x91 is no KingMeter code), other
// faults a generic code (0x25 / 0x26 mean over temperature on KingMeter displays, KT error 04 is 0x26)
static uint8_t km_error (void)
{
  uint8_t ui8_e = motor_controller_get_error ();
  if (ui8_e == MOTOR_CONTROLLER_ERROR_91_BATTERY_UNDER_VOLTAGE) { return 0x00; }
  return ui8_e ? 0x30 : 0x00;
}

// free bytes in the TX ring
static uint8_t tx_free (void)
{
  return (uint8_t) ((ui8_tx_tail - ui8_tx_head - 1) & (TX_SIZE - 1));
}

static void tx (uint8_t ui8_byte)
{
  uint8_t ui8_next = (ui8_tx_head + 1) & (TX_SIZE - 1);
  if (ui8_next == ui8_tx_tail) { return; } // full: a message is never longer than the buffer, so drop
  disableInterrupts ();
  if (!ui8_tx_busy)
  {
    ui8_tx_busy = 1;
    (void) UART2->SR;
    UART2->DR = ui8_byte;
    UART2->CR2 |= UART2_CR2_TCIEN;
  }
  else
  {
    ui8_tx_buf[ui8_tx_head] = ui8_byte;
    ui8_tx_head = ui8_next;
  }
  enableInterrupts ();
}

static uint8_t rx_count (void)
{
  return (ui8_rx_head - ui8_rx_tail) & (RX_SIZE - 1);
}

static uint8_t rx_peek (uint8_t ui8_offset)
{
  return ui8_rx_buf[(ui8_rx_tail + ui8_offset) & (RX_SIZE - 1)];
}

static void rx_drop (uint8_t ui8_n)
{
  ui8_rx_tail = (ui8_rx_tail + ui8_n) & (RX_SIZE - 1);
}

// ------------------------------------------------------------------------------------------- common values
static uint16_t wheel_period_ms (void)
{
  // standstill: a very long period (shows 0 km/h). 36000 x circumference overflowed 16 bits for wheels over
  // 1.82 m (26": 74250 -> 8714 ms = 0.9 km/h on the display)
  if (f_wheel_speed < 1) { return 65000; }
  return (uint16_t) ((3600.0 * f_wheel_perimeter) / f_wheel_speed);
}

static uint8_t battery_bars_x4 (void) // 16 full, 12, 8, 4, 3 empty (KT display scale)
{
  uint16_t ui16_volts = motor_get_ADC_battery_voltage_filtered () * ADC_BATTERY_VOLTAGE_K;
  if (ui16_volts > kt_settings.ui16_battery_pack_volts_x256[0]) { return 16; }
  if (ui16_volts > kt_settings.ui16_battery_pack_volts_x256[1]) { return 12; }
  if (ui16_volts > kt_settings.ui16_battery_pack_volts_x256[2]) { return 8; }
  if (ui16_volts > kt_settings.ui16_battery_pack_volts_x256[3]) { return 4; }
  return 3;
}

static uint16_t battery_current_x10 (void) // A x10, motoring only
{
  int16_t i16_current = motor_get_current_filtered_10b (); // 0.125 A
  if (i16_current < 0) { return 0; }
  return (uint16_t) ((i16_current * 10) >> 3);
}

static void use_default_lcd_values (void)
{
  // displays without the KT parameters: values from the settings block
  lcd_configuration_variables.ui8_motor_characteristic = kt_settings.ui8_motor_characteristic;
  lcd_configuration_variables.ui8_power_assist_control_mode = 1; // speed limit only, power from throttle / PAS
  lcd_configuration_variables.ui8_controller_max_current = 10;   // 100 %
}

// ------------------------------------------------------------------------------------------- Kunteng
static void kt_send_status (void)
{
  uint8_t ui8_buf[12];
  uint8_t ui8_i;
  uint8_t ui8_crc = 0;
  uint8_t ui8_moving = 0;
  uint8_t ui8_error = motor_controller_get_error ();
  uint8_t ui8_soc = battery_bars_x4 ();
  uint16_t ui16_period = wheel_period_ms ();
  int16_t i16_current = motor_get_current_filtered_10b ();

  if (ui8_error == MOTOR_CONTROLLER_ERROR_91_BATTERY_UNDER_VOLTAGE) { ui8_soc = 1; ui8_error = 0; } // empty, flashing
  if (brake_is_set ()) { ui8_moving |= (1 << 5); }
  if (ebike_app_cruise_control_is_set ()) { ui8_moving |= (1 << 3); }
  if (throttle_is_set ()) { ui8_moving |= (1 << 1); }
  if (i16_current < 0) { i16_current = 0; }
  i16_current <<= 1; // 0.125 A -> 0.25 A
  if (i16_current > 255) { i16_current = 255; }

  ui8_buf[0] = 65;
  ui8_buf[1] = ui8_soc;
  ui8_buf[2] = kt_settings.ui8_communications_battery_voltage;
  ui8_buf[3] = (uint8_t) (ui16_period >> 8);
  ui8_buf[4] = (uint8_t) ui16_period;
  ui8_buf[5] = ui8_error;
  ui8_buf[6] = 0;
  ui8_buf[7] = ui8_moving;
  ui8_buf[8] = (uint8_t) i16_current;
  ui8_buf[9] = 0;
  ui8_buf[10] = 0;
  ui8_buf[11] = 0;
  for (ui8_i = 1; ui8_i <= 11; ui8_i++) { ui8_crc ^= ui8_buf[ui8_i]; }
  ui8_buf[6] = ui8_crc;
  for (ui8_i = 0; ui8_i <= 11; ui8_i++) { tx (ui8_buf[ui8_i]); }
}

// casainho's framing: the packet is taken from the bytes 50, 14 (the end of the previous display packet) on,
// 12 bytes, B10 of the display packet is not used and counts as 0 in the CRC
static void kt_poll (void)
{
  uint8_t ui8_p[13];
  uint8_t ui8_i;
  uint8_t ui8_crc = 0;

  while (rx_count () >= 2 && !((rx_peek (0) == 50) && (rx_peek (1) == 14))) { rx_drop (1); }
  if (rx_count () < 12) { return; }
  for (ui8_i = 0; ui8_i < 12; ui8_i++) { ui8_p[ui8_i] = rx_peek (ui8_i); }
  ui8_p[12] = 0;
  rx_drop (12);
  for (ui8_i = 0; ui8_i <= 12; ui8_i++) { if (ui8_i != 7) { ui8_crc ^= ui8_p[ui8_i]; } }
  if (((ui8_crc ^ 10) == ui8_p[7]) || ((ui8_crc ^ 9) == ui8_p[7]) || ((ui8_crc ^ 2) == ui8_p[7]))
  {
    lcd_configuration_variables.ui8_assist_level = ui8_p[3] & 7;
    lcd_configuration_variables.ui8_motor_characteristic = ui8_p[5];
    lcd_configuration_variables.ui8_wheel_size = ((ui8_p[6] & 192) >> 6) | ((ui8_p[4] & 7) << 2);
    lcd_configuration_variables.ui8_max_speed = 10 + (((ui8_p[4] & 248) >> 3) | (ui8_p[6] & 32));
    lcd_configuration_variables.ui8_power_assist_control_mode = ui8_p[6] & 8;
    lcd_configuration_variables.ui8_controller_max_current = (ui8_p[9] & 15);
    eeprom_write_if_values_changed ();
  }
}

// ------------------------------------------------------------------------------------------- KingMeter 901U
static const uint8_t ui8_km_handshake[64] = {
  137, 159, 134, 249, 88, 11, 250, 61, 33, 150, 3, 193, 118, 141, 209, 94, 226, 68, 146, 158, 145, 127,
  216, 62, 116, 230, 101, 211, 251, 54, 229, 247, 20, 222, 59, 63, 35, 252, 142, 238, 23, 197, 84, 77,
  147, 173, 210, 57, 142, 223, 157, 97, 36, 160, 229, 237, 75, 80, 37, 113, 154, 88, 23, 120 };

static void km_answer (uint8_t ui8_cmd, uint8_t ui8_handshake_index)
{
  uint8_t ui8_buf[9];
  uint8_t ui8_i;
  uint16_t ui16_sum = 0;
  uint16_t ui16_period = wheel_period_ms ();
  uint16_t ui16_current = battery_current_x10 ();

  if (ui16_period > 0x0DAC) { ui16_period = 0x0DAC; } // KM_MAX_WHEELTIME: wheel stopped
  ui8_buf[0] = 0x3A;
  ui8_buf[1] = 0x1A;
  ui8_buf[2] = ui8_cmd;
  ui8_buf[3] = 0x05;
  if (ui8_cmd == 0x52)
  {
    ui8_buf[4] = (battery_bars_x4 () <= 3) ? 0x40 : 0x00;          // battery low symbol
    ui8_buf[5] = (uint8_t) ((ui16_current * 3) / 10);              // 1/3 A
    ui8_buf[6] = (uint8_t) (ui16_period >> 8);
    ui8_buf[7] = (uint8_t) ui16_period;
    ui8_buf[8] = km_error ();
  }
  else // 0x53 settings: handshake
  {
    ui8_buf[4] = 0x00;
    ui8_buf[5] = 0x00;
    ui8_buf[6] = 0x0D;
    ui8_buf[7] = ui8_km_handshake[ui8_handshake_index & 0x3F];
    ui8_buf[8] = 0x00;
  }
  for (ui8_i = 1; ui8_i < 9; ui8_i++) { ui16_sum += ui8_buf[ui8_i]; }
  for (ui8_i = 0; ui8_i < 9; ui8_i++) { tx (ui8_buf[ui8_i]); }
  tx ((uint8_t) ui16_sum);
  tx ((uint8_t) (ui16_sum >> 8));
  tx (0x0D);
  tx (0x0A);
}

// message: 0x3A, address, command, length n, n data bytes, checksum (sum of bytes 1 ... 3+n, low byte first), 0x0D 0x0A
static void km_poll (void)
{
  uint8_t ui8_n;
  uint8_t ui8_i;
  uint8_t ui8_cmd;
  uint16_t ui16_sum = 0;
  uint16_t ui16_got;
  uint8_t ui8_level;
  uint16_t ui16_mm;

  while (rx_count () && (rx_peek (0) != 0x3A)) { rx_drop (1); }
  if (rx_count () < 4) { return; }
  ui8_n = rx_peek (3);
  if (ui8_n > 20) { rx_drop (1); return; } // not a real length: resync
  if (rx_count () < (uint8_t) (ui8_n + 6)) { return; }
  for (ui8_i = 1; ui8_i < (uint8_t) (4 + ui8_n); ui8_i++) { ui16_sum += rx_peek (ui8_i); }
  ui16_got = rx_peek (4 + ui8_n) | (((uint16_t) rx_peek (5 + ui8_n)) << 8);
  ui8_cmd = rx_peek (2);
  if (ui16_sum == ui16_got)
  {
    if ((ui8_cmd == 0x52) && (ui8_n >= 2))
    {
      // assist level 0 ... 255 of the KM display -> KT levels 0 ... 5
      ui8_level = rx_peek (4);
      lcd_configuration_variables.ui8_assist_level = (uint8_t) ((((uint16_t) ui8_level) * 5 + 127) / 255);
      km_answer (0x52, 0);
    }
    else if ((ui8_cmd == 0x53) && (ui8_n >= 10))
    {
      ui16_mm = (((uint16_t) rx_peek (12)) << 8) | rx_peek (13);
      if ((ui16_mm >= 500) && (ui16_mm <= 3000)) { ui16_display_wheel_perimeter_mm = ui16_mm; }
      ui8_level = rx_peek (11);                       // speed limit (km/h)
      if ((ui8_level >= 5) && (ui8_level <= 70)) { lcd_configuration_variables.ui8_max_speed = ui8_level; }
      km_answer (0x53, rx_peek (9));
    }
  }
  rx_drop (ui8_n + 6);
}

// ------------------------------------------------------------------------------------------- Bafang
// requests 0x11 <cmd> are answered at once; infos 0x16 <cmd> ... set values
static void bf_poll (void)
{
  uint8_t ui8_cmd;
  uint8_t ui8_v;
  uint16_t ui16_v;
  uint16_t ui16_period;

  while (rx_count () && (rx_peek (0) != 0x11) && (rx_peek (0) != 0x16)) { rx_drop (1); }
  if (rx_count () < 2) { return; }
  ui8_cmd = rx_peek (1);
  if (rx_peek (0) == 0x11)
  {
    rx_drop (2);
    switch (ui8_cmd)
    {
      case 0x20: // speed: wheel rpm
      ui16_period = wheel_period_ms ();
      ui16_v = (ui16_period >= 36000U) ? 0 : (uint16_t) (60000UL / ui16_period);
      tx ((uint8_t) (ui16_v >> 8));
      tx ((uint8_t) ui16_v);
      tx ((uint8_t) ((uint8_t) (ui16_v >> 8) + (uint8_t) ui16_v + 32));
      break;
      case 0x08: // status
      tx (brake_is_set () ? 0x03 : 0x01);
      break;
      case 0x11: // battery %
      ui8_v = battery_bars_x4 ();
      ui8_v = (ui8_v <= 3) ? 5 : (uint8_t) (ui8_v * 6); // 4 -> 24 %, 8 -> 48, 12 -> 72, 16 -> 96
      tx (ui8_v);
      tx (ui8_v);
      break;
      case 0x0A: // current, 0.5 A
      ui16_v = battery_current_x10 () / 5;
      ui8_v = (ui16_v > 255) ? 255 : (uint8_t) ui16_v;
      tx (ui8_v);
      tx (ui8_v);
      break;
      case 0x31: // moving
      ui8_v = (ui16_motor_get_motor_speed_erps () > 0) ? '1' : '0';
      tx (ui8_v);
      tx (ui8_v);
      break;
      case 0x24: // battery voltage x100
      ui16_v = (uint16_t) ((((uint32_t) motor_get_ADC_battery_voltage_filtered ()) * 272) / 10);
      tx ((uint8_t) (ui16_v >> 8));
      tx ((uint8_t) ui16_v);
      tx ((uint8_t) ((uint8_t) (ui16_v >> 8) + (uint8_t) ui16_v));
      break;
      default:
      break;
    }
    return;
  }
  // info
  if (ui8_cmd == 0x0B) // assist level: 0x16 0x0B level checksum
  {
    if (rx_count () < 4) { return; }
    ui8_v = rx_peek (2);
    if (rx_peek (3) == (uint8_t) (0x16 + 0x0B + ui8_v))
    {
      // Bafang codes for the 9 level display, 3 and 5 level displays use a subset
      switch (ui8_v)
      {
        case 0: lcd_configuration_variables.ui8_assist_level = 0; break;
        case 1: case 11: lcd_configuration_variables.ui8_assist_level = 1; break;
        case 13: case 21: lcd_configuration_variables.ui8_assist_level = 2; break;
        case 2: case 23: case 3: lcd_configuration_variables.ui8_assist_level = 3; break;
        case 12: case 27: case 28: lcd_configuration_variables.ui8_assist_level = 4; break;
        case 29: case 30: lcd_configuration_variables.ui8_assist_level = 5; break;
        default: break; // 6 = walk assist: not supported
      }
    }
    rx_drop (4);
  }
  else if (ui8_cmd == 0x1A) { if (rx_count () < 3) { return; } rx_drop (3); } // light
  else if (ui8_cmd == 0x1F) { if (rx_count () < 5) { return; } rx_drop (5); } // wheel diameter
  else { rx_drop (2); }
}

// ------------------------------------------------------------------------------------------- shared by the ports
extern volatile uint16_t ui16_duty_cycle;

static void set_assist_level (uint8_t ui8_level)
{
  lcd_configuration_variables.ui8_assist_level = (ui8_level > 5) ? 5 : ui8_level;
}

static void set_wheel_mm (uint16_t ui16_mm)
{
  if ((ui16_mm >= 500) && (ui16_mm <= 3000)) { ui16_display_wheel_perimeter_mm = ui16_mm; }
}

static void set_max_speed (uint8_t ui8_kmh)
{
  if ((ui8_kmh >= 5) && (ui8_kmh <= 70)) { lcd_configuration_variables.ui8_max_speed = ui8_kmh; }
}

static uint8_t xor_sum (uint8_t ui8_from, uint8_t ui8_n) // XOR of rx bytes from ... from + n - 1
{
  uint8_t ui8_x = 0;
  while (ui8_n--) { ui8_x ^= rx_peek (ui8_from++); }
  return ui8_x;
}

static uint8_t add_sum (uint8_t ui8_n) // 8 bit sum of the first n rx bytes
{
  uint8_t ui8_s = 0;
  uint8_t ui8_i;
  for (ui8_i = 0; ui8_i < ui8_n; ui8_i++) { ui8_s += rx_peek (ui8_i); }
  return ui8_s;
}

static uint8_t battery_volts_x4 (void) // 0.25 V
{
  uint16_t ui16_v = (uint16_t) ((((uint32_t) motor_get_ADC_battery_voltage_filtered ()) * 1088) / 1000);
  return (ui16_v > 255) ? 255 : (uint8_t) ui16_v;
}

// ------------------------------------------------------------------------------------------- KingMeter J-LCD / 618U
static const uint16_t ui16_km618_wheel_mm[8] = { 1277, 1436, 1596, 1756, 1915, 2075, 2154, 2234 };

// display: 0x3A, b1 (assist bits 0..2, push assist bit 4, light bit 7), b2 (speed limit bits 3..7, wheel bits 0..2),
// b3, XOR of b1..b3. Answer: 0x46, battery, current (1/3 A), wheel time (ms, hi lo), 0x7A, error, XOR of 1..6
static void km618_poll (void)
{
  uint8_t ui8_buf[8];
  uint8_t ui8_i;
  uint16_t ui16_period;

  while (rx_count () && (rx_peek (0) != 0x3A)) { rx_drop (1); }
  if (rx_count () < 5) { return; }
  if (xor_sum (1, 3) != rx_peek (4)) { rx_drop (1); return; }
  set_assist_level (rx_peek (1) & 0x07);
  set_max_speed (((rx_peek (2) & 0xF8) >> 3) + 10);
  set_wheel_mm (ui16_km618_wheel_mm[rx_peek (2) & 0x07]);
  rx_drop (5);

  ui16_period = wheel_period_ms ();
  if (ui16_period > 0x0DAC) { ui16_period = 0x0DAC; }
  ui8_buf[0] = 0x46;
  ui8_buf[1] = (battery_bars_x4 () <= 3) ? 0x00 : 0x01; // none of bits 0..2: the battery symbol blinks
  ui8_buf[2] = (uint8_t) ((battery_current_x10 () * 3) / 10);
  ui8_buf[3] = (uint8_t) (ui16_period >> 8);
  ui8_buf[4] = (uint8_t) ui16_period;
  ui8_buf[5] = 0x7A;
  ui8_buf[6] = km_error ();
  ui8_buf[7] = 0;
  for (ui8_i = 1; ui8_i < 7; ui8_i++) { ui8_buf[7] ^= ui8_buf[ui8_i]; }
  for (ui8_i = 0; ui8_i < 8; ui8_i++) { tx (ui8_buf[ui8_i]); }
}

// ------------------------------------------------------------------------------------------- UART No.2
// display: 20 bytes 0x01 0x14 ..., byte 19 = XOR of bytes 0 ... 18. Assist level byte 4, wheel size inch x10
// bytes 7/8, speed limit byte 12. Answer: 14 bytes 0x02 0x0E 0x01, error, brake (bit 5), current x10 (6/7),
// wheel time ms (8/9), byte 13 = XOR of bytes 0 ... 12
static void no2_poll (void)
{
  uint8_t ui8_buf[14];
  uint8_t ui8_i;
  uint8_t ui8_level;
  uint8_t ui8_error;
  uint16_t ui16_v;

  while (rx_count () >= 2 && !((rx_peek (0) == 0x01) && (rx_peek (1) == 0x14))) { rx_drop (1); }
  if (rx_count () < 20) { return; }
  if (xor_sum (0, 19) != rx_peek (19)) { rx_drop (1); return; }
  ui8_level = rx_peek (4);                     // 0 ... 5, or 0 ... 9 on displays set to 9 levels
  // levels above 5 (9 level displays) stay at 5: scaling them gave 6 -> 3, less than level 5
  set_assist_level (ui8_level);
  ui16_v = (((uint16_t) rx_peek (7)) << 8) | rx_peek (8);
  set_wheel_mm ((uint16_t) (((uint32_t) ui16_v * 798) / 100)); // inch x10 -> mm (x 25.4 x pi / 10)
  set_max_speed (rx_peek (12));
  rx_drop (20);

  ui8_error = motor_controller_get_error ();
  if (ui8_error == MOTOR_CONTROLLER_ERROR_91_BATTERY_UNDER_VOLTAGE) { ui8_error = 0; } // display shows its own
  else if ((ui8_error == MOTOR_CONTROLLER_ERROR_HALL) || (ui8_error == MOTOR_CONTROLLER_ERROR_06_SHORT_CIRCUIT)) { ui8_error = 7; } // motor failure
  else if (ui8_error) { ui8_error = 9; }      // controller failure
  ui16_v = battery_current_x10 ();
  ui8_buf[0] = 0x02;
  ui8_buf[1] = 0x0E;
  ui8_buf[2] = 0x01;
  ui8_buf[3] = ui8_error;
  ui8_buf[4] = brake_is_set () ? 0x20 : 0x00;
  ui8_buf[5] = 0;
  ui8_buf[6] = (uint8_t) (ui16_v >> 8);
  ui8_buf[7] = (uint8_t) ui16_v;
  ui16_v = wheel_period_ms ();
  ui8_buf[8] = (uint8_t) (ui16_v >> 8);
  ui8_buf[9] = (uint8_t) ui16_v;
  ui8_buf[10] = 0;
  ui8_buf[11] = 0;
  ui8_buf[12] = 0xFF;
  ui8_buf[13] = 0;
  for (ui8_i = 0; ui8_i < 13; ui8_i++) { ui8_buf[13] ^= ui8_buf[ui8_i]; }
  for (ui8_i = 0; ui8_i < 14; ui8_i++) { tx (ui8_buf[ui8_i]); }
}

// ------------------------------------------------------------------------------------------- Tongsheng
// display: 7 bytes 0x59, b1 (light 0x01, level 3 0x02, 4 0x04, 5 0x08, 0 0x10, walk 0x20, 2 0x40, 1 0x80),
// b2, wheel inch, b4, speed limit, sum of bytes 0 ... 5. Answer: 9 bytes 0x43, battery 0 ... 12, status
// (bit 0 under voltage, bit 2 motor working), 0, current x10, error, wheel time (2 ms, low high), sum of 0 ... 7
static void ts_poll (void)
{
  uint8_t ui8_buf[9];
  uint8_t ui8_i;
  uint8_t ui8_b;
  uint16_t ui16_v;
  uint16_t ui16_lo;
  uint16_t ui16_hi;

  while (rx_count () && (rx_peek (0) != 0x59)) { rx_drop (1); }
  if (rx_count () < 7) { return; }
  if (add_sum (6) != rx_peek (6)) { rx_drop (1); return; }
  ui8_b = rx_peek (1);
  if (ui8_b & 0x08) { ui8_i = 5; }
  else if (ui8_b & 0x04) { ui8_i = 4; }
  else if (ui8_b & 0x02) { ui8_i = 3; }
  else if (ui8_b & 0x40) { ui8_i = 2; }
  else if (ui8_b & 0x80) { ui8_i = 1; }
  else { ui8_i = 0; }
  set_assist_level (ui8_i);                    // walk assist (0x20) and light: not on the KT controller
  ui8_b = rx_peek (3);
  if (ui8_b) { set_wheel_mm ((uint16_t) ui8_b * 80); } // inch -> mm (x 25.4 x pi)
  set_max_speed (rx_peek (5));
  rx_drop (7);

  // battery 0 ... 12 between the 20 % and 100 % voltages of the battery bars
  ui16_v = motor_get_ADC_battery_voltage_filtered () * ADC_BATTERY_VOLTAGE_K;
  ui16_lo = kt_settings.ui16_battery_pack_volts_x256[3];
  ui16_hi = kt_settings.ui16_battery_pack_volts_x256[0];
  if (ui16_v <= ui16_lo) { ui8_buf[1] = 0; }
  else if (ui16_v >= ui16_hi) { ui8_buf[1] = 12; }
  else { ui8_buf[1] = (uint8_t) (((uint32_t) (ui16_v - ui16_lo) * 12) / (ui16_hi - ui16_lo)); }
  ui8_buf[0] = 0x43;
  ui8_buf[2] = ((motor_controller_get_error () == MOTOR_CONTROLLER_ERROR_91_BATTERY_UNDER_VOLTAGE) ? 0x01 : 0x00) |
               (ui16_motor_get_motor_speed_erps () ? 0x04 : 0x00);
  ui8_buf[3] = 0;
  ui16_v = battery_current_x10 ();
  ui8_buf[4] = (ui16_v > 255) ? 255 : (uint8_t) ui16_v;
  ui8_buf[5] = 0;                              // only "temperature protection" is defined: no sensor on KT
  ui16_v = wheel_period_ms () >> 1;
  if (ui16_v > 0x0700) { ui16_v = 0x0700; }
  ui8_buf[6] = (uint8_t) ui16_v;
  ui8_buf[7] = (uint8_t) (ui16_v >> 8);
  ui8_buf[8] = 0;
  for (ui8_i = 0; ui8_i < 8; ui8_i++) { ui8_buf[8] += ui8_buf[ui8_i]; }
  for (ui8_i = 0; ui8_i < 9; ui8_i++) { tx (ui8_buf[ui8_i]); }
}

// ------------------------------------------------------------------------------------------- VESC UART
// packet: 0x02, length, payload, CRC16-CCITT (XMODEM) of the payload high low, 0x03. Answers COMM_FW_VERSION and
// COMM_GET_VALUES; COMM_SET_CURRENT replaces throttle / PAS until 1 s after the last command (the brake still
// wins in the motor control). Not a full VESC: no configuration from VESC Tool.
#define COMM_FW_VERSION         0
#define COMM_GET_VALUES         4
#define COMM_SET_DUTY           5
#define COMM_SET_CURRENT        6
#define COMM_SET_CURRENT_BRAKE  7
#define COMM_ALIVE              30
#define VESC_TIMEOUT            7  // x 150 ms

static uint16_t ui16_vesc_crc;
static uint16_t ui16_vesc_current_10b = 0;
static uint8_t ui8_vesc_timeout = 0;

static uint16_t crc16_step (uint16_t ui16_crc, uint8_t ui8_byte)
{
  uint8_t ui8_i;
  ui16_crc ^= ((uint16_t) ui8_byte) << 8;
  for (ui8_i = 0; ui8_i < 8; ui8_i++) { ui16_crc = (ui16_crc & 0x8000) ? ((ui16_crc << 1) ^ 0x1021) : (ui16_crc << 1); }
  return ui16_crc;
}

static void vesc_byte (uint8_t ui8_byte)
{
  ui16_vesc_crc = crc16_step (ui16_vesc_crc, ui8_byte);
  tx (ui8_byte);
}

static void vesc_16 (uint16_t ui16_v) { vesc_byte ((uint8_t) (ui16_v >> 8)); vesc_byte ((uint8_t) ui16_v); }
static void vesc_32 (int32_t i32_v) { vesc_16 ((uint16_t) (i32_v >> 16)); vesc_16 ((uint16_t) i32_v); }
static void vesc_zeros (uint8_t ui8_n) { while (ui8_n--) { vesc_byte (0); } }

static void vesc_start (uint8_t ui8_len, uint8_t ui8_cmd)
{
  tx (0x02);
  tx (ui8_len);
  ui16_vesc_crc = 0;
  vesc_byte (ui8_cmd);
}

static void vesc_end (void)
{
  tx ((uint8_t) (ui16_vesc_crc >> 8));
  tx ((uint8_t) ui16_vesc_crc);
  tx (0x03);
}

static void vesc_values (void)
{
  int32_t i32_input = ((int32_t) motor_get_current_filtered_10b () * 25) / 2; // 0.125 A -> 0.01 A
  int32_t i32_motor = i32_input;
  uint16_t ui16_duty = ui16_duty_cycle;
  uint8_t ui8_error = motor_controller_get_error ();
  uint8_t ui8_fault = 0;

  // phase current ~ battery current / duty cycle (not below 1/4 duty: the estimate gets too uncertain)
  if (ui16_duty < 127) { ui16_duty = 127; }
  if (ui16_duty < 508) { i32_motor = (i32_motor * 508) / ui16_duty; }
  if (ui8_error == MOTOR_CONTROLLER_ERROR_OVER_VOLTAGE) { ui8_fault = 1; }
  else if (ui8_error == MOTOR_CONTROLLER_ERROR_91_BATTERY_UNDER_VOLTAGE) { ui8_fault = 2; }
  else if (ui8_error == MOTOR_CONTROLLER_ERROR_06_SHORT_CIRCUIT) { ui8_fault = 4; }

  vesc_start (74, COMM_GET_VALUES);
  vesc_zeros (4);                                          // FET and motor temperature: no sensors
  vesc_32 (i32_motor);                                     // motor current, 0.01 A
  vesc_32 (i32_input);                                     // battery current, 0.01 A
  vesc_32 (0);                                             // id
  vesc_32 (i32_motor);                                     // iq
  vesc_16 ((uint16_t) (((uint32_t) ui16_duty_cycle * 1000) / 508)); // duty cycle, 0.1 %
  vesc_32 ((int32_t) ui16_motor_get_motor_speed_erps () * 60);     // electrical rpm
  vesc_16 ((uint16_t) (((uint32_t) motor_get_ADC_battery_voltage_filtered () * 272) / 100)); // 0.1 V (16 bit overflowed above 65 V)
  vesc_zeros (24);                                         // Ah, Ah charged, Wh, Wh charged, tacho, tacho abs
  vesc_byte (ui8_fault);
  vesc_zeros (4 + 1 + 6 + 8 + 1);                          // pid pos, controller id, 3 MOSFET temps, vd, vq, status
  vesc_end ();
}

static void vesc_poll (void)
{
  uint8_t ui8_len;
  uint8_t ui8_i;
  uint16_t ui16_crc = 0;
  int32_t i32_ma;

  while (rx_count () && ((rx_peek (0) != 0x02) || (rx_count () >= 2 && ((rx_peek (1) == 0) || (rx_peek (1) > 8)))))  // longest handled frame: a false header stalls 13 bytes at most
  {
    rx_drop (1);
  }
  if (rx_count () < 2) { return; }
  ui8_len = rx_peek (1);
  if (rx_count () < (uint8_t) (ui8_len + 5)) { return; }
  for (ui8_i = 0; ui8_i < ui8_len; ui8_i++) { ui16_crc = crc16_step (ui16_crc, rx_peek (2 + ui8_i)); }
  if ((rx_peek (4 + ui8_len) != 0x03) || (rx_peek (2 + ui8_len) != (uint8_t) (ui16_crc >> 8)) ||
      (rx_peek (3 + ui8_len) != (uint8_t) ui16_crc))
  {
    rx_drop (1);
    return;
  }
  switch (rx_peek (2))
  {
    case COMM_FW_VERSION:
    if (tx_free () < 33) { return; }                       // whole answer must fit: answered on a later call
    vesc_start (28, COMM_FW_VERSION);
    vesc_byte (6);                                         // major
    vesc_byte (0);                                         // minor
    vesc_byte ('E'); vesc_byte ('B'); vesc_byte ('i'); vesc_byte ('C'); vesc_byte ('S');
    vesc_byte ('-'); vesc_byte ('K'); vesc_byte ('T'); vesc_byte (0); // hardware name
    vesc_zeros (12 + 4);                                   // UUID (the STM8S105 has none), pairing, test, hw type, configs
    vesc_end ();
    break;
    case COMM_GET_VALUES:
    if (tx_free () < 79) { return; }
    vesc_values ();
    break;
    case COMM_SET_CURRENT:
    if (ui8_len >= 5)
    {
      i32_ma = (((int32_t) rx_peek (3)) << 24) | (((int32_t) rx_peek (4)) << 16) | (((uint16_t) rx_peek (5)) << 8) | rx_peek (6);
      if (i32_ma < 0) { i32_ma = 0; }                      // no regen over VESC commands
      if (i32_ma > 60000) { i32_ma = 60000; }
      ui16_vesc_current_10b = (uint16_t) (i32_ma / 125);   // ebike_app limits it to the motor current
      ui8_vesc_timeout = VESC_TIMEOUT;
    }
    break;
    case COMM_SET_CURRENT_BRAKE:
    case COMM_SET_DUTY:                                    // not supported: no drive
    ui16_vesc_current_10b = 0;
    ui8_vesc_timeout = VESC_TIMEOUT;
    break;
    case COMM_ALIVE:
    if (ui8_vesc_timeout) { ui8_vesc_timeout = VESC_TIMEOUT; }
    break;
    default:
    break;
  }
  rx_drop (ui8_len + 5);
}

uint16_t display_current_request (void)
{
  return ui8_vesc_timeout ? ui16_vesc_current_10b : 0xFFFF;
}

// ------------------------------------------------------------------------------------------- EBiCS (ANT+ LEV pages)
// 12 bytes: 164, 12, 0x4E, page, 7 data bytes, XOR of bytes 0 ... 10. The app / BLE module sends page 16 (wheel
// circumference mm low high, travel mode: assist level bits 3..5, display command), the controller sends
// page 1 (assist level, error, speed km/h x10) and page 4 (battery current A x100, voltage 0.25 V) in turn.
static uint8_t ui8_ebics_page = 1;

static void ebics_poll (void)
{
  while (rx_count () >= 2 && !((rx_peek (0) == 164) && (rx_peek (1) == 12))) { rx_drop (1); }
  if (rx_count () < 12) { return; }
  if (xor_sum (0, 11) != rx_peek (11)) { rx_drop (1); return; }
  if (rx_peek (3) == 16)
  {
    set_wheel_mm ((((uint16_t) rx_peek (5)) << 8) | rx_peek (4));
    set_assist_level ((rx_peek (6) >> 3) & 0x07);
  }
  rx_drop (12);
}

static void ebics_send_page (void)
{
  uint8_t ui8_buf[12];
  uint8_t ui8_i;
  uint16_t ui16_v;

  ui8_buf[0] = 164;
  ui8_buf[1] = 12;
  ui8_buf[2] = 0x4E;
  ui8_buf[3] = ui8_ebics_page;
  for (ui8_i = 4; ui8_i < 12; ui8_i++) { ui8_buf[ui8_i] = 0; }
  if (ui8_ebics_page == 1)
  {
    ui16_v = (f_wheel_speed < 0.1) ? 0 : (uint16_t) (f_wheel_speed * 10);
    if (ui16_v > 0x7FF) { ui16_v = 0x7FF; }
    ui8_buf[4] = 1;                                        // temperature state: no sensor
    ui8_buf[5] = (uint8_t) (lcd_configuration_variables.ui8_assist_level << 3);
    ui8_buf[8] = motor_controller_get_error ();
    ui8_buf[9] = (uint8_t) ui16_v;
    ui8_buf[10] = (uint8_t) (ui16_v >> 8);
    ui8_ebics_page = 4;
  }
  else
  {
    ui16_v = battery_current_x10 () * 10;                  // A x100, 12 bits
    if (ui16_v > 0x0FFF) { ui16_v = 0x0FFF; }
    ui8_buf[6] = (uint8_t) ((ui16_v >> 8) << 4);
    ui8_buf[7] = (uint8_t) ui16_v;
    ui8_buf[8] = battery_volts_x4 ();
    ui8_ebics_page = 1;
  }
  for (ui8_i = 0; ui8_i < 11; ui8_i++) { ui8_buf[11] ^= ui8_buf[ui8_i]; }
  for (ui8_i = 0; ui8_i < 12; ui8_i++) { tx (ui8_buf[ui8_i]); }
}

// ------------------------------------------------------------------------------------------- IONRISE
// The display is the master and sends every 100 ms; the protocol logic is in ionrise/ionctrl.c, here are its
// hooks. Settings: the display reads / writes a RAM copy of the 32 byte block (staging), APPLY checks it like
// the boot code, writes it into the data EEPROM (PERSIST) and restarts (REBOOT): all values are derived at boot.
#define ION_KT_VERSION 0x0100  // KT firmware version reported in HELLO

static uint8_t ui8_ion_active[SETTINGS_BLOCK_LENGTH];
static uint8_t ui8_ion_staging[SETTINGS_BLOCK_LENGTH];
uint8_t ui8_ion_assist = 0;               // assist demand of the display 0 ... 255, 0 until linked
static uint8_t ui8_ion_stop = 0;          // ACTION MOTOR_STOP: assist 0 until the demand goes back to 0
static uint8_t ui8_ion_reset = 0;         // reset once the answer is sent
static uint16_t ui16_ion_ms = 0;          // free running milliseconds (TIM2: 128 us steps)
static uint16_t ui16_ion_tim2 = 0;
static uint16_t ui16_ion_rand = 0x1D2B;

ionc_info_t ionc_info = { ION_FW_EBICS_KUNTENG, 0, ION_KT_VERSION, ION_LAYOUT_KT_V1, 1, SETTINGS_BLOCK_LENGTH,
                          ION_CAP_PERSIST | ION_CAP_HANDOFF | ION_CAP_HALL_DETECT, ui8_ion_active, ui8_ion_staging };

void ionc_port_send (const uint8_t *frame, uint8_t len)
{
  while (len--) { tx (*frame++); }
}

uint32_t ionc_port_random (void)
{
#ifdef SIMULATION
  return 0; // the simulator test computes the AUTH answer in advance (with the fixed clock below)
#endif
  ui16_ion_rand = (uint16_t) ((ui16_ion_rand * 75) + 74) ^ TIM2_GetCounter () ^ ((uint16_t) TIM1->CNTRL << 8);
  return ((uint32_t) ui16_ion_rand << 16) | (uint16_t) (ui16_ion_rand ^ TIM2_GetCounter () ^ ui16_ion_ms);
}

uint8_t ionc_port_source (void)
{
  return settings_state ();  // 0 compiled, 1 stored block, 2 stored block rejected = ION_SRC_*
}

uint8_t ionc_port_motor_stopped (void)
{
  return (ui16_motor_get_motor_speed_erps () == 0) && !(TIM1->BKR & TIM1_BKR_MOE);
}

void ionc_port_control (const uint8_t *req)
{
  uint16_t ui16_limit = ion_get16 (req + ION_TQ_SPEED_LIMIT);
  uint8_t ui8_pct = req[ION_TQ_POWER_PCT];
  if (!req[ION_TQ_ASSIST]) { ui8_ion_stop = 0; }
  ui8_ion_assist = ui8_ion_stop ? 0 : req[ION_TQ_ASSIST];
  lcd_configuration_variables.ui8_assist_level = req[ION_TQ_LEVEL]; // information (the demand is what counts)
  if (ui16_limit == 0) { ui16_limit = kt_settings.ui8_default_max_speed; }
  lcd_configuration_variables.ui8_max_speed = (ui16_limit > 255) ? 255 : (uint8_t) ui16_limit;
  ui8_display_power_pct = (ui8_pct > 100) ? 100 : ui8_pct;
  ui8_display_no_throttle = (req[ION_TQ_FLAGS] & ION_CF_NO_THROTTLE) ? 1 : 0;
}

void ionc_port_link_lost (void)
{
  ui8_ion_assist = 0;
}

void ionc_port_telemetry (uint8_t *r)
{
  uint8_t ui8_i;
  uint8_t ui8_e = motor_controller_get_error ();
  uint8_t ui8_state = motor_controller_get_state ();
  uint16_t ui16_status = 0;
  uint16_t ui16_errors = 0;
  uint16_t ui16_duty = ui16_duty_cycle;
  int16_t i16_current = motor_get_current_filtered_10b (); // 0.125 A
  int32_t i32_phase;
  float f_speed = f_wheel_speed * 100.0;

  for (ui8_i = 0; ui8_i < ION_TR_LEN; ui8_i++) { r[ui8_i] = 0; }
  ion_put16 (r + ION_TR_SPEED, (f_speed > 65534.0) ? 65534 : (uint16_t) f_speed);
  ion_put16 (r + ION_TR_VOLTAGE, (uint16_t) ((((uint32_t) motor_get_ADC_battery_voltage_filtered ()) * 272) / 10));
  ion_put16 (r + ION_TR_BATT_CURRENT, (uint16_t) ((i16_current * 10) / 8));
  // phase current from the battery current: battery = phase x duty cycle (the KT measures no phase current)
  i32_phase = ((int32_t) i16_current * 10 * 508) / 8 / ((ui16_duty < 64) ? 64 : ui16_duty);
  if (i32_phase > 32767) { i32_phase = 32767; }
  if (i32_phase < -32767) { i32_phase = -32767; }
  ion_put16 (r + ION_TR_PHASE_CURRENT, (uint16_t) i32_phase);
  r[ION_TR_MOTOR_TEMP] = 0x80;                             // no sensors
  r[ION_TR_CTRL_TEMP] = 0x80;
  r[ION_TR_CADENCE] = ui8_pas_cadence_rpm;
  if (kt_settings.ui8_throttle_type == EBIKE_THROTTLE_TYPE_TORQUE_SENSOR)
  {
    r[ION_TR_THROTTLE] = 255;
    ion_put16 (r + ION_TR_TORQUE, ui8_adc_throttle_value);
  }
  else
  {
    r[ION_TR_THROTTLE] = (uint8_t) (((uint16_t) ui8_throttle_value * 100) / 255);
    ion_put16 (r + ION_TR_TORQUE, 0xFFFF);
  }
  r[ION_TR_ASSIST_USED] = ui8_ion_assist;
  r[ION_TR_DUTY] = (uint8_t) (((uint32_t) ui16_duty * 100) / 508);

  if (TIM1->BKR & TIM1_BKR_MOE) { ui16_status |= ION_ST_MOTOR_ON; }
  if (ui8_state & MOTOR_CONTROLLER_STATE_BRAKE)
  {
    ui16_status |= ION_ST_BRAKE;
    if (kt_settings.ui8_regen_current_a) { ui16_status |= ION_ST_REGEN; }
  }
  if (!ebike_app_is_throttle_released ()) { ui16_status |= ION_ST_THROTTLE; }
  if (ui8_pas_cadence_rpm) { ui16_status |= ION_ST_PAS; }
  if (ebike_app_cruise_control_is_set ()) { ui16_status |= ION_ST_CRUISE; }
  if (ui8_fw_angle) { ui16_status |= ION_ST_FIELD_WEAK; }
  if (ionc_linked ()) { ui16_status |= ION_ST_LINKED; }
  if (ui8_ad_active) { ui16_status |= ION_ST_BUSY; }
  ion_put16 (r + ION_TR_STATUS, ui16_status);

  if (ui8_state & MOTOR_CONTROLLER_STATE_OVER_CURRENT) { ui16_errors |= ION_ER_OVERCURRENT; }
  if (ui8_state & MOTOR_CONTROLLER_STATE_UNDER_VOLTAGE) { ui16_errors |= ION_ER_UNDERVOLTAGE; }
  if (ui8_state & MOTOR_CONTROLLER_STATE_OVER_VOLTAGE) { ui16_errors |= ION_ER_OVERVOLTAGE; }
  if (ui8_state & MOTOR_CONTROLLER_STATE_HALL_ERROR) { ui16_errors |= ION_ER_HALL; }
  if (ui8_state & MOTOR_CONTROLLER_STATE_MOTOR_BLOCKED) { ui16_errors |= ION_ER_STALL; }
  if ((ui8_state & MOTOR_CONTROLLER_STATE_THROTTLE_ERROR) || (ui8_e == MOTOR_CONTROLLER_ERROR_01_THROTTLE)) { ui16_errors |= ION_ER_THROTTLE; }
  if (ui8_phase_fault) { ui16_errors |= ION_ER_PHASE; }
  if (settings_state () == 2) { ui16_errors |= ION_ER_SETTINGS; }
  ion_put16 (r + ION_TR_ERRORS, ui16_errors);
  // the KT codes are already the hex style codes of the displays (0x21 ...); under voltage = the battery symbol
  r[ION_TR_ERROR_CODE] = (ui8_e == MOTOR_CONTROLLER_ERROR_91_BATTERY_UNDER_VOLTAGE) ? 0 : ui8_e;
  if (!r[ION_TR_ERROR_CODE] && ui8_phase_fault) { r[ION_TR_ERROR_CODE] = 0x23; }
}

uint8_t ionc_port_apply (uint8_t mode)
{
  uint8_t ui8_i;
  if (mode & ION_APPLY_LIVE) { return ION_E_UNSUPPORTED; } // all values are derived at boot
  if (!settings_check (ui8_ion_staging)) { return ION_E_INVALID; }
  // writing the EEPROM blocks the main loop (current control) for up to 0.2 s: only with the motor stopped
  if ((mode & ION_APPLY_PERSIST) && !ionc_port_motor_stopped ()) { return ION_E_BUSY; }
  if (mode & ION_APPLY_PERSIST)
  {
    eeprom_write_bytes (SETTINGS_BLOCK_ADDRESS, ui8_ion_staging, SETTINGS_BLOCK_LENGTH);
    for (ui8_i = 0; ui8_i < SETTINGS_BLOCK_LENGTH; ui8_i++)
    {
      if (*((volatile uint8_t *) (SETTINGS_BLOCK_ADDRESS + ui8_i)) != ui8_ion_staging[ui8_i]) { return ION_E_FAILED; }
      ui8_ion_active[ui8_i] = ui8_ion_staging[ui8_i];
    }
  }
  if (mode & ION_APPLY_REBOOT) { ui8_ion_reset = 1; }
  return ION_OK;
}

uint8_t ionc_port_action (uint8_t action, const uint8_t *arg)
{
  (void) arg;
  switch (action)
  {
    case ION_ACT_REBOOT:
    if (!ionc_port_motor_stopped ()) { return ION_E_BUSY; }
    ui8_ion_reset = 1;
    return ION_OK;

    case ION_ACT_HALL_DETECT:
    if (!ionc_port_motor_stopped () || ui8_ad_active) { return ION_E_BUSY; }
    ui8_ad_cmd = 1;                                        // runs from the main loop (motor_autodetect)
    return ION_OK;

    case ION_ACT_CLEAR_ERRORS:
    motor_controller_clear_error ();
    return ION_OK;

    case ION_ACT_MOTOR_STOP:
    ui8_ion_stop = 1;
    ui8_ion_assist = 0;
    return ION_OK;
  }
  return ION_E_RANGE;
}

static void ionrise_poll (void)
{
  uint16_t ui16_now = TIM2_GetCounter ();
  uint16_t ui16_d = ui16_now - ui16_ion_tim2;
  ui16_ion_ms += ui16_d >> 3;                              // 8 x 128 us = 1.024 ms, close enough
#ifdef SIMULATION
  ui16_ion_ms = 0;
#endif
  ui16_ion_tim2 += ui16_d & 0xFFF8;
  while (rx_count ())
  {
    ionc_rx_byte (rx_peek (0));
    rx_drop (1);
    ionc_service (ui16_ion_ms); // a complete frame is answered at once, before the next one arrives
  }
  ionc_service (ui16_ion_ms);   // link supervision
  if (ui8_ion_reset && !ui8_tx_busy)
  {
    // the answer is out: restart with the new block (window watchdog: WDGA with T6 = 0 resets at once)
    WWDG->CR = 0x80;
  }
}

int16_t display_assist (void)
{
  return (kt_settings.ui8_display == DISPLAY_IONRISE) ? (int16_t) ui8_ion_assist : -1;
}

// ------------------------------------------------------------------------------------------- dispatch
uint32_t display_baudrate (void)
{
  if (kt_settings.ui8_display == DISPLAY_BAFANG_LCD) { return 1200; }
  if (kt_settings.ui8_display == DISPLAY_VESC) { return 115200; }
  return 9600;
}

void display_init (void)
{
  if (kt_settings.ui8_display != DISPLAY_KT) { use_default_lcd_values (); }
  if (kt_settings.ui8_display == DISPLAY_VESC) { lcd_configuration_variables.ui8_assist_level = 3; } // no levels from VESC devices
  if (kt_settings.ui8_display == DISPLAY_IONRISE)
  {
    settings_pack (ui8_ion_active);
    ionc_info.hw = kt_settings.ui8_controller_type;
    if (kt_settings.ui8_throttle_type == EBIKE_THROTTLE_TYPE_TORQUE_SENSOR) { ionc_info.caps |= ION_CAP_TORQUE; }
    else { ionc_info.caps |= ION_CAP_THROTTLE; }
    if (kt_settings.ui8_regen_current_a) { ionc_info.caps |= ION_CAP_REGEN; }
    if (kt_settings.ui8_fw_angle_deg) { ionc_info.caps |= ION_CAP_FIELD_WEAK; }
    ionc_init ();
    lcd_configuration_variables.ui8_assist_level = 0;
    // the wheel and the speed limit of the settings block (the IONRISE display sends the speed limit, the
    // controller computes the speed: the values a KT display left in the EEPROM do not count)
    lcd_configuration_variables.ui8_wheel_size = kt_settings.ui8_default_wheel_size;
    lcd_configuration_variables.ui8_max_speed = kt_settings.ui8_default_max_speed;
    ui16_ion_tim2 = TIM2_GetCounter ();
  }
  UART2->CR2 |= UART2_CR2_RIEN;
}

void display_poll (void)
{
  switch (kt_settings.ui8_display)
  {
    case DISPLAY_KINGMETER: km_poll (); break;
    case DISPLAY_BAFANG_LCD:
    case DISPLAY_BAFANG_TFT: bf_poll (); break;
    case DISPLAY_KM_618U: km618_poll (); break;
    case DISPLAY_NO2: no2_poll (); break;
    case DISPLAY_TONGSHENG: ts_poll (); break;
    case DISPLAY_VESC: vesc_poll (); break;
    case DISPLAY_EBICS: ebics_poll (); break;
    case DISPLAY_IONRISE: ionrise_poll (); break;
    default: kt_poll (); break;
  }
}

void display_controller (void)
{
  if (kt_settings.ui8_display == DISPLAY_KT) { kt_send_status (); } // the KT display expects it every 150 ms
  else if (kt_settings.ui8_display == DISPLAY_EBICS) { ebics_send_page (); }
  if (ui8_vesc_timeout) { ui8_vesc_timeout--; }
}
