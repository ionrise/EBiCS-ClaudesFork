/*
 * Display protocols of the KT firmware: Kunteng (KT-LCD3/LCD5), KingMeter 901U / KM5S and J-LCD / 618U,
 * Bafang LCD and 850C/860C, UART No.2, Tongsheng, VESC UART, EBiCS and IONRISE. Chosen at runtime with the settings
 * block (kt_settings.ui8_display).
 *
 * This file is part of the KT firmware, GPL v3.
 */

#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <stdint.h>

#define DISPLAY_KT          0   // Kunteng KT-LCD3 / LCD5 (9600 baud)
#define DISPLAY_KINGMETER   1   // KingMeter 901U / KM5S (9600 baud)
#define DISPLAY_BAFANG_LCD  2   // Bafang LCD (1200 baud)
#define DISPLAY_BAFANG_TFT  3   // Bafang 850C / 860C (9600 baud)
#define DISPLAY_KM_618U     4   // KingMeter J-LCD / 618U (9600 baud)
#define DISPLAY_NO2         5   // UART protocol No.2: S866, EN06 ... (9600 baud)
#define DISPLAY_TONGSHENG   6   // Tongsheng VLCD5 / VLCD6 / XH18 (9600 baud)
#define DISPLAY_VESC        7   // VESC UART: live values and current requests (115200 baud)
#define DISPLAY_EBICS       8   // EBiCS app / BLE module, ANT+ LEV pages (9600 baud)
#define DISPLAY_IONRISE     9   // IONRISE display (iondisp), ionrise/IONRISE_PROTOCOL.md (9600 baud)
#define DISPLAY_LAST        9

extern uint16_t ui16_display_wheel_perimeter_mm; // set by displays that send the wheel size in mm (0 = KT code)

uint32_t display_baudrate (void);
void display_init (void);        // after uart_init
void display_poll (void);        // as often as possible from the main loop: answers requests of the display
void display_controller (void);  // every 150 ms from ebike_app_controller
uint16_t display_current_request (void); // VESC current request (0.125 A), 0xFFFF = none
int16_t display_assist (void);           // IONRISE assist demand 0 ... 255, -1 = other displays (levels)
extern uint8_t ui8_display_power_pct;    // power limit of the display, % (100 = full)
extern uint8_t ui8_display_no_throttle;  // 1 = ignore the throttle

#endif /* _DISPLAY_H_ */
