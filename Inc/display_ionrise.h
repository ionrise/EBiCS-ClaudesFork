/*
 * display_ionrise.h - IONRISE display protocol (iondisp), EBiCS Lishui port.
 *
 * The protocol itself (frames, CRC, handshake, settings staging) is in ionrise/ionproto.c and
 * ionrise/ionctrl.c, shared with the iondisp display and the EBiCS Kunteng firmware. Specification:
 * ionrise/IONRISE_PROTOCOL.md. This file is the Lishui side: UART, telemetry values, control values,
 * settings apply (live, or RAM hand-off and reset).
 *
 * This file is part of EBiCS, GPL v3 or later.
 */
#ifndef DISPLAY_IONRISE_H_
#define DISPLAY_IONRISE_H_

#include <stdint.h>

#define ION_RX_BUFF   64      // DMA ring buffer, power of 2

typedef struct {
	uint8_t power_pct;        // power limit of the display, % of the current limits
	uint8_t walk;             // walk assist while set
	uint8_t stop;             // ACTION MOTOR_STOP: assist 0 until the display's demand goes back to 0
	uint8_t reset;            // reset (RAM hand-off) once the answer is sent
} ION_t;

extern ION_t ION;

void Ionrise_Init(void);
void Ionrise_Service(void);   // every main loop pass: receive, answer, link supervision

// RAM hand-off of a settings block through a software reset (settings.c checks it at boot)
#define ION_HANDOFF_MAGIC 0x494F4E48UL   // "IONH"
typedef struct {
	uint32_t magic;
	uint32_t crc;             // CRC-32 of block
	uint8_t block[256];
} ion_handoff_t;
extern ion_handoff_t ion_handoff;  // .noinit: kept through the reset

#endif /* DISPLAY_IONRISE_H_ */
