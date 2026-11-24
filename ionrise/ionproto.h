/*
 * ionproto.h - IONRISE display <-> controller protocol, version 1.0
 *
 * Shared by the iondisp display firmware and the EBiCS controller firmwares
 * (Lishui STM32F103, Kunteng STM8S105). Plain C99, no dynamic memory, no
 * floating point, endian-neutral (all multi-byte fields are little endian on
 * the wire and are packed/unpacked byte by byte).
 *
 * Full specification: protocol/IONRISE_PROTOCOL.md
 *
 * Frame:  A5 | LEN | CMD | SEQ | PAYLOAD[0..64] | CRC16 lo | CRC16 hi
 *   LEN   = 2 + payload length (CMD + SEQ + PAYLOAD)
 *   CRC16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over LEN..PAYLOAD
 *   The display is the master; every request gets exactly one response with
 *   CMD | 0x80 and the same SEQ.
 */
#ifndef IONPROTO_H
#define IONPROTO_H

#include <stdint.h>

#define ION_PROTO_MAJOR      1
#define ION_PROTO_MINOR      0

#define ION_BAUD_DEFAULT     9600u

/* ---------------- framing ---------------- */
#define ION_SOF              0xA5u
#define ION_MAX_PAYLOAD      64u
#define ION_MAX_FRAME        (ION_MAX_PAYLOAD + 6u)
#define ION_RESP             0x80u          /* response CMD = request CMD | ION_RESP */

/* ---------------- commands ---------------- */
#define ION_CMD_HELLO        0x01u   /* handshake step 1 */
#define ION_CMD_AUTH         0x02u   /* handshake step 2 */
#define ION_CMD_TELEMETRY    0x10u   /* control in, live values out (10 Hz) */
#define ION_CMD_SET_READ     0x20u   /* read settings bytes (active or staging) */
#define ION_CMD_SET_WRITE    0x21u   /* write settings bytes into the staging copy */
#define ION_CMD_SET_APPLY    0x22u   /* validate staging and use / save / reboot with it */
#define ION_CMD_ACTION       0x30u   /* reboot, hall autodetect, clear errors ... */
#define ION_CMD_DIAG         0x31u   /* optional: firmware specific raw values */

/* ---------------- status codes (first payload byte of most responses) ---------------- */
#define ION_OK               0x00u
#define ION_E_LENGTH         0x01u   /* payload length wrong for this command */
#define ION_E_RANGE          0x02u   /* offset/length outside the settings block, bad argument */
#define ION_E_NOT_LINKED     0x03u   /* handshake not done (or controller rebooted) */
#define ION_E_BUSY           0x04u   /* motor running / action in progress */
#define ION_E_INVALID        0x05u   /* staging block failed the firmware's own check */
#define ION_E_UNSUPPORTED    0x06u   /* command or mode not implemented by this firmware */
#define ION_E_FAILED         0x07u   /* write to non-volatile memory failed */
#define ION_E_AUTH           0x08u   /* handshake proof wrong */

/* ---------------- identities ---------------- */
#define ION_DISPLAY_IONDISP_LCD8H   0x01u   /* iondisp on Kunteng KT-LCD8H hardware */

#define ION_FW_EBICS_LISHUI  0x01u   /* EBiCS FOC, Lishui STM32F103C6 */
#define ION_FW_EBICS_KUNTENG 0x02u   /* EBiCS KT firmware, Kunteng STM8S105 */

#define ION_LAYOUT_EBICS_V1  0x01u   /* ebics_settings_t, 256 bytes (Inc/settings.h) */
#define ION_LAYOUT_KT_V1     0x02u   /* KT settings block, 32 bytes (kunteng/settings.h) */

/* settings source reported in HELLO */
#define ION_SRC_COMPILED     0x00u   /* config.h / main.h defaults */
#define ION_SRC_FLASH        0x01u   /* controller's own stored block */
#define ION_SRC_REJECTED     0x02u   /* stored block rejected, defaults used */
#define ION_SRC_DISPLAY      0x03u   /* block pushed by the display (live or RAM hand-off) */

/* capability bits (HELLO response) */
#define ION_CAP_LIVE_APPLY   (1u << 0)   /* SET_APPLY mode LIVE */
#define ION_CAP_PERSIST      (1u << 1)   /* SET_APPLY mode PERSIST */
#define ION_CAP_HANDOFF      (1u << 2)   /* SET_APPLY mode REBOOT (RAM hand-off) */
#define ION_CAP_DIAG         (1u << 3)   /* ION_CMD_DIAG */
#define ION_CAP_HALL_DETECT  (1u << 4)   /* ACTION hall autodetect */
#define ION_CAP_THROTTLE     (1u << 5)   /* throttle input present */
#define ION_CAP_TORQUE       (1u << 6)   /* torque sensor input present */
#define ION_CAP_SENSORLESS   (1u << 7)   /* runs sensorless */
#define ION_CAP_REGEN        (1u << 8)   /* regenerative braking */
#define ION_CAP_FIELD_WEAK   (1u << 9)   /* field weakening */

/* handshake key (constant; proves both ends speak IONRISE, it is not encryption) */
#define ION_KEY              "IONRISE"
#define ION_KEY_LEN          7u

/* ---------------- TELEMETRY ---------------- */
/* request payload (display -> controller), 8 bytes */
#define ION_TQ_ASSIST        0   /* u8  assist demand 0..255 (0 = no assist) */
#define ION_TQ_LEVEL         1   /* u8  level shown on the display 0..9 */
#define ION_TQ_LEVELS        2   /* u8  number of levels configured 3..9 */
#define ION_TQ_FLAGS         3   /* u8  ION_CF_* */
#define ION_TQ_SPEED_LIMIT   4   /* u16 km/h, 0 = controller setting, 0xFFFF = no limit */
#define ION_TQ_POWER_PCT     6   /* u8  power limit 0..100 %, 100 = full */
#define ION_TQ_RESERVED      7   /* u8  0 */
#define ION_TQ_LEN           8

#define ION_CF_LIGHT         (1u << 0)
#define ION_CF_WALK          (1u << 1)   /* walk assist while set */
#define ION_CF_CRUISE        (1u << 2)   /* cruise control request */
#define ION_CF_LEGAL         (1u << 3)   /* legal (street) mode requested */
#define ION_CF_NO_THROTTLE   (1u << 4)   /* ignore the throttle */
#define ION_CF_REGEN         (1u << 5)   /* regen request from the display (e.g. brake button) */

/* response payload (controller -> display), 22 bytes */
#define ION_TR_SPEED         0   /* u16 0.01 km/h (0..655.34), 0xFFFF = unknown */
#define ION_TR_VOLTAGE       2   /* u16 0.01 V battery */
#define ION_TR_BATT_CURRENT  4   /* i16 0.1 A battery, negative = regen */
#define ION_TR_PHASE_CURRENT 6   /* i16 0.1 A phase (Iq) */
#define ION_TR_MOTOR_TEMP    8   /* i8  deg C, -128 = no sensor */
#define ION_TR_CTRL_TEMP     9   /* i8  deg C, -128 = unknown */
#define ION_TR_CADENCE       10  /* u8  rpm, 255 = unknown */
#define ION_TR_THROTTLE      11  /* u8  0..100 %, 255 = no throttle */
#define ION_TR_TORQUE        12  /* u16 torque sensor raw ADC, 0xFFFF = none */
#define ION_TR_ASSIST_USED   14  /* u8  assist 0..255 actually applied after limits */
#define ION_TR_DUTY          15  /* u8  PWM duty 0..100 % */
#define ION_TR_STATUS        16  /* u16 ION_ST_* */
#define ION_TR_ERRORS        18  /* u16 ION_ER_* */
#define ION_TR_ERROR_CODE    20  /* u8  legacy display code (hex style, e.g. 0x25), 0 = none */
#define ION_TR_RESERVED      21  /* u8  0 */
#define ION_TR_LEN           22

#define ION_ST_MOTOR_ON      (1u << 0)
#define ION_ST_BRAKE         (1u << 1)
#define ION_ST_THROTTLE      (1u << 2)
#define ION_ST_PAS           (1u << 3)
#define ION_ST_WALK          (1u << 4)
#define ION_ST_CRUISE        (1u << 5)
#define ION_ST_REGEN         (1u << 6)
#define ION_ST_SPEED_LIMIT   (1u << 7)
#define ION_ST_CURRENT_LIMIT (1u << 8)
#define ION_ST_TEMP_DERATE   (1u << 9)
#define ION_ST_LOW_VOLTAGE   (1u << 10)
#define ION_ST_FIELD_WEAK    (1u << 11)
#define ION_ST_LINKED        (1u << 12)
#define ION_ST_DISPLAY_SET   (1u << 13)   /* settings pushed by the display are in use */
#define ION_ST_SENSORLESS    (1u << 14)
#define ION_ST_BUSY          (1u << 15)   /* action running (e.g. hall autodetect) */

#define ION_ER_OVERCURRENT   (1u << 0)
#define ION_ER_UNDERVOLTAGE  (1u << 1)
#define ION_ER_OVERVOLTAGE   (1u << 2)
#define ION_ER_MOTOR_TEMP    (1u << 3)
#define ION_ER_CTRL_TEMP     (1u << 4)
#define ION_ER_HALL          (1u << 5)
#define ION_ER_PHASE         (1u << 6)
#define ION_ER_THROTTLE      (1u << 7)
#define ION_ER_BRAKE         (1u << 8)
#define ION_ER_STALL         (1u << 9)
#define ION_ER_TORQUE        (1u << 10)
#define ION_ER_COMM          (1u << 11)   /* controller saw the display time out */
#define ION_ER_SETTINGS      (1u << 12)   /* stored settings rejected */
#define ION_ER_SPEED_SENSOR  (1u << 13)

/* ---------------- settings ---------------- */
#define ION_SPACE_ACTIVE     0u
#define ION_SPACE_STAGING    1u

#define ION_APPLY_LIVE       (1u << 0)   /* use the staging block now */
#define ION_APPLY_PERSIST    (1u << 1)   /* store it in the controller's non-volatile memory */
#define ION_APPLY_REBOOT     (1u << 2)   /* reboot and start with it (motor must be stopped) */

/* ---------------- actions ---------------- */
#define ION_ACT_REBOOT       0x01u   /* motor stopped only */
#define ION_ACT_HALL_DETECT  0x02u   /* motor stopped, wheel lifted; progress in ION_ST_BUSY */
#define ION_ACT_CLEAR_ERRORS 0x03u
#define ION_ACT_MOTOR_STOP   0x04u   /* PWM off until assist goes back to 0 */
#define ION_ACT_REVERT       0x05u   /* staging = active */

/* ---------------- codec API (ionproto.c) ---------------- */
uint16_t ion_crc16(uint16_t crc, const uint8_t *p, uint16_t n);

/* builds a frame into out (>= ION_MAX_FRAME bytes), returns the frame length */
uint8_t ion_build(uint8_t *out, uint8_t cmd, uint8_t seq, const uint8_t *payload, uint8_t len);

/* byte-at-a-time receiver with resynchronisation */
typedef struct {
    uint8_t buf[ION_MAX_FRAME];
    uint8_t pos;
} ion_rx_t;

/* feed one received byte; returns 1 when a complete, CRC-correct frame is in
 * rx->buf (cmd = buf[2], seq = buf[3], payload = buf + 4, length = buf[1] - 2) */
uint8_t ion_rx_byte(ion_rx_t *rx, uint8_t b);

#define ION_RX_CMD(rx)      ((rx)->buf[2])
#define ION_RX_SEQ(rx)      ((rx)->buf[3])
#define ION_RX_PAYLOAD(rx)  ((rx)->buf + 4)
#define ION_RX_LEN(rx)      ((uint8_t)((rx)->buf[1] - 2u))

/* little endian helpers */
static inline uint16_t ion_get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t ion_get32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline void ion_put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void ion_put32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

/* handshake proof: crc16 over nonce (LE) | id byte | ION_KEY */
uint16_t ion_proof(uint32_t nonce, uint8_t id);

#endif
