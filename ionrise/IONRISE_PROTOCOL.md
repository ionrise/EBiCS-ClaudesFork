# IONRISE display protocol, version 1.0

A serial protocol between the **iondisp** display (Kunteng KT-LCD8H hardware) and the **EBiCS** controller
firmwares: Lishui (STM32F103C6, 32 KB) and Kunteng (STM8S105, 32 KB). It replaces the borrowed display protocols
(KM5S, KT-LCD3 ...) with one that is designed for this pair:

- **Handshake** before the controller accepts any command: both sides prove they speak IONRISE.
- **Full live telemetry**: speed to 655 km/h at 0.01 km/h resolution, voltage, battery and phase current,
  temperatures, cadence, throttle, torque, status and error bits.
- **Real firmware settings**: the display reads and writes the controller's own settings block byte for byte, with
  descriptions of every field (name, unit, scaling, range) kept in the display, not in the controller.
- **3 to 9 PAS levels**, configured on the display, sent as a 0–255 assist demand.
- **Small controller side**: about 1 KB of flash for the protocol core on Cortex-M3, so it fits into the 32 KB Lishui
  in place of the KM5S driver.

Files in this folder:

| File | What it is |
|---|---|
| `ionproto.h` | Shared constants, payload offsets, codec API (display and controller) |
| `ionproto.c` | Codec: CRC-16, frame builder, resynchronising receiver, handshake proof |
| `controller/ionctrl.h/.c` | Controller side: handshake, link supervision, settings staging, dispatch to port hooks |
| `ionproto.py` | Python reference (same algorithms), self-test and the example frames of this document |

---

## 1. Physical layer

| Item | Value |
|---|---|
| Interface | UART, the display cable's TX/RX lines (same wiring as the KT / KM5S protocols) |
| Format | 9600 baud, 8 data bits, no parity, 1 stop bit |
| Roles | The display is the master. The controller only answers; it never sends on its own. |
| Byte time | 1.04 ms at 9600 baud |

The baud rate is fixed at 9600 in version 1: the cable to the handlebar is long and passes level shifters on both
boards. A faster rate can be added as a capability in a later minor version.

## 2. Frame format

```
 offset  0     1     2     3     4 ...           LEN+2      LEN+3
       +-----+-----+-----+-----+---------------+----------+----------+
       | A5  | LEN | CMD | SEQ | PAYLOAD 0..64 | CRC lo   | CRC hi   |
       +-----+-----+-----+-----+---------------+----------+----------+
```

| Field | Size | Meaning |
|---|---|---|
| SOF | 1 | Always `0xA5` |
| LEN | 1 | Number of bytes CMD + SEQ + PAYLOAD = payload length + 2. Valid: 2 … 66 |
| CMD | 1 | Command. Responses use `CMD | 0x80` |
| SEQ | 1 | Sequence number chosen by the display (incremented per request); the response repeats it |
| PAYLOAD | 0–64 | Command specific, all multi-byte values little endian |
| CRC | 2 | CRC-16/CCITT-FALSE over LEN … end of PAYLOAD, low byte first |

**CRC-16/CCITT-FALSE**: polynomial `0x1021`, initial value `0xFFFF`, no reflection, no final XOR. Check value:
the ASCII string `123456789` gives `0x29B1`.

Frame length = payload + 6, at most 70 bytes (73 ms at 9600 baud).

**Receiving / resynchronisation** (`ion_rx_byte`): wait for `A5`; if LEN is out of range or the CRC is wrong, drop
the first byte and rescan the bytes already received for the next `A5`. A `0xA5` inside a payload therefore never
blocks the next real frame. A frame is accepted only with a correct CRC.

## 3. Session and timing

```
  display                                   controller
     | HELLO (nonce_d) ------------------------> |
     | <------------------ HELLO resp (proof_c, nonce_c, identity, layout, caps)
     | AUTH (proof_d) -------------------------> |   linked = 1, staging = active
     | <----------------------------- AUTH resp (OK)
     | [settings sync: SET_READ / SET_WRITE / SET_APPLY]
     | TELEMETRY (assist, level, flags ...) ---> |   every 100 ms
     | <------------------------ TELEMETRY resp (speed, voltage, current, status ...)
     |   ...                                     |
```

| Rule | Value |
|---|---|
| Controller response time | starts sending within **20 ms** after the last byte of the request |
| Display response timeout | (request bytes + expected response bytes) × 1.05 ms + **30 ms** |
| TELEMETRY rate | every **100 ms** (10 Hz). Other requests are slotted between two telemetry exchanges. |
| Telemetry lost, controller | no valid TELEMETRY for **500 ms** → assist 0 and walk off **at once**. The session stays: the next valid TELEMETRY resumes control. Only HELLO or a controller reboot ends the session. |
| Telemetry lost, display | no valid TELEMETRY for **0.7 s** → shows **E30**, keeps requesting telemetry. A `NOT_LINKED` answer (controller rebooted) starts a new handshake. |
| Handshake / sync retries | a lost answer during AUTH / SET_READ / SET_WRITE / SET_APPLY is retried up to **6 times** before starting over with HELLO. SET_READ / SET_WRITE halve their chunk size after each failure (60 → 8 bytes) and grow it again after successes. |
| Reconnect | with a stored profile the display first reads only the block's integrity value (CRC-32 at 252, KT checksum at 31); the full block is read only when it differs. |

Bus load at 10 Hz: telemetry request 14 bytes + response 28 bytes = 42 ms per 100 ms.

## 4. Handshake

The handshake proves that both ends run IONRISE before the controller lets the display control the motor or touch
settings. It is **not** encryption; the key is public (`IONRISE`). It protects against a controller misreading another
display protocol and against random line noise starting the motor.

```
proof(nonce, id) = CRC16( CRC16(0xFFFF, nonce[4 bytes LE] | id[1 byte]),  "IONRISE" )
```
(CRC16 continued over the 7 ASCII bytes of the key, no terminator.)

### HELLO `0x01`

Request (display → controller), 10 bytes (controllers must accept ≥ 8 and ignore extra bytes):

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | protocol major (1) |
| 1 | u8 | protocol minor (0) |
| 2 | u8 | display id: `0x01` = iondisp on KT-LCD8H |
| 3 | u8 | reserved, 0 |
| 4 | u32 | nonce_d: random number from the display |
| 8 | u16 | display firmware version (major << 8 \| minor) |

Response `0x81`, 20 bytes. HELLO always resets the link (`linked = 0`).

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | protocol major |
| 1 | u8 | protocol minor |
| 2 | u8 | firmware family: `0x01` EBiCS Lishui, `0x02` EBiCS Kunteng |
| 3 | u8 | hardware variant (firmware defined, 0 = unknown; Kunteng: 1 = S06S, 2 = S12S) |
| 4 | u16 | firmware version (major << 8 \| minor) |
| 6 | u8 | settings layout id: `0x01` EBiCS settings v1 (256 B), `0x02` KT settings v1 (32 B) |
| 7 | u8 | settings layout version |
| 8 | u16 | settings block size in bytes |
| 10 | u8 | settings source: 0 compiled defaults, 1 controller's stored block, 2 stored block rejected, 3 pushed by the display |
| 11 | u8 | reserved, 0 |
| 12 | u16 | capabilities (table below) |
| 14 | u16 | proof_c = proof(nonce_d, firmware family) |
| 16 | u32 | nonce_c: random number from the controller |

The display checks proof_c. If it is wrong, it does not continue (wrong protocol or corrupted line).

| Capability bit | Name | Meaning |
|---|---|---|
| 0 | LIVE_APPLY | SET_APPLY mode LIVE works |
| 1 | PERSIST | SET_APPLY mode PERSIST works (controller can store settings itself) |
| 2 | HANDOFF | SET_APPLY mode REBOOT works (RAM hand-off, section 7.4) |
| 3 | DIAG | DIAG command implemented |
| 4 | HALL_DETECT | ACTION hall autodetect implemented |
| 5 | THROTTLE | a throttle is configured |
| 6 | TORQUE | a torque sensor is configured |
| 7 | SENSORLESS | motor runs sensorless |
| 8 | REGEN | regenerative braking configured |
| 9 | FIELD_WEAK | field weakening configured |
| 10–15 | | reserved, 0 |

### AUTH `0x02`

Request, 3 bytes:

| Offset | Type | Field |
|---|---|---|
| 0 | u16 | proof_d = proof(nonce_c, display id) |
| 2 | u8 | display id (same as in HELLO) |

Response `0x82`, 1 byte: status (`0x00` OK, `0x08` wrong proof). On OK the controller sets `linked = 1` and copies its
active settings block into the staging buffer.

Before AUTH succeeds, every other command is answered with status `0x03` NOT_LINKED, and the controller keeps assist
at 0.

## 5. Commands

### Status codes

Most responses start with a status byte:

| Code | Name | Meaning |
|---|---|---|
| 0x00 | OK | |
| 0x01 | LENGTH | payload length wrong for this command |
| 0x02 | RANGE | offset/length outside the settings block, or bad argument |
| 0x03 | NOT_LINKED | handshake not done, or the controller rebooted |
| 0x04 | BUSY | motor running (for actions that need it stopped) or an action is in progress |
| 0x05 | INVALID | staging block failed the firmware's own validity check |
| 0x06 | UNSUPPORTED | command or mode not implemented by this firmware |
| 0x07 | FAILED | writing non-volatile memory failed |
| 0x08 | AUTH | handshake proof wrong |

### TELEMETRY `0x10`

The display's control values go in, the live values come out. Sent every 100 ms.

Request, 8 bytes:

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | assist demand 0–255 (0 = no assist). Computed by the display from its level table (section 6). |
| 1 | u8 | level shown on the display, 0–9 (information for the controller, e.g. logging) |
| 2 | u8 | number of levels configured on the display, 3–9 |
| 3 | u8 | control flags (below) |
| 4 | u16 | speed limit in km/h: 0 = controller's own setting, 0xFFFF = no limit, else the limit (up to 65534) |
| 6 | u8 | power limit 0–100 % of the controller's current limits (100 = full) |
| 7 | u8 | reserved, 0 |

| Control flag bit | Name | Meaning |
|---|---|---|
| 0 | LIGHT | light on |
| 1 | WALK | walk assist, while set |
| 2 | CRUISE | cruise control requested |
| 3 | LEGAL | street-legal mode requested (controller applies its legal limits) |
| 4 | NO_THROTTLE | ignore the throttle |
| 5 | REGEN | regen request from the display (e.g. a brake button) |
| 6–7 | | reserved, 0 |

Response `0x90`, 22 bytes, **no status byte** (a NOT_LINKED controller answers with the 1-byte status `0x03` instead,
so the display tells the two apart by the length):

| Offset | Type | Unit | Field |
|---|---|---|---|
| 0 | u16 | 0.01 km/h | speed, 0–655.34 km/h; `0xFFFF` = unknown |
| 2 | u16 | 0.01 V | battery voltage (0–655.35 V) |
| 4 | i16 | 0.1 A | battery current, negative while regenerating |
| 6 | i16 | 0.1 A | phase current (Iq) |
| 8 | i8 | °C | motor temperature, -128 = no sensor |
| 9 | i8 | °C | controller temperature, -128 = unknown |
| 10 | u8 | rpm | pedal cadence, 255 = unknown |
| 11 | u8 | % | throttle position 0–100, 255 = no throttle |
| 12 | u16 | ADC | torque sensor raw value, `0xFFFF` = no sensor |
| 14 | u8 | 0–255 | assist actually applied after all limits |
| 15 | u8 | % | PWM duty cycle 0–100 |
| 16 | u16 | | status bits |
| 18 | u16 | | error bits |
| 20 | u8 | | legacy error code for the display, 0 = none (section 5.1) |
| 21 | u8 | | reserved, 0 |

The **controller computes the speed** from its own wheel circumference and speed source (hall, external sensor,
sensorless). 0.01 km/h steps up to 655 km/h cover very fast vehicles; the display shows up to 999.9.

| Status bit | Name | | Error bit | Name |
|---|---|---|---|---|
| 0 | MOTOR_ON (PWM enabled) | | 0 | OVERCURRENT |
| 1 | BRAKE | | 1 | UNDERVOLTAGE |
| 2 | THROTTLE active | | 2 | OVERVOLTAGE |
| 3 | PAS active (pedalling) | | 3 | MOTOR_TEMP (over temperature) |
| 4 | WALK active | | 4 | CTRL_TEMP |
| 5 | CRUISE active | | 5 | HALL sensor fault |
| 6 | REGEN | | 6 | PHASE (motor phase loss) |
| 7 | SPEED_LIMIT reached | | 7 | THROTTLE fault (e.g. not at zero at power up) |
| 8 | CURRENT_LIMIT reached | | 8 | BRAKE fault (brake active at power up) |
| 9 | TEMP_DERATE | | 9 | STALL (motor blocked) |
| 10 | LOW_VOLTAGE derate | | 10 | TORQUE sensor fault |
| 11 | FIELD_WEAK active | | 11 | COMM (controller saw the display time out) |
| 12 | LINKED | | 12 | SETTINGS (stored block rejected) |
| 13 | DISPLAY_SET (display's settings in use) | | 13 | SPEED_SENSOR fault |
| 14 | SENSORLESS | | 14–15 | reserved |
| 15 | BUSY (action running) | | | |

#### 5.1 Legacy error codes

Byte 20 lets controllers report the familiar numeric codes. The display prints the byte as two hex digits ("E25" for
`0x25`):

| Code | Meaning (KingMeter list) | | Code | Meaning (EBiCS) |
|---|---|---|---|---|
| 0x21 | abnormal current | | 0x25 | motor over temperature |
| 0x22 | throttle fault | | 0x26 | controller over temperature |
| 0x23 | motor phase loss | | 0x30 | communication |
| 0x24 | hall sensor fault | | | |
| 0x25 | brake fault (KingMeter) / motor over temperature (EBiCS) | | | |

The error bits are the authoritative information; the code is for compact display.

### SET_READ `0x20`

Request, 4 bytes: `space u8` (0 = active block, 1 = staging block), `offset u16`, `count u8` (1–61).
Response `0xA0`: `status u8`, `offset u16`, `data[count]`.

### SET_WRITE `0x21`

Request: `offset u16`, `data[1..62]`. Writes into the **staging** block only; nothing changes in the running firmware.
Response `0xA1`: `status u8`.

### SET_APPLY `0x22`

Request, 1 byte: mode bits. The controller validates the whole staging block with **its own** check (the same check it
uses at boot: magic, version, size, CRC / checksum, plausibility), then does what the bits ask:

| Bit | Mode | Meaning | Needs |
|---|---|---|---|
| 0 | LIVE | use the staging block from now on | cap LIVE_APPLY |
| 1 | PERSIST | store it in the controller's non-volatile memory | cap PERSIST |
| 2 | REBOOT | reboot and start with it (RAM hand-off, section 7.4) | cap HANDOFF, motor stopped |

Response `0xA2`: `status u8`. INVALID (`0x05`) leaves everything unchanged. REBOOT answers first, then resets after the
response has been sent; the display sees NOT_LINKED afterwards and runs the handshake again.

**The display must keep the block's own integrity fields correct** before APPLY: for the EBiCS block it recomputes
the CRC-32 at offset 252; for the KT block the checksum at offset 31.

### ACTION `0x30`

Request, 4 bytes: `action u8`, `argument[3]` (0 unless stated). Response `0xB0`: `status u8`.

| Action | Name | Conditions |
|---|---|---|
| 0x01 | REBOOT | motor stopped |
| 0x02 | HALL_DETECT | motor stopped, wheel lifted; runs in the background, `BUSY` status bit while running; cap HALL_DETECT |
| 0x03 | CLEAR_ERRORS | |
| 0x04 | MOTOR_STOP | PWM off immediately, stays off until the assist demand returns to 0 |
| 0x05 | REVERT | staging = active (discard unapplied writes) |

### DIAG `0x31` (optional, cap DIAG)

Request: `page u8`. Response `0xB1`: `status u8`, `page u8`, then up to 30 × i16 raw values defined per firmware
family (ADC throttle, torque, hall state, electrical speed, Id, Iq, Ud, Uq, raw temperatures …). Version 1 of the
Lishui integration does not implement it (flash budget); the display shows the page only when the cap is set.

## 6. PAS levels (3 to 9)

The display owns the level table, so the number of levels is free from 3 to 9 and does not depend on the controller.

- The rider chooses the number of levels (3–9) in the display menu; level 0 is always "off".
- Each level has a percentage 0–100 %. The display sends `assist = round(pct × 255 / 100)` in TELEMETRY byte 0, and
  the shown level in byte 1.
- The controller scales its current limit with `assist / 255` (EBiCS already has this path: the KM5S protocol also
  sends 0–255).

Default tables (from the KingMeter KM5S manual, Appendix 3; 4, 6 and 8 levels interpolated):

| Levels | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---|---|---|---|---|---|---|---|---|---|
| 3 | 47 | 72 | 92 | | | | | | |
| 4 | 42 | 60 | 78 | 94 | | | | | |
| 5 | 40 | 55 | 70 | 85 | 95 | | | | |
| 6 | 37 | 50 | 63 | 76 | 88 | 96 | | | |
| 7 | 35 | 46 | 57 | 68 | 79 | 90 | 97 | | |
| 8 | 30 | 40 | 50 | 60 | 70 | 80 | 90 | 97 | |
| 9 | 25 | 34 | 43 | 52 | 61 | 70 | 79 | 88 | 96 |

Walk assist: flag WALK; the controller uses its own walk current setting.

## 7. Settings model

### 7.1 Who owns the settings

**The display owns the settings profile.** The 32 KB Lishui has no room to rewrite its settings block in flash: the
block sits in flash page 29 together with program code, and the last 2 KB are the emulated EEPROM for hall angles
and KV. So:

1. The display stores one profile per controller family and layout (`fw_family`, `layout_id`, `layout_ver`) in its
   own flash.
2. After the handshake the display compares its profile with the controller's active block (read the integrity field
   first: CRC-32 at offset 252 for EBiCS, checksum byte 31 for KT; read the whole block only if it differs).
3. **No profile yet** → the display reads the whole active block and stores it as the profile (import).
4. **Profile differs** → the display writes the changed byte ranges with SET_WRITE and applies them (7.3).
5. A controller without an IONRISE display keeps working exactly as before with its own stored block or compiled
   defaults.

### 7.2 Staging

All writes go into a staging copy of the block in the controller's RAM (made at AUTH, or by ACTION REVERT). APPLY
validates the complete staging block before anything is used, so a half-written block can never reach the motor
control.

### 7.3 Applying changes

Each field in the display's descriptor table is marked **live** or **reboot**:

- Only live fields changed → `SET_APPLY LIVE` (Lishui) / `PERSIST` (Kunteng, which re-reads at boot: `PERSIST|REBOOT`).
- Any reboot field changed → `SET_APPLY REBOOT` (plus `PERSIST` where supported). Only accepted while the motor is
  stopped; the display asks the rider to stop first.

| Firmware | LIVE | PERSIST | REBOOT (hand-off) |
|---|---|---|---|
| EBiCS Lishui | yes (RS is used directly) | no (no free flash page) | yes, RAM hand-off |
| EBiCS Kunteng | no (values derived at start-up) | yes, data EEPROM 0x4080 | yes (plain reset after PERSIST) |

### 7.4 RAM hand-off reboot (Lishui)

The STM32 keeps its RAM through a software reset. The controller copies the validated staging block into a
`.noinit` RAM area with a header, then calls `NVIC_SystemReset()`:

```
struct { uint32_t magic = 0x494F4E48 ("IONH"); uint32_t crc32_of_block; uint8_t block[256]; } handoff;  // .noinit
```

`settings_load()` checks the hand-off area first: correct magic and CRC → use it (source = 3, DISPLAY), then clear
the magic so a later power cycle falls back to the stored block. After a power-off the display pushes its profile
again (7.1), so the bike always runs with the display's settings when an IONRISE display is connected.

## 8. Settings layouts

The display needs these tables to show and edit the settings. Columns: offset, C type, field, unit in the block,
how the display shows it, typical range, apply (L = live, R = reboot). "Units" = internal current units of EBiCS
Lishui: 1 unit = `CAL_I` = 38 mA (default calibration). "Ticks" = TIM3 interrupts at 64 MHz / 7814 ≈ 8.19 kHz,
1 tick ≈ 0.122 ms.

### 8.1 Layout 0x01: EBiCS settings v1 (Lishui), 256 bytes, little endian

Source: `Inc/settings.h` (`ebics_settings_t`), `setup_app/ebics_core.py` (`SETTINGS_FIELDS`). The block is valid
when magic, version and size match and the CRC-32 (IEEE, as zlib) of bytes 0–251 equals the value at 252.

| Off | Type | Field | Block unit | Display | Range | Apply |
|---|---|---|---|---|---|---|
| 0 | u32 | magic | `0xEB1C5E77` | hidden | fixed | – |
| 4 | u16 | version | 1 | hidden | fixed | – |
| 6 | u16 | size | 256 | hidden | fixed | – |
| 8 | u32 | flags | bits below | on/off per bit | | see bits |
| 12 | i32 | ph_current_max | units (38 mA) | A, 0.1 | 5–60 A | L |
| 16 | i32 | battery_current_max | mA | A, 0.1 | 3–60 A | L |
| 20 | i32 | regen_current | units | A, 0.1 | 0–30 A | L |
| 24 | i32 | regen_current_max | mA | A, 0.1 | 0–30 A | L |
| 28 | i32 | battery_voltage_min | mV | V, 0.1 (cutoff) | 18–90 V | L |
| 32 | i32 | battery_voltage_max | mV | V, 0.1 (full) | 20–100 V | L |
| 36 | i32 | voltage_ramp_mv | mV | V, 0.1 (soft cutoff band) | 0.1–5 V | L |
| 40 | i32 ×5 | battery_level[0..4] | mV × 10 | V, 0.1 (battery bars 1–5) | | L |
| 60 | i32 | speed_limit | km/h | km/h | 5–999 | L |
| 64 | i32 | wheel_circumference | mm | mm | 500–3000 | L |
| 68 | i32 | gear_ratio | pole pairs × gear ratio | number | 1–500 | R |
| 72 | i32 | pas_imp_per_turn | pulses / crank turn | number | 1–64 | R |
| 76 | i32 | pulses_per_revolution | magnets (external speed sensor) | number | 1–32 | R |
| 80 | i32 | pushassist_current | units | A, 0.1 (walk) | 0–20 A | L |
| 84 | i32 | ts_coef | gain | number (torque sensor) | 0–100000 | L |
| 88 | i32 | pas_timeout | ticks | ms (no-pedalling timeout) | 50–2000 ms | L |
| 92 | i32 | ramp_end | ticks | ms (PAS period for full assist) | 20–1000 ms | L |
| 96 | i32 | throttle_offset | ADC | counts | 0–4095 | L |
| 100 | i32 | throttle_max | ADC | counts | 0–4095 | L |
| 104 | i32 | motor_temp_threshold | °C | °C (derating starts) | 40–150 | L |
| 108 | i32 | motor_temp_max | °C | °C (motor off) | 45–160 | L |
| 112 | i32 | frac_high | PAS duty | number | 0–100 | L |
| 116 | i32 | frac_low | PAS duty | number | 0–100 | L |
| 120 | i32 | reverse | 1 / -1 | normal / reverse | ±1 | R |
| 124 | i32 | speed_temp_adc | index | AD1 (6) / AD2 (5) | 5, 6 | R |
| 128 | i32 | speed_temp_threshold | ADC | counts | 0–4095 | L |
| 132 | i32 | ntc_pullup_ohm | Ω | Ω (0 = no NTC) | 0–100000 | R |
| 136 | i32 | obs_inductance | observer | number (sensorless) | | R |
| 140 | i32 | obs_resistance | observer | number | | R |
| 144 | i32 | obs_flux_linkage | observer | number | | R |
| 148 | i32 | obs_gamma | shift | number | | R |
| 152 | i32 | obs_angle_offset | q31 (π = 2³¹) | degrees, 0.1 | -180–180 | L |
| 156 | i32 | current_ramp_ms | ms | ms (0 → full phase current) | 0–10000 | L |
| 160 | i32 | fw_current_max | units | A, 0.1 (field weakening Id, 0 = off) | 0–150 A | L |
| 164 | i32 | fw_battery_current_max | mA | A, 0.1 (0 = normal limit) | 0–60 A | L |
| 168 | i32 | fw_ramp_ms | ms | ms | 1–10000 | L |
| 172 | i32 | fw_voltage_pct | % | % (0 = 90 %) | 50–98 | L |
| 176 | i32 | melody_pct | % | % of phase current (0 = off) | 0–100 | R |
| 180 | u8 ×8 | assist_pct[0..7] | % | levels 1–5 (bytes 5–7 = 0) | 0–100 | L |
| 188 | u32 ×16 | reserved | 0 | hidden | 0 | – |
| 252 | u32 | crc | CRC-32 of 0–251 | hidden, recomputed by the display | | – |

Flags (offset 8):

| Bit | Name | Meaning | Apply |
|---|---|---|---|
| 0 | THROTTLE | throttle connected | L |
| 1 | LEGAL | EU legal mode | L |
| 2 | TORQUE_SENSOR | torque sensor mode | R |
| 3 | TORQUE_ON_AD1 | torque signal on AD1 | R |
| 4 | DIRDET | stop on back-pedalling | L |
| 5 | ADC_BRAKE | proportional regen on AD2 | R |
| 6 | SPEED_EXTERNAL | speed from external sensor / speed wire | R |
| 7 | SPEED_FROM_WIRE | speed pulses on the analog speed/temperature wire | R |
| 8 | TEMP_FROM_WIRE | motor NTC on the speed/temperature wire | R |

With an IONRISE display the assist levels come from the display (section 6); `assist_pct` stays the fallback for
other displays.

The live/reboot marks are conservative: a field is "live" only where EBiCS reads `RS.x` directly in its control
loop. Fields used to set up sensors, the observer or the startup melody are "reboot". Re-check the marks whenever
`settings_load()` gains derived values.

### 8.2 Layout 0x02: KT settings v1 (Kunteng), 32 bytes

Source: `kunteng/settings.h`. Valid when bytes 0–1 are `'K' 'T'`, byte 2 = 1, byte 3 = 32 and byte 31 =
`~(sum of bytes 0–30)`. All values are derived at start-up, so every field is **R** (PERSIST + REBOOT).

| Off | Type | Field | Display | Range |
|---|---|---|---|---|
| 0–1 | char ×2 | 'K','T' | hidden | fixed |
| 2 | u8 | version (1) | hidden | fixed |
| 3 | u8 | length (32) | hidden | fixed |
| 4 | u8 | battery cells | cells | 7–20 |
| 5 | u8 | motor current | A | 1–40 |
| 6 | u8 | regen current | A | 0–20 |
| 7 | u8 | throttle type | 1 / 2 | 1–2 |
| 8 | u8 | flags | bits below | |
| 9 | u8 | PAS magnets | number | 1–64 |
| 10 | u8 | cadence for full PAS power | rpm | 10–150 |
| 11–15 | u8 ×5 | assist level 1–5 | % | 0–100 |
| 16 | u8 | hall sensor angle offset | /256 turn | 0–255 |
| 17 | u8 | FOC current sample angle | /256 | 0–255 |
| 18 | u8 | erps for sine wave mode | erps | 0–255 |
| 19 | u8 | speed limit default | km/h | 5–99 |
| 20 | u8 | wheel size default | KT display code | 0–31 |
| 21 | u8 | controller type | 1 S06S, 2 S12S | 1–2 |
| 22 | u8 | phase current factor | ×0.1 | 10–40 |
| 23 | u8 | display protocol | 0 Kunteng … 8 EBiCS, 9 IONRISE | 0–9 |
| 24 | u8 | power ramp | ×0.1 s (0 = default) | 0–100 |
| 25 | i8 | FOC delay compensation | ×4 µs | -128–127 |
| 26 | u8 | motor characteristic | number (0 = 202) | 0–255 |
| 27 | u8 | field weakening max angle | degrees (0 = off) | 0–60 |
| 28 | u8 | field weakening battery current | A (0 = motor current) | 0–40 |
| 29 | u8 | field weakening phase current peak | A (0 = 15 A) | 0–40 |
| 30 | u8 | field weakening ramp | deg/s (0 = 30) | 0–255 |
| 31 | u8 | checksum | `~sum(0..30)`, recomputed by the display | |

KT flags (offset 8): bit 0 direct drive, 1 PAS on the left, 2 over-voltage protection, 3 no phase current limit,
4 no hall learning, 5 field weakening only with throttle, 6 no startup melody, 7 no FOC angle correction.

## 9. Integration in EBiCS

### 9.1 Lishui (STM32F103C6, 32 KB)

**Flash budget.** Code area 0x7700 bytes (30 464). The current KM5S build uses 29 780 bytes, leaving **684** free.
The KM5S driver occupies 1 075 bytes (`KingMeter_Init` 200, `KingMeter_Service` 648, `kingmeter_update` 136, tables 91).
`DISPLAY_TYPE_IONRISE` replaces it, so the protocol may use **~1 750 bytes**.

As built (EBiCS, 2026-11): the protocol core and the port are ~2.3 KB. They fit because the one call of CMSIS
`arm_sqrt_q31` (the only user of soft float, ~1.2 KB of library) was replaced by an integer square root: the
IONRISE sensorless build uses 29 492 of 30 464 bytes.

| Part | Flash (Cortex-M3, -Os) | RAM |
|---|---|---|
| `ionproto.c` (CRC, frames, receiver, proof) | 340 | 0 |
| `ionctrl.c` (handshake, link, staging, dispatch) | 628 | 152 |
| Port in `display_ionrise.c` (telemetry, control, apply, hand-off) | ~650 estimated | 256 staging + 264 hand-off (`.noinit`) |
| **Total** | **~1 620** of ~1 750 | ~670 of the free RAM |

The CRC-32 for the block check already exists (`settings_crc32`), so APPLY costs only a call.

**Wiring into the firmware:**

1. `config.h`: `#define DISPLAY_TYPE_IONRISE (1<<10)` and select it. UART1 at 9600 baud (as KM5S).
2. RX: the existing DMA ring + UART idle interrupt; on `ui8_UART_flag`, feed the new bytes to `ionc_rx_byte()`.
3. Main loop: `ionc_service(HAL_GetTick())`.
4. `ionc_port_control()`: `MS.assist_level = req[0]` (0–255 path, as KM5S), walk flag → `ui8_Push_Assist_flag`, light →
   `LIGHT` pin, speed limit / power limit → scale the targets.
5. `ionc_port_telemetry()`: speed ×100 from `get_wheeltime_ms()` and `RS.wheel_circumference`
   (`speed_x100 = circ_mm × 360000 / (wheeltime_µs …)` with enough resolution; prefer the internal tic count for
   high speeds); voltage `MS.Voltage × CAL_V / 10`; battery current `MS.Battery_Current / 100`; temperatures
   `MS.Temperature`, `MS.int_Temperature`; status and error bits from the existing flags.
6. `ionc_port_apply()`: check magic/version/size + `settings_crc32()`, then LIVE = `memcpy(&RS, staging, 256)`,
   REBOOT = hand-off area + `NVIC_SystemReset()` after the TX-complete callback.
7. `settings_load()`: check the hand-off area first (section 7.4).
8. `ionc_port_link_lost()`: `MS.assist_level = 0`, walk off.

### 9.2 Kunteng (STM8S105, 32 KB)

About **6 KB** of flash is free, so the integration is not constrained. UART2 RX interrupt → `ionc_rx_byte()`;
`ionc_service()` in the 10 ms loop. PERSIST writes the 32-byte block with `eeprom_write_bytes(0x4080, …)` and fixes the
checksum; REBOOT = watchdog reset. Display protocol value in byte 23 selects IONRISE. SDCC supports the C99 used here
(`static inline`, mixed declarations).

## 10. Display behaviour (iondisp)

- Boot: HELLO every 300 ms until answered (a controller may boot slower). Without an answer for 3 s the display can
  fall back to another protocol if configured.
- After AUTH: settings sync (7.1), then TELEMETRY at 10 Hz.
- Shows E30 after 5 missed responses, restarts the handshake.
- Settings menu: the controller settings are listed from the tables in section 8 with real units; changed values are
  written to staging and applied with the right mode; reboot fields are only applied with the wheel stopped.

## 11. Test vectors

Generated by `python protocol/ionproto.py` (nonce_d = `0x12345678`, nonce_c = `0x9ABCDEF0`, family = 1,
display id = 1):

```
HELLO req : A5 0C 01 01 01 00 01 00 78 56 34 12 00 01 02 9E
HELLO resp: A5 16 81 01 01 00 01 00 00 01 01 01 00 01 01 00 07 02 08 F6 F0 DE BC 9A D1 98   (proof_c = F608)
AUTH req  : A5 05 02 02 17 54 01 53 F0                                                       (proof_d = 5417)
AUTH resp : A5 03 82 02 00 44 2C
TELEM req : A5 0A 10 03 99 03 05 01 00 00 64 00 E0 00
TELEM resp: A5 18 90 03 E6 09 C6 13 7B 00 FA 00 29 23 48 FF FF FF 96 30 0D 10 00 00 00 00 B0 89
SET_READ  : A5 06 20 04 00 3C 00 08 29 DD
```

## 12. Verification

`make sim` (EBiCS layout) and `make sim-kt` (KT layout) build the **real** protocol code for the PC
(`src/ionlink.c`, `src/ion_desc.c`, `protocol/ionproto.c`, `protocol/controller/ionctrl.c`) and join display and
controller with a simulated 9600 baud cable. Scenarios: first contact, live and reboot settings changes, controller
power cycle, controller rejecting every block, a bad cable (0.4 % damaged bytes), a very noisy (4 %) and an extreme
(20 %) line, a 3 s cable cut, 2 million random bytes into each receiver, and an impostor display. Requirements:
no corrupted value is ever accepted, updates arrive at the physical limit of the line, E30 within 1 s of a real
outage, assist off within 0.6 s, recovery within 1 s, no reboot loops.

The same source files are used by iondisp, EBiCS Lishui and EBiCS Kunteng, so a fix is made once. They compile
unchanged with arm-none-eabi-gcc (Cortex-M3: 1 032 bytes) and SDCC (STM8: 1 873 bytes).

## 13. Versioning

- **Major** changes break compatibility (frame format, meaning of existing fields). A display only talks to the same
  major version.
- **Minor** changes only add: new commands, new capability bits, longer payloads (receivers ignore extra bytes,
  senders of older minor versions send the old length).
- Settings layouts have their own id and version; the display keeps one descriptor table per layout.
