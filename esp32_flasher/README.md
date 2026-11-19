# EBiCS Flasher (ESP32-C3) - wireless setup from any phone

An ESP32-C3 board (about 3 EUR, e.g. "ESP32-C3 SuperMini" or "Lolin C3 Mini") replaces the ST-Link. It opens its own
Wi-Fi hotspot with a web page: open it in the phone's browser, answer the bike questions, and the ESP installs EBiCS
on the Lishui controller over SWD by itself. No ST-Link, no PC, no app, works with Android and iPhone.

All prebuilt firmware variants (every display protocol, hall and sensorless) are inside the ESP firmware, so the
phone doesn't need internet. Your settings go into the runtime settings block, exactly like with the Android app.

For **Lishui** controllers (STM32F103). Kunteng (STM8, SWIM) is not supported by the ESP flasher.

## Wiring

| ESP32-C3 | Lishui programming plug |
|---|---|
| GPIO4 | SWCLK (green) |
| GPIO5 | SWDIO (black) |
| GND | GND (yellow) |
| (GPIO6) | (NRST, optional, not on the Lishui plug) |

- **Do not connect the red 3.3 V wire.** The controller is powered by its battery, the ESP by its USB port
  (power bank or phone charger). Only GND is shared.
- Colours can differ between controllers: check with a multimeter.
- Keep the wires short (under 20 cm) for reliable SWD.

## Use

1. Switch the controller on (battery), lift the wheel.
2. Power the ESP. On the phone, join the Wi-Fi **EBiCS-Flasher**, password **ebics1234**.
3. The page opens by itself (or open **http://192.168.4.1**).
4. **Setup** tab: the same steps as the apps - connect, unlock (once, erases the original firmware), your bike,
   install, motor calibration, direction, gearing, sensor test. Every step has Back, Retry and Skip.
5. Later: **Settings** tab -> change -> **Write to controller** (rewrites only the settings, about a second).
   **Live** shows what the controller sees. **Tools**: connection check, unlock, install, restart, and
   *install your own .bin* (e.g. a desktop build).

Hold any setting to see what it means. Settings are kept in the phone's browser; the profile JSON is the same as
`setup_app/my_bike.json` and the Android app's profile.

The flasher never erases or writes the last 2 KB of the chip (pages 30-31, `0x08007800`-`0x08007FFF`: hall
angles and KV), and never uses mass erase. Unlocking a locked chip erases it completely once (STM32 hardware).

## Install the flasher on the ESP32-C3

Download `EBiCS-Flasher-ESP32C3-<version>.bin` from GitHub (Actions -> newest "Android APK + ESP32 flasher" run ->
Artifacts -> *EBiCS-Flasher-ESP32C3*). It is a complete 4 MB image: write it at address **0x0**.

- Easiest, in Chrome or Edge on a PC: https://espressif.github.io/esptool-js/ -> Connect (choose the ESP's USB port)
  -> Flash Address `0x0`, the .bin file -> Program.
- Or with esptool: `esptool.py --chip esp32c3 write_flash 0x0 EBiCS-Flasher-ESP32C3-<version>.bin`

If the ESP doesn't show up, hold its BOOT button while plugging it in.

## Build it yourself

```bash
python setup_app/build_variants.py        # firmware variants -> android_app/assets/firmware
python esp32_flasher/gen_assets.py        # -> EBiCS_Flasher/fw_variants.h, web_page.h
arduino-cli compile -b "esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=no_ota" \
    --output-dir esp32_flasher/build esp32_flasher/EBiCS_Flasher
```

Needs the ESP32 Arduino core 3.3 (`esp32:esp32`). In the Arduino IDE: board *ESP32C3 Dev Module*, *USB CDC On
Boot: Enabled*, *Partition Scheme: No OTA (2MB APP/2MB SPIFFS)*. Pins and the Wi-Fi name/password are at the top
of `EBiCS_Flasher.ino`.

## How it works

| File | What |
|---|---|
| `EBiCS_Flasher/swd.cpp` | bit-banged ARM Serial Wire Debug: line reset, SW-DP, AHB-AP memory access (8/16/32 bit) |
| `EBiCS_Flasher/stm32f1.cpp` | halt/reset, flash erase/program (16-bit writes)/verify, page-29 settings patch, RDP unlock |
| `EBiCS_Flasher/EBiCS_Flasher.ino` | Wi-Fi hotspot, captive portal, web API, background jobs, live data from the symbol maps |
| `web/index.html`, `web/settings_block.js` | the web page; settings block built in the browser (same as the apps) |
| `gen_assets.py` | embeds the firmware variants (with CRC-32 and symbol addresses) and the gzipped page |
| `web/mock_server.py` | fake ESP API to try the page on a PC: `python esp32_flasher/web/mock_server.py` |
| `web/test.html` | checks `settings_block.js` against the Python reference vectors |

Web API: `GET /api/info`, `/api/variants`, `/api/job`, `/api/live`; `POST /api/install` (variant, block),
`/api/settings` (block), `/api/unlock` (confirm=erase), `/api/calibrate`, `/api/restart`, `/api/upload` (.bin).
