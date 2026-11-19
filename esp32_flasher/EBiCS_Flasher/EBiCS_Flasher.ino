/*
 * EBiCS Flasher - wireless SWD flasher for Lishui controllers (STM32F103) on an ESP32-C3.
 *
 * The ESP32-C3 opens its own Wi-Fi hotspot with a web page. Open it in the phone's browser: choose your
 * display and bike settings, and the ESP installs the EBiCS firmware over SWD by itself (no ST-Link, no PC,
 * no app). All prebuilt firmware variants are inside this sketch (fw_variants.h, see gen_assets.py).
 *
 * Wiring (ESP32-C3 SuperMini / C3 Mini):        Lishui programming plug:
 *   GPIO4  -> SWCLK                               GREEN
 *   GPIO5  -> SWDIO                               BLACK
 *   GND    -> GND                                 YELLOW
 *   (GPIO6 -> NRST, optional, not on the Lishui plug)
 *   Do NOT connect the red 3.3 V wire. Power the ESP from USB (power bank or phone charger).
 *
 * Wi-Fi: "EBiCS-Flasher", password "ebics1234", page http://192.168.4.1 (or http://ebics.local)
 *
 * Pages 30-31 of the STM32 (0x08007800..0x08007FFF, hall angles + KV) are never erased or written.
 *
 * Part of EBiCS, GPL v3 or later.
 */
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include "swd.h"
#include "stm32f1.h"
#include "fw_variants.h"
#include "web_page.h"

// ---------------------------------------------------------------- configuration
static const int PIN_SWCLK = 4;
static const int PIN_SWDIO = 5;
static const int PIN_NRST = 6;      // optional
static const int PIN_LED = 8;       // SuperMini blue LED, active low (-1 = none)
static const char *AP_SSID = "EBiCS-Flasher";
static const char *AP_PASS = "ebics1234";  // at least 8 characters
static const uint32_t SETUP_MAGIC = 0xEB1C5002;
static const uint32_t SETTINGS_MAGIC = 0xEB1C5E77;

// ---------------------------------------------------------------- state
Swd swd(PIN_SWCLK, PIN_SWDIO, PIN_NRST);
Stm32F1 chip(swd);
WebServer server(80);
DNSServer dns;
SemaphoreHandle_t swdLock;

static bool connected = false;
static ChipInfo chipInfo;
static int installed = -2;  // -2 unknown (not checked), -1 not one of ours, else index into FW_VARIANTS
static uint8_t img[0x7800];
static uint32_t uploadLen = 0;
static bool uploadTooBig = false;

struct Job {
  volatile bool busy = false;
  volatile bool ok = false;
  volatile bool done = false;
  char name[32] = "";
  char step[80] = "";
  volatile int progress = 0;
  char error[160] = "";
} job;

enum JobKind { JOB_INSTALL, JOB_SETTINGS, JOB_UNLOCK, JOB_UPLOAD };
static JobKind jobKind;
static int jobVariant = -1;
static uint8_t jobBlock[256];

// ---------------------------------------------------------------- helpers
static uint32_t crc32(const uint8_t *d, uint32_t n, uint32_t crc = 0) {
  crc = ~crc;
  while (n--) {
    crc ^= *d++;
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1)));
  }
  return ~crc;
}

static void led(bool on) {
  if (PIN_LED >= 0) digitalWrite(PIN_LED, on ? LOW : HIGH);
}

static String jsonEscape(const char *s) {
  String o;
  for (; *s; s++) {
    if (*s == '"' || *s == '\\') o += '\\';
    if (*s == '\n') { o += "\\n"; continue; }
    o += *s;
  }
  return o;
}

static void sendJson(int code, const String &body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}

static void sendError(int code, const char *msg) { sendJson(code, String("{\"error\":\"") + jsonEscape(msg) + "\"}"); }

static bool ensureConnected() {
  if (connected) {
    uint32_t v;
    if (swd.read32(0xE0042000, v)) return true;
    connected = false;
  }
  connected = swd.connect() && chip.info(chipInfo);
  if (!connected) installed = -2;
  return connected;
}

// which variant is on the controller: CRC of the flash vs. each variant
static void identify() {
  uint32_t maxLen = 0;
  for (int i = 0; i < FW_VARIANT_COUNT; i++) maxLen = max(maxLen, FW_VARIANTS[i].size);
  maxLen = (maxLen + 3) & ~3UL;
  installed = -1;
  if (chipInfo.readProtected || !swd.readBlock(Stm32F1::FLASH_BASE_ADDR, img, maxLen)) return;
  for (int i = 0; i < FW_VARIANT_COUNT; i++) {
    if (crc32(img, FW_VARIANTS[i].size) == FW_VARIANTS[i].crc) {
      installed = i;
      return;
    }
  }
}

static const FwSym *sym(int v, const char *name) {
  if (v < 0) return nullptr;
  for (int i = 0; i < FW_VARIANTS[v].nsyms; i++)
    if (!strcmp(FW_VARIANTS[v].syms[i].name, name)) return &FW_VARIANTS[v].syms[i];
  return nullptr;
}

static bool hexToBytes(const String &hex, uint8_t *out, size_t n) {
  if (hex.length() != n * 2) return false;
  for (size_t i = 0; i < n; i++) {
    char b[3] = {hex[2 * i], hex[2 * i + 1], 0};
    char *end;
    out[i] = (uint8_t)strtoul(b, &end, 16);
    if (*end) return false;
  }
  return true;
}

static bool blockValid(const uint8_t *b) {
  uint32_t magic, crc;
  memcpy(&magic, b, 4);
  memcpy(&crc, b + 252, 4);
  return magic == SETTINGS_MAGIC && b[4] == 1 && b[6] == 0 && b[7] == 1 && crc == crc32(b, 252);
}

// ---------------------------------------------------------------- background jobs
static void progressCb(const char *step, int percent) {
  strlcpy(job.step, step, sizeof(job.step));
  job.progress = percent;
  led((millis() / 150) & 1);
}

static void jobTask(void *) {
  xSemaphoreTake(swdLock, portMAX_DELAY);
  bool ok = ensureConnected();
  if (!ok) strlcpy(job.error, swd.lastError(), sizeof(job.error));
  if (ok && jobKind != JOB_UNLOCK && chipInfo.readProtected) {
    ok = false;
    strlcpy(job.error, "the chip is still locked: unlock it first, then switch the controller off and on", sizeof(job.error));
  }
  if (ok) {
    switch (jobKind) {
      case JOB_INSTALL: {
        const FwVariant &v = FW_VARIANTS[jobVariant];
        memset(img, 0xFF, sizeof(img));
        memcpy(img, v.data, v.size);
        memcpy(img + (Stm32F1::SETTINGS_ADDR - Stm32F1::FLASH_BASE_ADDR), jobBlock, 256);
        ok = chip.writeImage(img, Stm32F1::SETTINGS_ADDR + 256 - Stm32F1::FLASH_BASE_ADDR, progressCb);
        if (ok) {
          progressCb("Starting the firmware", 100);
          ok = chip.resetRun();
          installed = jobVariant;
        } else installed = -2;
        break;
      }
      case JOB_SETTINGS:
        progressCb("Writing the settings", 30);
        ok = chip.patchPage(Stm32F1::SETTINGS_ADDR, jobBlock, 256) && chip.resetRun();
        break;
      case JOB_UNLOCK:
        ok = chip.removeReadProtection(progressCb);
        installed = -2;
        connected = false;
        break;
      case JOB_UPLOAD:
        // custom image (e.g. a desktop build): also erase page 29 so an old settings block can't override config.h
        ok = chip.writeImage(img, uploadLen, progressCb, Stm32F1::SETTINGS_ADDR & ~(Stm32F1::PAGE - 1));
        if (ok) ok = chip.resetRun();
        installed = -2;
        break;
    }
    if (!ok) strlcpy(job.error, chip.error(), sizeof(job.error));
  }
  if (!ok) connected = false;
  job.ok = ok;
  job.done = true;
  job.busy = false;
  led(false);
  xSemaphoreGive(swdLock);
  vTaskDelete(nullptr);
}

static bool startJob(JobKind kind, const char *name) {
  if (job.busy) return false;
  job.busy = true;
  job.done = job.ok = false;
  job.progress = 0;
  job.error[0] = 0;
  strlcpy(job.name, name, sizeof(job.name));
  strlcpy(job.step, "Starting", sizeof(job.step));
  jobKind = kind;
  xTaskCreate(jobTask, "job", 8192, nullptr, 1, nullptr);
  return true;
}

// ---------------------------------------------------------------- web handlers
static void handleRoot() {
  server.sendHeader("Content-Encoding", "gzip");
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", (const char *)WEB_PAGE_GZ, WEB_PAGE_GZ_LEN);
}

static void handleVariants() {
  String s = String("{\"version\":\"") + FW_VERSION + "\",\"variants\":[";
  for (int i = 0; i < FW_VARIANT_COUNT; i++) {
    const FwVariant &v = FW_VARIANTS[i];
    if (i) s += ',';
    s += String("{\"name\":\"") + v.name + "\",\"display\":\"" + v.display + "\",\"label\":\"" + jsonEscape(v.label) +
         "\",\"sensorless\":" + (v.sensorless ? "true" : "false") + ",\"size\":" + v.size + "}";
  }
  sendJson(200, s + "]}");
}

static void handleInfo() {
  if (job.busy) {
    sendJson(200, String("{\"busy\":true,\"connected\":true,\"installed\":\"") +
                      (installed >= 0 ? FW_VARIANTS[installed].name : "") + "\"}");
    return;
  }
  xSemaphoreTake(swdLock, portMAX_DELAY);
  bool ok = ensureConnected();
  String err = ok ? "" : swd.lastError();
  if (ok && installed == -2) identify();
  xSemaphoreGive(swdLock);
  String s = String("{\"busy\":false,\"connected\":") + (ok ? "true" : "false");
  if (ok) {
    char id[16];
    snprintf(id, sizeof(id), "0x%08X", (unsigned)swd.dpidr());
    s += String(",\"dpidr\":\"") + id + "\",\"chip\":\"" + chipInfo.name() + "\",\"devId\":" + chipInfo.devId +
         ",\"flashKb\":" + chipInfo.flashKb + ",\"supported\":" + (chipInfo.supported() ? "true" : "false") +
         ",\"locked\":" + (chipInfo.readProtected ? "true" : "false") + ",\"installed\":\"" +
         (installed >= 0 ? FW_VARIANTS[installed].name : "") + "\"";
  } else {
    s += String(",\"error\":\"") + jsonEscape(err.c_str()) + "\"";
  }
  sendJson(200, s + "}");
}

static void handleJob() {
  sendJson(200, String("{\"busy\":") + (job.busy ? "true" : "false") + ",\"done\":" + (job.done ? "true" : "false") +
                    ",\"ok\":" + (job.ok ? "true" : "false") + ",\"name\":\"" + job.name + "\",\"step\":\"" +
                    jsonEscape(job.step) + "\",\"progress\":" + job.progress + ",\"error\":\"" + jsonEscape(job.error) +
                    "\"}");
}

static void handleInstall() {
  int idx = -1;
  String name = server.arg("variant");
  for (int i = 0; i < FW_VARIANT_COUNT; i++)
    if (name == FW_VARIANTS[i].name) idx = i;
  if (idx < 0) return sendError(400, "unknown firmware variant");
  if (!hexToBytes(server.arg("block"), jobBlock, 256) || !blockValid(jobBlock)) return sendError(400, "invalid settings block");
  jobVariant = idx;
  if (!startJob(JOB_INSTALL, "install")) return sendError(409, "busy");
  sendJson(200, "{\"started\":true}");
}

static void handleSettings() {
  if (!hexToBytes(server.arg("block"), jobBlock, 256) || !blockValid(jobBlock)) return sendError(400, "invalid settings block");
  if (installed < 0) return sendError(409, "the controller does not run firmware from this flasher: install first");
  if (!startJob(JOB_SETTINGS, "settings")) return sendError(409, "busy");
  sendJson(200, "{\"started\":true}");
}

static void handleUnlock() {
  if (server.arg("confirm") != "erase") return sendError(400, "confirmation missing");
  if (!startJob(JOB_UNLOCK, "unlock")) return sendError(409, "busy");
  sendJson(200, "{\"started\":true}");
}

static void handleUploadData() {
  HTTPUpload &up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    if (job.busy) { uploadTooBig = true; return; }
    uploadLen = 0;
    uploadTooBig = false;
    memset(img, 0xFF, sizeof(img));
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (uploadTooBig) return;
    if (uploadLen + up.currentSize > Stm32F1::SETTINGS_ADDR - Stm32F1::FLASH_BASE_ADDR) {
      uploadTooBig = true;
      return;
    }
    memcpy(img + uploadLen, up.buf, up.currentSize);
    uploadLen += up.currentSize;
  }
}

static void handleUploadDone() {
  if (uploadTooBig) return sendError(400, "file too big (max 30464 bytes: it would overwrite the settings block and the motor calibration) or busy");
  if (uploadLen < 1024) return sendError(400, "file too small - choose a firmware .bin");
  uint32_t sp, rv;
  memcpy(&sp, img, 4);
  memcpy(&rv, img + 4, 4);
  if ((sp & 0xFFF00000) != 0x20000000 || (rv & 0xFFF00000) != 0x08000000)
    return sendError(400, "this is not an STM32 firmware .bin starting at 0x08000000");
  uploadLen = (uploadLen + 1) & ~1UL;
  if (!startJob(JOB_UPLOAD, "upload")) return sendError(409, "busy");
  sendJson(200, "{\"started\":true}");
}

static void handleRestart() {
  if (job.busy) return sendError(409, "busy");
  xSemaphoreTake(swdLock, portMAX_DELAY);
  bool ok = ensureConnected() && chip.resetRun();
  xSemaphoreGive(swdLock);
  ok ? sendJson(200, "{\"ok\":true}") : sendError(500, swd.lastError());
}

static void handleCalibrate() {
  if (job.busy) return sendError(409, "busy");
  const FwSym *s = sym(installed, "ui8_setup_cmd");
  if (!s) return sendError(409, "firmware not recognised: install the firmware first");
  xSemaphoreTake(swdLock, portMAX_DELAY);
  bool ok = ensureConnected() && swd.write8(s->addr, 1);
  xSemaphoreGive(swdLock);
  ok ? sendJson(200, "{\"ok\":true}") : sendError(500, swd.lastError());
}

// live values: one block read covering all variables of the installed variant
static void handleLive() {
  if (job.busy) return sendError(409, "busy");
  xSemaphoreTake(swdLock, portMAX_DELAY);
  bool ok = ensureConnected();
  if (ok && installed == -2) identify();
  if (!ok || installed < 0) {
    xSemaphoreGive(swdLock);
    return sendError(409, ok ? "firmware not recognised: install the firmware from this flasher" : swd.lastError());
  }
  const FwVariant &v = FW_VARIANTS[installed];
  uint32_t lo = 0xFFFFFFFF, hi = 0;
  for (int i = 0; i < v.nsyms; i++) {
    lo = min(lo, v.syms[i].addr);
    hi = max(hi, v.syms[i].addr + v.syms[i].size);
  }
  lo &= ~3UL;
  hi = (hi + 3) & ~3UL;
  static uint8_t ram[4096];
  if (hi - lo > sizeof(ram)) hi = lo + sizeof(ram);
  ok = swd.readBlock(lo, ram, hi - lo);
  xSemaphoreGive(swdLock);
  if (!ok) {
    connected = false;
    return sendError(500, swd.lastError());
  }
  auto rd = [&](uint32_t addr, int size, bool sgn) -> long {
    if (addr < lo || addr + size > hi) return 0;
    const uint8_t *p = ram + (addr - lo);
    uint32_t u = p[0] | (size > 1 ? p[1] << 8 : 0) | (size > 2 ? (p[2] << 16) | ((uint32_t)p[3] << 24) : 0);
    if (sgn && size == 2) return (int16_t)u;
    if (sgn && size == 4) return (int32_t)u;
    return (long)u;
  };
  String s = "{";
  for (int i = 0; i < v.nsyms; i++) {
    const FwSym &y = v.syms[i];
    if (!strcmp(y.name, "MS") || !strcmp(y.name, "adcData")) continue;
    bool sgn = !strcmp(y.name, "ui32_KV");
    int size = y.size > 4 ? 4 : y.size;
    s += String("\"") + y.name + "\":" + String(rd(y.addr, size, sgn)) + ",";
  }
  if (const FwSym *ms = sym(installed, "MS")) {
    s += "\"MS.Voltage\":" + String(rd(ms->addr + 0, 4, true)) + ",\"MS.Speed\":" + String(rd(ms->addr + 4, 4, false)) +
         ",\"MS.i_q\":" + String(rd(ms->addr + 12, 4, true)) + ",\"MS.Battery_Current\":" + String(rd(ms->addr + 48, 4, true)) +
         ",\"MS.assist_level\":" + String(rd(ms->addr + 54, 1, false)) + ",\"MS.Temperature\":" + String(rd(ms->addr + 56, 2, true)) +
         ",\"MS.int_Temperature\":" + String(rd(ms->addr + 58, 2, true)) + ",";
  }
  if (const FwSym *adc = sym(installed, "adcData")) {
    for (int n = 0; n < 9; n++) s += "\"adc" + String(n) + "\":" + String(rd(adc->addr + 2 * n, 2, false)) + ",";
  }
  s += String("\"variant\":\"") + v.name + "\"}";
  sendJson(200, s);
}

static void handleNotFound() {
  // captive portal: send every unknown address to the page
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.send(302, "text/plain", "");
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  if (PIN_LED >= 0) {
    pinMode(PIN_LED, OUTPUT);
    led(false);
  }
  swdLock = xSemaphoreCreateMutex();
  swd.begin();
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASS);
  dns.start(53, "*", IPAddress(192, 168, 4, 1));
  MDNS.begin("ebics");
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/variants", HTTP_GET, handleVariants);
  server.on("/api/info", HTTP_GET, handleInfo);
  server.on("/api/job", HTTP_GET, handleJob);
  server.on("/api/live", HTTP_GET, handleLive);
  server.on("/api/install", HTTP_POST, handleInstall);
  server.on("/api/settings", HTTP_POST, handleSettings);
  server.on("/api/unlock", HTTP_POST, handleUnlock);
  server.on("/api/restart", HTTP_POST, handleRestart);
  server.on("/api/calibrate", HTTP_POST, handleCalibrate);
  server.on("/api/upload", HTTP_POST, handleUploadDone, handleUploadData);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.printf("EBiCS Flasher %s: Wi-Fi \"%s\", http://192.168.4.1\n", FW_VERSION, AP_SSID);
}

void loop() {
  dns.processNextRequest();
  server.handleClient();
  if (!job.busy) led(connected && (millis() / 1000) % 2);
  delay(2);
}
