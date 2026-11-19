// STM32F103 (and GD32F103) over SWD: halt/reset, flash erase/program/verify, read-out protection.
// Same rules as the Android app (android_app/lib/stlink/stm32f1.dart):
// pages 30 and 31 (0x08007800..0x08007FFF) hold the emulated EEPROM with the hall angles and KV and are
// never erased or written; no mass erase.
// Part of EBiCS, GPL v3 or later.
#pragma once
#include "swd.h"

typedef void (*ProgressFn)(const char *step, int percent);

struct ChipInfo {
  uint32_t devId = 0, revId = 0, flashKb = 0;
  bool readProtected = false;
  bool supported() const { return devId == 0x412 || devId == 0x410; }
  const char *name() const {
    return devId == 0x412 ? "STM32F103 low density" : devId == 0x410 ? "STM32F103 medium density" : "unknown chip";
  }
};

class Stm32F1 {
 public:
  static const uint32_t FLASH_BASE_ADDR = 0x08000000;
  static const uint32_t PAGE = 1024;
  static const uint32_t PROTECTED_START = 0x08007800;  // emulated EEPROM: never touch
  static const uint32_t SETTINGS_ADDR = 0x08007700;
  static const uint32_t SETTINGS_SIZE = 256;

  explicit Stm32F1(Swd &swd) : swd_(swd) {}

  const char *error() const { return err_; }
  bool info(ChipInfo &out);
  bool readProtected(bool &locked);
  bool halt();
  bool resetHalt();
  bool resetRun();
  // image starts at 0x08000000, must end at or before PROTECTED_START. Erases the pages it covers,
  // skips all-0xFF pages when writing, verifies. extraErasePage: also erase this page (or 0)
  bool writeImage(const uint8_t *image, uint32_t len, ProgressFn progress, uint32_t extraErasePage = 0);
  // rewrite [addr, addr+len) inside one page: read page, patch, erase, program, verify
  bool patchPage(uint32_t addr, const uint8_t *data, uint32_t len);
  bool removeReadProtection(ProgressFn progress);

  static bool writable(uint32_t addr, uint32_t len) {
    return addr >= FLASH_BASE_ADDR && addr + len <= PROTECTED_START && addr + len >= addr;
  }

 private:
  Swd &swd_;
  const char *err_ = "";
  uint8_t page_[1024];

  bool fail(const char *e) {
    err_ = e;
    return false;
  }
  bool waitHalted(uint32_t ms);
  bool unlockFlash();
  void lockFlash();
  bool waitBusy(uint32_t ms, uint32_t &sr);
  bool erasePage(uint32_t addr);
  bool programPage(uint32_t addr, const uint8_t *data, uint32_t len);
  bool verify(uint32_t addr, const uint8_t *data, uint32_t len);
};
