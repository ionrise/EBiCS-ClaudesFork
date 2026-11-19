// Bit-banged ARM Serial Wire Debug (ADIv5 SW-DP + AHB-AP) for the ESP32-C3.
// Part of EBiCS, GPL v3 or later.
#pragma once
#include <Arduino.h>

enum SwdResult : int8_t { SWD_OK = 0, SWD_WAIT = -1, SWD_FAULT = -2, SWD_NO_ACK = -3, SWD_PARITY = -4 };

class Swd {
 public:
  Swd(int pinClk, int pinDio, int pinRst = -1) : clk_(pinClk), dio_(pinDio), rst_(pinRst) {}

  void begin();
  void setHalfPeriodUs(uint32_t us) { half_ = us; }

  // Line reset, JTAG-to-SWD, read DPIDR, power up the debug domain. true on success.
  bool connect();
  uint32_t dpidr() const { return dpidr_; }
  const char *lastError() const { return err_; }

  // 32-bit memory access through the AHB-AP
  bool read32(uint32_t addr, uint32_t &value);
  bool write32(uint32_t addr, uint32_t value);
  bool write16(uint32_t addr, uint16_t value);
  bool write8(uint32_t addr, uint8_t value);
  // words from a 4-aligned address; split internally at 1 KB (TAR auto-increment limit)
  bool readBlock(uint32_t addr, uint8_t *out, uint32_t len);
  // half-words to flash (FPEC in PG mode): sequential 16-bit writes, bus stalls are retried
  bool writeBlock16(uint32_t addr, const uint8_t *data, uint32_t len);

  // drive NRST (if wired): true = in reset
  void reset(bool active);

 private:
  int clk_, dio_, rst_;
  uint32_t half_ = 1;
  uint32_t dpidr_ = 0;
  uint32_t csw_ = 0xFFFFFFFF;
  const char *err_ = "";

  inline void clockCycle();
  void writeBits(uint32_t value, int n);
  uint32_t readBits(int n);
  void dirOut();
  void dirIn();
  void lineReset();
  int transfer(uint8_t apndp, uint8_t rnw, uint8_t a23, uint32_t *data);
  int transferRetry(uint8_t apndp, uint8_t rnw, uint8_t a23, uint32_t *data);
  bool dpWrite(uint8_t a, uint32_t v);
  bool dpRead(uint8_t a, uint32_t &v);
  bool apWrite(uint8_t a, uint32_t v);
  bool setCsw(uint32_t v);
  void clearErrors();
};
