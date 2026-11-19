// Bit-banged ARM Serial Wire Debug for the ESP32-C3. See ARM IHI 0031 (ADIv5).
// Timing: the host changes SWDIO while SWCLK is low, the target samples on the rising edge;
// the target drives SWDIO after the rising edge, the host samples while SWCLK is low (like Black Magic Probe).
// Part of EBiCS, GPL v3 or later.
#include "swd.h"
#include <driver/gpio.h>
#include <rom/ets_sys.h>

static const uint8_t DP_ABORT = 0x0, DP_DPIDR = 0x0, DP_CTRLSTAT = 0x4, DP_SELECT = 0x8, DP_RDBUFF = 0xC;
static const uint8_t AP_CSW = 0x0, AP_TAR = 0x4, AP_DRW = 0xC;
static const uint32_t CSW_BASE = 0x23000000;  // DbgSwEnable, master type, HPROT
static const uint32_t CSW_INC = 0x10, CSW_8 = 0x0, CSW_16 = 0x1, CSW_32 = 0x2;

void Swd::begin() {
  gpio_reset_pin((gpio_num_t)clk_);
  gpio_reset_pin((gpio_num_t)dio_);
  gpio_set_direction((gpio_num_t)clk_, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)clk_, 0);
  gpio_set_pull_mode((gpio_num_t)dio_, GPIO_PULLUP_ONLY);
  dirOut();
  if (rst_ >= 0) {  // open drain, released
    gpio_reset_pin((gpio_num_t)rst_);
    gpio_set_direction((gpio_num_t)rst_, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_level((gpio_num_t)rst_, 1);
  }
}

void Swd::reset(bool active) {
  if (rst_ >= 0) gpio_set_level((gpio_num_t)rst_, active ? 0 : 1);
}

inline void Swd::clockCycle() {
  gpio_set_level((gpio_num_t)clk_, 1);
  ets_delay_us(half_);
  gpio_set_level((gpio_num_t)clk_, 0);
  ets_delay_us(half_);
}

void Swd::dirOut() { gpio_set_direction((gpio_num_t)dio_, GPIO_MODE_OUTPUT); }
void Swd::dirIn() { gpio_set_direction((gpio_num_t)dio_, GPIO_MODE_INPUT); }

void Swd::writeBits(uint32_t value, int n) {
  for (int i = 0; i < n; i++) {
    gpio_set_level((gpio_num_t)dio_, (value >> i) & 1);
    clockCycle();
  }
}

uint32_t Swd::readBits(int n) {
  uint32_t v = 0;
  for (int i = 0; i < n; i++) {
    ets_delay_us(half_);
    if (gpio_get_level((gpio_num_t)dio_)) v |= (1UL << i);
    gpio_set_level((gpio_num_t)clk_, 1);
    ets_delay_us(half_);
    gpio_set_level((gpio_num_t)clk_, 0);
  }
  return v;
}

void Swd::lineReset() {
  dirOut();
  writeBits(0xFFFFFFFF, 32);
  writeBits(0xFFFFFFFF, 24);
  writeBits(0xE79E, 16);  // JTAG-to-SWD
  writeBits(0xFFFFFFFF, 32);
  writeBits(0xFFFFFFFF, 24);
  writeBits(0, 16);  // idle
}

// One SWD packet. Returns SWD_OK / SWD_WAIT / SWD_FAULT / SWD_NO_ACK / SWD_PARITY.
int Swd::transfer(uint8_t apndp, uint8_t rnw, uint8_t a23, uint32_t *data) {
  uint8_t a2 = (a23 >> 2) & 1, a3 = (a23 >> 3) & 1;
  uint8_t parity = (apndp + rnw + a2 + a3) & 1;
  uint8_t req = 1 | (apndp << 1) | (rnw << 2) | (a2 << 3) | (a3 << 4) | (parity << 5) | (0 << 6) | (1 << 7);
  dirOut();
  writeBits(req, 8);
  dirIn();
  readBits(1);  // turnaround
  uint32_t ack = readBits(3);
  if (ack == 1) {
    if (rnw) {
      uint32_t v = readBits(32);
      uint32_t p = readBits(1);
      readBits(1);  // turnaround
      dirOut();
      writeBits(0, 8);  // idle
      if ((__builtin_popcount(v) & 1) != p) return SWD_PARITY;
      *data = v;
    } else {
      readBits(1);  // turnaround
      dirOut();
      writeBits(*data, 32);
      writeBits(__builtin_popcount(*data) & 1, 1);
      writeBits(0, 8);  // idle, lets the write complete
    }
    return SWD_OK;
  }
  readBits(1);  // turnaround back
  dirOut();
  writeBits(0, 8);
  if (ack == 2) return SWD_WAIT;
  if (ack == 4) return SWD_FAULT;
  return SWD_NO_ACK;
}

int Swd::transferRetry(uint8_t apndp, uint8_t rnw, uint8_t a23, uint32_t *data) {
  int r = SWD_NO_ACK;
  for (int i = 0; i < 200; i++) {
    r = transfer(apndp, rnw, a23, data);
    if (r != SWD_WAIT) break;
    ets_delay_us(20);
  }
  if (r == SWD_FAULT) {
    err_ = "SWD fault (access error)";
    clearErrors();
  } else if (r == SWD_WAIT) {
    err_ = "SWD wait timeout";
    uint32_t abort = 1;  // DAPABORT
    transfer(0, 0, DP_ABORT, &abort);
  } else if (r == SWD_NO_ACK) {
    err_ = "no answer from the controller (check wires, power)";
  } else if (r == SWD_PARITY) {
    err_ = "SWD parity error (bad wires?)";
  }
  return r;
}

void Swd::clearErrors() {
  uint32_t v = 0x1E;  // ORUNERRCLR | WDERRCLR | STKERRCLR | STKCMPCLR
  transfer(0, 0, DP_ABORT, &v);
}

bool Swd::dpWrite(uint8_t a, uint32_t v) { return transferRetry(0, 0, a, &v) == SWD_OK; }
bool Swd::dpRead(uint8_t a, uint32_t &v) { return transferRetry(0, 1, a, &v) == SWD_OK; }
bool Swd::apWrite(uint8_t a, uint32_t v) { return transferRetry(1, 0, a, &v) == SWD_OK; }

bool Swd::setCsw(uint32_t v) {
  if (v == csw_) return true;
  if (!apWrite(AP_CSW, v)) return false;
  csw_ = v;
  return true;
}

bool Swd::connect() {
  csw_ = 0xFFFFFFFF;
  dpidr_ = 0;
  lineReset();
  uint32_t id = 0;
  if (transfer(0, 1, DP_DPIDR, &id) != SWD_OK) {
    lineReset();
    if (transfer(0, 1, DP_DPIDR, &id) != SWD_OK) {
      err_ = "no answer from the controller (check wires, power)";
      return false;
    }
  }
  dpidr_ = id;
  clearErrors();
  if (!dpWrite(DP_SELECT, 0)) return false;
  if (!dpWrite(DP_CTRLSTAT, 0x50000000)) return false;  // CSYSPWRUPREQ | CDBGPWRUPREQ
  for (int i = 0; i < 100; i++) {
    uint32_t s = 0;
    if (!dpRead(DP_CTRLSTAT, s)) return false;
    if ((s & 0xA0000000) == 0xA0000000) return true;
    delay(1);
  }
  err_ = "debug power-up failed";
  return false;
}

bool Swd::read32(uint32_t addr, uint32_t &value) {
  if (!setCsw(CSW_BASE | CSW_32)) return false;
  if (!apWrite(AP_TAR, addr)) return false;
  uint32_t dummy;
  if (transferRetry(1, 1, AP_DRW, &dummy) != SWD_OK) return false;  // posted
  return dpRead(DP_RDBUFF, value);
}

bool Swd::write32(uint32_t addr, uint32_t value) {
  if (!setCsw(CSW_BASE | CSW_32)) return false;
  if (!apWrite(AP_TAR, addr)) return false;
  if (!apWrite(AP_DRW, value)) return false;
  uint32_t s;
  return dpRead(DP_RDBUFF, s);  // wait for the write
}

bool Swd::write16(uint32_t addr, uint16_t value) {
  if (!setCsw(CSW_BASE | CSW_16)) return false;
  if (!apWrite(AP_TAR, addr)) return false;
  if (!apWrite(AP_DRW, (uint32_t)value << (8 * (addr & 2)))) return false;
  uint32_t s;
  return dpRead(DP_RDBUFF, s);
}

bool Swd::write8(uint32_t addr, uint8_t value) {
  if (!setCsw(CSW_BASE | CSW_8)) return false;
  if (!apWrite(AP_TAR, addr)) return false;
  if (!apWrite(AP_DRW, (uint32_t)value << (8 * (addr & 3)))) return false;
  uint32_t s;
  return dpRead(DP_RDBUFF, s);
}

bool Swd::readBlock(uint32_t addr, uint8_t *out, uint32_t len) {
  if ((addr & 3) || (len & 3)) {
    err_ = "unaligned read";
    return false;
  }
  if (!setCsw(CSW_BASE | CSW_INC | CSW_32)) return false;
  uint32_t done = 0;
  while (done < len) {
    uint32_t a = addr + done;
    uint32_t n = 0x400 - (a & 0x3FF);
    if (n > len - done) n = len - done;
    if (!apWrite(AP_TAR, a)) return false;
    uint32_t v;
    if (transferRetry(1, 1, AP_DRW, &v) != SWD_OK) return false;  // first read is posted
    for (uint32_t i = 0; i < n; i += 4) {
      bool last = (i + 4 >= n);
      int r = last ? transferRetry(0, 1, DP_RDBUFF, &v) : transferRetry(1, 1, AP_DRW, &v);
      if (r != SWD_OK) return false;
      memcpy(out + done + i, &v, 4);
    }
    done += n;
  }
  return true;
}

bool Swd::writeBlock16(uint32_t addr, const uint8_t *data, uint32_t len) {
  if ((addr & 1) || (len & 1)) {
    err_ = "unaligned half-word write";
    return false;
  }
  if (!setCsw(CSW_BASE | CSW_INC | CSW_16)) return false;
  uint32_t done = 0;
  while (done < len) {
    uint32_t a = addr + done;
    uint32_t n = 0x400 - (a & 0x3FF);
    if (n > len - done) n = len - done;
    if (!apWrite(AP_TAR, a)) return false;
    for (uint32_t i = 0; i < n; i += 2) {
      uint32_t hw = data[done + i] | (data[done + i + 1] << 8);
      if (!apWrite(AP_DRW, hw << (8 * ((a + i) & 2)))) return false;
    }
    done += n;
  }
  uint32_t s;
  return dpRead(DP_RDBUFF, s);
}
