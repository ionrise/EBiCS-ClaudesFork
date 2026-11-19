// STM32F103 flash programming over SWD. Register map: RM0008 (flash: PM0075).
// Part of EBiCS, GPL v3 or later.
#include "stm32f1.h"

static const uint32_t DHCSR = 0xE000EDF0, DEMCR = 0xE000EDFC, AIRCR = 0xE000ED0C;
static const uint32_t DBGKEY = 0xA05F0000, C_DEBUGEN = 1, C_HALT = 2, S_HALT = 1UL << 17;
static const uint32_t DBGMCU_IDCODE = 0xE0042000, DBGMCU_CR = 0xE0042004, FLASH_SIZE_REG = 0x1FFFF7E0;
static const uint32_t F_KEYR = 0x40022004, F_OPTKEYR = 0x40022008, F_SR = 0x4002200C, F_CR = 0x40022010;
static const uint32_t F_AR = 0x40022014, F_OBR = 0x4002201C;
static const uint32_t KEY1 = 0x45670123, KEY2 = 0xCDEF89AB;
static const uint32_t SR_BSY = 1, SR_PGERR = 4, SR_WRPRTERR = 16, SR_EOP = 32;
static const uint32_t CR_PG = 1, CR_PER = 2, CR_OPTPG = 16, CR_OPTER = 32, CR_STRT = 64, CR_LOCK = 128, CR_OPTWRE = 512;
static const uint32_t OPTION_BYTES = 0x1FFFF800;

bool Stm32F1::info(ChipInfo &out) {
  uint32_t id, obr, fs = 0;
  if (!swd_.read32(DBGMCU_IDCODE, id)) return fail(swd_.lastError());
  if (!swd_.read32(F_OBR, obr)) return fail(swd_.lastError());
  swd_.read32(FLASH_SIZE_REG, fs);  // may fail when protected
  out.devId = id & 0xFFF;
  out.revId = id >> 16;
  out.flashKb = fs & 0xFFFF;
  out.readProtected = (obr & 2) != 0;
  return true;
}

bool Stm32F1::readProtected(bool &locked) {
  uint32_t obr;
  if (!swd_.read32(F_OBR, obr)) return fail(swd_.lastError());
  locked = (obr & 2) != 0;
  return true;
}

bool Stm32F1::waitHalted(uint32_t ms) {
  uint32_t t0 = millis(), v;
  while (millis() - t0 < ms) {
    if (swd_.read32(DHCSR, v) && (v & S_HALT)) return true;
    delay(2);
  }
  return fail("the controller did not stop (halt)");
}

bool Stm32F1::halt() {
  if (!swd_.write32(DHCSR, DBGKEY | C_HALT | C_DEBUGEN)) return fail(swd_.lastError());
  return waitHalted(500);
}

bool Stm32F1::resetHalt() {
  if (!halt()) return false;
  uint32_t demcr = 0, cr = 0;
  swd_.read32(DEMCR, demcr);
  swd_.write32(DEMCR, demcr | 1);       // VC_CORERESET
  swd_.write32(AIRCR, 0x05FA0004);      // SYSRESETREQ (the SWD link keeps working)
  delay(50);
  if (!waitHalted(500) && !halt()) return false;
  swd_.write32(DEMCR, demcr & ~1UL);
  swd_.read32(DBGMCU_CR, cr);
  swd_.write32(DBGMCU_CR, cr | (1 << 8) | (1 << 9));  // freeze the watchdogs while halted
  return true;
}

bool Stm32F1::resetRun() {
  uint32_t demcr = 0;
  swd_.read32(DEMCR, demcr);
  swd_.write32(DEMCR, demcr & ~1UL);
  swd_.write32(AIRCR, 0x05FA0004);
  delay(50);
  if (!swd_.write32(DHCSR, DBGKEY | C_DEBUGEN)) return fail(swd_.lastError());  // run
  delay(50);
  return true;
}

bool Stm32F1::unlockFlash() {
  uint32_t cr;
  if (!swd_.read32(F_CR, cr)) return fail(swd_.lastError());
  if (cr & CR_LOCK) {
    swd_.write32(F_KEYR, KEY1);
    swd_.write32(F_KEYR, KEY2);
    if (!swd_.read32(F_CR, cr) || (cr & CR_LOCK)) return fail("could not unlock the flash");
  }
  swd_.write32(F_SR, SR_EOP | SR_PGERR | SR_WRPRTERR);
  return true;
}

void Stm32F1::lockFlash() { swd_.write32(F_CR, CR_LOCK); }

bool Stm32F1::waitBusy(uint32_t ms, uint32_t &sr) {
  uint32_t t0 = millis();
  while (true) {
    if (!swd_.read32(F_SR, sr)) return fail(swd_.lastError());
    if (!(sr & SR_BSY)) return true;
    if (millis() - t0 > ms) return fail("flash busy for too long");
    delay(1);
  }
}

bool Stm32F1::erasePage(uint32_t addr) {
  if (!writable(addr, PAGE) || (addr % PAGE)) return fail("refusing to erase outside the firmware area");
  uint32_t sr;
  swd_.write32(F_CR, CR_PER);
  swd_.write32(F_AR, addr);
  swd_.write32(F_CR, CR_PER | CR_STRT);
  bool ok = waitBusy(1000, sr);
  swd_.write32(F_CR, 0);
  if (!ok) return false;
  if (sr & (SR_PGERR | SR_WRPRTERR)) {
    swd_.write32(F_SR, SR_EOP | SR_PGERR | SR_WRPRTERR);
    return fail("page erase failed (write protected?)");
  }
  swd_.write32(F_SR, SR_EOP);
  return true;
}

bool Stm32F1::programPage(uint32_t addr, const uint8_t *data, uint32_t len) {
  if (!writable(addr, len)) return fail("refusing to write outside the firmware area");
  if (len == 0) return true;
  uint32_t sr;
  swd_.write32(F_CR, CR_PG);
  bool ok = swd_.writeBlock16(addr, data, len) && waitBusy(100, sr);
  swd_.write32(F_CR, 0);
  if (!ok) return fail(swd_.lastError());
  if (sr & (SR_PGERR | SR_WRPRTERR)) {
    swd_.write32(F_SR, SR_EOP | SR_PGERR | SR_WRPRTERR);
    return fail(sr & SR_WRPRTERR ? "programming failed: write protected" : "programming failed: not erased");
  }
  return true;
}

bool Stm32F1::verify(uint32_t addr, const uint8_t *data, uint32_t len) {
  uint32_t done = 0;
  while (done < len) {
    uint32_t n = len - done > PAGE ? PAGE : len - done;
    uint32_t n4 = (n + 3) & ~3UL;
    if (!swd_.readBlock(addr + done, page_, n4)) return fail(swd_.lastError());
    if (memcmp(page_, data + done, n)) return fail("verify failed");
    done += n;
  }
  return true;
}

bool Stm32F1::writeImage(const uint8_t *image, uint32_t len, ProgressFn progress, uint32_t extraErasePage) {
  if (!writable(FLASH_BASE_ADDR, len)) return fail("image too big: it would overwrite the motor calibration");
  if (extraErasePage && !writable(extraErasePage, PAGE)) return fail("refusing to erase outside the firmware area");
  progress("Stopping the controller", 0);
  if (!resetHalt() || !unlockFlash()) return false;
  uint32_t pages = (len + PAGE - 1) / PAGE;
  bool ok = true;
  for (uint32_t p = 0; ok && p < pages; p++) {
    progress("Erasing", p * 100 / pages);
    ok = erasePage(FLASH_BASE_ADDR + p * PAGE);
  }
  if (ok && extraErasePage && extraErasePage >= FLASH_BASE_ADDR + pages * PAGE) ok = erasePage(extraErasePage);
  for (uint32_t p = 0; ok && p < pages; p++) {
    progress("Writing", p * 100 / pages);
    uint32_t start = p * PAGE, n = (start + PAGE > len) ? len - start : PAGE;
    bool blank = true;
    for (uint32_t i = 0; i < n && blank; i++) blank = image[start + i] == 0xFF;
    if (!blank) ok = programPage(FLASH_BASE_ADDR + start, image + start, (n + 1) & ~1UL);
  }
  lockFlash();
  if (!ok) return false;
  for (uint32_t p = 0; p < pages; p++) {
    progress("Verifying", p * 100 / pages);
    uint32_t start = p * PAGE, n = (start + PAGE > len) ? len - start : PAGE;
    if (!verify(FLASH_BASE_ADDR + start, image + start, n)) return false;
  }
  progress("Done", 100);
  return true;
}

bool Stm32F1::patchPage(uint32_t addr, const uint8_t *data, uint32_t len) {
  uint32_t page = addr & ~(PAGE - 1);
  if (addr + len > page + PAGE) return fail("patch crosses a page");
  if (!writable(page, PAGE)) return fail("refusing to write outside the firmware area");
  if (!resetHalt()) return false;
  static uint8_t content[1024];
  if (!swd_.readBlock(page, content, PAGE)) return fail(swd_.lastError());
  memcpy(content + (addr - page), data, len);
  bool ok = unlockFlash() && erasePage(page) && programPage(page, content, PAGE);
  lockFlash();
  return ok && verify(page, content, PAGE);
}

bool Stm32F1::removeReadProtection(ProgressFn progress) {
  progress("Stopping the core", 5);
  if (!resetHalt() || !unlockFlash()) return false;
  swd_.write32(F_OPTKEYR, KEY1);
  swd_.write32(F_OPTKEYR, KEY2);
  uint32_t cr = 0, sr;
  swd_.read32(F_CR, cr);
  if (!(cr & CR_OPTWRE)) return fail("could not unlock the option bytes");
  progress("Erasing option bytes and the whole flash", 20);
  swd_.write32(F_CR, CR_OPTER | CR_OPTWRE);
  swd_.write32(F_CR, CR_OPTER | CR_STRT | CR_OPTWRE);
  if (!waitBusy(10000, sr)) return false;
  if (sr & SR_WRPRTERR) return fail("option byte erase failed");
  swd_.write32(F_SR, SR_EOP | SR_PGERR | SR_WRPRTERR);
  progress("Writing RDP = 0xA5, no write protection", 70);
  // RDP, USER, DATA0, DATA1, WRP0..3; the complement bytes are written by the flash controller
  static const uint8_t ob[16] = {0xA5, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0};
  swd_.write32(F_CR, CR_OPTPG | CR_OPTWRE);
  bool ok = true;
  for (int i = 0; ok && i < 16; i += 2) {
    ok = swd_.write16(OPTION_BYTES + i, ob[i] | (ob[i + 1] << 8)) && waitBusy(100, sr);
    if (ok && (sr & (SR_PGERR | SR_WRPRTERR))) ok = fail("option byte write failed");
  }
  swd_.write32(F_CR, CR_LOCK);
  if (!ok) return fail(err_[0] ? err_ : swd_.lastError());
  progress("Done - switch the controller off and on", 100);
  return true;
}
