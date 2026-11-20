#!/usr/bin/env python3
"""Run the real KT firmware in the ucsim STM8 simulator (ships with SDCC) and check the motor code.

What is simulated: the CPU, timers (PWM interrupt every 64 us), GPIO pins and the data EEPROM.
The ADC is not modelled by ucsim, so the firmware is built with -DSIMULATION (skips only the ADC
wait loops) and the ADC result registers are written by the test.

Checks:
  1. the settings block written into the EEPROM is used (kt_settings in RAM)
  2. hall sensor sequence at constant speed -> speed (erps), sine wave mode, interpolation step
  3. hall sensors that are not 60 deg apart -> the hall learning finds their real positions
  4. without a settings block the compile time defaults are used

Usage: python kunteng/sim/run_sim.py
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.dirname(HERE)
sys.path.insert(0, FW)
sys.path.insert(0, os.path.join(os.path.dirname(FW), "setup_app"))
import build  # noqa: E402
import kunteng_core as kt  # noqa: E402

PWM_HZ = 15625
CPU_HZ = 16_000_000
# ucsim ignores the TIM1 repetition counter: the PWM interrupt comes at every counter turn (31.25 kHz)
# instead of every second one (15.625 kHz). The firmware counts PWM interrupts, so for the firmware
# time in the interrupt runs twice as fast. Hall timing is scaled by this to get the wanted erps.
SIM_ISR_SCALE = 2


def ucsim():
    sdcc = build.find_sdcc()
    p = os.path.join(os.path.dirname(sdcc), "ucsim_stm8.exe" if os.name == "nt" else "ucsim_stm8")
    if not os.path.isfile(p):
        p = os.path.join(os.path.dirname(sdcc), "sstm8" + (".exe" if os.name == "nt" else ""))
    return p


def symbols(map_file):
    syms = {}
    for line in open(map_file):
        m = re.match(r"\s+([0-9A-F]{8})\s+(_\w+)\s", line)
        if m:
            syms[m.group(2)[1:]] = int(m.group(1), 16)
    return syms


def struct_offsets():
    """Offsets inside struct_kt_settings (all uint8 except the uint16 array and ticks)."""
    names = ["cells", "motor_current_a", "regen_current_a", "throttle_type", "flags", "pas_magnets",
             "pas_max_cadence", "a1", "a2", "a3", "a4", "a5", "rotor_offset_angle", "foc_angle_adjust",
             "erps_interpolation", "default_max_speed", "default_wheel_size", "controller_type",
             "phase_current_factor_x10", "display", "ramp_x10", "foc_delay_x4us", "motor_characteristic",
             "fw_angle_deg", "fw_battery_a", "fw_phase_a", "fw_ramp_dps",
             "adc_battery_voltage_max", "adc_battery_voltage_min",
             "communications_battery_voltage"]
    off = {n: i for i, n in enumerate(names)}
    i = len(names)
    off["pack_volts"] = i
    i += 8
    off["pas_max_cadence_ticks"] = i
    off["pas_min_cadence_ticks"] = i + 2
    off["phase_current_max"] = i + 4
    off["fw_angle_max"] = i + 5
    off["fw_battery_x2"] = i + 6
    off["fw_phase_peak"] = i + 7
    off["fw_step_x256"] = i + 8          # uint16, big endian
    off["from_eeprom"] = i + 10
    return off


class Sim:
    def __init__(self, ihx, syms):
        self.ihx, self.syms, self.cmds = ihx, syms, []
        self.dumps = []

    def cmd(self, c):
        self.cmds.append(c)

    def setmem(self, addr, data):
        vals = " ".join("0x%02x" % b for b in data)
        if 0x4000 <= addr < 0x4800:   # the EEPROM is write protected through the address space
            self.cmd("set memory eeprom_chip 0x%x %s" % (addr - 0x4000, vals))
            shown = addr - 0x4000
        else:
            self.cmd("set memory rom 0x%x %s" % (addr, vals))
            shown = addr
        # ucsim prints the written bytes like a dump: consume that output so it is not taken for a later dump
        self.dumps.append((None, shown, len(data)))

    def run_ms(self, ms):
        self.cmd("step %d us" % int(round(ms * 1000)))  # ucsim does not take fractional ms

    def dump(self, tag, name, n):
        a = self.syms[name] if isinstance(name, str) else name
        self.dumps.append((tag, a, n))
        self.cmd("dump rom 0x%x 0x%x 16" % (a, a + n - 1))

    def go(self, timeout=60, uart_in=None):
        """Feed the commands one by one, paced by the simulated time (ucsim drops input that arrives while a
        'step' is still running). uart_in: bytes for UART2 (the display line); returns what the firmware sent
        in self.uart_out."""
        import queue
        import threading
        import tempfile
        args = [ucsim(), "-t", "STM8S105", "-X", "16M"]
        tmp = None
        if uart_in is not None:
            tmp = tempfile.mkdtemp(prefix="ktsim_")
            fin, fout = os.path.join(tmp, "in.bin"), os.path.join(tmp, "out.bin")
            open(fin, "wb").write(bytes(uart_in))
            args += ["-S", "uart=2,in=%s,out=%s" % (fin, fout)]
        p = subprocess.Popen(args + [self.ihx], stdin=subprocess.PIPE,
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        q, out = queue.Queue(), []

        def reader():
            for line in p.stdout:
                q.put(line)
            q.put(None)
        threading.Thread(target=reader, daemon=True).start()

        # ucsim buffers its output in a pipe, so it can't be watched: pace by the simulated time instead
        # (it simulates at roughly 0.15 ... 0.25 x real time on a normal PC, 0.04 is on the safe side)
        import time
        try:
            stepping = False
            for c in self.cmds:
                m = re.match(r"step (\d+) us", c)
                if stepping and c.startswith("dump"):
                    time.sleep(1.0)   # a long run of steps may still be busy: never drop a dump
                p.stdin.write(c + "\n")
                p.stdin.flush()
                stepping = bool(m)
                if m:
                    time.sleep(0.03 + int(m.group(1)) * 1e-6 / 0.04)
            time.sleep(2)  # the last dumps must not be cut off by kill
            p.stdin.write("kill\n")
            p.stdin.close()
            while True:
                line = q.get(timeout=timeout)
                if line is None:
                    break
                out.append(line)
        finally:
            p.kill()
            p.wait()
        if tmp:
            fout = os.path.join(tmp, "out.bin")
            self.uart_out = open(fout, "rb").read() if os.path.isfile(fout) else b""
            self.uart_out = self.uart_out.replace(bytes([13, 13, 10]), bytes([13, 10]))  # ucsim writes 0x0A as 0x0D 0x0A
        self.output = "".join(out)
        return parse_dumps(self.output, self.dumps)


def parse_dumps(out, dumps):
    """Dump results in command order, found by their start address (ucsim prints echo text out of order)."""
    lines = out.splitlines()
    res, pos = {}, 0
    row = re.compile(r"0x([0-9a-f]{3,6}) {6,}((?:[0-9a-f]{2} )+)")  # anywhere in the line: input echo may be mixed in
    for tag, addr, n in dumps:
        data = bytearray()
        start = pos
        while pos < len(lines):
            m = row.search(lines[pos])
            pos += 1
            if m and int(m.group(1), 16) == addr:
                data += bytes(int(x, 16) for x in m.group(2).split())
                while len(data) < n and pos < len(lines):
                    m = row.search(lines[pos])
                    if not m:
                        break
                    data += bytes(int(x, 16) for x in m.group(2).split())
                    pos += 1
                break
        if not data:
            pos = start  # not found: the next dump may still be there
        if tag is not None:
            res[tag] = data[:n]
    return res


def boot(sim, block=None):
    sim.cmd("expression pa_pins=0xff")       # brake released (active low), all port A high
    sim.cmd("expression pd_pins=0xff")       # PAS / over current inputs idle high
    sim.cmd("expression pe_pins=0x04")       # hall state 4
    sim.setmem(0x53F2, [100])                # ADC battery voltage (8 bit), set per test below
    sim.setmem(0x53F0, [0x40, 0x00])         # motor current ADC at zero current: calibrated as offset at start up
    # display values with a valid key, so the firmware does not write EEPROM defaults at start up
    # (ucsim's flash controller never reports "done", the write would wait forever)
    sim.setmem(0x4000, [0xCA, 2, 202, 0x14, 25, 1, 10])
    if block is not None:
        sim.setmem(kt.SETTINGS_BLOCK_ADDR, block)
    sim.run_ms(400)                          # start up (timer2 delays, ADC calibration)


def hall_run(sim, erps, revs, widths=(1, 1, 1, 1, 1, 1)):
    """Turn the 'motor' at erps. widths: relative length of the hall states 4,6,2,3,1,5."""
    order = [4, 6, 2, 3, 1, 5]
    tot = sum(widths)
    per_rev_ms = 1000.0 / erps / SIM_ISR_SCALE
    for _ in range(revs):
        for st, w in zip(order, widths):
            sim.cmd("expression pe_pins=0x%02x" % st)
            sim.run_ms(per_rev_ms * w / tot)


KM_HANDSHAKE_7 = 61    # KM_901U_HANDSHAKE[7]


def sum_xor(b):
    """b = bytes 1 ... 11 of a KT status packet: byte 6 (index 5 here) is the XOR of the others."""
    x = 0
    for i, v in enumerate(b):
        if i != 5:
            x ^= v
    return 0 if x == b[5] else 1


def km_msg(cmd, data):
    m = [0x3A, 0x1A, cmd, len(data)] + list(data)
    sm = sum(m[1:]) & 0xFFFF
    return bytes(m + [sm & 0xFF, sm >> 8, 0x0D, 0x0A])


def km_sum_ok(m):
    n = m[3]
    sm = sum(m[1:4 + n]) & 0xFFFF
    return m[4 + n] == (sm & 0xFF) and m[5 + n] == (sm >> 8)


def xor(b):
    x = 0
    for v in b:
        x ^= v
    return x


def crc16(b):
    c = 0
    for v in b:
        c ^= v << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def vesc_packet(payload):
    c = crc16(payload)
    return bytes([0x02, len(payload)] + list(payload) + [c >> 8, c & 0xFF, 0x03])


def vesc_packets(out):
    """Valid VESC packets in the UART output: [payload, ...]"""
    res, i = [], 0
    while i + 5 <= len(out):
        n = out[i + 1]
        if out[i] == 0x02 and n and i + n + 5 <= len(out) and out[i + n + 4] == 0x03 and \
                crc16(out[i + 2:i + 2 + n]) == (out[i + 2 + n] << 8 | out[i + 3 + n]):
            res.append(out[i + 2:i + 2 + n])
            i += n + 5
        else:
            i += 1
    return res


def display_run(ihx, syms, display, frames, extra_ms=50, dumps=()):
    """Boot with the display protocol, feed the frames (20 ms apart), return (dumps, uart output)."""
    s = kt.default_settings()
    s.update(display=display)
    sim = Sim(ihx, syms)
    boot(sim, kt.settings_block(s))
    sim.run_ms(100)
    pos = 0
    for f in frames:
        pos = feed_display(sim, syms, f, pos)
        sim.run_ms(20)
    sim.run_ms(extra_ms)
    sim.dump("lcd", syms["lcd_configuration_variables"], 6)
    sim.dump("wheel", "ui16_display_wheel_perimeter_mm", 2)
    for tag, name, n in dumps:
        sim.dump(tag, name, n)
    res = sim.go(uart_in=b"")
    return res, sim.uart_out


def feed_display(sim, syms, data, pos):
    """Put bytes into the firmware's UART receive buffer as if the display had sent them (ucsim's UART input
    model does not deliver them). pos: current head position, returns the new one."""
    buf = syms["ui8_rx_buf"]
    for b in data:
        sim.setmem(buf + pos, [b])
        pos = (pos + 1) & 63
    sim.setmem(syms["ui8_rx_head"], [pos])
    return pos


def check(cond, text, fails):
    print(("  ok    " if cond else "  FAIL  ") + text)
    if not cond:
        fails.append(text)


def main():
    # own build directory per run: two runs at the same time used to rebuild kunteng/build/sim under each
    # other, and one of them read the other's map file (wrong addresses, e.g. autodetect "state 6")
    import tempfile
    out = tempfile.mkdtemp(prefix="kt_sim_build_")
    print("building firmware for the simulator (-DSIMULATION) ...")
    ihx = build.build(out, ["SIMULATION"], log=lambda _l: None)
    syms = symbols(os.path.join(out, "main.map"))
    off = struct_offsets()
    fails = []

    # ---------------------------------------------------------------- 1 + 2 + 3
    s = kt.default_settings()
    s.update(cells=13, motor_current_a=18, controller=2, pas_magnets=12, rotor_offset_angle=200, erps_interpolation=15,
             fw_angle_deg=40, fw_battery_a=0, fw_phase_a=10, fw_ramp_dps=60)
    block = kt.settings_block(s)
    sim = Sim(ihx, syms)
    boot(sim, block)
    sim.setmem(0x53F2, [180])   # 13S at ~49 V: 180 * 0.272
    sim.dump("settings", "kt_settings", 52)
    # constant speed, hall sensors evenly spaced
    hall_run(sim, 50, 30)
    sim.dump("erps", "ui16_motor_speed_erps", 2)
    sim.dump("commutation", "ui8_motor_commutation_type", 1)
    sim.dump("step", "ui16_angle_step_x256", 2)
    sim.dump("period", "ui16_period_sum", 2)
    sim.dump("offsets_even", "i8_hall_offset", 8)
    # now hall sensors misplaced: state 6 starts 10/256 late, state 3 starts 8/256 early
    # widths in 1/256 of a revolution for states 4,6,2,3,1,5 (nominal 42.67 each)
    w = [52.67, 32.67, 34.67, 50.67, 42.67, 42.67]
    hall_run(sim, 50, 200, w)
    sim.dump("offsets_learned", syms["i8_hall_offset"] - 1, 9)   # one byte earlier: not the same address as the dump above
    print("running the simulation (takes a minute) ...")
    r = sim.go()

    st = r.get("settings", b"")
    print("settings block -> kt_settings")
    check(len(st) >= 52 and st[off["from_eeprom"]] == 1, "settings block accepted", fails)
    if len(st) >= 52:
        check(st[off["cells"]] == 13 and st[off["motor_current_a"]] == 18 and st[off["pas_magnets"]] == 12,
              "cells 13, current 18 A, 12 PAS magnets", fails)
        check(st[off["adc_battery_voltage_max"]] == (13 * 420) // 27, "battery max from cells (%d)" % st[off["adc_battery_voltage_max"]], fails)
        check(st[off["communications_battery_voltage"]] == 48, "display voltage 48 V", fails)
        check(st[off["phase_current_max"]] == 90, "phase current limit 18 A * 2.5 = 90 steps (%d)" % st[off["phase_current_max"]], fails)
        fw = (st[off["fw_angle_max"]], st[off["fw_battery_x2"]], st[off["fw_phase_peak"]],
              int.from_bytes(st[off["fw_step_x256"]:off["fw_step_x256"] + 2], "big"))
        check(fw == (28, 36, 83, 109), "field weakening 40 deg -> 28/256, battery 0 -> 18 A = 36 steps, phase 10 A -> 83 "
              "steps, 60 deg/s -> 109 per 10 ms %s" % (fw,), fails)
    erps = int.from_bytes(r.get("erps", b"\0\0")[:2], "little")
    print("hall sequence at 50 erps")
    # SDCC stores 16 bit values big endian
    erps = int.from_bytes(r.get("erps", b"\0\0")[:2], "big")
    check(48 <= erps <= 52, "speed %d erps" % erps, fails)
    check(r.get("commutation", b"\0")[0] == 2, "sine wave interpolation active", fails)
    period = int.from_bytes(r.get("period", b"\0\0")[:2], "big")
    step = int.from_bytes(r.get("step", b"\0\0")[:2], "big")
    check(abs(period - PWM_HZ / 50) < 8, "6 sector sum %d PWM cycles (expected %d)" % (period, PWM_HZ // 50), fails)
    check(step == 0xFFFF // period if period else False, "angle step %d per PWM cycle" % step, fails)
    even = [int.from_bytes([b], "big", signed=True) for b in r.get("offsets_even", bytes(8))]
    check(all(abs(even[i]) <= 1 for i in range(1, 7)), "evenly spaced halls -> offsets ~0 %s" % even[1:7], fails)
    lr = [int.from_bytes([b], "big", signed=True) for b in r.get("offsets_learned", bytes(9))[1:]]
    # expected edge errors (state: offset) relative to the mean
    pos, exp = 0.0, {}
    nominal = {4: 0, 6: 42, 2: 85, 3: 127, 1: 170, 5: 212}
    for stt, wd in zip([4, 6, 2, 3, 1, 5], w):
        exp[stt] = pos - nominal[stt]
        pos += wd
    mean = sum(exp.values()) / 6
    exp = {k: v - mean for k, v in exp.items()}
    print("misplaced halls, learned offsets (state: learned / real)")
    for stt in [4, 6, 2, 3, 1, 5]:
        print("        %d: %+d / %+.1f" % (stt, lr[stt], exp[stt]))
    check(all(abs(lr[k] - exp[k]) <= 2.5 for k in exp), "learned offsets within 2.5/256 (3.5 deg) of the real ones", fails)

    # ---------------------------------------------------------------- 4: no block -> defaults
    sim = Sim(ihx, syms)
    boot(sim, None)
    sim.dump("settings", "kt_settings", 52)
    r = sim.go()
    st = r.get("settings", b"")
    print("no settings block")
    check(len(st) >= 52 and st[off["from_eeprom"]] == 0 and st[off["cells"]] == 7,
          "compile time defaults used (7S)", fails)

    # ---------------------------------------------------------------- 5: phase current limit
    # 10 A battery current at a low duty cycle means a high phase current: the limit must pull the duty
    # cycle down. Same situation with the limit switched off in the settings: duty cycle stays.
    print("phase current limit (10 A battery current, duty cycle 100/508)")
    duty_after = {}
    for limit_on in (True, False):
        s = kt.default_settings()
        s.update(cells=10, motor_current_a=15, phase_current_limit=limit_on)
        sim = Sim(ihx, syms)
        boot(sim, kt.settings_block(s))
        sim.setmem(0x53F2, [140])                    # 10S at 38 V
        sim.setmem(0x53F0, [0x40 + 20, 0])           # total current: 20 steps of 0.5 A above the zero offset
        sim.setmem(syms["ui16_duty_cycle"], [0, 100])
        sim.setmem(syms["ui16_duty_cycle_target"], [0, 100])
        sim.run_ms(2)                                # ~60 PWM interrupts in the simulator
        sim.dump("duty", "ui16_duty_cycle", 2)
        r = sim.go()
        duty_after[limit_on] = int.from_bytes(r.get("duty", b"\0\0")[:2], "big")
    print("        duty cycle after 2 ms: limit on %d, limit off %d" % (duty_after[True], duty_after[False]))
    check(duty_after[True] < 80, "limit on: duty cycle reduced", fails)
    check(duty_after[False] >= 95, "limit off: duty cycle kept", fails)

    # ---------------------------------------------------------------- 6: display protocols over UART2
    # the input file is fed from the start at the line speed: zero bytes as padding until the firmware listens
    lcd = syms["lcd_configuration_variables"]   # assist, motor characteristic, wheel, max speed, mode, current

    print("Kunteng display: status every 150 ms")
    sim = Sim(ihx, syms)
    boot(sim, kt.settings_block(kt.default_settings()))
    sim.run_ms(500)
    sim.go(uart_in=b"")
    out = sim.uart_out
    pk = [out[i:i + 12] for i in range(out.find(b"\x41"), len(out) - 11, 12)] if b"\x41" in out else []
    ok = bool(pk) and all(q[0] == 65 and (sum_xor(q[1:12]) == 0) for q in pk)
    check(ok and len(pk) >= 3, "%d status packets, 12 bytes, start 65, CRC ok" % len(pk), fails)
    if pk:
        check((pk[-1][3] << 8 | pk[-1][4]) == 65000, "standstill wheel period 65000 ms (was 74250 wrapped to 8714)", fails)

    print("KingMeter 901U / KM5S display")
    s = kt.default_settings()
    s.update(display=1)
    sim = Sim(ihx, syms)
    boot(sim, kt.settings_block(s))
    km53 = km_msg(0x53, [0x00, 0x05, 0x00, 0x00, 0x07, 0x07, 0x80, 32, 0x08, 0x2C])   # handshake index 7, 32 km/h, 2092 mm
    km52 = km_msg(0x52, [204, 0x00])                                                 # assist 204/255 -> level 4
    sim.run_ms(100)
    pos = feed_display(sim, syms, km53, 0)
    sim.run_ms(50)
    feed_display(sim, syms, km52, pos)
    sim.run_ms(50)
    sim.dump("lcd", lcd, 6)
    sim.dump("wheel", "ui16_display_wheel_perimeter_mm", 2)
    res = sim.go(uart_in=b"")
    out = sim.uart_out
    a53 = out.find(b"\x3a\x1a\x53")
    a52 = out.find(b"\x3a\x1a\x52")
    check(a53 >= 0 and len(out) >= a53 + 13 and out[a53 + 7] == KM_HANDSHAKE_7 and km_sum_ok(out[a53:a53 + 13]),
          "settings answer with handshake byte %d and checksum" % KM_HANDSHAKE_7, fails)
    check(a52 >= 0 and len(out) >= a52 + 13 and km_sum_ok(out[a52:a52 + 13]) and out[a52 + 11:a52 + 13] == b"\r\n",
          "operation answer, checksum, CR LF", fails)
    lv = res.get("lcd", b"")
    wh = int.from_bytes(res.get("wheel", b"\0\0")[:2], "big")
    check(len(lv) >= 4 and lv[0] == 4 and lv[3] == 32 and wh == 2092,
          "assist level 4, speed limit 32 km/h, wheel 2092 mm taken over (%s, %d)" % (list(lv[:4]), wh), fails)

    print("Bafang 850C / 860C display")
    s = kt.default_settings()
    s.update(display=3)
    sim = Sim(ihx, syms)
    boot(sim, kt.settings_block(s))
    sim.run_ms(100)
    pos = 0
    for msg in ([0x16, 0x0B, 0x17, (0x16 + 0x0B + 0x17) & 0xFF], [0x11, 0x08], [0x11, 0x20], [0x11, 0x11]):
        pos = feed_display(sim, syms, msg, pos)
        sim.run_ms(20)
    sim.dump("lcd", lcd, 6)
    res = sim.go(uart_in=b"")
    out = sim.uart_out
    lv = res.get("lcd", b"")
    check(len(lv) >= 1 and lv[0] == 3, "assist level code 23 -> level 3 (%s)" % list(lv[:1]), fails)
    check(len(out) >= 6 and out[0] == 0x01 and out[3] == (out[1] + out[2] + 32) & 0xFF and out[4] == out[5],
          "status, speed (with checksum), battery answers: %s" % list(out[:6]), fails)

    print("KingMeter J-LCD / 618U display")
    f = [0x3A, 0x83, (15 << 3) | 5, 0x00]                  # light + level 3, 25 km/h, 26" (2075 mm)
    res, out = display_run(ihx, syms, 4, [bytes(f + [xor(f[1:4])])])
    lv, wh = res.get("lcd", b""), int.from_bytes(res.get("wheel", b"\0\0")[:2], "big")
    check(len(lv) >= 4 and lv[0] == 3 and lv[3] == 25 and wh == 2075,
          "level 3, 25 km/h, wheel 2075 mm (%s, %d)" % (list(lv[:4]), wh), fails)
    a = out.find(b"\x46")
    check(a >= 0 and len(out) >= a + 8 and xor(out[a + 1:a + 7]) == out[a + 7] and out[a + 3:a + 5] == b"\x0d\xac",
          "answer 0x46, XOR ok, wheel stopped 0x0DAC: %s" % list(out[a:a + 8]), fails)

    print("UART No.2 display (S866, EN06)")
    f = [0x01, 0x14, 0, 0, 4, 0, 0, 0x01, 0x04, 0, 0, 0, 32, 0, 0, 0, 0, 0, 0]   # level 4, 26.0", 32 km/h
    res, out = display_run(ihx, syms, 5, [bytes(f + [xor(f)])])
    lv, wh = res.get("lcd", b""), int.from_bytes(res.get("wheel", b"\0\0")[:2], "big")
    check(len(lv) >= 4 and lv[0] == 4 and lv[3] == 32 and wh == 2074,
          "level 4, 32 km/h, wheel 26.0 inch = 2074 mm (%s, %d)" % (list(lv[:4]), wh), fails)
    a = out.find(b"\x02\x0e\x01")
    check(a >= 0 and len(out) >= a + 14 and xor(out[a:a + 13]) == out[a + 13] and out[a + 12] == 0xFF,
          "answer 14 bytes 02 0E 01, XOR ok: %s" % list(out[a:a + 14]), fails)

    print("Tongsheng display (VLCD5 / VLCD6 / XH18)")
    f = [0x59, 0x40, 0, 28, 0, 30]                         # level 2, 28", 30 km/h
    res, out = display_run(ihx, syms, 6, [bytes(f + [sum(f) & 0xFF])])
    lv, wh = res.get("lcd", b""), int.from_bytes(res.get("wheel", b"\0\0")[:2], "big")
    check(len(lv) >= 4 and lv[0] == 2 and lv[3] == 30 and wh == 2240,
          "level 2, 30 km/h, wheel 28 inch = 2240 mm (%s, %d)" % (list(lv[:4]), wh), fails)
    a = out.find(b"\x43")
    check(a >= 0 and len(out) >= a + 9 and (sum(out[a:a + 8]) & 0xFF) == out[a + 8] and out[a + 6:a + 8] == b"\x00\x07",
          "answer 0x43, sum ok, wheel stopped 0x0700: %s" % list(out[a:a + 9]), fails)

    print("VESC UART (115200 baud)")
    res, out = display_run(ihx, syms, 7, [vesc_packet([0]), vesc_packet([4]),
                                          vesc_packet([6, 0, 0, 0x13, 0x88])],  # 5000 mA
                           extra_ms=400, dumps=[("target", "ui16_target_current_10b", 2),
                                                ("vbat", syms["ui8_adc_battery_voltage_filtered"[:31]], 1)])
    vb = res.get("vbat", b"\0")[0]     # still settling (1/64 filter every 100 ms): compare with +-3 steps
    pk = vesc_packets(out)
    fw_ = [p for p in pk if p[0] == 0]
    val = [p for p in pk if p[0] == 4]
    check(len(fw_) == 1 and len(fw_[0]) == 28 and fw_[0][3:11] == b"EBiCS-KT",
          "COMM_FW_VERSION answer, CRC ok, name EBiCS-KT", fails)
    check(len(val) == 1 and len(val[0]) == 74, "COMM_GET_VALUES answer, 74 bytes, CRC ok", fails)
    if val:
        v = int.from_bytes(val[0][27:29], "big")
        check(abs(v - vb * 2.72) <= 9, "input voltage in 0.1 V = battery ADC x 0.272 V (%d, ADC %d)" % (v, vb), fails)
    tg = int.from_bytes(res.get("target", b"\0\0")[:2], "big")
    check(tg == 40, "COMM_SET_CURRENT 5 A -> target current 40 x 0.125 A (%d)" % tg, fails)

    print("EBiCS app / BLE module (ANT+ LEV pages)")
    f = [164, 12, 0x4E, 16, 0x2C, 0x08, 4 << 3, 0, 0, 0, 0]   # 2092 mm, level 4
    res, out = display_run(ihx, syms, 8, [bytes(f + [xor(f)])], extra_ms=400,
                           dumps=[("vbat", syms["ui8_adc_battery_voltage_filtered"[:31]], 1)])
    vb = res.get("vbat", b"\0")[0]
    lv, wh = res.get("lcd", b""), int.from_bytes(res.get("wheel", b"\0\0")[:2], "big")
    check(len(lv) >= 1 and lv[0] == 4 and wh == 2092, "level 4, wheel 2092 mm (%s, %d)" % (list(lv[:1]), wh), fails)
    pages = [out[i:i + 12] for i in range(len(out) - 11) if out[i] == 164 and out[i + 1] == 12 and out[i + 2] == 0x4E
             and xor(out[i:i + 11]) == out[i + 11]]
    got = sorted(set(p[3] for p in pages))
    p1 = [p for p in pages if p[3] == 1]   # pages before the app's frame still show level 0
    check(got == [1, 4] and p1[-1][5] == 4 << 3 and
          all(abs(p[8] - vb * 1.088) <= 4 for p in pages if p[3] == 4),
          "pages 1 and 4 in turn, XOR ok, level 4, battery in 0.25 V (%d pages, %s, ADC %d)"
          % (len(pages), [p[8] for p in pages if p[3] == 4], vb), fails)

    # ---------------------------------------------------------------- 7: learned values from the EEPROM
    print("learned hall offsets and back-EMF constant loaded at power on")
    blk = [ord("L"), 3, (-2) & 0xFF, 1, (-4) & 0xFF, 2, 0, 0x01, 0x2C]
    blk.append((~sum(blk)) & 0xFF)
    sim = Sim(ihx, syms)
    sim.setmem(0x4100, blk)
    boot(sim, kt.settings_block(kt.default_settings()))
    sim.dump("offs", "i8_hall_offset", 8)
    sim.dump("emf", "ui16_emf_duty_per_erps_x256", 2)
    res = sim.go()
    offs = [int.from_bytes([b], "big", signed=True) for b in res.get("offs", bytes(8))]
    emf = int.from_bytes(res.get("emf", b"\0\0")[:2], "big")
    check([offs[st] for st in [4, 6, 2, 3, 1, 5]] == [3, -2, 1, -4, 2, 0] and emf == 300,
          "offsets %s, back-EMF constant %d (old 10 byte block)" % ([offs[st] for st in [4, 6, 2, 3, 1, 5]], emf), fails)

    print("measured rotor offset from the new 12 byte block replaces the setting")
    blk = [ord("L"), 1, 0, 0, 0, 0, 0, 0x01, 0x00, 0xA5, 177]
    blk.append((~sum(blk)) & 0xFF)
    sim = Sim(ihx, syms)
    sim.setmem(0x4100, blk)
    boot(sim, kt.settings_block(kt.default_settings()))
    sim.dump("settings", "kt_settings", 13)
    res = sim.go()
    st = res.get("settings", b"")
    check(len(st) >= 13 and st[12] == 177, "rotor offset 177 instead of 202 (%s)" % (list(st[12:13])), fails)

    # ---------------------------------------------------------------- 8: hall autodetect
    # SIMULATION build: hall states from the forced field angle with true rotor offset 190, edge errors
    # +3 -2 +4 -3 +2 -4 (states 4 6 2 3 1 5) and 4/256 lag in the direction of travel (kunteng/motor.c sim_hall)
    print("hall autodetect, forward + backward (simulated halls: offset 190, edge errors, 4/256 lag)")
    sim = Sim(ihx, syms)
    sim.setmem(0x53E8, [0])          # throttle released
    boot(sim, kt.settings_block(kt.default_settings()))
    sim.setmem(0x53E8, [0])
    sim.setmem(0x53F2, [140])        # 10S at 38 V
    sim.run_ms(400)
    sim.setmem(syms["ui8_ad_cmd"], [1])
    for _ in range(14):
        sim.run_ms(500)
    for n, k in (("ui8_ad_state", 1), ("ui8_ad_spread", 1), ("ui8_ad_rotor_offset", 1), ("ui8_ad_fwd", 8),
                 ("i8_hall_offset", 8)):
        sim.dump(n, n, k)
    res = sim.go(timeout=120)
    state = res.get("ui8_ad_state", b"\0")[0] if res.get("ui8_ad_state") else -1
    offs = [int.from_bytes([b], "big", signed=True) for b in res.get("i8_hall_offset", bytes(8))]
    check(state == 3, "autodetect finished (state %d)" % state, fails)
    check(res.get("ui8_ad_rotor_offset", b"\0")[:1] == bytes([190]),
          "rotor offset %s (true 190, forward only would be ~194)" % list(res.get("ui8_ad_rotor_offset", b"")), fails)
    check([offs[s] for s in (4, 6, 2, 3, 1, 5)] == [3, -2, 4, -3, 2, -4],
          "edge offsets %s (true +3 -2 +4 -3 +2 -4)" % [offs[s] for s in (4, 6, 2, 3, 1, 5)], fails)
    check(8 <= (res.get("ui8_ad_spread", b"\0")[0]) <= 10, "spread %s/256 (2 x 4 lag)" % list(res.get("ui8_ad_spread", b"")), fails)

    print()
    print("ALL OK" if not fails else "%d FAILED" % len(fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
