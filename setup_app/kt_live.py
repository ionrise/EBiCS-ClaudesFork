"""Live data from a running KT firmware over SWIM (no reset, the motor keeps running).

The addresses come from the SDCC map file of the build (kunteng/build/main.map) or, for the prebuilt firmware
of the Android app, from kunteng.sym.json. SDCC stores multi byte values big endian.
"""
import re
import struct

import swim

# name in the firmware, type, label, unit, scale
LIVE_VARS = [
    ("ui8_adc_battery_voltage_filtered", "u8", "Battery", "V", 0.272),
    ("i16_motor_current_filtered_10b", "i16", "Battery current", "A", 0.125),
    ("ui16_motor_speed_erps", "u16", "Motor speed", "erps", 1),
    ("f_wheel_speed", "f32", "Speed", "km/h", 1),
    ("ui16_duty_cycle", "u16", "Duty cycle", "/508", 1),
    ("ui8_adc_throttle_value", "u8", "Throttle (ADC)", "", 1),
    ("ui8_pas_cadence_rpm", "u8", "Cadence", "rpm", 1),
    ("lcd_configuration_variables", "u8", "Assist level", "", 1),
    ("ui8_motor_controller_state", "u8", "Controller state", "", 1),
    ("ui8_motor_controller_error", "u8", "Error code", "", 1),
    ("ui8_motor_state", "u8", "Motor state", "", 1),
    ("ui8_motor_commutation_type", "u8", "Commutation", "", 1),
    ("ui8_hall_sensors_last", "u8", "Hall state", "", 1),
    ("ui8_angle_correction", "u8", "FOC angle correction", "/256", 1),
    ("ui8_phase_b_peak", "u8", "Phase current peak", "A", 0.121),
    ("ui8_coasting", "u8", "Coasting (PWM off)", "", 1),
    ("ui16_emf_duty_per_erps_x256", "u16", "Back-EMF constant", "", 1),
    ("i8_hall_offset", "i8x8", "Learned hall offsets", "/256", 1),
    ("ui8_fw_angle", "u8", "Field weakening angle", "deg", 360 / 256),
    ("ui8_phase_fault", "u8", "Phase test fault (bit 0..2 = axis A, B, C)", "", 1),
]
SIZES = {"u8": 1, "i8": 1, "u16": 2, "i16": 2, "f32": 4, "i8x8": 8, "u8x8": 8}

# hall autodetect (kunteng/motor.c motor_autodetect): the app writes 1 into ui8_ad_cmd
AD_VARS = [("ui8_ad_cmd", "u8"), ("ui8_ad_state", "u8"), ("ui8_ad_progress", "u8"), ("ui8_ad_spread", "u8"),
           ("ui8_ad_rotor_offset", "u8"), ("ui8_ad_fwd", "u8x8"), ("ui8_ad_bwd", "u8x8")]
AD_STATES = {0: "idle", 1: "turning forward", 2: "turning backward", 3: "done", 4: "failed"}
HALL_ORDER = (4, 6, 2, 3, 1, 5)

STATE_BITS = [(2, "brake"), (4, "over current"), (8, "under voltage"), (32, "motor blocked"), (64, "over voltage"),
              (128, "hall error")]
MOTOR_STATES = {0: "coast", 1: "stop", 2: "start up", 3: "cool down", 4: "running"}
ERRORS = {0: "-", 0x21: "21 over current", 0x22: "22 throttle", 0x24: "24 hall sensors", 0x28: "over voltage",
          0x91: "battery empty"}


def read_map(path):
    """SDCC .map -> {name: address}. Names are cut at 31 characters in the map."""
    syms = {}
    with open(path) as f:
        for line in f:
            m = re.match(r"\s+([0-9A-F]{8})\s+_(\w+)\s", line)
            if m:
                syms[m.group(2)] = int(m.group(1), 16)
    return syms


def lookup(syms, name):
    if name in syms:
        return syms[name]
    return syms.get(name[:31])


def live_symbols(syms):
    """Live and autodetect variables, {name: [address, size]} (also what goes into kunteng.sym.json)."""
    out = {}
    for name, typ in AD_VARS:
        a = lookup(syms, name)
        if a is not None:
            out[name] = [a, SIZES[typ]]
    for name, typ, *_ in LIVE_VARS:
        a = lookup(syms, name)
        if a is not None:
            out[name] = [a, SIZES[typ]]
    return out


def decode(typ, raw):
    if typ == "u8":
        return raw[0]
    if typ == "i8":
        return struct.unpack(">b", raw[:1])[0]
    if typ == "u16":
        return struct.unpack(">H", raw[:2])[0]
    if typ == "i16":
        return struct.unpack(">h", raw[:2])[0]
    if typ == "f32":
        return struct.unpack(">f", raw[:4])[0]
    if typ == "i8x8":
        return [struct.unpack(">b", bytes([b]))[0] for b in raw[:8]]
    if typ == "u8x8":
        return list(raw[:8])
    raise ValueError(typ)


class LiveSession:
    """Keeps a SWIM connection to the running controller and reads the live variables."""

    def __init__(self, symbols, transport=None):
        self.symbols = symbols          # {name: [address, size]}
        self.s = swim.Swim(transport)
        self.open = False

    def connect(self):
        self.s.open_live()
        self.open = True

    def release_stall(self):
        """The live link runs with SWIM_DM on (reads fail without it). If the CPU got stalled (seen on a real KT
        controller while the motor ran: SWIM disturbed by the motor current), the PWM keeps its last duty cycle and
        the firmware no longer controls the motor: clear the stall bit at once."""
        d = self.s.read_byte(swim.DM_CSR2)
        if d & 0x08:
            self.s.write_byte(swim.DM_CSR2, d & ~0x08)
            return True
        return False

    def snapshot(self):
        if not self.open:
            self.connect()
        self.release_stall()
        snap = {}
        for name, typ, label, unit, scale in LIVE_VARS:
            if name not in self.symbols:
                continue
            addr, size = self.symbols[name]
            v = decode(typ, self.s.read_mem(addr, size))
            snap[name] = v if typ == "i8x8" else v * scale
        return snap

    def close(self):
        if self.open:
            self.s.close()
        self.open = False

    def read_vars(self, names_types):
        if not self.open:
            self.connect()
        self.release_stall()
        out = {}
        for name, typ in names_types:
            if name in self.symbols:
                addr, size = self.symbols[name]
                out[name] = decode(typ, self.s.read_mem(addr, size))
        return out

    def autodetect(self, log=print, sleep=None, timeout_s=20.0):
        """Starts the hall autodetect and waits for the result (dict of the AD_VARS values)."""
        import time
        sleep = sleep or time.sleep
        if "ui8_ad_cmd" not in self.symbols:
            raise swim.SwimError("this firmware has no hall autodetect (install the current firmware)")
        if not self.open:
            self.connect()
        self.s.write_byte(self.symbols["ui8_ad_cmd"][0], 1)
        last = -1
        t = 0.0
        res = {}
        while t < timeout_s:
            sleep(0.3)
            t += 0.3
            res = self.read_vars(AD_VARS)
            if res.get("ui8_ad_cmd") == 1 and t > 1.0:
                raise swim.SwimError("the firmware did not start the autodetect (motor turning, throttle pressed or "
                                     "a fault active?)")
            p = res.get("ui8_ad_progress", 0)
            if p // 10 != last and res.get("ui8_ad_state") in (1, 2):
                last = p // 10
                log("  %s ... %d %%" % (AD_STATES.get(res.get("ui8_ad_state")), p))
            if res.get("ui8_ad_state") in (3, 4) and res.get("ui8_ad_cmd") == 0:
                return res
        raise swim.SwimError("no result after %d s" % timeout_s)


def describe_autodetect(res):
    """Text lines for the autodetect result."""
    state = res.get("ui8_ad_state")
    if state != 3:
        return ["Autodetect FAILED: not all hall edges seen in both directions, edges more than 28 deg off 60 deg, "
                "or the motor did not follow (lift the wheel, release the brake, check the hall wires)."]
    deg = 360.0 / 256
    lines = ["Hall autodetect done (angles in deg, 0 = state 4 edge):",
             "  hall state   forward  backward  average  difference"]
    fwd, bwd = res.get("ui8_ad_fwd", [0] * 8), res.get("ui8_ad_bwd", [0] * 8)
    for st in HALL_ORDER:
        d = ((bwd[st] - fwd[st] + 128) & 0xFF) - 128
        avg = (fwd[st] + int(d / 2)) & 0xFF
        lines.append("  %d            %6.1f    %6.1f   %6.1f    %+5.1f" % (st, fwd[st] * deg, bwd[st] * deg, avg * deg, d * deg))
    spread = res.get("ui8_ad_spread", 0) * deg
    lines.append("Rotor offset: %d/256 (setting default 202). Largest difference %.0f deg: %s" % (
        res.get("ui8_ad_rotor_offset", 0), spread,
        "good" if spread <= 15 else "the motor turns stiffly (brake, freewheel?)" if spread <= 40 else "very large"))
    lines.append("Saved in the controller, used from now on (also after power off).")
    return lines


def describe(snap):
    """[(label, text)] for the live page."""
    rows = []
    for name, typ, label, unit, scale in LIVE_VARS:
        if name not in snap:
            continue
        v = snap[name]
        if name == "ui8_motor_controller_state":
            bits = [t for b, t in STATE_BITS if int(v) & b]
            text = "ok" if int(v) == 1 else ", ".join(bits) or str(int(v))
        elif name == "ui8_motor_state":
            text = MOTOR_STATES.get(int(v), str(int(v)))
        elif name == "ui8_motor_controller_error":
            text = ERRORS.get(int(v), "%02x" % int(v))
        elif name == "ui8_motor_commutation_type":
            text = {1: "block (low speed)", 2: "sine wave"}.get(int(v), str(int(v)))
        elif name == "i8_hall_offset":
            text = " ".join("%+d" % v[st] for st in (4, 6, 2, 3, 1, 5))
        elif name == "ui8_coasting":
            text = "yes" if v else "no"
        elif isinstance(v, float) and scale != 1 or typ == "f32":
            text = "%.2f %s" % (v, unit)
        else:
            text = "%d %s" % (v, unit)
        rows.append((label, text.strip()))
    return rows
