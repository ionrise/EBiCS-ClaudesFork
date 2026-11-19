// EBiCS bike settings + the 256 byte runtime settings block (Inc/settings.h).
// Same keys, defaults and conversions as setup_app/ebics_core.py and android_app/lib/firmware/*.dart;
// checked against android_app/assets/firmware/test_vectors.json (see esp32_flasher/README.md).
// Part of EBiCS, GPL v3 or later.
"use strict";

const EB = (() => {
  const DISPLAYS = [
    ["KM5S", "KingMeter 5S / KM5S / 901U (recommended)"], ["JLCD", "KingMeter J-LCD / 618U"],
    ["BAFANG_LCD", "Bafang LCD (1200 baud)"], ["BAFANG_TFT", "Bafang 850C / 860C (9600 baud)"],
    ["KUNTENG", "Kunteng KT-LCD3 / LCD5 / LCD8 protocol"], ["NO2", "UART No.2 (S866, EN06 ...)"],
    ["TONGSHENG", "Tongsheng VLCD5 / VLCD6 / XH18"], ["VESC", "VESC UART (115200 baud)"],
    ["EBICS", "EBiCS app / BLE module"], ["DEBUG", "No display (debug text on UART)"]];
  const WHEELS = [['16"', 1270], ['20"', 1590], ['24"', 1900], ['26"', 2070], ['27.5"', 2200],
    ['28" / 700C', 2240], ['29"', 2290], ["custom", null]];
  const BATTERIES = [["24 V (7S)", 7], ["36 V (10S)", 10], ["48 V (13S)", 13], ["52 V (14S)", 14], ["custom", 0]];
  const SPEED_TEMP = [["not used", 0], ["AD1 (PB1)", 6], ["AD2 (PB0)", 5]];

  // key, label, kind, default, section, unit, help
  const FIELDS = [
    ["display", "Display protocol", "display", "KM5S", "basic", "", "Which protocol your display speaks. Multi-protocol displays: choose the same in the display menu."],
    ["battery", "Battery", "battery", 10, "basic", "", "Nominal battery voltage. Sets the low-voltage cutoff and the battery bars."],
    ["full_voltage_v", "Full charge voltage (0 = automatic)", "float", 0.0, "basic", "V", "Battery voltage when fully charged. Regen is ramped down to zero at this voltage. 0 = cells x 4.2 V. Must be set for a custom battery."],
    ["voltage_min_v", "Cutoff voltage (0 = automatic)", "float", 0.0, "basic", "V", "Below this voltage the motor stops (soft ramp just above it). 0 = cells x 3.1 V. Must be set for a custom battery."],
    ["wheel", "Wheel size", "wheel", '28" / 700C', "basic", "", "Used for speed and the speed limit."],
    ["wheel_mm", "Wheel circumference", "int", 2240, "basic", "mm", "Only used when wheel size is 'custom'. Roll the bike one turn and measure."],
    ["speed_limit", "Speed limit", "int", 25, "basic", "km/h", "Assist stops above this speed."],
    ["phase_current_a", "Motor current max", "float", 15.0, "basic", "A", "Maximum motor phase current = motor power. Start low (10-15 A) and increase slowly."],
    ["battery_current_a", "Battery current max", "float", 15.0, "basic", "A", "Maximum current taken from the battery. Must not exceed what your battery/BMS allows."],
    ["pas_magnets", "Pedal sensor magnets", "int", 12, "basic", "", "Number of magnets/pulses per crank turn of the pedal (PAS) sensor. Usually 12 or 32."],
    ["torque_sensor", "Torque sensor", "bool", false, "basic", "", "On if you have a torque sensor instead of a plain cadence PAS sensor."],
    ["throttle", "Throttle", "bool", true, "basic", "", "On if a throttle is connected."],
    ["legal_mode", "Legal mode (EU)", "bool", false, "basic", "", "Ramps the motor down at the speed limit and limits to 6 km/h when you don't pedal."],
    ["current_ramp_ms", "Current ramp", "int", 300, "basic", "ms", "Time to rise from zero to the full motor current: smoother take-off, easier on gears and freewheel. The current drops immediately when you release the throttle or brake. 0 = no ramp."],
    ["walk_current_a", "Walk assist current", "float", 6.0, "basic", "A", "Motor current for the walk / push assist button."],
    ["sensorless", "Sensorless motor (no hall sensors) - experimental", "bool", false, "basic", "", "Run the motor without hall sensors. Hall calibration and gearing measurement are not possible. Test carefully."],
    ["reverse", "Reverse motor direction", "bool", false, "basic", "", "On if the wheel turns backwards. The setup asks you and sets this."],
    ["gear_ratio", "Pole pairs x gear ratio", "int", 60, "basic", "", "Motor electrical turns per wheel turn. The setup measures it when you turn the wheel by hand."],
    ["speed_sensor_external", "External speed sensor", "bool", false, "advanced", "", "Use a spoke magnet speed sensor instead of the motor hall sensors for speed."],
    ["pulses_per_rev", "Speed sensor magnets", "int", 1, "advanced", "", "Magnets on the wheel for the external speed sensor."],
    ["speed_temp_input", "Speed/temperature wire on", "speedtemp", 0, "advanced", "", "Motors with one wire for speed sensor AND temperature (NTC to GND): connect it to AD1 or AD2 with a pull-up to 3.3 V."],
    ["speed_on_wire", "  use it for speed", "bool", true, "advanced", "", "Speed pulses from this wire. Sets the speed source to external."],
    ["temp_on_wire", "  use it for motor temperature", "bool", true, "advanced", "", "Motor temperature from the NTC on this wire."],
    ["ntc_pullup_ohm", "  NTC pull-up resistor", "int", 3000, "advanced", "ohm", "Pull-up resistor from the input to 3.3 V. NTC: 10k B3950."],
    ["speed_temp_threshold", "  speed pulse threshold", "int", 100, "advanced", "ADC", "Readings below = speed pulse, above = temperature."],
    ["motor_temp_derate", "Motor temp: derating start", "int", 100, "advanced", "°C", "Motor current is reduced from here ..."],
    ["motor_temp_max", "Motor temp: maximum", "int", 130, "advanced", "°C", "... down to zero at this temperature."],
    ["torque_on_ad1", "Torque signal on AD1", "bool", false, "advanced", "", "Torque sensor wired to AD1 instead of the throttle input."],
    ["ts_coef", "Torque sensor gain", "int", 60000, "advanced", "", "Higher = more assist per torque."],
    ["pas_timeout", "PAS stop timeout", "int", 8000, "advanced", "ticks", "Motor stops after this time without PAS pulse. 8000 = 1 s."],
    ["ramp_end", "PAS full assist at", "int", 1600, "advanced", "ticks", "PAS pulse interval for full assist (8 ticks = 1 ms)."],
    ["dirdet", "Stop on backpedalling", "bool", false, "advanced", "", "PAS direction detection (needs an asymmetric PAS sensor)."],
    ["frac_high", "Direction window max", "int", 30, "advanced", "", ""],
    ["frac_low", "Direction window min", "int", 15, "advanced", "", ""],
    ["throttle_offset", "Throttle min (ADC)", "int", 750, "advanced", "", "Default only: measured automatically at power-up."],
    ["throttle_max", "Throttle max (ADC)", "int", 2600, "advanced", "", "ADC value at full throttle (see Live data)."],
    ["regen_current_a", "Brake regen current", "float", 0.0, "advanced", "A", "Phase current for regenerative braking (0 = off)."],
    ["regen_max_a", "Regen max into battery", "float", 0.0, "advanced", "A", ""],
    ["linear_regen", "Proportional regen on AD2", "bool", false, "advanced", "", "Analog brake sensor on AD2."],
    ["voltage_ramp_v", "Soft voltage ramp width", "float", 1.0, "advanced", "V", "Current is reduced smoothly within this voltage above the cutoff and below full charge."],
    ["inductance", "Sensorless: inductance", "str", "11", "expert", "", "Observer inductance = phase inductance [uH] x 0.0437. Phase value = half of what you measure between two motor wires. 250 uH -> 11. Only used in sensorless mode."],
    ["resistance", "Sensorless: resistance", "str", "51", "expert", "", "Observer resistance = phase resistance [mOhm] x 0.341. Phase value = half of the resistance between two motor wires. 150 mOhm -> 51. Only used in sensorless mode."],
    ["flux_linkage", "Sensorless: flux linkage", "str", "2560", "expert", "", "Observer flux = flux linkage [mWb] x 256 (flux = battery V / (1.732 x electrical no-load rad/s)). 10 mWb -> 2560. Only used in sensorless mode."],
    ["gamma", "Sensorless: observer gain (shift)", "str", "13", "expert", "", "Observer correction gain as a shift, 12..14. Only used in sensorless mode."],
    ["obs_angle_deg", "Sensorless: angle offset", "float", 90.0, "expert", "deg", "+90 matches the open loop start direction. Fine tune in small steps (+-10) for smooth running. Wrong way: Reverse motor direction."],
    // compile-time values the block uses for conversions (fixed in the prebuilt firmware)
    ["cal_i", "Current cal.", "hidden", "38LL<<8", "hidden", "", ""],
    ["cal_v", "Voltage cal.", "hidden", 25, "hidden", "", ""],
  ];

  const defaults = () => Object.fromEntries(FIELDS.map(f => [f[0], f[3]]));
  const has = (s, k) => Object.prototype.hasOwnProperty.call(s, k);
  const B = (s, k) => { const v = s[k]; if (typeof v === "boolean") return v; if (typeof v === "number") return v !== 0;
    if (typeof v === "string") return v.toLowerCase() === "true" || v === "1"; return false; };
  const D = (s, k) => { const v = s[k]; if (typeof v === "number") return v; const x = parseFloat(String(v).replace(",", ".")); return isNaN(x) ? 0 : x; };
  // Python int(): truncate toward zero
  const I = (s, k) => { const v = s[k]; if (typeof v === "number") return Math.trunc(v); const t = String(v).trim();
    if (/^-?\d+$/.test(t)) return parseInt(t, 10); const x = parseFloat(t); return isNaN(x) ? 0 : Math.trunc(x); };
  const S = (s, k) => (s[k] === undefined || s[k] === null) ? "" : String(s[k]);
  const cInt = (txt, fb) => { const m = /^\s*(-?\d+)/.exec(txt); return m ? parseInt(m[1], 10) : fb; };
  // Python round(): halves to even
  const pyRound = x => { const f = Math.floor(x), d = x - f; if (d > 0.5) return f + 1; if (d < 0.5) return f; return (f % 2 === 0) ? f : f + 1; };

  function wheelCircumference(s) {
    for (const [n, mm] of WHEELS) if (n === s.wheel && mm) return mm;
    return has(s, "wheel_mm") ? I(s, "wheel_mm") : 2240;
  }

  function batteryMv(s) {
    const cells = has(s, "battery") ? I(s, "battery") : 10;
    let cutoff = cells * 3100, full = cells * 4200;
    if (D(s, "voltage_min_v") > 0) cutoff = Math.trunc(D(s, "voltage_min_v") * 1000);
    if (D(s, "full_voltage_v") > 0) full = Math.trunc(D(s, "full_voltage_v") * 1000);
    const span = (full - cutoff) < 1 ? 1 : full - cutoff;
    return [cutoff, full, [0.10, 0.25, 0.45, 0.65, 0.85].map(f => Math.trunc(cutoff + span * f))];
  }

  function problems(s) {
    const p = [];
    const [cutoff, full] = batteryMv(s);
    if (I(s, "battery") === 0 && (D(s, "voltage_min_v") <= 0 || D(s, "full_voltage_v") <= 0)) p.push("Custom battery: set both 'Full charge voltage' and 'Cutoff voltage'.");
    if (full <= cutoff + 1000) p.push("'Full charge voltage' must be clearly above the 'Cutoff voltage'.");
    if (D(s, "voltage_ramp_v") * 1000 >= (full - cutoff) / 2) p.push("'Soft voltage ramp width' is too big for this battery voltage range.");
    const idx = I(s, "speed_temp_input");
    if (idx === 6 && B(s, "torque_on_ad1")) p.push("Speed/temperature wire and torque sensor can't both use AD1.");
    if (idx === 5 && B(s, "linear_regen")) p.push("Speed/temperature wire and proportional regen can't both use AD2.");
    if (idx !== 0 && !(B(s, "speed_on_wire") || B(s, "temp_on_wire"))) p.push("Speed/temperature wire is selected but neither speed nor temperature is ticked.");
    if (cutoff < 10000 || full > 100000) p.push("Battery voltages must be between 10 V and 100 V.");
    const sl = I(s, "speed_limit"); if (sl < 1 || sl > 100) p.push("Speed limit must be 1..100 km/h.");
    const wc = wheelCircumference(s); if (wc < 500 || wc > 3500) p.push("Wheel circumference must be 500..3500 mm.");
    const pm = I(s, "pas_magnets"); if (pm < 1 || pm > 64) p.push("Pedal sensor magnets must be 1..64.");
    if (I(s, "pas_timeout") <= I(s, "ramp_end") || I(s, "ramp_end") < 1) p.push("'PAS stop timeout' must be above 'PAS full assist at'.");
    const g = I(s, "gear_ratio"); if (g < 1 || g > 1000) p.push("'Pole pairs x gear ratio' must be 1..1000.");
    if (D(s, "phase_current_a") <= 0) p.push("Motor current must be above 0.");
    if (D(s, "battery_current_a") <= 0 || D(s, "battery_current_a") > 60) p.push("Battery current must be 0..60 A.");
    return p;
  }

  const FLAGS = ["throttle", "legal", "torque_sensor", "torque_on_ad1", "dirdet", "adc_brake", "speed_external", "speed_from_wire", "temp_from_wire"];
  const NAMES = ["ph_current_max", "battery_current_max", "regen_current", "regen_current_max", "battery_voltage_min", "battery_voltage_max",
    "voltage_ramp_mv", "battery_level_1", "battery_level_2", "battery_level_3", "battery_level_4", "battery_level_5", "speed_limit",
    "wheel_circumference", "gear_ratio", "pas_imp_per_turn", "pulses_per_revolution", "pushassist_current", "ts_coef", "pas_timeout",
    "ramp_end", "throttle_offset", "throttle_max", "motor_temp_threshold", "motor_temp_max", "frac_high", "frac_low", "reverse",
    "speed_temp_adc", "speed_temp_threshold", "ntc_pullup_ohm", "obs_inductance", "obs_resistance", "obs_flux_linkage", "obs_gamma",
    "obs_angle_offset", "current_ramp_ms"];

  function values(s) {
    const calI = cInt(S(s, "cal_i"), 38);
    const [cutoff, vmax, bars] = batteryMv(s);
    const units = k => pyRound(D(s, k) * 1000 / calI);
    const ma = k => pyRound(D(s, k) * 1000);
    const atLeast1 = v => v < 1 ? 1 : v;
    const st = I(s, "speed_temp_input");
    const speedFromWire = st !== 0 && B(s, "speed_on_wire"), tempFromWire = st !== 0 && B(s, "temp_on_wire");
    const flags = { throttle: B(s, "throttle"), legal: B(s, "legal_mode"), torque_sensor: B(s, "torque_sensor"),
      torque_on_ad1: B(s, "torque_on_ad1"), dirdet: B(s, "dirdet"), adc_brake: B(s, "linear_regen"),
      speed_external: B(s, "speed_sensor_external") || speedFromWire, speed_from_wire: speedFromWire, temp_from_wire: tempFromWire };
    const derate = has(s, "motor_temp_derate") ? I(s, "motor_temp_derate") : 100;
    const tmax = has(s, "motor_temp_max") ? I(s, "motor_temp_max") : 130;
    const ramp = Math.trunc((has(s, "voltage_ramp_v") ? D(s, "voltage_ramp_v") : 1.0) * 1000);
    let angle = pyRound(D(s, "obs_angle_deg") / 180.0 * 2147483648.0);
    angle = ((angle % 4294967296) + 4294967296) % 4294967296;
    if (angle >= 2147483648) angle -= 4294967296;
    const v = {
      ph_current_max: units("phase_current_a"), battery_current_max: ma("battery_current_a"),
      regen_current: units("regen_current_a"), regen_current_max: ma("regen_max_a"),
      battery_voltage_min: cutoff, battery_voltage_max: vmax, voltage_ramp_mv: ramp < 100 ? 100 : ramp,
      speed_limit: I(s, "speed_limit"), wheel_circumference: wheelCircumference(s),
      gear_ratio: atLeast1(I(s, "gear_ratio")), pas_imp_per_turn: atLeast1(I(s, "pas_magnets")),
      pulses_per_revolution: atLeast1(I(s, "pulses_per_rev")), pushassist_current: units("walk_current_a"),
      ts_coef: I(s, "ts_coef"), pas_timeout: I(s, "pas_timeout"), ramp_end: I(s, "ramp_end"),
      throttle_offset: I(s, "throttle_offset"), throttle_max: I(s, "throttle_max"),
      motor_temp_threshold: derate, motor_temp_max: (derate + 5) > tmax ? derate + 5 : tmax,
      frac_high: I(s, "frac_high"), frac_low: I(s, "frac_low"), reverse: B(s, "reverse") ? -1 : 1,
      speed_temp_adc: st !== 0 ? st : 6,
      speed_temp_threshold: has(s, "speed_temp_threshold") ? I(s, "speed_temp_threshold") : 100,
      ntc_pullup_ohm: tempFromWire ? atLeast1(has(s, "ntc_pullup_ohm") ? I(s, "ntc_pullup_ohm") : 3000) : 0,
      obs_inductance: cInt(S(s, "inductance"), 0), obs_resistance: cInt(S(s, "resistance"), 0),
      obs_flux_linkage: cInt(S(s, "flux_linkage"), 0), obs_gamma: cInt(S(s, "gamma"), 0), obs_angle_offset: angle,
      current_ramp_ms: Math.max(0, Math.min(10000, has(s, "current_ramp_ms") ? I(s, "current_ramp_ms") : 300)),
    };
    bars.forEach((mv, n) => { v["battery_level_" + (n + 1)] = mv * 10; });
    return [flags, v];
  }

  const CRC_TABLE = (() => { const t = new Uint32Array(256); for (let n = 0; n < 256; n++) { let c = n;
    for (let k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1); t[n] = c >>> 0; } return t; })();
  function crc32(bytes) { let c = 0xFFFFFFFF; for (const x of bytes) c = CRC_TABLE[(c ^ x) & 0xFF] ^ (c >>> 8); return (c ^ 0xFFFFFFFF) >>> 0; }

  function buildBlock(s) {
    const [flags, v] = values(s);
    const buf = new ArrayBuffer(256), dv = new DataView(buf), b = new Uint8Array(buf);
    let f = 0; FLAGS.forEach((k, i) => { if (flags[k]) f |= (1 << i); });
    dv.setUint32(0, 0xEB1C5E77, true); dv.setUint16(4, 1, true); dv.setUint16(6, 256, true); dv.setUint32(8, f >>> 0, true);
    NAMES.forEach((k, i) => dv.setInt32(12 + 4 * i, v[k], true));
    dv.setUint32(252, crc32(b.subarray(0, 252)), true);
    return b;
  }

  const hex = b => Array.from(b, x => x.toString(16).padStart(2, "0")).join("");

  return { DISPLAYS, WHEELS, BATTERIES, SPEED_TEMP, FIELDS, defaults, B, D, I, S, wheelCircumference, batteryMv,
    problems, values, buildBlock, crc32, hex, pyRound };
})();
if (typeof module !== "undefined") module.exports = EB;
