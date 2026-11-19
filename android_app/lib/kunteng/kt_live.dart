import 'dart:typed_data';

/// Live variables of the KT firmware (same list as setup_app/kt_live.py). SDCC stores values big endian.
class KtLiveVar {
  final String name, type, label, unit;
  final double scale;
  const KtLiveVar(this.name, this.type, this.label, this.unit, this.scale);
}

const ktLiveVars = <KtLiveVar>[
  KtLiveVar('ui8_adc_battery_voltage_filtered', 'u8', 'Battery', 'V', 0.272),
  KtLiveVar('i16_motor_current_filtered_10b', 'i16', 'Battery current', 'A', 0.125),
  KtLiveVar('ui16_motor_speed_erps', 'u16', 'Motor speed', 'erps', 1),
  KtLiveVar('f_wheel_speed', 'f32', 'Speed', 'km/h', 1),
  KtLiveVar('ui16_duty_cycle', 'u16', 'Duty cycle', '/508', 1),
  KtLiveVar('ui8_adc_throttle_value', 'u8', 'Throttle (ADC)', '', 1),
  KtLiveVar('ui8_pas_cadence_rpm', 'u8', 'Cadence', 'rpm', 1),
  KtLiveVar('lcd_configuration_variables', 'u8', 'Assist level', '', 1),
  KtLiveVar('ui8_motor_controller_state', 'u8', 'Controller state', '', 1),
  KtLiveVar('ui8_motor_controller_error', 'u8', 'Error code', '', 1),
  KtLiveVar('ui8_motor_state', 'u8', 'Motor state', '', 1),
  KtLiveVar('ui8_motor_commutation_type', 'u8', 'Commutation', '', 1),
  KtLiveVar('ui8_hall_sensors_last', 'u8', 'Hall state', '', 1),
  KtLiveVar('ui8_angle_correction', 'u8', 'FOC angle correction', '/256', 1),
  KtLiveVar('ui8_phase_b_peak', 'u8', 'Phase current peak', 'A', 0.121),
  KtLiveVar('ui8_coasting', 'u8', 'Coasting (PWM off)', '', 1),
  KtLiveVar('ui16_emf_duty_per_erps_x256', 'u16', 'Back-EMF constant', '', 1),
  KtLiveVar('i8_hall_offset', 'i8x8', 'Learned hall offsets', '/256', 1),
];

const _stateBits = <(int, String)>[
  (2, 'brake'),
  (4, 'over current'),
  (8, 'under voltage'),
  (32, 'motor blocked'),
  (64, 'over voltage'),
  (128, 'hall error'),
];

/// Hall autodetect variables (kunteng/motor.c motor_autodetect), the app writes 1 into ui8_ad_cmd.
const ktAdVars = <(String, String)>[
  ('ui8_ad_cmd', 'u8'),
  ('ui8_ad_state', 'u8'),
  ('ui8_ad_progress', 'u8'),
  ('ui8_ad_spread', 'u8'),
  ('ui8_ad_rotor_offset', 'u8'),
  ('ui8_ad_fwd', 'u8x8'),
  ('ui8_ad_bwd', 'u8x8'),
];
const ktAdStates = {0: 'idle', 1: 'turning forward', 2: 'turning backward', 3: 'done', 4: 'failed'};

/// Text lines for an autodetect result (same as setup_app/kt_live.py describe_autodetect).
List<String> ktDescribeAutodetect(Map<String, Object> r) {
  if (r['ui8_ad_state'] != 3) {
    return [
      'Autodetect FAILED: not all hall edges seen in both directions, edges more than 28 deg off 60 deg, or the '
          'motor did not follow (lift the wheel, release the brake, check the hall wires).',
    ];
  }
  const deg = 360.0 / 256;
  final fwd = r['ui8_ad_fwd'] as List<int>, bwd = r['ui8_ad_bwd'] as List<int>;
  final lines = ['Hall autodetect done (deg, 0 = state 4 edge):', 'state  forward  backward  average  diff'];
  for (final st in [4, 6, 2, 3, 1, 5]) {
    final d = ((bwd[st] - fwd[st] + 128) & 0xFF) - 128;
    final avg = (fwd[st] + d ~/ 2) & 0xFF;
    String f(double v) => v.toStringAsFixed(1).padLeft(7);
    lines.add(
      '  $st  ${f(fwd[st] * deg)}  ${f(bwd[st] * deg)}  ${f(avg * deg)}  ${(d >= 0 ? '+' : '') + (d * deg).toStringAsFixed(1)}',
    );
  }
  final spread = (r['ui8_ad_spread'] as int) * deg;
  final verdict = spread <= 15 ? 'good' : (spread <= 40 ? 'the motor turns stiffly (brake, freewheel?)' : 'very large');
  lines.add(
    'Rotor offset: ${r['ui8_ad_rotor_offset']}/256 (setting default 202). '
    'Largest difference ${spread.round()} deg: $verdict',
  );
  lines.add('Saved in the controller, used from now on (also after power off).');
  return lines;
}

/// Raw bytes -> value (a number, or a list for the arrays).
Object ktDecode(String type, Uint8List raw) {
  final d = ByteData.sublistView(raw);
  return switch (type) {
    'u8' => raw[0],
    'i8' => d.getInt8(0),
    'u16' => d.getUint16(0, Endian.big),
    'i16' => d.getInt16(0, Endian.big),
    'f32' => d.getFloat32(0, Endian.big),
    'i8x8' => [for (var i = 0; i < 8; i++) d.getInt8(i)],
    'u8x8' => [for (var i = 0; i < 8; i++) raw[i]],
    _ => throw ArgumentError(type),
  };
}

/// Text for one variable on the live screen.
String ktDescribe(KtLiveVar v, Object value) {
  if (value is List<int>) {
    return [
      for (final st in [4, 6, 2, 3, 1, 5]) value[st],
    ].map((x) => x >= 0 ? '+$x' : '$x').join(' ');
  }
  final n = value as num;
  final i = n.round();
  switch (v.name) {
    case 'ui8_motor_controller_state':
      if (i == 1) return 'ok';
      final bits = [
        for (final (b, t) in _stateBits)
          if (i & b != 0) t,
      ];
      return bits.isEmpty ? '$i' : bits.join(', ');
    case 'ui8_motor_state':
      return const {0: 'coast', 1: 'stop', 2: 'start up', 3: 'cool down', 4: 'running'}[i] ?? '$i';
    case 'ui8_motor_controller_error':
      return const {
            0: '-',
            0x21: '21 over current',
            0x22: '22 throttle',
            0x24: '24 hall sensors',
            0x28: 'over voltage',
            0x91: 'battery empty',
          }[i] ??
          i.toRadixString(16);
    case 'ui8_motor_commutation_type':
      return const {1: 'block (low speed)', 2: 'sine wave'}[i] ?? '$i';
    case 'ui8_coasting':
      return i != 0 ? 'yes' : 'no';
  }
  final scaled = n * v.scale;
  return (v.scale != 1 || v.type == 'f32') ? '${scaled.toStringAsFixed(2)} ${v.unit}'.trim() : '$i ${v.unit}'.trim();
}
