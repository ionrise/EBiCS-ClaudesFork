import 'dart:convert';

import 'package:flutter/services.dart';

import '../stlink/swim.dart';
import '../usb/usb_bridge.dart';
import 'kt_live.dart';
import 'kt_settings.dart';

/// Result of a SWIM connection check.
class KtProbe {
  final bool stlinkFound, permission, targetFound, locked;
  final String error;
  KtProbe({
    this.stlinkFound = false,
    this.permission = false,
    this.targetFound = false,
    this.locked = false,
    this.error = '',
  });

  String describe() {
    if (!stlinkFound) return 'ST-Link NOT found. Plug it into the phone with a USB OTG adapter.';
    if (!permission) return 'ST-Link found, but the app may not use it yet. Allow USB access and try again.';
    if (!targetFound) {
      return 'ST-Link found, but the KT controller does not answer.\n'
          '  - is the controller switched on (battery)?\n'
          '  - SWIM wire on the SWIM pin of the ST-Link (not SWDIO), GND connected (3.3V and RST optional)'
          '${error.isNotEmpty ? '\n  ($error)' : ''}';
    }
    return locked
        ? 'Controller found - chip is LOCKED (original KT firmware). Unlock it before installing.'
        : 'Controller found - chip is unlocked, ready to install.';
  }
}

/// Kunteng firmware actions over SWIM. Each action opens its own session and closes it again.
class KtController {
  static const _fwAsset = 'assets/firmware/kunteng.ihx';
  final AssetBundle bundle;
  final UsbTransport Function() _transport;
  final Future<void> Function(Duration)? sleep;

  KtController({AssetBundle? bundle, UsbTransport Function()? transport, this.sleep})
    : bundle = bundle ?? rootBundle,
      _transport = transport ?? UsbBridge.new;

  Future<bool> hasFirmware() async {
    try {
      await bundle.loadString(_fwAsset);
      return true;
    } catch (_) {
      return false;
    }
  }

  Map<String, (int, int)>? _liveSyms;
  UsbTransport? _liveT;
  Swim? _live;

  /// Live variables of the running firmware (no reset). The addresses are those of the prebuilt firmware
  /// (assets/firmware/kunteng.sym.json): the controller must run the firmware installed by this app.
  Future<Map<String, Object>> liveSnapshot() async {
    _liveSyms ??= {
      for (final e
          in (jsonDecode(await bundle.loadString('assets/firmware/kunteng.sym.json')) as Map<String, dynamic>).entries)
        e.key: ((e.value as List)[0] as int, (e.value as List)[1] as int),
    };
    if (_live == null) {
      final t = _transport();
      if (t is UsbBridge) {
        final dev = await UsbBridge.find();
        if (dev == null) throw SwimException('ST-Link not found');
        if (!dev.permission && !await UsbBridge.requestPermission()) throw SwimException('no USB permission');
        await t.open();
      }
      final s = Swim(t, sleep: sleep);
      try {
        await s.openLive();
      } catch (e) {
        if (t is UsbBridge) await t.close();
        rethrow;
      }
      _liveT = t;
      _live = s;
    }
    final out = <String, Object>{};
    try {
      // the live link runs with SWIM_DM on: if the CPU got stalled (SWIM disturbed by the motor current), the PWM
      // keeps its last duty cycle and nothing controls the motor - clear the stall bit at once (DM_CSR2 bit 3)
      final dm = await _live!.readByte(Swim.dmCsr2);
      if (dm & 0x08 != 0) await _live!.writeByte(Swim.dmCsr2, dm & ~0x08);
      for (final v in ktLiveVars) {
        final sym = _liveSyms![v.name];
        if (sym == null) continue;
        out[v.name] = ktDecode(v.type, await _live!.readMem(sym.$1, sym.$2));
      }
    } catch (e) {
      await stopLive();
      rethrow;
    }
    return out;
  }

  /// Starts the hall autodetect over the live connection and waits for the result (ktAdVars values).
  Future<Map<String, Object>> autodetect(
    void Function(String) log, {
    Duration poll = const Duration(milliseconds: 300),
  }) async {
    await liveSnapshot(); // connects and loads the addresses
    final cmd = _liveSyms!['ui8_ad_cmd'];
    if (cmd == null) throw SwimException('this firmware has no hall autodetect (install the current firmware)');
    try {
      await _live!.writeByte(cmd.$1, 1);
      var last = -1;
      for (var i = 0; i < 70; i++) {
        await Future<void>.delayed(poll);
        final r = <String, Object>{};
        for (final (name, type) in ktAdVars) {
          final s = _liveSyms![name];
          if (s != null) r[name] = ktDecode(type, await _live!.readMem(s.$1, s.$2));
        }
        if (r['ui8_ad_cmd'] == 1 && i > 3) {
          throw SwimException(
            'the firmware did not start the autodetect (motor turning, throttle pressed or a fault?)',
          );
        }
        final st = r['ui8_ad_state'] as int? ?? 0, p = r['ui8_ad_progress'] as int? ?? 0;
        if ((st == 1 || st == 2) && p ~/ 10 != last) {
          last = p ~/ 10;
          log('  ${ktAdStates[st]} ... $p %');
        }
        if ((st == 3 || st == 4) && r['ui8_ad_cmd'] == 0) return r;
      }
      throw SwimException('no result after 20 s');
    } finally {
      await stopLive();
    }
  }

  Future<void> stopLive() async {
    final s = _live, t = _liveT;
    _live = null;
    _liveT = null;
    if (s != null) await s.close();
    if (t is UsbBridge) await t.close();
  }

  Future<T> _session<T>(Future<T> Function(Swim s) f) async {
    final t = _transport();
    if (t is UsbBridge) {
      final dev = await UsbBridge.find();
      if (dev == null) throw SwimException('ST-Link not found. Plug it in with a USB OTG adapter.');
      if (!dev.permission && !await UsbBridge.requestPermission()) {
        throw SwimException('no permission to use the ST-Link');
      }
      await t.open();
    }
    final s = Swim(t, sleep: sleep);
    try {
      await s.open();
      return await f(s);
    } finally {
      await s.close();
      if (t is UsbBridge) await t.close();
      await _releaseAfterReset();
    }
  }

  /// After a reset through SWIM the debug mode can keep the CPU stalled (real KT controller: no melody, no motor
  /// after installing until power off). Connect live and clear the stall bit (DM_CSR2 bit 3).
  Future<void> _releaseAfterReset() async {
    for (var i = 0; i < 6; i++) {
      await Future<void>.delayed(const Duration(milliseconds: 300));
      final t = _transport();
      try {
        if (t is UsbBridge) await t.open();
        final s = Swim(t, sleep: sleep);
        try {
          await s.openLive();
          final d = await s.readByte(Swim.dmCsr2);
          if (d & 0x08 != 0) await s.writeByte(Swim.dmCsr2, d & ~0x08);
          return;
        } finally {
          await s.close();
        }
      } catch (_) {
        // try again
      } finally {
        if (t is UsbBridge) await t.close();
      }
    }
  }

  Future<KtProbe> probe() async {
    final t = _transport();
    if (t is UsbBridge) {
      final dev = await UsbBridge.find();
      if (dev == null) return KtProbe();
      if (!dev.permission && !await UsbBridge.requestPermission()) return KtProbe(stlinkFound: true);
    }
    try {
      return await _session((s) async {
        final opt = await s.readOptionBytes(4);
        return KtProbe(stlinkFound: true, permission: true, targetFound: true, locked: opt[0] == 0xAA);
      });
    } catch (e) {
      return KtProbe(stlinkFound: true, permission: true, error: '$e');
    }
  }

  /// Removes read-out protection (erases the chip) and sets the option bytes the KT firmware needs.
  Future<void> unlock(void Function(String) log) => _session((s) async {
    log('writing option bytes (erases the original firmware) ...');
    await s.writeOptionBytes();
    final got = await s.readOptionBytes();
    log(
      _eq(got, Swim.ktOptionBytes) ? 'option bytes ok' : 'option bytes read back differently - power cycle and check',
    );
    await s.resetTarget();
  });

  /// Installs the prebuilt firmware and writes the settings block.
  Future<void> install(KtSettings settings, void Function(String step, double progress) report) async {
    final img = ihxToFlashImage(await bundle.loadString(_fwAsset));
    await _session((s) async {
      final opt = await s.readOptionBytes(4);
      if (opt[0] == 0xAA) throw SwimException('the chip is locked (original firmware) - unlock it first');
      if (opt[3] != Swim.ktOptionBytes[3]) {
        report('setting option bytes', 0);
        await s.writeOptionBytes();
      }
      await s.writeBlocks(Swim.flashStart, img, progress: (d, n) => report('writing firmware', d / n));
      report('verifying', 1);
      final back = await s.readMem(Swim.flashStart, img.length);
      if (!_eq(back, img)) throw SwimException('verify failed');
      await _writeSettings(s, settings);
      await s.resetTarget();
    });
  }

  /// Writes only the settings block (no reinstall needed) and restarts the controller.
  Future<void> writeSettings(KtSettings settings) => _session((s) async {
    await _writeSettings(s, settings);
    await s.resetTarget();
  });

  Future<void> _writeSettings(Swim s, KtSettings settings) async {
    // whole EEPROM block 1 (0x4080..0x40FF), keep what is behind the settings
    final cur = await s.readMem(KtSettings.blockAddress, Swim.block);
    final blk = Uint8List.fromList(cur)..setRange(0, KtSettings.blockLength, settings.block());
    await s.writeBlocks(KtSettings.blockAddress, blk);
    // forget the display values of the old firmware so the new defaults apply until the display sends its own
    final b0 = await s.readMem(Swim.eepromStart, Swim.block);
    if (b0[0] != 0) {
      final z = Uint8List.fromList(b0)..[0] = 0;
      await s.writeBlocks(Swim.eepromStart, z);
    }
  }

  static bool _eq(Uint8List a, Uint8List b) {
    if (a.length != b.length) return false;
    for (var i = 0; i < a.length; i++) {
      if (a[i] != b[i]) return false;
    }
    return true;
  }
}
