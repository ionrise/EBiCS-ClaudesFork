"""
Fake ESP32 flasher API for trying the web page on a PC (no ESP, no controller):

    python esp32_flasher/web/mock_server.py      then open http://127.0.0.1:8080

Simulates a connected, locked STM32F103: unlock, install, settings, calibration and live data.
Part of EBiCS, GPL v3 or later.
"""
import base64
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent
FW = HERE.parents[1] / "android_app" / "assets" / "firmware"

state = {"locked": True, "installed": "", "job": {"busy": False, "done": False, "ok": False, "step": "", "progress": 0, "error": "", "name": ""},
         "cal": 0, "cal_t": 0.0, "hall_events": 1000, "pas": 0}


def page():
    html = (HERE / "index.html").read_text(encoding="utf-8")
    js = (HERE / "settings_block.js").read_text(encoding="utf-8")
    html = html.replace('<script src="settings_block.js"></script>', "<script>\n" + js + "\n</script>")
    return html.replace("{{LOGO}}", base64.b64encode((HERE / "logo.png").read_bytes()).decode())


def run_job(name, steps, after):
    j = state["job"]
    j.update(busy=True, done=False, ok=False, error="", name=name, progress=0)

    def work():
        for s in steps:
            for p in range(0, 101, 20):
                j.update(step=s, progress=p)
                time.sleep(0.15)
        after()
        j.update(busy=False, done=True, ok=True, step="Done", progress=100)
    threading.Thread(target=work, daemon=True).start()


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def send(self, code, obj, ctype="application/json"):
        body = obj.encode() if isinstance(obj, str) else json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path).path
        if u == "/":
            return self.send(200, page(), "text/html; charset=utf-8")
        if u == "/api/variants":
            m = json.loads((FW / "manifest.json").read_text())
            return self.send(200, {"version": m["firmware_version"], "variants": [
                {"name": v["name"], "display": v["display"], "label": v["display_label"], "sensorless": v["sensorless"], "size": v["size"]}
                for v in m["variants"]]})
        if u == "/api/info":
            return self.send(200, {"busy": state["job"]["busy"], "connected": True, "dpidr": "0x1BA01477", "chip": "STM32F103 low density",
                                   "devId": 0x412, "flashKb": 32, "supported": True, "locked": state["locked"], "installed": state["installed"]})
        if u == "/api/job":
            return self.send(200, state["job"])
        if u == "/api/live":
            if not state["installed"]:
                return self.send(409, {"error": "firmware not recognised: install the firmware from this flasher"})
            t = time.time() - state["cal_t"]
            cs = 0 if not state["cal"] else 1 if t < 4 else 2 if t < 9 else 3
            if cs in (1, 2):
                state["hall_events"] += 37
            state["pas"] += 1
            return self.send(200, {"ui32_setup_magic": 0xEB1C5002, "ui8_setup_state": cs, "ui8_setup_progress": min(100, int(t * 12) % 101),
                                   "ui8_setup_halls_found": 63, "ui8_hall_calibrated": 1 if cs == 3 else 0, "ui32_hall_events": state["hall_events"],
                                   "ui32_pas_events": state["pas"], "brake_flag": int(time.time()) % 7 == 0, "ui16_throttle": 900,
                                   "ui16_throttle_offset": 760, "uint16_full_rotation_counter": 100, "ui32_KV": 224, "ui8_watchdog_reset": 0,
                                   "ui16_motor_temp_adc": 2100, "ui8_settings_source": 1, "MS.Voltage": 1560, "MS.Speed": 4000, "MS.i_q": 50,
                                   "MS.Battery_Current": 3200, "MS.assist_level": int(time.time() / 5) % 5, "MS.Temperature": 25,
                                   "MS.int_Temperature": 31, "adc5": 1234, "adc6": 2100, "Hall_45": 1837291766, "Hall_51": 2565050131,
                                   "Hall_13": 3245086636, "Hall_32": 4032497326, "Hall_26": 417566535, "Hall_64": 1109533505,
                                   "variant": state["installed"]})
        self.send(404, {"error": "not found"})

    def do_POST(self):
        u = urlparse(self.path).path
        n = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(n)
        q = {k: v[0] for k, v in parse_qs(raw.decode(errors="replace")).items()} if b"=" in raw[:200] else {}
        if u == "/api/unlock":
            run_job("unlock", ["Erasing option bytes and the whole flash", "Writing RDP = 0xA5"], lambda: state.update(locked=False))
            return self.send(200, {"started": True})
        if u == "/api/install":
            if len(q.get("block", "")) != 512:
                return self.send(400, {"error": "invalid settings block"})
            v = q.get("variant", "")
            run_job("install", ["Erasing", "Writing", "Verifying"], lambda: state.update(installed=v))
            return self.send(200, {"started": True})
        if u == "/api/settings":
            run_job("settings", ["Writing the settings"], lambda: None)
            return self.send(200, {"started": True})
        if u == "/api/calibrate":
            state.update(cal=1, cal_t=time.time())
            return self.send(200, {"ok": True})
        if u in ("/api/restart", "/api/upload"):
            if u == "/api/upload":
                run_job("upload", ["Erasing", "Writing", "Verifying"], lambda: state.update(installed=""))
                return self.send(200, {"started": True})
            return self.send(200, {"ok": True})
        self.send(404, {"error": "not found"})


if __name__ == "__main__":
    print("http://127.0.0.1:8080")
    ThreadingHTTPServer(("127.0.0.1", 8080), H).serve_forever()
