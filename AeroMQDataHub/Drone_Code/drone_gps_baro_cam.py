#!/usr/bin/env python3
"""
Betaflight MSP -> MQTT + Picamera2 JPEG publisher

Publishes:
- Telemetry JSON to:  flight/telemetry
- Camera JSON+base64 JPEG to: flight/camera

Tested on Raspberry Pi OS (Bookworm), Pi 4.
"""

import json
import struct
import time
import threading
import signal
import sys
from datetime import datetime
from io import BytesIO

import serial
import paho.mqtt.client as mqtt

# Try Picamera2 (optional: script still runs if camera not available)
CAM_AVAILABLE = True
try:
    from picamera2 import Picamera2
    from PIL import Image
except Exception as _e:
    CAM_AVAILABLE = False
    CAM_IMPORT_ERROR = _e

# ========= CONFIG =========
MQTT_HOST   = "xxx.xxx.xxx.xxx"   # broker IP or "localhost"
MQTT_PORT   = 1883
TOPIC_TEL   = "flight/telemetry"
TOPIC_CAM   = "flight/camera"
CLIENT_ID   = "pi-drone"

# MSP serial
MSP_PORT    = "/dev/ttyACM0"
MSP_BAUD    = 115200
MSP_TIMEOUT = 0.3
POLL_HZ     = 5.0               # telemetry rate (Hz)

# Camera
CAM_INTERVAL_SEC = 10           # how often to publish a photo
CAM_WIDTH  = 1280               # None to use default
CAM_HEIGHT = 720
JPEG_QUALITY = 85               # 60–90 recommended
# ==========================

# MSP command IDs
MSP_API_VERSION = 1
MSP_STATUS      = 101
MSP_RC          = 105
MSP_GPS         = 106
MSP_ALTITUDE    = 109
MSP_ANALOG      = 110
MSP_ATTITUDE    = 108

stop_flag = threading.Event()
ser_lock  = threading.Lock()

# ---------- Common helpers ----------

def graceful_exit(signum, frame):
    stop_flag.set()

def mqtt_connect():
    c = mqtt.Client(client_id=CLIENT_ID)
    # c.username_pw_set("user","pass")   # if your broker requires auth
    c.enable_logger()                    # logs to stdout (optional)

    def _on_conn(client, userdata, flags, rc):
        print(f"[MQTT] Connected rc={rc}")

    def _on_disc(client, userdata, rc):
        print(f"[MQTT] Disconnected rc={rc}")

    c.on_connect = _on_conn
    c.on_disconnect = _on_disc
    c.loop_start()
    while not stop_flag.is_set():
        try:
            c.connect(MQTT_HOST, MQTT_PORT, keepalive=30)
            break
        except Exception as e:
            print(f"[MQTT] Connect failed: {e}; retrying in 2s")
            time.sleep(2)
    return c

# ---------- MSP side ----------

def open_serial():
    while not stop_flag.is_set():
        try:
            ser = serial.Serial(MSP_PORT, MSP_BAUD, timeout=MSP_TIMEOUT)
            print(f"[MSP] Opened {MSP_PORT} @ {MSP_BAUD}")
            return ser
        except Exception as e:
            print(f"[MSP] Cannot open {MSP_PORT}: {e}; retrying in 2s")
            time.sleep(2)
    return None

def msp_request(ser: serial.Serial, cmd: int):
    """
    MSP v1 (no payload):
      -> $M< [len=0] [cmd] [chk]
      <- $M> [len]   [cmd] [data...] [chk]
    """
    frame = b"$M<" + bytes([0, cmd, (0 ^ cmd) & 0xFF])
    with ser_lock:
        ser.reset_input_buffer()
        ser.write(frame)
        hdr = ser.read(5)
        if not (hdr and len(hdr) == 5 and hdr.startswith(b"$M>")):
            return None
        ln = hdr[3]
        data = ser.read(ln)
        _   = ser.read(1)  # checksum (ignored)
    return data

def parse_status(stat_bytes):
    """
    MSP_STATUS classic layout:
      [0..1]=cycleTime(us) u16
      [4..7]=flightModeFlags u32
    """
    if not stat_bytes or len(stat_bytes) < 10:
        return {}
    cyc_us = struct.unpack("<H", stat_bytes[0:2])[0]
    flags  = struct.unpack("<I", stat_bytes[4:8])[0]

    ARM_BIT, ANGLE_BIT, HORIZON_BIT, GPSRESC_BIT = 0, 1, 2, 10
    armed   = bool(flags & (1 << ARM_BIT))
    angle   = bool(flags & (1 << ANGLE_BIT))
    horizon = bool(flags & (1 << HORIZON_BIT))
    gpsres  = bool(flags & (1 << GPSRESC_BIT))

    mode = "ACRO"
    if angle:   mode = "ANGLE"
    if horizon: mode = "HORIZON"

    return {"Armed": armed, "Mode": mode, "GpsRescue": gpsres, "CycleTimeUs": cyc_us}

def telemetry_loop(mqttc: mqtt.Client):
    ser = open_serial()
    if ser is None:
        print("[MSP] Serial unavailable; telemetry thread exiting.")
        return

    api = msp_request(ser, MSP_API_VERSION)
    print(f"[MSP] API_VERSION: {list(api) if api else 'None'}")

    period, next_tick = 1.0 / POLL_HZ, time.time()
    while not stop_flag.is_set():
        att = msp_request(ser, MSP_ATTITUDE)
        ana = msp_request(ser, MSP_ANALOG)
        gps = msp_request(ser, MSP_GPS)
        sta = msp_request(ser, MSP_STATUS)
        rc  = msp_request(ser, MSP_RC)
        alt = msp_request(ser, MSP_ALTITUDE)

        payload = {}

        if att and len(att) >= 6:
            roll, pitch, yaw = struct.unpack("<hhh", att[:6])
            payload.update({"Roll": roll/10.0, "Pitch": pitch/10.0, "Yaw": float(yaw)})

        if ana and len(ana) >= 2:
            vbat_dV = struct.unpack("<H", ana[0:2])[0]
            payload["Volts"] = vbat_dV / 10.0
            if len(ana) >= 4:
                payload["Mah"] = struct.unpack("<H", ana[2:4])[0]
            if len(ana) >= 8:
                amps_raw = struct.unpack("<h", ana[6:8])[0]
                payload["Amps"] = amps_raw / 100.0  # adjust if off

        if gps and len(gps) >= 16:
            payload["Fix"]  = int(gps[0]); payload["Sats"] = int(gps[1])
            payload["Lat"]  = struct.unpack("<i", gps[2:6])[0] / 10_000_000.0
            payload["Lon"]  = struct.unpack("<i", gps[6:10])[0] / 10_000_000.0
            payload["Alt"]  = struct.unpack("<H", gps[10:12])[0]
            if len(gps) >= 18:
                speed_cms  = struct.unpack("<H", gps[12:14])[0]
                course_d10 = struct.unpack("<H", gps[14:16])[0]
                payload["GpsSpeedMs"]   = speed_cms / 100.0
                payload["GpsCourseDeg"] = course_d10 / 10.0

        payload.update(parse_status(sta))

        if rc and len(rc) >= 8:
            ch = list(struct.unpack("<" + "H" * (len(rc)//2), rc))
            payload.update({
                "RcRoll": ch[0], "RcPitch": ch[1], "RcYaw": ch[2], "RcThrottle": ch[3]
            })

        if alt and len(alt) >= 6:
            est_cm   = struct.unpack("<i", alt[0:4])[0]
            var_cm_s = struct.unpack("<h", alt[4:6])[0]
            payload["BaroAltM"] = est_cm / 100.0
            payload["VSpeedMs"] = var_cm_s / 100.0

        if payload:
            j = json.dumps(payload, separators=(",", ":"))
            mqttc.publish(TOPIC_TEL, j, qos=1)
            print("[TEL]", j)

        next_tick += period
        time.sleep(max(0.0, next_tick - time.time()))

    try: ser.close()
    except Exception: pass
    print("[MSP] Telemetry thread stopped.")

# ---------- Camera side ----------

def camera_loop(mqttc: mqtt.Client):
    if not CAM_AVAILABLE:
        print(f"[CAM] Picamera2 not available: {CAM_IMPORT_ERROR}")
        return

    try:
        cam = Picamera2()
        cfg = cam.create_still_configuration()
        if CAM_WIDTH and CAM_HEIGHT:
            cfg["main"]["size"] = (CAM_WIDTH, CAM_HEIGHT)
        cam.configure(cfg)
        cam.start()
        time.sleep(0.2)
        print(f"[CAM] Started, publishing every {CAM_INTERVAL_SEC}s as JPEG ({CAM_WIDTH}x{CAM_HEIGHT})")
    except Exception as e:
        print(f"[CAM] Init failed: {e}")
        return

    while not stop_flag.is_set():
        t0 = time.time()
        try:
            # Get RGB array and encode to JPEG in-memory
            frame = cam.capture_array("main")
            from PIL import Image  # ensure PIL present
            buf = BytesIO()
            Image.fromarray(frame).save(buf, format="JPEG", quality=JPEG_QUALITY, optimize=True)
            jpg = buf.getvalue()
            h, w = frame.shape[:2]

            payload = {
                "Timestamp": datetime.utcnow().isoformat(timespec="seconds") + "Z",
                "Format": "jpeg",
                "Width":  w,
                "Height": h,
                "Quality": JPEG_QUALITY,
                "Data": base64_encode(jpg)  # base64 text
            }
            mqttc.publish(TOPIC_CAM, json.dumps(payload), qos=1)
            print(f"[CAM] {w}x{h} {len(jpg)/1024:.1f}KB")
        except Exception as e:
            print(f"[CAM] Capture/publish error: {e}")

        elapsed = time.time() - t0
        time.sleep(max(0.0, CAM_INTERVAL_SEC - elapsed))

    try: cam.stop()
    except Exception: pass
    print("[CAM] Camera thread stopped.")

def base64_encode(b: bytes) -> str:
    import base64
    return base64.b64encode(b).decode("ascii")

# ---------- Main ----------

def main():
    signal.signal(signal.SIGINT,  graceful_exit)
    signal.signal(signal.SIGTERM, graceful_exit)

    mqttc = mqtt_connect()

    # Start telemetry thread
    t_tel = threading.Thread(target=telemetry_loop, args=(mqttc,), daemon=True)
    t_tel.start()

    # Start camera thread
    t_cam = threading.Thread(target=camera_loop, args=(mqttc,), daemon=True)
    t_cam.start()

    print("[MAIN] Running. Ctrl+C to stop.")
    try:
        while not stop_flag.is_set():
            time.sleep(0.5)
    finally:
        mqttc.loop_stop()
        mqttc.disconnect()
        print("[MAIN] Stopped.")

if __name__ == "__main__":
    main()