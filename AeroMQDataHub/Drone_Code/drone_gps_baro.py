#!/usr/bin/env python3
"""
Betaflight MSP -> MQTT publisher (PascalCase fields for .NET dashboard)

Reads over USB VCP (/dev/ttyACM0):
- MSP_API_VERSION (1)
- MSP_ATTITUDE (108): Roll/Pitch/Yaw
- MSP_ANALOG (110): Volts, Amps, mAh (if configured)
- MSP_GPS (106): Fix, Sats, Lat, Lon, Alt, Speed, Course
- MSP_STATUS (101): Armed + basic flight mode + cycle time
- MSP_RC (105): RC sticks (Roll/Pitch/Yaw/Throttle)
- MSP_ALTITUDE (109): Baro altitude & vertical speed (if baro present)

Publishes JSON to MQTT topic 'flight/telemetry'.
"""

import json
import struct
import time
import threading
import signal
import sys

import serial
import paho.mqtt.client as mqtt

# ========= CONFIG =========
MQTT_HOST   = "xxx.xxx.x.xxx"  # broker IP or "localhost"
MQTT_PORT   = 1883
MQTT_TOPIC  = "flight/telemetry"
CLIENT_ID   = "pi-drone"

MSP_PORT    = "/dev/ttyACM0"   # FC USB VCP device
MSP_BAUD    = 115200
MSP_TIMEOUT = 0.3

POLL_HZ     = 5.0              # publish rate (msg/sec)
# ==========================

# MSP command IDs
MSP_API_VERSION = 1
MSP_STATUS      = 101
MSP_RC          = 105
MSP_GPS         = 106
MSP_ALTITUDE    = 109
MSP_ANALOG      = 110
MSP_ATTITUDE    = 108
# Optional (if supported): MSP_ESC_SENSOR_DATA = 245

stop_flag = threading.Event()
ser_lock  = threading.Lock()

def graceful_exit(signum, frame):
    stop_flag.set()

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
    Minimal MSP v1 request (no payload):
    -> $ M < [len=0] [cmd] [checksum]
    <- $ M > [len] [cmd] [payload len bytes] [checksum]
    """
    frame = b"$M<" + bytes([0, cmd, (0 ^ cmd) & 0xFF])
    with ser_lock:
        ser.reset_input_buffer()
        ser.write(frame)
        hdr = ser.read(5)  # $ M > len cmd
        if not (hdr and len(hdr) == 5 and hdr.startswith(b"$M>")):
            return None
        ln = hdr[3]
        data = ser.read(ln)
        _   = ser.read(1)  # checksum (ignored here)
    return data

def mqtt_connect():
    c = mqtt.Client(client_id=CLIENT_ID)
    # c.username_pw_set("user","pass")  # uncomment if broker requires auth

    def on_connect(client, userdata, flags, rc):
        print(f"[MQTT] Connected (rc={rc})")

    def on_disconnect(client, userdata, rc):
        print(f"[MQTT] Disconnected (rc={rc})")

    c.on_connect = on_connect
    c.on_disconnect = on_disconnect

    c.loop_start()
    while not stop_flag.is_set():
        try:
            c.connect(MQTT_HOST, MQTT_PORT, keepalive=30)
            break
        except Exception as e:
            print(f"[MQTT] Connect failed: {e}; retrying in 2s")
            time.sleep(2)
    return c

def parse_status(stat_bytes):
    """
    MSP_STATUS (classic layout):
      [0..1]=cycleTime(us) uint16
      [2]   =i2cErr (uint8)
      [3]   =activeSensors (uint8)
      [4..7]=flightModeFlags uint32 (bitfield)
      [8]   =profile (uint8)
      [9]   =rateProfile (uint8)
    """
    if not stat_bytes or len(stat_bytes) < 10:
        return {}

    cyc_us = struct.unpack("<H", stat_bytes[0:2])[0]
    flags  = struct.unpack("<I", stat_bytes[4:8])[0]

    # Common flag bits (may vary by BF version, these are widely used)
    ARM_BIT      = 0
    ANGLE_BIT    = 1
    HORIZON_BIT  = 2
    GPSRESC_BIT  = 10

    armed   = bool(flags & (1 << ARM_BIT))
    angle   = bool(flags & (1 << ANGLE_BIT))
    horizon = bool(flags & (1 << HORIZON_BIT))
    gpsres  = bool(flags & (1 << GPSRESC_BIT))

    mode = "ACRO"
    if angle:   mode = "ANGLE"
    if horizon: mode = "HORIZON"

    return {
        "Armed": armed,
        "Mode": mode,
        "GpsRescue": gpsres,
        "CycleTimeUs": cyc_us
    }

def main():
    signal.signal(signal.SIGINT,  graceful_exit)
    signal.signal(signal.SIGTERM, graceful_exit)

    ser = open_serial()
    if ser is None:
        print("Exiting (serial not available).")
        return

    # Probe API version (sanity)
    api = msp_request(ser, MSP_API_VERSION)
    print(f"[MSP] API_VERSION: {list(api) if api else 'None'}")

    mqttc = mqtt_connect()

    period    = 1.0 / POLL_HZ
    next_tick = time.time()

    while not stop_flag.is_set():
        # ---- MSP requests ----
        att = msp_request(ser, MSP_ATTITUDE)
        ana = msp_request(ser, MSP_ANALOG)
        gps = msp_request(ser, MSP_GPS)
        sta = msp_request(ser, MSP_STATUS)
        rc  = msp_request(ser, MSP_RC)
        alt = msp_request(ser, MSP_ALTITUDE)

        payload = {}

        # ATTITUDE: int16 roll,pitch,yaw (roll/pitch are deg*10)
        if att and len(att) >= 6:
            roll, pitch, yaw = struct.unpack("<hhh", att[:6])
            payload["Roll"]  = roll / 10.0
            payload["Pitch"] = pitch / 10.0
            payload["Yaw"]   = float(yaw)

        # ANALOG: vbat deciVolts, mAh, RSSI, amperage (varies)
        if ana and len(ana) >= 2:
            vbat_dV = struct.unpack("<H", ana[0:2])[0]
            payload["Volts"] = vbat_dV / 10.0
            if len(ana) >= 4:
                payload["Mah"] = struct.unpack("<H", ana[2:4])[0]
            if len(ana) >= 8:
                amps_raw = struct.unpack("<h", ana[6:8])[0]
                payload["Amps"] = amps_raw / 100.0  # adjust if values look off

        # GPS: fix, sats, lat/lon (1e-7 deg), alt (m), speed (cm/s), course (deg/10)
        if gps and len(gps) >= 16:
            payload["Fix"]  = int(gps[0])
            payload["Sats"] = int(gps[1])
            payload["Lat"]  = struct.unpack("<i", gps[2:6])[0] / 10_000_000.0
            payload["Lon"]  = struct.unpack("<i", gps[6:10])[0] / 10_000_000.0
            payload["Alt"]  = struct.unpack("<H", gps[10:12])[0]
            # Optional extras if present
            if len(gps) >= 18:
                speed_cms  = struct.unpack("<H", gps[12:14])[0]
                course_d10 = struct.unpack("<H", gps[14:16])[0]
                payload["GpsSpeedMs"]   = speed_cms / 100.0
                payload["GpsCourseDeg"] = course_d10 / 10.0

        # STATUS: armed, mode, cycle time
        payload.update(parse_status(sta))

        # RC: first 4 channels (us)
        if rc and len(rc) >= 8:
            ch = list(struct.unpack("<" + "H" * (len(rc)//2), rc))
            # Some RX maps may swap order; these are BF default indices
            payload["RcRoll"]     = ch[0]
            payload["RcPitch"]    = ch[1]
            payload["RcYaw"]      = ch[2]
            payload["RcThrottle"] = ch[3]

        # ALTITUDE: estAlt (cm), vario (cm/s)
        if alt and len(alt) >= 6:
            est_cm   = struct.unpack("<i", alt[0:4])[0]
            var_cm_s = struct.unpack("<h", alt[4:6])[0]
            payload["BaroAltM"] = est_cm / 100.0
            payload["VSpeedMs"] = var_cm_s / 100.0

        # ---- Publish ----
        if payload:
            j = json.dumps(payload, separators=(",", ":"))
            mqttc.publish(MQTT_TOPIC, j, qos=1)
            print("[PUB]", j)

        # ---- Pace loop ----
        next_tick += period
        time.sleep(max(0.0, next_tick - time.time()))

    # Cleanup
    try: mqttc.loop_stop(); mqttc.disconnect()
    except Exception: pass
    try: ser.close()
    except Exception: pass
    print("Done.")

if __name__ == "__main__":
    main()