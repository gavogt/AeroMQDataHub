#!/usr/bin/env python3
"""
Drone MSP -> MQTT publisher (PascalCase keys for .NET)

- Reads MSP ATTITUDE (108), ANALOG (110), GPS (106) from Betaflight via USB VCP.
- Publishes JSON to MQTT topic 'flight/telemetry'.
- Keys: Roll, Pitch, Yaw, Volts, Amps, Fix, Sats, Lat, Lon, Alt.

Adjust MQTT_HOST / PORT / TOPIC and MSP_PORT if needed.
"""

import json
import struct
import time
import threading
import signal
import sys

import serial
import paho.mqtt.client as mqtt

# ====== CONFIG ======
MQTT_HOST  = "xxx.xxx.x.xxx"     # Your broker IP (or "localhost" if broker on same Pi)
MQTT_PORT  = 1883
MQTT_TOPIC = "flight/telemetry"

MSP_PORT   = "/dev/ttyACM0"      # FC USB VCP device
MSP_BAUD   = 115200
MSP_TIMEOUT= 0.3

# MSP command IDs (Betaflight)
MSP_API_VERSION = 1
MSP_ATTITUDE    = 108
MSP_ANALOG      = 110
MSP_GPS         = 106

POLL_HZ   = 5.0                   # publish rate (messages/sec)
# =====================

stop_flag = threading.Event()
ser_lock = threading.Lock()

def open_serial():
    return serial.Serial(MSP_PORT, MSP_BAUD, timeout=MSP_TIMEOUT)

def msp_request(ser: serial.Serial, cmd: int):
    """
    Minimal MSP v1 request (no payload): $M< [len=0] [cmd] [checksum]
    Returns raw data bytes or None on bad header/timeout.
    """
    frame = b"$M<" + bytes([0, cmd, (0 ^ cmd) & 0xFF])
    with ser_lock:
        ser.reset_input_buffer()
        ser.write(frame)
        hdr = ser.read(5)  # $ M > len cmd
        if not (hdr and len(hdr) == 5 and hdr.startswith(b"$M>")):
            return None
        length = hdr[3]
        data = ser.read(length)
        _ = ser.read(1)    # checksum (ignored here)
    return data

def mqtt_connect_loop():
    """Create MQTT client with auto-reconnect loop."""
    client = mqtt.Client(client_id="pi-drone")
    # client.username_pw_set("user","pass")  # if your broker requires auth

    # Optional: callbacks
    def on_connect(c, u, flags, rc):
        print(f"[MQTT] Connected (rc={rc})")

    def on_disconnect(c, u, rc):
        print(f"[MQTT] Disconnected (rc={rc})")

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect

    # Start background network thread
    client.loop_start()

    # Simple retry loop
    while not stop_flag.is_set():
        try:
            client.connect(MQTT_HOST, MQTT_PORT, keepalive=30)
            break
        except Exception as e:
            print(f"[MQTT] Connect failed: {e}; retrying in 2s")
            time.sleep(2)

    return client

def graceful_exit(signum, frame):
    stop_flag.set()

def main():
    signal.signal(signal.SIGINT,  graceful_exit)
    signal.signal(signal.SIGTERM, graceful_exit)

    # Open serial
    while not stop_flag.is_set():
        try:
            ser = open_serial()
            print(f"[MSP] Opened {MSP_PORT} @ {MSP_BAUD}")
            break
        except Exception as e:
            print(f"[MSP] Cannot open {MSP_PORT}: {e}; retrying in 2s")
            time.sleep(2)
    else:
        return

    # Probe API version (sanity)
    api = msp_request(ser, MSP_API_VERSION)
    if api:
        print(f"[MSP] API_VERSION raw: {list(api)}")
    else:
        print("[MSP] WARNING: No API_VERSION reply (is MSP enabled on USB VCP?)")

    # MQTT
    mqttc = mqtt_connect_loop()

    period = 1.0 / POLL_HZ
    next_tick = time.time()

    while not stop_flag.is_set():
        start = time.time()

        att = msp_request(ser, MSP_ATTITUDE)
        ana = msp_request(ser, MSP_ANALOG)
        gps = msp_request(ser, MSP_GPS)

        payload = {}

        # ATTITUDE: int16 roll,pitch,yaw (tenths of degree for roll/pitch)
        if att and len(att) >= 6:
            roll, pitch, yaw = struct.unpack("<hhh", att[:6])
            payload["Roll"]  = roll / 10.0
            payload["Pitch"] = pitch / 10.0
            payload["Yaw"]   = float(yaw)
        else:
            # keep previous / skip if missing
            pass

        # ANALOG: vbat (deciVolts) + (amperage depends on config/BF version)
        # Common BF layout: [0..1]=vbat_dV (uint16), [2..3]=mAh (uint16), [4]=rssi (uint8), [5]=amperage (uint8 or part of int16), [6..7]=amperage*X (int16)
        if ana and len(ana) >= 2:
            vbat_dV = struct.unpack("<H", ana[0:2])[0]
            payload["Volts"] = vbat_dV / 10.0
            if len(ana) >= 8:
                amps_raw = struct.unpack("<h", ana[6:8])[0]
                # Usually 0.01 A per LSB; adjust if values look off
                payload["Amps"] = amps_raw / 100.0

        # GPS: fix(1B), sats(1B), lat(i32), lon(i32), alt(u16), speed(u16), ground course(u16)
        if gps and len(gps) >= 12:
            payload["Fix"]  = int(gps[0])
            payload["Sats"] = int(gps[1])
            payload["Lat"]  = struct.unpack("<i", gps[2:6])[0] / 10_000_000.0
            payload["Lon"]  = struct.unpack("<i", gps[6:10])[0] / 10_000_000.0
            payload["Alt"]  = struct.unpack("<H", gps[10:12])[0]  # meters

        # Publish only if we have at least one field
        if payload:
            j = json.dumps(payload, separators=(",", ":"))
            mqttc.publish(MQTT_TOPIC, j, qos=1)
            # Debug print:
            print("[PUB]", j)

        # pacing
        next_tick += period
        sleep_for = max(0.0, next_tick - time.time())
        time.sleep(sleep_for)

    # Cleanup
    try:
        mqttc.loop_stop()
        mqttc.disconnect()
    except Exception:
        pass
    try:
        ser.close()
    except Exception:
        pass
    print("Done.")

if __name__ == "__main__":
    main()