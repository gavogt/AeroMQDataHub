#!/usr/bin/env python3
import threading
import time
import json
import struct

import serial
from pyubx2 import UBXReader, UBX_PROTOCOL
import paho.mqtt.client as mqtt

# ===== Configuration =====

# MQTT broker
MQTT_BROKER = "192.168.0.204"
MQTT_PORT   = 1883
MQTT_TOPIC  = "flight/telemetry"

# Serial ports—set these to your two passtru-output ports
GPS_PORT = "/dev/ttyACM0"   # must be the GPS UART at 4800 baud
MSP_PORT = "/dev/ttyACM1"   # must be the MSP UART at 115200 baud

# GPS settings
GPS_BAUD    = 4800
GPS_TIMEOUT = 0.5

# MSP settings (exactly your original code)
MSP_BAUD     = 115200
MSP_TIMEOUT  = 0.3
ATTITUDE_CMD = 108
BATTERY_CMD  = 130

# ===== MQTT Setup =====

mqttc = mqtt.Client("PiTelemetryPub")
mqttc.connect(MQTT_BROKER, MQTT_PORT)
mqttc.loop_start()

# ===== GPS Thread =====

def gps_loop():
    try:
        ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=GPS_TIMEOUT)
    except Exception as e:
        print(f"[GPS] ERROR opening {GPS_PORT}: {e}")
        return

    ubr = UBXReader(ser, protfilter=UBX_PROTOCOL, quitonerror=True)
    print(f"[GPS] Listening on {GPS_PORT} @ {GPS_BAUD}")

    while True:
        try:
            raw, parsed = ubr.read()    # one NAV-PVT frame
        except Exception:
            continue

        if parsed and parsed.identity == 'NAV-PVT':
            payload = {
                "type":    "gps",
                "fixType": parsed.fixType,
                "lat":     parsed.lat   / 1e7,
                "lon":     parsed.lon   / 1e7,
                "alt_m":   parsed.height/1000,
                "sats":    parsed.numSV
            }
            print("[GPS]", payload)
            mqttc.publish(MQTT_TOPIC, json.dumps(payload), qos=0)

    # never reaches, but good form:
    # ser.close()

# ===== MSP Thread =====

def msp_loop():
    try:
        port = serial.Serial(MSP_PORT, MSP_BAUD, timeout=MSP_TIMEOUT)
    except Exception as e:
        print(f"[MSP] ERROR opening {MSP_PORT}: {e}")
        return

    print(f"[MSP] Polling on {MSP_PORT} @ {MSP_BAUD}")

    def msp(cmd):
        frame = b'$M<\x00' + bytes([cmd]) + bytes([cmd])
        port.write(frame)
        hdr = port.read(5)
        if len(hdr) != 5 or not hdr.startswith(b'$M<'):
            return None
        length = hdr[3]
        data = port.read(length)
        port.read(1)  # checksum
        return data

    while True:
        att = msp(ATTITUDE_CMD)
        bat = msp(BATTERY_CMD)

        if att and bat:
            roll, pitch, yaw = struct.unpack('<hhh', att[:6])
            vbat, amps       = struct.unpack('<hH', bat[:4])
            payload = {
                "type":  "msp",
                "roll":  roll  / 10,
                "pitch": pitch / 10,
                "yaw":   yaw,
                "volts": vbat  / 1000,
                "amps":  amps  / 100
            }
            print("[MSP]", payload)
            mqttc.publish(MQTT_TOPIC, json.dumps(payload), qos=0)

        time.sleep(0.2)

    # port.close()

# ===== Main =====

if __name__ == "__main__":
    # Start both loops in background threads
    threading.Thread(target=gps_loop, daemon=True).start()
    threading.Thread(target=msp_loop, daemon=True).start()

    print("Telemetry bridge running. Ctrl-C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down…")
    finally:
        mqttc.loop_stop()
        mqttc.disconnect()
        print("Exited.")