#!/usr/bin/env python3
import time, json, struct, serial
import paho.mqtt.client as mqtt

# ----- serial link -------------------------------------------------------------
port = serial.Serial('/dev/ttyACM0', 115200, timeout=.3)

def msp(cmd):
    frame = b'$M<\x00' + bytes([cmd]) + bytes([cmd])
    port.write(frame)
    hdr  = port.read(5)
    ln   = hdr[3]
    data = port.read(ln)
    port.read(1)                 # checksum byte
    return data

# ---------- MQTT ---------------------------------------------------------------
mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id="fcPub")

def on_publish(client, userdata, mid):
    print(f"published #{mid}")

mqttc.on_publish = on_publish
mqttc.connect("xxx.xxx.x.xxxx", 1883, 60)
mqttc.loop_start()

ATTITUDE, BATTERY = 108, 130

while True:
    att = msp(ATTITUDE)
    bat = msp(BATTERY)

    if att and bat:
        roll, pitch, yaw = struct.unpack('<hhh', att[:6])
        vbat, amps       = struct.unpack('<hH',  bat[:4])
        payload = json.dumps({
            "roll":  roll  / 10,
            "pitch": pitch / 10,
            "yaw":   yaw,
            "volts": vbat  / 1000,
            "amps":  amps  / 100
        })
        mqttc.publish("flight/telemetry", payload, qos=0)

    time.sleep(0.2)