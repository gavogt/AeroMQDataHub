# 🚀 **AeroMQ DataHub**  
**Real-time drone telemetry dashboard** powered by **.NET MAUI Blazor**, **SignalR**, and **MQTT** — with **Raspberry Pi** flight telemetry (MSP) and **Pi Camera** snapshots.

<p align="center">
  <img src="docs/screenshot-home.jpg" alt="AeroMQ DataHub UI" width="820">
</p>

<p align="center">
  <a href="#"><img alt=".NET" src="https://img.shields.io/badge/.NET-9.0-512BD4?logo=dotnet&logoColor=white"></a>
  <a href="#"><img alt="SignalR" src="https://img.shields.io/badge/SignalR-Realtime-00B5E2?logo=azuredevops&logoColor=white"></a>
  <a href="#"><img alt="MQTT" src="https://img.shields.io/badge/MQTT-Mosquitto-660066?logo=eclipsemosquitto&logoColor=white"></a>
  <a href="#"><img alt="Raspberry Pi" src="https://img.shields.io/badge/Raspberry%20Pi-Bookworm-C51A4A?logo=raspberrypi&logoColor=white"></a>
  <a href="#"><img alt="License" src="https://img.shields.io/badge/License-MIT-2ea44f"></a>
</p>

---

## 🧭 Table of Contents
- [✨ Features](#-features)
- [🧱 Architecture](#-architecture)
- [🛠 Tech Stack](#-tech-stack)
- [🚀 Getting Started](#-getting-started)
  - [1) MQTT Broker](#1-mqtt-broker)
  - [2) Raspberry Pi Publisher](#2-raspberry-pi-publisher)
  - [3) ASP.NET Core Server](#3-aspnet-core-server)
  - [4) MAUI Blazor UI](#4-maui-blazor-ui)
- [📦 Message Topics & Schemas](#-message-topics--schemas)
- [🖥 UI Highlights](#-ui-highlights)
- [🔧 Troubleshooting](#-troubleshooting)
- [🗺 Roadmap](#-roadmap)
- [📝 License](#-license)
- [🙌 Credits](#-credits)

---

## ✨ Features
- **Attitude**: Roll / Pitch / Yaw, **Armed** state, **Mode**, **Cycle Time**
- **RC & Baro**: RC channels (roll/pitch/yaw/throttle), Baro Altitude & Vertical Speed
- **Battery**: Voltage, Current, mAh consumed
- **GPS**: Fix, Sats, Lat/Lon, Altitude, Speed, Ground Course (+ optional mini-map)
- **Camera**: Pi Camera snapshots (JPEG) published to MQTT every _N_ seconds
- **Neon/Cyberpunk UI** themed to match an alien skater backstory 😎

---

## 🧱 Architecture

```
[Betaflight FC]
   └── MSP over USB (/dev/ttyACM0)
           │
           ├── Telemetry (pyserial) ┐
           └── PiCam (Picamera2)    ├─> MQTT Broker (Mosquitto, 1883)
                                     │     • flight/telemetry
                                     │     • flight/camera
                                     │
ASP.NET Core (SignalR Hub @ http://<host>:8080/telemetryHub)
           │
MAUI Blazor UI (SignalR client → live dashboard)
```

---

## 🛠 Tech Stack
- **Raspberry Pi 4 + Raspberry Pi OS (Bookworm)**
  - MSP: `pyserial`
  - MQTT: `paho-mqtt`
  - Camera: `python3-picamera2` + `Pillow`
- **Broker**: Mosquitto (local or LAN)
- **Server**: ASP.NET Core (.NET 9)
  - SignalR hub (`/telemetryHub`)
  - `MqttBridgeService` → subscribes to MQTT and forwards to SignalR
- **Client**: .NET **MAUI Blazor** page (your `Home.razor`)

---

## 🚀 Getting Started

### 1) MQTT Broker
```bash
sudo apt update
sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable --now mosquitto
# test
mosquitto_sub -h localhost -t test -v &
mosquitto_pub -h localhost -t test -m hi
```

For LAN access edit `/etc/mosquitto/mosquitto.conf` (dev settings):
```conf
listener 1883 0.0.0.0
allow_anonymous true
```
> In production, disable anonymous and use `password_file`.

Restart:
```bash
sudo systemctl restart mosquitto
```

### 2) Raspberry Pi Publisher
Install dependencies:
```bash
sudo apt update
sudo apt install -y python3-picamera2
pip install paho-mqtt pyserial pillow
```

Run the combined publisher (MSP + PiCam):
```bash
python3 picam_comb.py
```
Update `MQTT_HOST`, topics, and camera interval in the script.

> **Betaflight tips**  
> • Ports: enable **MSP** on **USB VCP** (115200).  
> • GPS on UART (if used).  
> • For Volts/Amps you need a LiPo + current sensor configured.

### 3) ASP.NET Core Server
The server listens on `http://0.0.0.0:8080` and exposes the SignalR hub at `/telemetryHub`.
```bash
dotnet run --project FlightTelemetryGateway
```

### 4) MAUI Blazor UI
The dashboard page connects to the hub and renders telemetry + camera.  
For desktop: `http://localhost:8080/telemetryHub`.  
For mobile emulators, use the appropriate host (e.g., `10.0.2.2:8080` for Android).

---

## 📦 Message Topics & Schemas

### Topic: `flight/telemetry` (JSON, PascalCase)
```json
{
  "Roll": -16.2, "Pitch": 6.5, "Yaw": 98,
  "Volts": 12.3, "Amps": 10.5, "Mah": 345,
  "Armed": true, "Mode": "ACRO", "CycleTimeUs": 125,
  "RcRoll": 1486, "RcPitch": 1472, "RcYaw": 1525, "RcThrottle": 1493,
  "BaroAltM": 0.12, "VSpeedMs": 0.0,
  "Fix": 1, "Sats": 6, "Lat": 33.5194431, "Lon": -112.106055, "Alt": 345,
  "GpsSpeedMs": 0.2, "GpsCourseDeg": 11.4, "GpsRescue": false
}
```

### Topic: `flight/camera` (JSON with base64 JPEG)
```json
{
  "Timestamp": "2025-10-04T18:53:14Z",
  "Format": "jpeg",
  "Width": 1280,
  "Height": 720,
  "Quality": 85,
  "Data": "<base64-encoded-jpeg>"
}
```

---

## 🖥 UI Highlights
- Neon “alien” theme (cards, badges, icons) to match the dashboard aesthetic.
- Attitude, RC/Baro, Voltage, Current, GPS blocks.
- Live PiCam panel with timestamp & metadata.
- Optional backstory card for **Kryx of Reticula‑7 (Skater Zeta)**.

---

## 🔧 Troubleshooting

**SignalR “connection refused”**  
- Hub not listening on 8080 or HTTP→HTTPS redirect enabled without HTTPS binding.  
  Remove `UseHttpsRedirection` for dev or configure HTTPS + use `https://…`.

**MQTT won’t connect**  
- Broker bound to `127.0.0.1`. Change to `0.0.0.0` (or LAN IP) and open TCP **1883**.

**Camera errors: “Device or resource busy / pipeline handler in use”**  
- Another process uses the camera.  
  ```bash
  sudo fuser -v /dev/video* /dev/media*
  sudo pkill -f 'rpicam|libcamera|picamera2|motion|v4l2|rtsp'
  ```
- Ensure Legacy Camera is **OFF** in `raspi-config`.  
- Test: `rpicam-still -n -o test.jpg`.

**Voltage/Current always 0**  
- USB only → plug a LiPo. Configure Battery & Current Sensor in Betaflight.

**Mobile client can’t reach hub**  
- Use device‑reachable host (emulator `10.0.2.2:<port>` or PC’s LAN IP).  
- Open OS firewall for **8080** inbound.

---

## 🗺 Roadmap
- ESC telemetry (RPM/Temp/Per‑motor current) via `MSP_ESC_SENSOR_DATA`
- Stream video (H.264 over RTSP or fMP4 over HTTP)
- History/analytics (InfluxDB/SQLite + sparklines)
- Geofence & RTH warnings

---

## 📝 License
MIT — see `LICENSE`.

---

## 🙌 Credits
- Betaflight MSP docs & community  
- Raspberry Pi / Picamera2  
- Mosquitto MQTT  
- .NET MAUI & SignalR
