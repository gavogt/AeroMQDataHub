#!/usr/bin/env python3
import serial
import time
from pyubx2 import UBXReader, UBX_PROTOCOL

PORT        = '/dev/ttyACM0'
BAUD        = 4800
TIMEOUT     = 0.5  # seconds

def main():
    # Open the serial port (catch all exceptions)
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    except Exception as e:
        print(f"[ERROR] Cannot open {PORT} @ {BAUD}: {e}")
        return

    # UBXReader: only UBX frames, swallow checksum errors
    ubr = UBXReader(
        ser,
        protfilter=UBX_PROTOCOL,
        quitonerror=False
    )

    print(f"Listening for UBX NAV-PVT on {PORT} @ {BAUD} (Ctrl-C to exit)…")

    try:
        while True:
            raw, parsed = ubr.read()  # returns (bytes, UBXMessage or None)
            # Skip any invalid or filtered frames
            if parsed is None:
                continue

            # Only NAV-PVT messages
            if parsed.identity == 'NAV-PVT':
                fix = parsed.fixType
                lat = parsed.lat / 1e7       # convert 1e-7° → deg
                lon = parsed.lon / 1e7
                alt = parsed.height / 1000   # mm → m
                sats = parsed.numSV

                status = "3D FIX" if fix == 3 else "no 3D fix"
                print(f"FixType: {fix} ({status}), "
                      f"Lat: {lat:.7f}, Lon: {lon:.7f}, "
                      f"Alt: {alt:.2f} m, Sats: {sats}")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == '__main__':
    main()