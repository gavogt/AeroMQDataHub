#!/usr/bin/env python3
import os
import termios
import time

# The serial device to scan
PORT = '/dev/ttyACM0'

# Bauds to try
BAUDS = [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

# Build a map from integer → termios constant (where available)
BAUD_CONST = {
    baud: getattr(termios, f'B{baud}')
    for baud in BAUDS
    if hasattr(termios, f'B{baud}')
}

def set_baud(fd, baud):
    """Reconfigure the already-open fd to the given baud."""
    if baud not in BAUD_CONST:
        raise ValueError(f"Unsupported baud: {baud}")
    attrs = termios.tcgetattr(fd)
    # [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
    attrs[4] = BAUD_CONST[baud]
    attrs[5] = BAUD_CONST[baud]
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

def scan_bauds():
    fd = os.open(PORT, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        for baud in BAUDS:
            print(f"\n--- Testing {PORT} @ {baud} baud ---")
            try:
                set_baud(fd, baud)
            except Exception as e:
                print(f"[ERROR] Cannot set {baud}: {e}")
                continue

            time.sleep(0.5)  # let data buffer up
            try:
                data = os.read(fd, 64)
            except OSError as e:
                print(f"[ERROR] Read error: {e}")
                continue

            if data:
                print(f"[DATA] {len(data)} bytes at {baud}:")
                print(' '.join(f"{b:02X}" for b in data))
            else:
                print("[EMPTY] No data (timeout or wrong baud)")
    finally:
        os.close(fd)
        print("\nDone scanning all baud rates.")

if __name__ == "__main__":
    scan_bauds()