#!/usr/bin/env python3
"""
Serial test for ROS2_Comm project.
Builds a frame matching `crc.h` layout: [START][payload][CRC_L][CRC_H][STOP]
Sends it to the MCU over the USB CDC device (e.g. /dev/ttyACM0) and prints any responses.

Usage:
  python3 tools/serial_test.py --port /dev/ttyACM0
"""
import argparse
import serial
import struct
import time

START = 0xAA
STOP = 0xBB

# CRC16 from crc.c (same polynomial 0x8408 reflected)
def crc16_compute(data: bytes) -> int:
    crc = 0x0000
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = ((crc >> 1) ^ 0x8408) & 0xFFFF
            else:
                crc = (crc >> 1) & 0xFFFF
    return crc


def build_frame(payload: bytes) -> bytes:
    frame = bytearray()
    frame.append(START)
    frame.extend(payload)
    crc = crc16_compute(payload)
    frame.append(crc & 0xFF)
    frame.append((crc >> 8) & 0xFF)
    frame.append(STOP)
    return bytes(frame)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port','-p', required=True)
    parser.add_argument('--baud','-b', type=int, default=115200)
    args = parser.parse_args()

    s = serial.Serial(args.port, args.baud, timeout=1)
    payload = b'host-ping'
    frame = build_frame(payload)
    print('Sending:', frame.hex())
    s.write(frame)
    time.sleep(0.1)
    data = s.read(512)
    if data:
        print('Received:', data.hex())
    else:
        print('No response')
    s.close()

if __name__ == '__main__':
    main()
