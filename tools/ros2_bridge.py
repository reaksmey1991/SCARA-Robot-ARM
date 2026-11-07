#!/usr/bin/env python3
"""
Simple ROS2 <-> STM32 USB CDC bridge.
- Reads frames from serial, parses them, and republishes as ROS2 topics
- Subscribes to ROS2 command topics and sends framed messages to the device

Usage:
  source /opt/ros/<distro>/setup.bash
  python3 tools/ros2_bridge.py --port /dev/ttyACM0

This example maps:
 - /robot/cmd_speeds (Float32MultiArray) -> MSG_CMD_SET_SPEEDS (3 floats)
 - /robot/telemetry (Float32MultiArray) <- MSG_TELEMETRY (uptime,batt_mv,...) (example)

Note: run this on the host (PC) that has ROS2 installed.
"""

import argparse
import struct
import threading
import time
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Protocol constants (must match firmware)
COMM_START1 = 0xAA
COMM_START2 = 0x55
COMM_VERSION = 0x01
MSG_TELEMETRY = 0x10
MSG_SENSOR_RANGE = 0x20
MSG_CMD_SET_SPEEDS = 0x30
MSG_CMD_ENABLE_FLAGS = 0x31
MSG_ACK = 0x40
MSG_LOG = 0x50

# CRC16-CCITT
def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

class FrameParser:
    def __init__(self, handler):
        self.handler = handler
        self.state = 0
        self.version = 0
        self.msgid = 0
        self.len = 0
        self.payload = bytearray()
        self.crc = bytearray()

    def feed(self, data: bytes):
        for b in data:
            if self.state == 0:
                if b == COMM_START1:
                    self.state = 1
            elif self.state == 1:
                if b == COMM_START2:
                    self.state = 2
                else:
                    self.state = 0
            elif self.state == 2:
                self.version = b
                if self.version == COMM_VERSION:
                    self.state = 3
                else:
                    self.state = 0
            elif self.state == 3:
                self.msgid = b
                self.state = 4
            elif self.state == 4:
                self.len = b
                self.payload = bytearray()
                if self.len == 0:
                    self.state = 6
                    self.crc = bytearray()
                else:
                    self.state = 5
            elif self.state == 5:
                self.payload.append(b)
                if len(self.payload) >= self.len:
                    self.state = 6
                    self.crc = bytearray()
            elif self.state == 6:
                self.crc.append(b)
                if len(self.crc) == 2:
                    received = self.crc[0] | (self.crc[1] << 8)
                    comp = crc16_ccitt(bytes([self.version, self.msgid, self.len]) + bytes(self.payload))
                    if comp == received:
                        try:
                            self.handler(self.msgid, bytes(self.payload))
                        except Exception as e:
                            print('Handler error:', e)
                    else:
                        print('CRC mismatch: got', hex(received), 'comp', hex(comp))
                    self.state = 0

class Ros2Bridge(Node):
    def __init__(self, port, baud=115200):
        super().__init__('stm32_bridge')
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.parser = FrameParser(self.handle_frame)

        # Publishers and subscribers
        self.tele_pub = self.create_publisher(Float32MultiArray, 'robot/telemetry', 10)
        self.cmd_sub = self.create_subscription(Float32MultiArray, 'robot/cmd_speeds', self.on_cmd_speeds, 10)

        self._stop = False
        self._thread = threading.Thread(target=self._reader_thread, daemon=True)
        self._thread.start()
        self.get_logger().info(f'Opened serial {port} @ {baud}')

    def _reader_thread(self):
        while not self._stop and self.ser.is_open:
            try:
                data = self.ser.read(256)
                if data:
                    self.parser.feed(data)
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(0.5)

    def handle_frame(self, msgid, payload: bytes):
        if msgid == MSG_TELEMETRY:
            # Example telemetry: uptime_ms (uint32), battery_mv (uint16), fault_flags (uint16)
            if len(payload) >= 8:
                uptime, batt, flags = struct.unpack('<IHH', payload[:8])
                msg = Float32MultiArray()
                msg.data = [float(uptime), float(batt), float(flags)]
                self.tele_pub.publish(msg)
                self.get_logger().debug(f'Telemetry: uptime={uptime} batt={batt} flags={flags}')
        elif msgid == MSG_LOG:
            try:
                print('LOG:', payload.decode('ascii', errors='ignore'))
            except:
                pass
        else:
            self.get_logger().debug(f'Received msgid={hex(msgid)} len={len(payload)}')

    def on_cmd_speeds(self, msg: Float32MultiArray):
        # Expect 3 floats for this example
        arr = msg.data
        if len(arr) < 3:
            self.get_logger().warn('cmd_speeds expects at least 3 floats')
            return
        payload = struct.pack('<fff', float(arr[0]), float(arr[1]), float(arr[2]))
        self.send_frame(MSG_CMD_SET_SPEEDS, payload)

    def send_frame(self, msgid: int, payload: bytes):
        frame = bytearray()
        frame.append(COMM_START1)
        frame.append(COMM_START2)
        frame.append(COMM_VERSION)
        frame.append(msgid & 0xFF)
        frame.append(len(payload) & 0xFF)
        frame.extend(payload)
        crc = crc16_ccitt(bytes([COMM_VERSION, msgid & 0xFF, len(payload) & 0xFF]) + payload)
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)
        try:
            self.ser.write(frame)
        except Exception as e:
            self.get_logger().error(f'Failed to write frame: {e}')

    def destroy_node(self):
        self._stop = True
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', '-p', required=True, help='Serial port device (e.g. /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()

    rclpy.init(args=None)
    node = Ros2Bridge(args.port, args.baud)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
