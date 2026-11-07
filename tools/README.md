ROS2 <-> STM32 bridge tools

Files:
- ros2_bridge.py : rclpy bridge node that maps frames to ROS2 topics and back.
- serial_test.py : simple host-side script to build and send frames (no ROS).

How to use:
1. Build and flash STM32 firmware that includes comm protocol support.
2. Connect the board; find device node (dmesg | tail or ls /dev/ttyACM*).
3. Test with: python3 tools/serial_test.py --port /dev/ttyACM0
4. Run bridge: source /opt/ros/<distro>/setup.bash && python3 tools/ros2_bridge.py --port /dev/ttyACM0

Requirements (host):
- Python3
- pip install pyserial rclpy (rclpy is provided by installing ROS2), std_msgs
