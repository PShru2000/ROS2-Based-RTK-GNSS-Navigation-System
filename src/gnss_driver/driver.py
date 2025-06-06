#!/usr/bin/env python3
# -- coding: utf-8 --

import rclpy
from rclpy.node import Node
import serial
import utm
import glob
import argparse
import sys
from gps_msgs.msg import GpsMsg
from std_msgs.msg import Header

def detect_serial_port():
    """
    Automatically detects the first available /dev/ttyUSB* or /dev/pts/* port.
    If no ports are found, it returns None.
    """
    possible_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/pts/*')
    return possible_ports[0] if possible_ports else None

class GPSDriver(Node):
    """
    ROS2 Node for reading GPS data over serial and publishing it as a custom GPS message.
    """

    def __init__(self, port):
        super().__init__('gps_driver')

        self.publisher_ = self.create_publisher(GpsMsg, '/gps', 10)

        # Serial port setup
        try:
            self.serial_port = serial.Serial(port, baudrate=4800, timeout=1)
            self.get_logger().info(f"Connected to GPS device on {port}")
        except serial.SerialException as e:
            self.get_logger().fatal(f"Failed to open serial port: {e}")
            self.serial_port = None  # Avoid AttributeError in destroy_node()
            return

        # Timer to read data at 10Hz (every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.read_serial_data)

    def read_serial_data(self):
        """
        Reads GPS data from the serial port, processes it, and publishes it.
        """
        if not self.serial_port:
            return  # If serial port is not available, do nothing

        try:
            raw_data = self.serial_port.readline().decode('utf-8').strip()
            if raw_data.startswith("$GNGGA"):  # Check for GPGGA sentence
                self.get_logger().info(f"Received: {raw_data}")
                parsed_data = self.parse_gpgga(raw_data)
                if parsed_data:
                    # Convert lat/lon to UTM
                    utm_data = self.convert_to_utm(*parsed_data[1:5])
                    # Create the GPS message
                    msg = self.create_gps_msg(parsed_data, utm_data, parsed_data[0])
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published: {msg}")
        except Exception as e:
            self.get_logger().error(f"Error reading or publishing data: {e}")

    def parse_gpgga(self, data):
        """
        Parse the GNGGA string into individual GPS components.
        """
        fields = data.split(',')
        if len(fields) < 15:
            self.get_logger().error("Incomplete GNGGA string.")
            return None

        try:
            utc_time = float(fields[1]) if fields[1] else 0.0
            lat_raw = fields[2] if fields[2] else '0'
            lat_dir = fields[3]
            lon_raw = fields[4] if fields[4] else '0'
            lon_dir = fields[5]
            altitude = float(fields[9]) if fields[9] else 0.0
            hdop = float(fields[8]) if fields[8] else 0.0

            return utc_time, lat_raw, lat_dir, lon_raw, lon_dir, altitude, hdop
        except Exception as e:
            self.get_logger().error(f"Error parsing GPGGA string: {e}")
            return None

    def convert_to_utm(self, lat_raw, lat_dir, lon_raw, lon_dir):
        """
        Convert raw latitude and longitude values to UTM coordinates.
        """
        latitude = float(lat_raw[:2]) + float(lat_raw[2:]) / 60
        if lat_dir == 'S':
            latitude = -latitude

        longitude = float(lon_raw[:3]) + float(lon_raw[3:]) / 60
        if lon_dir == 'W':
            longitude = -longitude

        easting, northing, zone_number, zone_letter = utm.from_latlon(latitude, longitude)
        return latitude, longitude, easting, northing, zone_number, zone_letter

    def create_gps_msg(self, parsed_data, utm_data, utc_time):
        """
        Populate the GPS message with parsed and converted data.
        """
        msg = GpsMsg()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "GPS1_FRAME"
        msg.latitude, msg.longitude = utm_data[:2]
        msg.utm_easting, msg.utm_northing = utm_data[2:4]
        msg.altitude = parsed_data[5]
        msg.hdop = parsed_data[6]
        msg.zone = str(utm_data[4])
        msg.letter = utm_data[5]
        return msg

    def destroy_node(self):
        """
        Close the serial port when the node is destroyed.
        """
        if hasattr(self, 'serial_port') and self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            if rclpy.ok():  # Log only if ROS 2 is still running
                self.get_logger().info("Serial connection closed.")
        super().destroy_node()


def main(args=None):
    """
    Main function to initialize the ROS2 node and run it.
    """
    rclpy.init(args=args)

    # Auto-detect available serial port
    default_port = detect_serial_port()
    if not default_port:
        print("No available serial port found!")
        return

    parser = argparse.ArgumentParser(description="ROS2 GPS Driver")
    parser.add_argument('--port', type=str, default=default_port, help="Serial port for GPS device")
    parsed_args, unknown = parser.parse_known_args()

    gps_node = GPSDriver(parsed_args.port)

    try:
        rclpy.spin(gps_node)
    except KeyboardInterrupt:
        if rclpy.ok():
            gps_node.get_logger().info("GPS Driver shutting down...")
    finally:
        gps_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

