#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class SerialTransmitterNode(Node):
    def __init__(self):
        super().__init__("serial_transmitter_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)

        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        self.sub_ = self.create_subscription(String, "serial_transmitter", self.msgCallback, 10)
        self.pico_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)

    def msgCallback(self, msg):
        #self.get_logger().info("New message received, publishing on serial: %s" % self.pico_.name)
        self.pico_.write(msg.data.encode("utf-8"))


def main():
    rclpy.init()

    serial_transmitter_node = SerialTransmitterNode()
    rclpy.spin(serial_transmitter_node)
    
    serial_transmitter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()