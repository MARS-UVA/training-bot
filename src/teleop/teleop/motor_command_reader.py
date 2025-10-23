#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt8MultiArray
from serial_msgs import MotorChanges

class MotorCommandNode(Node):
    def __init__(self):
        super().__init__('motor_command_node')

        # ROS Subscriber: listens to velocity commands
        self.subscription = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # ROS Publisher: publishes motor values
        self.publisher = self.create_publisher(MotorChanges, '/teleop', 10)

        # Initialize motor values to neutral
        self.motor_currents = [127] * 6  # [tl, bl, tr, br, drum, actuator]

    def cmd_vel_callback(self, msg):
        linear = msg.twist.linear.x
        angular = msg.twist.angular.z

        self.motor_currents = self.twist_to_motor_currents(linear, angular)

        # Publish over ROS
        self.publisher.publish(out_msg)

        self.get_logger().info(f'Published & sent motor currents: {self.motor_currents}')

    def twist_to_motor_currents(self, linear, angular):
        max_speed = 1.0  # adjust to your system

        def speed_to_byte(speed):
            speed = max(min(speed, max_speed), -max_speed)
            return int(127 + (speed / max_speed) * 127)

        left_speed = linear - angular
        right_speed = linear + angular

        motor_currents = [127] * 6
        motor_currents[0] = speed_to_byte(left_speed)   # top-left
        motor_currents[1] = speed_to_byte(left_speed)   # bottom-left
        motor_currents[2] = speed_to_byte(right_speed)  # top-right
        motor_currents[3] = speed_to_byte(right_speed)  # bottom-right
        motor_currents[4] = 127  # drum (neutral)
        motor_currents[5] = 127  # actuator (neutral)

        return motor_currents


def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
