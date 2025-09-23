#!/usr/bin/env python3

import rclpy
from rclpy.node import Node, QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, Duration
from serial_comms.serial_handler import SerialHandler
from serial_msgs.msg import MotorCurrents
from serial_msgs.msg import Feedback

MOTOR_CURRENT_MSG = 0
SEND_DELAY_SEC = 0.02
RECV_DELAY_SEC = 0.03
MOTOR_STILL = 127

class SerialNode(Node):

    def __init__(self):
        self.data = [MOTOR_STILL]*2 # [0:1]:2 wheels
        super().__init__('serial_node')
        self.subscription = self.create_subscription(
            msg_type=MotorCurrents,
            topic='motor_currents',
            callback=self.listener_callback,
            qos_profile=QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth= 1, reliability=QoSReliabilityPolicy.RELIABLE)) #1 queued message
        # FEEDBACK COMMENTED OUT FOR NOW
        # self.feedback_publisher = self.create_publisher(
        #     msg_type=Feedback,
        #     topic='feedback',
        #     qos_profile=1
        # )
        self.send_timer = self.create_timer(SEND_DELAY_SEC, self.sendCurrents)
        self.recv_timer = self.create_timer(RECV_DELAY_SEC, self.readFromNucleo)
        self.serial_handler = SerialHandler()

    def listener_callback(self, msg):
        # self.get_logger().warn("Received motor query")
        # motors = ["FL", "FR", "BL", "BR"]
        self.data[0] = msg.left_wheels
        self.data[1] = msg.right_wheels 

    def sendCurrents(self):
        #print(ok)
        self.get_logger().info(f"Sending currents: {self.data}")
        self.serial_handler.send(MOTOR_CURRENT_MSG, self.data, self.get_logger())
        
    def readFromNucleo(self):
        data = self.serial_handler.readMsg(logger=self.get_logger())
        if data:
            mf = Feedback(us_sensor = data[0])
            self.get_logger().warn(f"Ultrasonic data: {data[0]}")
            # self.feedback_publisher.publish(mf)
        else:
            self.get_logger().warn("no data")

def main(args=None):
    rclpy.init(args=args)

    node = SerialNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
