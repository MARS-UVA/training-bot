import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetections
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import PID

CONSTANT_p = 1
CONSTANT_i = 1
CONSTANT_d = 1

class TurnToAprilTagNode (Node):
    def __init__ (self):
        super().__init__('turn_to_apriltag')
        self.sub = self.create_subscription(AprilTagDetections, 'awareness/apriltags', self.detection_callback, 10)
        self.image_sub = self.create_subscription(Image, 'awareness/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Twist, 'control/twist', 10)

        self.width = 640
        self.height = 480
        

    def image_callback(self, msg : Image):
        self.width = msg.width
        self.height = msg.height

    def detection_callback(self, msg):
        twist = Twist()
        if not msg.detections:
            return 
        for tag in msg.detections:
            if tag.tag_id == 0:
                twist.linear.x = 0.0
                sw = self.width
                center_sx = sw//2
                ac = tag.center
                offset_x = ac[0] - center_sx
                time = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
                controller = PID(CONSTANT_p, CONSTANT_i, CONSTANT_d)
                controller.set_goal(center_sx)
                while True:
                    controller_value = controller.get_value(time, offset_x)
                    if (offset_x < -5):
                        print(f"(left) {offset_x}")
                        twist.angular.z = controller_value
                    elif (offset_x > 5):
                        print(f"(right) {offset_x}")
                        twist.angular.z = controller_value
                    else:
                        print(f"(center) {offset_x}")   
                        twist.angular.z = 0.0  

                    self.pub.publish(twist)
                    return


def main():
    rclpy.init()
    node = TurnToAprilTagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

