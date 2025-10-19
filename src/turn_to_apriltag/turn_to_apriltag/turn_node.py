import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetections
from geometry_msgs.msg import Twist
from turn_to_apriltag.PID import PID

##PID controller tunable variables
CONSTANT_P = 1
CONSTANT_I = 1
CONSTANT_D = 1

##Acceptable range of error from the center in pixels
UPPER_X_ERROR = 0.1
LOWER_X_ERROR = -0.1

class TurnToAprilTagNode (Node):
    def __init__ (self):
        super().__init__('turn_to_apriltag')
        ##where I publish
        self.sub = self.create_subscription(AprilTagDetections, 'awareness/apriltags', self.detection_callback, 10)
        self.pub = self.create_publisher(Twist, 'control/twist', 10)
        ##intializes PID controller with the tunable variables
        self.controller = PID(CONSTANT_P, CONSTANT_I, CONSTANT_D)

    def detection_callback(self, msg):
        ##Robot controller
        twist = Twist()
        ##Leave if there is no detection
        if not msg.detections:
            return 
        ##if there is a tag
        for tag in msg.detections:
            if tag.tag_id == 0:
                ##stops all movement
                twist.linear.x = 0.0

                ##aprilTag Center
                april_tag_center = tag.center
                
                ##difference between center of tag and the center of the camera
                offset_x = (april_tag_center[0])
                ##give the current time
                time = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9

                ##set goal to be 0 which is the center of the image
                self.controller.set_goal(0)
                
                ##calculates PID controller values
                controller_value = self.controller.get_value(time, offset_x)

                if (offset_x < LOWER_X_ERROR):
                    print(f"(left) {offset_x}")
                    twist.angular.z = controller_value
                elif (offset_x > UPPER_X_ERROR):
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

