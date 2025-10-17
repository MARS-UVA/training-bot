import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetections
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import math

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
                if (offset_x < -5):
                    print(f"(left) {offset_x}")
                    twist.angular.z = 0.2
                elif (offset_x > 5):
                    print(f"(right) {offset_x}")
                    twist.angular.z = -0.2
                else:
                    print(f"(center) {offset_x}")   
                    twist.angular.z = 0.0  
                    #-----------------------------------------------
                    # New part for also driving forward/backward
                    #-----------------------------------------------
                    image_size = math.dist(tag.corners[0], tag.corners[2])
                    ideal_size = 150
                    tolerance = 5
                    if(image_size < ideal_size - tolerance):
                        twist.linear.x = 0.1
                        print(f"(forward) size: {image_size}")
                    elif(image_size > ideal_size + tolerance):
                        twist.linear.x = -0.1
                        print(f"(backward) size: {image_size}")
                    else:
                        twist.linear.x = 0.0
                        print("Staying stationary")
                    #-----------------------------------------------


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

