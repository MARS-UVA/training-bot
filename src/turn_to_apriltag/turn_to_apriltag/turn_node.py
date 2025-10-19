import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetections
from geometry_msgs.msg import Twist
from turn_to_apriltag.PID import PID
import threading

#PID controller tunable variable defaults
CONSTANT_P = 1
CONSTANT_I = 1
CONSTANT_D = 1

#Acceptable range of error from the center irrelevant of actual width
UPPER_X_ERROR = 0.1
LOWER_X_ERROR = -0.1

class TurnToAprilTagNode (Node):
    def __init__ (self):
        super().__init__('turn_to_apriltag')

        #Determine CONSTANT_P, CONSTANT_I, CONSTANT_D
        self.declare_parameter('parameter_p', CONSTANT_P)
        self.declare_parameter('parameter_i', CONSTANT_I)
        self.declare_parameter('parameter_d', CONSTANT_D)

        self.__pid_lock = threading.Lock()

        #subscribed to detection_callbacks
        self.sub = self.create_subscription(AprilTagDetections, 'awareness/apriltags', self.detection_callback, 10)
        
        #publish to twist
        self.pub = self.create_publisher(Twist, 'control/twist', 10)

        #intializes PID controller with the tunable variables
        self.parameter_p = self.get_parameter('parameter_p').get_parameter_value().double_value
        self.parameter_i = self.get_parameter('parameter_i').get_parameter_value().double_value
        self.parameter_d = self.get_parameter('parameter_d').get_parameter_value().double_value
        self.controller = PID(self.parameter_p, self.parameter_i, self.parameter_d)

    def detection_callback(self, msg: AprilTagDetections):
        #Robot motor controller which this will publish to (update)
        twist = Twist()

        #Leave if there is no detection
        if not msg.detections:
            return 
        
        #pulls all tags from msg
        for tag in msg.detections:
            #selects only tag id 0
            if tag.tag_id == 0:
                input_time = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
                self.set_twist_turn(tag, twist, input_time)
                
    def __parameters_update_callback(self, parameters: list[rclpy.Parameter]) -> None:
        CHEESE = 0
    def set_twist_turn(self, tag, twist:Twist, input_time):
        #stops all movement
        twist.linear.x = 0.0

        #apriltag center
        april_tag_center = tag.center
        
        #difference between center of tag and the center of the camera
        offset_x = (april_tag_center[0])

        #give the current time
        time = input_time

        #set goal to be 0 which is the center of the image
        self.controller.set_goal(0)
        
        #calculates PID controller values
        controller_value = self.controller.get_value(time, offset_x)

        #calculates and decides the way to turn
        if (offset_x < LOWER_X_ERROR):
            print(f"(left) {offset_x}")
            twist.angular.z = controller_value
        elif (offset_x > UPPER_X_ERROR):
            print(f"(right) {offset_x}")
            twist.angular.z = controller_value
        else:
            print(f"(center) {offset_x}")   
            twist.angular.z = 0.0  
        

        #publish to twist
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

