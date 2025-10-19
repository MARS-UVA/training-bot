import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetection as AprilTagDetectionMsg
from apriltag_msgs.msg import AprilTagDetections
from cv_bridge import CvBridge
import cv2
from apriltag_pose_estimation.core import AprilTagDetector

class ApriltagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        self.get_logger().info("AprilTag detector node started")

        # Publisher for AprilTag detections
        self.publisher_ = self.create_publisher(
            AprilTagDetections, 'awareness/apriltags', 10)

        # Subscriber for raw images
        self.subscription = self.create_subscription(
            Image, 'awareness/image_raw', self.image_callback, 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Initialize the AprilTag detector from the new library
        self.detector = AprilTagDetector(families='tagstandar41h12',
                                         nthreads=1,
                                         quad_decimate=1.0,
                                         quad_sigma=0.0,
                                         refine_edges=True,
                                         decode_sharpening=0.25,
                                         debug=False)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Get image dimensions
        image_height, image_width, _ = cv_image.shape

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detect AprilTags
        detections = self.detector.detect(gray)
        
        # Create a message to publish
        detection_array_msg = AprilTagDetections()
        detection_array_msg.header = msg.header
        
        for detection in detections:
            # Create an instance of the AprilTagDetection message
            detection_msg = AprilTagDetectionMsg()

            # Populate the basic fields
            detection_msg.tag_id = detection.tag_id
            detection_msg.family = detection.tag_family

            # Original center and corners from the detector
            center = detection.center
            corners = detection.corners

            # 1. Shift coordinate system to the center of the tag
            corners_centered = corners - center

            # 2. Normalize coordinates
            normalized_center = [(center[0] / image_width) - 0.5, (center[1] / image_width) - 0.5]
            normalized_corners = corners_centered / image_width
            
            # Populate the center and corner fields of the message
            detection_msg.center = normalized_center
            
            detection_msg.corner1 = normalized_corners[0].tolist()
            detection_msg.corner2 = normalized_corners[1].tolist()
            detection_msg.corner3 = normalized_corners[2].tolist()
            detection_msg.corner4 = normalized_corners[3].tolist()

            detection_array_msg.detections.append(detection_msg)

        # Publish the detections
        if len(detection_array_msg.detections) > 0:
            self.publisher_.publish(detection_array_msg)
            self.get_logger().info(f"Published {len(detection_array_msg.detections)} AprilTag detections.")    
            
        


def main(args=None):
    rclpy.init(args=args)
    apriltag_detector = ApriltagDetector()
    rclpy.spin(apriltag_detector)
    apriltag_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()