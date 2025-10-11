import cv2
import rclpy
import threading
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class WebcamCaptureNode(Node):
    def __init__(self, **kwargs):
        super().__init__("webcam_capture", **kwargs)

        self.declare_parameter('cv_cam_index', 0)
        self.add_post_set_parameters_callback(self.__parameters_update_callback)
        self.publisher = self.create_publisher(Image, "awareness/image_raw", 10)
        cv_cam_index = self.get_parameter('cv_cam_index').get_parameter_value().integer_value
        self.capture = cv2.VideoCapture(cv_cam_index, cv2.CAP_V4L2)
        self.cv_bridge = CvBridge()
        self.timer = self.create_timer(1/15, self.publish_frame)
        self.get_logger().info(f'Capturing from camera {cv_cam_index}')

        self.__capture_lock = threading.Lock()

    def publish_frame(self):
        if self.__capture_lock.locked():
            return
        with self.__capture_lock:
            ret, frame = self.capture.read()
        if ret:
            image_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(image_msg)

    def destroy_node(self):
        super().destroy_node()
        with self.__capture_lock:
            self.capture.release()

    def __parameters_update_callback(self, parameters: list[rclpy.Parameter]) -> None:
        cv_cam_index = next((param.get_parameter_value().integer_value
                             for param in parameters
                             if param.name == 'cv_cam_index'), None)
        if cv_cam_index is not None:
            with self.__capture_lock:
                self.capture.release()
                self.capture = cv2.VideoCapture(cv_cam_index, cv2.CAP_V4L2)
            self.get_logger().info(f'Changed camera being captured to {cv_cam_index}')


def main():
    rclpy.init()
    node = WebcamCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
