# Information about a detected AprilTag.
#
# Corner points are guaranteed to be provided in clockwise order.

std_msgs/Header header  # Header containing information about the original image capture.

uint16 tag_id  # ID of the detected AprilTag.
string<=16 tag_family  # Family of the detected AprilTag.

float64[2] center  # The location of the center in the camera view (as a proportion of the original image's width).

float64[2] corner1  # The first corner point as a coordinate in the camera view (as a proportion of the original image's width).
float64[2] corner2  # The second corner point as a coordinate in the camera view (as a proportion of the original image's width).
float64[2] corner3  # The third corner point as a coordinate in the camera view (as a proportion of the original image's width).
float64[2] corner4  # The fourth corner point as a coordinate in the camera view (as a proportion of the original image's width).
