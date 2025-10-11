# Follow an AprilTag tasks

Now that you've learned the basics of ROS and AprilTags, it's time to start the onboarding project in full. I've provided two tasks to complete which will allow us to make significant progress on the project, and it would be great to start on them before we meet again this Friday.

Since there are two tasks, let's divide up into one group of 2 and one group of 3. You can choose the groups (or, if you want me to, I can assign the groups). Please don't spend more than 1 hour on this unless you want to.

Also, for each task, please create a new branch off of this one (`apriltags`).

## Setup

Before starting any of the tasks, please perform the following setup:

### In a VM/WSL

1. This project uses ROS2 Jazzy Jalisco instead of Kilted Kaiju, so you'll need to
[install](https://docs.ros.org/en/jazzy/Installation.html) that first (sorry, I didn't realize we were still using Jazzy on the training bot). You should also [uninstall Kilted Kaiju](https://roboticsbackend.com/how-to-uninstall-ros2-completely/).
1. Source the ROS2 environment with `source /opt/ros/jazzy/setup.bash`.
2. Update your local package repository by running `sudo apt update`.
3. Then install necessary dependencies:
   ```bash
   sudo apt install libsdl2-dev python3-pip python3.12-venv ninja-build xauth openssh-client wget at software-properties-common ros-jazzy-cv-bridge ros-jazzy-image-transport
   ```

4. Also install any necessary Python dependencies by running the `py_install_dependencies.py` script. You should also
   run this before any `colcon build`, in addition to any other dependency management stuff such as `apt` or `rosdep`.
5. Finally run `colcon build` to build your project.
6. If you want Visual Studio Code to give you better autocompletes, then open up the preferences JSON file with the
   command palette (`Ctrl-Shift-P`, run command "Preferences: Open Workspace Settings (JSON)") and add the following:
   ```json
   "python.analysis.extraPaths": [
     "/opt/ros/jazzy/lib/python3.12/site-packages/**",
     "${workspaceFolder}/install/*/lib/python3.12/site-packages/**"
   ],
   "python.autoComplete.extraPaths": [
     "/opt/ros/jazzy/lib/python3.12/site-packages/**",
     "${workspaceFolder}/install/*/lib/python3.12/site-packages/**"
   ],
   "C_Cpp.default.includePath": [
     "${default}",
     "/opt/ros/jazzy/include/**",
     "${workspaceFolder}/install/*/include/**",
     "${workspaceFolder}/src/*/include/**"
   ]
   ```

    If some of these keys already exist in your preferences file, just append the list elements.

### In a Dev container

This branch already has a Dev container configuration defined on it which will do all the setup for you except for
installing Python dependencies and building the project. To create a Dev container from the configuration, open the
command palette with `Ctrl-Shift-P` (`Cmd-Shift-P` on macOS) and run "Dev Containers: Reopen in Container".

## Existing packages

This branch contains the following packages:
* `apriltag_msgs` defines two message types which contain AprilTag detection information,
  `apriltag_msgs/msg/AprilTagDetection` and `apriltag_msgs/msg/AprilTagDetections`. You can check out their definitions
  by opening the message definition files or running `ros2 interface show` on each of them after a `colcon build`.
* `motor_control` defines a single node called `motor_control` which subscribes to `control/twist` (type
  `geometry_msgs/msg/Twist`, which is a combination of linear and angular velocity) and publishes motor currents to
  `motor_currents` (type `serial_msgs/msg/MotorCurrents`). In other words, it receives a message which describes how
  the robot should move and selects motor currents which achieve that movement.
* `serial_comms` defines a node called `serial_node` which subscribes to `motor_currents` (type
  `serial_msgs/msg/MotorCurrents`) and sends messages over a serial port to the robot motors. It actually performs the
  action of commanding the motors.
* `serial_msgs` defines messages for controlling the motors (`MotorCurrents`).
* `teleop` defines a node called `teleop` which reads input from a USB-connected game controller and publishes controls
  to `control/twist` (type `geometry_msgs/msg/Twist`).
* `webcam` defines a single node called `webcam_capture` that captures images from the camera and publishes them to
  `awareness/image_raw`. It takes a single parameter (which you can set with `ros2 param set`) which sets the camera
  index.

## Task 1: Write an AprilTag detection node

Create a new package called `apriltag_detection` in the `ament_python` configuration. Write a single node which
subscribes to images from the `awareness/image_raw` topic, performs 2D detection on any AprilTags in the document, and
then publishes the results as an `apriltag_msgs/msg/AprilTagDetections` message on the `awareness/apriltags` topic.

### Hints

1. Remember to add your dependencies to `package.xml` (except for the `apriltag_pose_estimation` library, see below)
   and your main function to `console_scripts` in `setup.py`!
2. To add a dependency on the `apriltag_pose_estimation` library, add the following to the `install_requires` parameter
   in the node's `setup.py`:

       'apriltag_pose_estimation @ git+https://github.com/MARS-UVA/apriltag_pose_estimation.git'

    You'll need to run `py_install_dependencies.py` to actually install it.
3. If Visual Studio Code isn't autocompleting the message fields, then check step 7 from the setup.
4. You shouldn't create a new `AprilTagDetector` every single time you receive a message from `awareness/image_raw`;
   store it in an instance variable instead!
5. The names of the `AprilTagDetection` message and `AprilTagDetection` dataclass conflict. There are many ways to
   resolve this, but the best is probably to give an alias to the `AprilTagDetection` message as follows:
   ```python
   from apriltag_msgs.msg import AprilTagDetection as AprilTagDetectionMsg
   from apriltag_pose_estimation.core import AprilTagDetection  # yay, no conflict!
   ```
6. Note that I'm asking you to send an `apriltag_msgs/msg/AprilTagDetections`, *not* an
   `apriltag_msgs/msg/AprilTagDetection`.
7. Since you are running ROS in a VM, testing this will not be possible without the training robot. Just try your best
   to get something which might work.

## Task 2: Write a node which turns the robot based on AprilTag detections

Create a new package called `turn_to_apriltag` in the `ament_python` configuration. Write a single node which
subscribes to AprilTag detections from the `awareness/apriltags` topic and publishes a twist to `control/twist` that
will cause the robot to turn towards the AprilTag with ID 0 and family `tagStandard41h12`.

### Hints

1. Remember to add your dependencies to `package.xml` and your main function to `console_scripts` in `setup.py`!
2. This node should not depend on the `apriltag_pose_estimation` library.
3. If Visual Studio Code isn't autocompleting the message fields, then check step 7 from the setup.
4. A *twist* is a mathematical object representing both linear and angular velocity in 3D space. For our 2D case, set
   the fields of `geometry_msgs/msg/Twist` as follows:

   * Set `twist.linear.x` to the desired velocity in the forward direction as a value between -1 and 1. Negative values
     indicate backward movement. This is because the X axis is defined on our robot as forward.
   * Set `twist.angular.z` to the desired angular velocity in the counterclockwise direction as a value between -1
     and 1. Negative values indicate clockwise rotation. This is because the Z axis is defined on our robot to point
     out of the floor, and we use a right-handed frame.
   * Keep all other values at their default (0). Linear movement in the Z direction is obviously impossible (it's
     vertical movement), as well as rotation around the X and Y axes. Linear movement in the Y direction would imply
     an ability to strafe (the Y axis points leftward), which our drivetrain cannot do.

5. Since you are running ROS in a VM, testing this will not be possible without the training robot. Just try your best
   to get something which might work.