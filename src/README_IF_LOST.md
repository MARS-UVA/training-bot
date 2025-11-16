# Start here

If you make changes, update this file accordingly.

People who actually read the whole thing actually accomplish things.

Last updated 11/16/2025 by Dylan Stone (vha3kn)


# Maintaing convention

### C++
C++ packages should be as follows:

```
<packageName>
├── CMakeLists.txt
├── package.xml
├── resource
│   └── <fileName>
└── src
    └── <fileName>.cpp
```

### Python
```
<package_name>
├── <package_name>
│   └── <fileName>.py
├── package.xml
├── resource
│   └── <fileName>
├── setup.cfg
└── setup.py
```

**Note**: It is also acceptable to use a `scripts/` folder to contain .py files instead of the package name a second time.

# What does it all mean?

### There are a few ROS packages in the src/ folder 

### bot_launch:

Contains the file `bot.launch.py`
This is a python launch folder that launches both other packages and other launch files. 

    - Other launch files are launched using IncludeLaunchDescription
      I promise it has to be like this. Do not try to modify this if you don't know what you are doing.

### serial_comms:

This package handles serial messages. 

Essentially do not touch unless you need to modify how you are publishing messages to the microcontroller.

### serial_msgs:

This includes the definition for the message type `motor_currents`, used by the training bots

    The message should be a current for each half of the training bot, not a velocity.

    The value should be between 0 and 253. 254 does not work at the moment (idk why either) and 255 indicates the start of a message to prevent misalignment.


### teleop

Teleop contains one file, `motor_command_reader`. 

The file `motor_command_reader` subscribes to the topic `\cmd_vel` which publihes a `TwistStamped` message.
 
From there, it converts the `TwistStamped` type into a `motor_currents` type, and returns it.

### bot_launch/rviz2

This isn't actually a ros2 package. It simply is a file that tells rviz2 to put topics on the screen so we do not have to manually add them each time we launch. 

If you would like to modify it, the easiest way is to save the rviz config (should save to ~/.rviz2) with what you want and use that file. Make sure to keep the name as `nav2_default_view.rviz`

Don't want to do that? Good luck doing it another way.

### Training_Bot

This was written for the Arduino (notice how the naming conventions are off).
This is what gets deployed to the Arduino. 

May or may not get deployed every time, idk ask someone who knows Arduino.

# Modifying to the launch file. 

### Adding a singular file from a package:

This is the piece of code that launches `motor_command_reader`
```
Node(
    package='teleop',
    executable='motor_command_reader',
    name='motor_command_reader',
    output='screen',
    namespace='bot'
),
```
In this example, 

    the ROS package is teleop

    the name of the file (without extension for c++, with extension for python) is motor_command_reader

    the name of the node is motor_command_reader

    the output will print to the terminal

    the namespace is bot/

For name and namespace, it may seem reduntant to have both, but they actually are both very helpful.

If, for example, you ran `ros2 node list`, you would see (among the other nodes) 

>
     `/bot/motor_command_reader`


The namespace we defined, `/bot`, allows us to put `motor_command_reader` into a larger directory for easier searching and readability

Keep this consistent. I will find you.

The "exact" meanings of the arguments are as follows:

```
Node(
    package='name_ros_package',     #the top level file name of the package
    executable='file_name',         #what file in the package to launch
    name='name_of_node',            #tells ros what to name the node
    output='screen',                #prints to terminal
    namespace='top_layer_of_node'   #tells ros to put the node under a larger node's name
),
```

**Note**: Other parameters do exist. These are just the strictly necessary ones.


### Adding an entire other launch file:

Less out of necessity and more so out of laziness, we decided to simply launch entire launch files rather than tracking down each individual file that nav2 requires. 

This piece of code is what launches the slam toolbox for nav2:

```
IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('slam_toolbox'),
            'launch',
            'online_async_launch.py',
        ]),
```

In this example, the original command to launch the slam toolbox was 

> 
    ros2 launch slam_toolbox online_async_launch.py

From here, it should be obvious how to plug in a new launch file, but in case it is not I will explain in more detail:

```
IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('package_name_of_launch_file'),
            'launch', #if you are launching another launch file do not change this
            'online_async_launch.py', #the name of the other launch file
        ]),
```


### What if I want to do too much and add fancy arguments and/or other stuff?

#### Adding arguments (to launch file):
From the slam toolbox launch line:
```
launch_arguments={
    'resolution': '0.0005',
}.items()
```
This simply adds the argument `resolution=0.0005` to the end on the launch command

 `launch_arguments={}` is a dict that contains the argument (key) and the value. You can add arguments like this:

```
launch_arguments={
    'resolution': '0.0005',
    'argument_1` : `something`,
    `argument_2` : `thingie`
    `key` : `value`
}.items()
```

Keep the `.items()` as it is needed for the launch file. (Source: trust me bro)

#### Adding a delay:

```
rviz_config_file = os.path.join(
    get_package_share_directory('bot_launch'), 
    'rviz2',
    'nav2_default_view.rviz')

rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_file],
    output='screen'
)

def generate_launch_description():
    return LaunchDescription([
        TimerAction(
            period=5.0,      # Delay in seconds
            actions=[rviz_node]
        )
    ])

```

In this example, we are launching rviz2 with a delay of 5 seconds to ensure the topics exist at launchtime. 

The `TimerAction` should go inside the `LaunchDescription`, but the node should be defined outside `generate_launch_description`. Simply use this same format for your node.


### What if I want to use a `package.xml`?

Don't.

It isn't possible (as far as I know) to do some of the more advanced things that the flexibility of a python based launch file.

And take a shower.

# CMake and/or General Errors with Building?

Since the `CMakeLists.txt` file is heavily dependant for each package, I cannot list every possible way to fix things here.

However, here is a few tips to save a few headaches:

### Python in your package?

If the code is purely python, there is no `CMakeLists.txt` required. Instead, you need a setup.py and a setup.cfg.

Additionally, the folder containing your files should also contain an `__init__.py`, but does not need any code in it.
In other words, make sure there is a blank `__init__.py` so python is happy.

If you are using both c++ and python, you can use `ament_cmake_python` to manage all you files from a single `CMakeLists.txt`.

### Include errors? 

If it is just VScode highlighting `import` or `#include` lines, it might work regardless of what VScode things. If, however, it throws an error when running or building, it is definitely an error in your `CMakeLists.txt`. 

### Miscellaneous errors?

C++ packages require a resource/<nameOfYourPackageHere> file. They don't need an extension or any content.

Did you try deleting the `build` `install` and `log` folders before rebuidling? AKA a clean build.