# joystick_controller Package

The **ROS Joystick Control Package** is a ROS (Robot Operating System) package developed specifically for ROS Noetic. This package provides a convenient way to interface with a joystick input device and use its positional information for controlling robots or other ROS-enabled systems.

## Features

- Read the joystick position (X and Y coordinates).
- increase & decrease the robot's velocity (linear and angular).
- Support for different joystick types (analog, digital).
- Publish the joystick position as ROS messages.
- Easy integration with ROS Noetic.
- Customizable settings and configurations.


## Nodes
1. **joy_node**  [official page](http://wiki.ros.org/joy)
   
2. **joy_robot_control**
   
   2.1 Subscribed Topics:
   
      - /joy (sensor_msgs/Joy)
   
   2.2 Published Topics:

      - /cmd_vel (geometry_msgs/Twist)
        
   2.3 Parameters:
   
      - ~cmd_vel_topic (default: "/cmd_vel")
      - ~linear_direction_index (default: "1")
      - ~angular_direction_index (default: "0")
      - ~linear_speed_increase_index (default: "0")
      - ~linear_speed_decrease_index (default: "1")
      - ~angular_speed_increase_index (default: "3")
      - ~angular_speed_decrease_index (default: "2")

## Installation

To use the **ROS Joystick Control Package**, follow these steps:

1. Create a ROS workspace (if you don't have one already):

   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/
   catkin_make
   
2. Clone the repository into your ROS workspace's `src` directory:
   
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/KroNton/joystick_control.git
   
3. Install any necessary dependencies using ROS package manager:
   
   ```bash
   cd ~/catkin_ws/
   rosdep install --from-paths src --ignore-src -r -y
   
4. Build the ROS package:
   
    ```bash
    cd ~/catkin_ws/
    catkin_make
## Usage
**Launching Joystick Node**

To launch the joystick node and start reading joystick data, use the following command:
   ```bash
   roslaunch joystick_control joystick_control.launch
   ```
   
This will start the joystick node, which reads the joystick position and publishes it as Twist message.

![Joystick Robot Controller](https://github.com/KroNton/joystick_control/blob/main/images/joystick_control.png)

   ![simulation Robot Control](https://github.com/KroNton/joystick_control/blob/main/images/control_robot.gif)

## License
  The  **Joystick Control Package** is open source and is distributed under the [MIT License](https://opensource.org/license/mit/). You are free to use, modify, and distribute the package in both personal and commercial projects.
  
## Support
If you encounter any issues or have questions regarding the **Joystick Control Package**, please open an issue on the GitHub repository. The project maintainers will do their best to assist you.


