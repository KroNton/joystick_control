# joystick_controller Package

The **ROS Joystick Control Package** is a ROS (Robot Operating System) package developed specifically for ROS Noetic. This package provides a convenient way to interface with a joystick input device and use its positional information for controlling robots or other ROS-enabled systems.

## Features

- Read joystick position (X and Y coordinates)
- Support for different joystick types (analog, digital)
- Publish joystick position as ROS messages
- Easy integration with ROS Noetic
- Customizable settings and configurations
- Example launch files and code for reference and testing

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
## License
  The ROS **Joystick Control Package** is open source and is distributed under the [MIT License](https://opensource.org/license/mit/) . You are free to use, modify, and distribute the package in both personal and commercial projects.

