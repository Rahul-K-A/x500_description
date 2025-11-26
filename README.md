# Iris X500 description and simulation

This ROS2 Humble package contains a URDF description for the x500 drone. This description allows for visualization using Rviz2

## Build instructions
1. Clone into the `src` folder of your ROS2 workspace
2. Use `colcon build` to build package
3. Source the installed package by using `<workspace directory>/source/install/setup.bash`


## Launching Rviz
To launch RViz, use `ros2 launch x500_description rviz.launch.py`

## Notices and disclosure.
Based off of original source code from [here](https://github.com/PX4/PX4-gazebo-models/). Check LICENSE file for more details. All CAD models are properties of original creators. This repository contains code that was modified using Generative Artifical Intelligence and as such may contain errors/inaccuracies.
