# ModalAI Starling 2 Max description and simulation

This ROS2 Humble package contains a URDF description for the ModalAI Starling2 Max drone. This description allows for visualization using Rviz2 and Gazebo.

## Build instructions
1. Clone into the `src` folder of your ROS2 workspace
2. Use `colcon build` to build package
3. Source the installed package by using `<workspace directory>/source/install/setup.bash`


## Launching Rviz
To launch RViz, use `ros2 launch starling2_description rviz.launch.py`

## Launching Gazebo simulation
To launch Gazebo simulation, use `ros2 launch starling2_description gazebo.launch.py`

To list all available Gazebo topics (these are *not* ROS2 topics), use
```
ign topic -l
```

To send velocity commands to Gazebo, open a new terminal and use
```
ign topic -t "/starling2/gazebo/command/twist" -m ignition.msgs.Twist -p "linear: {x:<x velocity> y: <velocity> z: <z velocity>} angular {z: <yaw velocity>}"
```
e.g to go straight up
```
ign topic -t "/starling2/gazebo/command/twist" -m ignition.msgs.Twist -p "linear: {x:0 y: 0 z: 0.1} angular {z: 0}"
```
To hover in place:

```
ign topic -t "/starling2/gazebo/command/twist" -m ignition.msgs.Twist -p "linear: {x:0 y: 0 z: 0.0} angular {z: 0}"
```

To descend:

```
ign topic -t "/starling2/gazebo/command/twist" -m ignition.msgs.Twist -p "linear: {x:0 y: 0 z: -0.1} angular {z: 0}"
```


## TODOs
1. Add sensor simulation for Gazebo
2. Add new worlds for Gazebo
3. Enable ROS-Gazebo bridge to enable cross-communication
4. Enable PX4 plugin for Gazebo

# Notes:
1. The physics (MoI, rotor thrust physics, etc. )described in the URDF **are not** 1:1 with the real drone. These values were taken from the X500 drone SDF in the [PX4-gazebo-models repo](https://github.com/PX4/PX4-gazebo-models). It looks like most other simulations available in the internet just use the same values as the ones provided here.
2. Despite being continous joints, the rotors are modelled as revolute joints with extremely large joint limits. This is intentional since Gazebo's default physics engine doesn't have support for continous joints.
3. Collision model has been simplified and will not 100 percent correlate with real world. However it should be good enough for most use-cases
4. Camera resolutions and FPS might not line up exactly with real-world
5. The Solidworks generated joints are slightly wonky and some joints look slightly off. How bad this affects things is yet to be seen. So far theres no issues though

## Troubleshooting
If GPS does not work, try replacing 
```
<plugin name="ignition::gazebo::systems::NavSat" filename="ignition-gazebo-navsat-system"/>
```
with
```
<plugin name="gz::sim::systems::NavSat" filename="gz-sim-navsat-system"/>
```
in the `.sdf` files in the `worlds/` direcrtory. There's some naming confusion with [Gazebo/Ignition](https://gazebosim.org/about) so you might be out of luck depending on the package version installed on your device



