# DynamicGoal

This repository contains ROS packages for dynamic goal handling, including plugins, utilities, and test cases for robust navigation in robot systems. It is structured into three primary packages: dyn_goal, and two modifications to voxel_grid and costmap_2d packages.

## Dependencies Packages Overview
1. costmap_2d

This package provides the core components to work with 2D costmaps, which are essential for navigation in ROS-based systems. It includes various plugins, configuration files, and test cases.
Key Changes to original costmap_2d:

    It does not add costmap information from the sensors if dynamic goal is active around the frame to follow.

2. voxel_grid

The voxel_grid package provides an efficient 3D voxel representation for obstacle and environment mapping, commonly used in navigation systems.
Changes were made to make this code compatible with dyn_goal.

## Package description

### dyn_goal

This package provides the logic and infrastructure to handle dynamic goals in a robot navigation system. It is split into ROS-dependent and ROS-independent components.
Key Directories and Files:

    ros/src/dyn_goal_ros/: ROS-dependent scripts, including dyn_goal_node.py and topic_to_tf.py.
    msg/: Message definition dyn_goal_msg.msg for sending dynamic goals between nodes.
    launch/: Launch files for bringing up the dyn_goal node and other configurations.
    scripts/: Executable scripts such as dyn_goal_node and topic_to_tf_node for testing and running the dynamic goal system.

ROS Nodes:

    dyn_goal_node: Node that processes dynamic goal information.
    teleop_tf_test.py: A utility script to test teleoperation and transformation logic.

## Installation


1. Clone the repository into your ROS workspace:

```
cd ~/catkin_ws/src
git clone <repository_url>
```

2. Build the workspace:

```
cd ~/catkin_ws
catkin build
```

3. Source the workspace:

```
source devel/setup.bash
```

## Usage


You can launch the dyn_goal node with:

```
roslaunch dyn_goal dyn_goal.launch
```

To test the method you can teleoperate the tf by running:

```
python3 ros/src/dyn_goal_ros/teleop_tf_test.py
```

## Acknowledgements

This repository incorporates code from the [ros-planning/navigation](https://github.com/ros-planning/navigation) repository for the costmap_2d and voxel_grid packages. We acknowledge and appreciate the contributions from the ROS community.

## License

This repository is licensed under the MIT License. See the LICENSE file for more details.