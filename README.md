# hex_ros_ik_saber_750d

## What does this package do

This package implements a kinematics solver for the **Saber 750D** 6-DOF robotic arm, supporting both **ROS 1** and **ROS 2**. It subscribes to a target end-effector pose and publishes the solved joint states in real time using an SE(3) interpolation strategy.

## Maintainer

[Dong Zhaorui](https://github.com/IBNBlank)

## Prerequisites

Ensure the following software and hardware are installed:

* **ROS**:  
   Refer to the [ROS Installation guide](http://wiki.ros.org/ROS/Installation)

### Verified Platforms

* [x] **x64**
* [ ] **Jetson Orin Nano**
* [x] **Jetson Orin NX**
* [ ] **Jetson AGX Orin**
* [ ] **Horizon RDK X5**
* [ ] **Rockchip RK3588**

## Public APIs

### Published Topics

| Topic           | Msg Type                         | Description                                          |
| --------------- | -------------------------------- | ---------------------------------------------------- |
| `/joint_states` | `sensor_msgs/(msg/)JointState`   | Solved joint positions, velocities, and efforts.     |
| `/debug_pose`   | `geometry_msgs/(msg/)PoseStamped`| Debug end-effector pose from FK after IK solve.      |
| `/ik_success`   | `std_msgs/(msg/)Bool`            | Whether the last IK solve was successful.            |

### Subscribed Topics

| Topic          | Msg Type                         | Description                      |
| -------------- | -------------------------------- | -------------------------------- |
| `/target_pose` | `geometry_msgs/(msg/)PoseStamped`| Target end-effector pose for IK. |

### Parameters

| Name                 | Data Type        | Description                              |
| -------------------- | ---------------- | ---------------------------------------- |
| `rate_ros`           | `float`          | Main loop rate (Hz).                     |
| `rate_state`         | `float`          | Joint state publish rate (Hz).           |
| `prog_debug`         | `bool`           | Enable debug pose publishing.            |
| `model_path`         | `string`         | Path to the URDF model file.             |
| `model_base`         | `string`         | Base link name.                          |
| `model_joint_names`  | `vector<string>` | Ordered joint names.                     |
| `model_end_effector` | `string`         | End-effector link name.                  |
| `model_end_quat`     | `vector<float>`  | End-effector orientation offset (wxyz).  |
| `limit_pos`          | `vector<string>` | Joint position limits [[min, max], …].   |
| `limit_vel`          | `vector<string>` | Joint velocity limits [[min, max], …].   |
| `limit_acc`          | `vector<string>` | Joint acceleration limits.              |
| `limit_joint_err`    | `float`          | Joint-space error tolerance.             |
| `limit_se3_err`      | `float`          | SE(3) error tolerance.                   |

## Getting Started

Follow these steps to set up the project for development and testing on your local machine:

1. Create a workspace `catkin_ws` and navigate to the `src` directory:

   ```shell
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   ```

2. Clone the repository:

   ```shell
   git clone https://github.com/hexfellow/hex_ros_ik_saber_750d.git
   ```

3. Navigate back to the `catkin_ws` directory and build the workspace:

   For ROS 1:

   ```shell
   cd ../
   catkin_make
   ```

   For ROS 2:

   ```shell
   cd ../
   colcon build
   ```

4. Source the `setup.bash` file:

   For ROS 1:

   ```shell
   source devel/setup.bash --extend
   ```

   For ROS 2:

   ```shell
   source install/setup.bash --extend
   ```

### Usage

1. Launch the `ik_saber_750d` node:

   For ROS 1:

   ```shell
   roslaunch hex_ros_ik_saber_750d ik_saber_750d.launch
   ```

   For ROS 2:

   ```shell
   ros2 launch hex_ros_ik_saber_750d ik_saber_750d.launch.py
   ```

2. Publish a target pose to `/target_pose` (`geometry_msgs/PoseStamped`).
3. View the solved joint states on `/joint_states` (`sensor_msgs/JointState`) and the IK success status on `/ik_success` (`std_msgs/Bool`).

### Test Tools

A test script is provided under `urdf/test_tool.py` for verifying kinematics offline without ROS:

```shell
cd urdf
python3 test_tool.py
```
