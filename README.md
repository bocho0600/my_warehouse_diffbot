## To install relevants ros2_control controllers packages:
```sh
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```

## Source ROS2 Setup file to ENABLE ROS2:
```sh
source /opt/ros/jazzy/setup.bash
```

## Build **diffbot** & **ros2_control_diffbot_description** Packages:
Build everything with:
```sh
colcon build --symlink-install --continue-on-error
``` 
Build my_diffbot package with:
```sh 
build --packages-select my_diffbot
``` 
Build robot_description package with:
```sh 
colcon build --packages-select ros2_control_diffbot_description
``` 

## Source & update environment for packages:
``` sh
source install/setup.bash
```

### This project is referenced from ros2_control example packages: 
* [ros2_control framework information](https://control.ros.org/jazzy/doc/ros2_control/doc/index.html)
* [ros2_control differential drive (``diff_drive_controller``) contorllers](https://control.ros.org/jazzy/doc/ros2_control/doc/index.html)
* [ros2_control differential drive example (example 2)](https://control.ros.org/jazzy/doc/ros2_control_demos/example_2/doc/userdoc.html)
* [ros2_control controllers-chaining example (example 12)](https://control.ros.org/jazzy/doc/ros2_control_demos/example_12/doc/userdoc.html)

* [ros2_control tutorials 1 (Youtube Channel)](https://control.ros.org/jazzy/doc/ros2_control_demos/example_12/doc/userdoc.html)




# DiffBot

**DiffBot**, or "Differential Mobile Robot," is a simple mobile base with differential drive.
The robot is essentially a box moving according to differential drive kinematics.

For **DiffBot**, the hardware interface plugin is implemented with a single interface:
- The communication is done using a proprietary API to communicate with the robot control box.
- Data for all joints is exchanged at once.

The **DiffBot** URDF files can be found in the `description/urdf` folder.

## Running in Docker
Refer to the [run_from_docker](../../doc/run_from_docker.rst) guide.

## Tutorial Steps

### 1. Check DiffBot Description
To check if the **DiffBot** description is working properly, use the following launch command:

```sh
ros2 launch my_diffbot view_robot.launch.py
```

> **Warning:** If you see the message: `Warning: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist`, it is expected. The `joint_state_publisher_gui` node needs some time to start.

![DiffBot](diffbot.png)

### 2. Start the DiffBot Robot
Open a terminal, source your ROS 2 workspace, and execute the launch file:

```sh
ros2 launch my_diffbot diffbot.launch.py
```

This launch file loads and starts the robot hardware, controllers, and opens **RViz**. You should see an orange box in **RViz** if everything started properly.

### 3. Verify Hardware Interface
In another terminal, check if the hardware interface loaded properly:

```sh
ros2 control list_hardware_interfaces
```

Expected output:

```sh
command interfaces
      left_wheel_joint/velocity [available] [claimed]
      right_wheel_joint/velocity [available] [claimed]
state interfaces
      left_wheel_joint/position
      left_wheel_joint/velocity
      right_wheel_joint/position
      right_wheel_joint/velocity
```

### 4. Check Running Controllers
Verify if the controllers are active:

```sh
ros2 control list_controllers
```

Expected output:

```sh
diffbot_base_controller[diff_drive_controller/DiffDriveController] active
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
```

### 5. Send Movement Commands
You can send a velocity command to **DiffBot** using ROS 2 CLI:

```sh
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped "
twist:
  linear:
    x: 0.7
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 1.0"
```

This should make the orange box move in **RViz**.

### 6. Inspect the Hardware Component
To list hardware components:

```sh
ros2 control list_hardware_components
```

Expected output:

```sh
Hardware Component 1
        name: DiffBot
        type: system
        plugin name: my_diffbot/DiffBotSystemHardware
        state: id=3 label=active
        command interfaces
                left_wheel_joint/velocity [available] [claimed]
                right_wheel_joint/velocity [available] [claimed]
```

### 7. Run with Mock Hardware
For testing without a real robot, restart the launch file with:

```sh
ros2 launch my_diffbot diffbot.launch.py use_mock_hardware:=True
```

Check the hardware component again:

```sh
ros2 control list_hardware_components
```

Expected output:

```sh
Hardware Component 1
    name: DiffBot
    type: system
    plugin name: mock_components/GenericSystem
    state: id=3 label=active
    command interfaces
            left_wheel_joint/velocity [available] [claimed]
            right_wheel_joint/velocity [available] [claimed]
```

This confirms that the mock plugin was loaded. You can now test the setup with the same commands.

## Files Used in This Source

- **Launch file:** [`diffbot.launch.py`](https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/bringup/launch/diffbot.launch.py)
- **Controllers YAML:** [`diffbot_controllers.yaml`](https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/bringup/config/diffbot_controllers.yaml)
- **URDF file:** [`diffbot.urdf.xacro`](https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/description/urdf/diffbot.urdf.xacro)
  - **Description:** [`diffbot_description.urdf.xacro`](https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/diffbot/urdf/diffbot_description.urdf.xacro)
  - **ros2_control tag:** [`diffbot.ros2_control.xacro`](https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/description/ros2_control/diffbot.ros2_control.xacro)
- **RViz configuration:** [`diffbot.rviz`](https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/ros2_control_demo_description/diffbot/rviz/diffbot.rviz)
- **Hardware interface plugin:** [`diffbot_system.cpp`](https://github.com/ros-controls/ros2_control_demos/tree/{REPOS_FILE_BRANCH}/example_2/hardware/diffbot_system.cpp)

## Controllers Used in This Source

- **Joint State Broadcaster** ([ros2_controllers repo](https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster))
- **Diff Drive Controller** ([ros2_controllers repo](https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/diff_drive_controller))