## To install relevants ros2_control controllers packages:
``sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers``

## Source ROS2 Setup file to ENABLE ROS2:
``source /opt/ros/jazzy/setup.bash`` 

## Build **diffbot** & **ros2_control_diffbot_description** Packages:
``colcon build --symlink-install --continue-on-error`` 
``colcon build --packages-select my_diffbot`` 
``colcon build --packages-select ros2_control_diffbot_description`` 

## Source & update environment for packages:
``source install/setup.bash`` 

***This project is referenced from ros2_control example packages: ***
### * [ros2_control framework information](https://control.ros.org/jazzy/doc/ros2_control/doc/index.html)
### * [ros2_control differential drive (``diff_drive_controller``) contorllers](https://control.ros.org/jazzy/doc/ros2_control/doc/index.html)
### * [ros2_control differential drive example (example 2)](https://control.ros.org/jazzy/doc/ros2_control_demos/example_2/doc/userdoc.html)
### * [ros2_control controllers-chaining example (example 12)](https://control.ros.org/jazzy/doc/ros2_control_demos/example_12/doc/userdoc.html)