# Gazebo ROS2 demo project
This is a simple functional demo project using Gazebo and ROS2. All commands must be executed from the project root.
## Build
```
colcon build
```
## Run
On the Docker instance:
```
source install/setup.bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$(pwd)/install/collision_ros2/lib/collision_ros2
GAZEBOIP=<server_ip> GAZEBO_MASTER_URI=<server_ip>:<port> ros2 launch gazebo_ros gzserver.launch.py world:="install/collision_ros2/share/collision_ros2/world/model_push.world"
```
## Visualize
On a computer with a display server:
```
GAZEBOIP=<client_ip> GAZEBO_MASTER_URI=<server_ip>:<port> ros2 launch gazebo_ros gzclient.launch.py
```
## Start moving one of the cubes
On the docker instance:
```
source install/setup.bash
ros2 topic pub --once /subscriber std_msgs/msg/String "{data: '1'}"
```

