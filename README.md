## Quick Start

### Step 1: Copy the Package to Your ROS 2 Workspace

```bash
cp -r ./vector_field_pkg ~/ros2_ws/src/
```

### Step 2: Build package

```bash
cd ~/ros2_ws
colcon build --packages-select vector_field_pkg
source install/setup.bash
```

### Step 3: Run controller

```bash
ros2 run vector_field_pkg controller
```



## Gazebo

Whenever you make changes to gazebo, rebuild the package and source install.sh. 



## Installing TurtleBot

Install turtlebot: 

sudo apt install ros-humble-turtlebot3*

Add to your .bashrc for terminal session:

export TURTLEBOT3_MODEL=burger


## Launching Turtlebot

Launch this (make sure "which python" is not pointing to the anaconda python)

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py x_pose:=1.17 y_pose:=-0.89

ros2 launch turtlebot3_navigation2 navigation2.launch.py   use_sim_time:=true   map:=/opt/ros/humble/share/turtlebot3_navigation2/map/map_house.yaml

## LTL Planning Code
This code is mean to make it easy to see the code added for the project, but the repo needed to run it is located here: https://github.com/cameronschloer/ts
