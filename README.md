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
