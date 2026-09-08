# Spatial 2R forward-kinematics visualization

Start the spatial 2R demo and RViz together. The demo publishes the current arm geometry, end-effector axes, and text labels; RViz loads the saved view. The display has no end-effector trail.

## Build and prepare the terminal

Use an Ubuntu WSL zsh terminal with ROS 2 Jazzy, the repository's development environment, and working GUI support. Start from a fresh terminal that has not sourced this workspace's installed overlay.

```zsh
cd /home/maxwell/Repos/robotics_autonomous/ros_ws
source /opt/ros/jazzy/setup.zsh
source ../.venv/bin/activate

colcon build --packages-select \
  rigid_body_kinematics rigid_body_kinematics_visualization

source install/setup.zsh
```

The build installs both the launch file and RViz configuration. Rebuild after editing either file so the installed copies are updated.

## Launch the animation

Stop any separately running copy of this demo and RViz first, then run:

```zsh
ros2 launch rigid_body_kinematics_visualization \
  spatial_2r_forward_kinematics.launch.xml animate:=true
```

This starts both programs. The arm moves from home to the bent configuration over four seconds, then back over four seconds, repeating. Press `Ctrl+C` in the launch terminal to stop both programs.

## Launch the static pose

```zsh
ros2 launch rigid_body_kinematics_visualization \
  spatial_2r_forward_kinematics.launch.xml animate:=false
```

The static pose has joint angles `(pi/2, -pi/2)` radians, elbow position `(0, 2, 0)` metres, and end-effector position `(0, 2, 1)` metres in `world`. Static mode is the default, so omitting `animate:=false` gives the same result. To switch modes, stop the launch and restart it with the other setting.

## Saved RViz setup

The [launch file](launch/spatial_2r_forward_kinematics.launch.xml) loads [the RViz configuration](rviz/spatial_2r_forward_kinematics.rviz) from the installed package. It selects:

- Fixed frame: `world`.
- Display type: `MarkerArray`.
- Topic: `/spatial_2r_forward_kinematics_demo/markers`.
- Reliable, volatile subscription and the saved camera view.

Both nodes use `use_sim_time=false`; no simulator or `/clock` publisher is needed. The animation phase uses elapsed steady-clock time. RViz illustrates the core's results; it is not a numerical correctness test.

## Check launch discovery without opening RViz

In the prepared terminal, run:

```zsh
ros2 launch rigid_body_kinematics_visualization \
  spatial_2r_forward_kinematics.launch.xml --show-args
```

The output should list `animate` with default `false`. This checks launch-file discovery and parsing, not live rendering. If the package or launch file cannot be found, rebuild and source `install/setup.zsh` in the same terminal.
