# Instructions for Launching the Navigation Stack

## Dependencies

```bash
sudo apt update

sudo apt install \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-topic-tools
```

## Launching slam-toolbox mapping

Terminal 1 (launch the simulator):
```bash
cd ~/go2_sim_ws
source install/setup.bash
ros2 launch go2_config gazebo_velodyne.launch.py rviz:=true
```

Terminal 2 (launch slam with settings from the config):
```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$HOME/go2_sim_ws/src/unitree-go2-ros2/robots/configs/go2_config/config/autonomy/slam_toolbox.yaml use_sim_time:=true
```

Optional: Terminal 3 (launch teleoperation):
```bash
cd ~/go2_sim_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Launching NAV2 goal pose

After launching slam, change `Fixed Frame` in `Global Options` to `map`!

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=$HOME/go2_sim_ws/src/unitree-go2-ros2/robots/configs/go2_config/config/autonomy/nav2_params.yaml
```

Set `2D Goal Pose` points so that the robot moves to the desired position.