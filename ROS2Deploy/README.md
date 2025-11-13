# TurtleBot3 CBF Controller - ROS 2 Package

This ROS 2 package implements a Control Barrier Function (CBF) based controller for safe navigation of TurtleBot3 robots using point cloud-based obstacle avoidance.

## Package Structure

```
turtle_pkg/
├── package.xml              # ROS 2 package manifest
├── setup.py                 # Python package setup
├── setup.cfg                # Python package configuration
├── resource/                # Resource marker files
├── turtle_pkg/              # Python package
│   ├── __init__.py
│   ├── main.py              # Main controller node
│   ├── controller.py        # Control algorithms
│   ├── state_estimation.py  # State observers
│   ├── lidar.py            # LiDAR interface
│   ├── pointcloud.py       # Point cloud processing
│   └── trajectory.py       # Trajectory generation
├── launch/                  # Launch files
│   ├── turtlebot_cbf.launch.py      # Main controller launch
│   └── simulation.launch.py         # Simulation launch
├── data/                    # Data files
└── README.md               # This file
```

## Dependencies

### ROS 2 Packages
```bash
sudo apt install ros-humble-tf-transformations
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-turtlebot3-*
```

### Python Packages
```bash
pip3 install numpy scipy
```

## Installation

1. **Clone the repository:**
   ```bash
   cd ~/ros2_ws/src
   git clone <your-repo-url>
   ```

2. **Build the package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select turtle_pkg
   source install/setup.bash
   ```

3. **Install dependencies:**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

## Usage

### Running on Real TurtleBot3

1. **Set up TurtleBot3 environment:**
   ```bash
   export TURTLEBOT3_MODEL=burger  # or waffle, waffle_pi
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

2. **Launch the controller:**
   ```bash
   ros2 launch turtle_pkg turtlebot_cbf.launch.py
   ```

3. **Or run the node directly:**
   ```bash
   ros2 run turtle_pkg turtlebot_controller
   ```

### Running in Simulation

1. **Launch Gazebo simulation with controller:**
   ```bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtle_pkg simulation.launch.py
   ```

## Configuration

### Controller Parameters

The controller can be configured by modifying parameters in the launch files:

- `use_sim_time`: Use simulation time (default: false for real robot, true for simulation)
- `robot_namespace`: Namespace for the robot (default: empty)

### Trajectory Settings

Modify the trajectory in `main.py`:
```python
start_position = np.array([[0, 0, 0]]).T
end_position = np.array([[3, 0, 0]]).T
time_duration = 1
```

## Topics

### Subscribed Topics
- `/scan` (sensor_msgs/LaserScan): LiDAR data
- `/odom` → `/tf` transforms: Robot odometry

### Published Topics
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/sys_data` (geometry_msgs/Twist): System data for monitoring

## Safety Features

- **Control Barrier Functions**: Ensures safe navigation around obstacles
- **Emergency stop**: Automatic stopping when obstacles are too close
- **Point cloud-based avoidance**: Real-time obstacle detection using LiDAR

## Troubleshooting

### Common Issues

1. **Transform errors**: Ensure TurtleBot3 is properly configured and running
   ```bash
   ros2 topic list | grep tf
   ros2 topic echo /tf --qos-reliability reliable --qos-durability transient_local
   ```

2. **No LiDAR data**: Check if scan topic is published
   ```bash
   ros2 topic echo /scan
   ```

3. **Import errors**: Ensure all dependencies are installed
   ```bash
   pip3 list | grep -E "(numpy|scipy)"
   ```

### Debug Mode

Run with verbose logging:
```bash
ros2 run turtle_pkg turtlebot_controller --ros-args --log-level DEBUG
```

## Performance Monitoring

The controller saves timing data to `data/timingtest.npy` for performance analysis. Load with:
```python
import numpy as np
timing_data = np.load('data/timingtest.npy')
print(f"Average frequency: {np.mean(timing_data):.2f} Hz")
```

## Contributing

1. Follow ROS 2 coding standards
2. Test thoroughly in simulation before real robot deployment
3. Update documentation for any parameter changes

## License

MIT License - See package.xml for details

## Support

For issues and questions, please use the GitHub issue tracker.