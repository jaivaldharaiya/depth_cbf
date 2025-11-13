# TurtleBot3 CBF Controller - Integrated Package

This package contains a complete implementation of Control Barrier Function (CBF) based safe navigation for TurtleBot3 robots using ROS2. The package integrates advanced CBF controllers, depth processing, and point cloud analysis in a self-contained ROS2 package.

## Features

- **Advanced CBF Implementation**: Full Control Barrier Function optimization using OSQP
- **Depth Processing**: Point cloud processing and spatial transformation utilities
- **Multiple Controllers**: Choice between feedback linearization, simplified CBF, and advanced CBF
- **Trajectory Generation**: Support for waypoints, circular, and figure-eight trajectories
- **LiDAR Integration**: Real-time obstacle detection and avoidance
- **Self-Contained**: All dependencies included in the package

## Package Structure

```
turtle_pkg/
├── __init__.py
├── main.py                    # Main controller node
├── controller.py              # Controller implementations
├── cbf_controllers.py         # Advanced CBF controllers
├── depth_proc.py              # Depth processing utilities
├── pointcloud.py              # Point cloud data structures
├── lidar.py                   # LiDAR sensor interface
├── state_estimation.py        # State observer
├── trajectory.py              # Basic trajectory class
├── trajectory_generators.py   # Advanced trajectory generators
└── data/                      # Experimental data
```

## Dependencies

### Required ROS2 Packages
- `rclpy`
- `geometry_msgs`
- `sensor_msgs`
- `std_msgs`
- `tf2_ros`
- `tf2_geometry_msgs`

### Required Python Packages
- `numpy`
- `scipy`
- `osqp` (for advanced CBF optimization)

### Optional Dependencies
- `matplotlib` (for visualization)
- `python3-tk` (for GUI components)

## Installation

### Method 1: Using the Deploy Script

1. Clone the repository:
```bash
cd ~/
git clone https://github.com/jaivaldharaiya/depth_cbf.git
cd depth_cbf/ROS2Deploy
```

2. Run the deployment script:
```bash
chmod +x deploy.sh
./deploy.sh [optional_workspace_path]
```

### Method 2: Manual Installation

1. Create a ROS2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone the package:
```bash
git clone https://github.com/jaivaldharaiya/depth_cbf.git
cp -r depth_cbf/ROS2Deploy turtle_pkg
```

3. Install dependencies:
```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

4. Install Python dependencies:
```bash
pip3 install osqp
```

5. Build the package:
```bash
colcon build --packages-select turtle_pkg
source install/setup.bash
```

## Usage

### Basic Usage

1. Set up TurtleBot3 environment:
```bash
export TURTLEBOT3_MODEL=burger
source ~/ros2_ws/install/setup.bash
```

2. Launch the controller:
```bash
ros2 launch turtle_pkg turtlebot_cbf.launch.py
```

### Simulation Usage

For simulation testing:
```bash
ros2 launch turtle_pkg simulation.launch.py
```

### Controller Types

The package supports three controller types (configurable in `main.py`):

1. **Feedback Linearization**: Basic trajectory tracking
2. **Simplified CBF**: CBF-based safety with fallback logic  
3. **Advanced CBF**: Full CBF optimization with OSQP

To switch between controllers, modify the flags in `main.py`:
```python
useCBFQP = True        # Enable CBF controllers
useAdvancedCBF = True  # Use advanced CBF with optimization
```

### Configuration Parameters

Key parameters can be adjusted in the controller initialization:

```python
# CBF Parameters
DELTA = 0.3        # Safety buffer distance (meters)
alpha0 = 1.0       # CBF constraint parameter
alpha1 = 1.0       # CBF derivative parameter

# Control Frequency
frequency = 300    # Hz
```

## Advanced Features

### Custom Trajectories

Use the trajectory generators for complex motion:

```python
from turtle_pkg.trajectory_generators import create_trajectory

# Waypoint trajectory
traj = create_trajectory('waypoint', 
                        waypoints=[[0,0,0], [2,1,0], [3,3,0]], 
                        total_time=10)

# Circular trajectory  
traj = create_trajectory('circular', 
                        center=[0,0], 
                        radius=2, 
                        angular_velocity=0.5)

# Figure-eight trajectory
traj = create_trajectory('figure_eight', 
                        center=[0,0], 
                        scale=2, 
                        period=20)
```

### CBF Customization

Modify the CBF function for different safety behaviors:

```python
def custom_cbf(self, q, qC):
    """Custom barrier function."""
    # Elliptical safety region
    A = np.diag([1/0.5**2, 1/0.3**2, 0])
    return (q - qC).T @ A @ (q - qC) - 1
```

## Data Collection and Analysis

The package automatically saves timing and performance data:

- `data/timingtest.npy`: Control loop frequencies
- `data/cbfList*.npy`: CBF values during execution
- `data/freqList*.npy`: Frequency analysis data

Load and analyze the data:
```python
import numpy as np
timing_data = np.load('data/timingtest.npy')
print(f"Average frequency: {np.mean(timing_data):.2f} Hz")
```

## Troubleshooting

### Common Issues

1. **OSQP not found**: Install with `pip3 install osqp`
2. **CBF optimization fails**: Check that point cloud data is available
3. **Low control frequency**: Reduce grid size in `get_cbf_quad_fit_3D`
4. **Robot stops unexpectedly**: Check DELTA parameter and obstacle distances

### Debug Mode

Enable detailed logging by modifying the node logger level:
```python
node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

### Performance Monitoring

Monitor real-time performance:
```bash
ros2 topic echo /sys_data  # CBF values and control inputs
ros2 topic echo /cmd_vel   # Control commands
```

## Contributing

This package integrates research from multiple sources:

1. Original CBF implementation from `qrotor_sim_examples`
2. Depth processing algorithms from `depth_processing`
3. ROS2 integration and TurtleBot adaptations

When contributing:
- Maintain backward compatibility
- Add appropriate unit tests
- Document any new parameters
- Follow ROS2 coding standards

## License

MIT License - see LICENSE file for details.

## Citation

If you use this package in your research, please cite:

```
@misc{depth_cbf_turtlebot,
  title={Point Cloud-Based Control Barrier Function Regression for Safe and Efficient Vision-Based Control on TurtleBot3},
  author={Your Name},
  year={2025},
  url={https://github.com/jaivaldharaiya/depth_cbf}
}
```