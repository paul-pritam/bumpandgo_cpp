# FSM Bump and Go

A ROS 2 package implementing a Finite State Machine (FSM) for reactive obstacle avoidance and navigation. The robot explores its environment by moving forward, detecting obstacles via laser scan, and autonomously deciding between backing up, turning, or stopping based on its current state and sensor feedback.

## Overview

The `fsm_bumpgo_cpp` package implements a simple yet effective four-state finite state machine:

1. **FORWARD**: Move toward goal
2. **BACK**: Retreat from obstacle
3. **TURN**: Rotate to find free space
4. **STOP**: Pause and reassess

The robot uses laser scan data to make state transitions, enabling reactive navigation without requiring pre-planned paths. A frontier analysis algorithm intelligently selects turning direction toward the most open space.

## Features

- **Finite State Machine**: Four-state architecture for reactive control
- **Obstacle Detection**: Real-time collision avoidance using laser scan data
- **Frontier Analysis**: Intelligent turning direction based on available space
- **Configurable Parameters**: Adjustable speeds, distances, and timing
- **50ms Control Cycle**: Responsive behavior at 20 Hz update rate
- **Simple and Lightweight**: Minimal computational overhead
- **ROS 2 Native**: Built with modern ROS 2 architecture

## Dependencies

- **ROS 2**: Core middleware
- **rclcpp**: ROS 2 C++ client library
- **sensor_msgs**: Laser scan data types
- **geometry_msgs**: Velocity command messages
- **std_msgs**: Standard message types

## Architecture

### Main Component

#### BumpGoNode

A single ROS 2 node implementing the FSM with the following responsibilities:

- Subscribes to laser scan data
- Processes sensor input for obstacle detection
- Maintains state machine logic
- Publishes velocity commands
- Runs control loop at 50ms intervals

### State Machine Diagram

```
┌─────────────────────────────────────────────────┐
│                                                 │
│  ┌────────────┐     Obstacle Detected          │
│  │ FORWARD    │──────────────────────┐          │
│  │ (Go ahead) │                      │          │
│  └─────┬──────┘                      v          │
│        │                       ┌──────────┐     │
│        │ Scan Timeout          │ STOP     │     │
│        │                       │ (Assess) │     │
│        └──────────────────────>└──────────┘     │
│             (if no timeout)          │          │
│                                      │ 1 sec    │
│        ┌──────────────────────────────┘          │
│        │                                         │
│        v                                         │
│    ┌────────────┐                               │
│    │ BACK       │                               │
│    │ (Retreat)  │                               │
│    └─────┬──────┘                               │
│          │ 0.5 sec                              │
│          │                                       │
│          v                                       │
│    ┌──────────────┐                             │
│    │ TURN         │                             │
│    │ (Reorient)   │                             │
│    └─────┬────────┘                             │
│          │ 0.5 sec                              │
│          │                                       │
│          └──────────────┬───────────────────────┘
│                         │
│                         v
│ ┌─────────────────────────────────────────────┐
└─>│ Back to FORWARD                             │
   └─────────────────────────────────────────────┘
```

## Building

From the workspace root:

```bash
colcon build --packages-select fsm_bumpgo_cpp
```

## Running

### Launch the Package

```bash
ros2 launch fsm_bumpgo_cpp bump_and_go.launch.py
```

### Manual Node Execution

```bash
ros2 run fsm_bumpgo_cpp bumpgo_node_exe
```

### Complete Example with Simulation

```bash
# Terminal 1: Start simulation
ros2 launch mybot launch_sim.launch.py

# Terminal 2: Run the FSM node
ros2 launch fsm_bumpgo_cpp bump_and_go.launch.py

# Terminal 3: Monitor velocity commands
ros2 topic echo /output_vel

# Terminal 4: Visualize in RViz (optional)
ros2 launch mybot rsp.launch.py
```

## Topics

### Subscribed Topics

- `input_scan` (`sensor_msgs/LaserScan`): Laser scan data from robot's LIDAR

### Published Topics

- `output_vel` (`geometry_msgs/Twist`): Velocity commands for robot motion
  - `linear.x`: Forward/backward velocity (-0.3 to +0.3 m/s)
  - `angular.z`: Rotational velocity (±0.2 rad/s)

## Configuration

### State Timing Parameters

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| `OBSTACLE_DISTANCE` | 0.5 | meters | Distance threshold for obstacle detection |
| `SCAN_TIMEOUT` | 0.5 | seconds | Time before stopping if no scan received |
| `BACKING_TIME` | 0.5 | seconds | Duration to reverse before turning |
| `TURNING_TIME` | 0.5 | seconds | Duration to rotate to new orientation |
| `STOP_TIME` | 1.0 | seconds | Duration to pause when obstacle appears |

### Speed Parameters

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| `FORWARD_SPEED` | 0.3 | m/s | Forward movement velocity |
| `BACKWARD_SPEED` | -0.3 | m/s | Backward movement velocity |
| `TURN_SPEED` | 0.2 | rad/s | Rotational velocity when turning |

### Control Parameters

- **Control Cycle**: 50ms (20 Hz update rate)
- **Front Obstacle Window**: 5% of laser ranges on each side of center (~30° FOV)
- **Frontier Analysis**: Compares left vs right free space to choose turning direction

## State Transitions

### FORWARD → STOP
- **Condition**: No laser scan received for more than `SCAN_TIMEOUT` (0.5s)
- **Action**: Pause and wait for sensor data

### FORWARD → BACK
- **Condition**: Obstacle detected in front (range < 0.5m)
- **Action**: Start backing away

### STOP → FORWARD
- **Condition**: `STOP_TIME` (1.0s) has elapsed
- **Action**: Resume forward motion

### BACK → TURN
- **Condition**: `BACKING_TIME` (0.5s) has elapsed
- **Action**: Begin turning toward the most open direction

### TURN → FORWARD
- **Condition**: `TURNING_TIME` (0.5s) has elapsed
- **Action**: Resume forward motion in new direction

## Frontier Analysis Algorithm

When entering the TURN state, the robot analyzes available space:

```cpp
// Compare total free space on each side
right_dist_sum = sum(ranges[0] to ranges[n/2])
left_dist_sum = sum(ranges[n/2] to ranges[n])

// Turn toward the side with greater free space
if (left_dist_sum > right_dist_sum)
    turn_dir_multiplier = 1.0   // Turn left
else
    turn_dir_multiplier = -1.0  // Turn right
```

This ensures the robot naturally navigates toward open space when backing away from obstacles.

## Usage Examples

### Basic Autonomous Navigation

```bash
# Robot will explore environment avoiding obstacles
ros2 launch fsm_bumpgo_cpp bump_and_go.launch.py
```

### Monitor State Transitions

```bash
# View state changes in logs
ros2 launch fsm_bumpgo_cpp bump_and_go.launch.py --ros-args --log-level INFO
```

### Test with Different Obstacle Distances

Edit the `OBSTACLE_DISTANCE` parameter in [bumpgonode.hpp](include/fsm_bumpgo_cpp/bumpgonode.hpp):

```cpp
static constexpr float OBSTACLE_DISTANCE = 1.0;  // Detect farther obstacles
```

## Integration with Other Packages

This package works well with:

- **vff_avoidance**: Replace bump-and-go with vector field forces
- **tf2_obstacle_detector**: Use TF2 obstacle frames for enhanced detection
- **cv_object_tracker**: Add visual target tracking alongside obstacle avoidance
- **Navigation2**: Integrate as a fallback behavior

## Troubleshooting

### Robot Not Moving

- Verify laser scan is publishing: `ros2 topic echo /input_scan`
- Check if velocity commands are being received by robot controller
- Ensure robot has power and motor drivers are enabled

### Robot Stuck in Loop

- Increase `OBSTACLE_DISTANCE` for earlier detection
- Adjust `BACKING_TIME` and `TURNING_TIME` for more aggressive maneuvering
- Verify laser scan quality (check for NaN or Inf values)

### Unpredictable Turning Direction

- Check frontier analysis calculation
- Verify laser scan ranges are valid on both sides
- Inspect `turn_dir_multiplier_` value in logs

### Slow Response

- Decrease control cycle time (edit timer in constructor)
- Reduce `STOP_TIME` for faster resumption
- Verify no heavy processing is blocking the control loop

## Testing

### Manual Testing

1. Launch simulation with obstacles
2. Run the node
3. Observe robot behavior
4. Monitor state transitions via logs

### Automated Testing (if implemented)

```bash
colcon test --packages-select fsm_bumpgo_cpp
```

## License

Apache License 2.0 - See LICENSE file for details

## Maintainer

- **Email**: pritampaulwork7@gmail.com
- **Author**: ubuntu
