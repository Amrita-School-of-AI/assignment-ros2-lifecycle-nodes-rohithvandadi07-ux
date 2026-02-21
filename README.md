[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/nvEbKyTZ)
# Assignment: ROS2 Lifecycle Nodes

**Course:** ROS2 Advanced  
**Build System:** ament_cmake

## Objective

The goal of this assignment is to verify your understanding of:

1. How to create ROS2 Lifecycle (Managed) Nodes.
2. Understanding the lifecycle states and transitions.
3. Implementing lifecycle callbacks.

## Background

Lifecycle nodes provide a standard interface for managing the state of a node. The states are:

- **Unconfigured**: Initial state after creation
- **Inactive**: Configured but not processing
- **Active**: Fully operational
- **Finalized**: Shutdown state

## Problem Statement

You must complete the provided skeleton code to create a lifecycle-managed sensor node.

### Requirements

1. **Source Code (`src/lifecycle_sensor.cpp`)**:
   - Implement a class `LifecycleSensor` that inherits from `rclcpp_lifecycle::LifecycleNode`.
   - Initialize the node with the name `"lifecycle_sensor"`.
   - Implement the following callbacks:
     - `on_configure`: Initialize the publisher, log "Sensor configured"
     - `on_activate`: Start the timer, log "Sensor activated"
     - `on_deactivate`: Stop the timer, log "Sensor deactivated"
     - `on_cleanup`: Reset publisher, log "Sensor cleaned up"
     - `on_shutdown`: Log "Sensor shutting down"
   - Create a publisher to `/sensor_data` with type `std_msgs::msg::Float64`.
   - Timer should publish random sensor values (0.0 to 100.0) every 500ms when active.

2. **Build Configuration (`CMakeLists.txt`)**:
   - Add executable and link `rclcpp_lifecycle`.

3. **Package Metadata (`package.xml`)**:
   - Add required dependencies.

## How to Test Locally

```bash
colcon build --packages-select ros2_lifecycle
source install/setup.bash

# Terminal 1: Run the lifecycle node
ros2 run ros2_lifecycle lifecycle_sensor

# Terminal 2: Manage the lifecycle
ros2 lifecycle set /lifecycle_sensor configure
ros2 lifecycle set /lifecycle_sensor activate
ros2 topic echo /sensor_data  # Should see data now
ros2 lifecycle set /lifecycle_sensor deactivate
ros2 lifecycle set /lifecycle_sensor cleanup
```

#### Expected Behavior:

```shell
# After configure:
[lifecycle_sensor]: Sensor configured

# After activate:
[lifecycle_sensor]: Sensor activated
[lifecycle_sensor]: Publishing sensor data: 45.23
...

# After deactivate:
[lifecycle_sensor]: Sensor deactivated
# (No more publishing)
```
