# Underwater Vehicle and Manipulator System

Ros2_control plugin providing virtual underwater vehicle and manipulator capabilities.

## Library Details

- **Pluginlib-Library**: uvms_controller
- **Plugin Type**: uvms_controller/UvmsController (implements `controller_interface::ControllerInterface`)

## Usage


### Clone the repository into the `src` folder of your ROS2 workspace:

```bash
cd ~/your_workspace/src
git clone https://github.com/edxmorgan/uvms_dynamics_ros2_control
```
### Add parameter library generation to project
**package.xml**
```xml
<exec_depend>uvms_controller</exec_depend>
```

### Edit accordingly and add to your robot controllers yaml config file in ros2_control

```yaml
  
uvms_controller:
  ros__parameters:
    agents:
      - alpha

    joints:
      - floating_base
      - joint0
      - joint1
      - joint2
      - joint3 
      - endeffector

    alpha:
      name_space: alpha
      base_TF_translation: [0.190, 0.000, -0.120]
      base_TF_rotation: [3.142, 0.000, 0.000]

    floating_base:
      name : "{namespace}_IOs"
      pose_topic_interface: ["position.x", "position.y", "position.z","position.qw","position.qx","position.qy","position.qz"]
      velocity_topic_interface: ["velocity.x","velocity.y","velocity.z","velocity.roll","velocity.pitch","velocity.yaw"]
      pose_command_interface: ["position.x", "position.y", "position.z","position.qw","position.qx","position.qy","position.qz"]
      velocity_command_interface: ["velocity.x","velocity.y","velocity.z","velocity.roll","velocity.pitch","velocity.yaw"]
      effort_command_interface: [effort.x, effort.y, effort.z, effort.roll, effort.pitch, effort.yaw]

    joint0:
      name : "{namespace}_axis_e"
      pose_topic_interface: ["filtered_position"]
      velocity_topic_interface: ["filtered_velocity"]
      pose_command_interface: ["position"]
      velocity_command_interface: ["velocity"]
      effort_command_interface: [effort]

    joint1:
      name : "{namespace}_axis_d"
      pose_topic_interface: ["filtered_position"]
      velocity_topic_interface: ["filtered_velocity"]
      pose_command_interface: ["position"]
      velocity_command_interface: ["velocity"]
      effort_command_interface: [effort]

    joint2:
      name : "{namespace}_axis_c"
      pose_topic_interface: ["filtered_position"]
      velocity_topic_interface: ["filtered_velocity"]
      pose_command_interface: ["position"]
      velocity_command_interface: ["velocity"]
      effort_command_interface: [effort]

    joint3:
      name : "{namespace}_axis_b"
      pose_topic_interface: ["filtered_position"]
      velocity_topic_interface: ["filtered_velocity"]
      pose_command_interface: ["position"]
      velocity_command_interface: ["velocity"]
      effort_command_interface: [effort]
      
    endeffector:
      name : "{namespace}_axis_a"
      pose_topic_interface: ["filtered_position"]
      velocity_topic_interface: ["filtered_velocity"]
      pose_command_interface: ["position"]
      velocity_command_interface: ["velocity"]
      effort_command_interface: [effort]

```

### Build Workspace
```bash
colcon build
```

### Send command
```bash
ros2 topic pub /uvms_controller/uvms/commands uvms_interfaces/msg/Command "{command_type: velocity ,input:{data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}" --once
```

