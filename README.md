# Underwater Vehicle and Manipulator System

Ros2_control plugin providing virtual underwater vehicle and manipulator capabilities.

## Library Details

- **Pluginlib-Library**: uvms_controller
- **Plugin Type**: uvms_controller/UvmsController (implements `controller_interface::ControllerInterface`)

## Usage


### Clone the repository into the `src` folder of your ROS2 workspace:

```bash
cd ~/your_workspace/src
git clone https://github.com/edxmorgan/uvms_ros2_controller
```
### Add parameter library generation to project
**package.xml**
```xml
<exec_depend>uvms_controller</exec_depend>
```

### Edit accordingly and add to your robot controllers yaml config file in ros2_control

```yaml
controller_manager:
  ros__parameters:
    update_rate: 150  #Hz

    uvms_controller:
      type: uvms_controller/UvmsController

uvms_controller:
  ros__parameters:
    base_TF_translation: [0.140, 0.000, -0.120]
    base_TF_rotation: [3.142, 0.000, 0.000]
    joints:
      - alpha_IOs
      - alpha_axis_e
      - alpha_axis_d
      - alpha_axis_c
      - alpha_axis_b 
      - alpha_axis_a

    alpha_IOs:
      dynamics_identifier: "floating_base"
      pose_topic_interface: ["position.x", "position.y", "position.z","position.qw","position.qx","position.qy","position.qz"]
      velocity_topic_interface: ["velocity.x","velocity.y","velocity.z","velocity.roll","velocity.pitch","velocity.yaw"]
      pose_command_interface: ["position.x", "position.y", "position.z","position.qw","position.qx","position.qy","position.qz"]
      velocity_command_interface: ["velocity.x","velocity.y","velocity.z","velocity.roll","velocity.pitch","velocity.yaw"]
      effort_command_interface: [effort.x, effort.y, effort.z, effort.roll, effort.pitch, effort.yaw]

    alpha_axis_e:
      dynamics_identifier: "joint0"
      pose_topic_interface: ["filtered_position"]
      velocity_topic_interface: ["filtered_velocity"]
      pose_command_interface: ["position"]
      velocity_command_interface: ["velocity"]
      effort_command_interface: [effort]

    alpha_axis_d:
      dynamics_identifier: "joint1"
      pose_topic_interface: ["filtered_position"]
      velocity_topic_interface: ["filtered_velocity"]
      pose_command_interface: ["position"]
      velocity_command_interface: ["velocity"]
      effort_command_interface: [effort]

    alpha_axis_c:
      dynamics_identifier: "joint2"
      pose_topic_interface: ["filtered_position"]
      velocity_topic_interface: ["filtered_velocity"]
      pose_command_interface: ["position"]
      velocity_command_interface: ["velocity"]
      effort_command_interface: [effort]

    alpha_axis_b:
      dynamics_identifier: "joint3"
      pose_topic_interface: ["filtered_position"]
      velocity_topic_interface: ["filtered_velocity"]
      pose_command_interface: ["position"]
      velocity_command_interface: ["velocity"]
      effort_command_interface: [effort]
      
    alpha_axis_a:
      dynamics_identifier: "endeffector"
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