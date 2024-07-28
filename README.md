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
    joints:
      - alphaimu_sensor
      - alpha_axis_e
      - alpha_axis_d
      - alpha_axis_c
      - alpha_axis_b 
      - alpha_axis_a

    alphaimu_sensor:
      dynamics_identifier: "floating_base"
      publish_pose_interface: ["position.x", "position.y", "position.z","orientation.w","orientation.x","orientation.y","orientation.z"]
      publish_velocity_interface: ["velocity.u","velocity.v","velocity.w","velocity.p","velocity.q","velocity.r"]
      subscribe_pose_interface: ["position.x", "position.y", "position.z","orientation.w","orientation.x","orientation.y","orientation.z"]
      subscribe_velocity_interface: ["velocity.u","velocity.v","velocity.w","velocity.p","velocity.q","velocity.r"]
      effort_command_interface: [alphathruster1_joint/effort, alphathruster2_joint/effort, alphathruster3_joint/effort,
       alphathruster4_joint/effort, alphathruster5_joint/effort, alphathruster6_joint/effort, alphathruster7_joint/effort, alphathruster8_joint/effort]

    alpha_axis_e:
      dynamics_identifier: "joint0"
      publish_pose_interface: ["position"]
      publish_velocity_interface: ["velocity"]
      subscribe_pose_interface: ["filtered_position"]
      subscribe_velocity_interface: ["filtered_velocity"]
      effort_command_interface: [alpha_axis_e/effort]

    alpha_axis_d:
      dynamics_identifier: "joint1"
      publish_pose_interface: ["position"]
      publish_velocity_interface: ["velocity"]
      subscribe_pose_interface: ["filtered_position"]
      subscribe_velocity_interface: ["filtered_velocity"]
      effort_command_interface: [alpha_axis_d/effort]

    alpha_axis_c:
      dynamics_identifier: "joint2"
      publish_pose_interface: ["position"]
      publish_velocity_interface: ["velocity"]
      subscribe_pose_interface: ["filtered_position"]
      subscribe_velocity_interface: ["filtered_velocity"]
      effort_command_interface: [alpha_axis_c/effort]

    alpha_axis_b:
      dynamics_identifier: "joint3"
      publish_pose_interface: ["position"]
      publish_velocity_interface: ["velocity"]
      subscribe_pose_interface: ["filtered_position"]
      subscribe_velocity_interface: ["filtered_velocity"]
      effort_command_interface: [alpha_axis_b/effort]
      
    alpha_axis_a:
      dynamics_identifier: "endeffector"
      publish_pose_interface: ["position"]
      publish_velocity_interface: ["velocity"]
      subscribe_pose_interface: ["filtered_position"]
      subscribe_velocity_interface: ["filtered_velocity"]
      effort_command_interface: [alpha_axis_a/effort]
```

### Build Workspace
```bash
colcon build
```