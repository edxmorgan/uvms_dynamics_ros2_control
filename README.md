# Underwater Vehicle and Manipulator System

Ros2_control plugin providing virtual underwater vehicle and manipulator capabilities.

## Library Details

- **Pluginlib-Library**: uvms
- **Plugin Type**: uvms/Dynamics (implements `controller_interface::ControllerInterface`)

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

### Add to your robot controllers yaml config file in ros2_control

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
      - alpha_axis_a
      - alpha_axis_b
      - alpha_axis_c
      - alpha_axis_d
      - alpha_axis_e

    alphaimu_sensor:
      publish_pose_interface: ["position.x", "position.y", "position.z","orientation.w","orientation.x","orientation.y","orientation.z"]
      publish_velocity_interface: ["velocity.u","velocity.v","velocity.w","velocity.p","velocity.q","velocity.r"]
      subscribe_pose_interface: ["position.x", "position.y", "position.z","orientation.w","orientation.x","orientation.y","orientation.z"]
      subscribe_velocity_interface: ["velocity.u","velocity.v","velocity.w","velocity.p","velocity.q","velocity.r"]
      effort_interface: effort
       
    alpha_axis_a:
      publish_pose_interface: ["position"]
      publish_velocity_interface: ["velocity"]
      subscribe_pose_interface: ["filtered_position"]
      subscribe_velocity_interface: ["filtered_velocity"]
      effort_interface: "effort"

    alpha_axis_b:
      publish_pose_interface: ["position"]
      publish_velocity_interface: ["velocity"]
      subscribe_pose_interface: ["filtered_position"]
      subscribe_velocity_interface: ["filtered_velocity"]
      effort_interface: "effort"

    alpha_axis_c:
      publish_pose_interface: ["position"]
      publish_velocity_interface: ["velocity"]
      subscribe_pose_interface: ["filtered_position"]
      subscribe_velocity_interface: ["filtered_velocity"]
      effort_interface: "effort"

    alpha_axis_d:
      publish_pose_interface: ["position"]
      publish_velocity_interface: ["velocity"]
      subscribe_pose_interface: ["filtered_position"]
      subscribe_velocity_interface: ["filtered_velocity"]
      effort_interface: "effort"

    alpha_axis_e:
      publish_pose_interface: ["position"]
      publish_velocity_interface: ["velocity"]
      subscribe_pose_interface: ["filtered_position"]
      subscribe_velocity_interface: ["filtered_velocity"]
      effort_interface: "effort"
```

### Build Workspace
```bash
colcon build
```