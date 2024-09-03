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

```

### Build Workspace
```bash
colcon build
```

### Send command
```bash
ros2 topic pub /uvms_controller/uvms/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [{label: 'row', size: 10, stride: 1}, {label: 'column', size: 1, stride: 10}], data_offset: 0}, data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.31]}" --once
```