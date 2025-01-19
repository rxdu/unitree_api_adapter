# unitree_api_adapter

This package implements a ROS2 adapter for the Unitree dogs. The adapter is a ros2_control controller that interfaces
with the Gazebo simulator and publishes the robot state and receives control commands via ROS topics.

## Build the package

You need to get the ros message definitions from the unitree_ros2 package.

```bash
cd <path-to-workspace>/src
git clone https://github.com/rxdu/unitree_ros2.git
git clone https://github.com/rxdu/unitree_api_adapter.git
cd ..
colcon build --symlink-install
```

## Use the controller plugin

You can refer to the `config/controller.yaml file` for the configuration of the adapter plugin. The configuration file
should be passed to the `gz_ros2_control-system` plugin in the Gazebo model sdf file.

```xml

<gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
        <parameters>$(find unitree_api_adapter)/config/controller.yaml</parameters>
    </plugin>
</gazebo>
```

If you use this plugin with `unitree_go1_gazebo`, you can specify the controller configuration file in the launch file:

```python
robot_description = (
    xacro.process_file(xacro_file, mappings={
        'DEBUG': 'false',
        'GAZEBO': 'true',
        'CONTROLLER_CONFIG': '$(find unitree_api_adapter)/config/controller.yaml'
    }).toxml())
```

Please make sure the configuration for other controllers is correct if you use this plugin with your own simulation
setup. In such cases, you may consider copying the "controller.yaml" file to your own package and modify the content
accordingly.