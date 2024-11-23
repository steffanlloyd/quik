# QuIK: A faster and more robust inverse kinematics library for ROS2.

Build using the release flag (optimizations help a lot in increasing speed here):
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Running the CPP node:
```bash
ros2 run quik sample_ros_cpp_node --ros-args --params-file ./src/quik/config/ik_service_kuka_kr6.yaml 
```

Running the service:
```bash
ros2 run quik ros_kinematics_service_node --ros-args --params-file ./src/quik/config/ik_service_kuka_kr6.yaml
```

Running the sample client:
```bash
ros2 run quik sample_ros_client_node --ros-args --params-file ./src/quik/config/ik_service_kuka_kr6.yaml
```