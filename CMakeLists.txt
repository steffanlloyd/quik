cmake_minimum_required(VERSION 3.8)
project(quik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/IKService.srv"
  "srv/FKService.srv"
  "srv/JacobianService.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Add executable
add_executable(sample_cpp_usage
  src/sample_cpp_usage.cpp
  src/geometry.cpp)
ament_target_dependencies(sample_cpp_usage Eigen3)

add_executable(ros_kinematics_service_node
  src/ros_kinematics_service_node.cpp
  src/geometry.cpp
  src/ros_helpers.cpp)
ament_target_dependencies(ros_kinematics_service_node
  rclcpp
  Eigen3
  rosidl_default_runtime
  geometry_msgs
  std_msgs)

add_executable(sample_ros_client_node
  src/sample_ros_client_node.cpp
  src/geometry.cpp
  src/ros_helpers.cpp)
ament_target_dependencies(sample_ros_client_node
  rclcpp
  Eigen3
  rosidl_default_runtime
  geometry_msgs
  std_msgs)

add_executable(sample_ros_cpp_node
    src/sample_ros_cpp_node.cpp
    src/geometry.cpp
    src/ros_helpers.cpp)
ament_target_dependencies(sample_ros_cpp_node
    rclcpp
    Eigen3
    rosidl_default_runtime
    geometry_msgs
    std_msgs)

ament_export_dependencies(rosidl_default_runtime)

# Install library and create export set
install(TARGETS
  sample_cpp_usage
  ros_kinematics_service_node
  sample_ros_client_node
  sample_ros_cpp_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
    DESTINATION lib/${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_export
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Export include directories
ament_export_include_directories(include)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
  
target_link_libraries(ros_kinematics_service_node "${cpp_typesupport_target}")
target_link_libraries(sample_ros_client_node "${cpp_typesupport_target}")
target_link_libraries(sample_ros_cpp_node "${cpp_typesupport_target}")

ament_package()
