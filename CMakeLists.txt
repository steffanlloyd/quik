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
  DEPENDENCIES geometry_msgs std_msgs
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Add executable
add_executable(example src/example.cpp)
ament_target_dependencies(example Eigen3)

add_executable(ik_service_node src/ik_service_node.cpp)
ament_target_dependencies(ik_service_node
  rclcpp
  Eigen3
  rosidl_default_runtime
  geometry_msgs
  std_msgs)

# add_dependencies(ik_service_node ${PROJECT_NAME}__rosidl_typesupport_cpp)
ament_export_dependencies(rosidl_default_runtime)


# Install library and create export set
install(TARGETS
  example
  ik_service_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
    DESTINATION lib/${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_export
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Export include directories
ament_export_include_directories(include)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
  
target_link_libraries(ik_service_node "${cpp_typesupport_target}")


# Export targets
# ament_export_targets(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)

ament_package()
