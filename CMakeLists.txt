cmake_minimum_required(VERSION 3.8)
project(quik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package (Eigen3 REQUIRED NO_MODULE)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

# add_library(${PROJECT_NAME} SHARED
#   src/IKSolver.cpp
#   src/Robot.cpp
# )

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Add executable
add_executable(example src/example.cpp)

# Link libraries
ament_target_dependencies(example Eigen3)

# Install library and create export set
install(TARGETS
    example
    DESTINATION lib/${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_export
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Export include directories
ament_export_include_directories(include)

# Export targets
# ament_export_targets(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)

ament_package()
