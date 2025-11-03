cmake_minimum_required(VERSION 3.8)
project(control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bada_msg REQUIRED)
find_package(aura_msg REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Find GeographicLib using pkg-config if CMake's find_package fails
find_package(PkgConfig)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)
else()
  find_package(GeographicLib REQUIRED)
endif()

# Include directories
include_directories(include)
if(DEFINED GEOGRAPHICLIB_INCLUDE_DIRS)
  include_directories(${GEOGRAPHICLIB_INCLUDE_DIRS})
else()
  include_directories(${GeographicLib_INCLUDE_DIRS})
endif()

# Add executable nodes
add_executable(aura_wpt_model_carrot_pid_thrust src/aura_wpt_model_carrot_pid_thrust.cpp)
ament_target_dependencies(aura_wpt_model_carrot_pid_thrust 
  rclcpp 
  std_msgs 
  bada_msg
  aura_msg 
  sensor_msgs 
  tf2
)
if(DEFINED GEOGRAPHICLIB_LIBRARIES)
  target_link_libraries(aura_wpt_model_carrot_pid_thrust ${GEOGRAPHICLIB_LIBRARIES})
else()
  target_link_libraries(aura_wpt_model_carrot_pid_thrust ${GeographicLib_LIBRARIES})
endif()

# Uncomment these when adding more executable nodes
# add_executable(controller_node src/controller_node.cpp)
# ament_target_dependencies(controller_node rclcpp std_msgs bada_msg)

# add_executable(estimator_node src/estimator_node.cpp)
# ament_target_dependencies(estimator_node rclcpp std_msgs bada_msg)

# Install executables
install(TARGETS
  aura_wpt_model_carrot_pid_thrust
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()