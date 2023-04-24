cmake_minimum_required(VERSION 3.8)
project(path_coverage)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find_package(catkin REQUIRED COMPONENTS rospy  std_msgs  geometry_msgs  message_generation)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(message_generation REQUIRED)


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables     --- make sure to chmod +x XYZ.py
install(PROGRAMS scripts/path_coverage_node.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()












## Generate added messages and services with any dependencies listed here
# --- generate_messages(
# --- DEPENDENCIES
# --- std_msgs
# --- geometry_msgs
# ---)

## DEPENDS: system dependencies of this project that dependent projects also need
# --- catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_coverage
# ---  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
# --- )
