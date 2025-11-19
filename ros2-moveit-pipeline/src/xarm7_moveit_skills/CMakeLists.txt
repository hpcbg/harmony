cmake_minimum_required(VERSION 3.8)
project(xarm7_moveit_skills)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)


# Add move_to_pose executable
add_executable(move_to_pose src/move_to_pose.cpp)
target_include_directories(move_to_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_to_pose PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  move_to_pose
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

# Add move_to_pose_with_rviz executable
add_executable(move_to_pose_with_rviz src/move_to_pose_with_rviz.cpp)
target_include_directories(move_to_pose_with_rviz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_to_pose_with_rviz PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  move_to_pose_with_rviz
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

# Add move_to_pose_with_ca executable
add_executable(move_to_pose_with_ca src/move_to_pose_with_ca.cpp)
target_include_directories(move_to_pose_with_ca PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_to_pose_with_ca PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  move_to_pose_with_ca
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

# Add move_to_pose_with_ca_isaac executable
add_executable(move_to_pose_with_ca_isaac src/move_to_pose_with_ca_isaac.cpp)
target_include_directories(move_to_pose_with_ca_isaac PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_to_pose_with_ca_isaac PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  move_to_pose_with_ca_isaac
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

# Add open_gripper executable
add_executable(open_gripper src/open_gripper.cpp)
target_include_directories(open_gripper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(open_gripper PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  open_gripper
  "moveit_ros_planning_interface"
  "moveit_task_constructor_core"
  "rclcpp"
)

# Add close_gripper executable
add_executable(close_gripper src/close_gripper.cpp)
target_include_directories(close_gripper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(close_gripper PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  close_gripper
  "moveit_ros_planning_interface"
  "moveit_task_constructor_core"
  "rclcpp"
)

# Add pick executable
add_executable(pick src/pick.cpp)
target_include_directories(pick PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pick PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pick
  "moveit_ros_planning_interface"
  "moveit_task_constructor_core"
  "rclcpp"
)


install(TARGETS move_to_pose move_to_pose_with_rviz 
  move_to_pose_with_ca move_to_pose_with_ca_isaac open_gripper
  close_gripper pick
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
