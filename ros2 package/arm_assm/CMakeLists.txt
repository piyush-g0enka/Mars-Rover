cmake_minimum_required(VERSION 3.8)
project(arm_assm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros2_control REQUIRED)

install(DIRECTORY
urdf
meshes
launch
worlds
config
models
DESTINATION share/${PROJECT_NAME}/
)

install(
    DIRECTORY include/arm_assm
    DESTINATION include
    )


install(
    PROGRAMS
    launch/robot_description_publisher.py
    launch/debug.launch.py
    launch/display.launch.py
    launch/gazebo.launch.py
    launch/spawn_robot_ros2.launch.py
    scripts/ee_pos_output.py
    scripts/ik_controller.py
    scripts/inverse_kinematics.py
    scripts/joint_state_listener.py
    scripts/teleop.py
    DESTINATION lib/${PROJECT_NAME}
    )
    
    
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
