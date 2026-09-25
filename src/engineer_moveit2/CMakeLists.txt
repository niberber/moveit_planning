cmake_minimum_required(VERSION 3.22)
project(engineer_moveit2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

set(srv_files "srv/ComputeIK.srv")
rosidl_generate_interfaces(${PROJECT_NAME} ${srv_files}
  DEPENDENCIES moveit_msgs
)

add_executable(plan_to_csv src/plan_to_csv.cpp)
ament_target_dependencies(plan_to_csv
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  tf2_geometry_msgs
)
target_include_directories(plan_to_csv PRIVATE /usr/include)
target_link_libraries(plan_to_csv serial)

add_executable(play_trajectory src/play_trajectory.cpp)
ament_target_dependencies(play_trajectory
  rclcpp
  sensor_msgs
)

add_executable(ik_server src/ik_server.cpp)
ament_target_dependencies(ik_server
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
)
target_include_directories(ik_server PRIVATE /usr/include)
target_link_libraries(ik_server serial)
rosidl_target_interfaces(ik_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(ik_client src/ik_client.cpp)
ament_target_dependencies(ik_client
  rclcpp
  moveit_msgs
)
rosidl_target_interfaces(ik_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(moveit_planning src/moveit_planning.cpp src/feedback_pub.cpp)
ament_target_dependencies(moveit_planning
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  sensor_msgs
)

target_include_directories(moveit_planning PRIVATE /usr/include)
target_link_libraries(moveit_planning serial)

add_executable(test_constrained src/test_constrained.cpp)
ament_target_dependencies(test_constrained
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
  trajectory_msgs
  sensor_msgs
)

add_executable(my_fk src/my_fk.cpp)

ament_target_dependencies(my_fk
  rclcpp
  moveit_core
  moveit_ros_planning_interface
)

add_executable(independent_collision_check src/independent_collision_check.cpp)

target_include_directories(independent_collision_check PRIVATE /usr/include)

ament_target_dependencies(independent_collision_check
  rclcpp
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

target_link_libraries(independent_collision_check fcl)

install(TARGETS plan_to_csv play_trajectory ik_server ik_client moveit_planning test_constrained my_fk independent_collision_check DESTINATION lib/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
