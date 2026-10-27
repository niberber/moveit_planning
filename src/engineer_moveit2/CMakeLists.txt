cmake_minimum_required(VERSION 3.22)
project(engineer_moveit2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

# 视觉：PnP 解算 + 海康相机（不是 ament 包，只能用 add_subdirectory）
# 注意：不要 add_subdirectory(../vision)，那是独立 project，会额外建 PnP 可执行目标
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../vision/PnPDate   ${CMAKE_CURRENT_BINARY_DIR}/PnPDate)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../vision/PnPSolver ${CMAKE_CURRENT_BINARY_DIR}/PnPSolver)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../vision/task4/lib ${CMAKE_CURRENT_BINARY_DIR}/HKCamera)

add_executable(moveit_planning src/moveit_planning.cpp src/feedback_pub.cpp)
ament_target_dependencies(moveit_planning
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  sensor_msgs
  ament_index_cpp
)

target_link_libraries(moveit_planning PnPSolver HKCamera)

install(TARGETS moveit_planning DESTINATION lib/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
