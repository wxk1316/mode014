cmake_minimum_required(VERSION 3.8)
project(autoware_planning_msgs)

# 默认使用 C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 查找 ROS2 包
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)  
  

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneSequence.msg"
  "msg/Path.msg"
  "msg/PathPoint.msg"
  "msg/PathWithLaneId.msg"
  "msg/PathPointWithLaneId.msg"
  "msg/Route.msg"
  "msg/RouteSection.msg"
  "msg/Scenario.msg"
  "msg/Trajectory.msg"
  "msg/TrajectoryPoint.msg"

  
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
    nav_msgs

)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
