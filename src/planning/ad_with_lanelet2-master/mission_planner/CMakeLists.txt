cmake_minimum_required(VERSION 3.8)
project(mission_planner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)

find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)
find_package(lanelet2_extension REQUIRED)  
find_package(visualization_msgs REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(mission_planner
  lib/mission_planner_base.cpp
  src/mission_planner_lanelet2/mission_planner_lanelet2.cpp
  src/mission_planner_lanelet2/route_handler.cpp
  src/mission_planner_lanelet2/mission_planner_main.cpp
  src/mission_planner_lanelet2/utility_functions.cpp
)


# 安装目标（ament_auto 会自动安装，但可以自定义）
install(TARGETS mission_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  autoware_auto_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  visualization_msgs
)

ament_package()