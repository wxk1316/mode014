cmake_minimum_required(VERSION 3.8)
project(pathplan)

# 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 如果您有自定义消息 (原文件中被注释掉了，这里保留 ROS 2 的写法以备用)
# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/PointArray.msg"
#   DEPENDENCIES std_msgs geometry_msgs
# )

include_directories(include)

# 2. 编译可执行文件: globalpathplan
add_executable(globalpathplan src/globalpathplan.cpp)

# 设置包含目录
target_include_directories(globalpathplan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 链接 ROS 2 依赖
ament_target_dependencies(globalpathplan
  rclcpp
  std_msgs
  common
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

# 3. 其他被注释掉的节点 (移植为 ROS 2 格式供参考)

# add_executable(localpathplan src/localpathplan.cpp)
# target_include_directories(localpathplan PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# ament_target_dependencies(localpathplan rclcpp std_msgs common nav_msgs)


# add_executable(Interpolation_core src/Interpolation_core.cpp src/Interpolation.cpp)
# target_include_directories(Interpolation_core PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# ament_target_dependencies(Interpolation_core rclcpp std_msgs common)


# 4. 安装规则 (ROS 2 必须)

# 安装可执行文件
install(TARGETS 
  globalpathplan
  # localpathplan 
  # Interpolation_core
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Launch 文件 (如果有)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()