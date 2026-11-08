cmake_minimum_required(VERSION 3.8)
project(car_ctr)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(common REQUIRED)
# package.xml 里提到的另外两个包，如果代码里用到了，需要取消注释
# find_package(data_comm REQUIRED) 
# find_package(mqtt_comm REQUIRED)

add_executable(car_ctr_node src/car_ctr.cpp)
set_target_properties(car_ctr_node PROPERTIES OUTPUT_NAME "car_ctr")

ament_target_dependencies(car_ctr_node
  rclcpp
  std_msgs
  tf2
  tf2_ros
  common
  control_msgs
  # data_comm
  # mqtt_comm
)

install(TARGETS car_ctr_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()