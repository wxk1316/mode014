cmake_minimum_required(VERSION 3.8)
project(autoware_vehicle_msgs)


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
find_package(autoware_control_msgs REQUIRED) 
  
  

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlMode.msg"
  "msg/RawControlCommand.msg"
  "msg/RawVehicleCommand.msg"
  "msg/Shift.msg"
  "msg/ShiftStamped.msg"  
  "msg/Steering.msg"
  "msg/TurnSignal.msg"
  "msg/VehicleCommand.msg"          

  
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
    autoware_control_msgs

)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
