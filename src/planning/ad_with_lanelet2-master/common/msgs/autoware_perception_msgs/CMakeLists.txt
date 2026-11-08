cmake_minimum_required(VERSION 3.8)
project(autoware_perception_msgs)

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
find_package(sensor_msgs REQUIRED)  
find_package(unique_identifier_msgs REQUIRED)  

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/object_recognition/DynamicObject.msg"
  "msg/object_recognition/DynamicObjectArray.msg"
  "msg/object_recognition/Shape.msg"
  "msg/object_recognition/Semantic.msg"
  "msg/object_recognition/State.msg"
  "msg/object_recognition/Feature.msg"
  "msg/object_recognition/PredictedPath.msg"
  "msg/object_recognition/DynamicObjectWithFeature.msg"
  "msg/object_recognition/DynamicObjectWithFeatureArray.msg"

  "msg/traffic_light_recognition/LampState.msg"
  "msg/traffic_light_recognition/TrafficLightRoi.msg"
  "msg/traffic_light_recognition/TrafficLightRoiArray.msg"
  "msg/traffic_light_recognition/TrafficLightState.msg"
  "msg/traffic_light_recognition/TrafficLightStateArray.msg"

  
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
    sensor_msgs
    unique_identifier_msgs

)

# 安装规则
install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/
)

# 导出
ament_export_dependencies(
  rosidl_default_runtime
  builtin_interfaces
  std_msgs
  geometry_msgs
  sensor_msgs
  unique_identifier_msgs
)

ament_package()