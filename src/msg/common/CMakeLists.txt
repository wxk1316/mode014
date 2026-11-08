cmake_minimum_required(VERSION 3.8)
project(common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_library(public src/public.cpp)
target_include_directories(public PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(public rclcpp std_msgs geometry_msgs)

add_library(can src/can.cpp)
target_include_directories(can PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(can rclcpp std_msgs geometry_msgs)

ament_export_dependencies(rclcpp std_msgs geometry_msgs)
ament_export_include_directories(include)
ament_export_libraries(public can)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS public can
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()