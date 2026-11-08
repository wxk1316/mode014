cmake_minimum_required(VERSION 3.8)
project(map_loader)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lanelet2_extension REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(autoware_lanelet2_msgs REQUIRED)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_IO_INCLUDE_DIRS}
)


add_executable(lanelet2_map_loader src/lanelet2_map_loader/lanelet2_map_loader.cpp)
target_link_libraries(lanelet2_map_loader ${catkin_LIBRARIES})
ament_target_dependencies(lanelet2_map_loader
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  visualization_msgs
  lanelet2_core
  lanelet2_extension
)

add_executable(lanelet2_map_visualization src/lanelet2_map_loader/lanelet2_map_visualization.cpp)
target_link_libraries(lanelet2_map_visualization ${catkin_LIBRARIES})
ament_target_dependencies(lanelet2_map_visualization
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  visualization_msgs
  autoware_lanelet2_msgs
  lanelet2_core
  lanelet2_extension
)

## Install executables and/or libraries
install(
  TARGETS
  lanelet2_map_loader
  lanelet2_map_visualization
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY data/
        DESTINATION share/${PROJECT_NAME}/data
        FILES_MATCHING PATTERN "*")

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.xml"
  PATTERN "*.py" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

# 导出依赖
ament_package()
