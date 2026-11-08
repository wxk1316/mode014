cmake_minimum_required(VERSION 3.8)
project(lanelet2_extension)

# 默认使用 C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_maps REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)
find_package(lanelet2_validation REQUIRED)
find_package(autoware_lanelet2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# 查找其他依赖
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

find_library(PUGIXML_LIBRARIES
  NAMES pugixml
)

find_path(PUGIXML_INCLUDE_DIRS
  NAMES pugixml.hpp
  PATH_SUFFIXES pugixml
)

# 添加编译选项
add_compile_options(-Wall -Wextra -Wpedantic)

# 包含目录
include_directories(
  include
  ${GeographicLib_INCLUDE_DIRS}
  ${PUGIXML_INCLUDE_DIRS}
)

# 查找所有头文件
file(GLOB_RECURSE headers include/*.hpp include/*.h)

# 添加库
add_library(lanelet2_extension_lib
  lib/autoware_osm_parser.cpp
  lib/autoware_traffic_light.cpp
  lib/detection_area.cpp
  lib/message_conversion.cpp
  lib/mgrs_projector.cpp
  lib/query.cpp
  lib/utilities.cpp
  lib/visualization.cpp
)

# 包含 ROS2 消息生成的头文件目录
target_include_directories(lanelet2_extension_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GeographicLib_INCLUDE_DIRS}
  ${PUGIXML_INCLUDE_DIRS}
  ${autoware_lanelet2_msgs_INCLUDE_DIRS}
)

# 添加可执行文件示例
add_executable(lanelet2_extension_sample src/sample_code.cpp)
target_link_libraries(lanelet2_extension_sample
  lanelet2_extension_lib
  rclcpp::rclcpp
)

# 添加验证可执行文件
add_executable(autoware_lanelet2_validation src/validation.cpp)
target_link_libraries(autoware_lanelet2_validation
  lanelet2_extension_lib
  ${PUGIXML_LIBRARIES}
  rclcpp::rclcpp
)

ament_target_dependencies(lanelet2_extension_lib
  rclcpp
  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  autoware_lanelet2_msgs
  geometry_msgs
  visualization_msgs
  tf2_geometry_msgs
  tf2
)


# 安装规则
install(TARGETS
  lanelet2_extension_lib
  lanelet2_extension_sample
  autoware_lanelet2_validation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
  PATTERN "*.hpp"
  PATTERN "*.h"
)

# 如果有测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_gtest REQUIRED)
  
  # 添加测试
  ament_add_gtest(message_conversion_test
    test/src/test_message_conversion.cpp
  )
  target_link_libraries(message_conversion_test
    lanelet2_extension_lib
    rclcpp::rclcpp
  )
  
  ament_add_gtest(projector_test
    test/src/test_projector.cpp
  )
  target_link_libraries(projector_test
    lanelet2_extension_lib
    rclcpp::rclcpp
  )
  
  ament_add_gtest(query_test
    test/src/test_query.cpp
  )
  target_link_libraries(query_test
    lanelet2_extension_lib
    rclcpp::rclcpp
  )
  
  ament_add_gtest(regulatory_elements_test
    test/src/test_regulatory_elements.cpp
  )
  target_link_libraries(regulatory_elements_test
    lanelet2_extension_lib
    rclcpp::rclcpp
  )
  
  ament_add_gtest(utilities_test
    test/src/test_utilities.cpp
  )
  target_link_libraries(utilities_test
    lanelet2_extension_lib
    rclcpp::rclcpp
  )
endif()

# 导出依赖
ament_export_dependencies(
  rclcpp
  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  autoware_lanelet2_msgs
  geometry_msgs
  visualization_msgs
)

ament_export_include_directories(include)
ament_export_libraries(lanelet2_extension_lib)

ament_package()