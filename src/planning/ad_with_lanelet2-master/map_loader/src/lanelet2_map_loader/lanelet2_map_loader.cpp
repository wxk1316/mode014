/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 *
 */

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"  // ROS2 核心头文件

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <lanelet2_extension/projection/mgrs_projector.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/utilities.h>

#include <autoware_lanelet2_msgs/msg/map_bin.hpp>

#include <string>

void printUsage()
{
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("1"),"Usage:");
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("1"),"rosrun map_loader lanelet2_map_loader [.OSM]");
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("1"),
    "rosrun map_loader lanelet2_map_loader download [X] [Y]: WARNING not implemented");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("lanelet_map_loader"); 

  if (argc < 2) {
    printUsage();
    return EXIT_FAILURE;
  }

  std::string mode(argv[1]);
  if (mode == "download" && argc < 4) {
    printUsage();
    return EXIT_FAILURE;
  }

  std::string lanelet2_filename(argv[1]);

  lanelet::ErrorMessages errors;

  lanelet::projection::MGRSProjector projector;
  lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, "autoware_osm_handler", projector, &errors);

  for (const auto & error : errors) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("1"),error);
  }
  if (!errors.empty()) {
    return EXIT_FAILURE;
  }

  lanelet::utils::overwriteLaneletsCenterline(map, false);

  std::string format_version, map_version;
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_filename, &format_version, &map_version);

  rclcpp::Publisher<autoware_lanelet2_msgs::msg::MapBin>::SharedPtr map_bin_pub =
      node->create_publisher<autoware_lanelet2_msgs::msg::MapBin>(
          "output/lanelet2_map",
          rclcpp::QoS(1).transient_local());     
  autoware_lanelet2_msgs::msg::MapBin map_bin_msg;
  auto now = node->get_clock()->now();
  map_bin_msg.header.stamp.sec     = now.seconds();
  map_bin_msg.header.stamp.nanosec = now.nanoseconds() % 1'000'000'000ULL;
  
  map_bin_msg.header.frame_id = "map";
  map_bin_msg.format_version = format_version;
  map_bin_msg.map_version = map_version;
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

  map_bin_pub->publish(map_bin_msg);

  rclcpp::spin(node);     
  rclcpp::shutdown();  

  return 0;
}
