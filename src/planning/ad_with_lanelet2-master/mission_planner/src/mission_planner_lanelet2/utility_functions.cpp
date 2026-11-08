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
 */

#include <lanelet2_core/geometry/Lanelet.h>
#include <mission_planner/lanelet2_impl/utility_functions.h>
#include <rclcpp/rclcpp.hpp>

bool exists(const std::unordered_set<lanelet::Id> & set, const lanelet::Id & id)
{
  return set.find(id) != set.end();
}

std::string toString(const geometry_msgs::msg::Pose & pose)
{
  std::stringstream ss;
  ss << "(" << pose.position.x << ", " << pose.position.y << "," << pose.position.z << ")";
  return ss.str();
}

void setColor(std_msgs::msg::ColorRGBA * cl, double r, double g, double b, double a)
{
  cl->r = r;
  cl->g = g;
  cl->b = b;
  cl->a = a;
}

void insertMarkerArray(
  visualization_msgs::msg::MarkerArray * a1, const visualization_msgs::msg::MarkerArray & a2)
{
  a1->markers.insert(a1->markers.end(), a2.markers.begin(), a2.markers.end());
}

bool getClosestLanelet(
  const geometry_msgs::msg::Pose & search_pose, const lanelet::LaneletMapPtr & lanelet_map_ptr_,
  lanelet::Lanelet * closest_lanelet, double distance_thresh)
{
  lanelet::BasicPoint2d search_point(search_pose.position.x, search_pose.position.y);
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelet =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 1);
  if (nearest_lanelet.empty()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("1"),
      "Failed to find the closest lane!" << std::endl
                                         << "search point: " << toString(search_pose) << std::endl);
    return false;
  }
  if (nearest_lanelet.front().first > distance_thresh) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("1"),
      "Closest lane is too far away!" << std::endl
                                      << "search point: " << toString(search_pose) << std::endl
                                      << "lane id: " << nearest_lanelet.front().second.id());
    return false;
  }

  *closest_lanelet = nearest_lanelet.front().second;

  return true;
}
