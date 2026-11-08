#ifndef COMMON_MYDISPLAY_H
#define COMMON_MYDISPLAY_H

// ROS 2 includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>

// TF2 includes (Replacements for <tf/transform_listener.h>)
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>

// C++ includes
#include <vector>
#include <string>
#include <cmath>

using namespace std;

// ROS 2 Helper: Convert logic requires passing the TF Buffer
bool transformPoint(std::shared_ptr<tf2_ros::Buffer> tf_buffer, string target_frame, geometry_msgs::msg::PointStamped &src_p, geometry_msgs::msg::PointStamped &dst_p, string tag="")
{
    if(target_frame == src_p.header.frame_id) 
    {
        dst_p = src_p;
        return true;
    }

    bool res = false;
    try
    {
        // ROS 2 uses canTransform instead of waitForTransform usually, or wait on buffer
        if (tf_buffer->canTransform(target_frame, src_p.header.frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.3)))
        {
            dst_p = tf_buffer->transform(src_p, target_frame);
            res = true;
        }
    }
    catch (const std::exception &e)
    {
        res = false;
        if(tag != "") 
        {
            // Logging needs a logger, using stdout for header simplicity or rely on caller
            printf("tag=%s %s\n", tag.c_str(), e.what());
        }
    }

    return res;
}

bool transformPose(std::shared_ptr<tf2_ros::Buffer> tf_buffer, string target_frame, geometry_msgs::msg::PoseStamped src_p, geometry_msgs::msg::PoseStamped &dst_p, string tag="")
{
    if(target_frame == src_p.header.frame_id) 
    {
        dst_p = src_p;
        return true;
    }

    bool res = false;
    try
    {
        if (tf_buffer->canTransform(target_frame, src_p.header.frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.3)))
        {
            dst_p = tf_buffer->transform(src_p, target_frame);
            res = true;
        }
    }
    catch (const std::exception &e)
    {
        res = false;
        if(tag != "") printf("tag=%s %s\n", tag.c_str(), e.what());
    }

    return res;
}

float GetYawFromPose(geometry_msgs::msg::PoseStamped pose)
{
    double roll, pitch, yaw;
    tf2::Quaternion q;
    tf2::fromMsg(pose.pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

geometry_msgs::msg::PoseStamped GetExtendPoseByPose(geometry_msgs::msg::PoseStamped targetpose, float l)
{
    float yaw = GetYawFromPose(targetpose);
    geometry_msgs::msg::PoseStamped pose = targetpose;
    pose.pose.position.x += l * cos(yaw);
    pose.pose.position.y += l * sin(yaw);
    return pose;
}

float GetAngleByPoses(geometry_msgs::msg::PoseStamped p1, geometry_msgs::msg::PoseStamped p2)
{
    float angle1 = GetYawFromPose(p1);
    float angle2 = GetYawFromPose(p2);
    float angle = angle2 - angle1;
    return angle;
}

float GetAngleByPoints(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2)
{
    float x1 = p1.x, y1 = p1.y;
    float x2 = p2.x, y2 = p2.y;
    float angle = atan2(y2 - y1, x2 - x1);
    return angle;
}

geometry_msgs::msg::Quaternion GetQuaternionMsgByPoints(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2)
{
    float angle = GetAngleByPoints(p1, p2);
    tf2::Quaternion q;
    q.setRPY(0, 0, angle);
    return tf2::toMsg(q);
}

float GetDistanceXY(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2)
{
    float x1 = p1.x, y1 = p1.y;
    float x2 = p2.x, y2 = p2.y;
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// 注意：此函数必须传入 tf_buffer
float GetDistance(std::shared_ptr<tf2_ros::Buffer> tf_buffer, geometry_msgs::msg::PoseStamped p)
{
    geometry_msgs::msg::PointStamped p1, p2;
    p1.header = p.header;
    p1.point = p.pose.position;
    
    // 使用传入的 buffer 进行转换
    transformPoint(tf_buffer, "base_link", p1, p2, "XXX");

    float ds = sqrt(pow(p2.point.x, 2) + pow(p2.point.y, 2));
    return ds;
}

float GetPathLength(nav_msgs::msg::Path path, int start_id = 0, int stop_id = -1)
{
    if (stop_id < 0 || stop_id >= (int)path.poses.size()-1)  stop_id = path.poses.size() - 1;
    if(path.poses.size() == 0)  return 0;

    float res = 0;
    for (int i = start_id; i <= stop_id - 1; i++)
    {
        geometry_msgs::msg::Point p1 = path.poses[i].pose.position;
        geometry_msgs::msg::Point p2 = path.poses[i + 1].pose.position;
        res += GetDistanceXY(p1, p2);
    }
    return res;
}

void GetPathLength(nav_msgs::msg::Path path, int start_id, float &passed_path, float &remain_path)
{
    passed_path = remain_path = 0;
    if (path.poses.size() == 0)  return;

    for (int i = 0; i < start_id; i++)
    {
        geometry_msgs::msg::Point p1 = path.poses[i].pose.position;
        geometry_msgs::msg::Point p2 = path.poses[i + 1].pose.position;
        passed_path += GetDistanceXY(p1, p2);
    }

    for (int i = start_id; i < (int)path.poses.size()-1; i++)
    {
        geometry_msgs::msg::Point p1 = path.poses[i].pose.position;
        geometry_msgs::msg::Point p2 = path.poses[i + 1].pose.position;
        remain_path += GetDistanceXY(p1, p2);
    }
}

// Modified to accept TF Buffer for coordinate transformation inside
void FindNearestPointInPath(std::shared_ptr<tf2_ros::Buffer> tf_buffer, nav_msgs::msg::Path path, geometry_msgs::msg::PointStamped point, int &id, float max_len)
{
    if (path.poses.size() == 0)  return;

    geometry_msgs::msg::PointStamped point_path;
    transformPoint(tf_buffer, path.header.frame_id, point, point_path);

    float dmin = 9999999;
    float len = 0;
    for (int i = id; i < (int)path.poses.size(); i++)
    {
        float dd = GetDistanceXY(path.poses[i].pose.position, point_path.point);
        if (dd <= dmin)  id = i, dmin = dd;

        if(i > 0)  len += GetDistanceXY(path.poses[i].pose.position, path.poses[i-1].pose.position);
        if(max_len > 0 && len > max_len) 
        {
            break;
        }
    }
}

string CheckMoveDir(nav_msgs::msg::Path path, int id)
{
    string move_dir = "";
    
    if (path.poses.size() < 2 || id + 1 >= (int)path.poses.size())  return move_dir;

    geometry_msgs::msg::PoseStamped p1 = path.poses[0];
    geometry_msgs::msg::PoseStamped p2 = path.poses[1];
    float pose_angle = GetYawFromPose(p1);
    float path_angle = GetAngleByPoints(p1.pose.position, p2.pose.position);
    float anglex = (path_angle - pose_angle) * 180 / M_PI;

    if (anglex > 180)  anglex -= 360;
    else if (anglex < -180)  anglex += 360;

    if (fabs(anglex) > 140)   move_dir = "back";
    else if (abs(anglex) < 40)   move_dir = "front";
    
    else if (fabs(anglex + 90) < 20)  move_dir = "right";
    else if (fabs(anglex - 90) < 20)  move_dir = "left";

    return move_dir;
}

visualization_msgs::msg::Marker displayCarPosition(geometry_msgs::msg::PoseStamped pose_stamped, int Quality)
{
    visualization_msgs::msg::Marker marker;
    marker.header = pose_stamped.header;
    marker.ns = "car_position_marker";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.9;
    marker.scale.y = 0.4;
    marker.scale.z = 0.3;
    marker.color.a = 1.0f;

    marker.frame_locked = true;
    // marker.lifetime = rclcpp::Duration(0,0); // Infinite
    marker.pose.position.x = pose_stamped.pose.position.x;
    marker.pose.position.y = pose_stamped.pose.position.y;
    marker.pose.position.z = 0;
    marker.pose.orientation = pose_stamped.pose.orientation;

    if(Quality == 4)
    {
        marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f;
    }
    else if(Quality == -1)
    {
        marker.color.r = 0.0f; marker.color.g = 0.0f; marker.color.b = 1.0f;
    }
    else
    {
        marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.0f;
    }
    return marker;
}


visualization_msgs::msg::Marker displayPoint(geometry_msgs::msg::PointStamped p, std_msgs::msg::ColorRGBA c, geometry_msgs::msg::Vector3 scale)
{
    visualization_msgs::msg::Marker marker;
    marker.header = p.header;
    marker.ns = "point_marker";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale = scale;
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    marker.color = c;
    marker.pose.position = p.point;
    return marker;
}

visualization_msgs::msg::Marker GetVisualMarker(geometry_msgs::msg::Point p, std_msgs::msg::ColorRGBA color)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;
    // marker.lifetime = rclcpp::Duration(0,0);
    marker.pose.position = p;
    marker.color = color;
    marker.color.a = 1.0f;
    return marker;
}

nav_msgs::msg::Path GetVisualPath(vector<geometry_msgs::msg::Point> path_buf)
{
    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Clock().now();
    path.header.frame_id = "map";
    for(size_t i = 0; i < path_buf.size(); i++)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position = path_buf[i];
        pose.header.stamp = rclcpp::Clock().now();
        pose.header.frame_id = "map";
        path.poses.push_back(pose);
    }
    return path;
}

std_msgs::msg::ColorRGBA GetColor(float r,float g,float b)
{
    std_msgs::msg::ColorRGBA color;
    color.r = r; color.g = g; color.b = b;
    return color;
}

#endif // COMMON_MYDISPLAY_H