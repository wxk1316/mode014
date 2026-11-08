#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// TF2 相关头文件
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// 自定义工具头文件
#include <common/public.hpp>
#include <common/mydisplay.hpp> 

using namespace std;
using std::placeholders::_1;

class TGlobalPathPlan : public rclcpp::Node
{
private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr frontpath_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rawpath_pub;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr rawpath_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;

    // TF Buffer 和 Listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    nav_msgs::msg::Path rawpath, path, frontpath;
    float localrange = 10.0;
    TNodeCheck *nodecheck;
    geometry_msgs::msg::PoseStamped cur_carPos, goal_pos;
    int path_id = 0;

public:
    TGlobalPathPlan() : Node("globalpathplan")
    {
        // 1. 初始化 TF Buffer 和 Listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ROS 2 参数声明
        this->declare_parameter<float>("local_range", 8.0);
        this->declare_parameter<float>("path_space", 0.05);
        this->declare_parameter<double>("check/node_rate", 10.0);
        this->declare_parameter<double>("check/gps_rate", 10.0);
        this->declare_parameter<std::string>("check/msg", "");

        // 获取参数
        this->get_parameter("local_range", localrange);
        
        // 创建发布者
        path_pub = this->create_publisher<nav_msgs::msg::Path>("path", 10);
        rawpath_pub = this->create_publisher<nav_msgs::msg::Path>("rawpath", 10);
        frontpath_pub = this->create_publisher<nav_msgs::msg::Path>("frontpath", 10); 
        
        // 创建订阅者
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/localization", 10, std::bind(&TGlobalPathPlan::odomCallback, this, _1));
            
        rawpath_sub = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10, std::bind(&TGlobalPathPlan::rawpathCallback, this, _1));
            
        goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal", 10, std::bind(&TGlobalPathPlan::goalCallback, this, _1));

        // NodeCheck 初始化
        nodecheck = new TNodeCheck(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), "node_rate gps_rate");
        nodecheck->Find("node_rate")->SetLimit(10);
        nodecheck->Find("gps_rate")->SetLimit(10);
    };

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void rawpathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void PathInterpolation(const nav_msgs::msg::Path inputPath);
    void Run();
};

void TGlobalPathPlan::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    goal_pos = *msg;
}

void TGlobalPathPlan::rawpathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    rawpath.header.frame_id = "map";
    rawpath.header.stamp = this->get_clock()->now();
    rawpath.poses.clear();

    if (msg->poses.empty()) return;

    for (const auto& p : msg->poses) {
        rawpath.poses.push_back(p);
    }
        
    PathInterpolation(rawpath);

    geometry_msgs::msg::PointStamped point;
    point.header = goal_pos.header;
    point.point = goal_pos.pose.position;
    // --- 【修复代码开始：防止 goal_pos 为空导致的刷屏报错】 ---
    if (point.header.frame_id.empty())
    {
        point.header.frame_id = "map"; 
    }
    if (path.header.frame_id.empty())
    {
        path.header.frame_id = "map";
    }
    // --- 【修复代码结束】 ---
    path_id = 0;
    
    // 【修改点】传入 tf_buffer_ 作为第一个参数
    FindNearestPointInPath(tf_buffer_, path, point, path_id, -1);
    
    if(path_id > 0) path.poses.resize(path_id + 1);
    path_id = 0;
    // RCLCPP_INFO(this->get_logger(), "%d %d", path_id, (int)path.poses.size());
}

// 对rawpath进行插值处理
void TGlobalPathPlan::PathInterpolation(const nav_msgs::msg::Path inputPath)
{
    if (inputPath.poses.size() <= 1) return;
    
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";
    path.poses.clear();

    float ds = 0.1;
    this->get_parameter("path_space", ds); 
    
    for (size_t i = 0; i < inputPath.poses.size() - 1; i++)
    {
        geometry_msgs::msg::Point p1 = inputPath.poses[i].pose.position;
        geometry_msgs::msg::Point p2 = inputPath.poses[i + 1].pose.position;
        float dd = GetDistanceXY(p1, p2);
        
        if (ds <= 0) ds = 0.1; // 防止除以0
        int n = dd / ds;
        if (n < 1) continue;
        n += 1;
        
        float dx = (p2.x - p1.x) / n;
        float dy = (p2.y - p1.y) / n;
        float dz = (p2.z - p1.z) / n;

        for (int j = 0; j < n; j++)
        {
            geometry_msgs::msg::PoseStamped p;

            p.pose.position.x = p1.x + dx * j;
            p.pose.position.y = p1.y + dy * j;
            p.pose.position.z = p1.z + dz * j;

            p.pose.orientation = inputPath.poses[i + 1].pose.orientation;
            p.header = inputPath.header;

            path.poses.push_back(p);
        }
    }
}

void TGlobalPathPlan::Run()
{
    nodecheck->Find("node_rate")->Beat();

    rawpath.header.frame_id = "map";
    rawpath.header.stamp = this->get_clock()->now(); // 原始路径
    
    path.header = rawpath.header;
    frontpath.header = rawpath.header;
    
    rawpath_pub->publish(rawpath);
    path_pub->publish(path);
    frontpath_pub->publish(frontpath);
}

void TGlobalPathPlan::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    nodecheck->Find("gps_rate")->Beat();
    
    cur_carPos.header = msg->header;
    cur_carPos.pose = msg->pose.pose;

    if(path.poses.size() == 0) return;

    geometry_msgs::msg::PointStamped point;
    point.header = msg->header;
    point.point = cur_carPos.pose.position;

    // 【修改点】传入 tf_buffer_ 作为第一个参数
    FindNearestPointInPath(tf_buffer_, path, point, path_id, 10);

    frontpath.poses.clear();
    frontpath.header = path.header;
    int p_id = path_id;
    float distance = 0;
    
    if (p_id >= 0 && p_id < (int)path.poses.size())
    {
        geometry_msgs::msg::Point p0 = path.poses[p_id].pose.position;
        while (p_id >= 0 && p_id < (int)path.poses.size() && distance <= localrange)
        {
            geometry_msgs::msg::Point p1 = path.poses[p_id].pose.position;
            distance += GetDistanceXY(p0, p1);
            p0 = p1;
            frontpath.poses.push_back(path.poses[p_id]);
            p_id++;
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TGlobalPathPlan>();

    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        node->Run();
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}