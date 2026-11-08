#include <common/public.hpp>
#include <common/myudp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <control_msgs/msg/car_state.hpp>
#include <control_msgs/msg/car_ctr.hpp>

const float RPM2SPD = 3.1416 * 0.24 / 60;
const float SPD2RPM = 1 / RPM2SPD;

class TCarCtr 
{
private:
    TUDP *udp;
    TTimer delay_time;
    
    rclcpp::Subscription<control_msgs::msg::CarCtr>::SharedPtr carctr_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speedlimit_sub; 
    rclcpp::Publisher<control_msgs::msg::CarState>::SharedPtr carstate_pub;

    control_msgs::msg::CarCtr car_ctr_msg; 
    control_msgs::msg::CarState car_state_msg;
    
    bool run_enable=false;
    float mode_change_time=3.0;
    int  mode_latched = -1;          // P2-修复⑤：模式切换锁存（-1=未在切换；否则=已下发切换的指令侧模式）
    TNodeCheck *nodecheck;

    // 反馈侧(CarState.msg) turnmode → 指令侧(CarCtr.msg) turnmode 语义映射
    // ★2026-11-06 依《底层控制接口文档》4.2/第5章 + BITCar.cpp:287 修正（P2 直接根因）：
    //   上行帧 "Car <Active> <CtrMode> <turnmode>" 的 turnmode 与下行
    //   "Car SteerModeCtrl <m>" 是【同一套编号】，且 turnmode 在收到指令的瞬间即被赋值回显
    //   （BITCar.cpp:287  `turnmode = mode; giv_spd = 0;`），并非"四轮机械到位后才更新"。
    //   本车实际编号：0=阿克曼 1=斜行(对称转向) 2=横移(平移) 4=原地自转 5=自转占位(轮速强制0)
    //   旧实现按"反馈 2=自转、4=斜行、1=差速"的错误注释编码，导致下发指令 4 后
    //   回显 fb_tm=map(4)=1 ≠ cmd_tm=4 → 每帧重发 SteerModeCtrl 且 delay_time 反复清零
    //   → 永远不满足 3s 空窗、永远不发 Run → 车物理上不转 → 上游 spin_timeout(6s) 超时
    //   → 切回阿克曼。这正是"切了自转却没转就切回"的直接原因。
    // 注意：本函数仅用于"模式是否已到位"的比较，不回写 car_state_msg.turnmode 原值。
    //      因 turnmode 是回显语义，本比较只证明"指令已被底盘接收"；四轮机械到位由
    //      car_ctr 的 3s 空窗(mode_change_time) + 底层约 1s 过渡斜坡共同兜底。
    // 返回 -1 表示未知/未到位，触发重新下发。
    int map_state_turnmode_to_cmd(int state_tm)
    {
        switch(state_tm)
        {
            case 0:  return 0;   // 阿克曼     → 阿克曼
            case 1:  return 1;   // 斜行(对称转向) → 斜行
            case 2:  return 2;   // 横移(平移) → 横移
            case 4:  return 4;   // 原地自转   → 自转（本车指令与反馈同为 4）
            case 5:  return -1;  // 自转占位模式：底层轮速强制为 0，不算到位
            default: return -1;  // 未知编号同样视为未到位
        }
    }

    void CarCmdCallback(const control_msgs::msg::CarCtr::SharedPtr msg)
    {
        car_ctr_msg = *msg;

        if(!run_enable || car_state_msg.ctrmode==0) return;

        char str[100]={0};
        float speed = car_ctr_msg.speed;
        car_ctr_msg.angle *=-1; // 底层角度反的

        int cmd_tm = (int)car_ctr_msg.turnmode;
        int fb_tm  = map_state_turnmode_to_cmd(car_state_msg.turnmode);

        // ---- P2-修复⑤：模式切换锁存 ----
        // 底盘机械切换（四轮摆角）有物理时延，反馈滞后停留在旧值；若仅比较"当前帧反馈
        // vs 当前指令"，切换途中上游指令一变化就会被误判"已匹配"，反向 SteerModeCtrl
        // 不补发、3s 后按新指令发 Run 造成底盘模式混乱。
        // 锁存语义：下发切换后，反馈必须真正到达锁存目标模式才认为切换完成；期间持续
        // 补发锁存目标（防 UDP 丢包）并忽略上游中途变化，完成后自然跟随新指令。
        if (mode_latched < 0)
        {
            if (fb_tm == -2)
            {
                // 注：新映射（2026-11-06）不再返回 -2，本分支实际不可达；
                //     保留仅为兼容旧回滚版本，勿据此判断存在"差速中间态"。
                // 反馈处于切换中间态（差速）：以当前指令为切换目标，开始锁存
                mode_latched = cmd_tm;
                sprintf(str, "Car SteerModeCtrl %d", cmd_tm);
                udp->Send(str);
                delay_time.Clear();
                return;
            }
            if (fb_tm != cmd_tm)
            {
                // 模式不匹配：下发切换命令并锁存
                mode_latched = cmd_tm;
                sprintf(str, "Car SteerModeCtrl %d", cmd_tm);
                udp->Send(str);
                delay_time.Clear();
                return;
            }
        }
        else
        {
            // 切换进行中：反馈必须到达锁存目标模式
            if (fb_tm == mode_latched)
            {
                // 切换完成：清除锁存，模式空窗从 0 起算
                mode_latched = -1;
                delay_time.Clear();
            }
            else
            {
                // 未到位：持续补发锁存目标（不采信上游中途变化）；中间态(-2)不重置空窗
                //（避免切换慢时 3s 窗口永远起算不了），其他未到位状态重置空窗
                sprintf(str, "Car SteerModeCtrl %d", mode_latched);
                udp->Send(str);
                if (fb_tm != -2)
                    delay_time.Clear();
                return;
            }
        }

        // ---- 模式已匹配（或锁存刚清除）：按 3s 模式空窗后发 Run ----
        if(delay_time.GetValue() > mode_change_time && (cmd_tm==0 || cmd_tm==1))
        {
            sprintf(str, "Car Run %.2f %.2f", speed, car_ctr_msg.angle);
            udp->Send(str);
            // RCLCPP_INFO(nh->get_logger(), "UDP Send: %s", str);
        }
        else if(delay_time.GetValue() > mode_change_time && (cmd_tm==2 || cmd_tm==4))
        {
            float send_speed = speed;

            if (cmd_tm==4)
            {
                send_speed = -2.0 * send_speed;
            }
            sprintf(str, "Car Run %.2f 0.0", send_speed);
            udp->Send(str);
            // RCLCPP_INFO(nh->get_logger(), "UDP Send: %s", str);
        }
    }

public:
    rclcpp::Node::SharedPtr nh;

    TCarCtr()
    {
        nh = rclcpp::Node::make_shared("car_ctr_node");

        nh->declare_parameter<std::string>("remote_ip", "");
        nh->declare_parameter<int>("local_port", 8090);   // 本地监听端口，默认 8090
        nh->declare_parameter<int>("remote_port", 8080);  // 远程目标端口，默认 8080
        nh->declare_parameter<bool>("run_enable", false);
        nh->declare_parameter<double>("mode_change_time", 3.0);  // 模式切换空窗期(秒)，默认3.0与原硬编码行为一致
        nh->declare_parameter<double>("check/node_rate", 10.0);
        nh->declare_parameter<double>("check/track_rate", 10.0);
        nh->declare_parameter<std::string>("check/msg", "");

        std::string remote_ip = "";
        int local_port = 8090;
        int remote_port = 8080;

        nh->get_parameter("remote_ip", remote_ip);
        nh->get_parameter("local_port", local_port);
        nh->get_parameter("remote_port", remote_port);
        nh->get_parameter("mode_change_time", mode_change_time);

        nodecheck = new TNodeCheck(nh, "node_rate");
        nodecheck->Find("node_rate")->SetLimit(10);

        udp = new TUDP(local_port); 
        
        if(!remote_ip.empty()) {
             udp->AddRemoteIP(remote_ip, remote_port); 
        }

        udp->Send((char*)"OK");
        usleep(10000);

        carstate_pub = nh->create_publisher<control_msgs::msg::CarState>("/car_state", 10);

        carctr_sub = nh->create_subscription<control_msgs::msg::CarCtr>(
            "/car_cmd", 
            10, 
            std::bind(&TCarCtr::CarCmdCallback, this, std::placeholders::_1)
        );

        Enable(true);  
    }
    
    void Stop(int flag)
    {
        char str[50];
        sprintf(str, "Car Stop %d", flag);
        udp->Send(str);
    }

    void AutoCtr(bool v)
    {
        char str[50];
        if (v)  sprintf(str, "Car Code_Ctrl");
        else  sprintf(str, "Car Key_Ctrl");
        udp->Send(str);
        usleep(2000);
    }

    void Enable(bool e)
    {
        char str[50];
        if (e)  sprintf(str, "Car Enable");
        else  sprintf(str, "Car Disable");
        udp->Send(str);
        usleep(2000);
    }

    void UDP_Proc() 
    {
        if (udp->rec_count == 0)  return;
        udp->rec_count = 0;

        char recbuf[1000];
        strcpy(recbuf, udp->rec_buf);

        vector<string> strs = split(recbuf, ";");
        
        for (size_t i = 0; i < strs.size(); i++)
        {
            vector<string> substrs = split(strs.at(i), " ");
            if (substrs.size()>=4 && substrs[0] == "Car")
            {
                car_state_msg.enable = atoi(substrs.at(1).c_str());
                car_state_msg.ctrmode = atoi(substrs.at(2).c_str());
                car_state_msg.turnmode = atoi(substrs.at(3).c_str());
                nodecheck->Find("node_rate")->Beat();
            }
            if (substrs.size()>0 && substrs[0].find("Wheel") != string::npos) 
            {
                int len = substrs[0].length();
                int id = atoi(substrs[0].substr(len - 1, len).c_str());
                
                // 修复：移除 resize，增加边界检查
                if(id >= 0 && id < (int)car_state_msg.speed.size()) {
                    car_state_msg.speed[id] = atof(substrs.at(2).c_str()) * RPM2SPD;
                }
            }
            if (substrs.size()>=9 && substrs[0].find("errcode") != string::npos) 
            {
                // 修复：移除 resize，直接填充固定数组
                for(int k=0; k<8 && k < (int)car_state_msg.errcode.size(); k++)  
                    car_state_msg.errcode[k]=atoi(substrs[1+k].c_str());
            }
        }
            
        carstate_pub->publish(car_state_msg);

        static TTimer tmr;
        if(tmr.GetValue()>3)
        {
            tmr.Clear();
            if(run_enable)
            {
                if(car_state_msg.ctrmode==0)  
                    AutoCtr(true);  
                else if(car_state_msg.enable != 255)  
                    Enable(true);
            }
            else 
            {
                if(car_state_msg.enable!=0)  Enable(false);
                else if(car_state_msg.ctrmode==1)  AutoCtr(false);
            }    
        }    
    }

    void run()
    {
        nh->get_parameter("run_enable", run_enable);
        UDP_Proc();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    TCarCtr myCar;
    rclcpp::Rate rate(20);
    while (rclcpp::ok())
    {
        myCar.run();
        rclcpp::spin_some(myCar.nh);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
};