#include <cmath>
#include <cstdint>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <common/public.hpp>
#include <cmath>
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
    TTimer command_watchdog;
    bool command_seen=false;
    
    rclcpp::Subscription<control_msgs::msg::CarCtr>::SharedPtr carctr_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speedlimit_sub; 
    rclcpp::Publisher<control_msgs::msg::CarState>::SharedPtr carstate_pub;

    control_msgs::msg::CarCtr car_ctr_msg; 
    control_msgs::msg::CarState car_state_msg;
    
    bool run_enable=false;
    float mode_change_time=3.0;
    int  mode_latched = -1;          // P2-修复⑤：模式切换锁存（-1=未在切换；否则=已下发切换的指令侧模式）
    TNodeCheck *nodecheck;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr driver_diag_pub,driver_io_pub;
    uint64_t io_sequence=0,driver_sequence=0,rx_batches=0,tx_calls=0,command_callbacks=0;
    bool echo_seen=false,err_seen=false;
    bool wheel_seen[4]={false,false,false,false};
    TTimer echo_age,err_age,wheel_age[4],driver_loop_timer;
    double driver_dt_s=0;bool driver_tick_seen=false;
    std::string driver_reason="startup",last_tx,last_rx;
    static long long unix_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();}
    static long long steady_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
    static std::string quote(const std::string& value){
        std::ostringstream out;out << '"';
        for(unsigned char c:value){if(c=='"' || c=='\\')out << '\\' << c;else if(c<32)out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << (int)c << std::dec;else out << c;}
        out << '"';return out.str();
    }
    static bool numeric_token(const std::string& value){char* end=nullptr;double v=strtod(value.c_str(),&end);return end!=value.c_str() && *end==0 && std::isfinite(v);}
    static std::string number(double value){if(!std::isfinite(value))return "null";std::ostringstream out;out << std::setprecision(10) << value;return out.str();}
    void io_event(const std::string& direction,const std::string& packet,int batch_count=1){
        if(!driver_io_pub)return;
        std_msgs::msg::String msg;std::ostringstream out;
        out << "{\"schema_version\":1,\"sequence\":" << ++io_sequence << ",\"producer_wall_ns\":" << unix_ns() << ",\"producer_steady_ns\":" << steady_ns()
            << ",\"direction\":" << quote(direction) << ",\"payload\":" << quote(packet) << ",\"rx_batch_count\":" << batch_count << "}";
        msg.data=out.str();try{driver_io_pub->publish(msg);}catch(const std::exception&){}
    }
    void drain_socket_diagnostics(){
        for(const auto& p:udp->diagnostics.take()){
            if(!driver_io_pub)continue;
            std_msgs::msg::String msg;std::ostringstream out;
            out << "{\"schema_version\":1,\"sequence\":" << ++io_sequence << ",\"socket_sequence\":" << p.sequence << ",\"producer_wall_ns\":" << p.wall_ns << ",\"producer_steady_ns\":" << p.steady_ns
                << ",\"direction\":" << quote(p.direction) << ",\"payload\":" << quote(p.payload) << ",\"peer_ip\":" << quote(p.peer_ip) << ",\"peer_port\":" << p.peer_port << ",\"result_bytes\":" << p.result_bytes << ",\"errno\":" << p.error_code << "}";
            msg.data=out.str();try{driver_io_pub->publish(msg);}catch(const std::exception&){}
        }
    }
    void send_packet(char* packet){udp->Send(packet);last_tx=packet;++tx_calls;io_event("tx_send_called",last_tx);}
    void publish_driver_diag(){
        if(!driver_diag_pub)return;
        std_msgs::msg::String msg;std::ostringstream out;
        out << "{\"schema_version\":1,\"sequence\":" << ++driver_sequence << ",\"producer_wall_ns\":" << unix_ns() << ",\"producer_steady_ns\":" << steady_ns()
            << ",\"driver_dt_s\":" << number(driver_dt_s) << ",\"reason\":" << quote(driver_reason) << ",\"socket_diagnostic_dropped\":" << udp->diagnostics.dropped() << ",\"command_callbacks\":" << command_callbacks << ",\"tx_send_calls\":" << tx_calls << ",\"rx_batches\":" << rx_batches
            << ",\"command_seen\":" << (command_seen?"true":"false") << ",\"command_age_s\":" << (command_seen?number(command_watchdog.GetValue()):"null")
            << ",\"command_heartbeat\":" << car_ctr_msg.heartbeat << ",\"command_mode\":" << car_ctr_msg.turnmode << ",\"command_speed\":" << number(car_ctr_msg.speed)
            << ",\"firmware_angle_command_deg\":" << number(car_ctr_msg.angle) << ",\"run_enable\":" << (run_enable?"true":"false") << ",\"mode_latched\":" << mode_latched
            << ",\"mode_wait_elapsed_s\":" << number(delay_time.GetValue()) << ",\"echo_seen\":" << (echo_seen?"true":"false") << ",\"echo_age_s\":" << (echo_seen?number(echo_age.GetValue()):"null")
            << ",\"feedback_mode_raw\":" << car_state_msg.turnmode << ",\"feedback_enable_raw\":" << car_state_msg.enable << ",\"feedback_ctrmode_raw\":" << car_state_msg.ctrmode
            << ",\"wheel_feedback_age_s\":[";
        for(int i=0;i<4;++i){if(i)out << ',';out << (wheel_seen[i]?number(wheel_age[i].GetValue()):"null");}
        out << "],\"wheel_speed_cached_mps\":[";
        for(int i=0;i<4;++i){if(i)out << ',';out << number(car_state_msg.speed[i]);}
        out << "],\"errcode_seen\":" << (err_seen?"true":"false") << ",\"errcode_age_s\":" << (err_seen?number(err_age.GetValue()):"null")
            << ",\"actual_steering_feedback_available\":false,\"last_tx\":" << quote(last_tx) << ",\"last_rx\":" << quote(last_rx) << "}";
        msg.data=out.str();try{driver_diag_pub->publish(msg);}catch(const std::exception&){}
    }

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
        ++command_callbacks;driver_reason="command_received";
        car_ctr_msg = *msg;
        command_watchdog.Clear();
        command_seen=true;

        // 2026-11-06：指令被丢弃时必须能看见原因，否则"上层在发 0.7m/s 而车不动"无从判断。
        //   ctrmode==0 = 底盘在手动/未知档，或从未收到过底盘回包（CarState 默认值 0）。
        static TTimer drop_tmr;
        if(!run_enable || car_state_msg.ctrmode==0)
        {
            if (drop_tmr.GetValue() > 3)
            {
                drop_tmr.Clear();
                RCLCPP_WARN(nh->get_logger(),
                    "[car_ctr] 丢弃 /car_cmd：run_enable=%d ctrmode=%d "
                    "（ctrmode=0 表示底盘在手动档或从未收到回包）",
                    (int)run_enable, (int)car_state_msg.ctrmode);
            }
            driver_reason=!run_enable?"run_disabled":"manual_or_no_echo";
            return;
        }

        char str[100]={0};
        float speed = car_ctr_msg.speed;
        car_ctr_msg.angle *=-1; // 底层角度反的

        // A stop must not wait behind a mechanical mode transition. Run 0
        // stops the CURRENT firmware mode before any SteerModeCtrl request.
        if (!car_ctr_msg.enable) {
            driver_reason="command_disabled_stop";
            sprintf(str,"Car Run 0.0 %.2f",car_ctr_msg.angle);
            send_packet(str);
            return; // A disabled command must never fall through to nonzero Run.
        }
        if (fabs(speed)<1e-6) {
            sprintf(str,"Car Run 0.0 %.2f",car_ctr_msg.angle);
            send_packet(str);
        }
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
                driver_reason="mode_switch_send_called";
                sprintf(str, "Car SteerModeCtrl %d", cmd_tm);
                send_packet(str);
                delay_time.Clear();
                return;
            }
            if (fb_tm != cmd_tm)
            {
                // 模式不匹配：下发切换命令并锁存
                mode_latched = cmd_tm;
                driver_reason="mode_switch_send_called";
                sprintf(str, "Car SteerModeCtrl %d", cmd_tm);
                send_packet(str);
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
                driver_reason="mode_switch_send_called";
                sprintf(str, "Car SteerModeCtrl %d", mode_latched);
                send_packet(str);
                if (fb_tm != -2)
                    delay_time.Clear();
                return;
            }
        }

        driver_reason="mechanical_mode_wait";
        // ---- 模式已匹配（或锁存刚清除）：按 3s 模式空窗后发 Run ----
        if(delay_time.GetValue() > mode_change_time && (cmd_tm==0 || cmd_tm==1))
        {
            driver_reason="ack_run_send_called";
            sprintf(str, "Car Run %.2f %.2f", speed, car_ctr_msg.angle);
            send_packet(str);
            // RCLCPP_INFO(nh->get_logger(), "UDP Send: %s", str);
        }
        else if(delay_time.GetValue() > mode_change_time && (cmd_tm==2 || cmd_tm==4))
        {
            float send_speed = speed;

            if (cmd_tm==4)
            {
                send_speed = -2.0 * send_speed;
            }
            driver_reason="spin_or_lateral_run_send_called";
            sprintf(str, "Car Run %.2f 0.0", send_speed);
            send_packet(str);
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

        driver_diag_pub=nh->create_publisher<std_msgs::msg::String>("/car_driver/diag",10);
        driver_io_pub=nh->create_publisher<std_msgs::msg::String>("/car_driver/io",10);
        udp = new TUDP(local_port);
        udp->diagnostics.enable(true); 
        
        if(!remote_ip.empty()) {
             udp->AddRemoteIP(remote_ip, remote_port); 
        }

        send_packet((char*)"OK");
        usleep(10000);

        carstate_pub = nh->create_publisher<control_msgs::msg::CarState>("/car_state", 10);

        carctr_sub = nh->create_subscription<control_msgs::msg::CarCtr>(
            "/car_cmd", 
            1, 
            std::bind(&TCarCtr::CarCmdCallback, this, std::placeholders::_1)
        );

        Enable(true);  
    }
    
    void Stop(int flag)
    {
        char str[50];
        sprintf(str, "Car Stop %d", flag);
        send_packet(str);
    }

    void AutoCtr(bool v)
    {
        char str[50];
        if (v)  sprintf(str, "Car Code_Ctrl");
        else  sprintf(str, "Car Key_Ctrl");
        send_packet(str);
        usleep(2000);
    }

    void Enable(bool e)
    {
        char str[50];
        if (e)  sprintf(str, "Car Enable");
        else  sprintf(str, "Car Disable");
        send_packet(str);
        usleep(2000);
    }

    void UDP_Proc() 
    {
        // ★2026-11-06 修复（实车 run_19700108_135820）：周期性的"切程控 + 使能"重试
        //   原来写在 `if (udp->rec_count == 0) return;` 之后 —— 而底盘在自身状态没有
        //   变化时往往不再回包（本 run 全 183s 只在 t=12.9s 回过 1 帧 `Car 0 1 0`，
        //   即 enable=0 未使能）。于是 car_ctr 只发过构造里那一次 `Car Enable`，
        //   此后永远不再重发 → enable 一直是 0 → 底盘收到 `Car Run` 也不执行 →
        //   车物理上不动；同时 /car_state 再无新帧（本 run car_state 全程 0Hz）。
        //   这是一条闭环死锁：不回包 → 不重试使能 → 不使能 → 不动也不回包。
        //   修复：把重试提前到回包判断之前，与"有没有回包"彻底解耦，并加无回包告警。
        static TTimer tmr;          // 3s 周期：重发 Code_Ctrl / Enable
        static TTimer no_rec_tmr;   // 距最后一次有效回包的时间

        if (udp->rec_count != 0)  no_rec_tmr.Clear();
        if (tmr.GetValue() > 3)
        {
            tmr.Clear();
            if(run_enable)
            {
                if(car_state_msg.ctrmode==0)
                {
                    AutoCtr(true);      // 先切程控
                    Enable(true);       // 再请求使能（两个请求均幂等，可重复下发）
                }
                else if(car_state_msg.enable != 255)
                {
                    Enable(true);
                }
            }
            else 
            {
                if(car_state_msg.enable!=0)  Enable(false);
                else if(car_state_msg.ctrmode==1)  AutoCtr(false);
            }    
            if (no_rec_tmr.GetValue() > 3.0)
            {
                RCLCPP_WARN(nh->get_logger(),
                    "[car_ctr] 已 %.1fs 收不到底盘 UDP 回包(/car_state 不刷新)，"
                    "已重发 Code_Ctrl+Enable 仍无响应。请检查：网线/网段(192.168.15.x)、"
                    "底盘上电与急停、底盘是否在程控档。当前 enable=%d ctrmode=%d",
                    no_rec_tmr.GetValue(), (int)car_state_msg.enable, (int)car_state_msg.ctrmode);
            }
        }

        if (udp->rec_count == 0)  return;
        const int received_batch_count=udp->rec_count;
        udp->rec_count = 0;

        char recbuf[1025] = {};
        strncpy(recbuf, udp->rec_buf, sizeof(recbuf)-1);
        ++rx_batches;last_rx=recbuf;io_event("rx_observed",last_rx,received_batch_count);

        vector<string> strs = split(recbuf, ";");
        
        for (size_t i = 0; i < strs.size(); i++)
        {
            vector<string> substrs = split(strs.at(i), " ");
            if (substrs.size()>=4 && substrs[0] == "Car")
            {
                echo_seen=numeric_token(substrs[1]) && numeric_token(substrs[2]) && numeric_token(substrs[3]);echo_age.Clear();
                car_state_msg.enable = atoi(substrs.at(1).c_str());
                car_state_msg.ctrmode = atoi(substrs.at(2).c_str());
                car_state_msg.turnmode = atoi(substrs.at(3).c_str());
                nodecheck->Find("node_rate")->Beat();
            }
            if (substrs.size()>=3 && substrs[0].find("Wheel") != string::npos) 
            {
                int len = substrs[0].length();
                int id = atoi(substrs[0].substr(len - 1, len).c_str());
                
                // 修复：移除 resize，增加边界检查
                if(id >= 0 && id < (int)car_state_msg.speed.size()) {
                    car_state_msg.speed[id] = atof(substrs.at(2).c_str()) * RPM2SPD;
                    wheel_seen[id]=numeric_token(substrs[2]) && std::isfinite(car_state_msg.speed[id]);wheel_age[id].Clear();
                }
            }
            if (substrs.size()>=9 && substrs[0].find("errcode") != string::npos) 
            {
                err_seen=true;err_age.Clear();
                // 修复：移除 resize，直接填充固定数组
                for(int k=0; k<8 && k < (int)car_state_msg.errcode.size(); k++)  
                    car_state_msg.errcode[k]=atoi(substrs[1+k].c_str());
            }
        }
            
        carstate_pub->publish(car_state_msg);
    }

    void run()
    {
        driver_dt_s=driver_tick_seen?driver_loop_timer.GetValue():0.;driver_tick_seen=true;driver_loop_timer.Clear();
        nh->get_parameter("run_enable", run_enable);
        UDP_Proc();
        // Independent of ROS-time and of the 3 s steering-mode delay.
        if(command_seen && command_watchdog.GetValue()>.5) {
            char stop_packet[100];
            sprintf(stop_packet,"Car Run 0.0 %.2f",car_ctr_msg.angle);
            driver_reason="command_watchdog_stop";
            send_packet(stop_packet);
        }
        drain_socket_diagnostics();
        publish_driver_diag();
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
