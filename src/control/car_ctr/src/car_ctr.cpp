#include <cmath>
#include "mode_transaction.hpp"
#include <cstdint>
#include <stdexcept>
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
    ModeTransaction transaction;
    TTimer steady_clock,body_age;
    bool turn_seen[4]={false,false,false,false};double turn_position[4]={0,0,0,0};TTimer turn_age[4];
    double command_at=-1e9,car_echo_at=-1e9,body_at=-1e9,error_at=-1e9,management_until=-1e9;
    bool body_valid=false;double body_speed=0.,body_yaw_rate=0.,body_source_age=0.;
    double wheel_feedback_scale=1.,angle_sign=-1.,spin_command_scale=.429,spin_sign=-1.;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr body_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_state_pub;
    TTimer delay_time;
    TTimer command_watchdog;
    bool command_seen=false;
    
    rclcpp::Subscription<control_msgs::msg::CarCtr>::SharedPtr carctr_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speedlimit_sub; 
    rclcpp::Publisher<control_msgs::msg::CarState>::SharedPtr carstate_pub;

    control_msgs::msg::CarCtr car_ctr_msg; 
    control_msgs::msg::CarState car_state_msg;
    
    bool run_enable=false;
    double mode_change_time=3.0;
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
        const double now=steady_ns()*1e-9;
        if(!driver_diag_pub)return;
        std_msgs::msg::String msg;std::ostringstream out;
        out << "{\"schema_version\":1,\"sequence\":" << ++driver_sequence << ",\"producer_wall_ns\":" << unix_ns() << ",\"producer_steady_ns\":" << steady_ns()
            << ",\"driver_dt_s\":" << number(driver_dt_s) << ",\"reason\":" << quote(driver_reason) << ",\"socket_diagnostic_dropped\":" << udp->diagnostics.dropped() << ",\"command_callbacks\":" << command_callbacks << ",\"tx_send_calls\":" << tx_calls << ",\"rx_batches\":" << rx_batches
            << ",\"command_seen\":" << (command_seen?"true":"false") << ",\"command_age_s\":" << (command_seen?number(now-command_at):"null")
            << ",\"command_heartbeat\":" << car_ctr_msg.heartbeat << ",\"command_mode\":" << car_ctr_msg.turnmode << ",\"command_speed\":" << number(car_ctr_msg.speed)
            << ",\"firmware_angle_command_deg\":" << number(car_ctr_msg.angle) << ",\"run_enable\":" << (run_enable?"true":"false") << ",\"mode_latched\":" << mode_latched
            << ",\"mode_wait_elapsed_s\":" << number(transaction.phase=="WAIT_MECHANICAL" && transaction.echo_at>=0?now-transaction.echo_at:0.) << ",\"echo_seen\":" << (echo_seen?"true":"false") << ",\"echo_age_s\":" << (echo_seen?number(now-car_echo_at):"null")
            << ",\"feedback_mode_raw\":" << car_state_msg.turnmode << ",\"feedback_enable_raw\":" << car_state_msg.enable << ",\"feedback_ctrmode_raw\":" << car_state_msg.ctrmode
            << ",\"wheel_feedback_age_s\":[";
        for(int i=0;i<4;++i){if(i)out << ',';out << (wheel_seen[i]?number(wheel_age[i].GetValue()):"null");}
        out << "],\"wheel_speed_cached_mps\":[";
        for(int i=0;i<4;++i){if(i)out << ',';out << number(car_state_msg.speed[i]);}
        out << "],\"errcode_seen\":" << (err_seen?"true":"false") << ",\"errcode_age_s\":" << (err_seen?number(now-error_at):"null")
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
        ++command_callbacks;car_ctr_msg=*msg;command_watchdog.Clear();command_seen=true;command_at=steady_ns()*1e-9;
        car_ctr_msg.angle *= angle_sign;
    }
    void ControlTick() {
        const double now=steady_ns()*1e-9;
        if(!run_enable){transaction.rearm();char stop[100];std::snprintf(stop,sizeof(stop),"Car Run 0 %.2f",transaction.last_run_angle);if(now-transaction.last_tx>.08){send_packet(stop);transaction.last_tx=now;}return;}
        const bool fresh=echo_seen && now-car_echo_at<=transaction.feedback_timeout;
        // Serialized acquisition uses >=80ms between distinct commands.
        if((!fresh || car_state_msg.ctrmode!=1 || car_state_msg.enable!=255) && now-transaction.last_tx>.1){
            char packet[100];
            if(!fresh || car_state_msg.ctrmode!=1)std::snprintf(packet,sizeof(packet),"Car Code_Ctrl");
            else std::snprintf(packet,sizeof(packet),"Car Enable");
            send_packet(packet);transaction.last_tx=now;management_until=now+.08;transaction.ready=false;
        }
        bool alarms=false;for(int e:car_state_msg.errcode)alarms|=(e!=0);
        const bool body_fresh=body_valid && now-body_at+body_source_age<.35;
        const bool stopped=body_speed<.035 && std::abs(body_yaw_rate)<.06;
        const bool diag_fresh=err_seen && now-error_at<2.;
        const bool live=command_seen && now-command_at<.5 && car_ctr_msg.enable && diag_fresh;
        double speed=car_ctr_msg.speed;
        if(car_ctr_msg.turnmode==4)speed=spin_sign*spin_command_scale*speed;
        speed=std::clamp(speed,-2.,2.);
        const double limit=car_ctr_msg.turnmode==1?90.:30.;
        const double angle=std::clamp((double)car_ctr_msg.angle,-limit,limit);
        std::string packet=now<management_until?"":transaction.next(now,car_ctr_msg.turnmode,speed,angle,
            car_state_msg.turnmode,fresh,car_state_msg.ctrmode==1,car_state_msg.enable==255,
            alarms,body_fresh,stopped,live);
        if(!packet.empty())send_packet(packet.data());
        driver_reason=!fresh?"feedback_stale":(!body_fresh?"body_feedback_stale":(!diag_fresh?"diagnostics_stale":(transaction.fault.empty()?transaction.phase:transaction.fault)));
        mode_latched=transaction.target;
        std_msgs::msg::String state;std::ostringstream o;
        o << "{\"schema_version\":2,\"actual_mode\":" << car_state_msg.turnmode
          << ",\"target_mode\":" << (transaction.target<0?car_state_msg.turnmode:transaction.target)
          << ",\"ready\":" << (transaction.ready?"true":"false")
          << ",\"phase\":" << quote(transaction.phase) << ",\"ready_level\":\"timed_open_loop\""
          << ",\"echo_age_s\":" << number(now-car_echo_at) << ",\"fault_reason\":" << quote(transaction.fault) << "}";
        state.data=o.str();mode_state_pub->publish(state);
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
        nh->declare_parameter<double>("wheel_feedback_scale",1.0);
        nh->declare_parameter<double>("angle_sign",-1.0);
        nh->declare_parameter<double>("spin_command_scale",0.429);
        nh->declare_parameter<double>("spin_sign",-1.0);
        nh->declare_parameter<double>("feedback_timeout",0.5);
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

        nh->get_parameter("wheel_feedback_scale",wheel_feedback_scale);nh->get_parameter("angle_sign",angle_sign);
        nh->get_parameter("feedback_timeout",transaction.feedback_timeout);
        nh->get_parameter("spin_command_scale",spin_command_scale);nh->get_parameter("spin_sign",spin_sign);
        if(!std::isfinite(spin_command_scale) || spin_command_scale<=0. || !std::isfinite(spin_sign) || std::abs(spin_sign)!=1. || !std::isfinite(wheel_feedback_scale) || wheel_feedback_scale<=0.)throw std::invalid_argument("Invalid mode unit conversion");
        transaction.wait_s=std::max(3.,(double)mode_change_time);
        mode_state_pub=nh->create_publisher<std_msgs::msg::String>("/car_driver/mode_state",10);
        body_sub=nh->create_subscription<std_msgs::msg::Float32MultiArray>("/rda/body_motion",5,[this](std_msgs::msg::Float32MultiArray::SharedPtr m){
            if(m->data.size()!=5)return;for(float v:m->data)if(!std::isfinite(v))return;
            body_speed=std::hypot(m->data[0],m->data[1]);body_yaw_rate=m->data[2];body_valid=m->data[3]>.5;body_source_age=std::max(0.,(double)m->data[4]);body_age.Clear();body_at=steady_ns()*1e-9;
        });
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

        // Acquisition and enable are serialized by ControlTick.  
    }
    
    void Shutdown() {
        char packet[100];std::snprintf(packet,sizeof(packet),"Car Run 0 %.2f",transaction.last_run_angle);send_packet(packet);
        usleep(80000);send_packet((char*)"Car Disable");usleep(80000);
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
                if(!echo_seen)continue;
                car_echo_at=steady_ns()*1e-9;
                car_state_msg.enable = atoi(substrs.at(1).c_str());
                car_state_msg.ctrmode = atoi(substrs.at(2).c_str());
                car_state_msg.turnmode = atoi(substrs.at(3).c_str());
                nodecheck->Find("node_rate")->Beat();
            }
            if (substrs.size()>=3 && substrs[0].size()==6 && substrs[0].substr(0,5)=="Wheel") 
            {
                int len = substrs[0].length();
                int id = atoi(substrs[0].substr(len - 1, len).c_str());
                
                // 修复：移除 resize，增加边界检查
                if(id >= 0 && id < (int)car_state_msg.speed.size()) {
                    car_state_msg.speed[id] = atof(substrs.at(2).c_str()) * wheel_feedback_scale;
                    wheel_seen[id]=numeric_token(substrs[2]) && std::isfinite(car_state_msg.speed[id]);wheel_age[id].Clear();
                }
            }
            if(substrs.size()>=3 && substrs[0].size()==5 && substrs[0].substr(0,4)=="Turn") {
                int id=substrs[0][4]-'0';
                if(id>=0 && id<4 && numeric_token(substrs[2])) {
                    turn_position[id]=atof(substrs[2].c_str());turn_seen[id]=std::abs(turn_position[id])>1.;turn_age[id].Clear();
                }
            }
            if (substrs.size()>=9 && substrs[0].find("errcode") != string::npos) 
            {
                bool valid=true;for(int k=0;k<8;++k)valid=valid && numeric_token(substrs[1+k]);
                if(!valid)continue;
                err_seen=true;err_age.Clear();error_at=steady_ns()*1e-9;
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
        ControlTick();
        drain_socket_diagnostics();
        publish_driver_diag();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    TCarCtr myCar;
    rclcpp::Rate rate(25);
    while (rclcpp::ok())
    {
        myCar.run();
        rclcpp::spin_some(myCar.nh);
        rate.sleep();
    }
    myCar.Shutdown();
    rclcpp::shutdown();
    return 0;
};
