#pragma once
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

// Pure deterministic policy, monotonic seconds. The caller supplies actual
// body-motion evidence, not commanded wheel speed. No ROS or UDP dependencies.
struct ModeTransaction {
    double wait_s=3.,feedback_timeout=.5,deadline_s=12.;
    int target=-1;std::string phase="READY",fault;
    double started=0.,stopped_at=-1.,zero_at=0.,echo_at=-1.,sent_at=-10.,last_tx=-10.;
    double old_angle=0.,last_run_angle=0.;int retries=0;
    bool ready=false;
    static bool legal(int m){return m==0 || m==1 || m==4;}
    std::string run(double v,double a){char b[100];std::snprintf(b,sizeof(b),"Car Run %.3f %.2f",v,a);return b;}
    std::string next(double now,int desired,double velocity,double angle,int actual,
                     bool feedback_fresh,bool control,bool enabled,bool alarms,
                     bool body_valid,bool stopped,bool command_live) {
        if(now-last_tx<.039 || (phase=="ECHO" && now-last_tx<.08))return "";
        ready=false;
        if(!legal(desired) || !std::isfinite(velocity) || !std::isfinite(angle)) {fault="invalid_command";phase="FAULT";}
        if(alarms){fault="axis_alarm";phase="FAULT";}
        if(!feedback_fresh || !control || !enabled || !body_valid || !command_live || !fault.empty()) {
            stopped_at=-1.;if(phase=="RESET")phase="BRAKE";
            last_tx=now;return run(0.,last_run_angle);
        }
        if(body_valid && stopped){if(stopped_at<0)stopped_at=now;}else stopped_at=-1.;
        if(phase=="FAULT") {last_tx=now;return run(0.,last_run_angle);} // explicit re-arm required
        if(target<0 && actual!=desired){target=desired;started=now;phase="BRAKE";stopped_at=-1.;old_angle=last_run_angle;retries=0;echo_at=-1.;}
        if(target>=0 && now-started>deadline_s){phase="FAULT";fault="switch_timeout";last_tx=now;return run(0.,last_run_angle);}
        if(phase=="BRAKE") {
            if(body_valid && stopped){if(stopped_at<0)stopped_at=now;}else stopped_at=-1.;
            if(stopped_at>=0 && now-stopped_at>=.3){phase="RESET";zero_at=now;}
            last_tx=now;return run(0.,old_angle);
        }
        if(phase=="RESET") {
            if(!stopped){phase="BRAKE";stopped_at=-1.;old_angle=0.;last_tx=now;return run(0.,0.);}
            if(now-zero_at>=std::abs(old_angle)/30.+.2){phase="SEND";}
            last_tx=now;return run(0.,0.);
        }
        if(phase=="SEND" && !stopped){phase="BRAKE";stopped_at=-1.;old_angle=0.;last_tx=now;return run(0.,0.);}
        if(phase=="SEND" || phase=="ECHO") {
            if(phase=="ECHO" && actual==target){phase="WAIT_MECHANICAL";echo_at=now;}
            else if(now-last_tx>=.08 && (phase=="SEND" || now-sent_at>=.5)) {
                if(retries>=4){fault="echo_timeout";phase="FAULT";last_tx=now;return run(0.,0.);}
                phase="ECHO";sent_at=now;last_tx=now;++retries;
                return "Car SteerModeCtrl "+std::to_string(target);
            } else return ""; // reserve mode-command slot; do not overwrite with Run
        }
        if(phase=="WAIT_MECHANICAL") {
            if(actual!=target){fault="mode_echo_changed";phase="FAULT";last_tx=now;return run(0.,0.);}
            if(now-last_tx<.08)return ""; // protect last distinct command
            if(now-echo_at>=std::max(3.,wait_s) && body_valid && stopped && stopped_at>=0 && now-stopped_at>=.3){phase="READY";target=-1;last_run_angle=angle;}
            else {last_tx=now;return run(0.,angle);}
        }
        if(phase=="READY") {
            if(actual!=desired){last_tx=now;return run(0.,last_run_angle);}
            ready=true;last_run_angle=angle;last_tx=now;return run(velocity,angle);
        }
        last_tx=now;return run(0.,last_run_angle);
    }
    void rearm(){fault.clear();phase="READY";target=-1;ready=false;stopped_at=-1.;}
};
