#pragma once
#include <deque>
#include <mutex>
#include <vector>
#include <string>
#include <chrono>
#include <cstdint>
#include <algorithm>

struct UdpDiagnosticPacket {
    uint64_t sequence=0;
    long long steady_ns=0,wall_ns=0;
    std::string direction,payload,peer_ip;
    int peer_port=0,result_bytes=0,error_code=0;
};

// Optional observer. It never changes the send result or legacy receive cache.
// Full queues discard their oldest diagnostic packet and count that loss.
class UdpDiagnosticTap {
    std::mutex mutex_;
    std::deque<UdpDiagnosticPacket> packets_;
    bool enabled_=false;
    uint64_t sequence_=0,dropped_=0;
    static constexpr size_t capacity_=256;
public:
    void enable(bool enabled){std::lock_guard<std::mutex> lock(mutex_);enabled_=enabled;}
    void push(const std::string& direction,const char* bytes,size_t length,const std::string& peer_ip,int peer_port,int result_bytes,int error_code){
        std::lock_guard<std::mutex> lock(mutex_);
        if(!enabled_)return;
        UdpDiagnosticPacket p;p.sequence=++sequence_;
        p.steady_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        p.wall_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        p.direction=direction;p.payload.assign(bytes,std::min(length,size_t(1024)));p.peer_ip=peer_ip;p.peer_port=peer_port;p.result_bytes=result_bytes;p.error_code=error_code;
        if(packets_.size()==capacity_){packets_.pop_front();++dropped_;}
        packets_.push_back(std::move(p));
    }
    std::vector<UdpDiagnosticPacket> take(size_t maximum=64){
        std::lock_guard<std::mutex> lock(mutex_);std::vector<UdpDiagnosticPacket> result;
        while(!packets_.empty() && result.size()<maximum){result.push_back(std::move(packets_.front()));packets_.pop_front();}
        return result;
    }
    uint64_t dropped(){std::lock_guard<std::mutex> lock(mutex_);return dropped_;}
};
