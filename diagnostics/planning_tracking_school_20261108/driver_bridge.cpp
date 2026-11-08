#include "../../src/control/car_ctr/src/mode_transaction.hpp"
extern "C" {
void* mode_create(){return new ModeTransaction;}
void mode_destroy(void* p){delete static_cast<ModeTransaction*>(p);}
const char* mode_tick(void* raw,double now,int desired,double v,double angle,int actual,int fresh,int body_valid,int stopped){
 static thread_local std::string out;
 out=static_cast<ModeTransaction*>(raw)->next(now,desired,v,angle,actual,fresh,true,true,false,body_valid,stopped,true);return out.c_str();
}
int mode_ready(void* p){return static_cast<ModeTransaction*>(p)->ready;}
const char* mode_phase(void* p){return static_cast<ModeTransaction*>(p)->phase.c_str();}
}
