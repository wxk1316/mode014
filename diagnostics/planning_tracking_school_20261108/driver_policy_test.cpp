#include "../../src/control/car_ctr/src/mode_transaction.hpp"
#include <cassert>
#include <iostream>
int main(){
 ModeTransaction p;double echo=-1.;double first_run=-1.;int actual=0;
 for(int i=0;i<220;++i){double t=i*.04;
  auto out=p.next(t,1,.3,90.,actual,true,true,true,false,true,true,true);
  if(out=="Car SteerModeCtrl 1"){assert(echo<0);echo=t+.08;}
  if(echo>=0 && t>=echo)actual=1;
  if(p.ready && first_run<0){first_run=t;assert(t-echo>=3.);}
 }
 assert(first_run>0 && p.target<0);
 auto stopped=p.next(10.,1,.3,90.,1,false,true,true,false,true,true,true);
 assert(stopped.find("Car Run 0.000") == 0 && !p.ready);
 ModeTransaction no_stop;
 for(int i=0;i<100;++i){auto out=no_stop.next(i*.04,4,.4,0,0,true,true,true,false,false,false,true);assert(out.find("SteerModeCtrl")==std::string::npos);}
 ModeTransaction bad;auto out=bad.next(0,3,1,0,0,true,true,true,false,true,true,true);assert(!bad.ready && out.find("Car Run 0.000")==0);
 std::cout<<"driver policy PASS: timed switch, one mode request, stale stop, unknown motion, whitelist\n";
}
