#include "../../src/control/car_ctr/src/mode_transaction.hpp"
#include <cassert>
#include <iostream>
int main(){
 for(int source:{0,1,4})for(int destination:{0,1,4})if(source!=destination){
  ModeTransaction p;p.last_run_angle=source==1?90.:source==0?30.:0.;int actual=source;double echo=-1,first=-1;int sends=0;
  for(int i=0;i<700;++i){double t=i*.04;if(echo>=0 && t>=echo)actual=destination;
   auto packet=p.next(t,destination,.3,destination==1?90.:0.,actual,true,true,true,false,true,true,true);
   if(packet=="Car SteerModeCtrl "+std::to_string(destination)){assert(sends++==0);echo=t+.08;}
   if(p.ready && first<0){first=t;assert(echo>=0 && t-echo>=3.);}
  }
  assert(first>0 && sends==1);std::cout<<source<<"->"<<destination<<" PASS first_ready="<<first<<" echo="<<echo<<"\n";
 }
 ModeTransaction p;
 p.next(0,1,.3,90,0,true,true,true,false,true,true,true);
 p.next(.32,1,.3,90,0,true,true,true,false,true,true,true);
 for(double t:{.4,.64,.8}){auto packet=p.next(t,1,.3,90,0,true,true,true,false,true,false,true);assert(packet.find("SteerModeCtrl")==std::string::npos);}
 std::cout<<"renewed-motion-stop-hold PASS\n";
}
