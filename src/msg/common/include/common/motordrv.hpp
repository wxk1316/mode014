#ifndef MOTOR_DRV_HPP
#define MOTOR_DRV_HPP

#include "common/can.hpp"

struct DrvInfo
{
    float pos = 0, pos_abs = 0;
    float spd = 0;
    float ia = 0;
    float bv = 0;
    uint enable = 0;
    uint digit_in = 0;
    uint digit_out = 0;
    uint wrn = 0;  
    uint mode = 0;
    uint src = 0;
    uint id = 0;
};


class TMotorDrv: public Thread
{
private:
    TCan *can;
    bool ctr_value_flag=false;
    
    int canMsgProc()
    {
        int r=0;
        for(size_t n=0; n<can->rec_frames.size(); n++)
        {
            can_frame *frame=&(can->rec_frames[n]);
            if((frame->can_id&0x0F)!=client_id+1) continue;

            int frame_id=frame->can_id&0xFF0;
            if(frame_id==0x640)
            {
                r++;
                drv_info.wrn = frame->data[1] * 256 + frame->data[0];
                drv_info.digit_in = frame->data[3];
                drv_info.digit_out = frame->data[4];
                drv_info.mode = frame->data[5];
                drv_info.src = frame->data[6];
                drv_info.bv = frame->data[7]*2;
                drv_info.enable = frame->data[2];
            }
            else if(frame_id==0x440)
            {
                r++;

                float spd_temp=0.0, ia_temp=0.0;

                ia_temp = (frame->data[4]*256+frame->data[5])*0.01-50.0;
                spd_temp = frame->data[6]*256+frame->data[7]-32000;

                if( fabs(ia_temp - 100.0) <= 0.5 && fabs(spd_temp - 100.0) < 1.0 )
                { 
                    char tmpbuf[4];
                    for (int i = 0; i < 4; ++i) tmpbuf[i]=frame->data[3-i];
                    memcpy(&drv_info.pos_abs,&tmpbuf,4);
                }
                else
                { 
                    drv_info.ia=(frame->data[4]*256+frame->data[5])*0.01-50.0;
                    drv_info.spd=frame->data[6]*256+frame->data[7]-32000;
                    char tmpbuf[4];
                    for (int i = 0; i < 4; ++i) tmpbuf[i]=frame->data[3-i];
                    memcpy(&drv_info.pos,&tmpbuf,4);
                }
            }
            frame->can_id=0;
        }
        return r;
    };

    
public:
    DrvInfo drv_info;
    int pre_drv_enable;
    uint client_id;
    int heartbeat=0;

    int channel;
    float giv_value;
    float pos,speed,ia;
    int active;
    uint wrn,wrn_flag=1;

    float ctr_value=0;

    TMotorDrv(uint clt_id, TCan *c, int mode=3)
    {
        can=c;
        can->send_frame.can_dlc=8;
        client_id=clt_id;

        Monitor(true);  
        start();

        Enable(false);
        usleep(2000000);
       
        ClrFlt();
        RunMode(mode);
    };

    void Enable(bool v)
    {
        if(v)  
        {
            Unlock(v);
            ClrFlt();
        }    

        can->send_frame.data[0]='3';
        can->send_frame.data[1]='2';
        can->send_frame.data[4]=v;
        can->send_frame.can_id=0x401+client_id;
        can->Send();
        usleep(5000);

        if(!v)  Unlock(v);
    };

    void Monitor(bool v)
    {
        can->send_frame.data[0]='8';
        can->send_frame.data[1]='0';
        can->send_frame.data[4]=v;
        can->send_frame.can_id=0x401+client_id;
        can->Send();
        usleep(5000);
    };

    int RunMode(uint run_mode)
    {
        if (drv_info.enable) 
        {
            printf("enable!, set mode is not ok!\n");
            return 0;
        }   

        can->send_frame.data[0] = '3';
        can->send_frame.data[1] = '0';
        can->send_frame.data[4] = run_mode;
        can->send_frame.can_id=0x401+client_id;

        int flag=0;
        for(int i=0; i<1000; i++)
        {
            can->Send();
            if(drv_info.mode==run_mode)
            {
                flag=1;
                break;
            }
            usleep(1000);
        }    

        if(flag==0)  printf("set mode error!\n");
        
        return 1;
    };

    void Unlock(bool v)  
    {
        can->send_frame.data[0]='7';
        can->send_frame.data[1]='0';
        can->send_frame.data[4]=v;
        can->send_frame.can_id=0x401+client_id;
        can->Send();
        usleep(1000);
    };

    int SetZero()
    {
        if(drv_info.enable) return 0;

        can->send_frame.data[0]='1';
        can->send_frame.data[1]='A';
        can->send_frame.can_id=0x401+client_id;
        can->Send();
        return 1;
    };

    void ClrFlt()
    {
        if (drv_info.enable) return;
        can->send_frame.data[0]='5';
        can->send_frame.data[1]='4';
        can->send_frame.can_id=0x401+client_id;
        can->Send();
        usleep(5000);
    };

    void SetCtrValue(float value)
    {
        ctr_value_flag=true;
        ctr_value=value;
        Moving(value);
    }

    void Moving(float val)
    {
        if(!drv_info.enable) return;

        char tmpbuf[4];
        memcpy(&tmpbuf,&val,4);

        can->send_frame.data[0]='5';
        can->send_frame.data[1]='2';
        for (int i = 0; i < 4; ++i) can->send_frame.data[7-i] = tmpbuf[i];
        can->send_frame.can_id=0x401+client_id;
        can->Send();
    };

    void run()
    {
        TTimer tim;
        int tmp_beat=0;

        while(1)
        {
            usleep(500);
            tmp_beat+=canMsgProc();

            if(tim.GetValue()>1)
            {
                heartbeat=tmp_beat/tim.GetValue();
                tmp_beat=0;
                tim.Clear();
            }

            pos = drv_info.pos;
            speed = drv_info.spd;
            ia = drv_info.ia;
            active = drv_info.enable;
            wrn = drv_info.wrn;
        }
    };

};


#endif
