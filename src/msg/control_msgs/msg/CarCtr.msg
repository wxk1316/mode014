int8 source  # 消息来源 0-遥控器  1-udp  2-can总线
int8 ctr_mode  #  1-程控  0-手动
int8 up_down_flag  #  0-上升  1-下降
int32 enable
int32 turnmode    #  0-阿克曼    1-斜行  2-横移模式  4-自转模式  x-差速模式  
float32 speed  # 上层模式0/1: m/s；模式4: rad/s，驱动转换为轮面m/s
float32 angle
int32 heartbeat