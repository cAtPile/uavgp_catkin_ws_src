# CamTrack.msg

std_msgs/Header header   # 时间戳、坐标系等信息，备用。
bool system_ok           # 系统健康状态（True=正常运行，False=检测异常）

int32 ball_num #检测到的ball数量 下面的xy坐标是节点决策后最适合抓取的哪个球的坐标 所以只输出一个坐标
float32 ball_x
float32 ball_y
float32 ball_dis #到ball的距离，由d455测量，留作后续开发的接口，现在都输出0

int32 car_num #检测到的car数量 下面的
float32[] car_x
float32[] car_y
float32[] car_dis #到car的距离，由d455测量，留作后续开发的接口，现在都输出0

bool in_gripper #用于检测夹爪是否成功抓取，True为夹爪有球，False为夹爪无球
