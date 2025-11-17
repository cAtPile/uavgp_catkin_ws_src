# CamTrack.msg
# Cam Tracker Detection Message
# 用于发布ball和car的检测结果

std_msgs/Header header      # 时间戳、坐标系等信息

bool system_ok              # 系统健康状态（True=正常运行，False=检测异常）

# Ball检测信息（抓取区使用）
int32 ball_num              # 检测到的ball数量
int32 ball_id
float32 ball_x              # 最适合抓取的ball的x坐标
float32 ball_y              # 最适合抓取的ball的y坐标
float32 ball_dis            # 到ball的距离（预留接口，当前输出0）

# Car检测信息（释放区使用）
int32 car_num               # 检测到的car数量
int32[] car_ids
float32[] car_x             # 所有car的x坐标数组
float32[] car_y             # 所有car的y坐标数组
float32[] car_dis           # 到car的距离数组（预留接口，当前输出0）


# 夹爪状态（预留接口）
bool in_gripper             # 夹爪是否抓取到球（当前输出False）