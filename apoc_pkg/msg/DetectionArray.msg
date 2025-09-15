# 多个检测目标的集合
std_msgs/Header header

# 检测目标数组
Detection[] detections

# 图像信息
int32 image_width           # 图像宽度
int32 image_height          # 图像高度
int32 total_objects         # 总检测目标数量
float32 processing_time     # 处理时间(秒)