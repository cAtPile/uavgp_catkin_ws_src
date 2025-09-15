# 单个检测目标的信息
std_msgs/Header header

# 目标基本信息
int32 id                    # 跟踪ID
string class_name           # 目标类别名称 (如 "person", "car")
float32 confidence          # 检测置信度 (0.0-1.0)

# 边界框信息 (XYXY格式数组)
float32[] xyxy              # 边界框 [x1, y1, x2, y2]

# 中心点信息
float32 center_x            # 中心点X坐标
float32 center_y            # 中心点Y坐标

# 尺寸信息
float32 width               # 宽度
float32 height              # 高度