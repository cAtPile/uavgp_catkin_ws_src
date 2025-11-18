#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
# 导入自定义消息类型（替换为实际包名）
from your_package.msg import CamTrack


# 固定物体在map坐标系中的位置（请修改为实际坐标）
FIXED_OBJECT_X = 5.0   # 物体X坐标
FIXED_OBJECT_Y = 3.0   # 物体Y坐标
FIXED_OBJECT_Z = 0.0   # 物体Z坐标（固定为0）

# pix绝对值限制范围
MIN_PIX_X = -640
MAX_PIX_X = 640
MIN_PIX_Y = -320
MAX_PIX_Y = 320


def calculate_relative_position(drone_x, drone_y, drone_z, obj_x, obj_y, obj_z):
    """计算无人机相对于固定物体的位置"""
    rel_x = drone_x - obj_x
    rel_y = drone_y - obj_y
    rel_z = drone_z - obj_z
    return rel_x, rel_y, rel_z


def calculate_pix(rel_x, rel_y, drone_z):
    """计算pix_x和pix_y，包含除以零保护和绝对值限制（[-640,640]和[-320,320]）"""
    min_z = 0.001  # 最小高度阈值，避免除以零
    if abs(drone_z) < min_z:
        rospy.logwarn("无人机高度接近0，无法计算pix_x/pix_y（避免除以零）")
        return None, None, None, None  # 后两个为限制状态描述
    
    # 原始计算
    raw_pix_x = rel_x / (0.003125 * drone_z)
    raw_pix_y = rel_y / (0.00265 * drone_z)
    
    # 应用绝对值限制（同时限制上下界）
    limited_pix_x = max(MIN_PIX_X, min(raw_pix_x, MAX_PIX_X))  # 限制在[-640, 640]
    limited_pix_y = max(MIN_PIX_Y, min(raw_pix_y, MAX_PIX_Y))  # 限制在[-320, 320]
    
    # 确定限制状态（超过最大值/小于最小值/未限制）
    if raw_pix_x > MAX_PIX_X:
        x_limit_status = f"(超过最大值{MAX_PIX_X}，已限制)"
    elif raw_pix_x < MIN_PIX_X:
        x_limit_status = f"(小于最小值{MIN_PIX_X}，已限制)"
    else:
        x_limit_status = "(未限制)"
    
    if raw_pix_y > MAX_PIX_Y:
        y_limit_status = f"(超过最大值{MAX_PIX_Y}，已限制)"
    elif raw_pix_y < MIN_PIX_Y:
        y_limit_status = f"(小于最小值{MIN_PIX_Y}，已限制)"
    else:
        y_limit_status = "(未限制)"
    
    return limited_pix_x, limited_pix_y, x_limit_status, y_limit_status


def create_cam_track_msg(pix_x, pix_y):
    """创建并填充CamTrack消息"""
    cam_msg = CamTrack()
    
    # 填充header
    cam_msg.header.stamp = rospy.Time.now()
    cam_msg.header.frame_id = ""  # 可根据需要设置坐标系
    
    # 系统状态
    cam_msg.system_ok = True
    
    # Ball检测信息（核心：填充限制后的pix_x和pix_y）
    cam_msg.ball_num = 1  # 固定为1
    cam_msg.ball_id = 0   # 预留ID
    cam_msg.ball_x = pix_x if pix_x is not None else 0.0  # 异常时设为0
    cam_msg.ball_y = pix_y if pix_y is not None else 0.0
    cam_msg.ball_dis = 0.0  # 预留接口
    
    # Car检测信息（预留）
    cam_msg.car_num = 0
    cam_msg.car_ids = []
    cam_msg.car_x = []
    cam_msg.car_y = []
    cam_msg.car_dis = []
    
    # 夹爪状态
    cam_msg.in_gripper = False
    
    return cam_msg


def pose_callback(msg, pub):
    """回调函数：计算pix值（带绝对值限制）并发布消息"""
    # 提取无人机位置
    drone_x = msg.pose.position.x
    drone_y = msg.pose.position.y
    drone_z = msg.pose.position.z

    # 计算相对位置
    rel_x, rel_y, rel_z = calculate_relative_position(
        drone_x, drone_y, drone_z,
        FIXED_OBJECT_X, FIXED_OBJECT_Y, FIXED_OBJECT_Z
    )

    # 计算pix_x和pix_y（带绝对值限制）
    pix_x, pix_y, x_status, y_status = calculate_pix(rel_x, rel_y, drone_z)

    # 打印信息
    rospy.loginfo("===== 位置与像素转换信息 =====")
    rospy.loginfo(f"无人机位置: ({drone_x:.4f}, {drone_y:.4f}, {drone_z:.4f})")
    rospy.loginfo(f"相对位置: (x: {rel_x:.4f}, y: {rel_y:.4f})")
    
    if pix_x is not None and pix_y is not None:
        rospy.loginfo(f"pix_x（范围{MIN_PIX_X}~{MAX_PIX_X}）: {pix_x:.2f} {x_status}")
        rospy.loginfo(f"pix_y（范围{MIN_PIX_Y}~{MAX_PIX_Y}）: {pix_y:.2f} {y_status}")
    else:
        rospy.loginfo("pix_x/pix_y计算异常（将发布默认值0）")
    rospy.loginfo("----------------------")

    # 创建并发布消息
    cam_msg = create_cam_track_msg(pix_x, pix_y)
    pub.publish(cam_msg)


def main():
    rospy.init_node('drone_cam_tracker_with_abs_limit', anonymous=True)
    
    try:
        # 创建发布者
        cam_pub = rospy.Publisher('/cam_tracker/info', CamTrack, queue_size=10)
        
        # 订阅无人机位置话题
        rospy.Subscriber(
            '/mavros/local_position/pose', 
            PoseStamped, 
            lambda msg: pose_callback(msg, cam_pub)
        )
        
        rospy.loginfo(f"固定物体位置: ({FIXED_OBJECT_X}, {FIXED_OBJECT_Y}, {FIXED_OBJECT_Z})")
        rospy.loginfo(f"pix限制范围: pix_x={MIN_PIX_X}~{MAX_PIX_X}, pix_y={MIN_PIX_Y}~{MAX_PIX_Y}")
        rospy.loginfo("开始计算并发布/cam_tracker/info话题...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.logwarn("节点被中断")
    except Exception as e:
        rospy.logerr(f"发生错误: {str(e)}")


if __name__ == '__main__':
    main()