#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
# 导入自定义消息类型（请替换为实际的包名，例如your_package/CamTrack）
from your_package.msg import CamTrack  # 需根据实际包名修改


# 固定物体在map坐标系中的位置（请修改为实际坐标）
FIXED_OBJECT_X = 5.0   # 物体X坐标
FIXED_OBJECT_Y = 3.0   # 物体Y坐标
FIXED_OBJECT_Z = 0.0   # 物体Z坐标（固定为0）

# pix最大限制值
MAX_PIX_X = 640
MAX_PIX_Y = 320


def calculate_relative_position(drone_x, drone_y, drone_z, obj_x, obj_y, obj_z):
    """计算无人机相对于固定物体的位置"""
    rel_x = drone_x - obj_x
    rel_y = drone_y - obj_y
    rel_z = drone_z - obj_z
    return rel_x, rel_y, rel_z


def calculate_pix(rel_x, rel_y, drone_z):
    """计算pix_x和pix_y，包含除以零保护和最大值限制"""
    min_z = 0.001  # 最小高度阈值，避免除以零
    if abs(drone_z) < min_z:
        rospy.logwarn("无人机高度接近0，无法计算pix_x/pix_y（避免除以零）")
        return None, None, False, False  # 后两个为限制状态
    
    # 原始计算
    raw_pix_x = rel_x / (0.003125 * drone_z)
    raw_pix_y = rel_y / (0.00265 * drone_z)
    
    # 应用最大值限制
    limited_pix_x = min(raw_pix_x, MAX_PIX_X)
    limited_pix_y = min(raw_pix_y, MAX_PIX_Y)
    
    # 记录限制状态
    x_limited = raw_pix_x > MAX_PIX_X
    y_limited = raw_pix_y > MAX_PIX_Y
    
    return limited_pix_x, limited_pix_y, x_limited, y_limited


def create_cam_track_msg(pix_x, pix_y):
    """创建并填充CamTrack消息"""
    cam_msg = CamTrack()
    
    # 填充header（时间戳为当前时间，frame_id可根据实际情况设置）
    cam_msg.header.stamp = rospy.Time.now()
    cam_msg.header.frame_id = ""  # 可根据需要设置坐标系（如"map"）
    
    # 系统状态：正常运行
    cam_msg.system_ok = True
    
    # Ball检测信息（核心：填充pix_x和pix_y）
    cam_msg.ball_num = 1  # 固定为1
    cam_msg.ball_id = 0   # 预留ID，可根据需要修改
    cam_msg.ball_x = pix_x if pix_x is not None else 0.0  # 异常时设为0
    cam_msg.ball_y = pix_y if pix_y is not None else 0.0
    cam_msg.ball_dis = 0.0  # 预留接口，固定为0
    
    # Car检测信息（预留，暂设为无检测）
    cam_msg.car_num = 0
    cam_msg.car_ids = []
    cam_msg.car_x = []
    cam_msg.car_y = []
    cam_msg.car_dis = []
    
    # 夹爪状态（预留，固定为False）
    cam_msg.in_gripper = False
    
    return cam_msg


def pose_callback(msg, pub):
    """回调函数：计算pix值并发布CamTrack消息"""
    # 提取无人机位置
    drone_x = msg.pose.position.x
    drone_y = msg.pose.position.y
    drone_z = msg.pose.position.z

    # 计算相对位置
    rel_x, rel_y, rel_z = calculate_relative_position(
        drone_x, drone_y, drone_z,
        FIXED_OBJECT_X, FIXED_OBJECT_Y, FIXED_OBJECT_Z
    )

    # 计算pix_x和pix_y（带限制）
    pix_x, pix_y, x_limited, y_limited = calculate_pix(rel_x, rel_y, drone_z)

    # 打印信息
    rospy.loginfo("===== 位置与像素转换信息 =====")
    rospy.loginfo(f"无人机位置: ({drone_x:.4f}, {drone_y:.4f}, {drone_z:.4f})")
    rospy.loginfo(f"相对位置: (x: {rel_x:.4f}, y: {rel_y:.4f})")
    
    if pix_x is not None and pix_y is not None:
        rospy.loginfo(f"pix_x（限制{MAX_PIX_X}）: {pix_x:.2f} {'(已限制)' if x_limited else ''}")
        rospy.loginfo(f"pix_y（限制{MAX_PIX_Y}）: {pix_y:.2f} {'(已限制)' if y_limited else ''}")
    else:
        rospy.loginfo("pix_x/pix_y计算异常（将发布默认值0）")
    rospy.loginfo("----------------------")

    # 创建并发布CamTrack消息
    cam_msg = create_cam_track_msg(pix_x, pix_y)
    pub.publish(cam_msg)


def main():
    # 初始化节点
    rospy.init_node('drone_cam_tracker_publisher', anonymous=True)
    
    try:
        # 创建CamTrack消息发布者
        cam_pub = rospy.Publisher('/cam_tracker/info', CamTrack, queue_size=10)
        
        # 订阅无人机位置话题（将发布者作为参数传入回调函数）
        rospy.Subscriber(
            '/mavros/local_position/pose', 
            PoseStamped, 
            lambda msg: pose_callback(msg, cam_pub)  # 匿名函数传递发布者
        )
        
        rospy.loginfo(f"固定物体位置: ({FIXED_OBJECT_X}, {FIXED_OBJECT_Y}, {FIXED_OBJECT_Z})")
        rospy.loginfo(f"pix限制: 最大pix_x={MAX_PIX_X}, 最大pix_y={MAX_PIX_Y}")
        rospy.loginfo("开始计算并发布/cam_tracker/info话题...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.logwarn("节点被中断")
    except Exception as e:
        rospy.logerr(f"发生错误: {str(e)}")


if __name__ == '__main__':
    main()