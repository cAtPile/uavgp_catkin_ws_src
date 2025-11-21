#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import math
# 导入自定义消息类型（替换为实际包名）
from mission_master_pkg.msg import CamTrack


# 固定物体在map坐标系中的位置（请修改为实际坐标）
FIXED_OBJECT_X = 5.0   # 物体X坐标
FIXED_OBJECT_Y = 3.0   # 物体Y坐标
FIXED_OBJECT_Z = 0.0   # 物体Z坐标（固定为0）

# pix绝对值限制范围
MIN_PIX_X = -640
MAX_PIX_X = 640
MIN_PIX_Y = -320
MAX_PIX_Y = 320

# Car运动参数
# Car0: 匀速直线运动
CAR0_START_X = 2.0
CAR0_START_Y = 2.0
CAR0_VEL_X = 0.5  # m/s
CAR0_VEL_Y = 0.3  # m/s

# Car1: 匀变速运动
CAR1_START_X = 8.0
CAR1_START_Y = 4.0
CAR1_VEL_X = 0.2  # 初始速度 m/s
CAR1_VEL_Y = 0.1
CAR1_ACC_X = 0.1  # 加速度 m/s^2
CAR1_ACC_Y = 0.05

# Car2: 圆周运动
CAR2_CENTER_X = 6.0
CAR2_CENTER_Y = 6.0
CAR2_RADIUS = 2.0
CAR2_ANGULAR_VEL = 0.5  # rad/s

# 全局变量记录起始时间
start_time = None


def calculate_relative_position(drone_x, drone_y, drone_z, obj_x, obj_y, obj_z):
    """计算无人机相对于固定物体的位置"""
    rel_x = drone_x - obj_x
    rel_y = drone_y - obj_y
    rel_z = drone_z - obj_z
    return rel_x, rel_y, rel_z


def get_car_positions(t):
    """计算三辆car在时刻t的位置
    
    Args:
        t: 从开始的时间 (秒)
    
    Returns:
        list of tuples: [(car0_x, car0_y), (car1_x, car1_y), (car2_x, car2_y)]
    """
    # Car0: 匀速直线运动
    car0_x = CAR0_START_X + CAR0_VEL_X * t
    car0_y = CAR0_START_Y + CAR0_VEL_Y * t
    
    # Car1: 匀变速运动
    car1_x = CAR1_START_X + CAR1_VEL_X * t + 0.5 * CAR1_ACC_X * t * t
    car1_y = CAR1_START_Y + CAR1_VEL_Y * t + 0.5 * CAR1_ACC_Y * t * t
    
    # Car2: 圆周运动
    angle = CAR2_ANGULAR_VEL * t
    car2_x = CAR2_CENTER_X + CAR2_RADIUS * math.cos(angle)
    car2_y = CAR2_CENTER_Y + CAR2_RADIUS * math.sin(angle)
    
    return [(car0_x, car0_y), (car1_x, car1_y), (car2_x, car2_y)]


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


def create_cam_track_msg(pix_x, pix_y, car_pix_data):
    """创建并填充CamTrack消息
    
    Args:
        pix_x: ball的pix_x
        pix_y: ball的pix_y
        car_pix_data: list of tuples [(car_id, car_pix_x, car_pix_y), ...]
    """
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
    
    # Car检测信息
    cam_msg.car_num = len(car_pix_data)
    cam_msg.car_ids = [data[0] for data in car_pix_data]
    cam_msg.car_x = [data[1] if data[1] is not None else 0.0 for data in car_pix_data]
    cam_msg.car_y = [data[2] if data[2] is not None else 0.0 for data in car_pix_data]
    cam_msg.car_dis = [0.0] * len(car_pix_data)  # 预留接口
    
    # 夹爪状态
    cam_msg.in_gripper = False
    
    return cam_msg


def pose_callback(msg, pub):
    """回调函数：计算pix值（带绝对值限制）并发布消息"""
    global start_time
    
    # 初始化起始时间
    if start_time is None:
        start_time = rospy.Time.now()
    
    # 计算经过的时间
    current_time = rospy.Time.now()
    elapsed_time = (current_time - start_time).to_sec()
    
    # 提取无人机位置
    drone_x = msg.pose.position.x
    drone_y = msg.pose.position.y
    drone_z = msg.pose.position.z

    # 计算ball的相对位置和pix值
    rel_x, rel_y, rel_z = calculate_relative_position(
        drone_x, drone_y, drone_z,
        FIXED_OBJECT_X, FIXED_OBJECT_Y, FIXED_OBJECT_Z
    )

    # 计算pix_x和pix_y（带绝对值限制）
    pix_x, pix_y, x_status, y_status = calculate_pix(rel_x, rel_y, drone_z)

    # 获取car位置并计算pix值
    car_positions = get_car_positions(elapsed_time)
    car_pix_data = []
    
    for car_id, (car_x, car_y) in enumerate(car_positions):
        # 计算car相对于无人机的位置
        car_rel_x, car_rel_y, _ = calculate_relative_position(
            drone_x, drone_y, drone_z,
            car_x, car_y, 0.0  # car的Z坐标固定为0
        )
        
        # 计算car的pix值
        car_pix_x, car_pix_y, _, _ = calculate_pix(car_rel_x, car_rel_y, drone_z)
        car_pix_data.append((car_id, car_pix_x, car_pix_y))

    # 打印信息
    rospy.loginfo("===== 位置与像素转换信息 =====")
    rospy.loginfo(f"时间: {elapsed_time:.2f}s")
    rospy.loginfo(f"无人机位置: ({drone_x:.4f}, {drone_y:.4f}, {drone_z:.4f})")
    rospy.loginfo(f"Ball相对位置: (x: {rel_x:.4f}, y: {rel_y:.4f})")
    
    if pix_x is not None and pix_y is not None:
        rospy.loginfo(f"Ball pix_x（范围{MIN_PIX_X}~{MAX_PIX_X}）: {pix_x:.2f} {x_status}")
        rospy.loginfo(f"Ball pix_y（范围{MIN_PIX_Y}~{MAX_PIX_Y}）: {pix_y:.2f} {y_status}")
    else:
        rospy.loginfo("Ball pix_x/pix_y计算异常（将发布默认值0）")
    
    # 打印car信息
    for i, (car_x, car_y) in enumerate(car_positions):
        car_pix_x = car_pix_data[i][1]
        car_pix_y = car_pix_data[i][2]
        rospy.loginfo(f"Car{i}位置: ({car_x:.4f}, {car_y:.4f}), pix: ({car_pix_x:.2f if car_pix_x else 'N/A'}, {car_pix_y:.2f if car_pix_y else 'N/A'})")
    
    rospy.loginfo("----------------------")

    # 创建并发布消息
    cam_msg = create_cam_track_msg(pix_x, pix_y, car_pix_data)
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
        
        rospy.loginfo(f"固定物体(Ball)位置: ({FIXED_OBJECT_X}, {FIXED_OBJECT_Y}, {FIXED_OBJECT_Z})")
        rospy.loginfo(f"pix限制范围: pix_x={MIN_PIX_X}~{MAX_PIX_X}, pix_y={MIN_PIX_Y}~{MAX_PIX_Y}")
        rospy.loginfo("Car运动模式:")
        rospy.loginfo(f"  Car0: 匀速直线运动, 起点({CAR0_START_X}, {CAR0_START_Y}), 速度({CAR0_VEL_X}, {CAR0_VEL_Y}) m/s")
        rospy.loginfo(f"  Car1: 匀变速运动, 起点({CAR1_START_X}, {CAR1_START_Y}), 初速({CAR1_VEL_X}, {CAR1_VEL_Y}) m/s, 加速度({CAR1_ACC_X}, {CAR1_ACC_Y}) m/s^2")
        rospy.loginfo(f"  Car2: 圆周运动, 圆心({CAR2_CENTER_X}, {CAR2_CENTER_Y}), 半径{CAR2_RADIUS}m, 角速度{CAR2_ANGULAR_VEL} rad/s")
        rospy.loginfo("开始计算并发布/cam_tracker/info话题...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.logwarn("节点被中断")
    except Exception as e:
        rospy.logerr(f"发生错误: {str(e)}")


if __name__ == '__main__':
    main()