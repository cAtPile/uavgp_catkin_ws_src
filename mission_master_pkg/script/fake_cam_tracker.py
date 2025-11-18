#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

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
        return None, None
    
    # 原始计算
    raw_pix_x = rel_x / (0.003125 * drone_z)
    raw_pix_y = rel_y / (0.00265 * drone_z)
    
    # 应用最大值限制
    limited_pix_x = min(raw_pix_x, MAX_PIX_X)
    limited_pix_y = min(raw_pix_y, MAX_PIX_Y)
    
    # 记录限制状态（用于日志提示）
    x_limited = raw_pix_x > MAX_PIX_X
    y_limited = raw_pix_y > MAX_PIX_Y
    
    return limited_pix_x, limited_pix_y, x_limited, y_limited

def pose_callback(msg):
    """回调函数：计算相对位置、pix值（带限制）并输出"""
    # 提取无人机当前位置
    drone_x = msg.pose.position.x
    drone_y = msg.pose.position.y
    drone_z = msg.pose.position.z  # 当前高度（用于pix计算）

    # 计算相对位置
    rel_x, rel_y, rel_z = calculate_relative_position(
        drone_x, drone_y, drone_z,
        FIXED_OBJECT_X, FIXED_OBJECT_Y, FIXED_OBJECT_Z
    )

    # 计算pix_x和pix_y（带限制）
    pix_x, pix_y, x_limited, y_limited = calculate_pix(rel_x, rel_y, drone_z)

    # 打印所有信息
    rospy.loginfo("===== 位置与像素转换信息 =====")
    rospy.loginfo(f"固定物体位置 (map系): ({FIXED_OBJECT_X:.2f}, {FIXED_OBJECT_Y:.2f}, {FIXED_OBJECT_Z:.2f})")
    rospy.loginfo(f"无人机当前位置 (map系): ({drone_x:.4f}, {drone_y:.4f}, {drone_z:.4f})")
    rospy.loginfo(f"相对位置 (无人机-物体): (x: {rel_x:.4f}, y: {rel_y:.4f}, z: {rel_z:.4f})")
    
    if pix_x is not None and pix_y is not None:
        # 输出pix值及限制状态
        rospy.loginfo(f"pix_x（限制最大值{MAX_PIX_X}）: {pix_x:.2f} {'(已限制)' if x_limited else ''}")
        rospy.loginfo(f"pix_y（限制最大值{MAX_PIX_Y}）: {pix_y:.2f} {'(已限制)' if y_limited else ''}")
    rospy.loginfo("----------------------")

def main():
    rospy.init_node('drone_pix_with_limit', anonymous=True)
    
    try:
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
        rospy.loginfo(f"固定物体位置已设置为: ({FIXED_OBJECT_X}, {FIXED_OBJECT_Y}, {FIXED_OBJECT_Z})")
        rospy.loginfo(f"pix限制: 最大pix_x={MAX_PIX_X}, 最大pix_y={MAX_PIX_Y}")
        rospy.loginfo("开始计算相对位置及带限制的pix_x/pix_y...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.logwarn("节点被中断")
    except Exception as e:
        rospy.logerr(f"发生错误: {str(e)}")

if __name__ == '__main__':
    main()