#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

# 固定物体在map坐标系中的位置（请修改为实际坐标）
FIXED_OBJECT_X = 5.0   # 物体X坐标
FIXED_OBJECT_Y = 3.0   # 物体Y坐标
FIXED_OBJECT_Z = 0.0   # 物体Z坐标（固定为0）

def calculate_relative_position(drone_x, drone_y, drone_z, obj_x, obj_y, obj_z):
    """计算无人机相对于固定物体的位置"""
    rel_x = drone_x - obj_x
    rel_y = drone_y - obj_y
    rel_z = drone_z - obj_z
    return rel_x, rel_y, rel_z

def calculate_pix(rel_x, rel_y, drone_z):
    """计算pix_x和pix_y，包含除以零保护"""
    # 避免无人机高度为0导致的除法错误（设置一个极小值作为阈值）
    min_z = 0.001
    if abs(drone_z) < min_z:
        rospy.logwarn("无人机高度接近0，无法计算pix_x/pix_y（避免除以零）")
        return None, None
    
    # 按公式计算
    pix_x = rel_x / (0.003125 * drone_z)
    pix_y = rel_y / (0.00265 * drone_z)
    return pix_x, pix_y

def pose_callback(msg):
    """回调函数：计算相对位置和pix值并输出"""
    # 提取无人机当前位置
    drone_x = msg.pose.position.x
    drone_y = msg.pose.position.y
    drone_z = msg.pose.position.z  # 当前高度（用于pix计算）

    # 计算相对位置
    rel_x, rel_y, rel_z = calculate_relative_position(
        drone_x, drone_y, drone_z,
        FIXED_OBJECT_X, FIXED_OBJECT_Y, FIXED_OBJECT_Z
    )

    # 计算pix_x和pix_y
    pix_x, pix_y = calculate_pix(rel_x, rel_y, drone_z)

    # 打印所有信息
    rospy.loginfo("===== 位置与像素转换信息 =====")
    rospy.loginfo(f"固定物体位置 (map系): ({FIXED_OBJECT_X:.2f}, {FIXED_OBJECT_Y:.2f}, {FIXED_OBJECT_Z:.2f})")
    rospy.loginfo(f"无人机当前位置 (map系): ({drone_x:.4f}, {drone_y:.4f}, {drone_z:.4f})")
    rospy.loginfo(f"相对位置 (无人机-物体): (x: {rel_x:.4f}, y: {rel_y:.4f}, z: {rel_z:.4f})")
    
    if pix_x is not None and pix_y is not None:
        rospy.loginfo(f"pix_x = rel_x / (0.003125 * z) = {pix_x:.2f}")
        rospy.loginfo(f"pix_y = rel_y / (0.00265 * z) = {pix_y:.2f}")
    rospy.loginfo("----------------------")

def main():
    rospy.init_node('drone_pix_calculator', anonymous=True)
    
    try:
        # 订阅无人机位置话题（需确保与物体在同一坐标系）
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
        
        rospy.loginfo(f"固定物体位置已设置为: ({FIXED_OBJECT_X}, {FIXED_OBJECT_Y}, {FIXED_OBJECT_Z})")
        rospy.loginfo("开始计算相对位置及pix_x/pix_y...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.logwarn("节点被中断")
    except Exception as e:
        rospy.logerr(f"发生错误: {str(e)}")

if __name__ == '__main__':
    main()