#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

# 固定物体在map坐标系中的位置（请根据实际位置修改这三个值）
FIXED_OBJECT_X = 5.0   # 物体X坐标
FIXED_OBJECT_Y = 3.0   # 物体Y坐标
FIXED_OBJECT_Z = 0.0   # 物体Z坐标（用户指定为0）

def calculate_relative_position(drone_x, drone_y, drone_z, obj_x, obj_y, obj_z):
    """计算无人机相对于固定物体的位置（无人机位置 - 物体位置）"""
    rel_x = drone_x - obj_x
    rel_y = drone_y - obj_y
    rel_z = drone_z - obj_z
    return rel_x, rel_y, rel_z

def pose_callback(msg):
    """回调函数：接收无人机位置，计算相对位置并打印"""
    # 提取无人机位置
    drone_x = msg.pose.position.x
    drone_y = msg.pose.position.y
    drone_z = msg.pose.position.z

    # 计算相对位置
    rel_x, rel_y, rel_z = calculate_relative_position(
        drone_x, drone_y, drone_z,
        FIXED_OBJECT_X, FIXED_OBJECT_Y, FIXED_OBJECT_Z
    )

    # 打印日志
    rospy.loginfo("===== 位置信息 =====")
    rospy.loginfo(f"固定物体位置 (map系): ({FIXED_OBJECT_X:.2f}, {FIXED_OBJECT_Y:.2f}, {FIXED_OBJECT_Z:.2f})")
    rospy.loginfo(f"无人机当前位置 (map系): ({drone_x:.4f}, {drone_y:.4f}, {drone_z:.4f})")
    rospy.loginfo(f"无人机相对物体位置: (x: {rel_x:.4f}, y: {rel_y:.4f}, z: {rel_z:.4f})")
    rospy.loginfo("----------------------")

def main():
    # 初始化节点
    rospy.init_node('drone_relative_position', anonymous=True)
    
    try:
        # 订阅无人机位置话题（需确保与固定物体在同一坐标系，如map）
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
        
        rospy.loginfo(f"开始监听无人机位置，固定物体位置已设置为: ({FIXED_OBJECT_X}, {FIXED_OBJECT_Y}, {FIXED_OBJECT_Z})")
        rospy.loginfo("相对位置定义：无人机位置 - 固定物体位置")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.logwarn("节点被中断")
    except Exception as e:
        rospy.logerr(f"发生错误: {str(e)}")

if __name__ == '__main__':
    main()