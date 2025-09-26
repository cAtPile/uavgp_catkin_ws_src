#!/usr/bin/env python3
import rospy
import math  
from apoc_pkg.msg import detection_data
from geometry_msgs.msg import PoseStamped  # 导入位置消息类型

'''
向'/detection/data'发布
一个虚拟的目标点（1,1）相对无人机的位置信息
'''

# 全局变量存储当前位置和比例系数
current_x = 0.0
current_y = 0.0
vl_ratio = 1.0  # 比例系数，可根据需要调整

def pose_callback(pose_msg):
    """位置回调函数，更新无人机当前位置"""
    global current_x, current_y
    current_x = pose_msg.pose.position.x
    current_y = pose_msg.pose.position.y

def simple_publisher():
    rospy.init_node('circular_detection_pub')
    # 订阅无人机本地位置话题
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
    pub = rospy.Publisher('/detection/data', detection_data, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz发布频率
    
    # 虚拟目标参数配置
    fake_goal_x = 1.0
    fake_goal_y = 1.0
    
    # 初始化角度相关变量（如果不需要可以删除）
    angle = 0.0
    angle_step = 0.1  # 角度步长，控制变化速度
    
    while not rospy.is_shutdown():
        msg = detection_data()
        msg.detection_id = 1  # 强制有目标

        # 计算相对目标位置并乘以比例系数
        v_x = fake_goal_x - current_x
        v_y = fake_goal_y - current_y
        msg.detection_x = v_x * vl_ratio
        msg.detection_y = v_y * vl_ratio
        
        # 发布消息并打印日志
        pub.publish(msg)
        rospy.loginfo(f"有目标 | id=1, x={msg.detection_x:.1f}, y={msg.detection_y:.1f}")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        simple_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("发布器已停止")
