#!/usr/bin/env python3
import rospy
import math  # 用于计算圆坐标
from apoc_pkg.msg import detection_data

'''
向'/detection/data'持续发布
目标沿以(320, 320)为中心、半径21像素的圆运动
'''

def simple_publisher():
    rospy.init_node('circular_detection_pub')
    pub = rospy.Publisher('/detection/data', detection_data, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz发布频率
    
    # 圆参数配置
    center_x = 320.0       # 圆心X坐标
    center_y = 320.0       # 圆心Y坐标
    radius = 21.0          # 圆半径（像素）
    angle = 0.0            # 初始角度（弧度）
    angle_step = 0.1       # 每次循环的角度增量（控制移动速度，值越大越快）
    
    while not rospy.is_shutdown():
        msg = detection_data()
        msg.detection_id = 1  # 强制有目标
        
        # 计算圆上的坐标
        msg.detection_x = center_x + radius * math.cos(angle)
        msg.detection_y = center_y + radius * math.sin(angle)
        
        # 更新角度
        angle += angle_step
        if angle >= 2 * math.pi:
            angle = 0.0
        
        # 发布消息并打印日志
        pub.publish(msg)
        rospy.loginfo(f"有目标 | id=1, x={msg.detection_x:.1f}, y={msg.detection_y:.1f}")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        simple_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("发布器已停止")
