#!/usr/bin/env python3
import rospy
import random  # 导入随机数模块，用于生成波动
from apoc_pkg.msg import detection_data

def simple_publisher():
    # 1. 初始化ROS节点
    rospy.init_node('simple_detection_pub')
    
    # 2. 创建话题发布者
    pub = rospy.Publisher('/detection/data', detection_data, queue_size=10)
    
    # 3. 配置参数：目标中心 + 波动范围（可按需调整）
    TARGET_X = 320.0       # 目标基准X坐标
    TARGET_Y = 320.0       # 目标基准Y坐标
    FLUCTUATION = 150.0     # 波动范围（±20像素，模拟检测误差，值越大波动越明显）
    PUB_RATE = 10          # 发布频率（10Hz，与原脚本一致）
    rate = rospy.Rate(PUB_RATE)
    
    rospy.loginfo(f"=== 简单波动发布器启动 ===")
    rospy.loginfo(f"目标中心: ({TARGET_X}, {TARGET_Y}), 波动范围: ±{FLUCTUATION}像素")

    # 4. 循环发布消息（带波动）
    while not rospy.is_shutdown():
        msg = detection_data()
        msg.detection_id = 1  # 强制有目标
        
        # 核心：生成带波动的坐标（基准值 + 随机波动）
        # random.uniform(a, b) 生成 [a, b] 之间的随机浮点数
        msg.detection_x = TARGET_X + random.uniform(-FLUCTUATION, FLUCTUATION)
        msg.detection_y = TARGET_Y + random.uniform(-FLUCTUATION, FLUCTUATION)
        
        # 发布消息 + 打印日志（保留1位小数，避免日志冗余）
        pub.publish(msg)
        rospy.loginfo(f"有目标 | id=1, x={msg.detection_x:.1f}, y={msg.detection_y:.1f}")
        
        # 按设定频率休眠（确保10Hz稳定发布）
        rate.sleep()

if __name__ == '__main__':
    try:
        simple_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("=== 简单波动发布器停止 ===")