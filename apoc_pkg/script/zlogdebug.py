#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
import time
import os

class AltitudeLogger:
    def __init__(self):
        # 初始化数据存储列表
        self.timestamps = []
        self.altitudes = []
        
        # 记录开始时间，用于相对时间计算
        self.start_time = None
        
        # 创建日志文件
        self.log_file = open("zlog.txt", "w")
        
        # 订阅mavros的本地位置话题
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        
        # 初始化节点
        rospy.init_node('altitude_logger', anonymous=True)
        
        # 设置节点关闭时的回调函数
        rospy.on_shutdown(self.shutdown_callback)
        
        rospy.loginfo("Altitude logger started. Recording data...")
        rospy.loginfo("Press Ctrl+C to stop recording and generate plot.")

    def pose_callback(self, msg):
        # 获取当前时间（使用相对时间，从开始记录时算起）
        current_time = time.time()
        if self.start_time is None:
            self.start_time = current_time
        
        relative_time = current_time - self.start_time
        
        # 获取海拔高度（z轴位置）
        altitude = msg.pose.position.z
        
        # 存储数据
        self.timestamps.append(relative_time)
        self.altitudes.append(altitude)
        
        # 写入日志文件
        self.log_file.write(f"{relative_time:.6f}:{altitude:.6f} ")
        
        # 每10条数据刷新一次文件，确保数据及时写入
        if len(self.timestamps) % 10 == 0:
            self.log_file.flush()

    def shutdown_callback(self):
        # 关闭文件
        self.log_file.close()
        rospy.loginfo(f"Recording stopped. Data saved to {os.path.abspath('zlog.txt')}")
        
        # 生成图像
        if len(self.timestamps) > 0:
            self.generate_plot()
        else:
            rospy.logwarn("No data recorded, cannot generate plot.")

    def generate_plot(self):
        # 创建图像
        plt.figure(figsize=(10, 6))
        plt.plot(self.timestamps, self.altitudes, 'b-', linewidth=1.5)
        
        # 设置图像标题和轴标签
        plt.title('Drone Altitude vs Time', fontsize=14)
        plt.xlabel('Time (seconds)', fontsize=12)
        plt.ylabel('Altitude (meters)', fontsize=12)
        
        # 添加网格
        plt.grid(True, linestyle='--', alpha=0.7)
        
        # 调整布局
        plt.tight_layout()
        
        # 保存图像
        plt.savefig('altitude_vs_time.png', dpi=300)
        rospy.loginfo(f"Plot saved to {os.path.abspath('altitude_vs_time.png')}")
        
        # 显示图像
        plt.show()

if __name__ == '__main__':
    try:
        logger = AltitudeLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
