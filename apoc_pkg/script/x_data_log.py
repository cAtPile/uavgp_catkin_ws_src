#!/usr/bin/env python3


import rospy
import time
import os
from geometry_msgs.msg import PointStamped

class PositionLogger:
    def __init__(self):
        # 初始化节点
        rospy.init_node('position_logger', anonymous=True)
        
        # 存储最新的位置数据
        self.local_position_x = None
        self.setpoint_position_x = None
        
        # 订阅相关话题
        rospy.Subscriber('/mavros/local_position/pose', PointStamped, self.local_position_callback)
        rospy.Subscriber('/mavros/setpoint_position/local', PointStamped, self.setpoint_position_callback)
        
        # 创建日志目录（如果不存在）
        self.log_dir = os.path.expanduser('~/catkin_ws/src/apoc_pkg/log/')
        os.makedirs(self.log_dir, exist_ok=True)
        
        # 获取当前时间用于生成文件名
        self.create_log_file()
        
        # 设置循环频率
        self.rate = rospy.Rate(10)  # 10Hz
        
    def local_position_callback(self, msg):
        """处理本地位置消息"""
        self.local_position_x = msg.point.x
        
    def setpoint_position_callback(self, msg):
        """处理设定点位置消息"""
        self.setpoint_position_x = msg.point.x
        
    def create_log_file(self):
        """根据当前时间创建日志文件"""
        current_time = time.localtime()
        # 文件名格式: 日_时分.txt (例如: 2_1920.txt)
        filename = f"{current_time.tm_mday}_{current_time.tm_hour:02d}{current_time.tm_min:02d}.txt"
        self.file_path = os.path.join(self.log_dir, filename)
        
        # 打开文件准备写入
        self.log_file = open(self.file_path, 'a')
        rospy.loginfo(f"日志文件已创建: {self.file_path}")
        
    def log_data(self):
        """记录数据到文件"""
        if self.local_position_x is not None and self.setpoint_position_x is not None:
            # 获取当前时间（自纪元以来的秒数，带小数）
            current_time = time.time()
            
            # 格式化数据: 时间-local_position-setpoint_position
            log_line = f"{current_time:.4f}-{self.local_position_x:.5f}-{self.setpoint_position_x:.5f}\n"
            
            # 写入文件并刷新
            self.log_file.write(log_line)
            self.log_file.flush()
            
    def run(self):
        """主循环"""
        try:
            while not rospy.is_shutdown():
                self.log_data()
                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass
        finally:
            # 关闭文件
            self.log_file.close()
            rospy.loginfo("日志文件已关闭")

if __name__ == '__main__':
    try:
        logger = PositionLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass

'''
记录
'/mavros/local_position/pose.position.x'
'/mavros/setpoint_position/local.position.x'

数据格式为
时间-local_position-setpoint_position
例如：12.3454-1.32332-1.55531

文件名称为
日_时分.txt
例如：2_1920.txt(某月2日19:20)

位置
~/catkin_ws/src/apoc_pkg/log/
'''