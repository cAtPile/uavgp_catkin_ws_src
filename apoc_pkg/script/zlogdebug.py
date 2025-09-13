#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
import time
import os
from datetime import datetime

class AltitudeLogger:
    def __init__(self):
        # 1. 初始化双话题数据存储列表（确保时间戳与z值一一对应）
        self.timestamps = []  # 统一时间戳（以第一个数据的时间为基准）
        self.pose_z = []      # /mavros/local_position/pose 的 z 值
        self.setpoint_z = []  # /mavros/setpoint_position/local 的 z 值
        
        # 2. 初始化话题数据缓存（解决双话题回调不同步问题）
        self.last_pose_z = None    # 缓存最新的 pose z 值
        self.last_setpoint_z = None# 缓存最新的 setpoint z 值
        self.start_time = None     # 记录开始时间（用于相对时间计算）
        
        # 3. 创建日志文件（按要求路径和命名格式：~/catkin_ws/src/apoc_pkg/log/zlog_{时间戳}）
        self.log_dir = os.path.expanduser("~/catkin_ws/src/apoc_pkg/log")
        # 检查日志目录是否存在，不存在则创建（避免路径错误）
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        # 生成时间戳（格式：年-月-日_时-分-秒，避免文件名特殊字符）
        self.timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        # 日志文件路径
        self.log_file_path = os.path.join(self.log_dir, f"zlog_{self.timestamp_str}.txt")
        # 打开日志文件（使用with语法更安全，此处因需持续写入暂用open）
        self.log_file = open(self.log_file_path, "w", encoding="utf-8")
        
        # 4. 订阅两个目标话题（设置队列大小避免数据堆积）
        rospy.Subscriber(
            '/mavros/local_position/pose', 
            PoseStamped, 
            self.pose_callback,
            queue_size=10
        )
        rospy.Subscriber(
            '/mavros/setpoint_position/local', 
            PoseStamped, 
            self.setpoint_callback,
            queue_size=10
        )
        
        # 5. 初始化ROS节点
        rospy.init_node('altitude_logger', anonymous=True)
        
        # 6. 设置节点关闭回调（确保程序退出时保存数据和图像）
        rospy.on_shutdown(self.shutdown_callback)
        
        rospy.loginfo("Altitude logger started. Recording data...")
        rospy.loginfo(f"Log file saved to: {self.log_file_path}")
        rospy.loginfo("Press Ctrl+C to stop recording and generate plot.")

    def pose_callback(self, msg):
        """回调函数：处理/mavros/local_position/pose话题，缓存最新z值"""
        self.last_pose_z = msg.pose.position.z
        # 有新数据时尝试同步记录（确保两个z值对应同一时间点）
        self.sync_and_record_data()

    def setpoint_callback(self, msg):
        """回调函数：处理/mavros/setpoint_position/local话题，缓存最新z值"""
        self.last_setpoint_z = msg.pose.position.z
        # 有新数据时尝试同步记录
        self.sync_and_record_data()

    def sync_and_record_data(self):
        """同步两个话题的z值，仅当两者都有有效数据时记录"""
        # 检查是否已获取到两个话题的初始数据
        if self.last_pose_z is None or self.last_setpoint_z is None:
            return
        
        # 初始化开始时间（以第一次有效记录的时间为基准）
        current_time = time.time()
        if self.start_time is None:
            self.start_time = current_time
        # 计算相对时间（相对于开始记录的时间，单位：秒）
        relative_time = current_time - self.start_time
        
        # 存储数据（确保时间戳与两个z值一一对应）
        self.timestamps.append(relative_time)
        self.pose_z.append(self.last_pose_z)
        self.setpoint_z.append(self.last_setpoint_z)
        
        # 按要求格式写入日志文件：{时间戳}：{pose.z}_{setpoint.z}
        log_line = f"{relative_time:.6f}:{self.last_pose_z:.6f}_{self.last_setpoint_z:.6f}\n"
        self.log_file.write(log_line)
        
        # 每10条数据刷新一次文件（避免数据缓存导致丢失）
        if len(self.timestamps) % 10 == 0:
            self.log_file.flush()

    def shutdown_callback(self):
        """节点关闭时的回调：关闭文件、生成图像"""
        # 安全关闭日志文件
        self.log_file.close()
        rospy.loginfo(f"Recording stopped. Log saved to: {os.path.abspath(self.log_file_path)}")
        
        # 生成对比图像（仅当有有效数据时）
        if len(self.timestamps) > 0:
            self.generate_comparison_plot()
        else:
            rospy.logwarn("No valid data recorded, cannot generate plot.")

    def generate_comparison_plot(self):
        """生成双z值对比图（按要求设置颜色、坐标轴、标题）"""
        # 创建图像（设置尺寸为10x6，分辨率300dpi）
        plt.figure(figsize=(10, 6), dpi=300)
        
        # 绘制两条曲线：蓝色=pose.z，红色=setpoint.z
        plt.plot(
            self.timestamps, self.pose_z, 
            'b-', linewidth=1.5, label='/mavros/local_position/pose.z'
        )
        plt.plot(
            self.timestamps, self.setpoint_z, 
            'r-', linewidth=1.5, label='/mavros/setpoint_position/local.z'
        )
        
        # 设置图像样式（按要求配置标题、坐标轴标签）
        plt.title('Drone Z-Position Comparison (Actual vs Setpoint)', fontsize=14, pad=15)
        plt.xlabel('Time (seconds)', fontsize=12, labelpad=10)
        plt.ylabel('Distance (meters)', fontsize=12, labelpad=10)  # 纵坐标按要求改为“距离（m）”
        
        # 添加网格（虚线、半透明，提升可读性）
        plt.grid(True, linestyle='--', alpha=0.7)
        # 添加图例（区分两条曲线）
        plt.legend(fontsize=10, loc='best')
        # 调整布局（避免标签被截断）
        plt.tight_layout()
        
        # 按要求保存图像（路径与日志文件一致，命名格式相同）
        plot_file_path = os.path.join(self.log_dir, f"zlog_{self.timestamp_str}.png")
        plt.savefig(plot_file_path, bbox_inches='tight')  # bbox_inches避免图例被截断
        rospy.loginfo(f"Comparison plot saved to: {os.path.abspath(plot_file_path)}")
        
        # 显示图像（可选，关闭图像后程序才会退出）
        plt.show()

if __name__ == '__main__':
    try:
        # 初始化日志器并阻塞等待回调
        logger = AltitudeLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        # 捕获ROS中断异常（如Ctrl+C），确保程序优雅退出
        pass