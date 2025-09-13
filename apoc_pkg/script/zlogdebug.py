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
        
        # 3. 【关键修复】文件状态标记：避免“闭后写”
        self.log_file_open = False  # 初始为关闭状态，打开成功后设为True
        self.log_file = None        # 日志文件对象（初始为None）
        
        # 4. 创建日志文件（按要求路径和命名格式）
        self.log_dir = os.path.expanduser("~/catkin_ws/src/apoc_pkg/log")
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        self.timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file_path = os.path.join(self.log_dir, f"zlog_{self.timestamp_str}.txt")
        
        # 尝试打开日志文件（添加异常捕获，避免文件打开失败导致崩溃）
        try:
            self.log_file = open(self.log_file_path, "w", encoding="utf-8")
            self.log_file_open = True  # 打开成功，标记为True
        except Exception as e:
            rospy.logerr(f"Failed to open log file: {str(e)}")
            self.log_file_open = False
        
        # 5. 订阅两个目标话题（设置队列大小避免数据堆积）
        self.pose_sub = rospy.Subscriber(
            '/mavros/local_position/pose', 
            PoseStamped, 
            self.pose_callback,
            queue_size=10
        )
        self.setpoint_sub = rospy.Subscriber(
            '/mavros/setpoint_position/local', 
            PoseStamped, 
            self.setpoint_callback,
            queue_size=10
        )
        
        # 6. 初始化ROS节点
        rospy.init_node('altitude_logger', anonymous=True)
        
        # 7. 设置节点关闭回调（确保程序退出时安全关闭文件和生成图像）
        rospy.on_shutdown(self.shutdown_callback)
        
        # 打印初始化信息（根据文件状态调整）
        if self.log_file_open:
            rospy.loginfo("Altitude logger started. Recording data...")
            rospy.loginfo(f"Log file saved to: {self.log_file_path}")
        else:
            rospy.logwarn("Altitude logger started, but log file is unavailable (no data will be saved)")
        rospy.loginfo("Press Ctrl+C to stop recording and generate plot.")

    def pose_callback(self, msg):
        """回调函数：处理/mavros/local_position/pose话题，缓存最新z值"""
        # 【关键检查】如果文件已关闭或节点已开始关闭，直接返回（不处理数据）
        if not self.log_file_open:
            return
        self.last_pose_z = msg.pose.position.z
        self.sync_and_record_data()

    def setpoint_callback(self, msg):
        """回调函数：处理/mavros/setpoint_position/local话题，缓存最新z值"""
        # 【关键检查】如果文件已关闭或节点已开始关闭，直接返回（不处理数据）
        if not self.log_file_open:
            return
        self.last_setpoint_z = msg.pose.position.z
        self.sync_and_record_data()

    def sync_and_record_data(self):
        """同步两个话题的z值，仅当两者都有有效数据且文件打开时记录"""
        # 1. 先检查文件状态：关闭则直接返回
        if not self.log_file_open:
            rospy.logdebug_throttle(1, "Attempt to write to closed log file (ignored)")
            return
        
        # 2. 检查是否已获取到两个话题的初始数据
        if self.last_pose_z is None or self.last_setpoint_z is None:
            return
        
        # 3. 计算相对时间并存储数据
        current_time = time.time()
        if self.start_time is None:
            self.start_time = current_time
        relative_time = current_time - self.start_time
        
        self.timestamps.append(relative_time)
        self.pose_z.append(self.last_pose_z)
        self.setpoint_z.append(self.last_setpoint_z)
        
        # 4. 写入日志文件（添加异常捕获，避免写入失败导致崩溃）
        try:
            log_line = f"{relative_time:.6f}:{self.last_pose_z:.6f}_{self.last_setpoint_z:.6f}\n"
            self.log_file.write(log_line)
            # 每10条数据刷新一次文件（避免数据缓存丢失）
            if len(self.timestamps) % 10 == 0:
                self.log_file.flush()
        except Exception as e:
            rospy.logerr(f"Failed to write to log file: {str(e)}")
            self.log_file_open = False  # 写入失败后，标记文件为关闭状态

    def shutdown_callback(self):
        """节点关闭时的回调：安全关闭文件、生成图像（核心：确保只关闭一次）"""
        # 1. 【关键修复】仅当文件处于打开状态时才关闭（避免重复关闭）
        if self.log_file_open and self.log_file is not None:
            try:
                self.log_file.flush()  # 最后一次刷新缓存，确保数据写入
                self.log_file.close()  # 关闭文件
                rospy.loginfo(f"Recording stopped. Log saved to: {os.path.abspath(self.log_file_path)}")
            except Exception as e:
                rospy.logerr(f"Failed to close log file: {str(e)}")
            finally:
                self.log_file_open = False  # 无论是否成功关闭，都标记为关闭状态
                self.log_file = None        # 置空文件对象
        
        # 2. 取消订阅（避免关闭后仍接收消息触发回调）
        self.pose_sub.unregister()
        self.setpoint_sub.unregister()
        rospy.loginfo("ROS subscribers unregistered.")
        
        # 3. 生成对比图像（仅当有有效数据时）
        if len(self.timestamps) > 0:
            self.generate_comparison_plot()
        else:
            rospy.logwarn("No valid data recorded, cannot generate plot.")

    def generate_comparison_plot(self):
        """生成双z值对比图（按要求设置颜色、坐标轴、标题）"""
        try:
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
            
            # 设置图像样式
            plt.title('Drone Z-Position Comparison (Actual vs Setpoint)', fontsize=14, pad=15)
            plt.xlabel('Time (seconds)', fontsize=12, labelpad=10)
            plt.ylabel('Distance (meters)', fontsize=12, labelpad=10)  # 纵坐标按要求设置
            
            # 添加网格和图例
            plt.grid(True, linestyle='--', alpha=0.7)
            plt.legend(fontsize=10, loc='best')
            plt.tight_layout()  # 调整布局，避免标签被截断
            
            # 保存图像
            plot_file_path = os.path.join(self.log_dir, f"zlog_{self.timestamp_str}.png")
            plt.savefig(plot_file_path, bbox_inches='tight')
            rospy.loginfo(f"Comparison plot saved to: {os.path.abspath(plot_file_path)}")
            
            # 显示图像（可选，关闭图像后程序退出）
            plt.show()
        except Exception as e:
            rospy.logerr(f"Failed to generate comparison plot: {str(e)}")

if __name__ == '__main__':
    try:
        # 初始化日志器并阻塞等待回调
        logger = AltitudeLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        # 捕获ROS中断异常（如Ctrl+C），确保程序优雅退出
        pass
    except Exception as e:
        # 捕获其他未知异常，避免程序崩溃
        rospy.logerr(f"Altitude logger crashed: {str(e)}")