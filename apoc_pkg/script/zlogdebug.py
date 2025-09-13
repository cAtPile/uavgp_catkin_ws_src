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
        # 1. 初始化数据存储列表（仅保留pose的时间戳和Z值）
        self.timestamps = []  # 相对时间戳（从首次记录开始计时）
        self.pose_z = []      # /mavros/local_position/pose 的 Z 轴数据
        
        # 2. 时间基准初始化（用于计算相对时间）
        self.start_time = None
        
        # 3. 文件状态标记（避免“闭后写”错误，保留原鲁棒性设计）
        self.log_file_open = False
        self.log_file = None
        
        # 4. 日志文件路径配置（保持原路径格式，文件名增加“pose_only”区分）
        self.log_dir = os.path.expanduser("~/catkin_ws/src/apoc_pkg/log")
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        self.timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file_path = os.path.join(self.log_dir, f"zlog_pose_only_{self.timestamp_str}.txt")
        
        # 5. 尝试打开日志文件（保留异常捕获）
        try:
            self.log_file = open(self.log_file_path, "w", encoding="utf-8")
            self.log_file_open = True
        except Exception as e:
            rospy.logerr(f"Failed to open log file: {str(e)}")
            self.log_file_open = False
        
        # 6. 仅订阅 /mavros/local_position/pose 话题（删除setpoint订阅）
        self.pose_sub = rospy.Subscriber(
            '/mavros/local_position/pose', 
            PoseStamped, 
            self.pose_callback,
            queue_size=10  # 队列大小10，避免数据堆积
        )
        
        # 7. ROS节点初始化与关闭回调
        rospy.init_node('altitude_pose_logger', anonymous=True)  # 节点名改为“pose专用”
        rospy.on_shutdown(self.shutdown_callback)
        
        # 8. 初始化信息打印（适配单话题逻辑）
        if self.log_file_open:
            rospy.loginfo("Altitude Pose Logger started (only record /mavros/local_position/pose)")
            rospy.loginfo(f"Log file saved to: {self.log_file_path}")
        else:
            rospy.logwarn("Altitude Pose Logger started, but log file is unavailable (no data will be saved)")
        rospy.loginfo("Press Ctrl+C to stop recording and generate plot.")

    def pose_callback(self, msg):
        """仅处理pose话题回调：直接记录Z值（无需同步）"""
        # 检查文件状态，关闭则不处理
        if not self.log_file_open:
            return
        
        # 获取当前pose的Z值
        current_pose_z = msg.pose.position.z
        # 记录数据（无需缓存，直接处理）
        self.record_pose_data(current_pose_z)

    def record_pose_data(self, current_z):
        """单话题数据记录函数（替代原sync_and_record_data，简化逻辑）"""
        # 1. 计算相对时间（首次记录时初始化时间基准）
        current_time = time.time()
        if self.start_time is None:
            self.start_time = current_time
        relative_time = current_time - self.start_time
        
        # 2. 存储数据到列表（用于后续绘图）
        self.timestamps.append(relative_time)
        self.pose_z.append(current_z)
        
        # 3. 写入日志文件（格式简化为“相对时间:Z值”）
        try:
            log_line = f"{relative_time:.6f}:{current_z:.6f}\n"  # 仅保留时间和pose.z
            self.log_file.write(log_line)
            # 每10条数据刷新一次缓存（避免数据丢失）
            if len(self.timestamps) % 10 == 0:
                self.log_file.flush()
        except Exception as e:
            rospy.logerr(f"Failed to write to log file: {str(e)}")
            self.log_file_open = False  # 写入失败则标记文件关闭

    def shutdown_callback(self):
        """节点关闭逻辑（简化：仅处理pose订阅和文件）"""
        # 1. 安全关闭日志文件
        if self.log_file_open and self.log_file is not None:
            try:
                self.log_file.flush()
                self.log_file.close()
                rospy.loginfo(f"Recording stopped. Log saved to: {os.path.abspath(self.log_file_path)}")
            except Exception as e:
                rospy.logerr(f"Failed to close log file: {str(e)}")
            finally:
                self.log_file_open = False
                self.log_file = None
        
        # 2. 取消pose话题订阅（删除setpoint订阅取消逻辑）
        self.pose_sub.unregister()
        rospy.loginfo("ROS pose subscriber unregistered.")
        
        # 3. 生成仅包含pose的Z轴曲线图
        if len(self.timestamps) > 0:
            self.generate_pose_plot()
        else:
            rospy.logwarn("No valid pose data recorded, cannot generate plot.")

    def generate_pose_plot(self):
        """生成仅pose Z轴的曲线图（简化标题、图例）"""
        try:
            # 图像配置（保持10x6尺寸和300dpi分辨率）
            plt.figure(figsize=(10, 6), dpi=300)
            
            # 仅绘制pose Z轴曲线（蓝色实线，线宽1.5）
            plt.plot(
                self.timestamps, self.pose_z, 
                'b-', linewidth=1.5, label='/mavros/local_position/pose.z'
            )
            
            # 图像样式调整（适配单曲线逻辑）
            plt.title('Drone Z-Position (Only /mavros/local_position/pose)', fontsize=14, pad=15)
            plt.xlabel('Time (seconds)', fontsize=12, labelpad=10)
            plt.ylabel('Altitude (meters)', fontsize=12, labelpad=10)  # 纵坐标改为“高度”更贴切
            
            # 网格和图例（图例仅显示pose曲线）
            plt.grid(True, linestyle='--', alpha=0.7)
            plt.legend(fontsize=10, loc='best')
            plt.tight_layout()  # 避免标签被截断
            
            # 保存图像（文件名与日志对应，增加“pose_only”）
            plot_file_path = os.path.join(self.log_dir, f"zlog_pose_only_{self.timestamp_str}.png")
            plt.savefig(plot_file_path, bbox_inches='tight')
            rospy.loginfo(f"Pose Z-axis plot saved to: {os.path.abspath(plot_file_path)}")
            
            # 显示图像（可选，关闭后程序退出）
            plt.show()
        except Exception as e:
            rospy.logerr(f"Failed to generate pose Z-axis plot: {str(e)}")

if __name__ == '__main__':
    try:
        # 初始化日志器并阻塞运行
        logger = AltitudeLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Altitude Pose Logger crashed: {str(e)}")