'''
用于trace功能调整
记录mavros/setpoint_position/local.x
和mavros/local_position/pose.x
到~/catkin_ws/src/apoc_pkg/log/trace_x_log_<时间戳用来区分>.txt
格式为
时间-mavros/setpoint_position/local.x-mavros/local_position/pose.x
例如
21.0767-1.0032-2.0045
并作一张图
到到~/catkin_ws/src/apoc_pkg/log/trace_x_log_<时间戳用来区分>.png
横轴时间
纵轴距离
红色曲线mavros/setpoint_position/local.x
蓝色曲线mavros/local_position/pose.x
'''
#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
import time
import os
from datetime import datetime

class XTraceLogger:
    def __init__(self):
        # 初始化数据存储列表
        self.timestamps = []  # 相对时间戳
        self.setpoint_x = []  # 设定位置X值
        self.pose_x = []      # 实际位置X值
        
        # 数据缓存，用于处理不同步的回调
        self.last_setpoint_x = None
        self.last_pose_x = None
        self.start_time = None
        
        # 设置日志目录
        self.log_dir = os.path.expanduser("~/catkin_ws/src/apoc_pkg/log")
        # 确保日志目录存在
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        # 生成时间戳用于文件名
        self.timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 日志文件路径
        self.log_file_path = os.path.join(
            self.log_dir, 
            f"trace_x_log_{self.timestamp_str}.txt"
        )
        
        # 打开日志文件
        self.log_file = open(self.log_file_path, "w")
        
        # 订阅相关话题
        rospy.Subscriber(
            '/mavros/setpoint_position/local', 
            PoseStamped, 
            self.setpoint_callback,
            queue_size=10
        )
        rospy.Subscriber(
            '/mavros/local_position/pose', 
            PoseStamped, 
            self.pose_callback,
            queue_size=10
        )
        
        # 初始化节点
        rospy.init_node('x_trace_logger', anonymous=True)
        
        # 设置关闭回调
        rospy.on_shutdown(self.shutdown_callback)
        
        rospy.loginfo("X trace logger started. Recording data...")
        rospy.loginfo(f"Log file: {self.log_file_path}")
        rospy.loginfo("Press Ctrl+C to stop recording and generate plot.")

    def setpoint_callback(self, msg):
        """处理设定位置话题的回调函数"""
        self.last_setpoint_x = msg.pose.position.x
        self.sync_and_record()

    def pose_callback(self, msg):
        """处理实际位置话题的回调函数"""
        self.last_pose_x = msg.pose.position.x
        self.sync_and_record()

    def sync_and_record(self):
        """同步并记录数据，只有当两个话题都有数据时才记录"""
        # 检查是否已经获取到两个话题的数据
        if self.last_setpoint_x is None or self.last_pose_x is None:
            return
        
        # 初始化开始时间
        current_time = time.time()
        if self.start_time is None:
            self.start_time = current_time
        
        # 计算相对时间
        relative_time = current_time - self.start_time
        
        # 存储数据
        self.timestamps.append(relative_time)
        self.setpoint_x.append(self.last_setpoint_x)
        self.pose_x.append(self.last_pose_x)
        
        # 写入日志文件，格式：时间-setpoint.x-pose.x
        log_line = f"{relative_time:.4f}-{self.last_setpoint_x:.4f}-{self.last_pose_x:.4f}\n"
        self.log_file.write(log_line)
        
        # 每10条数据刷新一次文件
        if len(self.timestamps) % 10 == 0:
            self.log_file.flush()

    def shutdown_callback(self):
        """节点关闭时的处理函数"""
        # 关闭日志文件
        self.log_file.close()
        rospy.loginfo(f"Recording stopped. Log saved to: {self.log_file_path}")
        
        # 生成图表
        if len(self.timestamps) > 0:
            self.generate_plot()
        else:
            rospy.logwarn("No data recorded, cannot generate plot.")

    def generate_plot(self):
        """生成对比图表"""
        # 创建图表
        plt.figure(figsize=(10, 6))
        
        # 绘制曲线：红色为设定值，蓝色为实际值
        plt.plot(self.timestamps, self.setpoint_x, 'r-', linewidth=1.5, 
                 label='mavros/setpoint_position/local.x')
        plt.plot(self.timestamps, self.pose_x, 'b-', linewidth=1.5, 
                 label='mavros/local_position/pose.x')
        
        # 设置图表属性
        plt.title('X Position Trace: Setpoint vs Actual', fontsize=14)
        plt.xlabel('Time (seconds)', fontsize=12)
        plt.ylabel('Distance (m)', fontsize=12)
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend()
        plt.tight_layout()
        
        # 保存图表
        plot_file_path = os.path.join(
            self.log_dir, 
            f"trace_x_log_{self.timestamp_str}.png"
        )
        plt.savefig(plot_file_path, dpi=300)
        rospy.loginfo(f"Plot saved to: {plot_file_path}")
        
        # 显示图表
        plt.show()

if __name__ == '__main__':
    try:
        logger = XTraceLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
