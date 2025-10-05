#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from std_msgs.msg import Header
from avoid_planner_pkg.msg import PolarHistogramMsg  # 已使用正确的包名
import math

# 全局变量存储最新的直方图数据
latest_histogram = None
lock = False
max_range_global = 0.01  # 初始最大距离，会根据实际数据更新

def callback(msg):
    """处理接收到的直方图消息，转换为360×12的二维数组（方位角×俯仰角）"""
    global latest_histogram, lock, max_range_global
    if not lock:
        # 将一维数据转换为360×12的二维数组（方位角数量×俯仰角数量）
        data = np.array(msg.data)
        data_2d = data.reshape((msg.num_azimuth_bins, msg.num_elevation_bins))  # 关键修改：360×12
        
        # 将inf替换为max_range以便更好地可视化
        data_2d[np.isinf(data_2d)] = msg.max_range
        
        # 更新全局最大距离
        max_range_global = msg.max_range
        
        # 存储完整的直方图信息
        latest_histogram = {
            'data': data_2d,  # 现在形状为(360, 12)
            'header': msg.header,
            'num_azimuth_bins': msg.num_azimuth_bins,
            'num_elevation_bins': msg.num_elevation_bins
        }

def update_plot(frame, ax, surf, cbar, text, elevations_deg, azimuths_rad):
    """更新三维极坐标图数据"""
    global latest_histogram, lock, max_range_global
    
    # 清除上一帧的图像但保留坐标轴设置
    ax.clear()
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    max_dim = max_range_global
    ax.set_xlim(-max_dim, max_dim)
    ax.set_ylim(-max_dim, max_dim)
    ax.set_zlim(-max_dim/2, max_dim/2)  # Z轴范围适当缩小
    
    if latest_histogram is None:
        text = ax.text(0, 0, 0, "等待接收直方图数据...", fontsize=12)
        return surf, cbar, text
    
    # 加锁防止数据更新冲突
    lock = True
    hist = latest_histogram
    lock = False
    
    # 获取数据（360×12格式：方位角×俯仰角）
    data = hist['data']
    
    # 创建网格（方位角×俯仰角）
    A, E = np.meshgrid(azimuths_rad, np.radians(elevations_deg))
    # 数据需要转置以匹配网格形状 (12, 360)
    R = data.T  # 转置为(12, 360)以匹配俯仰角×方位角的网格
    
    # 极坐标转换为笛卡尔坐标（x,y,z）
    X = R * np.cos(E) * np.cos(A)
    Y = R * np.cos(E) * np.sin(A)
    Z = R * np.sin(E)
    
    # 创建三维表面图
    surf = ax.plot_surface(X, Y, Z, 
                          cmap='viridis', 
                          linewidth=0.5, 
                          edgecolors='k',  # 边缘线颜色，增强立体感
                          vmin=0, 
                          vmax=max_range_global)
    
    # 更新颜色条范围（不重新创建）
    cbar.mappable.set_clim(0, max_range_global)
    cbar.mappable.set_array(R)
    
    # 添加标题和时间戳
    timestamp = hist['header'].stamp
    ax.set_title(f"三维极坐标直方图 (360×12格式) (时间戳: {timestamp.secs}.{timestamp.nsecs//1000000})")
    
    # 添加俯仰角标签
    for i, elev_deg in enumerate(elevations_deg):
        # 在边缘位置标注俯仰角
        ax.text(X[i, 0], Y[i, 0], Z[i, 0], f"{elev_deg}°", fontsize=8)
    
    return surf, cbar, text

def main():
    rospy.init_node('3d_polar_histogram_visualizer', anonymous=True)
    
    # 订阅指定的话题
    rospy.Subscriber('/polar_histogram', PolarHistogramMsg, callback)
    
    # 预计算角度数据（固定不变）
    num_azimuth_bins = 90
    num_elevation_bins = 10
    azimuths_deg = np.linspace(0, 360, num_azimuth_bins, endpoint=False)
    azimuths_rad = np.radians(azimuths_deg)
    elevations_deg = np.linspace(0, 50, num_elevation_bins, endpoint=True)  # -7到53度，步长5度
    
    # 创建3D图形
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 初始创建一个空的表面图用于初始化颜色条
    dummy_X, dummy_Y = np.meshgrid(azimuths_rad[:2], np.radians(elevations_deg[:2]))
    dummy_Z = np.zeros_like(dummy_X)
    surf = ax.plot_surface(dummy_X, dummy_Y, dummy_Z, cmap='viridis', vmin=0, vmax=50)
    
    # 创建一次颜色条并保持在图中
    cbar = fig.colorbar(surf, ax=ax, pad=0.1)
    cbar.set_label('障碍物距离 (m)')
    
    # 初始文本
    text = ax.text(0, 0, 0, "等待接收数据...", fontsize=12)
    
    # 设置初始坐标轴
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_xlim(-50, 50)
    ax.set_ylim(-50, 50)
    ax.set_zlim(-25, 25)
    
    # 创建动画
    ani = FuncAnimation(fig, update_plot, 
                       fargs=(ax, surf, cbar, text, elevations_deg, azimuths_rad), 
                       interval=200, blit=False)
    
    plt.tight_layout()
    plt.show()
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"可视化出错: {str(e)}")
    finally:
        plt.close('all')