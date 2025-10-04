#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from std_msgs.msg import Header
from avoid_planner_pkg.msg import PolarHistogramMsg  # 已使用正确的包名
import math
'''

1.
订阅:/polar_histogram
/polar_histogram的消息类型是:
/*
std_msgs/Header header
float64 azimuth_resolution
float64 elevation_resolution
float64 min_azimuth
float64 max_azimuth
float64 min_elevation
float64 max_elevation
float64 max_range
uint32 num_azimuth_bins
uint32 num_elevation_bins
float64[] data
*/

2.
其中data[] 需要转换成12*360的数组
12是俯仰角的-7,-2,3,8,13.....53
从-7到53步长5度
360是方位角
0,1,2,3,4....,360
从0到360步长1
存储的数值的距离极中心的距离

3.
请帮我构建一个三维极坐标系
将有数值的面填充
例如data[10][120]=3
在俯仰角为43 方位角为120处最近的障碍物为3m

'''


# 全局变量存储最新的直方图数据
latest_histogram = None
lock = False

def callback(msg):
    """处理接收到的直方图消息，转换为12×360的二维数组"""
    global latest_histogram, lock
    if not lock:
        # 将一维数据转换为12×360的二维数组（俯仰角×方位角）
        data = np.array(msg.data)
        # 注意：根据消息定义重塑为(俯仰角数量, 方位角数量)
        data_2d = data.reshape((msg.num_elevation_bins, msg.num_azimuth_bins))
        
        # 将inf替换为max_range以便更好地可视化
        data_2d[np.isinf(data_2d)] = msg.max_range
        
        # 存储完整的直方图信息
        latest_histogram = {
            'data': data_2d,
            'azimuth_resolution': msg.azimuth_resolution,
            'elevation_resolution': msg.elevation_resolution,
            'min_azimuth': msg.min_azimuth,
            'max_azimuth': msg.max_azimuth,
            'min_elevation': msg.min_elevation,
            'max_elevation': msg.max_elevation,
            'max_range': msg.max_range,
            'num_azimuth_bins': msg.num_azimuth_bins,
            'num_elevation_bins': msg.num_elevation_bins,
            'header': msg.header
        }

def update_plot(frame, ax, surf, cbar, text):
    """更新三维极坐标图数据"""
    global latest_histogram, lock
    
    # 清除上一帧的图像
    ax.clear()
    
    if latest_histogram is None:
        text = ax.text(0, 0, 0, "等待接收直方图数据...", fontsize=12)
        return surf, cbar, text
    
    # 加锁防止数据更新冲突
    lock = True
    hist = latest_histogram
    lock = False
    
    # 获取数据
    data = hist['data']  # 形状为(12, 360)
    max_range = hist['max_range']
    
    # 准备角度数据（转换为弧度用于计算）
    # 方位角：0~360度，步长1度
    azimuths_deg = np.linspace(0, 360, hist['num_azimuth_bins'], endpoint=False)
    azimuths_rad = np.radians(azimuths_deg)
    
    # 俯仰角：-7~53度，步长5度（共12个角度）
    elevations_deg = np.linspace(-7, 53, hist['num_elevation_bins'], endpoint=True)
    elevations_rad = np.radians(elevations_deg)
    
    # 创建网格
    A, E = np.meshgrid(azimuths_rad, elevations_rad)
    
    # 极坐标转换为笛卡尔坐标（x,y,z）
    R = data  # 距离值
    X = R * np.cos(E) * np.cos(A)  # 考虑俯仰角的三维坐标计算
    Y = R * np.cos(E) * np.sin(A)
    Z = R * np.sin(E)
    
    # 创建三维表面图
    surf = ax.plot_surface(X, Y, Z, 
                          cmap='viridis', 
                          linewidth=0.5, 
                          edgecolors='k',  # 边缘线颜色，增强立体感
                          vmin=0, 
                          vmax=max_range)
    
    # 设置坐标轴标签和范围
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    max_dim = max_range
    ax.set_xlim(-max_dim, max_dim)
    ax.set_ylim(-max_dim, max_dim)
    ax.set_zlim(-max_dim/2, max_dim/2)  # Z轴范围适当缩小
    
    # 添加标题和时间戳
    timestamp = hist['header'].stamp
    ax.set_title(f"三维极坐标直方图 (时间戳: {timestamp.secs}.{timestamp.nsecs//1000000})")
    
    # 更新颜色条
    if cbar:
        cbar.remove()
    cbar = plt.colorbar(surf, ax=ax, pad=0.1)
    cbar.set_label('障碍物距离 (m)')
    
    # 添加俯仰角标签
    for i, elev_deg in enumerate(elevations_deg):
        # 在边缘位置标注俯仰角
        ax.text(X[i, 0], Y[i, 0], Z[i, 0], f"{elev_deg}°", fontsize=8)
    
    return surf, cbar, text

def main():
    rospy.init_node('3d_polar_histogram_visualizer', anonymous=True)
    
    # 订阅指定的话题
    rospy.Subscriber('/polar_histogram', PolarHistogramMsg, callback)
    
    # 创建3D图形
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 初始变量
    surf = None
    cbar = None
    text = ax.text(0, 0, 0, "等待接收数据...", fontsize=12)
    
    # 创建动画
    ani = FuncAnimation(fig, update_plot, fargs=(ax, surf, cbar, text), 
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
    