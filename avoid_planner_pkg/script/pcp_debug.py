#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Header
from avoid_planner_pkg.msg import PolarHistogramMsg  # 替换为你的包名
import math

'''


'''

# 全局变量存储最新的直方图数据
latest_histogram = None
lock = False

def callback(msg):
    """处理接收到的直方图消息"""
    global latest_histogram, lock
    if not lock:
        # 将一维数据转换为二维数组
        data = np.array(msg.data)
        data_2d = data.reshape((msg.num_azimuth_bins, msg.num_elevation_bins))
        
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

def update_plot(frame, ax, heatmap, cbar, text):
    """更新绘图数据"""
    global latest_histogram, lock
    
    if latest_histogram is None:
        text.set_text("等待接收直方图数据...")
        return heatmap, cbar, text
    
    # 加锁防止数据更新冲突
    lock = True
    hist = latest_histogram
    lock = False
    
    # 更新标题和时间戳
    timestamp = hist['header'].stamp
    text.set_text(f"时间戳: {timestamp.secs}.{timestamp.nsecs//1000000}")
    
    # 准备极坐标角度数据
    azimuths = np.linspace(hist['min_azimuth'], hist['max_azimuth'], 
                          hist['num_azimuth_bins'], endpoint=False)
    # 转换为度以便更好地显示
    azimuths_deg = np.rad2deg(azimuths)
    
    # 准备仰角数据
    elevations = np.linspace(hist['min_elevation'], hist['max_elevation'], 
                            hist['num_elevation_bins'], endpoint=False)
    elevations_deg = np.rad2deg(elevations)
    
    # 由于极坐标图通常以0为前方，我们将数据旋转90度
    data_rotated = np.rot90(hist['data'], 1)
    
    # 更新热力图数据
    heatmap.set_array(data_rotated)
    heatmap.set_clim(0, hist['max_range'])
    
    return heatmap, cbar, text

def main():
    rospy.init_node('polar_histogram_visualizer', anonymous=True)
    
    # 替换为你的直方图话题名
    topic_name = rospy.get_param('~histogram_topic', '/polar_histogram')
    rospy.Subscriber(topic_name, PolarHistogramMsg, callback)
    
    # 创建极坐标图
    fig, ax = plt.subplots(figsize=(10, 8), subplot_kw=dict(projection='polar'))
    fig.suptitle('极坐标直方图可视化 - 障碍物距离分布')
    
    # 初始热力图（将在更新时替换）
    initial_data = np.zeros((1, 1))
    heatmap = ax.imshow(initial_data, cmap='viridis', aspect='auto', 
                       extent=[0, 360, np.rad2deg(-0.122), np.rad2deg(0.984)])
    
    # 添加颜色条
    cbar = fig.colorbar(heatmap, ax=ax)
    cbar.set_label('距离 (m)')
    
    # 添加时间戳文本
    text = ax.text(0.05, 0.95, "", transform=ax.transAxes, 
                  verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # 设置极坐标显示
    ax.set_theta_zero_location('N')  # 北方向为0度
    ax.set_theta_direction(-1)       # 顺时针为正方向
    ax.set_rlabel_position(0)        # 半径标签位置
    
    # 创建动画
    ani = FuncAnimation(fig, update_plot, fargs=(ax, heatmap, cbar, text), 
                       interval=100, blit=False)
    
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
    