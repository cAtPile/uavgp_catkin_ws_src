#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from avoid_planner_pkg.msg import PolarFieldMsg  # 极坐标场消息类型
import math

# 全局变量存储最新的dismap数据
latest_dismap = None
lock = False
max_range_global = 0.01  # 初始最大距离，会根据实际数据更新

def callback(msg):
    """处理接收到的PolarField消息，提取障碍物距离图(dismap)"""
    global latest_dismap, lock, max_range_global
    if not lock:
        # 将一维距离数据转换为二维数组（方位角×仰角）
        dis_data = np.array(msg.obstacle_distances)
        dismap_2d = dis_data.reshape((msg.num_azimuth_bins, msg.num_elevation_bins))
        
        # 替换无穷大值为传感器最大距离，便于可视化
        dismap_2d[np.isinf(dismap_2d)] = msg.max_range
        
        # 更新全局最大距离参数
        max_range_global = msg.max_range
        
        # 存储完整的dismap信息
        latest_dismap = {
            'data': dismap_2d,
            'header': msg.header,
            'num_azimuth_bins': msg.num_azimuth_bins,
            'num_elevation_bins': msg.num_elevation_bins,
            'min_azimuth': msg.min_azimuth,
            'max_azimuth': msg.max_azimuth,
            'min_elevation': msg.min_elevation,
            'max_elevation': msg.max_elevation
        }

def update_plot(frame, ax, surf, cbar, text):
    """更新3D极坐标图显示dismap数据"""
    global latest_dismap, lock, max_range_global
    
    # 清除上一帧但保留坐标轴设置
    ax.clear()
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    max_dim = max_range_global if max_range_global > 0.1 else 50  # 避免初始值过小
    ax.set_xlim(-max_dim, max_dim)
    ax.set_ylim(-max_dim, max_dim)
    ax.set_zlim(-max_dim/2, max_dim/2)  # Z轴范围减半，符合实际场景
    
    if latest_dismap is None:
        text = ax.text(0, 0, 0, "等待接收dismap数据...", fontsize=12)
        return surf, cbar, text
    
    # 加锁防止数据更新冲突
    lock = True
    dismap = latest_dismap
    lock = False
    
    # 提取数据和参数
    data = dismap['data']
    num_az = dismap['num_azimuth_bins']
    num_el = dismap['num_elevation_bins']
    
    # 生成角度网格（弧度）
    azimuths = np.linspace(dismap['min_azimuth'], dismap['max_azimuth'], num_az, endpoint=False)
    elevations = np.linspace(dismap['min_elevation'], dismap['max_elevation'], num_el, endpoint=True)
    A, E = np.meshgrid(azimuths, elevations)  # 构建网格（仰角×方位角）
    
    # 数据转置以匹配网格形状
    R = data.T  # 转置为(仰角数, 方位角数)
    
    # 极坐标转笛卡尔坐标
    X = R * np.cos(E) * np.cos(A)  # 径向×仰角余弦×方位角余弦
    Y = R * np.cos(E) * np.sin(A)  # 径向×仰角余弦×方位角正弦
    Z = R * np.sin(E)              # 径向×仰角正弦
    
    # 绘制3D表面图
    surf = ax.plot_surface(
        X, Y, Z,
        cmap='plasma',  # 使用plasma配色，距离越近颜色越亮
        linewidth=0.3,
        edgecolors='gray',
        vmin=0,
        vmax=max_range_global
    )
    
    # 更新颜色条
    cbar.mappable.set_clim(0, max_range_global)
    cbar.mappable.set_array(R)
    
    # 添加标题和时间戳
    timestamp = dismap['header'].stamp
    ax.set_title(
        f"障碍物距离图(dismap) - {num_az}×{num_el}网格\n"
        f"时间戳: {timestamp.secs}.{timestamp.nsecs//1000000}",
        fontsize=10
    )
    
    # 标注仰角角度（度）
    elev_deg = np.degrees(elevations)
    for i, el in enumerate(elev_deg):
        ax.text(X[i, 0], Y[i, 0], Z[i, 0], f"{el:.1f}°", fontsize=8)
    
    return surf, cbar, text

def main():
    rospy.init_node('dismap_visualizer', anonymous=True)
    
    # 订阅极坐标场话题（包含dismap数据）
    rospy.Subscriber('/polar_field_test', PolarFieldMsg, callback)
    rospy.loginfo("已订阅 /polar_field_test 话题，等待dismap数据...")
    
    # 创建3D图形
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 初始虚拟数据用于初始化
    dummy_az = np.linspace(-np.pi, np.pi, 5)
    dummy_el = np.linspace(-0.122, 0.984, 3)  # 对应-7°到56°
    dummy_A, dummy_E = np.meshgrid(dummy_az, dummy_el)
    dummy_R = np.zeros_like(dummy_A)
    surf = ax.plot_surface(dummy_A, dummy_E, dummy_R, cmap='plasma', vmin=0, vmax=50)
    
    # 创建颜色条（表示距离）
    cbar = fig.colorbar(surf, ax=ax, pad=0.1)
    cbar.set_label('障碍物距离 (m)', fontsize=10)
    
    # 初始提示文本
    text = ax.text(0, 0, 0, "等待接收数据...", fontsize=12)
    
    # 设置初始视角
    ax.view_init(elev=30, azim=45)
    
    # 创建动画更新机制
    ani = FuncAnimation(
        fig, update_plot,
        fargs=(ax, surf, cbar, text),
        interval=200,  # 5Hz更新频率
        blit=False
    )
    
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