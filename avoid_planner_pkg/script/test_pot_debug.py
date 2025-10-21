#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from avoid_planner_pkg.msg import PolarFieldMsg  # 极坐标场消息类型（包含势场数据）
import math

# 全局变量存储最新的势场数据
latest_potmap = None
lock = False
max_force_global = 1.0  # 初始最大力值，会根据实际数据更新

def callback(msg):
    """处理接收到的PolarField消息，提取势场数据(pot_map)"""
    global latest_potmap, lock, max_force_global
    if not lock:
        # 将一维势场数据转换为二维数组（方位角×仰角）
        pot_data = np.array(msg.pot_map)
        potmap_2d = pot_data.reshape((msg.num_azimuth_bins, msg.num_elevation_bins))
        
        # 更新全局最大力值（取绝对值，因为力可能有正负）
        current_max = np.max(np.abs(potmap_2d))
        if current_max > 1e-6:  # 避免零值影响
            max_force_global = current_max
        
        # 存储完整的势场信息
        latest_potmap = {
            'data': potmap_2d,
            'header': msg.header,
            'num_azimuth_bins': msg.num_azimuth_bins,
            'num_elevation_bins': msg.num_elevation_bins,
            'min_azimuth': msg.min_azimuth,
            'max_azimuth': msg.max_azimuth,
            'min_elevation': msg.min_elevation,
            'max_elevation': msg.max_elevation,
            'force_vector': np.array([msg.force_vector.x, msg.force_vector.y, msg.force_vector.z])
        }

def update_plot(frame, ax, surf, cbar, text):
    """更新3D极坐标图显示势场数据"""
    global latest_potmap, lock, max_force_global
    
    # 清除上一帧但保留坐标轴设置
    ax.clear()
    ax.set_xlabel('X (力分量)')
    ax.set_ylabel('Y (力分量)')
    ax.set_zlabel('Z (力分量)')
    max_dim = max(max_force_global, 1.0)  # 确保初始视图有足够范围
    ax.set_xlim(-max_dim, max_dim)
    ax.set_ylim(-max_dim, max_dim)
    ax.set_zlim(-max_dim/2, max_dim/2)
    
    if latest_potmap is None:
        text = ax.text(0, 0, 0, "等待接收势场数据...", fontsize=12)
        return surf, cbar, text
    
    # 加锁防止数据更新冲突
    lock = True
    potmap = latest_potmap
    lock = False
    
    # 提取数据和参数
    data = potmap['data']  # 势场数据 [azimuth][elevation] = 力大小
    num_az = potmap['num_azimuth_bins']
    num_el = potmap['num_elevation_bins']
    total_force = potmap['force_vector']  # 合力矢量
    
    # 生成角度网格（弧度）
    azimuths = np.linspace(potmap['min_azimuth'], potmap['max_azimuth'], num_az, endpoint=False)
    elevations = np.linspace(potmap['min_elevation'], potmap['max_elevation'], num_el, endpoint=True)
    A, E = np.meshgrid(azimuths, elevations)  # 构建网格（仰角×方位角）
    
    # 数据转置以匹配网格形状，力的大小作为径向值
    force_magnitude = data.T  # 转置为(仰角数, 方位角数)
    
    # 极坐标转笛卡尔坐标（力的分量）
    X = force_magnitude * np.cos(E) * np.cos(A)  # x分量 = 力大小 × 角度余弦乘积
    Y = force_magnitude * np.cos(E) * np.sin(A)  # y分量
    Z = force_magnitude * np.sin(E)              # z分量
    
    # 绘制3D表面图（用红蓝色系区分正负力，红色为斥力，蓝色为引力）
    surf = ax.plot_surface(
        X, Y, Z,
        cmap='coolwarm',  # 冷暖色映射，适合表示正负力
        linewidth=0.3,
        edgecolors='gray',
        vmin=-max_force_global,
        vmax=max_force_global
    )
    
    # 绘制合力矢量（加粗箭头表示总体运动方向）
    ax.quiver(
        0, 0, 0,  # 起点：原点
        total_force[0], total_force[1], total_force[2],  # 终点：合力方向
        color='black', linewidth=3, arrow_length_ratio=0.1,
        label=f"合力: ({total_force[0]:.2f}, {total_force[1]:.2f}, {total_force[2]:.2f})"
    )
    ax.legend()
    
    # 更新颜色条（力的大小，负值表示引力，正值表示斥力）
    cbar.mappable.set_clim(-max_force_global, max_force_global)
    cbar.mappable.set_array(force_magnitude)
    
    # 添加标题和时间戳
    timestamp = potmap['header'].stamp
    ax.set_title(
        f"势场图(potmap) - {num_az}×{num_el}网格\n"
        f"时间戳: {timestamp.secs}.{timestamp.nsecs//1000000}",
        fontsize=10
    )
    
    # 标注仰角角度（度）
    elev_deg = np.degrees(elevations)
    for i, el in enumerate(elev_deg):
        ax.text(X[i, 0], Y[i, 0], Z[i, 0], f"{el:.1f}°", fontsize=8)
    
    return surf, cbar, text

def main():
    rospy.init_node('potmap_visualizer', anonymous=True)
    
    # 订阅极坐标场话题（包含势场数据）
    rospy.Subscriber('/polar_field', PolarFieldMsg, callback)
    rospy.loginfo("已订阅 /polar_field 话题，等待势场数据...")
    
    # 创建3D图形
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 初始虚拟数据用于初始化
    dummy_az = np.linspace(-np.pi, np.pi, 5)
    dummy_el = np.linspace(-0.122, 0.984, 3)  # 对应-7°到56°
    dummy_A, dummy_E = np.meshgrid(dummy_az, dummy_el)
    dummy_force = np.zeros_like(dummy_A)
    surf = ax.plot_surface(dummy_A, dummy_E, dummy_force, cmap='coolwarm', vmin=-1, vmax=1)
    
    # 创建颜色条（表示力的大小，正负区分引力/斥力）
    cbar = fig.colorbar(surf, ax=ax, pad=0.1)
    cbar.set_label('力大小 (N)', fontsize=10)
    cbar.set_ticks(np.linspace(-max_force_global, max_force_global, 5))
    
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