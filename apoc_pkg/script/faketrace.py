#!/usr/bin/env python3
import rospy
import random
import sys
import threading
from apoc_pkg.msg import detection_data  # 导入自定义消息类型

# 全局变量：控制当前目标状态（1=有目标，0=无目标），用锁保证线程安全
target_state = 0  # 初始默认无目标
state_lock = threading.Lock()

def keyboard_listener():
    """键盘监听线程：单独线程处理输入，避免阻塞消息发布"""
    rospy.loginfo("=== 键盘控制说明 ===")
    rospy.loginfo("输入 1 并回车：切换为【有目标】模式")
    rospy.loginfo("输入 0 并回车：切换为【无目标】模式")
    rospy.loginfo("输入 exit 并回车：退出脚本")
    rospy.loginfo("====================")

    while not rospy.is_shutdown():
        # 读取键盘输入（strip() 去除空格和换行）
        user_input = sys.stdin.readline().strip()
        
        # 加锁修改目标状态（避免与发布线程冲突）
        with state_lock:
            if user_input == "1":
                target_state = 1
                rospy.loginfo("=== 已切换为【有目标】模式 ===")
            elif user_input == "0":
                target_state = 0
                rospy.loginfo("=== 已切换为【无目标】模式 ===")
            elif user_input.lower() == "exit":
                rospy.loginfo("收到退出指令，脚本即将停止...")
                rospy.signal_shutdown("User exited")  # 触发ROS节点关闭
            else:
                rospy.logwarn("无效输入！请输入 1（有目标）、0（无目标）或 exit（退出）")

def fake_detection_publisher():
    # 1. 初始化ROS节点
    rospy.init_node('fake_detection_publisher_keyboard', anonymous=True)
    
    # 2. 创建发布者（话题/detection/data，队列大小10）
    pub = rospy.Publisher('/detection/data', detection_data, queue_size=10)
    
    # 3. 发布频率（10Hz，与订阅端匹配）
    rate = rospy.Rate(10)
    
    # 4. 模拟参数（可调整）
    TARGET_CENTER_X = 320.0  # 目标中心X（与trackSwitch一致）
    TARGET_CENTER_Y = 320.0  # 目标中心Y（与trackSwitch一致）
    FLUCTUATION_RANGE = 50.0  # 坐标波动范围（模拟检测误差）

    rospy.loginfo("=== 虚拟检测消息发布者启动成功 ===")
    
    # 启动键盘监听线程（非阻塞，不影响消息发布）
    keyboard_thread = threading.Thread(target=keyboard_listener)
    keyboard_thread.daemon = True  # 线程随节点关闭而退出
    keyboard_thread.start()

    while not rospy.is_shutdown():
        # 创建消息对象
        msg = detection_data()
        
        # 加锁读取当前目标状态（避免线程冲突）
        with state_lock:
            current_state = target_state

        # 根据当前状态设置消息内容
        if current_state == 1:
            # 有目标：detection_id=1，坐标围绕中心波动
            msg.detection_id = 1
            msg.detection_x = TARGET_CENTER_X + random.uniform(-FLUCTUATION_RANGE, FLUCTUATION_RANGE)
            msg.detection_y = TARGET_CENTER_Y + random.uniform(-FLUCTUATION_RANGE, FLUCTUATION_RANGE)
            # 每1秒打印一次有目标日志（避免刷屏）
            rospy.loginfo_throttle(1, f"当前状态：有目标 | detection_id=1, x={msg.detection_x:.1f}, y={msg.detection_y:.1f}")
        else:
            # 无目标：detection_id=0，坐标设为0
            msg.detection_id = 0
            msg.detection_x = 0.0
            msg.detection_y = 0.0
            # 无目标日志用debug级别（避免干扰）
            rospy.logdebug("当前状态：无目标 | detection_id=0")

        # 发布消息
        pub.publish(msg)
        
        # 按频率休眠
        rate.sleep()

if __name__ == '__main__':
    try:
        fake_detection_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("=== 虚拟检测消息发布者已停止 ===")