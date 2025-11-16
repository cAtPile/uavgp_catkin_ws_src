#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt8
# 请根据实际自定义消息所在的包名修改导入路径
from cam_tracker_msgs.msg import CamTrack  # 假设CamTrack.msg在cam_tracker_msgs包中

class FakeCamTracker:
    def __init__(self):
        # 初始化发布者，发布/cam_tracker/info话题，消息类型为CamTrack
        self.info_pub = rospy.Publisher(
            '/cam_tracker/info', 
            CamTrack, 
            queue_size=10
        )
        
        # 初始化订阅者，订阅/cam_tracker/tracker_control话题，消息类型为UInt8
        self.control_sub = rospy.Subscriber(
            '/cam_tracker/tracker_control', 
            UInt8, 
            self.control_callback
        )
        
        rospy.loginfo("Fake Cam Tracker initialized. Waiting for control signal...")

    def control_callback(self, msg):
        """处理收到的控制指令，当指令为1时发布目标信息"""
        if msg.data == 1:
            rospy.loginfo("Received control signal 1, publishing cam track info...")
            
            # 构造CamTrack消息
            cam_track_msg = CamTrack()
            
            # 填充消息字段
            cam_track_msg.header.stamp = rospy.Time.now()  # 时间戳设为当前时间
            cam_track_msg.system_ok = True  # 系统正常运行
            
            # 球的信息
            cam_track_msg.ball_num = 1  # 检测到1个球
            cam_track_msg.ball_x = 20.0  # 指定x坐标
            cam_track_msg.ball_y = 0.0   # 指定y坐标
            cam_track_msg.ball_dis = 0.0  # 距离留空为0
            
            # 车的信息（无检测到车）
            cam_track_msg.car_num = 0
            cam_track_msg.car_x = []
            cam_track_msg.car_y = []
            cam_track_msg.car_dis = []
            
            # 夹爪状态（默认无球）
            cam_track_msg.in_gripper = False
            
            # 发布消息
            self.info_pub.publish(cam_track_msg)

if __name__ == '__main__':
    try:
        # 初始化节点
        rospy.init_node('fake_cam_tracker', anonymous=True)
        
        # 创建实例，开始监听和发布
        fake_tracker = FakeCamTracker()
        
        # 保持节点运行
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted.")