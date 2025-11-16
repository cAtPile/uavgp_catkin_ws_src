#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fake Cam Tracker Node
模拟 cam_tracker 的行为，发布假数据用于仿真测试
"""

import rospy
import random
from std_msgs.msg import UInt8
from mission_master_pkg.msg import CamTrack


class FakeCamTracker:
    def __init__(self):
        rospy.init_node('fake_cam_tracker', anonymous=False)
        
        # 加载配置参数
        self.load_params()
        
        # 当前控制模式: 0=停止, 1=检测ball, 2=检测car
        self.control_mode = 0
        
        # 偏移循环计数器（用于cycle模式）
        self.cycle_counter = 0
        self.cycle_directions = ['center', 'right', 'down', 'left', 'up']
        
        # 订阅控制指令
        self.control_sub = rospy.Subscriber(
            '/cam_tracker/tracker_control',
            UInt8,
            self.control_callback,
            queue_size=1
        )
        
        # 发布检测结果
        self.info_pub = rospy.Publisher(
            '/cam_tracker/info',
            CamTrack,
            queue_size=10
        )
        
        # 序列号计数器
        self.seq = 0
        
        rospy.loginfo("Fake Cam Tracker initialized")
        rospy.loginfo(f"Simulation mode: {self.simulation_mode}")
        rospy.loginfo(f"Image center: ({self.center_x}, {self.center_y})")
        
    def load_params(self):
        """加载配置参数"""
        # 发布频率
        self.rate = rospy.get_param('~publish_rate', 10.0)
        
        # 模拟模式
        self.simulation_mode = rospy.get_param('~simulation_mode', 'center')
        
        # 画面中心
        self.center_x = rospy.get_param('~image_center_x', 320)
        self.center_y = rospy.get_param('~image_center_y', 240)
        
        # 偏移参数
        self.offset_x = rospy.get_param('~offset_x', 20)
        self.offset_y = rospy.get_param('~offset_y', 20)
        self.offset_direction = rospy.get_param('~offset_direction', 'right')
        
        # 随机参数
        self.random_range_x = rospy.get_param('~random_range_x', 20)
        self.random_range_y = rospy.get_param('~random_range_y', 20)
        
        # Ball检测参数
        self.ball_enabled = rospy.get_param('~ball_detection/enabled', True)
        self.ball_num = rospy.get_param('~ball_detection/default_num', 1)
        
        # Car检测参数
        self.car_enabled = rospy.get_param('~car_detection/enabled', True)
        self.car_num = rospy.get_param('~car_detection/default_num', 3)
        self.car_spacing_x = rospy.get_param('~car_detection/spacing_x', 100)
        self.car_spacing_y = rospy.get_param('~car_detection/spacing_y', 50)
        
        # 系统状态
        self.system_ok = rospy.get_param('~system_ok', True)
        self.in_gripper = rospy.get_param('~in_gripper', False)
        
    def control_callback(self, msg):
        """处理控制指令"""
        mode = msg.data
        if mode in [0, 1, 2]:
            self.control_mode = mode
            if mode == 0:
                rospy.loginfo("Control: STOP publishing")
            elif mode == 1:
                rospy.loginfo("Control: Detect BALL (pickup zone)")
            elif mode == 2:
                rospy.loginfo("Control: Detect CAR (release zone)")
        else:
            rospy.logwarn(f"Invalid control mode: {mode}")
            
    def calculate_position(self):
        """根据模拟模式计算坐标"""
        if self.simulation_mode == 'center':
            # 模式1: 画面中心
            return self.center_x, self.center_y
            
        elif self.simulation_mode == 'offset':
            # 模式2: 偏移坐标
            if self.offset_direction == 'cycle':
                # 循环模式: 中心 -> 右 -> 下 -> 左 -> 上 -> 中心
                direction = self.cycle_directions[self.cycle_counter % len(self.cycle_directions)]
                self.cycle_counter += 1
            else:
                direction = self.offset_direction
                
            if direction == 'center':
                return self.center_x, self.center_y
            elif direction == 'right':
                return self.center_x + self.offset_x, self.center_y
            elif direction == 'left':
                return self.center_x - self.offset_x, self.center_y
            elif direction == 'up':
                return self.center_x, self.center_y - self.offset_y
            elif direction == 'down':
                return self.center_x, self.center_y + self.offset_y
            else:
                return self.center_x, self.center_y
                
        elif self.simulation_mode == 'random':
            # 模式3: 随机偏移
            x = self.center_x + random.uniform(-self.random_range_x, self.random_range_x)
            y = self.center_y + random.uniform(-self.random_range_y, self.random_range_y)
            return x, y
            
        else:
            # 默认返回中心
            return self.center_x, self.center_y
            
    def generate_ball_data(self):
        """生成ball检测数据"""
        x, y = self.calculate_position()
        return {
            'ball_num': self.ball_num if self.ball_enabled else 0,
            'ball_x': float(x),
            'ball_y': float(y),
            'ball_dis': 0.0
        }
        
    def generate_car_data(self):
        """生成car检测数据"""
        car_x_list = []
        car_y_list = []
        car_dis_list = []
        
        if self.car_enabled and self.car_num > 0:
            # 基准位置
            base_x, base_y = self.calculate_position()
            
            # 生成多个车辆坐标（以基准位置为中心分布）
            for i in range(self.car_num):
                offset = i - (self.car_num - 1) / 2.0  # 居中分布
                x = base_x + offset * self.car_spacing_x
                y = base_y + offset * self.car_spacing_y
                car_x_list.append(float(x))
                car_y_list.append(float(y))
                car_dis_list.append(0.0)
                
        return {
            'car_num': len(car_x_list),
            'car_x': car_x_list,
            'car_y': car_y_list,
            'car_dis': car_dis_list
        }
        
    def publish_data(self):
        """发布检测数据"""
        if self.control_mode == 0:
            # 停止发布
            return
            
        msg = CamTrack()
        msg.header.seq = self.seq
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_link"
        msg.system_ok = self.system_ok
        msg.in_gripper = self.in_gripper
        
        if self.control_mode == 1:
            # 检测ball（抓取区）
            ball_data = self.generate_ball_data()
            msg.ball_num = ball_data['ball_num']
            msg.ball_x = ball_data['ball_x']
            msg.ball_y = ball_data['ball_y']
            msg.ball_dis = ball_data['ball_dis']
            
            # car数据为空
            msg.car_num = 0
            msg.car_x = []
            msg.car_y = []
            msg.car_dis = []
            
        elif self.control_mode == 2:
            # 检测car（释放区）
            car_data = self.generate_car_data()
            msg.car_num = car_data['car_num']
            msg.car_x = car_data['car_x']
            msg.car_y = car_data['car_y']
            msg.car_dis = car_data['car_dis']
            
            # ball数据为空
            msg.ball_num = 0
            msg.ball_x = 0.0
            msg.ball_y = 0.0
            msg.ball_dis = 0.0
            
        self.info_pub.publish(msg)
        self.seq += 1
        
    def run(self):
        """主循环"""
        rate = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            try:
                self.publish_data()
                rate.sleep()
            except rospy.ROSInterruptException:
                break
                
        rospy.loginfo("Fake Cam Tracker shutting down")


if __name__ == '__main__':
    try:
        node = FakeCamTracker()
        node.run()
    except rospy.ROSInterruptException:
        pass