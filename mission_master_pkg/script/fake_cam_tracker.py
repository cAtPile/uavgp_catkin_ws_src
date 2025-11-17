#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped  
from std_msgs.msg import UInt8
from mission_master_pkg.msg import CamTrack  # 需确保msg已正确编译

class FakeCamNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('fake_cam_node', anonymous=True)
        
        # 配置固定球地图坐标（可通过参数服务器读取，方便修改）
        self.ball_map_x = rospy.get_param('~ball_map_x', 5.0)  # 球在地图中的固定X坐标
        self.ball_map_y = rospy.get_param('~ball_map_y', 3.0)  # 球在地图中的固定Y坐标
        self.scale_ratio = rospy.get_param('~scale_ratio', 0.1)  # 相对位置缩放比例
        
        # 状态变量
        self.current_pose = PoseStamped()  # 无人机当前位置
        self.track_mode = 0  # 0:停止 1:抓取区(检测ball) 2:释放区(检测car)
        self.rate = rospy.Rate(10)  # 发布频率10Hz
        
        # 订阅者
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/cam_tracker/tracker_control', UInt8, self.control_callback)
        
        # 发布者
        self.cam_info_pub = rospy.Publisher('/cam_tracker/info', CamTrack, queue_size=10)
        
        rospy.loginfo("Fake Cam Node 已启动")

    def pose_callback(self, msg):
        # 更新无人机当前位置
        self.current_pose = msg

    def control_callback(self, msg):
        # 更新跟踪模式
        self.track_mode = msg.data
        mode_desc = {0: "停止发布", 1: "抓取区（检测ball）", 2: "释放区（检测car）"}
        rospy.loginfo(f"切换跟踪模式: {mode_desc.get(self.track_mode, '未知模式')}")

    def calculate_ball_relative_pos(self):
        # 计算球相对于无人机的位置，并应用缩放比例
        drone_x = self.current_pose.pose.position.x
        drone_y = self.current_pose.pose.position.y
        
        # 相对位置 = (球地图坐标 - 无人机坐标) * 缩放比例
        ball_rel_x = (self.ball_map_x - drone_x) * self.scale_ratio
        ball_rel_y = (self.ball_map_y - drone_y) * self.scale_ratio
        
        return ball_rel_x, ball_rel_y

    def run(self):
        while not rospy.is_shutdown():
            if self.track_mode == 0:
                # 停止发布，直接休眠
                self.rate.sleep()
                continue
            
            # 初始化CamTrack消息
            cam_msg = CamTrack()
            cam_msg.header.stamp = rospy.Time.now()
            cam_msg.header.frame_id = "base_link"  # 坐标系设为无人机基坐标系
            cam_msg.system_ok = True  # 系统正常运行
            cam_msg.in_gripper = False  # 夹爪未抓取（预留接口）

            if self.track_mode == 1:
                # 抓取区模式：只填充ball信息，car信息设为默认
                ball_x, ball_y = self.calculate_ball_relative_pos()
                cam_msg.ball_num = 1  # 固定检测到1个球
                cam_msg.ball_id = 1    # 球ID固定为1
                cam_msg.ball_x = ball_x
                cam_msg.ball_y = ball_y
                cam_msg.ball_dis = 0.0  # 预留接口，输出0
                
                # car信息置空
                cam_msg.car_num = 0
                cam_msg.car_ids = []
                cam_msg.car_x = []
                cam_msg.car_y = []
                cam_msg.car_dis = []

            elif self.track_mode == 2:
                # 释放区模式：只填充car信息，ball信息设为默认
                cam_msg.car_num = 1  # 固定检测到1个车（可根据需求修改）
                cam_msg.car_ids = [1]  # 车ID列表
                cam_msg.car_x = [0.5]  # 车的X坐标（示例值，可自定义）
                cam_msg.car_y = [0.3]  # 车的Y坐标（示例值，可自定义）
                cam_msg.car_dis = [0.0]  # 预留接口，输出0
                
                # ball信息置空
                cam_msg.ball_num = 0
                cam_msg.ball_id = 0
                cam_msg.ball_x = 0.0
                cam_msg.ball_y = 0.0
                cam_msg.ball_dis = 0.0

            # 发布消息
            self.cam_info_pub.publish(cam_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = FakeCamNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Fake Cam Node 正常退出")