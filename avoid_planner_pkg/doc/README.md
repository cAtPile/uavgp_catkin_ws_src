### 文件结构

### save

<launch>

	<!--user configure parameters for ros start-->
	<arg name="lvx_file_path" default="livox_test.lvx"/>
	<arg name="bd_list" default="100000000000000"/>
	<arg name="xfer_format" default="0"/>
	<arg name="multi_topic" default="0"/>
	<arg name="data_src" default="0"/>
	<arg name="publish_freq" default="10.0"/>
	<arg name="output_type" default="0"/>
	<arg name="rviz_enable" default="true"/>
	<arg name="rosbag_enable" default="false"/>
	<arg name="cmdline_arg" default="$(arg bd_list)"/>
	<arg name="msg_frame_id" default="livox_frame"/>
	<arg name="lidar_bag" default="true"/>
	<arg name="imu_bag" default="true"/>
	<!--user configure parameters for ros end--> 

	<param name="xfer_format" value="$(arg xfer_format)"/>
	<param name="multi_topic" value="$(arg multi_topic)"/>
	<param name="data_src" value="$(arg data_src)"/>
	<param name="publish_freq" type="double" value="$(arg publish_freq)"/>
	<param name="output_data_type" value="$(arg output_type)"/>
	<param name="cmdline_str" type="string" value="$(arg bd_list)"/>
	<param name="cmdline_file_path" type="string" value="$(arg lvx_file_path)"/>
	<param name="user_config_path" type="string" value="$(find livox_ros_driver2)/config/MID360_config.json"/>
	<param name="frame_id" type="string" value="$(arg msg_frame_id)"/>
	<param name="enable_lidar_bag" type="bool" value="$(arg lidar_bag)"/>
	<param name="enable_imu_bag" type="bool" value="$(arg imu_bag)"/>

	<node name="livox_lidar_publisher2" pkg="livox_ros_driver2"
	      type="livox_ros_driver2_node" required="true"
	      output="screen" args="$(arg cmdline_arg)"/>

	<group if="$(arg rviz_enable)">
		<node name="livox_rviz" pkg="rviz" type="rviz" respawn="true"
				args="-d $(find livox_ros_driver2)/config/display_point_cloud_ROS1.rviz"/>
    </group>

	<group if="$(arg rosbag_enable)">
    	<node pkg="rosbag" type="record" name="record" output="screen"
          		args="-a"/>
    </group>

</launch>

待办：
1.构建一个action
获取目标点位置和行动
发送位移方向

2.构建一个
同一的结构体

思路

有一个数组：
obstacle[][]
第一维度是俯仰角，从min_el 到 max_el步长step_el
第二维度是方位角，从min_az 到 max_az步长step_az
存储的数据是距离障碍物的距离
计算斥力
repulsion=repulsion_ration/obstacle_distance
将斥力存储到数组
repulsion[][]
有一个目标点 gaol_el,gaol_az,gaol_dis
计算引力gravity
计算合力
选择方向


