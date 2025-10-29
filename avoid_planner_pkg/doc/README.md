### 文件结构

### save
lanner_action/feedback
/pcp_test_node/avoid_planner_action/goal
/pcp_test_node/avoid_planner_action/result
/pcp_test_node/avoid_planner_action/status
/polar_field_test
/rosout
/rosout_agg^Ca@ubuntu:~$ rostopic list
/avoid_planner_action/cancel
/avoid_planner_action/feedback
/avoid_planner_action/goal
/avoid_planner_action/result
/avoid_planner_action/status
/livox/imu
/livox/lidar
/pcp_test_node/avoid_planner_action/cancel
/pcp_test_node/avoid_p
/tf
/tf_static
a@ubuntu:~$ rostopic /avoid_planner_action/goal
rostopic is a command-line tool for printing information about ROS Topics.

Commands:
	rostopic bw	display bandwidth used by topic
	rostopic delay	display delay of topic from timestamp in header
	rostopic echo	print messages to screen
	rostopic find	find topics by type
	rostopic hz	display publishing rate of topic    
	rostopic info	print information about active topic
	rostopic list	list active topics
	rostopic pub	publish data to topic
	rostopic type	print topic or field type

Type rostopic <command> -h for more detailed usage, e.g. 'rostopic echo -h'

a@ubuntu:~$ rostopic echo /avoid_planner_action/goal


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


