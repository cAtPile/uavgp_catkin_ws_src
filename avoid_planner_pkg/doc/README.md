### 文件结构

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