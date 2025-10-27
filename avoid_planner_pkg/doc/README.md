### 文件结构

### save
160
[WARN] [1761575858.523398251]: Transform exception: "base_link" passed to lookupTransform argument target_frame does not exist. 
[WARN] [1761575858.523564880]: Failed to transform pointcloud to body frame
[INFO] [1761575859.419747610]: resive PointsCloud,wide=20064,high=1,pointsNum=20064
[WARN] [1761575859.568099749]: Transform exception: "base_link" passed to lookupTransform argument target_frame does not exist. 
[WARN] [1761575859.568224041]: Failed to transform pointcloud to body frame
[INFO] [1761575860.464828770]: resive PointsCloud,wide=20064,high=1,pointsNum=20064
[WARN] [1761575860.612153964]: Transform exception: "base_link" passed to lookupTransform argument target_frame does not exist. 
[WARN] [1761575860.612284528]: Failed to transform pointcloud to body frame
[INFO] [1761575861.510198901]: resive PointsCloud,wide=20064,high=1,pointsNum=20064
[WARN] [1761575861.657552000]: Transform exception: "base_link" passed to lookupTransform argument target_frame does not exist. 
[WARN] [1761575861.657676036]: Failed to transform pointcloud to body frame
[INFO] [1761575862.256882097]: published: 10  | az bins: 37, el bins: 11
[INFO] [1761575862.556722127]: resive PointsCloud,wide=20064,high=1,pointsNum=20064
[WARN] [1761575862.704727119]: Transform exception: "base_link" passed to lookupTransform argument target_frame does not exist. 
[WARN] [1761575862.704842547]: Failed to transform pointcloud to body frame
[INFO] [1761575863.601459439]: resive PointsCloud,wide=19968,high=1,pointsNum=19968
[WARN] [1761575863.749107716]: Transform exception: "base_link" passed to lookupTransform argument target_frame does not exist. 
[WARN] [1761575863.749224072]: Failed to transform pointcloud to body frame
[INFO] [1761575864.645876966]: resive PointsCloud,wide=19968,high=1,pointsNum=19968
[WARN] [1761575864.793584605]: Transform exception: "base_link" passed to lookupTransform argument target_frame does not exist. 
[WARN] [1761575864.793710625]: Failed to transform pointcloud to body frame
[INFO] [1761575865.687311228]: resive PointsCloud,wide=20064,high=1,pointsNum=20064
[WARN] [1761575865.835685321]: Transform exception: "base_link" passed to lookupTransform argument target_frame does not exist. 
[WARN] [1761575865.835799020]: Failed to transform pointcloud to body frame
[INFO] [1761575866.731137217]: resive PointsCloud,wide=20064,high=1,pointsNum=20064
[WARN] [1761575866.876637360]: Transform exception: "base_link" passed to lookupTransform argument target_frame does not exist. 
[WARN] [1761575866.876760244]: Failed to transform pointcloud to body frame
^C[pcp_test_node-1] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done

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


