结合Mid360激光雷达特性与PX4+MAVROS的局部路径规划需求，以下是针对避障系统的文件结构设计及功能划分建议，参考了直方图+星形搜索的核心算法，并适配三维激光雷达的点云处理特性：


### 一、整体文件结构
```
px4_mid360_avoidance/
├── include/
│   ├── mid360_avoidance/
│   │   ├── local_planner.h        # 局部规划器类声明
│   │   ├── star_planner.h         # 星形搜索算法类声明
│   │   ├── cost_calculator.h      # 成本计算模块声明
│   │   ├── pointcloud_processor.h # 点云处理模块声明
│   │   ├── mavros_interface.h     # MAVROS通信接口声明
│   │   └── utils.h                # 通用工具函数声明
├── src/
│   ├── local_planner.cpp          # 规划器主逻辑实现
│   ├── star_planner.cpp           # 星形搜索算法实现
│   ├── cost_calculator.cpp        # 成本矩阵计算实现
│   ├── pointcloud_processor.cpp   # Mid360点云处理实现
│   ├── mavros_interface.cpp       # MAVROS通信实现
│   ├── utils.cpp                  # 工具函数实现
│   └── main.cpp                   # 节点入口
├── launch/
│   └── avoidance.launch           # 启动文件（配置参数、节点关系）
└── config/
    └── params.yaml                # 算法参数（如避障距离阈值、搜索深度等）
```


### 二、核心文件及函数功能说明

#### 1. `pointcloud_processor.h` / `.cpp`（Mid360点云处理）
**核心功能**：处理Mid360的原始点云，提取有效障碍物信息，适配直方图算法输入。  
**主要函数**：
- `PointcloudProcessor(ros::NodeHandle& nh)`：构造函数，初始化订阅者（订阅`/mid360/points`话题）、参数（如点云滤波范围）。
- `void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)`：点云回调函数，触发处理流程。
- `void filterPointcloud()`：点云滤波（移除无效点、离群点、地面点），利用Mid360的16线激光特性保留垂直方向细节。
- `void transformToBodyFrame()`：将点云从激光雷达坐标系转换到无人机机体坐标系（需预先校准TF变换）。
- `void generatePolarHistogram()`：将三维点云转换为极坐标直方图（方位角0-360°，仰角±15°，适配Mid360的FOV），每个网格存储最近障碍物距离。
- `const PolarHistogram& getHistogram()`：返回处理后的直方图，供成本计算模块使用。


#### 2. `cost_calculator.h` / `.cpp`（成本矩阵计算）
**核心功能**：基于直方图和目标位置，计算各方向的避障成本。  
**主要函数**：
- `CostCalculator()`：初始化成本参数（如障碍物安全距离、目标方向权重）。
- `void setHistogram(const PolarHistogram& hist)`：接收点云处理模块的直方图数据。
- `void setGoal(const Eigen::Vector3d& goal)`：设置局部目标点（来自全局路径规划器）。
- `void computeCostMatrix()`：计算极坐标下每个方向的成本，综合以下因素：
  - 障碍物距离成本（距离越近，成本越高，Mid360的100m测距范围可设阈值）。
  - 目标方向偏差成本（与目标方向夹角越大，成本越高）。
  - 平滑性成本（与当前速度方向的一致性，避免剧烈转向）。
- `const CostMatrix& getCostMatrix()`：返回成本矩阵，供星形搜索模块使用。


#### 3. `star_planner.h` / `.cpp`（星形搜索路径生成）
**核心功能**：基于成本矩阵搜索局部最优避障路径。  
**主要函数**：
- `StarPlanner()`：初始化搜索参数（如最大扩展节点数、搜索深度）。
- `void setCostMatrix(const CostMatrix& cost)`：接收成本矩阵数据。
- `void setStartAndGoal(const Eigen::Vector3d& start, const Eigen::Vector3d& goal)`：设置起点（无人机当前位置）和局部目标点。
- `bool searchPath()`：构建星形搜索树，生成避障路径：
  - 以起点为根节点，按成本矩阵低代价方向扩展子节点。
  - 每个节点成本 = 父节点成本 + 当前方向成本 + 启发式成本（到目标的距离）。
  - 达到最大深度或接近目标时停止，回溯生成路径。
- `const std::vector<Eigen::Vector3d>& getPath()`：返回生成的局部路径（一系列航点）。


#### 4. `local_planner.h` / `.cpp`（规划器主逻辑）
**核心功能**：协调各模块工作，输出最终避障路径。  
**主要函数**：
- `LocalPlanner(ros::NodeHandle& nh)`：初始化各子模块（点云处理、成本计算、星形搜索），订阅全局目标，发布局部路径。
- `void run()`：主循环函数，按频率（如10Hz）执行：
  1. 检查点云数据是否更新（`pointcloud_processor_.isUpdated()`）。
  2. 若有新数据，触发成本计算（`cost_calculator_.computeCostMatrix()`）。
  3. 调用星形搜索生成路径（`star_planner_.searchPath()`）。
  4. 验证路径安全性（`isPathSafe()`），若存在碰撞风险则重新规划。
- `void publishPath()`：将局部路径转换为ROS消息（`nav_msgs/Path`），发布给位置控制器。
- `bool isPathSafe()`：检查生成的路径是否与直方图中的障碍物冲突（冗余校验）。


#### 5. `mavros_interface.h` / `.cpp`（MAVROS通信接口）
**核心功能**：与PX4进行数据交互，获取无人机状态并发送路径指令。  
**主要函数**：
- `MavrosInterface(ros::NodeHandle& nh)`：初始化MAVROS订阅者/发布者，订阅无人机位置（`/mavros/local_position/pose`）、速度（`/mavros/local_position/velocity`），发布路径（`/mavros/setpoint_trajectory/local`）。
- `const Eigen::Vector3d getCurrentPose()`：返回无人机当前位置（机体坐标系→局部ENU坐标系转换）。
- `void sendPathToPx4(const std::vector<Eigen::Vector3d>& path)`：将局部路径转换为PX4可识别的轨迹消息，通过MAVROS发送（支持位置模式或轨迹模式）。
- `void setFlightMode(const std::string& mode)`：切换PX4飞行模式（如“OFFBOARD”模式执行自主避障）。


#### 6. `utils.h` / `.cpp`（通用工具）
**辅助函数**：
- `Eigen::Vector3d quaternionToYaw(const geometry_msgs::Quaternion& quat)`：从四元数提取偏航角（用于方向计算）。
- `double calculateDistance(const Eigen::Vector3d& a, const Eigen::Vector3d& b)`：计算两点距离（启发式成本计算用）。
- `void smoothPath(std::vector<Eigen::Vector3d>& path)`：路径平滑处理（减少航点抖动）。
- `bool loadParameters(ros::NodeHandle& nh, Params& params)`：从`params.yaml`加载算法参数（如避障安全距离、搜索树深度）。


### 三、模块依赖关系
1. **数据流向**：  
   `Mid360点云` → `pointcloud_processor`（生成直方图） → `cost_calculator`（生成成本矩阵） → `star_planner`（生成路径） → `local_planner`（验证路径） → `mavros_interface`（发送至PX4）。

2. **关键依赖**：  
   - 点云处理模块是基础，其输出的直方图直接影响成本计算准确性。  
   - 星形搜索依赖成本矩阵引导方向，需与成本计算模块的参数（如权重）协同调优。  
   - MAVROS接口需实时获取无人机状态，确保规划起点与实际位置一致。


### 四、适配Mid360的特殊设计
- **点云滤波**：针对Mid360的16线激光雷达，保留垂直方向（±15°）的点云细节，避免过滤低空障碍物（如无人机下方的线缆）。  
- **直方图分辨率**：方位角分辨率设为1°（360个网格），仰角分辨率设为5°（7个网格），平衡计算量与避障精度。  
- **通信频率**：点云处理和规划频率设为10Hz，匹配Mid360的默认输出频率（10Hz），避免数据积压。

通过以上结构，可实现基于Mid360的PX4局部避障，各模块职责清晰，便于后续调试和算法优化（如替换星形搜索为RRT*等其他算法）。