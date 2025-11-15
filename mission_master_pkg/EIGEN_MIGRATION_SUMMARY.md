# reachCheck函数类型迁移至Eigen数组形式

## 修改原因
`reachCheck` 函数原本使用 `std::vector<double>` 作为参数类型，在本工程中会产生类型冲突。现已全部改用 `Eigen::Vector3d` 类型。

## 修改内容

### 1. 头文件 (include/mission_master_pkg/mission_master.h)
- **函数声明修改**:
  - `void setPoint(std::vector<double> set_point)` → `void setPoint(const Eigen::Vector3d& set_point)`
  - `bool reachCheck(std::vector<double> check_point)` → `bool reachCheck(const Eigen::Vector3d& check_point)`

- **成员变量修改**:
  - 所有 `*_waypoint_re` 变量从 `std::vector<double>` 改为 `Eigen::Vector3d`:
    - `takeoff_waypoint_re`
    - `pickup_start_waypoint_re`
    - `pickup_end_waypoint_re`
    - `avoid_start_waypoint_re`
    - `avoid_end_waypoint_re`
    - `trace_start_waypoint_re`
    - `trace_end_waypoint_re`

### 2. 实现文件 (lib/utils.cpp)
- **setPoint 函数**: 使用 `set_point(0/1/2)` 代替 `set_point[0/1/2]`
- **reachCheck 函数**: 使用 `check_point(0/1/2)` 代替 `check_point[0/1/2]`

### 3. 航点加载函数 (lib/takeoff_kit.cpp)
- **loadWaypoints 函数**: 使用 `Eigen::Vector3d(x, y, z)` 构造函数代替列表初始化 `{x, y, z}`

### 4. 调用点修改
在以下文件中，将列表初始化语法改为 Eigen::Vector3d 构造函数：
- **lib/pickup_kit.cpp**: 
  - `[temp_pose.pose.position.x, ...]` → `Eigen::Vector3d(temp_pose.pose.position.x, ...)`
- **lib/trace_kit.cpp**: 
  - `[temp_pose.pose.position.x, ...]` → `Eigen::Vector3d(temp_pose.pose.position.x, ...)`
- **lib/land_kit.cpp**: 
  - `[home_pose.pose.position.x, ..., 3.0]` → `Eigen::Vector3d(home_pose.pose.position.x, ..., 3.0)`

### 5. 其他文件
- **lib/avoid_kit.cpp**: 无需修改（直接使用 `_re` 变量）
- **lib/takeoff_kit.cpp**: 无需修改（直接使用 `_re` 变量）

## 兼容性说明
- `std::vector<double> *_waypoint_v` 变量保持不变，用于从参数服务器读取配置
- `Eigen::Vector3d *_waypoint_re` 变量用于实际的位置计算和比较
- 所有直接使用 `*_waypoint_re` 变量调用 `setPoint()` 或 `reachCheck()` 的地方无需修改

## 编译验证
✅ 编译成功，无错误或警告

## 修改日期
2025年11月15日

## 注意事项
- Eigen::Vector3d 使用 `operator()` 访问元素，而不是 `operator[]`
- 参数传递使用 `const Eigen::Vector3d&` 以避免不必要的复制
