#ifndef COMMON_H
#define COMMON_H

#include "avoidance/histogram.h"  // 引入直方图相关定义（如 ALPHA_RES 分辨率）
#include <pcl/point_cloud.h>      // PCL 点云库
#include <pcl/point_types.h>      // PCL 点类型定义
#include <Eigen/Core>             // Eigen 线性代数库（向量/矩阵）
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>  // ROS 几何消息类型
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/Trajectory.h>  // MAVROS 轨迹消息
#include <tf/transform_listener.h>   // TF 坐标变换
#include <mutex>                     // 线程互斥锁

namespace avoidance {  // 避障系统命名空间

// 无人机状态枚举（参考 MAVLink 协议）
enum class MAV_STATE {
  MAV_STATE_UNINIT,            // 未初始化
  MAV_STATE_BOOT,              // 启动中
  MAV_STATE_CALIBRATIN,        // 校准中（注：原拼写可能为 CALIBRATING）
  MAV_STATE_STANDBY,           // 待机
  MAV_STATE_ACTIVE,            // 激活（正常运行）
  MAV_STATE_CRITICAL,          // 临界状态
  MAV_STATE_EMERGENCY,         // 紧急状态
  MAV_STATE_POWEROFF,          // 断电
  MAV_STATE_FLIGHT_TERMINATION // 飞行终止
};

// 导航状态枚举（无人机的主要操作模式）
enum class NavigationState {
  mission,       // 执行任务
  auto_takeoff,  // 自动起飞
  auto_land,     // 自动着陆
  auto_rtl,      // 自动返航
  auto_rtgs,     // 自动返回起飞点
  offboard,      // 外部控制模式
  auto_loiter,   // 自动悬停
  none           // 无状态
};

// 无人机指令枚举（参考 MAVLink 命令）
enum class MavCommand {
  MAV_CMD_NAV_LAND = 21,       // 着陆指令
  MAV_CMD_NAV_TAKEOFF,         // 起飞指令
  MAV_CMD_DO_CHANGE_SPEED = 178 // 改变速度指令
};

// 极坐标点结构（用于表示方位和距离）
struct PolarPoint {
  PolarPoint(float e_, float z_, float r_) : e(e_), z(z_), r(r_){};  // 带参构造
  PolarPoint() : e(0.0f), z(0.0f), r(0.0f){};                        // 默认构造
  float e;  // 仰角（单位：度）
  float z;  // 方位角（单位：度）
  float r;  // 距离（单位：米）
};

/**
 * @brief 传感器视场角（FOV）结构
 * 定义传感器的当前方位、水平和垂直视场角
 */
struct FOV {
  FOV() : yaw_deg(0.f), pitch_deg(0.f), h_fov_deg(0.f), v_fov_deg(0.f){};  // 默认构造
  FOV(float y, float p, float h, float v) : yaw_deg(y), pitch_deg(p), h_fov_deg(h), v_fov_deg(v){};  // 带参构造
  float yaw_deg;    // 偏航角（单位：度）
  float pitch_deg;  // 俯仰角（单位：度）
  float h_fov_deg;  // 水平视场角（单位：度）
  float v_fov_deg;  // 垂直视场角（单位：度）
};

/**
 * @brief 基于模型的轨迹规划参数结构
 * 当 MPC_AUTO_MODE=1 时，所有参数用于飞控的 jerk-limited 轨迹规划
 * 当 MPC_AUTO_MODE=0 时，仅使用垂直/水平加速度参数
 */
struct ModelParameters {
  int param_mpc_auto_mode = -1;        // 自动模式：0=默认轨迹跟踪，1= jerk-limited 轨迹
  float param_mpc_jerk_min = NAN;      // 基于速度的最小 jerk 限制
  float param_mpc_jerk_max = NAN;      // 基于速度的最大 jerk 限制
  float param_mpc_acc_up_max = NAN;    // 垂直向上最大加速度
  float param_mpc_z_vel_max_up = NAN;  // 最大上升速度
  float param_mpc_acc_down_max = NAN;  // 垂直向下最大加速度
  float param_mpc_z_vel_max_dn = NAN;  // 最大下降速度
  float param_mpc_acc_hor = NAN;       // 水平最大加速度
  float param_mpc_xy_cruise = NAN;     // 任务中的期望水平速度
  float param_mpc_tko_speed = NAN;     // 起飞爬升率
  float param_mpc_land_speed = NAN;    // 着陆下降率
  float param_mpc_yawrauto_max = NAN;  // 自动模式最大偏航角速度

  float param_nav_acc_rad = NAN;       // 导航加速度（弧度）

  float param_cp_dist = NAN;           // 避障安全距离（-1 表示禁用）
};

// 数学常量与编译属性定义
#define M_PI_F 3.14159265358979323846f  // 单精度 π
#define WARN_UNUSED __attribute__((warn_unused_result))  // 未使用返回值警告

// 角度转换常量
const float DEG_TO_RAD = M_PI_F / 180.f;  // 度转弧度
const float RAD_TO_DEG = 180.0f / M_PI_F;  // 弧度转度

/**
 * @brief 判断点是否在视场角（FOV）内
 * @param[in] fov_vec FOV 结构向量（多传感器）
 * @param[in] p_pol 待判断的极坐标点
 * @return 是否在视场内
 */
bool pointInsideFOV(const std::vector<FOV>& fov_vec, const PolarPoint& p_pol);
bool pointInsideFOV(const FOV& fov, const PolarPoint& p_pol);  // 单传感器版本

/**
 * @brief 判断直方图单元格是否在水平视场角内（边缘也算）
 * @param[in] fov_vec FOV 结构向量
 * @param[in] idx 直方图单元格列索引
 * @param[in] position 当前位置
 * @param[in] yaw_fcu_frame 无人机在全局坐标系中的偏航角（度）
 * @return 是否在水平视场内
 */
bool histogramIndexYawInsideFOV(const std::vector<FOV>& fov_vec, const int idx, Eigen::Vector3f position,
                                float yaw_fcu_frame);
bool histogramIndexYawInsideFOV(const FOV& fov, const int idx, Eigen::Vector3f position, float yaw_fcu_frame);

/**
 * @brief 判断点是否在偏航角视场（Yaw FOV）内
 * @param[in] fov_vec FOV 结构向量
 * @param[in] p_pol 待判断的极坐标点
 * @return 是否在偏航视场内
 */
bool pointInsideYawFOV(const std::vector<FOV>& fov_vec, const PolarPoint& p_pol);
bool pointInsideYawFOV(const FOV& fov, const PolarPoint& p_pol);

/**
 * @brief 判断点属于哪个传感器的视场
 * @param[in] fov_vec FOV 结构向量
 * @param[in] p_pol 极坐标点
 * @param[out] idx 传感器索引（-1 表示无或多个）
 * @return 是否仅属于一个传感器的视场
 */
bool isInWhichFOV(const std::vector<FOV>& fov_vec, const PolarPoint& p_pol, int& idx);

/**
 * @brief 判断点是否在视场边缘或两个传感器视场交界处
 * @param[in] fov_vec FOV 结构向量
 * @param[in] p_pol 极坐标点
 * @param[out] idx 边缘所属传感器索引（-1 表示不在边缘）
 * @return 是否在视场边缘
 */
bool isOnEdgeOfFOV(const std::vector<FOV>& fov_vec, const PolarPoint& p_pol, int& idx);

/**
 * @brief 计算极坐标点相对视场的缩放因子（用于成本计算）
 * @param[in] fov FOV 结构向量
 * @param[in] p_pol 极坐标点（无人机朝向）
 * @return 缩放因子 [0, 1]（0 表示完全不可见，1 表示完全可见）
 */
float scaleToFOV(const std::vector<FOV>& fov, const PolarPoint& p_pol);

/**
 * @brief 计算两个极坐标点的二维距离
 * @param[in] p1 极坐标点1
 * @param[in] p2 极坐标点2
 * @return 距离值
 */
float distance2DPolar(const PolarPoint& p1, const PolarPoint& p2);

/**
 * @brief 将直方图坐标系的极坐标点转换为笛卡尔坐标点
 * @param[in] p_pol 直方图坐标系的极坐标点
 * @param[in] pos 笛卡尔坐标原点
 * @return 转换后的笛卡尔坐标点
 * @note 直方图坐标系：方位角 0 为正 Y 轴，顺时针递增；仰角向上为正
 */
Eigen::Vector3f polarHistogramToCartesian(const PolarPoint& p_pol, const Eigen::Vector3f& pos);

/**
 * @brief 将 FCU 坐标系的极坐标点转换为笛卡尔坐标点
 * @param[in] p_pol FCU 坐标系的极坐标点
 * @param[in] pos 笛卡尔坐标原点
 * @return 转换后的笛卡尔坐标点
 * @note FCU 坐标系：偏航角正方向为逆时针（从正 X 轴开始），俯仰角向前为正
 */
Eigen::Vector3f polarFCUToCartesian(const PolarPoint& p_pol, const Eigen::Vector3f& pos);

/**
 * @brief 计算两个索引角度的差值
 * @param[in] a 角度1
 * @param[in] b 角度2
 * @return 角度差
 */
float indexAngleDifference(float a, float b);

/**
 * @brief 将直方图索引转换为极坐标点
 * @param[in] e 仰角索引
 * @param[in] z 方位角索引
 * @param[in] res 直方图分辨率（度）
 * @param[in] radius 距离
 * @return 极坐标点
 */
PolarPoint histogramIndexToPolar(int e, int z, int res, float radius);

/**
 * @brief 计算两点间的直方图坐标系极向量
 * @param[in] pos 目标点坐标
 * @param[in] origin 原点坐标
 * @return 极坐标点（方位角：-180~180 度，仰角：-90~90 度）
 */
PolarPoint cartesianToPolarHistogram(const Eigen::Vector3f& pos, const Eigen::Vector3f& origin);
PolarPoint cartesianToPolarHistogram(float x, float y, float z, const Eigen::Vector3f& pos);  // 重载版本

/**
 * @brief 计算两点间的 FCU 坐标系极向量
 * @param[in] pos 目标点坐标
 * @param[in] origin 原点坐标
 * @return 极坐标点（FCU 坐标系）
 * @note 原点为 (0,0,0) 时的重载版本
 */
PolarPoint cartesianToPolarFCU(const Eigen::Vector3f& pos, const Eigen::Vector3f& origin);
PolarPoint cartesianToPolarFCU(const pcl::PointXYZ& p);

/**
 * @brief 将极坐标点转换为直方图索引
 * @param[in] p_pol 极坐标点
 * @param[in] res 直方图分辨率（度）
 * @return 索引向量（x=方位角索引，y=仰角索引）
 * @note 输入无效时返回索引 0
 */
Eigen::Vector2i polarToHistogramIndex(const PolarPoint& p_pol, int res);

/**
 * @brief 标准化极坐标角度（处理超出范围的角度）
 * @param[in/out] p_pol 极坐标点（仰角限制在 [-90,90)，方位角限制在 [-180,180)）
 */
void wrapPolar(PolarPoint& p_pol);

/**
 * @brief 计算两点间的偏航角
 * @param[in] u 起点
 * @param[in] v 终点
 * @return 偏航角（弧度）
 */
float nextYaw(const Eigen::Vector3f& u, const Eigen::Vector3f& v);

/**
 * @brief 生成位姿消息（位置+姿态）
 * @param[out] out_waypt 输出位置
 * @param[out] out_q 输出姿态（四元数）
 * @param[in] in_waypt 输入位置
 * @param[in] yaw 偏航角
 */
void createPoseMsg(Eigen::Vector3f& out_waypt, Eigen::Quaternionf& out_q, const Eigen::Vector3f& in_waypt, float yaw);

/**
 * @brief 从四元数提取偏航角
 * @param[in] q 四元数
 * @return 偏航角（度）
 */
float getYawFromQuaternion(const Eigen::Quaternionf q);

/**
 * @brief 从四元数提取俯仰角
 * @param[in] q 四元数
 * @return 俯仰角（度）
 */
float getPitchFromQuaternion(const Eigen::Quaternionf q);

/**
 * @brief 将角度标准化到 [-π, π] 范围
 * @param[in] angle 角度（弧度）
 * @return 标准化后的角度
 */
float WARN_UNUSED wrapAngleToPlusMinusPI(float angle);

/**
 * @brief 将角度标准化到 [-180, 180] 范围
 * @param[in] angle 角度（度）
 * @return 标准化后的角度
 */
float WARN_UNUSED wrapAngleToPlusMinus180(float angle);

/**
 * @brief 计算两个角度的差值（0~180 度）
 * @param[in] a 角度1（度）
 * @param[in] b 角度2（度）
 * @return 角度差
 */
float angleDifference(float a, float b);

/**
 * @brief 计算达到目标偏航角所需的角速度
 * @param[in] desired_yaw 目标偏航角（弧度）
 * @param[in] curr_yaw 当前偏航角（弧度）
 * @return 角速度（弧度/秒）
 */
double getAngularVelocity(float desired_yaw, float curr_yaw);

/**
 * @brief 将 ROS 位姿和速度消息转换为 MAVROS 轨迹消息
 * @param[out] obst_avoid 输出的 MAVROS 轨迹消息
 * @param[in] pose 输入的位姿消息
 * @param[in] vel 输入的速度消息
 */
void transformToTrajectory(mavros_msgs::Trajectory& obst_avoid, geometry_msgs::PoseStamped pose,
                           geometry_msgs::Twist vel);

/**
 * @brief 填充未使用的轨迹点（用 NAN 表示）
 * @param[in/out] point 轨迹点
 */
void fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget& point);

/**
 * @brief 将贝塞尔曲线控制点从 Eigen 类型转换为 MAVROS 消息
 * @param[out] obst_avoid 输出的 MAVROS 轨迹消息
 * @param[in] control_points 输入的 Eigen 控制点
 * @param[in] duration 曲线执行时间
 */
void transformToBezier(mavros_msgs::Trajectory& obst_avoid, const std::array<Eigen::Vector4d, 5>& control_points,
                       double duration);

/**
 * @brief 将单个贝塞尔控制点从 Eigen 转换为 MAVROS 类型
 * @param[out] point_out 输出的 MAVROS 控制点
 * @param[in] point_in 输入的 Eigen 控制点
 */
void fillControlPoint(mavros_msgs::PositionTarget& point_out, const Eigen::Vector4d& point_in);

/**
 * @brief 移除点云中的 NAN 点并计算包围盒
 * @param[in/out] cloud 输入点云（会被修改）
 * @return 包含8个顶点的包围盒点云
 */
pcl::PointCloud<pcl::PointXYZ> removeNaNAndGetMaxima(pcl::PointCloud<pcl::PointXYZ>& cloud);

/**
 * @brief 根据包围盒更新视场角（FOV）
 * @param[in/out] fov 待更新的 FOV
 * @param[in] maxima 包含8个顶点的包围盒点云
 * @note 仅当新点云指示更大视场时更新
 */
void updateFOVFromMaxima(FOV& fov, const pcl::PointCloud<pcl::PointXYZ>& maxima);

// 以下为类型转换工具函数（简化不同库/消息类型之间的转换）

// 将 geometry_msgs::Point 转换为 Eigen::Vector3f
inline Eigen::Vector3f toEigen(const geometry_msgs::Point& p) {
  Eigen::Vector3f ev3(p.x, p.y, p.z);
  return ev3;
}

// 将 geometry_msgs::Vector3 转换为 Eigen::Vector3f
inline Eigen::Vector3f toEigen(const geometry_msgs::Vector3& v3) {
  Eigen::Vector3f ev3(v3.x, v3.y, v3.z);
  return ev3;
}

// 将 pcl::PointXYZ 转换为 Eigen::Vector3f
inline Eigen::Vector3f toEigen(const pcl::PointXYZ& p) {
  Eigen::Vector3f ev3(p.x, p.y, p.z);
  return ev3;
}

// 将 pcl::PointXYZI 转换为 Eigen::Vector3f（忽略强度）
inline Eigen::Vector3f toEigen(const pcl::PointXYZI& p) {
  Eigen::Vector3f ev3(p.x, p.y, p.z);
  return ev3;
}

// 将 geometry_msgs::Quaternion 转换为 Eigen::Quaternionf
inline Eigen::Quaternionf toEigen(const geometry_msgs::Quaternion& gmq) {
  Eigen::Quaternionf eqf;
  eqf.x() = gmq.x;
  eqf.y() = gmq.y;
  eqf.z() = gmq.z;
  eqf.w() = gmq.w;
  return eqf;
}

// 将 Eigen::Vector3f 转换为 geometry_msgs::Point
inline geometry_msgs::Point toPoint(const Eigen::Vector3f& ev3) {
  geometry_msgs::Point gmp;
  gmp.x = ev3.x();
  gmp.y = ev3.y();
  gmp.z = ev3.z();
  return gmp;
}

// 将 Eigen::Vector3f 转换为 geometry_msgs::Vector3
inline geometry_msgs::Vector3 toVector3(const Eigen::Vector3f& ev3) {
  geometry_msgs::Vector3 gmv3;
  gmv3.x = ev3.x();
  gmv3.y = ev3.y();
  gmv3.z = ev3.z();
  return gmv3;
}

// 将 Eigen::Quaternionf 转换为 geometry_msgs::Quaternion
inline geometry_msgs::Quaternion toQuaternion(const Eigen::Quaternionf& eqf) {
  geometry_msgs::Quaternion q;
  q.x = eqf.x();
  q.y = eqf.y();
  q.z = eqf.z();
  q.w = eqf.w();
  return q;
}

// 将 Eigen::Vector3f 转换为 pcl::PointXYZ
inline pcl::PointXYZ toXYZ(const Eigen::Vector3f& ev3) {
  pcl::PointXYZ xyz;
  xyz.x = ev3.x();
  xyz.y = ev3.y();
  xyz.z = ev3.z();
  return xyz;
}

// 将 Eigen::Vector3f 转换为 pcl::PointXYZI（带强度）
inline pcl::PointXYZI toXYZI(const Eigen::Vector3f& ev3, float intensity) {
  pcl::PointXYZI p;
  p.x = ev3.x();
  p.y = ev3.y();
  p.z = ev3.z();
  p.intensity = intensity;
  return p;
}

// 从坐标和强度创建 pcl::PointXYZI
inline pcl::PointXYZI toXYZI(float x, float y, float z, float intensity) {
  pcl::PointXYZI p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = intensity;
  return p;
}

// 将 pcl::PointXYZ 转换为 pcl::PointXYZI（添加强度）
inline pcl::PointXYZI toXYZI(const pcl::PointXYZ& xyz, float intensity) {
  pcl::PointXYZI p;
  p.x = xyz.x;
  p.y = xyz.y;
  p.z = xyz.z;
  p.intensity = intensity;
  return p;
}

// 将线速度和角速度转换为 geometry_msgs::Twist
inline geometry_msgs::Twist toTwist(const Eigen::Vector3f& l, const Eigen::Vector3f& a) {
  geometry_msgs::Twist gmt;
  gmt.linear = toVector3(l);
  gmt.angular = toVector3(a);
  return gmt;
}

// 创建 geometry_msgs::PoseStamped 消息
inline geometry_msgs::PoseStamped toPoseStamped(const Eigen::Vector3f& ev3, const Eigen::Quaternionf& eq) {
  geometry_msgs::PoseStamped gmps;
  gmps.header.stamp = ros::Time::now();
  gmps.header.frame_id = "local_origin";  // 本地原点坐标系
  gmps.pose.position = toPoint(ev3);
  gmps.pose.orientation = toQuaternion(eq);
  return gmps;
}

// ENU 坐标系转 NED 坐标系（东-北-天 → 北-东-地）
inline Eigen::Vector3f toNED(const Eigen::Vector3f& xyz_enu) {
  Eigen::Vector3f xyz_ned;
  xyz_ned.x() = xyz_enu.y();  // 北 = 东（ENU）
  xyz_ned.y() = xyz_enu.x();  // 东 = 北（ENU）
  xyz_ned.z() = -xyz_enu.z(); // 地 = -天（ENU）
  return xyz_ned;
}

// NED 坐标系转 ENU 坐标系
inline Eigen::Vector3f toENU(const Eigen::Vector3f& xyz_ned) {
  Eigen::Vector3f xyz_enu;
  xyz_enu.x() = xyz_ned.y();  // 东 = 北（NED）
  xyz_enu.y() = xyz_ned.x();  // 北 = 东（NED）
  xyz_enu.z() = -xyz_ned.z(); // 天 = -地（NED）
  return xyz_enu;
}

// ENU 偏航角转 NED 偏航角（度）
inline float yawToNEDdeg(const float yaw_enu) { return (90.f - yaw_enu); }

// ENU 偏航角转 NED 偏航角（弧度）
inline float yawToNEDrad(const float yaw_enu) { return (M_PI / 2.f - yaw_enu); }

// ENU 俯仰角转 NED 俯仰角
inline float pitchtoNED(const float pitch_enu) { return (-pitch_enu); }

// NED 偏航角转 ENU 偏航角（度）
inline float yawToENUdeg(const float yaw_ned) { return (90.f - yaw_ned); }

// NED 偏航角转 ENU 偏航角（弧度）
inline float yawToENUrad(const float yaw_ned) { return (M_PI / 2.f - yaw_ned); }

// NED 俯仰角转 ENU 俯仰角
inline float pitchToENU(const float pitch_ned) { return (-pitch_ned); }

// 从滚转-俯仰-偏航角创建四元数
Eigen::Quaterniond quaternionFromRPY(const Eigen::Vector3d& rpy);

// 将姿态从 ENU 转换为 NED 坐标系
Eigen::Quaterniond orientationToNED(const Eigen::Quaterniond& q);

// 将姿态从 NED 转换为 ENU 坐标系
Eigen::Quaterniond orientationToENU(const Eigen::Quaterniond& q);

// 坐标系转换常量（RPY 角度）
static const Eigen::Vector3d NED_ENU_RPY(M_PI, 0, M_PI_2);         // NED→ENU 的 RPY 转换
static const Eigen::Vector3d AIRCRAFT_BASELINK_RPY(M_PI, 0, 0);    // 飞行器坐标系→基坐标系的 RPY 转换

}  // namespace avoidance

#endif  // COMMON_H