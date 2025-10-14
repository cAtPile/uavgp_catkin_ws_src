/**
 * @file planner_bridge.cpp
 * @brief 规划桥实现文件
 * @details 实现PlannerBridge类的核心逻辑，处理Action请求、点云处理和势场计算
 * @author apoc
 * @date 2025/10/7
 */
#include "avoid_planner_pkg/planner_bridge.h"

namespace avoid_planner {

/**
 * @brief 构造函数
 * 
 */
PlannerBridge::PlannerBridge(const std::string& name)
    : nh_(),
      pnh_("~"),
      as_(nh_, name, false),
      action_name_(name),
      is_running_(false),
      update_rate_(10.0)  // 默认10Hz更新频率
{
    // 注册Action回调函数
    as_.registerGoalCallback(boost::bind(&PlannerBridge::goalCallback, this));
    as_.registerPreemptCallback(boost::bind(&PlannerBridge::preemptCallback, this));

    // 加载参数
    Params params;
    if (!loadParameters(pnh_, params)) {
        ROS_WARN("部分参数加载失败，使用默认值");
    }
    update_rate_ = ros::Rate(params.planner_update_rate);

    // 初始化子模块
    pointcloud_processor_ = std::make_unique<PointcloudProcessor>(pnh_);
    potential_field_calculator_ = std::make_unique<PotentialFieldCalculator>(pnh_);

    // 创建方向发布者
    direction_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/avoid_planner/direction", 10);

    ROS_INFO_STREAM("PlannerBridge初始化完成，Action服务: " << name);
}

PlannerBridge::~PlannerBridge() {
    is_running_ = false;
    if (process_thread_.joinable()) {
        process_thread_.join();
    }
}

void PlannerBridge::start() {
    as_.start();
    ROS_INFO("PlannerBridge start");
}

void PlannerBridge::goalCallback() {
    // 接收新目标
    auto goal = as_.acceptNewGoal();
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_goal_ = *goal;

    ROS_INFO_STREAM("收到新目标: 位置(" << goal->goal_x << ", " << goal->goal_y << ", " << goal->goal_z 
                   << "), 指令: " << static_cast<int>(goal->cmd));

    // 根据指令控制运行状态
    switch (goal->cmd) {
        case 1:  // 启动
            if (!is_running_) {
                is_running_ = true;
                process_thread_ = std::thread(&PlannerBridge::processLoop, this);
                ROS_INFO("规划器已启动");
            }
            break;
        case 0:  // 停止
            is_running_ = false;
            if (process_thread_.joinable()) {
                process_thread_.join();
            }
            ROS_INFO("规划器已停止");
            break;
        case 2:  // 重置
            is_running_ = false;
            if (process_thread_.joinable()) {
                process_thread_.join();
            }
            // 重置子模块状态
            pointcloud_processor_->resetUpdatedFlag();
            current_direction_.setZero();
            ROS_INFO("规划器已重置");
            break;
        default:
            ROS_WARN("未知指令: %d", static_cast<int>(goal->cmd));
    }
}

void PlannerBridge::preemptCallback() {
    ROS_INFO_STREAM("Action被抢占: " << action_name_);
    is_running_ = false;
    if (process_thread_.joinable()) {
        process_thread_.join();
    }
    as_.setPreempted();
}

void PlannerBridge::processLoop() {
    while (is_running_ && ros::ok()) {
        // 检查Action是否被抢占
        if (as_.isPreemptRequested() || !ros::ok()) {
            as_.setPreempted();
            is_running_ = false;
            break;
        }

        // 处理点云和计算方向
        bool success = true;
        if (!updatePointcloud()) {
            ROS_WARN_THROTTLE(1.0, "点云更新失败");
            success = false;
        }

        if (success && !calculatePotentialField()) {
            ROS_WARN_THROTTLE(1.0, "势场计算失败");
            success = false;
        }

        // 发布方向和反馈
        if (success) {
            publishDirection();
            
            // 更新反馈
            std::lock_guard<std::mutex> lock(data_mutex_);
            feedback_.direction_x = current_direction_.x();
            feedback_.direction_y = current_direction_.y();
            feedback_.direction_z = current_direction_.z();
            feedback_.confidence = 1.0;  // 简化处理，实际可根据势场强度计算
            as_.publishFeedback(feedback_);
        }

        update_rate_.sleep();
    }

    // 处理结束状态
    if (is_running_) {
        result_.success = true;
        result_.message = "规划完成";
        as_.setSucceeded(result_);
    } else {
        result_.success = false;
        result_.message = "规划被终止";
        as_.setAborted(result_);
    }
}

bool PlannerBridge::updatePointcloud() {
    if (!pointcloud_processor_->isUpdated()) {
        return false;
    }

    // 获取处理后的极坐标直方图
    const PolarHistogram& hist = pointcloud_processor_->getHistogram();
    
    // 上锁更新本地缓存
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        // 转换PolarHistogram到PolarField（根据实际字段映射）
        current_polar_field_.azimuth_resolution = hist.azimuth_resolution;
        current_polar_field_.elevation_resolution = hist.elevation_resolution;
        current_polar_field_.min_azimuth = hist.min_azimuth;
        current_polar_field_.max_azimuth = hist.max_azimuth;
        current_polar_field_.min_elevation = hist.min_elevation;
        current_polar_field_.max_elevation = hist.max_elevation;
        current_polar_field_.max_range = hist.max_range;
        current_polar_field_.min_range = hist.min_range;
        current_polar_field_.num_azimuth_bins = hist.num_azimuth_bins;
        current_polar_field_.num_elevation_bins = hist.num_elevation_bins;
        current_polar_field_.dis_map = hist.data;  // 假设数据结构兼容
        current_polar_field_.timestamp = hist.timestamp;
    }

    // 重置更新标志
    pointcloud_processor_->resetUpdatedFlag();
    return true;
}

bool PlannerBridge::calculatePotentialField() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 设置当前位置和目标位置（实际应用中可能需要从MAVROS获取当前位置）
    potential_field_calculator_->setCurrentPose(
        current_polar_field_.local_position.x(),
        current_polar_field_.local_position.y(),
        current_polar_field_.local_position.z()
    );
    
    potential_field_calculator_->setGoal(
        current_goal_.goal_x,
        current_goal_.goal_y,
        current_goal_.goal_z
    );

    // 更新目标极坐标
    potential_field_calculator_->updatePolarGoal();
    
    // 生成势场
    PotentialGrid field = potential_field_calculator_->generatePotentialField();
    
    // 计算合力方向
    current_direction_ = field.force_vector;
    
    // 更新势场缓存
    current_polar_field_.pot_map = field.data;
    current_polar_field_.force_vector = current_direction_;
    
    return true;
}

void PlannerBridge::publishDirection() {
    geometry_msgs::Vector3Stamped dir_msg;
    dir_msg.header.stamp = ros::Time::now();
    dir_msg.header.frame_id = "body";  // 假设使用机体坐标系
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    dir_msg.vector.x = current_direction_.x();
    dir_msg.vector.y = current_direction_.y();
    dir_msg.vector.z = current_direction_.z();
    
    direction_pub_.publish(dir_msg);
}

}  // namespace avoid_planner