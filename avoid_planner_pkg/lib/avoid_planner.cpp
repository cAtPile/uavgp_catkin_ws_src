/**
 * @file avoid_planner.cpp
 * @brief 避障规划器主类实现
 * @details 初始化避障规划器核心组件，包括参数加载、ROS通信通信接口初始化、
 *          势场数据结构初始化等，是系统的入口点
 * @author apoc
 * @date 2025/10/
 */
#include "avoid_planner_pkg/avoid_planner.h"

/**
 * @brief 构造函数
 * @details 初始化极坐标场数据结构、加载配置参数、
 *          建立ROS通信接口并启动动作服务器
 */
AvoidPlanner::AvoidPlanner() : 
    rate(10.0), nh_("~"), 
    as_(nh_, "avoid_planner_action", false),
    tf_buffer_(),  
    tf_listener_(tf_buffer_)
{

    // 加载配置参数
    if (!loadParams()) {
        ROS_ERROR("Failed to load critical parameters. Exiting...");
        ros::shutdown();
        return;
    }else{
        // 展示参数
        ROS_INFO("----------------------------------------");
        ROS_INFO("AvoidPlanner Parameters Loaded:");
        ROS_INFO("  Sensor Parameters:");
        ROS_INFO("    lidar_topic: %s", lidar_topic_.c_str());
        ROS_INFO("    body_frame_id: %s", body_frame_id_.c_str());
        ROS_INFO("    lidar_frame_id: %s", lidar_frame_id_.c_str());
        ROS_INFO("    min_sensor_range: %.2f m", min_sensor_range_);
        ROS_INFO("    max_sensor_range: %.2f m", max_sensor_range_);
        
        ROS_INFO("\n  Filter Parameters:");
        ROS_INFO("    voxel_grid_size: %.2f m", voxel_grid_size_);
        ROS_INFO("    statistical_filter_mean_k: %d", statistical_filter_mean_k_);
        ROS_INFO("    statistical_filter_std_dev: %.2f", statistical_filter_std_dev_);
        
        ROS_INFO("\n  Polar Field Parameters:");
        ROS_INFO("    azimuth_resolution: %.4f rad (%.1f deg)", 
                current_polar_field_.azimuth_resolution,
                current_polar_field_.azimuth_resolution * 180 / M_PI);
        ROS_INFO("    elevation_resolution: %.4f rad (%.1f deg)", 
                current_polar_field_.elevation_resolution,
                current_polar_field_.elevation_resolution * 180 / M_PI);
        ROS_INFO("    azimuth range: [%.2f, %.2f] rad ([%.1f deg, %.1f deg])",
                current_polar_field_.min_azimuth, current_polar_field_.max_azimuth,
                current_polar_field_.min_azimuth * 180 / M_PI,
                current_polar_field_.max_azimuth * 180 / M_PI);
        ROS_INFO("    elevation range: [%.2f, %.2f] rad ([%.1f deg, %.1f deg])",
                current_polar_field_.min_elevation, current_polar_field_.max_elevation,
                current_polar_field_.min_elevation * 180 / M_PI,
                current_polar_field_.max_elevation * 180 / M_PI);
        ROS_INFO("    num_azimuth_bins: %zu", current_polar_field_.num_azimuth_bins);
        ROS_INFO("    num_elevation_bins: %zu", current_polar_field_.num_elevation_bins);
        ROS_INFO("----------------------------------------");
    }

    // 初始化势场数据结构（默认参数）
    //current_polar_field_ = PolarField();

    // 初始化ROS通信接口
    // 点云订阅
    pointcloud_sub_ = nh_.subscribe(lidar_topic_, 10,
                                   &AvoidPlanner::pointcloudCB, this);
    
    // 目标动作服务器回调注册
    as_.registerGoalCallback(boost::bind(&AvoidPlanner::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&AvoidPlanner::preemptCB, this));

    // 启动动作服务器
    as_.start();
    ROS_INFO("AvoidPlanner initialized successfully");
}

/**
 * @brief 加载配置参数
 * @details 从ROS参数服务器读取传感器参数、滤波参数和极坐标场参数，
 *          使用PolarField的含参构造函数更新极坐标场配置
 * @return 参数加载成功返回true，关键参数缺失返回false
 */
bool AvoidPlanner::loadParams() {
    
    // 传感器参数
    if (!nh_.getParam("lidar_topic", lidar_topic_)) {
        ROS_WARN("Using default lidar topic: /velodyne_points");
        lidar_topic_ = "/velodyne_points";
    }
    nh_.param("body_frame_id", body_frame_id_, std::string("base_link"));
    nh_.param("lidar_frame_id", lidar_frame_id_, std::string("lidar_link"));

    // 读取传感器范围参数
    double min_range, max_range;
    nh_.param("min_sensor_range", min_range, 0.5);  // 单位: m
    nh_.param("max_sensor_range", max_range, 50.0); // 单位: m
    min_sensor_range_ = min_range;
    max_sensor_range_ = max_range;

    // 滤波参数
    nh_.param("voxel_grid_size", voxel_grid_size_, 0.1);    // 单位: m
    nh_.param("statistical_filter_mean_k", statistical_filter_mean_k_, 50);
    nh_.param("statistical_filter_std_dev", statistical_filter_std_dev_, 1.0);

    // 读取YAML中的参数 deg
    double az_res_deg;
    double el_res_deg;
    double min_az_deg;
    double max_az_deg;
    double min_el_deg;
    double max_el_deg;

    nh_.param("azimuth_resolution_deg", az_res_deg, 1.0);    // 默认1度
    nh_.param("elevation_resolution_deg", el_res_deg, 5.0);  // 默认5度
    nh_.param("min_azimuth_deg", min_az_deg, -180.0);        // 默认-180度
    nh_.param("max_azimuth_deg", max_az_deg, 180.0);         // 默认180度
    nh_.param("min_elevation_deg", min_el_deg, 0.0);         // 从YAML读取0度
    nh_.param("max_elevation_deg", max_el_deg, 50.0);        // 从YAML读取50度

    // 单位转换：度 → 弧度
    double az_res = az_res_deg * M_PI / 180.0;
    double el_res = el_res_deg * M_PI / 180.0;
    double min_az = min_az_deg * M_PI / 180.0;
    double max_az = max_az_deg * M_PI / 180.0;
    double min_el = min_el_deg * M_PI / 180.0;
    double max_el = max_el_deg * M_PI / 180.0;

    // 使用PolarField的含参构造函数更新极坐标场配置
    current_polar_field_ = PolarField(az_res, el_res,
                                      min_az, max_az,
                                      min_el, max_el,
                                      min_range, max_range);

    ROS_INFO("Parameters loaded: Azimuth bins=%zu, Elevation bins=%zu, Sensor range=[%0.2f, %0.2f]m",
             current_polar_field_.num_azimuth_bins,
             current_polar_field_.num_elevation_bins,
             min_sensor_range_,
             max_sensor_range_);
    return true;
}

/**
 * @brief 动作目标抢占回调函数
 * @details 处理外部发送的目标抢占请求，终止当前执行的避障任务，
 *          更新状态并通知客户端
 */
void AvoidPlanner::preemptCB() {
    ROS_INFO("Goal preempted");
    as_.setPreempted();  // 通知客户端当前目标已被抢占
    is_running_ = false; // 停止当前避障规划任务的执行循环
}
// 析构函数实现
AvoidPlanner::~AvoidPlanner() {
    // 可选：添加清理逻辑（如停止线程、关闭订阅等）
}

// getCurrentPolarField()实现（返回当前极坐标场数据）
PolarField AvoidPlanner::getCurrentPolarField() {
    // 根据你的逻辑返回数据，例如直接返回成员变量
    return current_polar_field_;
}