#include "avoid_planner_pkg/pointcloud_processor.h"

namespace avoid_planner {
    
/**
 * @brief 检查极坐标直方图是否已更新
 * @details 提供外部模块查询直方图状态的接口，判断是否有新的点云数据处理完成并更新了直方图
 * 
 * @return 若直方图已更新（有新点云处理结果）返回true；否则返回false
 * 
 * @note 函数通过`updated_mutex_`互斥锁保护状态变量`is_updated_`，确保多线程读写安全
 * @note 状态变量`is_updated_`通常在`pointcloudCallback`处理完点云并生成新直方图后设为true
 * @note 外部模块读取状态后，建议根据业务逻辑重置`is_updated_`（需自行实现重置接口），避免重复处理
 * 
 * @see is_updated_ 标记直方图是否更新的布尔型成员变量
 * @see updated_mutex_ 保护`is_updated_`的互斥锁
 * @see pointcloudCallback 点云回调函数，处理完点云后会更新`is_updated_`状态
 * 
 * @warning 若未正确加锁，多线程环境下可能出现"脏读"（读取到未完全更新的状态）
 * @warning 若未及时重置`is_updated_`，可能导致外部模块误判为"持续更新"，引发重复处理逻辑
 */
bool PointcloudProcessor::isUpdated() const {
    std::lock_guard<std::mutex> lock(updated_mutex_);
    return is_updated_;
}
}