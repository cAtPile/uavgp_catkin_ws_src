#include "avoid_planner_pkg/pointcloud_processor.h"

namespace avoid_planner {
    
/**
 * @brief 重置直方图的更新状态标志
 * @details 将标记直方图是否更新的布尔变量设为false，用于外部模块确认处理完更新后，
 *          清除更新状态，避免后续重复检测到"已更新"信号
 * 
 * @note 函数通过`updated_mutex_`互斥锁保护`is_updated_`变量，确保多线程环境下的读写安全性
 * @note 通常在外部模块调用`isUpdated()`检测到更新、且完成对新直方图数据的处理后，
 *       调用此函数重置状态，为下一次更新检测做准备
 * @note 该函数与`isUpdated()`配套使用，共同实现"更新检测-数据处理-状态重置"的完整逻辑闭环
 * 
 * @see is_updated_ 标记直方图更新状态的布尔型成员变量
 * @see updated_mutex_ 保护`is_updated_`变量的互斥锁
 * @see isUpdated() 用于检测直方图是否已更新的接口函数
 * @see pointcloudCallback 点云回调函数，会在生成新直方图后将`is_updated_`设为true
 * 
 * @warning 若在未处理完直方图数据前调用此函数，可能导致"更新数据被遗漏"的问题
 * @warning 多线程场景下必须通过此函数重置状态，禁止直接操作`is_updated_`变量，否则会引发线程安全问题
 */
void PointcloudProcessor::resetUpdatedFlag()   {
    std::lock_guard<std::mutex> lock(updated_mutex_);
    is_updated_ = false;
}

}