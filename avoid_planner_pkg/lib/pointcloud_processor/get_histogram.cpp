#include "avoid_planner_pkg/pointcloud_processor.h"

namespace avoid_planner {
/**
 * @brief 获取极坐标直方图的只读引用
 * @details 提供对内部极坐标直方图数据的只读访问，确保多线程环境下的数据安全性
 * 
 * @return 返回PolarHistogram类型的常量引用，包含直方图的参数和数据
 * 
 * @note 函数通过互斥锁(histogram_mutex_)保护直方图数据，防止在读取时被修改
 * @note 返回的是常量引用，调用者无法直接修改直方图内容，保证数据完整性
 * @note 适用于需要读取直方图数据进行避障决策或可视化的场景
 * 
 * @see histogram_ 存储极坐标直方图数据和参数的成员变量
 * @see histogram_mutex_ 保护直方图数据的互斥锁
 * @see PolarHistogram 极坐标直方图的数据结构定义
 * 
 * @warning 调用者应避免长时间持有返回的引用，以免阻塞写入操作
 * @warning 若需长期使用直方图数据，建议创建副本而非持有引用
 */
const PolarHistogram& PointcloudProcessor::getHistogram() const {
    std::lock_guard<std::mutex> lock(histogram_mutex_);
    return histogram_;
}

}