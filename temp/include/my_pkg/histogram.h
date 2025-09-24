#ifndef HISTOGRAM_H 
#define HISTOGRAM_H

#include <float.h>    
#include <math.h>    
#include <Eigen/Dense>
#include <vector>

namespace avoidance {

// 选择分辨率时需格外谨慎！有效的分辨率必须满足以下条件：
// 180 % (2 * ALPHA_RES) = 0 （%表示取模运算，即180能被2倍的ALPHA_RES整除）
// 有效分辨率示例：1、3、5、6、10、15、18、30、45、60
const int ALPHA_RES = 6;          // 定义角度分辨率常量（单位：度），此处设为6度
const int GRID_LENGTH_Z = 360 / ALPHA_RES;  // 方位角（Azimuth）方向的网格长度（360度/分辨率）
const int GRID_LENGTH_E = 180 / ALPHA_RES;  // 仰角（Elevation）方向的网格长度（180度/分辨率）

// 直方图类（用于存储和处理基于角度网格的障碍物距离信息）
class Histogram {

  int resolution_;  // 直方图的角度分辨率（与ALPHA_RES对应）
  int z_dim_;       // 方位角方向的网格维度（即GRID_LENGTH_Z）
  int e_dim_;       // 仰角方向的网格维度（即GRID_LENGTH_E）
  Eigen::MatrixXf dist_;  // 稠密矩阵，存储每个网格单元对应的障碍物距离（单位：米）

  /**
  * @brief     对仰角和方位角的索引进行直方图边界环绕处理（避免索引越界）
  * @param     x, 仰角（Elevation）对应的索引
  * @param     y, 方位角（Azimuth）对应的索引
  **/
  inline void wrapIndex(int &x, int &y) const {  
    x = x % e_dim_;  // 对仰角索引取模，确保在[e_dim_, 0)范围内
    if (x < 0) x += e_dim_;  // 若索引为负，加上维度值使其变为正索引
    y = y % z_dim_;  // 对方位角索引取模，确保在[z_dim_, 0)范围内
    if (y < 0) y += z_dim_;  // 若索引为负，加上维度值使其变为正索引
  }

 public:
  // 构造函数：根据传入的分辨率初始化直方图
  Histogram(const int res);
  // 析构函数：使用默认实现（因无动态分配的资源需手动释放）
  ~Histogram() = default;

  /**
  * @brief     获取直方图网格单元距离的方法（ getter ）
  * @param[in] x, 仰角对应的索引（输入参数，不可修改）
  * @param[in] y, 方位角对应的索引（输入参数，不可修改）
  * @returns   映射到(x, y)网格单元的障碍物与载具的距离（单位：米）
  **/
  inline float get_dist(int x, int y) const {
    wrapIndex(x, y);  // 先对索引进行边界环绕处理
    return dist_(x, y);  // 返回对应网格单元的距离值
  }

  /**
  * @brief     设置直方图网格单元距离的方法（ setter ）
  * @param[in] x, 仰角对应的索引（输入参数，不可修改）
  * @param[in] y, 方位角对应的索引（输入参数，不可修改）
  * @param[in] value, 映射到(x, y)网格单元的障碍物与载具的距离（单位：米）
  **/
  inline void set_dist(int x, int y, float value) { 
    dist_(x, y) = value;  // 将输入的距离值赋给对应网格单元
  }

  /**
  * @brief     计算直方图的上采样版本
  * @param[in] 调用该方法的当前对象（this指针），需是“大网格尺寸”（ALPHA_RES * 2）的直方图
  * @details   对直方图进行上采样，使其转换为“常规网格尺寸”（ALPHA_RES）的直方图。
  *            这意味着直方图矩阵的每个维度大小都会变为原来的2倍（网格更精细）
  * @returns   无返回值，直接修改调用该方法的对象，使其具备常规分辨率
  * @warning   该方法仅能由“大网格尺寸”的直方图对象调用（否则会出错）
  **/
  void upsample();

  /**
  * @brief     计算直方图的下采样版本
  * @param[in] 调用该方法的当前对象（this指针），需是“常规网格尺寸”（ALPHA_RES）的直方图
  * @details   对直方图进行下采样，使其转换为“大网格尺寸”（ALPHA_RES / 2）的直方图。
  *            这意味着直方图矩阵的每个维度大小都会变为原来的1/2（网格更粗糙）
  * @returns   无返回值，直接修改调用该方法的对象，使其具备大网格尺寸
  * @warning   该方法仅能由“常规网格尺寸”的直方图对象调用（否则会出错）
  **/
  void downsample();

  /**
  * @brief     将直方图所有网格单元的“时效”（age）和“距离”（distance）重置为0
  **/
  void setZero();

  /**
  * @brief     判断直方图是否为空（即距离层中所有网格单元的距离均不大于0）
  * @returns   布尔值：true表示直方图为空，false表示非空
  **/
  bool isEmpty() const;
};
}  // 命名空间avoidance结束

#endif