/**
 * @file mission_master_node.cpp
 * @brief 
 * @date
 */
#include<ros/ros.h>
//其他头文件

//构建任务枚举
enum mission_state{

ENUM_WATTING_TAKEOFF, //等待起飞

ENUM_TAKEOFF, //起飞
ENUM_TAKEOFF_SUCCEED, //起飞成功

ENUM_FLYTO_PICKUP_POINT, //飞行到拾取点
ENUM_PICKUP_POINT, //拾取点
ENUM_PICKUP_SUCCEED, //拾取成功

ENUM_FLYTO_AVOID_POINT, //飞行到避障点
ENUM_AVOID_POINT, //避障点
ENUM_AVOID_SUCCEED, //避障成功

ENUM_FLYTO_TRACE_POINT, //飞行到追踪点
ENUM_TRACE_POINT, //追踪点
ENUM_TRACE_SUCCEED, //追踪成功

ENUM_FLYTO_LAND_POINT, //飞行到降落点
ENUM_LAND_POINT, //降落点
ENUM_LAND_SUCCEED, //降落成功
}

/**
 * @class MissionMaster
 * @brief 任务主控
 * @details
 */
class MissionMaster{
private:
    //=========数据缓存=============
    //ros参量

    //--------(数据缓存)------------

    void loadParams();//参数加载
    void armCheckCB();//解锁检查回调
    void stateCheck();//状态检测
    void pickCilent();//调用拾取服务
    void avoidCilent();//调用避障服务
    void traceCilent();//调用跟踪服务

public:
    MissionMaster(/* args */);
    ~MissionMaster();
};

MissionMaster::MissionMaster(/* args */)
{
}

MissionMaster::~MissionMaster()
{
}


