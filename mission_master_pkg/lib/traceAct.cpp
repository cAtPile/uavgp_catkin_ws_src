/**
 * @file traceAct.cpp
 * @brief trace相关action
 */
#include "mission_master_pkg/mission_master.h"
#include <ros/console.h>
#include <boost/bind.hpp>

/**
 * @brief 追踪Action激活回调函数
 * @details 当追踪Action服务器开始处理目标时调用
 */
void MissionMaster::traceActiveCB() {
    ROS_INFO("Trace action server has accepted the goal and started processing");
    current_mission_state_ = ENUM_TRACE_POINT; // 更新任务状态为执行追踪
}

/**
 * @brief 追踪Action完成回调函数
 * @param state 动作客户端状态
 * @param result 追踪结果
 */
void MissionMaster::traceDoneCB(const actionlib::SimpleClientGoalState &state,
                const mission_master_pkg::TraceResultConstPtr &result) {
    ROS_INFO("Trace action finished with state: %s", state.toString().c_str());
    
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED && result->trace_success) {
        ROS_INFO("Trace mission completed successfully");
        current_mission_state_ = ENUM_TRACE_SUCCEED; // 更新状态为追踪成功
    } else {
        ROS_WARN("Trace mission failed or was preempted");
        // 失败处理逻辑可根据需求添加
    }
}

/**
 * @brief 追踪Action反馈回调函数
 * @param feedback 追踪过程中的反馈信息
 */
void MissionMaster::traceFeedbackCB(const mission_master_pkg::TraceFeedbackConstPtr &feedback) {
    ROS_INFO("Trace current step: %d", feedback->trace_current_step);
    
    // 根据步骤添加进度提示
    switch(feedback->trace_current_step) {
        case 0:
            ROS_INFO("Trace initializing...");
            break;
        case 1:
            ROS_INFO("Locking target for tracing...");
            break;
        case 2:
            ROS_INFO("Executing trace trajectory...");
            break;
        case 3:
            ROS_INFO("Maintaining trace position...");
            break;
        default:
            ROS_INFO("Trace proceeding with step %d", feedback->trace_current_step);
    }
}

/**
 * @brief 启动追踪Action
 */
void MissionMaster::traceAct() {
    // 创建并设置追踪目标
    mission_master_pkg::TraceGoal goal;
    goal.trace_enable = true; // 启动追踪

    // 发送目标并注册回调函数
    trace_clientor.sendGoal(goal,
        boost::bind(&MissionMaster::traceDoneCB, this, _1, _2),
        boost::bind(&MissionMaster::traceActiveCB, this),
        boost::bind(&MissionMaster::traceFeedbackCB, this, _1));

    ROS_INFO("Trace goal sent to action server");
}

/**
 * @brief 执行追踪任务逻辑
 * @return 追踪是否成功
 */
bool MissionMaster::traceExecute() {
    // 判断追踪Action客户端是否正在运行任务
    if (trace_clientor.getState().state_ == actionlib::SimpleClientGoalState::ACTIVE) {
        ROS_INFO("Previous trace action is still active, waiting for completion...");
        return false; // 上个追踪未完成，不重复执行
    }

    // 检查Action服务器是否就绪
    if (!trace_clientor.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("Trace action server not available within 5 seconds");
        return false;
    }

    // 发送追踪目标
    traceAct();

    // 等待任务完成（超时时间60秒，追踪任务通常耗时较长）
    if (!trace_clientor.waitForResult(ros::Duration(60.0))) {
        ROS_ERROR("Trace action timed out");
        return false;
    }

    // 获取追踪结果
    actionlib::SimpleClientGoalState state = trace_clientor.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        bool trace_succeed_result = trace_clientor.getResult()->trace_success;
        ROS_INFO("Trace action completed, success: %s", trace_succeed_result ? "true" : "false");
        return trace_succeed_result;
    } else {
        ROS_ERROR("Trace action failed with state: %s", state.toString().c_str());
        return false;
    }
}