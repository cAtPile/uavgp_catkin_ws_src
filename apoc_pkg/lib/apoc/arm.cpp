#include "apoc_pkg/apoc.h"

bool apoc::armSwitch(int arm_key) {

    bool target_arm_state = (arm_key == 1);
    std::string action = target_arm_state ? "ARM" : "DISARM";

    //处于正确状态直接退出
    if (current_state.armed == target_arm_state) {
        ROS_INFO_STREAM("Already " << action << "ED");
        return true;
    }

    ros::Duration timeout_duration(armswitch_timeout_);
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(20);  // 20Hz

    while (ros::ok() && (ros::Time::now() - start_time) < timeout_duration) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = target_arm_state;

        if (arming_client.call(arm_cmd)) {
            if (arm_cmd.response.success) {
                ROS_INFO_STREAM(action << " command accepted by FCU");
            } else {
                ROS_WARN_STREAM(action << " command rejected by FCU");
            }
        } else {
            ROS_ERROR_STREAM("Failed to call arming service");
        }

        if (current_state.armed == target_arm_state) {
            ROS_INFO_STREAM("Vehicle " << action << "ED successfully");
            return true;
        }

        ros::spinOnce();
        rate.sleep();
    }

    ROS_WARN_STREAM("Timeout while trying to " << action);
    return false;
}


