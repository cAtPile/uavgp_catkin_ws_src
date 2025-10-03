#include "apoc_pkg/apoc.h"

apoc::apoc(): rate(20.0){

    loadParams();
    
    /*****sub订阅初始化*****/
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &apoc::state_cb, this);
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &apoc::local_pos_cb, this);
    detection_data_sub = nh.subscribe<apoc_pkg::detection_data>("/detection/data",10,&apoc::detection_data_cb, this);

    /*****pub发布初始化*****/
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);  
    detection_action_pub = nh.advertise<std_msgs::Bool>("/tracker_action", 1);
    
    /*****clinet服务初始化*****/    
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //current_state
    current_state.connected = false;
    current_state.armed = false;
    current_state.mode = "STABILIZED";

    //pose
    pose.header.frame_id = "map";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    //home_pose
    home_pose.header.frame_id = "map";
    home_pose.pose.position.x = 0;
    home_pose.pose.position.y = 0;
    home_pose.pose.position.z = 0;
    home_pose.pose.orientation.x = 0.0;
    home_pose.pose.orientation.y = 0.0;
    home_pose.pose.orientation.z = 0.0;
    home_pose.pose.orientation.w = 1.0;

    //current_pose
    current_pose.header.frame_id = "map";
    current_pose.pose.position.x = 0.0;
    current_pose.pose.position.y = 0.0;
    current_pose.pose.position.z = 0.0;
    current_pose.pose.orientation.x = 0.0;
    current_pose.pose.orientation.y = 0.0;
    current_pose.pose.orientation.z = 0.0;
    current_pose.pose.orientation.w = 1.0;
    current_pose.header.stamp = ros::Time::now(); 

    //current_detection
    current_detection.detection_id = 0 ;
    current_detection.detection_x = 0 ;
    current_detection.detection_y = 0 ;

    //
    detection_action.data = false;

    //
    last_request = ros::Time::now();

}