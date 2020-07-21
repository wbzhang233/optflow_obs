//
// Created by wbzhang on 2020/6/25.
//

#ifndef OPTFLOW_OBS_OPFLOWAVOIDANCECLASS_H
#define OPTFLOW_OBS_OPFLOWAVOIDANCECLASS_H

#include <iostream>
#include <string>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/PositionTarget.h>
#include<cv_bridge/cv_bridge.h>

//OpenCV2标准头文件
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

// 光流文件
#include "denseFlow.h"
#include "munsell_color.h"

using namespace std;

class opflowAvoidanceClass
{
public:
    opflowAvoidanceClass(ros::NodeHandle &nh);
    ~opflowAvoidanceClass();
    void init();
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void getLocalPoseFromMsg(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void getLocalVelFromMsg(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void switchOffboard();
    void zweven();
    // 光流处理
    void optobsCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    ros::NodeHandle n_;
    // setpoint_raw消息发布器
    ros::Publisher raw_pub;
    mavros_msgs::PositionTarget cmd_raw_msg;

    // 加锁和模式控制
    ros::Subscriber sub_,state_sub;
    mavros_msgs::State current_state;
    ros::ServiceClient arming_client,set_mode_client;
    ros::Time last_request;

    //当前位置和当前速度订阅
    ros::Subscriber local_pose_sub,local_vel_sub;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::TwistStamped current_vel;

    // 光流变量
    bool init_frame;
    cv::Mat prev_frame,curr_frame;//灰度图

};//End of class opflowAvoidanceClass


#endif //OPTFLOW_OBS_OPFLOWAVOIDANCECLASS_H
