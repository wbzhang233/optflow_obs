//
// Created by wbzhang on 2020/6/11.
//

/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/Thrust.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

bool velControl = false;
void isVelControl(const geometry_msgs::Twist::ConstPtr& msg){
    if(msg== nullptr){
        velControl = false;
    }
    else if(msg->linear.x!=0 || msg->linear.y!=0){
        velControl = true;
    }else{
        velControl = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber vel_control_sub = nh.subscribe<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10, isVelControl);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

//    ros::Publisher pub_thrust = nh.advertise<mavros_msgs::Thrust>("mavros/setpoint_attitude/thrust", 30);
//    ros::Publisher pub_attitude = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude", 30);
//    ros::Publisher pub_vel = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_attitude/cmd_vel", 30);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 15;
    pose.pose.position.y = 2.5;
    pose.pose.position.z = 1.6;

//    mavros_msgs::Thrust cmd_thrust;
//    geometry_msgs::PoseStamped cmd_attitude;
//    geometry_msgs::TwistStamped cmd_att_vel;
//    // 油门
//    cmd_thrust.thrust = 0.1;
//    // 姿态
//    cmd_attitude.pose.position.x = 0.0;
//    cmd_attitude.pose.position.y = 0.0;
//    cmd_attitude.pose.position.z = 0.0;
//    cmd_attitude.pose.orientation.x = 0.0;
//    cmd_attitude.pose.orientation.y = 0.0;
//    cmd_attitude.pose.orientation.z = 0.0;
//    cmd_attitude.pose.orientation.w = 0.0;
//    // 姿态速度
//    cmd_att_vel.twist.angular.x = 0.0;
//    cmd_att_vel.twist.angular.y = 0.0;
//    cmd_att_vel.twist.angular.z = 0.0;
//    cmd_att_vel.twist.linear.x = 0.0;
//    cmd_att_vel.twist.linear.y = 1.0;
//    cmd_att_vel.twist.linear.z = 0.0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
//        pub_thrust.publish(cmd_thrust);
//        pub_attitude.publish(cmd_attitude);
//        pub_vel.publish(cmd_att_vel);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if(!velControl){
            local_pos_pub.publish(pose);
        }
//        pub_thrust.publish(cmd_thrust);
//        pub_attitude.publish(cmd_attitude);
//        pub_vel.publish(cmd_att_vel);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}