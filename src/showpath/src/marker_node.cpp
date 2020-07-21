//
// Created by wbzhang on 2020/7/2.
//

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <geometry_msgs/PoseStamped.h>
#include <ros/console.h>
#include <tf/tf.h>

geometry_msgs::PoseStamped current_pose;
visualization_msgs::Marker marker;
uint32_t shape = visualization_msgs::Marker::CUBE;

///获取当前位置

void getLocalPoseFromMsg(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose=*msg;
    marker.pose = current_pose.pose;
}

// 设置marker的属性
void setMarkerAttribute(visualization_msgs::Marker& marker){
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker_node";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    //        marker.pose.position.x = current_pose.pose.position.x;
//        marker.pose.position.y = current_pose.pose.position.y;
//        marker.pose.position.z = current_pose.pose.position.z;
//        marker.pose.orientation.x = current_pose.pose.orientation.x;
//        marker.pose.orientation.y = current_pose.pose.orientation.y;
//        marker.pose.orientation.z = current_pose.pose.orientation.z;
//        marker.pose.orientation.w = current_pose.pose.orientation.w;
    marker.pose = current_pose.pose;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"marker_node");
    ROS_INFO("marker_node start running...");

    ros::NodeHandle n;
    ros::Rate rate(30);
    // 订阅当前位置
    ros::Subscriber currPose_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,getLocalPoseFromMsg);

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/px4/visualization_marker",1);

    while(ros::ok())
    {
        setMarkerAttribute(marker);

        while(marker_pub.getNumSubscribers() < 1)
        {
            if(!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        marker_pub.publish(marker);
        ROS_INFO("Markers are publishing...");

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("marker_node killing...");
    return 0;
}