//
// Created by wbzhang on 2020/7/2.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker points, line_strip, line_list;
geometry_msgs::PoseStamped current_pose;

// 设置
void setTripleMarkers(visualization_msgs::Marker &points,visualization_msgs::Marker &line_strip,
                      visualization_msgs::Marker &line_list){
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "map";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "showvis_node";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation = line_strip.pose.orientation = line_list.pose.orientation = current_pose.pose.orientation;

    //分配三个不同的id到三个markers。points_and_lines名称空间的使用确保彼此不会相互冲突。
    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;

    //设置marker类型到 POINTS, LINE_STRIP 和 LINE_LIST
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // scale成员对于这些marker类型是不同的,POINTS marker分别使用x和y作为宽和高，然而LINE_STRIP和LINE_LIST marker仅仅使用x，定义为线的宽度。单位是米。
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;

    // 点为绿色
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip 是蓝色
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list 为红色
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    /// 位置
    geometry_msgs::Point p;
    p.x = current_pose.pose.position.x;
    p.y = current_pose.pose.position.y;
    p.z = current_pose.pose.position.z;

    points.points.push_back(p);
    line_strip.points.push_back(p);

    // The line list needs two points for each line
    line_list.points.push_back(p);
    p.z += 1.0;
    line_list.points.push_back(p);
}

///获取当前位置,并确定是否进入着陆区域
void getLocalPoseFromMsg(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose=*msg;

    points.pose.orientation = line_strip.pose.orientation = line_list.pose.orientation = current_pose.pose.orientation;

    geometry_msgs::Point p;
    p.x = current_pose.pose.position.x;
    p.y = current_pose.pose.position.y;
    p.z = current_pose.pose.position.z;

    points.points.push_back(p);
    line_strip.points.push_back(p);

    // The line list needs two points for each line
    line_list.points.push_back(p);
    p.z += 1.0;
    line_list.points.push_back(p);
}


int main( int argc, char** argv )
{
    ros::init(argc, argv, "showvis_node");
    ROS_INFO("showvis_node start running...");
    ros::NodeHandle n;
    ros::Rate rate(30);

    // 订阅当前位置
    ros::Subscriber currPose_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",30,getLocalPoseFromMsg);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/px4/marker_path", 10);

    while (ros::ok())
    {
        //创建一个 visualization_msgs/Marker消息，并且初始化所有共享的数据。消息成员默认为0，仅仅设置位姿成员w。
        setTripleMarkers(points,line_strip,line_list);

        //发布各个markers
        marker_pub.publish(points);
        marker_pub.publish(line_strip);
        marker_pub.publish(line_list);
        ROS_INFO("lines and points are publishing...");

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("showvis_node killing...");
    return 0;
}