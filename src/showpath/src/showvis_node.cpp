//
// Created by wbzhang on 2020/7/2.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <nav_msgs/Path.h>

using namespace std;

struct Point2f{
    float x;
    float y;
    Point2f(float _x,float _y):x(_x),y(_y){};
    Point2f Add(Point2f ano){return Point2f(ano.x+x,ano.y+y);};
};

const vector<Point2f> cylinders ={Point2f(18.93,-2.17),Point2f(21.49,-7.60),Point2f(23.51,5.12),Point2f(26.05,-3.94),
                                  Point2f(29.97,2.28),Point2f(33.43,6.38),Point2f(35.43,-2.73),Point2f(40.61,2.89)};
const vector<Point2f> cuboids = {Point2f(20.0,-11.99),Point2f(42.01,5.00),Point2f(43.00,0.007)};
const vector<Point2f> trees = {Point2f(56.59,-6.76),Point2f(60,-10.25)};

visualization_msgs::Marker points, line_strip, line_list;
visualization_msgs::Marker cylinder_marker,cuboid_marker,tree_marker;
geometry_msgs::PoseStamped current_pose,mission_pose;
nav_msgs::Path path,mission_path;

// 初始化圆柱标识
void initCylinderMarkerPose(visualization_msgs::Marker &marker,int i){
    marker.header.frame_id = "/map";
    marker.header.stamp= ros::Time::now();
    marker.ns = "showvis_node";
    marker.id = i+2;
    marker.lifetime = ros::Duration();
    // 形状、尺寸、颜色
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x =0.3; marker.scale.y = 0.3; marker.scale.z = 1.0;
    marker.color.r = 1.0f; marker.color.g = 0.2f; marker.color.b = 0.2f; marker.color.a = 1.0;
    // 位置
    marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = marker.pose.orientation.w = 0;
    marker.pose.position.x = cylinders[i].x+5;
    marker.pose.position.y = cylinders[i].y;
    marker.pose.position.z = 0;
}

// 初始化立方体标识
void initCuboidMarkerPose(visualization_msgs::Marker &marker,int i){
    marker.header.frame_id = "/map";
    marker.header.stamp= ros::Time::now();
    marker.ns = "showvis_node";
    marker.id = cylinders.size()+i+2;
    // 形状、尺寸、颜色
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x =1.2; marker.scale.y = 1.2; marker.scale.z = 2.0;
    marker.color.r = 0.2f; marker.color.g = 1.0f; marker.color.b = 0.2f; marker.color.a = 1.0;
    // 位置
    marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = marker.pose.orientation.w = 0;
    marker.pose.position.x = cuboids[i].x+5;
    marker.pose.position.y = cuboids[i].y;
    marker.pose.position.z = 0;
}

// 初始化树形表示
void initTreeMarkerPose(visualization_msgs::Marker &marker,int i){
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "showvis_node";
    marker.id = cylinders.size()+ cuboids.size()+ i+2;
    // 形状、尺寸、颜色
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x =2.0; marker.scale.y = 2.0; marker.scale.z = 3.0;
    marker.color.r = 0.2f; marker.color.g = 0.2f; marker.color.b = 1.0f; marker.color.a = 1.0;
    // 位置
    marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = marker.pose.orientation.w = 0;
    marker.pose.position.x = trees[i].x+5;
    marker.pose.position.y = trees[i].y;
    marker.pose.position.z = 0;
}


// 设置
void setTripleMarkers(visualization_msgs::Marker &points,visualization_msgs::Marker &line_strip,
                      visualization_msgs::Marker &line_list){
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/map";
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

///获取当前位置
void getLocalPoseFromMsg(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose=*msg;
    ROS_INFO("Received local pose......");
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

// 获取录制的任务航点的轨迹
void getMissionPoseFromMsg(const geometry_msgs::PoseStamped::ConstPtr& msg){
    mission_pose=*msg;
}


int main( int argc, char** argv )
{
    ros::init(argc, argv, "showvis_node");
    ROS_INFO("showvis_node start running...");
    ros::NodeHandle n;
    ros::Rate rate(30);

    // 订阅当前位置
    ros::Subscriber currPose_sub = n.subscribe<geometry_msgs::PoseStamped>("/px4/mission_pose",30,getMissionPoseFromMsg);
    ros::Subscriber missionPose_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",30,getLocalPoseFromMsg);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/px4/marker_path", 10);
    ros::Publisher obs_marker_pub = n.advertise<visualization_msgs::Marker>("/obs/markers", 15);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/px4/local_path",1, true);
    ros::Publisher missionpath_pub = n.advertise<nav_msgs::Path>("/px4/mission_path",1, true);



    // 障碍物标识
    vector<visualization_msgs::Marker> cylinder_markers(cylinders.size());
    vector<visualization_msgs::Marker> cuboid_markers(cuboids.size());
    vector<visualization_msgs::Marker> tree_markers(trees.size());

    while (ros::ok())
    {
////         先把障碍物的Marker发送出去
        for(int i=0;i<cylinders.size();++i){
            initCylinderMarkerPose(cylinder_markers[i],i);
            obs_marker_pub.publish(cylinder_markers[i]);
        }
        for(int i=0;i<cuboids.size();++i){
            initCuboidMarkerPose(cuboid_markers[i],i);
            obs_marker_pub.publish(cuboid_markers[i]);
        }
        for(int i=0;i<trees.size();++i){
            initTreeMarkerPose(tree_markers[i],i);
            obs_marker_pub.publish(tree_markers[i]);
        }

//        // 发布Marker路径
//        setTripleMarkers(points,line_strip,line_list);
//        marker_pub.publish(points);
//        marker_pub.publish(line_strip);
//        marker_pub.publish(line_list);

        path.header.stamp=ros::Time::now();
        path.header.frame_id="/map";
        path.poses.push_back(current_pose);
        path_pub.publish(path);

        mission_path.header.stamp=ros::Time::now();
        mission_path.header.frame_id="/map";
        mission_path.poses.push_back(mission_pose);
        missionpath_pub.publish(mission_path);
        ROS_INFO("PX4 Markers are publishing...");


        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("showvis_node killing...");
    return 0;
}