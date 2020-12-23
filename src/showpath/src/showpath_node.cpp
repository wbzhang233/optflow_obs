#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <vector>
using namespace std;

struct Point2f{
    float x;
    float y;
    Point2f(float _x,float _y):x(_x),y(_y){};
};

const vector<Point2f> cylinders={Point2f(18.93,-2.17),Point2f(21.49,-7.60),Point2f(23.51,5.12),Point2f(26.05,-3.94)
        ,Point2f(29.97,2.28),Point2f(33.43,6.38),Point2f(35.43,-2.73),Point2f(40.61,2.89)};
const vector<Point2f> cuboids = {Point2f(20.0,-11.99),Point2f(42.01,5.00),Point2f(43.00,0.007)};
const vector<Point2f> trees = {Point2f(56.59,-6.76)};

nav_msgs::Path path;
///获取当前位置,并确定是否进入着陆区域
geometry_msgs::PoseStamped current_pose;
void getLocalPoseFromMsg(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose=*msg;
}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "showpath_node");
    ros::NodeHandle ph;
    // 订阅当前位置
    ros::Subscriber currPose_sub = ph.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,getLocalPoseFromMsg);
    ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("/px4/local_path",1, true);

    path.header.stamp=ros::Time::now();
    path.header.frame_id="map_ned";

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        // 加入当前的位置
        path.poses.push_back(current_pose);

        // 发布路径
        path_pub.publish(path);
        ros::spinOnce();               // check for incoming messages

        loop_rate.sleep();
    }

    return 0;
}