/* 目标检测与显著性检测开发主函数 objection_detection_main
* 
*/

#include <iostream>
#include <iomanip>
#include <string>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Eigen>


#include <opencv2/opencv.hpp>
#include <opencv2/xobjdetect.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/saliency.hpp>

using namespace std;
using namespace cv;
using namespace Eigen;

#define MAX_VEL 2 // 最大速度，2m/s
#define NODE_NAME "objection_detection_main"


int main(int argc, char ** argv)
{
    ros::init(argc,argv,"objection_detection_main");
    ros::NodeHandle nh("~");

    // 节点运行频率　20hz
    ros::Rate rate(20.0);

    // 话题订阅
    ros::ros::Subscriber image_sub = nh.subscribe<sensor_msgs::ImagePtr>("/*topic_name*/", 10, /*subscribe_callback_name*/);
    
    // 消息发布

    // 设置cout输出流的格式
    cout.setf(ios::fixed);
    cout<<setprecision(4);
    cout.setf(ios::left);
    cout.setf(ios::showpoint);
    cout.setf(ios::showpos);

    // 等待输入信号

    while(ros::ok() )
    {
        

        // 进入回调
        ros::spinOnce();
        // 输出提示信息
        cout<< ">>>>>>>>>>>>>>>>>>>>>>>>>>>> Object Detection <<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    }


    return 0;
}