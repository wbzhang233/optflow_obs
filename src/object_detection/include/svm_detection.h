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

