//
// Created by wbzhang on 2020/7/9.
//

#ifndef OPTICALFLOW_GETMOVEMENT_H
#define OPTICALFLOW_GETMOVEMENT_H

// 参考：https://blog.csdn.net/z_johnking/article/details/106022261

#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/features2d.hpp>
//#include <opencv2/opencv.hpp>
//#include "opencv2/video/tracking.hpp"
//#include "opencv2/highgui/highgui.hpp"
#include <ctime>
#include "Eigen/Dense"
#include "cstdlib"
#include "unordered_set"
#include <fstream>

using namespace cv;
using namespace std;


class Movementor{
public:
    Movementor(Mat prev,Mat curr);
    ~Movementor();

    // 0- 运行全部流程
    void run(void);

    // 1-计算特征点
    void getKeypoints(void);
    // 1-2 特征描述符
    void getDescriptor(void);

    // 3-特征匹配
    void getGoodMatches();

    // 4-RANSAC去除错误匹配
    Mat getRansacMatches();

    // 5-判断运动类型
    bool judgeMovement(Mat prev, Mat curr);
    // 6-翘曲投影

private:
    bool movement;
    int rows,cols;
    cv::Mat prev_frame,curr_frame;
    std::vector<KeyPoint> keypoints1,keypoints2;
    cv::Mat descriptors1, descriptors2;
    std::vector<cv::DMatch> matches,good_matches,ransac_matches;
    std::vector<Point2f> ransac_pts1,ransac_pts2;

    const cv::Mat K = (Mat_<float>(3,3)<<
            1144.083451, 0.0, 960.0,0.0, 1144.083451, 600.0,0.0, 0.0, 1.0); //相机的本质矩阵

    cv::Mat H; //单应性矩阵

};


#endif //OPTICALFLOW_GETMOVEMENT_H
