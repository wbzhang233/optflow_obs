/*
 * @Author: wbzhang 
 * @Date: 2020-12-29 16:25:26 
 * @Last Modified by: wbzhang
 * @Last Modified time: 2020-12-29 16:31:42
 */


#ifndef UTILITIES_H
#define UTILITIES_H

#include <iostream>
#include <fstream>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include <unordered_set>

#include <opencv4/opencv2/opencv.hpp>
#include "opencv4/opencv2/video/tracking.hpp"
#include "opencv4/opencv2/highgui/highgui.hpp"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

using namespace std;
using namespace cv;
using namespace Eigen;

struct ValidFlowPts
{
    vector<Point2f> pts;
    vector<Point2f> flows;
    unordered_set<int> index;
    void clear()
    {
        pts.clear();
        flows.clear();
        index.clear();
    };
};

struct PtsFlowFoe
{
    vector<Point2f> pts;   //最终估计出的内点集合
    vector<Point2f> flows; // 内点集的光流
    Point2f foe;           // 最终估计出的FOE
    vector<double> diff;   //误差
    void clear()
    {
        pts.clear();
        flows.clear();
        foe = Point2f(0, 0);
        diff.clear();
    };
};

// 将Opencv_mat打印到txt
void printMatToTxt(Mat img, string save_name)
{
    ofstream ofstream1;
    ofstream1.open(save_name);

    // 此处限定img为CV_32FC1
    for (int i = 0; i < img.rows; ++i)
    {
        for (int j = 0; j < img.cols; ++j)
        {
            float value = img.at<float>(i, j);
            ofstream1 << to_string(value) << " ";
        }
        ofstream1 << endl;
    }

    ofstream1.close();
    cout << save_name << " has saved!" << endl;
}

/** 计算两个向量的夹角,
 * @tparam T 点的类型
 * @param pt1 点1代表向量1
 * @param pt2 点2代表向量2
 * @param method:0为角度,1为弧度
 * @return 点1向量旋转到点2向量的角度,不考虑顺逆方向的夹角最小值;0~pi之间
 */

template <typename T>
double_t getAngleDiff(T pt1, T pt2, int method = 0)
{
    double_t theta1 = atan2(pt1.y, pt1.x); //返回(-M_PI,M_PI)之间的反正切弧度值
    double_t theta2 = atan2(pt2.y, pt2.x);
    double_t result = abs(theta2 - theta1) > M_PI ? 2 * M_PI - abs(theta2 - theta1) : abs(theta2 - theta1);
    if (method == 1)
    {
        return result;
    }
    return result * 180 / M_PI;
}

template <typename T>
double_t getAngleDiff2(T pt1, T pt2)
{
    double_t mag1 = sqrt(pt1.x * pt1.x + pt1.y * pt1.y);
    double_t mag2 = sqrt(pt2.x * pt2.x + pt2.y * pt2.y);
    double_t dotmul = (pt1.x * pt2.x + pt1.y * pt2.y) / (mag1 * mag2);

    return acos(dotmul) * 180 / M_PI;
}


/// 随机取下标数组
unordered_set<int> getRandomIndex(int maxNum, int num)
{
    unordered_set<int> result;

    while (result.size() < num)
    {
        int ind = rand() / maxNum;
        if (result.find(ind) == result.end())
        {
            result.insert(ind);
        }
    }
    return result;
}

/// 随机取子集
ValidFlowPts getRandomSubPts(vector<Point2f> flows, vector<Point2f> pts, int maxNum, int num)
{
    ValidFlowPts res;
    res.flows.clear();
    res.pts.clear();
    res.index.clear();

    while (res.index.size() < num)
    {
        int ind = rand() % maxNum;
        if (res.index.find(ind) == res.index.end())
        {
            res.index.insert(ind);
            res.flows.push_back(flows[ind]);
            res.pts.push_back(pts[ind]);
        }
    }
    return res;
}

/// 将单通道图像归一化成uint8的灰度图
Mat normToGray(Mat img)
{
    if (img.channels() != 1)
    {
        cout << "please input an one-channel image!" << endl;
        return img;
    }
    Mat result = Mat(img.size(), CV_8UC1);
    normalize(img, result, 0, 255, NORM_MINMAX);
    result.convertTo(result, CV_8UC1, 1, 0); //格式转换

    return result;
}

#endif //UTILITIES_H