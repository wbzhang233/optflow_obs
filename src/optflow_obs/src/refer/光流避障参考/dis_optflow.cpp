//
// Created by wbzhang233 on 2020/3/25.
//

#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <cstdio>
using namespace std;
using namespace cv;

int main(){
    cv::Mat im1 = cv::imread("/home/wbzhang233/Code/Opticalflow/data/1.jpg");//比如1.jpg
    cv::Mat im2 = cv::imread("/home/wbzhang233/Code/Opticalflow/data/2.jpg");//比如2.jpg

    cv::resize(im1,im1,cv::Size(0.5*im1.cols,0.5*im1.rows),0,0,cv::INTER_LINEAR);
    cv::resize(im2,im2,cv::Size(0.5*im2.cols,0.5*im2.rows),0,0,cv::INTER_LINEAR);


    Mat im1Gray,im2Gray;
    cvtColor(im1,im1Gray,COLOR_BGR2GRAY);
    cvtColor(im2,im2Gray,COLOR_BGR2GRAY);

    cv::Mat map;//稠密光流图
    cv::Ptr<DenseOpticalFlow> algorithm = DISOpticalFlow::create(DISOpticalFlow::PRESET_ULTRAFAST);//PRESET_MEDIUM

    cout<<"chn1:"<<im1Gray.channels()<<endl;
    cout<<"chn2:"<<im2Gray.channels()<<endl;


    algorithm->calc(im1Gray,im2Gray,map);

    cout<<"chn_map:"<<map.channels()<<endl;
    cout<<"chn_type:"<<map.type()<<endl;

    vector<Mat> flow_uv;
    split(map,flow_uv);

    namedWindow("dis_optflow");

    multiply(flow_uv[1], -1, flow_uv[1]);//翻转

    Mat mag,ang;
    Mat hsv_split[3],hsv,rgb;
    cartToPolar(flow_uv[0], flow_uv[1], mag, ang, true);//转换到极坐标,分别输出幅度和角度
    normalize(mag, mag, 0, 1, NORM_MINMAX);//归一化

    hsv_split[0] = ang;
    hsv_split[1] = mag;
    hsv_split[2] = Mat::ones(ang.size(), ang.type());
    merge(hsv_split, 3, hsv);
    cvtColor(hsv, rgb, COLOR_HSV2BGR);
    rgb.convertTo(rgb, CV_8UC3,  255, 0);

    imshow("dis_optflow",rgb);
    waitKey(0);

    return 0;
}