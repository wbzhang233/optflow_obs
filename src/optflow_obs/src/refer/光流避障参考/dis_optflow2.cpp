//
// Created by wbzhang233 on 2020/3/25.
//

//#include "opencv2/core/utility.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/imgproc.hpp"
//#include "opencv2/video.hpp"
//#include <iostream>
//using namespace std;
//using namespace cv;

#include "munsell_color.h"
#include "denseFlow.h"
#include <stack>


static void help()
{
    printf("Usage: dis_optflow <video_file>\n");
}

double getAvgTenFps(queue<double > freqs){
    double sum = 0 ;
    for (int i = 0; i < freqs.size(); ++i) {
        sum +=freqs.front();
        freqs.pop();
    }
    return sum/freqs.size();
}


int main(int argc, char **argv)
{
    VideoCapture cap;
    if (argc < 2)
    {
        help();
        exit(1);
    }
    cap.open(argv[1]);
    if(!cap.isOpened())
    {
        printf("ERROR: Cannot open file %s\n", argv[1]);
        return -1;
    }
    Mat prevgray, gray, rgb, frame;
    // 1-dis光流
    Mat flow, flow_uv[2];//分别是u\v两个方向
    Mat mag, ang;
    Mat hsv_split[3], hsv;
    Mat plotFOE,plotFOEFb;

    // 2-farneback光流
    Mat flow_Farneback;
    Mat flow_uv_Farneback[2];
    Mat mag_Farneback, ang_Farneback;
    Mat hsv_split_Farneback[3], hsv_Farneback;
    Mat rgb_Farneback;

    queue<double> freqs;
    Ptr<DenseOpticalFlow> algorithm = DISOpticalFlow::create(DISOpticalFlow::PRESET_MEDIUM);
    /**
     * PRESET_ULTRAFAST 0 超快 460*614 260HZ
     * PRESET_FAST 1 快速 460*614 ~155HZ
     * PRESET_MEDIUM 2 中等 460*614 ~55HZ
     */

    int idx = 0;
    while(true)
    {
        cap >> frame;
        if (frame.empty())
            break;
        // 调整大小
        cv::resize(frame,frame,cv::Size(0.8*frame.cols,0.8*frame.rows),0,0,cv::INTER_LINEAR);

        frame.copyTo(plotFOE);
        frame.copyTo(plotFOEFb);

        idx++;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        cv::imshow("origin", frame);


        if (!prevgray.empty())
        {
            cout<<"image_size:"<<prevgray.rows<<","<<prevgray.cols<<endl;

#if 1
            /** 1- DISOpticalFlow **/
            /*main function of DISOpticalFlow*/
            double start_dis= getTickCount();
            algorithm->calc(prevgray, gray, flow);//0-真正的计算光流

            cout<<"type:"<<flow.type()<<endl;


            double dis_time = (getTickCount()-start_dis)/getTickFrequency();
            cout<<"DIS has spent:"<<dis_time<<" s"<<endl;
//                <<" FPS:"<<1/dis_time<<"Hz"<< endl;
            if(freqs.size()<11){
                freqs.push(1/dis_time);
            }else{
                cout<<"FPS:"<<getAvgTenFps(freqs)<<endl;
                freqs.pop();
                freqs.push(1/dis_time);
            }

            // 1-1 光流可视化
            split(flow, flow_uv);

            multiply(flow_uv[1], -1, flow_uv[1]);//翻转

            cartToPolar(flow_uv[0], flow_uv[1], mag, ang, true);//转换到极坐标,分别输出幅度和角度
            normalize(mag, mag, 0, 1, NORM_MINMAX);//归一化

            hsv_split[0] = ang;
            hsv_split[1] = mag;
            hsv_split[2] = Mat::ones(ang.size(), ang.type());//角度\幅度\全1构成HSV图像
            merge(hsv_split, 3, hsv);//转化成RGB进行显示
            cvtColor(hsv, rgb, COLOR_HSV2BGR);

            cv::Mat rgbU;
            rgb.convertTo(rgbU, CV_8UC3,  255, 0);
            cv::imshow("DISOpticalFlow", rgbU);

//            // 1-2 光流结果掩码
//            Mat rgbU_b = rgbU.clone();
//            Mat split_dis[3];
//            split(rgbU_b, split_dis);
//
//            split_dis[2] = prevgray;//把光流RGB可视化的第三个通道替换成先前的灰度图
//            Mat merge_dis;
//            merge(split_dis, 3, merge_dis);
//            cv::imshow("DISOpticalFlow_mask", merge_dis);

//            // 1-3 munsell颜色系统光流可视化
//            Mat munsell_flow;
//            motionToColor(flow, munsell_flow);
//            imshow("munsell_flow", munsell_flow);

            // 1-4 计算FOE
            double foe_start1= getTickCount();
            Point2f foe = getDenseFOE(flow,Size(flow.rows/32,flow.cols/32));
            double foe_time1 = (getTickCount()-foe_start1)/getTickFrequency();
            cout<<"FOE1 has spent:"<<foe_time1<<" s"<<endl;
            cout<<"FOE1 is:"<<foe.x<<","<<foe.y<<endl;
            circle(plotFOE,foe,6,Scalar(255,0,255),-1,FILLED);//品红

            // 1-4-2 迭代计算FOE
            double foe_start2= getTickCount();
            Point2f rfoe = getIterativeFOE(flow);
            double foe_time2 = (getTickCount()-foe_start2)/getTickFrequency();
            cout<<"FOE2 has spent:"<<foe_time2<<" s"<<endl;
            cout<<"FOE2 is:"<<rfoe.x<<","<<rfoe.y<<endl;
            circle(plotFOE,rfoe,3,Scalar(0,255,0),-1,FILLED);//绿色

            // 1-4-3 计算全图最小二乘的FOE
//            Point2f gt_foe = getDenseFOE(flow,1);
//            circle(plotFOE,gt_foe,5,Scalar(255,0,0),-1,FILLED);

            namedWindow("FOE");
            imshow("FOE",plotFOE);

//            // 1-5 计算TTC
//            Mat ttcMap = timeToContact(flow,foe);
//            Mat ttcResult = normToGray(ttcMap);
////            cout<<"type:"<<ttcResult.type()<<endl;
//            Mat pesudoTTC;
//            applyColorMap(ttcResult,pesudoTTC,COLORMAP_JET);
//            imshow("TTC",pesudoTTC);
//
//            if(idx==10){
//                imwrite("Results/dis_ttc1.png",pesudoTTC);
//            }

#endif

#if 0
            /** 2- Farneback 稠密光流法**/
            double start_fb = getTickCount();
            cv::calcOpticalFlowFarneback(prevgray, gray, flow_Farneback, 0.5, 3,15, 3, 5, 1.2, 0);
            double fb_time = (getTickCount()-start_dis)/getTickFrequency();
            cout<<"farneback has spent:"<<fb_time<<" s"<<endl;

            // 2-1 farneback 结果显示
            split(flow_Farneback, flow_uv_Farneback);
            multiply(flow_uv_Farneback[1], -1, flow_uv_Farneback[1]);
            cartToPolar(flow_uv_Farneback[0], flow_uv_Farneback[1], mag_Farneback, ang_Farneback, true);
            normalize(mag_Farneback, mag_Farneback, 0, 1, NORM_MINMAX);
            hsv_split_Farneback[0] = ang_Farneback;
            hsv_split_Farneback[1] = mag_Farneback;
            hsv_split_Farneback[2] = Mat::ones(ang_Farneback.size(), ang_Farneback.type());
            merge(hsv_split_Farneback, 3, hsv_Farneback);
            cvtColor(hsv_Farneback, rgb_Farneback, COLOR_HSV2BGR);

            // 2-2 显示mask
            cv::Mat rgbU_Farneback;
            rgb_Farneback.convertTo(rgbU_Farneback, CV_8UC3,  255, 0);
            cv::imshow("FlowFarneback", rgbU_Farneback);
            Mat rgbU_Farneback_b = rgbU_Farneback.clone();
            Mat split_Fb[3];
            split(rgbU_Farneback_b, split_Fb);
            split_Fb[2] = prevgray;
            Mat merge_Fb;
            merge(split_Fb, 3, merge_Fb);
            cv::imshow("FlowFarneback_mask", merge_Fb);

            // 2-3 计算FOE
            Point2f foe_fb = getDenseFOE(flow_Farneback,15);
            circle(plotFOEFb,foe_fb,5,Scalar(0,0,255),-1,FILLED);
            namedWindow("FOE");
            imshow("FOE-fb",plotFOEFb);

            // 2-4 计算TTC
            Mat ttcMapFb = timeToContact(flow_Farneback,foe_fb);
            Mat ttcResultFb = normToGray(ttcMapFb);
//            cout<<"type:"<<ttcResult.type()<<endl;
            Mat pesudoTTCFB;
            applyColorMap(ttcResultFb,pesudoTTCFB,COLORMAP_JET);
            imshow("TTC-fb",pesudoTTCFB);

            if(idx==10){
                imwrite("Results/fb_ttc.png",pesudoTTCFB);
            }
#endif

            cv::waitKey(10);
        }


        std::swap(prevgray, gray);
    }

    return 0;
}