//
// Created by wbzhang233 on 2020/3/25.
//

// Farneback dense optical flow calculate and show in Munsell system of colors
// Author : Zouxy
// Date   : 2013-3-15
// HomePage : http://blog.csdn.net/zouxy09
// Email  : zouxy09@qq.com

// API calcOpticalFlowFarneback() comes from OpenCV, and this
// 2D dense optical flow algorithm from the following paper:
// Gunnar Farneback. "Two-Frame Motion Estimation Based on Polynomial Expansion".
// And the OpenCV source code locate in ..\opencv2.4.3\modules\video\src\optflowgf.cpp


#include "munsell_color.h"


int main(int, char**argv)
{
    VideoCapture cap;
//    cap.open(0);
    cap.open(argv[1]);


    if( !cap.isOpened() )
        return -1;

    Mat prevgray, gray, flow, cflow, frame;
    namedWindow("flow", 1);

    Mat motion2color;

    for(;;)
    {
        double start_t = (double)getTickCount();

        cap >> frame;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        imshow("original", frame);

        if( prevgray.data )
        {
            calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
            motionToColor(flow, motion2color);
            imshow("flow", motion2color);
        }
        if(waitKey(10)>=0)
            break;
        std::swap(prevgray, gray);

        double cost_time = (double)getTickCount() - start_t;
        cout << "cost time: " << cost_time / (double)getTickFrequency()<<" s"<< endl;
    }

    return 0;
}