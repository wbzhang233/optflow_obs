//
// Created by wbzhang on 2020/2/25.
//

#include "denseFlow.h"
#include "munsell_color.h"

Mat plotDenseFlow,plotSparseFlow;
#define DENSE_FLOW false

int main(int argc, char* argv[])
{
    Mat curr_frame,prev_frame;
    Mat curr_gray,prev_gray;
    vector<Point2f> keyPts[2]; //特征点

    VideoCapture capture("vtest.avi");
    bool first_frame = true;
    int count = 0;
    if (capture.isOpened())
    {
        while (1){
            if (first_frame) {
                capture >> curr_frame;
                //resize(curr_frame,curr_frame,curr_frame.size()/2,0,0);
                first_frame = false;
                count++;
            } else {
                assert(!curr_frame.empty());
                curr_frame.copyTo(prev_frame);
                capture >> curr_frame;
                //resize(curr_frame,curr_frame,curr_frame.size()/2,0,0);

                // 灰度化
                cvtColor(curr_frame,curr_gray,COLOR_RGB2GRAY);
                cvtColor(prev_frame,prev_gray,COLOR_RGB2GRAY);

                //求特征点
                double qualityLevel = 0.01;
                double mindistance = 10;
                goodFeaturesToTrack(prev_gray,keyPts[0],MAX_CORNERS,qualityLevel,mindistance);

                /** 1-稠密光流 **/
                Mat flow;
                //clock_t start1;
                double start1=(double)getTickCount();
                calcOpticalFlowFarneback(prev_gray, curr_gray, flow, 0.5, 3, 15, 5, 5, 1.2, 0);
                //clock_t end1=clock();
                cout<<endl<<"dense_time:"<< (double)( getTickCount()-start1 )/getTickFrequency()<<endl;//0.02s
                cout << flow.size() << endl;  //对原图像每个像素都计算光流

                prev_frame.copyTo(plotDenseFlow);

                // 1-munsell_flow
                Mat munsell_flow;
                motionToColor(flow,munsell_flow);
                namedWindow("munsell_flow");
                imshow("munsell_flow",munsell_flow);

                //　计算延伸焦点FOE
//                Point2f foePt1 = getFoeFromPts(keyPts[0],flow);
//                circle(plotDenseFlow,foePt1,5,Scalar(255,0,0),-1,FILLED);

                double start2=(double)getTickCount();
                Point2f foePt = getDenseFOE(flow,15);
                cout<<endl<<"foe_time:"<< (double)( getTickCount()-start2 )/getTickFrequency()<<endl;//大概在0.72s左右
                circle(plotDenseFlow,foePt,5,Scalar(0,0,255),-1,FILLED);

                //// 计算到达时间图像TTC
                Mat ttcMap=timeToContact(flow,foePt);
                Mat ttcResult = normToGray(ttcMap);
                Mat pesudoTTC;
                applyColorMap(ttcResult,pesudoTTC,COLORMAP_JET);
                imshow("TTC",pesudoTTC);


                if(count==10){
                    imwrite("Results/ttc1.png",pesudoTTC);
                }


                for (size_t y = 0; y < prev_frame.rows; y += 10) {
                    for (size_t x = 0; x < prev_frame.cols; x += 10) {
                        Point2f fxy = flow.at<Point2f>(y, x);
                        line(plotDenseFlow, Point(x, y), Point(cvRound(x + fxy.x), cvRound(y + fxy.y)),
                             CV_RGB(0, 255, 0), 1, 8);
                    }
                }
                imshow("dense", plotDenseFlow);


                if(count==10){
                    imwrite("Results/denseflow.png",plotDenseFlow);
                }

#if DENSE_FLOW
                // 求稠密光流在各点的幅值
                Mat flowMag=Mat(flow.size(),CV_32FC1);
                for (int j = 0; j < flow.rows; ++j) {
                    for (int i = 0; i < flow.cols; ++i) {
                        Point2f fxy = flow.at<Point2f>(j, i);
                        float mag=sqrt(fxy.x*fxy.x+fxy.y*fxy.y);
                        flowMag.at<float_t >(j*flow.cols+i)=mag;
                    }
                }

                // 显示稠密光流伪彩色图
                Mat result = Mat(flow.size(),CV_8UC1);
                normalize(flowMag,result,0,255,NORM_MINMAX);//范围归一化
                result.convertTo(result,CV_8UC1,1,0);//格式转换

                Mat flowP;
                applyColorMap(result,flowP,COLORMAP_JET);
                imshow("dense_flow",flowP);
#endif

                /** 稀疏光流 **/
                TermCriteria criteria = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
                vector<uchar> status;
                vector<float> err;

                double start3=(double)getTickCount();
                calcOpticalFlowPyrLK(prev_gray, curr_gray, keyPts[0], keyPts[1], status, err, Size(15, 15), 3,
                                     criteria);
                cout<<endl<<"sparse_time:"<< (double)( getTickCount()-start3 )/getTickFrequency()<<endl;

                // 绘制FOE
                Point2f foes = getSparseFOE(keyPts[0],keyPts[1],status);
                cout<<"foes:"<<foes<<endl;

                curr_frame.copyTo(plotSparseFlow);
                circle(plotSparseFlow,foes,10,Scalar(0,255,0),-1,FILLED);

                for (size_t i = 0; i < keyPts[0].size() && i < keyPts[1].size(); i++) {
                    line(plotSparseFlow, Point(cvRound(keyPts[0][i].x), cvRound(keyPts[0][i].y)),
                         Point(cvRound(keyPts[1][i].x),
                               cvRound(keyPts[1][i].y)), Scalar(0, 50, 200), 1, LINE_AA);
                }
                imshow("sparse", plotSparseFlow);

                if(count==10){
                    imwrite("Results/sparseFlow.png",plotSparseFlow);
                }

                // 稀疏光流计算延伸焦点并且采用稠密光流场计算TTC
                Mat ttcMap2=timeToContact(flow,foes);
                Mat ttcResult2 = normToGray(ttcMap2);
                Mat pesudoTTC2;
                applyColorMap(ttcResult2,pesudoTTC2,COLORMAP_JET);
                imshow("TTC2",pesudoTTC2);

                if(count==10){
                    imwrite("Results/ttc.png",pesudoTTC2);
                }


                waitKey(10);
                count++;
            }
        }
    }

    waitKey(0);
    return 0;
}

