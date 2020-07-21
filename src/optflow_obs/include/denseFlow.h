//
// Created by wbzha on 2020/2/25.
//

#ifndef OPTICALFLOW_DENSEFLOW_H
#define OPTICALFLOW_DENSEFLOW_H

#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include "opencv4/opencv2/video/tracking.hpp"
#include "opencv4/opencv2/highgui/highgui.hpp"
#include <ctime>
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "cstdlib"
#include "unordered_set"
#include <fstream>
#include <math.h>

//#pragma comment(lib,"opencv_core310.lib")
//#pragma comment(lib,"opencv_highgui310.lib")
//#pragma comment(lib,"opencv_video310d.lib")
//#pragma comment(lib,"opencv_imgproc310.lib")
//#pragma comment(lib,"opencv_features2d310.lib")

#define MAX_CORNERS 1500

using namespace std;
using namespace cv;
using namespace Eigen;

struct ValidFlowPts{
    vector<Point2f> pts;
    vector<Point2f> flows;
    unordered_set<int> index;
    void clear(){pts.clear();flows.clear();index.clear();};
};

struct PtsFlowFoe{
    vector<Point2f> pts;//最终估计出的内点集合
    vector<Point2f> flows; // 内点集的光流
    Point2f foe; // 最终估计出的FOE
    vector<double> diff; //误差
    void clear() { pts.clear();flows.clear();foe=Point2f(0,0);diff.clear();};
};

PtsFlowFoe getSparseRansacFOE1(vector<Point2f> Pts,vector<Point2f> flows,float &loss_final,int iterTimes=200,double epislon=10.0);


// 将Opencv_mat打印到txt
void printMatToTxt(Mat img,string save_name)
{
    ofstream ofstream1;
    ofstream1.open(save_name);

    // 此处限定img为CV_32FC1
    for (int i = 0; i < img.rows; ++i) {
        for (int j = 0; j < img.cols; ++j) {
            float value =img.at<float>(i,j);
            ofstream1 <<to_string(value)<<" ";
        }
        ofstream1<<endl;
    }

    ofstream1.close();
    cout<<save_name<<" has saved!"<<endl;
}

/** 1 稠密光流计算 **/

/// 1-0 方形区块采点
ValidFlowPts getMBlockPts(Mat flow,int levels=4){
    ValidFlowPts res;

    Size2f blockSize;
    for(int i=0;i<levels;i++){
        blockSize.height = flow.rows/pow(2,i+2);
        blockSize.width =  flow.cols/pow(2,i+2);

        for (size_t y = 0; y < flow.rows; y += blockSize.height) {
            for (size_t x = 0; x < flow.cols; x += blockSize.width) {
                Point2f fxy = flow.at<Point2f>(y, x);
                res.pts.push_back(Point2f(x,y));
                res.flows.push_back(fxy);
            }
        }
    }

    return res;
}

ValidFlowPts getBlockPts(Mat flow,int levels=3){
    ValidFlowPts res;

    Size2f blockSize;
    blockSize.height = flow.rows/pow(2,levels+2);
    blockSize.width =  flow.cols/pow(2,levels+2);

    for (size_t y = 0; y < flow.rows; y += blockSize.height) {
        for (size_t x = 0; x < flow.cols; x += blockSize.width) {
            Point2f fxy = flow.at<Point2f>(y, x);
            res.pts.push_back(Point2f(x,y));
            res.flows.push_back(fxy);
//            cout<<"x:"<<x <<" y:"<<y<<endl;
        }
    }

    return res;
}

/// 1-1 从光流场图像计算FOE,方形区块采点
Point2f getDenseFOE(Mat flow,int blocksize){
    // 计算A和b
    int pts_size = (flow.rows/blocksize)*(flow.cols/blocksize);
    cout<<"pts_size:"<<pts_size<<endl;
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(pts_size,2);
    Eigen::VectorXf b = Eigen::VectorXf::Zero(pts_size);

    for (size_t y = 0; y < flow.rows; y += blocksize) {
        for (size_t x = 0; x < flow.cols; x += blocksize) {
            Point2f fxy = flow.at<Point2f>(y, x);
            int ind = (flow.cols/blocksize)*y/blocksize + x/blocksize;
            if(ind<pts_size){
                A(ind,0) = fxy.y;
                A(ind,1) = -fxy.x;
                b(ind) = fxy.y*x-fxy.x*y;
            }
        }
    }
    Eigen::MatrixXf temp1 = A.transpose()*A;
    Eigen::MatrixXf foe = temp1.inverse()*A.transpose()*b;
    cout<<"FOE:"<<foe(0,0)<<","<<foe(1,0)<<" one time..."<<endl;

    return Point2f(foe(0,0),foe(1,0));
}

/// 1-2 从光流场图像计算FOE,矩形区块采点
Point2f getDenseFOE(Mat flow,Size blocksize){
    // 计算A和b
    int pts_size = (flow.rows/blocksize.height)*(flow.cols/blocksize.width);
//    cout<<"pts_size:"<<pts_size<<endl;
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(pts_size,2);
    Eigen::VectorXf b = Eigen::VectorXf::Zero(pts_size);

    for (size_t y = 0; y < flow.rows; y += blocksize.height) {
        for (size_t x = 0; x < flow.cols; x += blocksize.width) {
            Point2f fxy = flow.at<Point2f>(y, x);
            int ind = (flow.cols/blocksize.width)*y/blocksize.height + x/blocksize.width;
            if(x==flow.cols ||y==flow.rows){
                cout<<"test:"<<ind<<endl;
            }
            if(ind<pts_size){
                A(ind,0) = fxy.y;
                A(ind,1) = -fxy.x;
                b(ind) = fxy.y*x-fxy.x*y;
//                cout<<"ind:"<<ind<<endl;
            }
        }
    }
//    cout<<"A:\n"<<A<<endl;
    Eigen::MatrixXf temp1 = A.transpose()*A;
    Eigen::MatrixXf foe = temp1.inverse()*A.transpose()*b;
//    cout<<"FOE:"<<foe(0,0)<<","<<foe(1,0)<<" one time..."<<endl;

    return Point2f(foe(0,0),foe(1,0));
}

/// 1-3 从序列点集中计算FOE
Point2f getDenseFoeFromPts(vector<Point2f> pts,Mat flow)
{
    assert(flow.channels()==2 && pts.size()!=0);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(pts.size(),2);
    Eigen::VectorXf b = Eigen::VectorXf::Zero(pts.size());

    // 从1500个点中抽取200个
    for (int i = 0; i < pts.size(); ++i) {
        Point2f fxy = flow.at<Point2f>(pts[i].x,pts[i].y);
        A(i,0) = fxy.y;
        A(i,1) = -fxy.x;
        b(i) = fxy.y*pts[i].x-fxy.x*pts[i].y;
    }

    Eigen::MatrixXf temp=A.transpose()*A;
    Eigen::MatrixXf foe = temp.inverse()*A.transpose()*b;

    return Point2f(foe(0,0),foe(1,0));
}

/// 1-4 迭代最小二乘,计算FOE
Point2f getIterativeFOE(Mat flow,int iterTimes=4,double epsilon=5.0){
    assert(flow.channels()==2);
    Point2f foe;

    // 0-生成分布的区块大小,大块逐渐到小块,最大为图像的1/16
    iterTimes = iterTimes>5 ? 5:iterTimes;
    vector<Size> blockSizes(iterTimes,Size(0,0));
    for(int i=0;i<iterTimes;i++){
        blockSizes[i].height = (int) flow.rows/pow(2,i+2);
        blockSizes[i].width = (int) flow.cols/pow(2,i+2);
    }

    // 1-迭代
    int i = 0;
    double min_error = DBL_MAX;//最小误差度
    double merror;//每个点平均误差
    cout<<endl<<"It's starting RANSAC iterative computing...";
    while(i<iterTimes && min_error>epsilon){
        // 2-从所有点中有分布的抽取若干个点,计算FOE
        Point2f curr_foe  = getDenseFOE(flow,blockSizes[i]);
        // 再随机抽取M个点,计算FOE
        Size val_block(blockSizes[i].height*0.8,blockSizes[i].width*0.8);
        Point val_foe = getDenseFOE(flow,val_block);

        // 计算平均每个点的误差,如果当前误差比先前误差更小,则认为本次估计更可靠
        int M = flow.rows*flow.cols/(val_block.height*val_block.width);
        // 误差采用两次估计的FOE的欧式距离在每个点上的均值
        double error = (val_foe.y-curr_foe.y)*(val_foe.y-curr_foe.y)+(val_foe.x-curr_foe.x)*(val_foe.x-curr_foe.x);

        // 第一次则初始化,否则进行更新
        if(i<1){
            min_error = error;
            merror = error/M;
            foe = curr_foe;
        }else if(error<merror*M){
            // 当前均点误差小于先前误差,则更新总内点集合
            min_error = error;
            merror = error/M;
            foe = curr_foe;
        }

        i++;
    }

    cout<<"\niterTime:"<<i<<endl;
    cout<<"epsilon:"<<min_error<<endl;
//    cout<<"FOE is:"<<foe.x<<","<<foe.y<<endl;
    cout<<"RANSAC computing done!"<<endl;
    return foe;
}

/** 计算两个向量的夹角,
 * @tparam T 点的类型
 * @param pt1 点1代表向量1
 * @param pt2 点2代表向量2
 * @param method:0为角度,1为弧度
 * @return 点1向量旋转到点2向量的角度,不考虑顺逆方向的夹角最小值;0~pi之间
 */

template <typename T>
double_t getAngleDiff(T pt1,T pt2,int method =0){
    double_t theta1 = atan2(pt1.y,pt1.x);//返回(-M_PI,M_PI)之间的反正切弧度值
    double_t theta2 = atan2(pt2.y,pt2.x);
    double_t result = abs(theta2-theta1)>M_PI?2*M_PI-abs(theta2-theta1):abs(theta2-theta1);
    if(method==1){
        return result;
    }
    return result*180/M_PI;
}

template <typename T>
double_t getAngleDiff2(T pt1,T pt2){
    double_t mag1 =sqrt(pt1.x*pt1.x+pt1.y*pt1.y);
    double_t mag2 =sqrt(pt2.x*pt2.x+pt2.y*pt2.y);
    double_t dotmul = (pt1.x*pt2.x+pt1.y*pt2.y)/(mag1*mag2);

    return acos(dotmul)*180/M_PI;
}


/// 1-5 RANSAC 最小二乘
Point2f getDenseRansacFOE(Mat flow,float &loss_final,int iterTimes=100,double epsilon=10.0){
    assert(flow.channels()==2);
    // 0-网格均匀稀疏化,得到pts和flows
    ValidFlowPts initFlowPts = getMBlockPts(flow,4);
//    cout<<"init_size:"<<initFlowPts.pts.size()<<endl;

    // 1-稀疏光流RANSAC估计方法
    PtsFlowFoe  res = getSparseRansacFOE1(initFlowPts.pts,initFlowPts.flows,loss_final,iterTimes,epsilon);
//    cout<<"Foe:"<<res.foe<<endl;
    return res.foe;
}

PtsFlowFoe getDenseRansacFOE( ValidFlowPts initFlowPts,int iterTimes=200,double epsilon=10.0){
    assert(initFlowPts.pts.size()>2);
//    cout<<"init_size:"<<initFlowPts.pts.size()<<endl;

    // 1-稀疏光流RANSAC估计方法
    float loss_final;
    PtsFlowFoe  res = getSparseRansacFOE1(initFlowPts.pts,initFlowPts.flows,loss_final,iterTimes,epsilon);
    return res;
}

Point2f getDenseRansacFOE2(Mat flow,int iterTimes=5,double epsilon=10.0){
    assert(flow.channels()==2);
    Point2f foe;

    // 0-生成分布的区块大小,大块逐渐到小块,最大为图像的1/32
    iterTimes = iterTimes>5 ? 5:iterTimes;
    vector<Size> blockSizes(iterTimes,Size(0,0));
    for(int i=0;i<iterTimes;i++){
        blockSizes[i].height = (int) flow.rows/pow(2,i+2);
        blockSizes[i].width = (int) flow.cols/pow(2,i+2);
    }

    // 1-迭代
    int i = 0;
    double min_error = DBL_MAX;//最小误差度
    double merror;//每个点平均误差
    cout<<endl<<"It's starting RANSAC iterative computing...";
    while(i<iterTimes && min_error>epsilon){
        // 2-从所有点中有分布的抽取若干个点,计算FOE
        Point2f curr_foe  = getDenseFOE(flow,blockSizes[i]);
        vector<Point2f> inLiers;

        // 再随机抽取M个点,计算FOE
        Size val_block(blockSizes[i].height*0.8,blockSizes[i].width*0.8);
        int M = flow.rows*flow.cols/(val_block.height*val_block.width);
        Point val_foe = getDenseFOE(flow,val_block);

        int conseousNum = 0;
        for (size_t y = 0; y < flow.rows; y += val_block.height) {
            for (size_t x = 0; x < flow.cols; x += val_block.width) {
                Point2f val_pt_flow = flow.at<Point2f>(y, x);
                // 计算该点与curr_foe连线向量 与 其光流向量的角度差
                Point2f pt2foe = Point2f(x,y)-curr_foe;
                float angleErr = getAngleDiff2(val_pt_flow,pt2foe);
                if(angleErr < epsilon){ //如果角度差在5度之间,则认为是内点
                    inLiers.push_back(val_pt_flow);
                    conseousNum++;
                }
            }
        }

//        if(conseousNum>=0.75*M){
//            return getDenseFoeFromPts(inLiers,flow);; // 如果验证集合中75%的点符合要求,则认为该模型可靠,返回当前内点集合的最小二乘解
//        }else if(conseousNum>=0.25*M){
//            // 如果验证集合中不到25%的点符合要求,则认为该模型不可靠
//        }

        // 计算平均每个点的误差,如果当前误差比先前误差更小,则认为本次估计更可靠
        // 误差采用两次估计的FOE的欧式距离在每个点上的均值
        double error = (val_foe.y-curr_foe.y)*(val_foe.y-curr_foe.y)+(val_foe.x-curr_foe.x)*(val_foe.x-curr_foe.x);

        // 第一次则初始化,否则进行更新
        if(i<1){
            min_error = error;
            merror = error/M;
            foe = curr_foe;
        }
        else if(error<merror*M){
            // 当前均点误差小于先前误差,则更新总内点集合
            min_error = error;
            merror = error/M;
            foe = curr_foe;
        }

        i++;
    }

    cout<<"epsilon:"<<min_error<<endl;
//    cout<<"FOE is:"<<foe.x<<","<<foe.y<<endl;
    cout<<"RANSAC computing done!"<<endl;
    return foe;
}


/** 2- 稀疏光流计算FOE **/
/// 2-1 获取稀疏光流中的有效向量
void getValidFlowPts(vector<Point2f> prevPts,vector<Point2f> currPts,
                     vector<uchar> status,vector<Point2f> &flows,vector<Point2f> &validPts)
{
    // 获取有效的光流向量
    for (int i = 0; i < status.size(); ++i) {
        if(status[i]==1){
            Point2f flow;
            flow.x=currPts[i].x-prevPts[i].x;
            flow.y=currPts[i].y-prevPts[i].y;
            flows.push_back(flow);
            validPts.push_back(prevPts[i]);
        }
    }
}

/// 2-2 有效光流点集最小二乘Ls计算FOE
Point2f calLsSparseFOE(vector<Point2f> flows,vector<Point2f> Pts){
    //计算 FOE
    Eigen::MatrixXf A(flows.size(),2);
    Eigen::VectorXf b(flows.size());

    for (int i = 0; i < flows.size(); ++i) {
        Point2f fxy = flows[i];
        A(i,0) = fxy.y;
        A(i,1) = -fxy.x;
        b(i) = fxy.y*Pts[i].x-fxy.x*Pts[i].y;
    }

    Eigen::MatrixXf temp=A.transpose()*A;
    Eigen::MatrixXf foe = temp.inverse()*A.transpose()*b;
//    cout<<"one_time:"<<foe(0,0)<<foe(1,0)<<endl;
    return Point2f(foe(0,0),foe(1,0));
}

Point2f calLsSparseFOE(vector<Point2f> flows,vector<Point2f> Pts,double &loss){
    //计算 FOE
    Eigen::MatrixXf A(flows.size(),2);
    Eigen::VectorXf b(flows.size());

    for (int i = 0; i < flows.size(); ++i) {
        Point2f fxy = flows[i];
        A(i,0) = fxy.y;
        A(i,1) = -fxy.x;
        b(i) = fxy.y*Pts[i].x-fxy.x*Pts[i].y;
    }

    Eigen::MatrixXf temp=A.transpose()*A;
    Eigen::MatrixXf foe = temp.inverse()*A.transpose()*b;
    Point2f result = Point2f(foe(0,0),foe(1,0));
    // 计算最小二乘角度总误差
    loss = 0.0;
    for(int i=0;i<flows.size();i++){
        loss += getAngleDiff2(flows[i],Pts[i]-result);
    }
    return result;
}

/// 2-3 根据稀疏光流计算FOE
Point2f getSparseFOE(vector<Point2f> prevPts,vector<Point2f> currPts,vector<uchar> status){
    vector<Point2f> flows;
    vector<Point2f> Pts;
    // 获取有效的光流点
    getValidFlowPts(prevPts,currPts,status,flows,Pts);
//    cout<<"valid_flow_pts:"<<flows.size()<<endl;

    double loss;
    Point2f foe = calLsSparseFOE(flows,Pts,loss);
//    cout<<"solo_loss:"<<loss/flows.size()<<endl;
    return foe;
}

/// 随机取下标数组
unordered_set<int> getRandomIndex(int maxNum,int num){
    unordered_set<int> result;

    while(result.size()<num){
        int ind = rand()/maxNum;
        if( result.find(ind)==result.end() ){
            result.insert(ind);
        }
    }
    return result;
}

/// 随机取子集
ValidFlowPts getRandomSubPts(vector<Point2f> flows,vector<Point2f> pts,int maxNum,int num){
    ValidFlowPts res;
    res.flows.clear();
    res.pts.clear();
    res.index.clear();

    while(res.index.size()<num){
        int ind = rand()%maxNum;
        if( res.index.find(ind)==res.index.end() ){
            res.index.insert(ind);
            res.flows.push_back(flows[ind]);
            res.pts.push_back(pts[ind]);
        }
    }
    return res;
}

/// 2-4 根据稀疏光流使用RANSAC来计算FOE

PtsFlowFoe getSparseRansacFOE(vector<Point2f> prevPts,vector<Point2f> currPts,
                              vector<uchar> status,int iterTimes=200,double epislon=10.0){
    PtsFlowFoe res;
    vector<Point2f> Pts;
    vector<Point2f> flows;
    getValidFlowPts(prevPts,currPts,status,flows,Pts);//获取有效光流向量
    cout<<"valid_flow_pts:"<<flows.size()<<endl;

    // 如果有效的光流点个数小于N个,则直接计算FOE
    if(flows.size()<100){
        res.pts = Pts;
        res.flows = flows;
        res.foe = calLsSparseFOE(res.flows,res.pts);
        return res;
    }

    // 否则采用RANSAC
    double accuracy = 0.0;
    double loss = DBL_MAX; //记录历史最低loss
    Point2f foe(0,0); // 最优的FOE
    int batch_size = flows.size()*0.30; //每次随机取的批的个数
    ValidFlowPts inlierFlowPts; //最终的有效内点,光流及其标号

    while(accuracy<0.80 && iterTimes>0)
    {
        // 从所有点中随机取55%的点计算最小二乘,得FOE;
        inlierFlowPts = getRandomSubPts(flows,Pts,flows.size(),batch_size);
        Point2f curr_foe = calLsSparseFOE(inlierFlowPts.flows,inlierFlowPts.pts);
        res.diff.clear();

        double curr_loss = 0;
        // 对剩余进行校验,以epislon为阈值,统计符合该模型的个数
        for(int i=0;i<flows.size();i++){
            if(inlierFlowPts.index.find(i)==inlierFlowPts.index.end() ){
                //获取该点光流向量vec1和该点指向foe点的向量vec2
                Point2f vec1 = flows[i];
                Point2f vec2 = Pts[i]-curr_foe;
                double diff = getAngleDiff2(vec1,vec2);
                //如果两个向量角度之差在阈值内,则视为内点,构造一致集合
                if( diff<epislon ){
                    inlierFlowPts.index.insert(i);
                    inlierFlowPts.pts.push_back(Pts[i]);
                    inlierFlowPts.flows.push_back(flows[i]);
                    res.diff.push_back(diff);
                }
                curr_loss += diff; //内点集合的总误差
            }else{
//                Point2f vec1 = flows[i];
//                Point2f vec2 = Pts[i]-curr_foe;
//                double diff = getAngleDiff2(vec1,vec2);
                res.diff.push_back(-1.0); //内点则视为为0
            }
        }
        // 更新模型估计,并计算模型准确率
        curr_foe = calLsSparseFOE(inlierFlowPts.flows,inlierFlowPts.pts);

        /// 新增:更新后再次对所有内点进行校验,剔除epsilon之外的点
        for(int i=0;i<inlierFlowPts.pts.size();++i){
            double diff = getAngleDiff2(inlierFlowPts.pts[i]-curr_foe,inlierFlowPts.flows[i]);
            if(diff>=epislon){
                inlierFlowPts.pts.erase(inlierFlowPts.pts.begin()+i);
                inlierFlowPts.flows.erase(inlierFlowPts.flows.begin()+i);
                res.diff.erase(res.diff.begin()+i);
                --i;
            }
        }

        // 如果验证集合中有足够多85%的点符合该模型,则认为该模型优秀,返回当前内点集合的最小二乘估计;
        if( double(inlierFlowPts.pts.size()) >=0.80*Pts.size() ){
            res.flows = inlierFlowPts.flows;
            res.pts = inlierFlowPts.pts;
            res.foe = curr_foe;
            return res;
        }

        // 如果本次模型优于之前模型,则更新最优解
        if( inlierFlowPts.pts.size() > accuracy*flows.size() ){
            accuracy = double(inlierFlowPts.pts.size()) /flows.size();
//            cout<<"accuracy_update:"<<accuracy<<endl;
            res.clear();
            res.flows = inlierFlowPts.flows;
            res.pts = inlierFlowPts.pts;
            res.foe = curr_foe;
        }

        // 如果本次估计的均点误差小于上一次,则更新最优解
        if( curr_loss< flows.size()*loss){
            loss = curr_loss/flows.size();
//            cout<<"curr_loss:"<<loss<<endl;
            res.clear();
            res.flows = inlierFlowPts.flows;
            res.pts = inlierFlowPts.pts;
            res.foe = curr_foe;
        }

        iterTimes--;
    }

    cout<<"accuracy_final:"<<accuracy<<endl;
    cout<<"loss_final:"<<loss<<endl;
    cout<<"finalValidPts:"<<res.pts.size()<<endl;
    cout<<"iterTime:"<<iterTimes<<endl;
    return res;
}

PtsFlowFoe getSparseRansacFOE1(vector<Point2f> Pts,vector<Point2f> flows,float &loss_final,int iterTimes,double epislon){
    PtsFlowFoe res;

    // 如果有效的光流点个数小于N个,则直接计算FOE
    if(flows.size()<100){
        res.pts = Pts;
        res.flows = flows;
        res.foe = calLsSparseFOE(res.flows,res.pts);
        return res;
    }

    // 否则采用RANSAC
    double accuracy = 0.0;
    double loss = DBL_MAX; //记录历史最低loss
    Point2f foe(0,0); // 最优的FOE
    int batch_size = flows.size()*0.30; //每次随机取的批的个数
    ValidFlowPts inlierFlowPts; //最终的有效内点,光流及其标号

    while(accuracy<0.80 && iterTimes>0)
    {
        // 从所有点中随机取55%的点计算最小二乘,得FOE;
        inlierFlowPts = getRandomSubPts(flows,Pts,flows.size(),batch_size);
        Point2f curr_foe = calLsSparseFOE(inlierFlowPts.flows,inlierFlowPts.pts);
        res.diff.clear();

        double curr_loss = 0;
        // 对剩余进行校验,以epislon为阈值,统计符合该模型的个数
        for(int i=0;i<flows.size();i++){
            if(inlierFlowPts.index.find(i)==inlierFlowPts.index.end() ){
                //获取该点光流向量vec1和该点指向foe点的向量vec2
                Point2f vec1 = flows[i];
                Point2f vec2 = Pts[i]-curr_foe;
                double diff = getAngleDiff2(vec1,vec2);
                //如果两个向量角度之差在阈值内,则视为内点,构造一致集合
                if( diff<epislon ){
                    inlierFlowPts.index.insert(i);
                    inlierFlowPts.pts.push_back(Pts[i]);
                    inlierFlowPts.flows.push_back(flows[i]);
                    res.diff.push_back(diff);
                }
                curr_loss += diff; //内点集合的总误差
            }else{
//                Point2f vec1 = flows[i];
//                Point2f vec2 = Pts[i]-curr_foe;
//                double diff = getAngleDiff2(vec1,vec2);
                res.diff.push_back(-1.0); //内点则视为为0
            }
        }
        // 更新模型估计,并计算模型准确率
        curr_foe = calLsSparseFOE(inlierFlowPts.flows,inlierFlowPts.pts);

        /// 新增:更新后再次对所有内点进行校验,剔除epsilon之外的点
        for(int i=0;i<inlierFlowPts.pts.size();++i){
            double diff = getAngleDiff2(inlierFlowPts.pts[i]-curr_foe,inlierFlowPts.flows[i]);
            if(diff>=epislon){
                inlierFlowPts.pts.erase(inlierFlowPts.pts.begin()+i);
                inlierFlowPts.flows.erase(inlierFlowPts.flows.begin()+i);
                res.diff.erase(res.diff.begin()+i);
                --i;
            }
        }

        // 如果验证集合中有足够多85%的点符合该模型,则认为该模型优秀,返回当前内点集合的最小二乘估计;
        if( double(inlierFlowPts.pts.size()) >=0.80*Pts.size() ){
            res.flows = inlierFlowPts.flows;
            res.pts = inlierFlowPts.pts;
            res.foe = curr_foe;
            return res;
        }

        // 如果本次模型优于之前模型,则更新最优解
        if( inlierFlowPts.pts.size() > accuracy*flows.size() ){
            accuracy = double(inlierFlowPts.pts.size()) /flows.size();
//            cout<<"accuracy_update:"<<accuracy<<endl;
            res.clear();
            res.flows = inlierFlowPts.flows;
            res.pts = inlierFlowPts.pts;
            res.foe = curr_foe;
        }

        // 如果本次估计的均点误差小于上一次,则更新最优解
        if( curr_loss< flows.size()*loss){
            loss = curr_loss/flows.size();
//            cout<<"curr_loss:"<<loss<<endl;
            res.clear();
            res.flows = inlierFlowPts.flows;
            res.pts = inlierFlowPts.pts;
            res.foe = curr_foe;
        }

        iterTimes--;
    }

    cout<<"accuracy_final:"<<accuracy<<endl;
    cout<<"loss_final:"<<loss<<endl;
    cout<<"finalValidPts:"<<res.pts.size()<<endl;
    cout<<"iterTime:"<<iterTimes<<endl;
//    cout<<"FOE:"<<res.foe<<endl;

    return res;
}

/// 采用内点个数和误差损失正则化作为最终的损失
PtsFlowFoe getSparseRansacFOE2(vector<Point2f> prevPts,vector<Point2f> currPts,
                               vector<uchar> status,int iterTimes=200,double epislon=5.0){
    PtsFlowFoe res;
    vector<Point2f> Pts;
    vector<Point2f> flows;
    getValidFlowPts(prevPts,currPts,status,flows,Pts);//获取有效光流向量
    cout<<"valid_flow_pts:"<<flows.size()<<endl;

    // 如果有效的光流点个数小于N个,则直接计算FOE
    if(flows.size()<100){
        res.pts = Pts;
        res.flows = flows;
        res.foe = calLsSparseFOE(res.flows,res.pts);
        return res;
    }

    // 否则采用RANSAC
    double accuracy = 0.0;
    double loss = DBL_MAX; //记录历史最低loss
    Point2f foe(0,0); // 最优的FOE
    int batch_size = flows.size()*0.30; //每次随机取的批的个数

    while(accuracy<0.80 && iterTimes>0)
    {
        // 从所有点中随机取30%的点计算最小二乘,得FOE;
        ValidFlowPts inlierFlowPts = getRandomSubPts(flows,Pts,flows.size(),batch_size);
        Point2f curr_foe = calLsSparseFOE(inlierFlowPts.flows,inlierFlowPts.pts);
        res.diff.clear();

        double curr_loss = 0;
        // 对所有点进行校验,以epislon为阈值,统计符合该模型的个数
        ValidFlowPts validInliers;
        validInliers.clear();
        vector<double> diffVec;
        for(int i=0;i<flows.size();i++){
            //获取该点光流向量vec1和该点指向foe点的向量vec2
            Point2f vec1 = flows[i];
            Point2f vec2 = Pts[i]-curr_foe;
            double diff = getAngleDiff2(vec1,vec2);
            //如果两个向量角度之差在阈值内,则视为内点,构造一致集合
            if( diff<epislon ){
                validInliers.index.insert(i);
                validInliers.pts.push_back(Pts[i]);
                validInliers.flows.push_back(flows[i]);
            }
            diffVec.push_back(diff);
//                curr_loss += diff; //总角度误差
        }
        // 更新模型估计,并计算模型准确率
        curr_foe = calLsSparseFOE(validInliers.flows,validInliers.pts,curr_loss);

        /// 新增:更新后再次对所有内点进行校验,剔除epsilon之外的点
        for(int i=0;i<validInliers.pts.size();++i){
            double diff = getAngleDiff2(validInliers.pts[i]-curr_foe,validInliers.flows[i]);
            if(diff>=epislon){
                validInliers.pts.erase(validInliers.pts.begin()+i);
                validInliers.flows.erase(validInliers.flows.begin()+i);
                diffVec.erase(diffVec.begin()+i);
                --i;
            }
        }

        // 如果验证集合中有足够多80%的点符合该模型,则认为该模型优秀,返回当前内点集合的最小二乘估计;
        if( double(validInliers.pts.size()) >=0.80*Pts.size() ){
            res.flows = validInliers.flows;
            res.pts = validInliers.pts;
            res.foe = curr_foe;
            res.diff = diffVec;
            return res;
        }

        // 如果本次模型优于之前模型,则更新最优解
        if( validInliers.pts.size() > accuracy*flows.size() ){
            accuracy = double(validInliers.pts.size()) /flows.size();
//            cout<<"accuracy_update:"<<accuracy<<endl;
            res.clear();
            res.flows = validInliers.flows;
            res.pts = validInliers.pts;
            res.foe = curr_foe;
            res.diff = diffVec;
        }

        // 如果本次估计的均点误差小于上一次,则更新最优解
        if( curr_loss< validInliers.pts.size()*loss){
            loss = curr_loss/validInliers.pts.size();
//            cout<<"curr_loss:"<<loss<<endl;
            res.clear();
            res.flows = validInliers.flows;
            res.pts = validInliers.pts;
            res.foe = curr_foe;
            res.diff = diffVec;
        }

        iterTimes--;
    }

    cout<<"accuracy_final:"<<accuracy<<endl;
    cout<<"loss_final:"<<loss<<endl;
    cout<<"finalValidPts:"<<res.pts.size()<<endl;
    cout<<"iterTime:"<<iterTimes<<endl;
    return res;
}

//Point2f getSparseRansacFOE(vector<Point2f> prevPts,vector<Point2f> currPts,
//        vector<uchar> status,int iterTimes=500,double epislon=5.0){
//    vector<Point2f> flows;
//    vector<Point2f> Pts;
//    getValidFlowPts(prevPts,currPts,status,flows,Pts);//获取有效光流向量
//    cout<<"valid_flow_pts:"<<flows.size()<<endl;
//
//    // 如果有效的光流点个数小于N个,则直接计算FOE
//    if(flows.size()<100){
//        return calLsSparseFOE(flows,Pts);
//    }
//
//    // 否则采用RANSAC
//    double accuracy = 0.0;
//    int batch_size = flows.size()*0.50; //每次随机取的批的个数
////    unordered_set<int> inliers_index; // 使用set标记以上内点的下标
////    vector<Point2f> inliers; // 内点集合
//    ValidFlowPts inlierFlowPts;
//    Point2f foe;
//    double loss = DBL_MAX;
//
//    while(accuracy<0.7 && iterTimes>0)
//    {
//        // 从所有点中随机取75%的点计算最小二乘,得FOE;
//        inlierFlowPts = getRandomSubPts(flows,Pts,flows.size(),batch_size);
//        Point2f curr_foe = calLsSparseFOE(inlierFlowPts.flows,inlierFlowPts.pts);
//
//        double curr_loss = 0;
//        // 对剩余进行校验,以epislon为阈值,统计符合该模型的个数
//        for(int i=0;i<flows.size();i++){
//            if(inlierFlowPts.index.find(i)==inlierFlowPts.index.end() ){
//                //获取该点光流向量vec1和该点指向foe点的向量vec2
//                Point2f vec1 = flows[i];
//                Point2f vec2 = Pts[i]-curr_foe;
//                double diff = getAngleDiff(vec1,vec2);
//                diff= diff>90?180-diff:diff;
////                cout<<"diff:"<<diff<<endl;
//
//                //如果两个向量角度之差在阈值内,则视为内点
//                if( diff<epislon ){
//                    inlierFlowPts.index.insert(i);
//                    inlierFlowPts.pts.push_back(Pts[i]);
//                    inlierFlowPts.flows.push_back(flows[i]);
//                }
//                curr_loss += diff; //内点集合的总误差
//
//            }
//        }
//
////        cout<<"inliers_size:"<<inliers.size()<<endl;
//        // 更新模型估计,并计算模型准确率
//        curr_foe = calLsSparseFOE(inlierFlowPts.flows,inlierFlowPts.pts);
//
//        // 如果验证集合中有足够多85%的点符合该模型,则认为该模型优秀,返回当前内点集合的最小二乘估计;
//        cout<<inlierFlowPts.pts.size()<<endl;
//        if(double(inlierFlowPts.pts.size())/Pts.size() >=0.7 ){
//            return curr_foe;
//        }
//
//        // 如果本次模型由于之前模型,则更新
//        if( inlierFlowPts.pts.size() > accuracy*flows.size() ){
//            accuracy = double(inlierFlowPts.pts.size()) /flows.size();
//            foe = curr_foe;
//            cout<<"accuracy_update:"<<accuracy<<endl;
//        }
//
//        // 如果本次估计的均点误差小于上一次,则更新
//        if(curr_loss<flows.size()*loss){
//            loss = curr_loss/flows.size();
//            foe = curr_foe;
//            cout<<"curr_loss:"<<loss<<endl;
//        }
//
//        iterTimes--;
//    }
//
//    cout<<"accuracy_final:"<<accuracy<<endl;
//    cout<<"loss_final:"<<loss<<endl;
//    cout<<"iterTime:"<<iterTimes<<endl;
//    return foe;
//}

/// 2-5 先计算单次最小二乘估计,然后判断各个点与FOE的角度差,更新内点
// 直到所有点,均为内点
//Point2f getSparseRazorFOE(vector<Point2f> prevPts,vector<Point2f> currPts, vector<uchar> status,double epislon=5){
//    vector<Point2f> flows;
//    vector<Point2f> Pts;
//    getValidFlowPts(prevPts,currPts,status,flows,Pts);//获取有效光流向量
//    cout<<"valid_flow_pts:"<<flows.size()<<endl;
//
//    Point2f curr_foe = calLsSparseFOE(flows,Pts);//初次计算FOE
//    // 如果有效的光流点个数小于N个,则直接计算FOE
//    if(flows.size()<50){
//        return curr_foe;
//    }
//
//    bool flag = true;
//    vector<Point2f> inlierFlows(flows);//光流内点集合
//    vector<Point2f> inlierPts(Pts);//内点集合
//
//    while(flag){
//        // 剔除误差限之外的点
//        int err_count = 0;
//        for(int i=0;i<inlierPts.size();i++){
//            if(getAngleDiff(inlierFlows[i],inlierPts[i]-curr_foe)>epislon){
//                inlierFlows.erase(inlierFlows.begin()+i);
//                inlierPts.erase(inlierPts.begin()+i);
//                err_count++;//误差限以外的点的个数
//            }
//        }
//        // 更新最小二乘估计
//        curr_foe = calLsSparseFOE(inlierFlows,inlierPts);
//
//        if(err_count==0){
//            flag =false;
//        }
//    }
//
//    cout<<"inliers_num:"<<inlierPts.size()<<endl;
//    return curr_foe;
//}

Point2f getSparseRazorFOE(vector<Point2f> prevPts,vector<Point2f> currPts, vector<uchar> status,
                          vector<Point2f> &inlierPts,vector<Point2f> &inlierFlows,double epislon=6.0){
    vector<Point2f> flows;
    vector<Point2f> Pts;
    getValidFlowPts(prevPts,currPts,status,flows,Pts);//获取有效光流向量
    cout<<"valid_flow_pts:"<<flows.size()<<endl;

    Point2f curr_foe = calLsSparseFOE(flows,Pts);//初次计算FOE
    // 如果有效的光流点个数小于N个,则直接计算FOE
    if(flows.size()<100){
        return curr_foe;
    }

    bool flag = true;
    inlierFlows = flows;//光流内点集合
    inlierPts = Pts;

    while(flag){
        // 剔除误差限之外的点
        int err_count = 0;
        for(int i=0;i<inlierPts.size();i++){
            if(getAngleDiff(inlierFlows[i],inlierPts[i]-curr_foe)>epislon){
                inlierFlows.erase(inlierFlows.begin()+i);
                inlierPts.erase(inlierPts.begin()+i);
                err_count++;//误差限以外的点的个数
            }
        }
        // 更新最小二乘估计
        curr_foe = calLsSparseFOE(inlierFlows,inlierPts);

        if(err_count==0){
            flag =false;
        }

        if(inlierPts.size()<100){
            return curr_foe;
        }
    }

    cout<<"RazorPts done..."<<endl;
    cout<<"inliers_num:"<<inlierPts.size()<<endl;
    return curr_foe;
}

PtsFlowFoe getSparseRazorFOE(vector<Point2f> prevPts,vector<Point2f> currPts, vector<uchar> status,double epislon=5.0){
    PtsFlowFoe res;
    getValidFlowPts(prevPts,currPts,status,res.flows,res.pts);//获取有效光流向量
    cout<<"valid_flow_pts:"<<res.flows.size()<<endl;

    Point2f curr_foe = calLsSparseFOE(res.flows,res.pts);//初次计算FOE
    // 如果有效的光流点个数小于N个,则直接计算FOE
    if(res.flows.size()<100){
        res.foe = curr_foe;
        return res;
    }

    bool flag = true;
    while(flag){
        // 剔除误差限之外的点
        int err_count = 0;
        for(int i=0;i<res.pts.size();i++){
            if(getAngleDiff2(res.flows[i],res.pts[i]-curr_foe)>epislon){
                res.flows.erase(res.flows.begin()+i);
                res.pts.erase(res.pts.begin()+i);
                err_count++;//误差限以外的点的个数
            }
        }
        // 更新最小二乘估计
        curr_foe = calLsSparseFOE(res.flows,res.pts);

        if(err_count==0){
            res.foe = curr_foe;
            flag =false;
        }

        if(res.pts.size()<100){
            res.foe = curr_foe;
            return res;
        }
    }

    cout<<"RazorPts done..."<<endl;
    cout<<"inliers_num:"<<res.pts.size()<<endl;
    return res;
}


///****************** 3- 计算到达时间 *********************///
Mat timeToContact0(Mat flow,Point2f foe)
{
    assert(flow.channels()==2);
    Mat ttcMap = Mat(flow.size(),CV_32FC1);

    float cx = foe.x;//水平方向
    float cy = foe.y;//垂直方向

    for (int y = 0; y < flow.rows; ++y) {
        for (int x = 0; x < flow.cols; ++x) {
            Point2f fxy = flow.at<Point2f>(y,x);
            float flowMag = sqrt( fxy.x*fxy.x+ fxy.y*fxy.y);
            float distance = sqrt( (x-cx)*(x-cx)+(y-cy)*(y-cy));;
            float ttc = distance/flowMag;
            ttcMap.at<float>(y,x) = ttc;
        }
    }
    return ttcMap;
}

Mat timeToContact(Mat flow,Point2f foe,int method=1,int maxTTC=100)
{
    assert(flow.channels()==2);
    Mat ttcMap = Mat(flow.size(),CV_32FC1);

    float cx = foe.x;//水平方向
    float cy = foe.y;//垂直方向

    for (int y = 0; y < flow.rows; ++y) {
        for (int x = 0; x < flow.cols; ++x) {
            Point2f fxy = flow.at<Point2f>(y,x);
            float flowMag = sqrt( fxy.x*fxy.x+ fxy.y*fxy.y);
            float distance = sqrt( (x-cx)*(x-cx)+(y-cy)*(y-cy));;
            float ttc = distance/flowMag;
//            float ttc = sqrt( ( float( (x-cx)*(x-cx)+(y-cy)*(y-cy) )/( fxy.x*fxy.x+ fxy.y*fxy.y)) );
            if(method==1){
                ttc = ttc>maxTTC?maxTTC:ttc;
            }
            ttcMap.at<float>(y,x) = ttc;
        }
    }
//    printMatToTxt(ttcMap,"./Results/ttc_trunct120.txt");
    if(method==2){
        // 舍弃后30%的值
        Mat meanMat,stdDev;
        meanStdDev(ttcMap,meanMat,stdDev);
//        cv::Scalar meanV = mean(ttcMap);
//        cout<<"meanV:"<<meanV<<endl;
        cout<<"meanMat:"<<meanMat.at<double>(0,0)<<endl;
        cout<<"stdDev:"<<stdDev.at<double>(0,0)<<endl;
        for(int j=0;j< ttcMap.rows; ++j){
            for (int i = 0; i < ttcMap.cols; ++i) {
                if(ttcMap.at<float>(j,i)>0.05*meanMat.at<float>(0,0)){
                    ttcMap.at<float>(j,i) = 0.05*meanMat.at<float>(0,0);
                }
            }
        }
    }

    return ttcMap;
}

///****************** 4- 避障决策与控制 *********************///
/// 4-0 通用
// 航向角
const float nav_yaws[] = {-35,-25,-15,-5,5,15,25,35};
const vector<float> nav_sgn = {-1,-1,-1,-1,1,1,1,1};

// 划分竖直条纹
vector<int > linspaceMat(int width,int stroop){
    vector<int> res(stroop+1);
    res[0] = 0;
    int stride = width/stroop;
    for(int i=1;i<stroop+1;++i){
        res[i] = stride+res[i-1];
    }

    return res;
}

// 绘制竖条
void plotVbar(Mat &img,int amuzi,int stroop=8){
    int width = img.cols;
    int height = img.rows;
    vector<int> stroops = linspaceMat(width,stroop);
    cout<<"bars_size:"<<stroops.size()<<endl;
    int j=0;
    for(auto ele:stroops){
        if(j==amuzi || j==amuzi+1){
            cv::line(img,Point2d(ele,0),Point2d(ele,height),Scalar(0,255,0),4,LINE_8);
        }else{
            cv::line(img,Point2d(ele,0),Point2d(ele,height),Scalar(255,0,0),2,LINE_8);
        }
        j++;
    }
    putText(img,"ENTRY",Point(cvRound((2*amuzi+1)*width/(2*stroop)), cvRound(height/2)),
            FONT_HERSHEY_PLAIN,1.0,Scalar(255,255,0),2,LINE_AA,false);
}

// 简易寻找地平线
int findHorizontalLine(Mat ttc){
    int res=0;
    for(int i=0;i<ttc.rows;++i){
        bool flag = true;
        for(int j=0;j<ttc.cols;++j){
            if(ttc.at<uchar >(i,j)>100){
                flag=false;
                break;
            }
        }
        if(flag){
            res=i;
            break;
        }
    }
    return res;
}

// 求矢量的平均值
float getMean(vector<float> vec){
    if(vec.size()==0) return 0.0;
    float res = 0.0;
    for(auto ele:vec){
        res += ele/vec.size();
    }
    return res;
}

// 把矢量归一化
void normVec(vector<float> &input){
    float minV = FLT_MAX;
    float maxV = -FLT_MIN;
    for(auto ele:input){
        if(ele<minV) minV = ele;
        if(ele>maxV) maxV = ele;
    }
    cout<<"minV:"<<minV<<" maxV:"<<maxV<<endl;
    sort(input.begin(),input.end());
    for(int i=0;i<input.size();++i){
        input[i] = (input[i]-minV)/(maxV-minV);
    }
}

void normVec(vector<float> &input,vector<int> &index){
    float minV = FLT_MAX;
    float maxV = -FLT_MIN;
    for(auto ele:input){
        if(ele<minV) minV = ele;
        if(ele>maxV) maxV = ele;
    }
    cout<<"minV:"<<minV<<" maxV:"<<maxV<<endl;
    // 对ttc排序
    vector<float> inputBak(input); //备份
    sort(input.begin(),input.end());
    index.clear();
    for(auto iter=input.begin();iter!=input.end();++iter){
        int i=0;
        while(*iter!=inputBak[i]){
            i++;
        }
        index.push_back(i);
    }

    for(int i=0;i<input.size();++i){
        input[i] = (input[i]-minV)/(maxV-minV);
    }
}

/// 4-1 平衡法
/// 计算每个竖条的平均光流幅值
vector<float> getVbarMag(Mat mag,int stroop=8){
    vector<float> res(stroop,0);

    int height = mag.rows;
    int width = mag.cols;
    vector<int> lin = linspaceMat(width,stroop);
    float stroop_area = width*height/stroop;
    for(int i=0;i<stroop;++i){
        int low = lin[i];
        int high = lin[i+1];
        for(int col =low; col<high; ++col){
            for(int row=0; row<height; ++row){
                res[i] += mag.at<float >(row,col)/stroop_area;
            }
        }
    }
    return res;
}

// 计算航向角
float getNavYaw(int barInd){
    barInd -=4;
    return nav_yaws[barInd]*M_PI/180;
}

// 基于光流平均幅值进行航向角决策
float getBalancedYawBaseOnMagnitude(vector<float> bar_mags,int stroops,float threshold = 1.10){
    float yaw;
    if(stroops==2){
        if(bar_mags[0]>threshold*bar_mags[1]){
            yaw = -M_PI/8; //左侧幅值大，更近；往右偏，顺时针旋转
        }else if(bar_mags[1]>threshold*bar_mags[0]){
            yaw = M_PI/8; //右侧幅值大，更近；往左偏，逆时针旋转
        }else{
            yaw = 0.0; //近似相等，不改变航向
        }
    }else{
        int maxBarMagInd = 0;
        float maxBarMag = 0.0;
        for (int j = 0; j < bar_mags.size(); ++j) {
            if(bar_mags[j]>maxBarMag){
                maxBarMag = bar_mags[j];
                maxBarMagInd = j;
            }
        }
        yaw = getNavYaw(maxBarMagInd);
    }
    return yaw;
}

// 基于光流幅值左右平衡避障，根据差值计算航向角
// method=2表示根据差值计算航向角，=1表示固定值
float getLRBalancedYawBaseOnMagnitude(vector<float> bar_mags,bool DIFF_METHOD = true,float maxYaw = 35.0,float threshold = 1.10){
    float yaw;
    if(bar_mags[0]>threshold*bar_mags[1]){
        yaw = DIFF_METHOD? -maxYaw*(bar_mags[0]-bar_mags[1])/(bar_mags[0]+bar_mags[1]): -M_PI/8; //左侧幅值大，更近；往右偏，顺时针旋转
    }else if(bar_mags[1]>threshold*bar_mags[0]){
        yaw = DIFF_METHOD? maxYaw*(bar_mags[0]-bar_mags[1])/(bar_mags[0]+bar_mags[1]): M_PI/8; //右侧幅值大，更近；往左偏，逆时针旋转
    }else{
        yaw = 0.0; //近似相等，不改变航向
    }

    return yaw;
}

// 基于幅值平衡策略获取偏航角
float getNavYawByBS(Mat mag,int stroops){
    vector<float> bar_mags = getVbarMag(mag,stroops);
    return getBalancedYawBaseOnMagnitude(bar_mags,stroops);
}


/// 4-2 多竖条航向决策
Point2f getAmuziVel(int amuzi,float mag = 2.0){
    Point2f vel;
    vel.y = mag;
    amuzi -= 1;
    vel.x = vel.y*tan(nav_yaws[amuzi]*M_PI/180);

    return vel;
}

// 计算偏航角
float getSteeringYaw(int amuzi){
    amuzi -= 4;
    return nav_yaws[amuzi]*M_PI/180;
}

/// 计算每个竖条的平均TTC
vector<float> getVbarTTC(Mat ttcResult,int stroop=8){
    vector<float> res(stroop,0);

    int height = ttcResult.rows;
    int width = ttcResult.cols;
    vector<int> lin = linspaceMat(width,stroop);
    float stroop_area = width*height/stroop;
    for(int i=0;i<stroop;++i){
        int low = lin[i];
        int high = lin[i+1];
        for(int col =low; col<high; ++col){
            for(int row=0; row<height; ++row){
                res[i] += ttcResult.at<uchar >(row,col)/stroop_area;
            }
        }
    }

    return res;
}

/*
 * 基于TTC的平衡避障
 * ttcResult 为CV_8UC1
 */
int balanceTTC(Mat ttcResult,int stroop=8)
{
    int res=-1;
    int height = ttcResult.rows;
    int width = ttcResult.cols;

    vector<int> lin = linspaceMat(width,stroop);
    vector<float> stroopTTC(stroop,0);
    float stroop_area = width*height/stroop;
    for(int i=0;i<stroop;++i){
        int low = lin[i];
        int high = lin[i+1];
        for(int col =low; col<high; ++col){
            for(int row=0; row<height; ++row){
                stroopTTC[i] += ttcResult.at<float >(row,col)/stroop_area;
            }
        }
    }

    // 退化为左右光流平衡避障
    if(stroop==2){
        cout<<"stroopTTC:"<<stroopTTC[0]<<" "<<stroopTTC[1]<<endl;
        cout<<"balance_ratio:"<<stroopTTC[1]/stroopTTC[0]<<endl;
        if(stroopTTC[0]>1.20*stroopTTC[1]){
            return 0;
        }else if(stroopTTC[1]>1.20*stroopTTC[0]){
            return 1;
        }else{
            return 2;
        }
    }

    vector<int> gapInt;
    // 从1-6这6个方向中选择可行方向
    for(auto iter=stroopTTC.begin()+1;iter<stroopTTC.end()-1;++iter){
        if( *iter>*(iter-1) && *iter>*(iter+1)){
            gapInt.push_back(iter-stroopTTC.begin());
        }
    }

    if(gapInt.size()>0){
        // 如果存在，则选择标号最大的方向
        for(auto ele:gapInt){
            res= (ele>res)?ele:res;
        }
    }else{
        float maxStroop=-1;
        for(int i=0;i<stroopTTC.size();++i){
            if(stroopTTC[i]>maxStroop){
                res = i;
                maxStroop = stroopTTC[i];
            }
        }
    }

    return res;
}



/// 4-3 APF避障控制率
/*
 * 输入: ttc分布(每个竖条的平均TTC),目标航向foe
 * 输出: 避障航向
 */


Point2f SteeringAPF(Point2f foe,Mat ttcResult){
    Point2f vel; //航向和速度
    const float k0 = 1;
    const float cpsi = 0.25; // 最大旋转角,40度
    const float cd = 0.5;

    int stroop = 8;
    // 求ttc的分布和各自的航向角度(弧度制)
    vector<float> ttc = getVbarTTC(ttcResult,stroop);
    normVec(ttc);
    cout<<"ttc:\t";
    for(auto ele:ttc){
        cout<<ele<<"\t";
    }

    int foe_ind = int(foe.x/(ttcResult.cols/stroop));
    cout<<"foe_ind:"<<foe_ind<<endl;
    float foe_ttc = getMean(ttc);
    cout<<"foe_ttc:"<<foe_ttc<<endl;

    // APF求和力
    float usc = 0.0;
    if(foe_ind>=0 && foe_ind<=stroop-1){
         usc += k0*nav_sgn[foe_ind]*exp(-cpsi*abs(nav_yaws[foe_ind])/35.0)*exp(-cd*abs(foe_ttc)); // 角度越大,产生的作用越大;越近,到达时间越小,产生的作用越大
    }
    for(int i=0;i<3;i++){
        usc -= k0*nav_sgn[i]*exp(-cpsi*abs(nav_yaws[i])/35.0)*exp(-cd*abs(ttc[i]));
    }
    usc = usc*exp(0.75)*M_PI*35/(180*4);
    cout<<"usc:"<<usc<<endl;

    // 输出最终的航向决策
    vel.x = 1.0;
//    vel.y = tan(usc);
    vel.y = usc;
    return vel;
}

/// 人工势场法2
Point2f APFDirection(double att_diff, Mat ttcResult){
    Point2f vel; //航向和速度
    const float k0 = 1;
    const float cpsi = 0.25; // 最大旋转角,40度
    const float cd = 0.5;

    int stroop = 8;
    // 求ttc的分布和各自的航向角度(弧度制)
    vector<float> ttc = getVbarTTC(ttcResult,stroop);
    normVec(ttc);
    cout<<"ttc:\t";
    for(auto ele:ttc){
        cout<<ele<<"\t";
    }

    // APF求和力
    float usc = 0.0;
    // 目标的引力
    usc += att_diff; // 角度越大,产生的作用越大;越近,到达时间越小,产生的作用越大

    // 障碍物的斥力
    for(int i=0;i<8;i++){
        usc -= k0*nav_sgn[i]*exp(-cpsi*abs(nav_yaws[i])/35.0)*exp(-cd*abs(ttc[i]));
    }
    // usc = usc*exp(0.75)*M_PI*35/(180*4);
    cout<<"usc:"<<usc<<endl;

    // 输出最终的航向决策
    vel.x = 1.0;
//    vel.y = tan(usc);
    vel.y = usc;

    return vel;
}

/// 人工势场法3
Point2f APFDirection3(double att_diff, Mat ttcResult){
    Point2f vel; //航向和速度

    int stroop = 8;
    // 求ttc的分布和各自的航向角度(弧度制)
    vector<float> ttc = getVbarTTC(ttcResult,stroop);
    vector<int> index;
    normVec(ttc,index);
    cout<<"ttc:\t";
    for(auto ele:ttc){
        cout<<ele<<"\t";
    }

    // APF求和力
    float usc = 0.0;
    // 目标的引力
    float att_sng = att_diff>0?1:-1;
    usc += att_sng*abs(att_diff); // 角度越大,产生的作用越大;越近,到达时间越小,产生的作用越大

    // 障碍物的斥力
    for(int i=0;i<3;i++){
        usc -= nav_yaws[index[i] ]/pow((1-ttc[index[i]]),2);
    }
    // usc = usc*exp(0.75)*M_PI*35/(180*4);
    cout<<"usc:"<<usc<<endl;

    // 输出最终的航向决策
    vel.x = 1.0;
    vel.y = usc;

    return vel;
}

/// 将单通道图像归一化成uint8的灰度图
Mat normToGray(Mat img)
{
    if (img.channels()!=1){
        cout<<"please input an one-channel image!"<<endl;
        return img;
    }
    Mat result = Mat(img.size(),CV_8UC1);
    normalize(img,result,0,255,NORM_MINMAX);
    result.convertTo(result,CV_8UC1,1,0);//格式转换

    return result;
}


#endif //OPTICALFLOW_DENSEFLOW_H