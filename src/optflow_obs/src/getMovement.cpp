//
// Created by wbzhang on 2020/7/9.
//
#include "../include/getMovement.h"


Movementor::Movementor(Mat prev,Mat curr):prev_frame(prev),curr_frame(curr){
    this->rows = curr_frame.rows;
    this->cols = curr_frame.cols;
    cout<<"Movementor initinalised...!"<<endl;
}

Movementor::~Movementor() {
    cout<<"Movementor done!"<<endl;
}


// 0-运行所有流程
void Movementor::run(void){
    // 1 计算ORB特征点和描述符
    cout<<"prev_size:"<<prev_frame.size()<<endl;
    cout<<"curr_size:"<<curr_frame.size()<<endl;

////    // 通过ORB算法检测两幅图像中的特征点，并计算各自的二值描述符
//    Ptr<ORB> orb = ORB::create(10000, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE, 31, 20);
//    orb->detectAndCompute(this->prev_frame, Mat(), this->keypoints1, this->descriptors1, false);
//    orb->detectAndCompute(this->curr_frame, Mat(), this->keypoints2, this->descriptors2, false);

    Ptr< FastFeatureDetector > fast = FastFeatureDetector::create(128,true,cv::FastFeatureDetector::TYPE_9_16);
    fast->detectAndCompute(this->prev_frame, Mat(), this->keypoints1, this->descriptors1, false);
    fast->detectAndCompute(this->curr_frame, Mat(), this->keypoints2, this->descriptors2, false);
    cout<<"keypoints1"<<this->keypoints1.size()<<endl;
    cout<<"keypoints2"<<this->keypoints2.size()<<endl;

    // 2- 特征匹配
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::MatcherType::BRUTEFORCE);
    //参数MatcherType：匹配器类型，这里使用MatcherType::BRUTEFORCE（暴力匹配算法）
    matcher->match(this->descriptors1, this->descriptors2, this->matches, Mat());

    cout<<"matchers:"<<this->matches.size()<<endl;
    float maxdist = 0;
    for (int i = 0; i < this->matches.size(); i++)
    {
        //寻找匹配特征点对中匹配质量最差的点对，也就是匹配距离最远的点对，获取该最大距离值
        maxdist = max(maxdist, this->matches[i].distance);
    }
    
    for (int j = 0; j < this->matches.size(); j++)
    {
        //如果匹配特征点对中，某个点对的匹配距离小于某个阈值（可以是最大距离值乘以一个小于1的系数），则可以认为是高度匹配的特征点对
        if (this->matches[j].distance < 0.20 * maxdist)
        {
            this->good_matches.push_back(this->matches[j]);
        }
    }
    
    
    //// 绘制匹配结果
//    Mat result;
//    drawMatches(this->prev_frame, this->keypoints1, this->curr_frame,this->keypoints2, this->good_matches, result, Scalar::all(-1), Scalar::all(-1));
//    //Scalar::all(-1)是选择随机颜色
//    imshow("result", result);

    /// 4 RANSAC去除误匹配并计算透视变换
    cout<<"Goor Matches is"<<good_matches.size()<<endl;
    if(good_matches.size()<5){
        cout<<"There are not enought good_match for RANSAC."<<endl;
    }else{
        for (int k = 0; k < this->good_matches.size(); ++k) {
            // 第一张图片的queryIdx
            int queryIdx = this->good_matches[k].queryIdx;
            this->ransac_pts1.push_back(this->keypoints1[queryIdx].pt);
            // 第一张图片的trainIdx
            int trainIdx = this->good_matches[k].trainIdx;
            this->ransac_pts2.push_back(this->keypoints2[trainIdx].pt);

        }

        Mat status = Mat::zeros(good_matches.size(), 1, CV_8UC1);
        this->H = findHomography(this->ransac_pts1, this->ransac_pts2, RANSAC,3,status);//计算透视变换
        // 输出单应性矩阵
        cout<<"Homography Matrix:"<<endl;
        for (int i = 0; i < H.rows; ++i) {
            for (int j = 0; j < H.cols; ++j) {
                cout<<H.at<float>(i,j)<<"\t";
            }
            cout<<endl;
        }

        uchar *status_p;
        vector<DMatch>::const_iterator it_match = good_matches.begin();
        for (int i = 0; i < good_matches.size(); i++) {
            status_p = status.ptr<uchar>(i);
            if (*status_p)
                ransac_matches.push_back(it_match[i]);
        }

        // 求基础矩阵
        cv::Mat F = findFundamentalMat(this->ransac_pts1, this->ransac_pts2, RANSAC,3);
        // 求本质矩阵
        cv::Mat E = cv::findEssentialMat(this->ransac_pts1, this->ransac_pts2, this->K,RANSAC);
        //从本质矩阵中恢复R,t
        cv::Mat R,t;
        cv::recoverPose(E, this->ransac_pts1, this->ransac_pts2, this->K, R, t);
        // 输出R,T
        cout<<"------ R Matrix ------"<<endl;
        for (int i = 0; i < R.rows; ++i) {
            for (int j = 0; j < R.cols; ++j) {
                cout<<R.at<float>(i,j)<<"\t";
            }
            cout<<endl;
        }
        cout<<"------ t Matrix ------"<<endl;
        for (int i = 0; i < t.rows; ++i) {
            for (int j = 0; j < t.cols; ++j) {
                cout<<t.at<float>(i,j)<<"\t";
            }
            cout<<endl;
        }

        /// 绘制匹配结果
        Mat img_detect = this->curr_frame.clone();
        vector<Point2f> model_corners(4);             //存储模板图像的四个角点
        vector<Point2f> scene_corners(4);             //存储模板图像在地图中的四个角点
        model_corners[0] = Point(0, 0);            //左上角为原点(0,0)；x轴指向→；y轴指向↓
        model_corners[1] = Point(this->cols, 0);
        model_corners[2] = Point(this->cols, this->rows);
        model_corners[3] = Point(0, this->rows);

        perspectiveTransform(model_corners, scene_corners, H);

        line(img_detect, scene_corners[0], scene_corners[1], Scalar(0, 255, 0), 2);   //绘制模板在场景中的范围
        line(img_detect, scene_corners[1], scene_corners[2], Scalar(0, 255, 0), 2);
        line(img_detect, scene_corners[2], scene_corners[3], Scalar(0, 255, 0), 2);
        line(img_detect, scene_corners[3], scene_corners[0], Scalar(0, 255, 0), 2);

        //绘制RANSAC筛选后的匹配结果
        Mat imgRansacMatches= Mat(rows,cols,CV_8UC3);;
        drawMatches(img_detect, this->keypoints1, this->curr_frame, this->keypoints2, ransac_matches, imgRansacMatches);   //在复制的地图图像中绘制
        namedWindow("RANSAC", 0);
//        resizeWindow("RANSAC", imgRansacMatches.cols / 2, imgRansacMatches.rows / 2);
        imshow("RANSAC", imgRansacMatches);

        // 投影
        Mat presp = Mat(2*rows,2*cols,CV_8UC3);
        warpPerspective(prev_frame, presp,H, Size(2*rows,2*cols));
        namedWindow("presp", 0);
        imshow("presp", presp);

    }

    // 5- 判断主要旋转还是平移
    
}

// 1-计算特征点
void Movementor::getKeypoints(void){
    // 1-创建OBR对象
    auto orb = ORB::create(100, 1.6, 8, 31, 0, 2, ORB::HARRIS_SCORE, 31, 20);
    // 通过ORB算法检测两幅图像中的特征点，并计算各自的二值描述符
    orb->detectAndCompute(this->prev_frame, Mat(), this->keypoints1, this->descriptors1, false);
    orb->detectAndCompute(this->curr_frame, Mat(), this->keypoints2, this->descriptors2, false);
}


// 1-2 特征描述符
void Movementor::getDescriptor(void){

    
}

// 3-特征匹配
void Movementor::getGoodMatches(){
    
}

// 4-RANSAC去除错误匹配
Mat Movementor::getRansacMatches(){
    
}

// 5-判断运动类型
bool Movementor::judgeMovement(Mat prev, Mat curr){
    
}
// 6-翘曲投影
