//
// Created by wbzhang on 2020/6/8.
//
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h> //ROS图象类型的编码函数
#include <image_transport/image_transport.h> //用来在ROS系统中的话题上发布和订阅图象消息
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/Thrust.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>

//OpenCV2标准头文件
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>     //cv_bridge中包含CvBridge库
#include <iostream> //C++标准输入输出库
#include <string>

// 光流文件
#include "../include/denseFlow.h"
#include "../include/munsell_color.h"

using namespace std;

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        //Topic you want to publish
        localvel_pub = n_.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 30);
        localpose_pub = n_.advertise<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 30);
//        pub_thrust = n_.advertise<mavros_msgs::Thrust>("mavros/setpoint_attitude/thrust", 30);
//        pub_attitude = n_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude", 30);
//        pub_vel = n_.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_attitude/cmd_vel", 30);

        //Topic you want to subscribe
        state_sub = n_.subscribe<mavros_msgs::State>("mavros/state", 10, &SubscribeAndPublish::state_cb,this);//先控制飞行模式
        local_pose_sub = n_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 5, &SubscribeAndPublish::getLocalPoseFromMsg,this);
        local_vel_sub = n_.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 5, &SubscribeAndPublish::getLocalVelFromMsg,this);
        arming_client = n_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client = n_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

        // 用于控制飞机保持OFFBOARD模式和解锁
        ros::Rate rate(20.0);
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        ros::Time last_request = ros::Time::now();
//        while(ros::ok()) {
            if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(3.0))) {
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(3.0))) {
                    if (arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
//            ros::spinOnce();
//            rate.sleep();
//        }

        sub_ = n_.subscribe("/iris_fpv_cam/usb_cam/image_raw", 1, &SubscribeAndPublish::callback, this);
//        ROS_INFO("Time is %s0.3f ",(double) (end-start)/getTickFrequency());
    }

    ///获取当前位置,并确定是否进入着陆区域
    void getLocalPoseFromMsg(const geometry_msgs::PoseStamped::ConstPtr& msg){
        current_pose=*msg;
    }

    ///获取当前速度与加速度
    void getLocalVelFromMsg(const geometry_msgs::TwistStamped::ConstPtr& msg){
        current_vel=*msg;
    }

    /// 获得当前的状态消息
    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }

    void callback(const sensor_msgs::ImageConstPtr& msg)
    {
//        mavros_msgs::Thrust cmd_thrust;
//        geometry_msgs::PoseStamped cmd_attitude;
//        geometry_msgs::TwistStamped cmd_att_vel;

        double start=(double) getTickCount();
        frameCount = 0;
        //.... do something with the input and generate the cmd_vel...
        try{
            cout << " image_L receive success " << endl;
            cv::Mat leftImg = cv_bridge::toCvShare(msg,"bgr8")->image;
            cv::cvtColor(leftImg,curr_frame,cv::COLOR_BGR2GRAY);
            cv::Mat flow,ttcResult,pesudoTTC;
            
            if(!init_frame){
                prev_frame = curr_frame;//初始化第一帧
                cv::imshow("left_optflow",leftImg);
                cout<<"****** First frame initialized! ******"<<endl;
                init_frame = true;
            }else{
//             0-计算光流
                double dis_start =  getTickCount();
                Ptr<DenseOpticalFlow> algorithm = DISOpticalFlow::create(DISOpticalFlow::PRESET_MEDIUM);
                algorithm->calc(prev_frame, curr_frame, flow);
                double dis_end = getTickCount();
                cout<<"***************************"<<endl;
                cout<<"dis_Time:"<<(double) (dis_end-dis_start)/getTickFrequency()<<endl;
                cout<<"***************************"<<endl;
                
//             1-计算FOE
                double foe_start =  getTickCount();
                float loss_final;
                Point2f rfoe = getDenseRansacFOE(flow,loss_final,20);
                cout<<"rFOE:"<<rfoe.x<<" "<<rfoe.y<<endl;
                double foe_end =  getTickCount();
                cout<<"***************************"<<endl;
                cout<<"foe_Time:"<<(double) (foe_end - foe_start)/getTickFrequency()<<endl;
                cout<<"***************************"<<endl;

                ValidFlowPts disFlow = getBlockPts(flow);
                for (int i = 0; i < disFlow.pts.size(); ++i) {
                    circle(leftImg,Point(cvRound(disFlow.pts[i].x), cvRound(disFlow.pts[i].y)),
                           3,Scalar(120,0,120),-1,LINE_AA); //Scalar(125,0,125)
                    line(leftImg, Point(cvRound(disFlow.pts[i].x), cvRound(disFlow.pts[i].y)),
                         Point(cvRound(disFlow.pts[i].x+disFlow.flows[i].x),cvRound(disFlow.pts[i].y+disFlow.flows[i].y)),
                         Scalar(0, 50, 255), 1, LINE_AA);//Scalar(0, 50, 200)
                }
                cv::circle(leftImg,rfoe,3,Scalar(0,255,255),-1,cv::LINE_AA);

                // 2-计算TTC
                Mat ttcMap = timeToContact(flow,rfoe);
                ttcResult = normToGray(ttcMap);
                // cv::applyColorMap(ttcResult,pesudoTTC,cv::COLORMAP_JET); //伪彩色
//                cv::imshow("ttc",pesudoTTC);

                /// 3-避障决策
//                int lineR = findHorizontalLine(ttcResult);
//                Mat aboveLine = ttcMap(Rect(0,0,ttcResult.cols,lineR));
                int stroop = 2;
                Point2f vel;
                if(stroop!=2){
                    int amuzi = balanceTTC(ttcResult,stroop);
                    cout<<"amuzi:"<<amuzi<<endl;
                    vel = getAmuziVel(amuzi);
                }else{
                    int amuzi = balanceTTC(ttcResult,stroop);
                    vel.x = 1.20;
//                    vel.y = (amuzi==0) ? 2.0 : -2.0;
                    switch(amuzi){
                        case 0:{
                            vel.y = 1.20;
                            break;
                        }case 1:{
                            vel.y = -1.20;
                        }case 2:{
                            vel.y = 0.0;
                        }
                    }
                }

//                // 3-避障决策
//                plotVbar(plotDescision,amuzi,stroop);
//                cv::imshow("decision",plotDescision);
                // 4-人工势场路径规划
//              Point2f vel = SteeringAPF(rfoe,ttcResult);

                /// 5-发送改变PX4航向的指令
                /// 5-1 速度控制
//                // 当前姿态角
//                Eigen::Quaterniond cur_allti_qua(current_pose.pose.orientation.w,current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z);
//                Eigen::Vector3d cur_eulerAngle = cur_allti_qua.matrix().eulerAngles(2,1,0); //欧拉角yaw,pitch,roll
//                cout<< "eulerAngle:"<<cur_eulerAngle[0]<<" "<<cur_eulerAngle[1]<<" "<<cur_eulerAngle[2]<<endl;
//                float nav_dire = cur_eulerAngle[0]>0?cur_eulerAngle[0]:-cur_eulerAngle[0];
////                cmd_vel.linear.x = vel.x*cos(cur_eulerAngle[0]) + vel.y*sin(cur_eulerAngle[0]) + current_vel.twist.linear.x; // 固定前向飞行速度
////                cmd_vel.linear.y = vel.y*cos(cur_eulerAngle[0]) + vel.x*sin(cur_eulerAngle[0]) + current_vel.twist.linear.y;
//                cmd_vel.linear.z = 0.0;
//                cmd_vel.angular.x = 0.0;
//                cmd_vel.angular.y = 0.0;
//                cmd_vel.angular.z = atan(vel.y/vel.x); // 航向角改变
//                cout<<"des_vel:"<<cmd_vel.linear.x<<" "<<cmd_vel.linear.y<<endl;
//                ROS_INFO( to_string(vel.x)+" "+ to_string(vel.y) );

//                // 油门
//                cmd_thrust.thrust = 0.8;
//                // 姿态
//                cmd_attitude.pose.position.x = 3.0;
//                cmd_attitude.pose.position.y = 3.0;
//                cmd_attitude.pose.position.z = 1.0;
//                cmd_attitude.pose.orientation.x = 0.0;
//                cmd_attitude.pose.orientation.y = 0.0;
//                cmd_attitude.pose.orientation.z = 0.0;
//                cmd_attitude.pose.orientation.w = 0.0;
//                // 姿态速度
//                cmd_att_vel.twist.angular.x = 0.0;
//                cmd_att_vel.twist.angular.y = 0.0;
//                cmd_att_vel.twist.angular.z = 0.0;
//                cmd_att_vel.twist.linear.x = 2.0;
//                cmd_att_vel.twist.linear.y = 3.0;
//                cmd_att_vel.twist.linear.z = 0.0;
//                    pub_thrust.publish(cmd_thrust);
//                    pub_attitude.publish(cmd_attitude);
//                    pub_vel.publish(cmd_att_vel);

                /// 5-2 位置控制+速度控制
                // 当前姿态角
                Eigen::Quaterniond cur_allti_qua(current_pose.pose.orientation.w,current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z);
                //欧拉角yaw,pitch,roll
                Eigen::Vector3d cur_eulerAngle = cur_allti_qua.matrix().eulerAngles(2,1,0);
                cout<< "eulerAngle:"<<cur_eulerAngle[0]<<" "<<cur_eulerAngle[1]<<" "<<cur_eulerAngle[2]<<endl;
                cout<<"angleChange:"<<atan(vel.y/vel.x)<<endl;
                // 改变航向角
                cur_eulerAngle[0] += atan(vel.y/vel.x);
                cout<<"cmd_pose:"<<cur_eulerAngle[0]<<endl;
                // 欧拉角转四元数
                Eigen::AngleAxisd rollAngle(AngleAxisd(cur_eulerAngle[2],Vector3d::UnitX()));
                Eigen::AngleAxisd pitchAngle(AngleAxisd(cur_eulerAngle[1],Vector3d::UnitY()));
                Eigen::AngleAxisd yawAngle(AngleAxisd(cur_eulerAngle[0],Vector3d::UnitZ()));
                Eigen::Quaterniond quaternion;
                quaternion=yawAngle*pitchAngle*rollAngle;
                cmd_pose.pose.orientation.x = quaternion.x();
                cmd_pose.pose.orientation.y = quaternion.y();
                cmd_pose.pose.orientation.z = quaternion.z();
                cmd_pose.pose.orientation.w = quaternion.w();
                // 速度控制
                cmd_vel.linear.x = current_vel.twist.linear.x; // 固定前向飞行速度
                cmd_vel.linear.y = current_vel.twist.linear.y;
                cmd_vel.linear.z = current_vel.twist.linear.z;
//                cmd_vel.angular.x = 0.0;
//                cmd_vel.angular.y = 0.0;
//                cmd_vel.angular.z = 0.0; // 航向角改变

                prev_frame = curr_frame.clone();
            }
            cv::imshow("left_optflow",leftImg);
//            file_name = save_path+"/image_left/"+to_string(frameCount)+".png";
//            cv::imwrite(file_name,leftImg);

            frameCount++;
            cv::waitKey(1);
        }catch (cv_bridge::Exception& e){
            cout << "Could not convert from " << msg->encoding.c_str() << "to 'brg8'." << endl;
        }

        localvel_pub.publish(cmd_vel);
        localpose_pub.publish(cmd_pose);

        cout<<"send vel "<<endl;
        cout<<"***************************"<<endl;
        double end =getTickCount();
        cout<<"Time:"<<(double) (end-start)/getTickFrequency()<<endl;
        cout<<"***************************"<<endl;
    }

private:
    ros::NodeHandle n_;
    ros::Publisher localvel_pub;
    ros::Publisher localpose_pub;

    ros::Publisher pub_thrust,pub_vel,pub_attitude;
    ros::Subscriber sub_,state_sub;
    ros::Subscriber local_pose_sub,local_vel_sub;//当前位置和当前速度订阅
    ros::ServiceClient arming_client,set_mode_client;

    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::TwistStamped current_vel;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::PoseStamped cmd_pose;
    bool init_frame = false;
    cv::Mat prev_frame,curr_frame;//灰度图
    string save_path = "/home/wbzhang/bags/optical_test/";
    string file_name;
    int frameCount;

};//End of class SubscribeAndPublish


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "px4_obs_node");

//    ros::Rate rate(5);
    //Create an object of class SubscribeAndPublish that will take care of everything
//    while(ros::ok()) {
        SubscribeAndPublish SAPObject;
//        ros::spinOnce();
//        rate.sleep();
//    }

    ros::spin();
    cout<<"px4_obs_node shutdown!"<<endl;
    return 0;
}
