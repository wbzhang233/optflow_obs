//
// Created by wbzhang on 2020/6/8.
//
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h> //ROS图象类型的编码函数
#include <image_transport/image_transport.h> //用来在ROS系统中的话题上发布和订阅图象消息
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/PositionTarget.h>
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
        ROS_INFO("SubscribeAndPublish: start initialising");

        //Topic you want to publish
        raw_pub = n_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 30);
        
        //Topic you want to subscribe
        this->state_sub = n_.subscribe<mavros_msgs::State>("mavros/state", 10, &SubscribeAndPublish::state_cb,this); //飞机状态
//        local_pose_sub = n_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 5, &SubscribeAndPublish::getLocalPoseFromMsg,this);//飞机当前位置
//        local_vel_sub = n_.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 5, &SubscribeAndPublish::getLocalVelFromMsg,this);//飞机当前速度
        arming_client = n_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client = n_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

        // 用于控制飞机保持OFFBOARD模式和解锁
        ros::Rate rate(50.0);
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        ros::Time last_request = ros::Time::now();
        while(current_state.mode != "OFFBOARD" || !current_state.armed) {
            if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(0.20))) {
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                    zweven();
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if (!current_state.armed) {
                    if (arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                        zweven();
                        ROS_INFO("Vehicle armed");
                    }
                }
            }
            ros::spinOnce();
            rate.sleep();
        }

        this->sub_ = n_.subscribe("/iris_fpv_cam/usb_cam/image_raw", 1, &SubscribeAndPublish::callback, this);
        ROS_INFO("SubscribeAndPublish: done");
    }

    void zweven()
    {
        mavros_msgs::PositionTarget velocity_msg;
        velocity_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        velocity_msg.header.frame_id = "drone";
        velocity_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                 mavros_msgs::PositionTarget::IGNORE_PY |
                                 mavros_msgs::PositionTarget::IGNORE_PZ |
                                 mavros_msgs::PositionTarget::IGNORE_AFX |
                                 mavros_msgs::PositionTarget::IGNORE_AFY |
                                 mavros_msgs::PositionTarget::IGNORE_AFZ |
                                 mavros_msgs::PositionTarget::FORCE |
                                 //mavros_msgs::PositionTarget::IGNORE_YAW |
                                 mavros_msgs::PositionTarget::IGNORE_YAW_RATE ;
        velocity_msg.header.stamp = ros::Time::now();

        velocity_msg.velocity.x = 0.0;
        velocity_msg.velocity.y = 0.0;
        velocity_msg.velocity.z = 0.0;
        velocity_msg.yaw = 0.0;

        this->raw_pub.publish(velocity_msg);
        ros::spinOnce();
        ROS_INFO("ZWEVEN DONE");
        return;
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
        double start=(double) getTickCount();
        frameCount = 0;
        //.... do something with the input and generate the cmd_vel...
        try{
            cout << " FPV image receive success " << endl;
            cv::Mat image = cv_bridge::toCvShare(msg,"bgr8")->image;
            cv::cvtColor(image,curr_frame,cv::COLOR_BGR2GRAY);
            cv::Mat flow,ttcResult,pesudoTTC;
            int amuzi;

            if(!init_frame){
                prev_frame = curr_frame;//初始化第一帧
                cv::imshow("optflow",image);
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
                    circle(image,Point(cvRound(disFlow.pts[i].x), cvRound(disFlow.pts[i].y)),
                           3,Scalar(120,0,120),-1,LINE_AA); //Scalar(125,0,125)
                    line(image, Point(cvRound(disFlow.pts[i].x), cvRound(disFlow.pts[i].y)),
                         Point(cvRound(disFlow.pts[i].x+disFlow.flows[i].x),cvRound(disFlow.pts[i].y+disFlow.flows[i].y)),
                         Scalar(0, 50, 255), 1, LINE_AA);//Scalar(0, 50, 200)
                }
                cv::circle(image,rfoe,3,Scalar(0,255,255),-1,cv::LINE_AA);

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
                    amuzi = balanceTTC(ttcResult,stroop);
                    vel.y = 1.20;
//                    vel.y = (amuzi==0) ? 2.0 : -2.0;
                    switch(amuzi){
                        case 0:{
                            vel.x = 1.20;
                            break;
                        }case 1:{
                            vel.x = -1.20;
                        }case 2:{
                            vel.x = 0.0;
                        }
                    }
                }
                plotVbar(image,amuzi,stroop);

                /// 4-人工势场路径规划
//              Point2f vel = SteeringAPF(rfoe,ttcResult);

                /// 5-setpoint_raw 速度加偏航控制
                cmd_raw_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
                cmd_raw_msg.header.frame_id = "drone";
                cmd_raw_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                         mavros_msgs::PositionTarget::IGNORE_PY |
                                         mavros_msgs::PositionTarget::IGNORE_PZ |
                                         mavros_msgs::PositionTarget::IGNORE_AFX |
                                         mavros_msgs::PositionTarget::IGNORE_AFY |
                                         mavros_msgs::PositionTarget::IGNORE_AFZ |
//                                         mavros_msgs::PositionTarget::IGNORE_VX |
//                                         mavros_msgs::PositionTarget::IGNORE_VY |
//                                         mavros_msgs::PositionTarget::IGNORE_VZ |
                                         mavros_msgs::PositionTarget::FORCE |
//                                         mavros_msgs::PositionTarget::IGNORE_YAW |
                                         mavros_msgs::PositionTarget::IGNORE_YAW_RATE ;
                cmd_raw_msg.header.stamp = ros::Time::now();
                cmd_raw_msg.velocity.x = 0.0;
                cmd_raw_msg.velocity.y = 1.00;
                cmd_raw_msg.velocity.z = 0.0;
                /// 偏航角控制
                if(stroop == 2){
                    if(vel.x>0){
                        cmd_raw_msg.yaw = -M_PI/8; //右转，顺时针为负
                    }else{
                        cmd_raw_msg.yaw = M_PI/8;
                    }
                } else cmd_raw_msg.yaw = atan(vel.x/vel.y);
                cout<<"cmd_raw_vel:"<<cmd_raw_msg.velocity.x<<" "<<cmd_raw_msg.velocity.y<< endl;
                cout<<"cmd_raw_yaw:"<<cmd_raw_msg.yaw<<endl;

                prev_frame = curr_frame.clone();
            }
            cv::imshow("optflow",image);
//            file_name = save_path+"/image_left/"+to_string(frameCount)+".png";
//            cv::imwrite(file_name,image);

            frameCount++;
            cv::waitKey(1);
        }catch (cv_bridge::Exception& e){
            cout << "Could not convert from " << msg->encoding.c_str() << "to 'brg8'." << endl;
        }

        raw_pub.publish(cmd_raw_msg);
        cout<<"send cmd_raw_msg"<<endl;
        cout<<"***************************"<<endl;
        double end =getTickCount();
        cout<<"Time:"<<(double) (end-start)/getTickFrequency()<<endl;
        cout<<"***************************"<<endl;
    }

private:
    ros::NodeHandle n_;
    // setpoint_raw消息发布器
    ros::Publisher raw_pub;
    mavros_msgs::PositionTarget cmd_raw_msg;

    // 加锁和模式控制
    ros::Subscriber sub_,state_sub;
    ros::ServiceClient arming_client,set_mode_client;

    //当前位置和当前速度订阅
    ros::Subscriber local_pose_sub,local_vel_sub;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::TwistStamped current_vel;

    // 光流变量
    bool init_frame = false;
    cv::Mat prev_frame,curr_frame;//灰度图
    string save_path = "/home/wbzhang/bags/optical_test/";
    string file_name;
    int frameCount;
};//End of class SubscribeAndPublish


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "px4_obs_node");

    SubscribeAndPublish SAPObject;

    ros::spin();
    cout<<"px4_obs_node shutdown!"<<endl;
    return 0;
}
