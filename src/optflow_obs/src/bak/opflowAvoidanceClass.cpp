//
// Created by wbzhang on 2020/6/25.
//

#include "../include/opflowAvoidanceClass.h"


/// 构造函数
opflowAvoidanceClass::opflowAvoidanceClass(ros::NodeHandle &nh):n_(nh)
{
    this->init_frame = false;
    this->init();
    this->switchOffboard();
}

// 析构函数
opflowAvoidanceClass::~opflowAvoidanceClass(){
    ROS_INFO("opflowAvoidanceClass is done...");
}

/// 初始化函数
inline void opflowAvoidanceClass::init(){
    ROS_INFO("opflowAvoidanceClass: start initialising");
    //Topic you want to publish
    this->raw_pub = this->n_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 30);
    this->path_pub = this->n_.advertise<nav_msgs::Path>("/mavros/px4/local_path", 30);
    path.header.frame_id="/map";

    //Topic you want to subscribe
    this->state_sub = this->n_.subscribe<mavros_msgs::State>("/mavros/state", 10, &opflowAvoidanceClass::state_cb,this); //飞机状态
    this->local_pose_sub = this->n_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 5, &opflowAvoidanceClass::getLocalPoseFromMsg,this);//飞机当前位置
    this->local_vel_sub = this->n_.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 5, &opflowAvoidanceClass::getLocalVelFromMsg,this);//飞机当前速度
    this->arming_client = this->n_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming"); //加锁
    this->set_mode_client = this->n_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode"); // 设置模式
    // 航迹点订阅
    this->waypoint_sub = this->n_.subscribe<mavros_msgs::WaypointList>("/mavros/mission/waypoints",5,&opflowAvoidanceClass::getWaypointList,this);

    ROS_INFO("opflowAvoidanceClass: iInitialising done");
    this->last_request = ros::Time::now();
    this->path_pub.publish(path);
}

/*
 * 切换到OFFBOARD模式
 */

inline void opflowAvoidanceClass::switchOffboard()
{
    ROS_INFO("switchOffboard function");
    ros::Rate rate(50.0);

    /// Variabelen voor switching naar offboard en arm
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    this->last_request = ros::Time::now();

    while(this->current_state.mode != "OFFBOARD" || !current_state.armed){
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(0.20))) {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                zweven();
                // 启用光流控制
                this->sub_ = n_.subscribe("/iris_fpv_cam/usb_cam/image_raw", 1, &opflowAvoidanceClass::optobsCallback, this);
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

    ros::spinOnce();
    return;
}


/* OFFBOARD模式自稳
 * Function used while switching the mode, 0m/s messages will be sended to the drone. The drone will hover
 */

inline void opflowAvoidanceClass::zweven(void)
{
    mavros_msgs::PositionTarget velocity_msg;
    velocity_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    velocity_msg.header.frame_id = "/drone";
    velocity_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                             mavros_msgs::PositionTarget::IGNORE_PY |
                             mavros_msgs::PositionTarget::IGNORE_PZ |
                             mavros_msgs::PositionTarget::IGNORE_AFX |
                             mavros_msgs::PositionTarget::IGNORE_AFY |
                             mavros_msgs::PositionTarget::IGNORE_AFZ |
                             mavros_msgs::PositionTarget::FORCE |
//                             mavros_msgs::PositionTarget::IGNORE_YAW |
                             mavros_msgs::PositionTarget::IGNORE_YAW_RATE ;
    velocity_msg.header.stamp = ros::Time::now();

    velocity_msg.velocity.x = 0.0;
    velocity_msg.velocity.y = 0.0;
    velocity_msg.velocity.z = 0.0;
    velocity_msg.yaw = 0.0;

    this->raw_pub.publish(velocity_msg);
    ros::spinOnce();
    return;
}

/*
 * 获得当前的航迹点列表
 */
inline void opflowAvoidanceClass::getWaypointList(const mavros_msgs::WaypointList::ConstPtr& msg){
    this->curr_waypoints = *msg;
}


/*
 * 获得当前的状态消息
 */
inline void opflowAvoidanceClass::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

/*
 * 获取当前位置消息
 */
inline void opflowAvoidanceClass::getLocalPoseFromMsg(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose=*msg;
    path.poses.push_back(current_pose);
}

/*
 * 获取当前速度消息
 */
inline void opflowAvoidanceClass::getLocalVelFromMsg(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_vel=*msg;
}

///  光流避障处理回调
inline void opflowAvoidanceClass::optobsCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("optobsCallback starting...");

    double start=(double) getTickCount();
    //.... do something with the input and generate the cmd_vel...
    try{
        cout << "FPV camera image receive success " << endl;
        cv::Mat image = cv_bridge::toCvShare(msg,"bgr8")->image;
        cv::cvtColor(image,curr_frame,cv::COLOR_BGR2GRAY);
        cv::Mat flow,ttcResult;
        cv::Mat pesudoTTC;
        int amuzi;

        if(!this->init_frame){
            this->prev_frame =  this->curr_frame;//初始化第一帧
            cv::imshow("optflow",image);
            cout<<"****** First frame initialized! ******"<<endl;
            init_frame = true;
        }else{
//            // 00- 先计算特征点匹配，前后者帧之间的运动
//            Movementor mv(prev_frame,curr_frame);
//            mv.run();

            // 0-DIS光流法计算光流
//            double dis_start =  getTickCount();
            Ptr<DenseOpticalFlow> algorithm = DISOpticalFlow::create(DISOpticalFlow::PRESET_MEDIUM);
            algorithm->calc(prev_frame, curr_frame, flow);
//            double dis_end = getTickCount();
//            cout<<"***************************"<<endl;
//            cout<<"dis_Time:"<<(double) (dis_end-dis_start)/getTickFrequency()<<endl;
//            cout<<"***************************"<<endl;

//             1-计算FOE
            double foe_start =  getTickCount();
            float loss_final;
            Point2f rfoe = getDenseRansacFOE(flow,loss_final,35);
//            cout<<"FOE:"<<rfoe.x<<" "<<rfoe.y<<endl;
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

            /// 新增;只有当FOE估计值足够准确时，产生径向光流才计算TTC和决策
            cmd_raw_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            cmd_raw_msg.header.frame_id = "/map";
            cmd_raw_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                    mavros_msgs::PositionTarget::IGNORE_PY |
                                    mavros_msgs::PositionTarget::IGNORE_PZ |
                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::FORCE |
                                    //mavros_msgs::PositionTarget::IGNORE_YAW |
                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE ;
            cmd_raw_msg.header.stamp = ros::Time::now();

            if(loss_final<60){
                cout<<"loss_final:"<<loss_final<<endl;
                // 2-计算TTC
                Mat ttcMap = timeToContact(flow,rfoe);
    //            ttcResult = normToGray(ttcMap);
                // cv::applyColorMap(ttcResult,pesudoTTC,cv::COLORMAP_JET); //伪彩色
                //cv::imshow("ttc",pesudoTTC);

                /// 3-避障决策
    ////                int lineR = findHorizontalLine(ttcResult);
    ////                Mat aboveLine = ttcMap(Rect(0,0,ttcResult.cols,lineR));
                int stroop = 8;
                Point2f vel;
                if(stroop!=2){
                    amuzi = balanceTTC(ttcMap,stroop);
                    cout<<"amuzi:"<<amuzi<<endl;
                    vel = getAmuziVel(amuzi);
                }else{
                    amuzi = balanceTTC(ttcResult,stroop);
                    vel.x = 1.20;
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
                plotVbar(image,amuzi,stroop);

                // 4-人工势场路径规划
                // Point2f vel = SteeringAPF(rfoe,ttcMap);
                // 人工势场法1
//                Point2f vel = APFDirection3(att_diff,ttcMap);

                /// 5-setpoint_raw 速度加偏航控制
                cmd_raw_msg.velocity.x = 0.0;
                cmd_raw_msg.velocity.y = 2.0;
                cmd_raw_msg.velocity.z = 0.0;
                /// 偏航角控制
                if(stroop == 2){
                    if(vel.x>0){
                        cmd_raw_msg.yaw = -M_PI/15; //右转，顺时针为负
                    }else{
                        cmd_raw_msg.yaw = M_PI/15;
                    }
                } else cmd_raw_msg.yaw = atan(vel.x/vel.y);
//                cout<<"cmd_raw_vel:"<<cmd_raw_msg.velocity.x<<" "<<cmd_raw_msg.velocity.y<< endl;
                cout<<"cmd_raw_yaw:"<<cmd_raw_msg.yaw<<endl;

            }else{
                // 4-1 获取下一个航迹点
//                mavros_msgs::Waypoint curr_waypoint = curr_waypoints.waypoints[curr_waypoints.current_seq];
                Point3f nextWaypoint(79.2,5,0.05);
                Point2f pose_vec(nextWaypoint.x-current_pose.pose.position.x, nextWaypoint.y-current_pose.pose.position.y);

                // 4-2 获取飞机朝向
                // 输出当前速度
                cout<<"current_vel:"<<current_vel.twist.linear.x<<" "<<current_vel.twist.linear.y<<" "<<current_vel.twist.linear.z<<endl;
                Point2f heading(current_vel.twist.linear.x, current_vel.twist.linear.y);
                // 当前航向与连线之间的角度差
                double att_diff = getAngleDiff2(heading,pose_vec);
                cout<<"att_diff:"<<att_diff<<endl;
                /// 速度加偏航控制
                cmd_raw_msg.velocity.x = 0.0;
                cmd_raw_msg.velocity.y = 2.0;
                cmd_raw_msg.velocity.z = 0.0;
                cmd_raw_msg.yaw = att_diff/2;
                cout<<"cmd_raw_yaw:"<<cmd_raw_msg.yaw<<endl;

            }

            prev_frame = curr_frame.clone();
        }
        cv::imshow("optflow",image);

        cv::waitKey(5);
    }catch (cv_bridge::Exception& e){
        cout << "Could not convert from " << msg->encoding.c_str() << "to 'brg8'." << endl;
    }

    this->raw_pub.publish(cmd_raw_msg);
    cout<<"send cmd_raw_msg"<<endl;
    cout<<"***************************"<<endl;
    double end =getTickCount();
    cout<<"Time:"<<(double) (end-start)/getTickFrequency()<<endl;
    cout<<"***************************"<<endl;
    this->path_pub.publish(path);

    /// 新增： 获取当前机头在北东地NED坐标系下的朝向
    ROS_INFO("Heading in NED now");
//    // 当前姿态角
//    Eigen::Quaterniond curAalltiQuantion(current_pose.pose.orientation.w,current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z);
//    //欧拉角yaw,pitch,roll
//    Eigen::Vector3d currEulerAngle = curAalltiQuantion.matrix().eulerAngles(2,1,0); //roll,pitch,yaw
//    cout<< "eulerAngle:"<<currEulerAngle[0]<<" "<<currEulerAngle[1]<<" "<<currEulerAngle[2]<<endl;



    ROS_INFO("optobsCallback done...");
}