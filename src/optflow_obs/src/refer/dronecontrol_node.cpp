/* DRONECONTROL_NODE
 * Node to send velocities obtained from drone_snelheden topic
 * Includes safetyswitch (channel 5) to switch between offboard an manual mode
 */

#include "classheader_dronecontrol.hpp"

DroneControl::DroneControl(ros::NodeHandle n)
{
    initRos(n);
}

/* INITROS()
 * declaring publishers/subscribers 
 * If boolean manual is true, mode will be switched to manual
 * else mode will be switched to offboard mode
 */

void DroneControl::initRos(ros::NodeHandle n)
{
    ROS_INFO("DroneControl: start initialising");
    this->state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, &DroneControl::stateCallback, this);
    this->rcin_sub = n.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 10, &DroneControl::rcinCallback, this);
    this->dronesnelheden_sub = n.subscribe("drone_snelheden", 1, &DroneControl::snelhedenCallback, this);
    this->local_pos_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    this->vel_pub = n.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    this->start_pidcontrol = n.advertise<dronecontrol_node::startPidcontrol>("pid_start", 10);
    this->arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    this->set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    this->velocity_pub = n.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ROS_INFO("DroneControl: iInitialising done");

    this->ofb = n.subscribe("ofb", 1, &DroneControl::ofbCallback, this);

    /// Als de tijd tussen twee OFFBOARD commando's groter is dan 500ms, dan valt
    /// OFFBOARD mode uit -> rate > 2Hz
    ros::Rate rate(50.0);

    /// Wachten tot pixhawk verbonden is en tot manual schakelaar aan staat
    while(ros::ok() && current_state.connected /*&& this->manual*/){
        ros::spinOnce();
        rate.sleep();
    }

    /// Enkele setpoints doorsturen anders wordt de mode switch genegeerd
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    if(this->manual){
        switchManual();
    }
    else{
        switchOffboard();
    }

    return;
}

/* DRONEBESTURING()
 * As long as boolean manual is false, messages with velocities will be published to the drone
 * For testing purposes, fixed velocities are used
 * When boolean manual is true, the function switchmanual will be called
 */

void DroneControl::dronebesturing()
{
    ROS_INFO("in dronebesturing");
    ros::Rate rate(50.0);

    while(ros::ok() && !this->manual){
        mavros_msgs::PositionTarget velocity_msg;
        velocity_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        velocity_msg.header.frame_id = "drone";
        velocity_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                                 mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                                 mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                                 mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        velocity_msg.header.stamp = ros::Time::now();

        velocity_msg.position.x = 0.0f;
        velocity_msg.position.y = 0.0f;
        velocity_msg.position.z = 0.0f;
        velocity_msg.acceleration_or_force.x = 0.0f;
        velocity_msg.acceleration_or_force.y = 0.0f;
        velocity_msg.acceleration_or_force.z = 0.0f;
        velocity_msg.velocity.x = 0.0f;
        velocity_msg.velocity.y = 0.0f;
        velocity_msg.velocity.z = 0.0;
        velocity_msg.yaw_rate = 0.0f;
      
        velocity_msg.velocity.x = this->xs;
        velocity_msg.velocity.y = this->ys;
        velocity_msg.velocity.z = this->zs;

        if(this->testen){
            velocity_msg.velocity.x = 0.4f;
            velocity_msg.velocity.y = 0.0f;
            velocity_msg.velocity.z = 0.4f;
        }

        //cout << velocity_msg << endl;
        this->velocity_pub.publish(velocity_msg);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("switchManual opvragen");
    switchManual();
    return;
}

/* ZWEVEN
 * Function used while switching the mode, 0m/s messages will be sended to the drone. The drone will hover
 */

void DroneControl::zweven()
{
    mavros_msgs::PositionTarget velocity_msg;
    velocity_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    velocity_msg.header.frame_id = "drone";
    velocity_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                             mavros_msgs::PositionTarget::IGNORE_PY |
                             mavros_msgs::PositionTarget::IGNORE_PZ |
                             mavros_msgs::PositionTarget::IGNORE_AFX |
                             mavros_msgs::PositionTarget::IGNORE_AFY |
                             mavros_msgs::PositionTarget::IGNORE_AFZ |
                             mavros_msgs::PositionTarget::FORCE |
                             mavros_msgs::PositionTarget::IGNORE_YAW |
                             mavros_msgs::PositionTarget::IGNORE_YAW_RATE ;
    velocity_msg.header.stamp = ros::Time::now();

    velocity_msg.velocity.x = 0.0;
    velocity_msg.velocity.y = 0.0;
    velocity_msg.velocity.z = 0.0;

    this->velocity_pub.publish(velocity_msg);
    ros::spinOnce();
    return;
}

/* SWITCHOFFBOARD
 * Switching to offboard, can take longer so multiple requests. While switching the drone
 * will hover caused by the function zweven
 * After switching to offboard mode, a start signal will be sended to the pidcontrol_node
 * to inform him that he can start sending velocities to the drone_snelheden topic
 * Function dronebesturing will be called to control the drone
 */

void DroneControl::switchOffboard()
{
    ROS_INFO("switchOffboard functie");

    ros::Rate rate(50.0);

    /// Variabelen voor switching naar offboard en armen
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    while(this->current_state.mode != "OFFBOARD"){
        set_mode_client.call(offb_set_mode);
        offb_set_mode.response.mode_sent;
        zweven();
        ros::spinOnce();
        arming_client.call(arm_cmd);

        zweven();
        ros::spinOnce();
        cout << "offboard lus" << endl;
    }
    ROS_INFO("Offboard enabled");
    ROS_INFO("Vehicle armed");

    this->startPidcontrol_msg.start = true;
    this->start_pidcontrol.publish(this->startPidcontrol_msg);
    ros::spinOnce();

    dronebesturing();
    return;
}

/* SWITCHMANUAL
 * Switching the mode back to manual
 * As long as the boolean manual is not false the node will be idle, otherwise
 * the function switchOffboard will be called
 */

void DroneControl::switchManual()
{
    ROS_INFO("switchManual functie");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "MANUAL";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    ros::Rate rate(50.0);

    set_mode_client.call(offb_set_mode);
    offb_set_mode.response.mode_sent;
    ROS_INFO("Offboard disabled");
    zweven();
    ros::spinOnce();

    this->startPidcontrol_msg.start = false;
    this->start_pidcontrol.publish(this->startPidcontrol_msg);
    ros::spinOnce();

    while(ros::ok()){
        if(!this->manual){
            ROS_INFO("switchOffboard opvragen");
            switchOffboard();
        }
        ros::spinOnce();
        rate.sleep();
    }
    return;
}

/* STATECALLBACK
 * Callbackfunction for state of drone
 * Heeft de vorm:
 * std_msgs/Header header  |   seq: 791
 *                         |   stamp: 1512122384.704551543
 *                         |   frame_id:
 * bool connected          |   connected: 1
 * bool armed              |   armed: 0
 * bool guided             |   guided: 8
 * string mode             |   mode: OFFBOARD
 * uint8 system_status     |   system_status: 3
 */

void DroneControl::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    this->current_state = *msg;
}


/*
 * RCINCALLBACK
 * Safetyswitch callback, if switch value >= 1520 then switching to manual
 * Plaatst het message van topic mavros/rc/in in rcin_value.
 * Als channel[SAFETYSWITCH] >= 1520 dan manual = true (naar manual mode).
 * Heeft de vorm:
 * std_msgs/Header header  |   seq: 791
 *                         |   stamp: 1512122384.704551543
 *                         |   frame_id:
 * uint8 rssi              |
 * uint16[] channels       |
 */

void DroneControl::rcinCallback(const mavros_msgs::RCIn::ConstPtr& msg)
{
    this->rcin_value = *msg;
    if(this->rcin_value.channels[SAFETYSWITCH] >= 1520){
        this->manual = true;
    }
    else{
        this->manual = false;
    }
}

/* OFBCALLBACK
 * Same as previous function
 */

void DroneControl::ofbCallback(const std_msgs::Int64::ConstPtr& msg)
{
    cout << "bericht ontvangen" << endl;
    cout << msg->data << endl;
    if(msg->data == 1){
        this->manual = false;
    }
    else{
        this->manual = true;
    }
}

/* SNELHEDENCALLBACK
 * Inserting the velocities from dronesnelheden topic into global variables
 */

void DroneControl::snelhedenCallback(const pidcontrol_node::droneSnelheden& msg)
{
    this->xs = (float)msg.xs;
    this->ys = (float)msg.ys;
    this->zs = (float)msg.zs;

    this->nieuweSnelheden = true;
}