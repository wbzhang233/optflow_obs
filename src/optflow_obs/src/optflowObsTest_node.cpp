//
// Created by wbzhang on 2020/6/25.
//

#include "../include/opflowAvoidanceClass.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "px4_obs_node");
    ros::NodeHandle nh;

    opflowAvoidanceClass optflowObs(nh);
//    optflowObs.run();

    ros::spin();
    cout<<"optflowClassTest_node shutdown!"<<endl;
    return 0;
}
