#include "wii_lib.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_wii_logger");
    wii_lib teleop_wii;

    ros::Rate loop_rate(10);

    static int count = 0;

    while(ros::ok())
    {
        //        teleop_wii.wii_communication_pub.publish(teleop_wii.wii_state_);
        //ROS_INFO("Debug message!");
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


