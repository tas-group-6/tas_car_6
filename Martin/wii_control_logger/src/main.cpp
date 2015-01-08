#include "wii_lib.h"

int main(int argc, char* argv[])
{
    if( argc != 2)
    {
        std::cout << "Wrong number of commandline arguments" << std::endl;
        std::cout << "Usage: rosrun wii_control_logger wii_control_logger [filename]" << std::endl;
        return -1;
    }

    ros::init(argc, argv, "teleop_wii_logger");

    wii_lib* teleop_wii = 0;

    try { teleop_wii = new wii_lib( argv[1]);    }
    catch( std::string s ) { std::cout << s << std::endl;    }


    std::cout << "Ready for recording. Press Button A" << std::endl;

    ros::Rate loop_rate(100.0);

//    static int count = 0;

    while(ros::ok())
    {
        //        teleop_wii.wii_communication_pub.publish(teleop_wii.wii_state_);
        //ROS_INFO("Debug message!");
        ros::spinOnce();
        loop_rate.sleep();
    }



    if( teleop_wii) { delete teleop_wii; teleop_wii = 0;}

    return 0;
}


