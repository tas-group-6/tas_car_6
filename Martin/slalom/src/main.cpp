//ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

//STL
#include <sstream>


//My includes

#include "RobotClass.h"
#include "control.h"

// Update rate for integration and main loop. In Hz
const double update_rate = 1.0;



void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}



int main(int argc, char **argv)
{

#ifdef USE_PLAYER
    try
    {
#endif

        ros::init(argc, argv, "slalom");

        //ros::NodeHandle n;
        //ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 1000);
        //ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("chatter", 5);
        //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);


        RobotClass robot;


        ros::Rate loop_rate(update_rate);


//        sensor_msgs::LaserScan scan;

        std::cout << "All stuff created" << std::endl;

        int count = 0;
        while (ros::ok())
        {



            //        if(autonomous_control.control_Mode.data==0)
            //        {
            //            ROS_INFO("Manually Control!");
            //        }
            //        else
            //        {
            //            if(autonomous_control.control_Brake.data==1)
            //            {
            //                autonomous_control.control_servo.x=1500;
            //                autonomous_control.control_servo.y=1500;
            //            }
            //            else
            //            {
            //                ROS_INFO("Automatic Control!");

            //                if(autonomous_control.cmd_linearVelocity>0)
            //                {
            //                    autonomous_control.control_servo.x = 1550;
            //                }

            //                else if(autonomous_control.cmd_linearVelocity<0)
            //                {
            //                    autonomous_control.control_servo.x = 1300;
            //                }
            //                else
            //                {
            //                    autonomous_control.control_servo.x = 1500;
            //                }

            //                autonomous_control.control_servo.y = autonomous_control.cmd_steeringAngle;
            //            }

            //            autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);

            //        }


            //Call the update function of the robot to get new values from stage / player
            robot.update();
            //robot.FillScanMessage(scan);

            //        // This is a message object. You stuff it with data, and then publish it.
            //        std_msgs::String msg;
            //        std::stringstream ss;
            //        ss << "hello world " << count;
            //        msg.data = ss.str();
            //        ROS_INFO("%s", msg.data.c_str());

            //std::cout << "Laser Scan: " <<  count << std::endl;

            //pub.publish(msg);
            //pub.publish(scan);



            ros::spinOnce(); //Callbacks are called when required

            loop_rate.sleep();
            count++;
        }

#ifdef USE_PLAYER
    }
    catch( PlayerCc::PlayerError e)
    {
        printf("-> Catched Player Error <-\nFunction:\t %s\nMessage:\t %s\nError Code:\t %d\n", e.GetErrorFun().c_str(), e.GetErrorStr().c_str(), e.GetErrorCode());
        return e.GetErrorCode();
    }
#endif

    return 1;
}
