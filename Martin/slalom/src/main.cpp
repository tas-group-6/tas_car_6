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
// Wii contorl logger writes with 100 Hz
// Lower rate here to give the car more time
const double update_rate = 50.0;



void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    std::printf("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{

    try
    {

        ros::init(argc, argv, "slalom");


        //ros::NodeHandle n;
        //ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 1000);
        //ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("chatter", 5);
        //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
        //sensor_msgs::LaserScan scan;

        RobotClass robot;
        control autonomous_control;
        ServoInstructionType instruction;

        ros::Rate loop_rate(update_rate);

        std::cout << "All stuff created" << std::endl;

        int count = 0;
        bool result = false;

        while (ros::ok())
        {
            if( autonomous_control.control_Mode.data==0) std::printf("Manually Control!\n");
            else
            {
                if( autonomous_control.control_Brake.data==1)
                {
                    std::cout << "Brake Active!" << std::endl;

                    autonomous_control.control_servo.x=1500;
                    autonomous_control.control_servo.y=1500;
                }
                else
                {
                    std::cout << "Automatic Control!"<< std::endl;

                    //Call the update function of the robot to get the new values for the servo
                    //When completed result is false
                    result = robot.update(instruction);
                    if(!result) break;

                    autonomous_control.control_servo.x=instruction.velocity;
                    autonomous_control.control_servo.y=instruction.steer;
                }

                autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);
                std::cout << "Pub: " << instruction.velocity << " " <<  instruction.steer << std::endl;
            }

            //robot.FillScanMessage(scan);

            //        // This is a message object. You stuff it with data, and then publish it.
            //        std_msgs::String msg;
            //        std::stringstream ss;
            //        ss << "hello world " << count;
            //        msg.data = ss.str();
            //        std::printf("%s\n", msg.data.c_str());

            //std::cout << "Laser Scan: " <<  count << std::endl;
            //pub.publish(msg);
            //pub.publish(scan);

            ros::spinOnce(); //Callbacks are called when required

            //loop_rate.sleep(); //ignore sleep for now, the robot class should handle the waiting
            count++;
        }
    }
    catch( ErrorType e)  {    e.what(); return 0; }

#ifdef USE_PLAYER

    catch( PlayerCc::PlayerError e)
    {
        std::cout << "-> Catched Player Error <-" << std::endl;
        std::cout << "Function: " << e.GetErrorFun() << std::endl;
        std::cout << "Message: " << e.GetErrorStr() << std::endl;
        std::cout << "Error Code: " << e.GetErrorCode() << std::endl;
        return e.GetErrorCode();
    }
#endif

    return 1;
}
