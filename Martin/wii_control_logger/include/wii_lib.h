#ifndef WII_LIB_H
#define WII_LIB_H

#include <ros/ros.h>
//#include <turtlesim/Velocity.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <wiimote/State.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>

#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <chrono>

#define WII_BUTTON_1            0
#define WII_BUTTON_2            1
#define WII_BUTTON_PLUS         2
#define WII_BUTTON_MINUS        3
#define WII_BUTTON_A            4
#define WII_BUTTON_B            5
#define WII_BUTTON_ROKERUP      6
#define WII_BUTTON_ROKERDOWN    7
#define WII_BUTTON_ROKERLEFT    8
#define WII_BUTTON_ROKERRIGHT   9
#define WII_BUTTON_HOME         10

#define WII_BUTTON_NUNCHUK_Z    0
#define WII_BUTTON_NUNCHUK_C    1

#define SCALE_FAKTOR_STEERING   500

#define PI                      3.14159265

#define CAR_LENGTH              0.355

class wii_lib
{
public:
    wii_lib(const char* filename);
    ~wii_lib();

    ros::NodeHandle nh_;
//    ros::Publisher wii_servo_pub_;
//    ros::Publisher wii_communication_pub;
    ros::Subscriber wii_sub_;

    /* flags for the control mode and brake */
    std_msgs::Int16 emergencyBrake;
    std_msgs::Int16 controlMode;

    /* scale factor for joystick*/
    double SCALE_FAKTOR_THROTTLE;

    geometry_msgs::Vector3 servo;
    std_msgs::Int16MultiArray wii_state_; //Public state of wii, used in main to publish brake and autonomus state

private:
    /* subscribe the wii joystick states and send servo messages when it is manual mode*/
    /*outputs: publishes directly to servo to control the car when its under manual control*/
    /*outputs: flags for wii_state_.data[0] and wii_state_.data[1] to indicate if the buttons are pressed or not*/
    void wiiStateCallback(const wiimote::State::ConstPtr& wiiState);

    /* initialize the msg for the communication between wii_control and trajector_control*/
    void msg_Initialization(std_msgs::Int16MultiArray& msg);


    std::ofstream m_ofs; // Output file stream for logging
    bool m_logging_enabled; //Variable to switch logging on and off


    std::vector<int> m_wii_state_old;
    std::vector<int> m_nunchuck_state_old;

    std::chrono::steady_clock::time_point m_time_start;

};

#endif // WII_LIB_H
