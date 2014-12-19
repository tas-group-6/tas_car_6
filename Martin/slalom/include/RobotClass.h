#pragma once


///Enables the use of player to simulate laser scans
//#define USE_PLAYER

#include <sensor_msgs/LaserScan.h>
#include <wiimote/State.h>


#ifdef USE_PLAYER
#include <libplayerc++/playerc++.h>
#endif

#include <string.h>
#include <sstream>
#include <iostream>


#include "DataType.h"
#include "control.h"


/// Update rate for integration and main loop. In Hz
const double update_rate = 1.0;

class RobotClass
{
public:

    /**
   * This constructor sets up a PlayerClient which connects to localhost:<default-port>
   */
    RobotClass();
    ~RobotClass();


    /// Calls the player client and retrives new data from the sensors
    void update();

    bool FillScanMessage(sensor_msgs::LaserScan& scan) const;

    PoseType Pose() const;
    void Pose(const PoseType& p);

    PoseType Speed() const;
    void Speed(const PoseType& p);

    double getRotation() const;
    void setRotation(double value);

    double getRotationYaw();
    void setRotationYaw(double value);

private:

    /// When the current goal is reached this functions sets the next goal
    bool nextGoal();


    /// Math helper
    double norm( const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) const
    {
        double a = p1.x - p2.x; double b = p1.y - p2.y;
        return sqrt(a*a + b*b );
    }


    /// Call to invert the steering angle
    void toggle_steer()
    {
        //Helper is the distance from the current value for the servo to the middle (hence zero) position

        if( m_autonomous_control.control_servo.y < 1500 )
        {
            int helper = m_autonomous_control.control_servo.y + 1500;
            m_autonomous_control.control_servo.y += helper * 2;
        }
        else if ( m_autonomous_control.control_servo.y > 1500 )
        {
            int helper = m_autonomous_control.control_servo.y - 1500;
            m_autonomous_control.control_servo.y -= helper * 2;
        }
    }


private:

    /// Uses the posetype to represent position and orientation
    PoseType m_pose;

    /// Uses the posetype to represent linear and angular velocity
    PoseType m_speed;


    /// A goal position;
    geometry_msgs::Point m_goal;
    wiimote::State m_wii;


    ros::NodeHandle m_nodeHandle;
    ros::Subscriber m_wii_sub;

    /// Stores the old value of Players GetYaw() to get the change in rotation -> independent of robot's / stage's definition of the zero
    double m_yaw_old;


    const double m_goal_distance = 0.1;   ///< Maximum distandce of goal to fulfill it

    control m_autonomous_control;    ///< Wrapper for motor comands

#ifdef USE_PLAYER
public:

    // Accessor Methods for the player stuff

    const PlayerCc::PlayerClient *playerClient() const;
    const PlayerCc::RangerProxy *laserProxy() const;
    const PlayerCc::Position2dProxy *position2dProxy() const;



private:

    //The player stuff

    PlayerCc::PlayerClient *m_playerClient;
    PlayerCc::RangerProxy* m_laserProxy;
    PlayerCc::Position2dProxy* m_position2dProxy;

#endif



};
