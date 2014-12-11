#pragma once

#include <sensor_msgs/LaserScan.h>

#include <libplayerc++/playerc++.h>

#include <string.h>
#include <sstream>
#include <iostream>


#include "DataType.h"


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

    const PlayerCc::PlayerClient *playerClient() const;
    const PlayerCc::RangerProxy *laserProxy() const;
    const PlayerCc::Position2dProxy *position2dProxy() const;


private:
    PlayerCc::PlayerClient *m_playerClient;
    PlayerCc::RangerProxy* m_laserProxy;
    PlayerCc::Position2dProxy* m_position2dProxy;

    PoseType m_pose;            ///< Uses the posetype to represent position and orientation
    PoseType m_speed;        ///< Uses the posetype to represent linear and angular velocity

    double yaw_old;             ///< Stores the old value of Players GetYaw() to get the change in rotation -> independent of robot's / stage's definition of the zero



};
