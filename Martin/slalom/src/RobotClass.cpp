#include "RobotClass.h"

RobotClass::RobotClass()
{
    m_playerClient =    new PlayerCc::PlayerClient("localhost");
    m_laserProxy =      new PlayerCc::RangerProxy(m_playerClient, 1);
    m_position2dProxy = new PlayerCc::Position2dProxy(m_playerClient, 0);

    m_pose.position.x = 0.0;
    m_pose.position.y = 0.0;
    m_pose.rotation = 0.0;

    m_speed.position.x = 0.0;
    m_speed.position.y = 0.0;
    m_speed.rotation = 0.0;

    m_playerClient->Read();

    yaw_old = m_position2dProxy->GetYaw(); //Store first value of rotation
}

RobotClass::~RobotClass()
{
    SAFE_DELETE( m_laserProxy);
    SAFE_DELETE(m_position2dProxy);
    SAFE_DELETE(m_playerClient);
}



void RobotClass::update()
{
    m_playerClient->Read();
}


bool RobotClass::FillScanMessage(sensor_msgs::LaserScan& scan) const
{
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "laser_frame";

    scan.angle_increment = m_laserProxy->GetAngularRes();
    scan.angle_max = m_laserProxy->GetMaxAngle();
    scan.angle_min = m_laserProxy->GetMinAngle();
    scan.range_max = m_laserProxy->GetMaxRange();
    scan.range_min = m_laserProxy->GetMinRange();
    scan.time_increment = (1 / m_laserProxy->GetFrequency()) / (m_laserProxy->GetRangeCount());

    scan.ranges.resize( m_laserProxy->GetRangeCount());
    scan.intensities.resize( m_laserProxy->GetIntensityCount());

    if (scan.ranges.size() != scan.intensities.size())
        throw ErrorType( "Got a different ammount of ranges than intensities");

    for (unsigned int i = 0; i < scan.ranges.size(); ++i)
    {
        scan.ranges[i] = m_laserProxy->GetRange(i);
        scan.intensities[i] = m_laserProxy->GetIntensity(i);
    }

    return true;
}



PoseType RobotClass::Pose() const { return m_pose;}
void RobotClass::Pose(const PoseType& p){ m_pose = p;}

PoseType RobotClass::Speed() const { return m_speed;}
void RobotClass::Speed(const PoseType& p){ m_speed = p;}

double RobotClass::getRotation() const {  return m_pose.rotation; }
void RobotClass::setRotation(double value) {  m_pose.rotation = value; }

double RobotClass::getRotationYaw()
{
    m_pose.rotation += ( yaw_old -  m_position2dProxy->GetYaw());
    yaw_old = m_position2dProxy->GetYaw();

    if( m_pose.rotation > 2.0 * PI ) m_pose.rotation -= 2.0*PI;
    if( m_pose.rotation < 0.0 * PI ) m_pose.rotation += 2.0*PI;

    return m_pose.rotation;
}
void RobotClass::setRotationYaw(double value) {  m_pose.rotation = value; }

const PlayerCc::PlayerClient *RobotClass::playerClient() const {  return m_playerClient; }
const PlayerCc::RangerProxy *RobotClass::laserProxy() const { return m_laserProxy; }
const PlayerCc::Position2dProxy *RobotClass::position2dProxy() const { return m_position2dProxy; }
