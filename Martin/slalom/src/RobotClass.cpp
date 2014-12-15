#include "RobotClass.h"

RobotClass::RobotClass()
{
    m_pose.position.x = 0.0;
    m_pose.position.y = 0.0;
    m_pose.rotation = 0.0;

    m_speed.position.x = 0.0;
    m_speed.position.y = 0.0;
    m_speed.rotation = 0.0;


    m_goal.x = 1.25;
    m_goal.y = 0.55;

    //Linear Velocity
    m_autonomous_control.control_servo.x = 1550;

    //Angular Velocity
    double a = RAD_TO_DEG * atan( m_goal.x / m_goal.y * CAR_LENGTH );
    m_autonomous_control.control_servo.y = DEG_TO_SERVO_SIGNAL( a );

//m_wii_sub = m_nodeHandle.subscribe< wiimote::State >("wii_communication",1000,&control::wiiCommunicationCallback,this);


#ifdef USE_PLAYER

    m_playerClient =    new PlayerCc::PlayerClient("localhost");
    m_laserProxy =      new PlayerCc::RangerProxy(m_playerClient, 1);
    m_position2dProxy = new PlayerCc::Position2dProxy(m_playerClient, 0);

    m_playerClient->Read();

    m_yaw_old = m_position2dProxy->GetYaw(); //Store first value of rotation

#else

    m_yaw_old = 0.0;

#endif

}

RobotClass::~RobotClass()
{
#ifdef USE_PLAYER
    SAFE_DELETE(m_laserProxy);
    SAFE_DELETE(m_position2dProxy);
    SAFE_DELETE(m_playerClient);
#endif

}



void RobotClass::update()
{
#ifdef USE_PLAYER

    m_playerClient->Read();

#endif


    std::printf("Odom: lin %.3f\t ang %.3f\n", m_autonomous_control.cmd_linearVelocity, m_autonomous_control.odom_angularVelocity);

    // Integrate Pose with explizit euler step
    m_speed.rotation = m_autonomous_control.odom_angularVelocity;
    m_pose.rotation += m_speed.rotation * update_rate;


    //Create current viewdirection from rotation
    double view_x = cos( m_pose.rotation );
    double view_y = sin( m_pose.rotation );

    m_speed.position.x = view_x * m_autonomous_control.odom_linearVelocity;
    m_speed.position.y = view_y * m_autonomous_control.odom_linearVelocity;

    m_pose.position.x +=  m_speed.position.x * update_rate;
    m_pose.position.y +=  m_speed.position.y * update_rate;



    //Get next goal when required, and ajust motor signals
    bool result = nextGoal();



    std::printf("Pos: %.3f %.3f %.3f\n", m_pose.position.x, m_pose.position.y, m_pose.rotation);
    std::printf("Vel: %.3f %.3f %.3f\n", m_speed.position.x, m_speed.position.y, m_speed.rotation);

    double test = norm( m_goal, m_pose.position);
    std::printf("Goal: %.f %.f \t Distance: %.3f", m_goal.x, m_goal.y, test);


    std::printf("Servo: %.3f\t%.3f", m_autonomous_control.control_servo.x, m_autonomous_control.control_servo.y);


    m_autonomous_control.control_servo_pub_.publish( m_autonomous_control.control_servo );

}


bool RobotClass::nextGoal()
{
    static bool first_goal_reached = false;

    if (norm( m_goal, m_pose.position) < m_goal_distance)
    {
        m_goal.x += 1.5; //Move goal to next cone
        m_goal.y *= -1.0; //Put it on the other side

        if (!first_goal_reached)
        {
            m_autonomous_control.control_servo.y = 1750;
            first_goal_reached = true;
        }

        toggle_steer();


        return true;
    }
    return false;
}

bool RobotClass::FillScanMessage(sensor_msgs::LaserScan& scan) const
{
#ifdef USE_PLAYER

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

#else
    return false;

#endif
}

PoseType RobotClass::Pose() const { return m_pose;}
void RobotClass::Pose(const PoseType& p){ m_pose = p;}

PoseType RobotClass::Speed() const { return m_speed;}
void RobotClass::Speed(const PoseType& p){ m_speed = p;}

double RobotClass::getRotation() const {  return m_pose.rotation; }
void RobotClass::setRotation(double value) {  m_pose.rotation = value; }

double RobotClass::getRotationYaw()
{
#ifdef USE_PLAYER
    m_pose.rotation += ( m_yaw_old -  m_position2dProxy->GetYaw());
    m_yaw_old = m_position2dProxy->GetYaw();

    if( m_pose.rotation > 2.0 * PI ) m_pose.rotation -= 2.0*PI;
    if( m_pose.rotation < 0.0 * PI ) m_pose.rotation += 2.0*PI;
#endif

    return m_pose.rotation;
}
void RobotClass::setRotationYaw(double value) {  m_pose.rotation = value; }

#ifdef USE_PLAYER

const PlayerCc::PlayerClient *RobotClass::playerClient() const {  return m_playerClient; }
const PlayerCc::RangerProxy *RobotClass::laserProxy() const { return m_laserProxy; }
const PlayerCc::Position2dProxy *RobotClass::position2dProxy() const { return m_position2dProxy; }

#endif
