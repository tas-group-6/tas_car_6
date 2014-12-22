#include "RobotClass.h"

RobotClass::RobotClass()
{    
    //Linear Velocity
    //m_autonomous_control.control_servo.x = 1550;

    //Angular Velocity
    //double a = RAD_TO_DEG * atan( m_goal.x / m_goal.y * CAR_LENGTH );
    //m_autonomous_control.control_servo.y = DEG_TO_SERVO_SIGNAL( a );


    bool result;

    result = loadInstructionVector(m_initial_turn, "/home/martin/TAS_ws/tas_car_6/trunk/Martin/slalom/wiimote_log_file.txt");
    if(!result) throw ErrorType("Could not load initial turn");


    result =  loadInstructionVector(m_initial_turn, "/home/martin/TAS_ws/tas_car_6/trunk/Martin/slalom/wiimote_log_file.txt");
    if(!result) throw ErrorType("Could not load left turn");


    result =  loadInstructionVector(m_initial_turn, "/home/martin/TAS_ws/tas_car_6/trunk/Martin/slalom/wiimote_log_file.txt");
    if(!result) throw ErrorType("Could not load right turn");




#ifdef USE_PLAYER

    m_playerClient =    new PlayerCc::PlayerClient("localhost");
    m_laserProxy =      new PlayerCc::RangerProxy(m_playerClient, 1);
    m_position2dProxy = new PlayerCc::Position2dProxy(m_playerClient, 0);

    m_playerClient->Read();

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


bool RobotClass::loadInstructionVector(std::vector<ServoInstructionType>& inst_vector, std::string filename)
{

    std::ifstream ifs;
    ifs.open(filename);
    if(!ifs.is_open()) return false;

    ServoInstructionType instruction;

    while (!ifs.eof())
    {
        ifs >> instruction.t >> instruction.velocity >> instruction.steer;
        inst_vector.push_back( instruction );
    }

    return true;
}






void RobotClass::update()
{
#ifdef USE_PLAYER

    m_playerClient->Read();

#endif

    static int counter = 0;
    std::printf("-----------------------------------\n");
    std::printf("Update Call: %d", counter++);

    for (int i = 0; i < m_initial_turn.size() -1; ++i)
    {
        m_autonomous_control.control_servo.x = m_initial_turn[i].velocity;
        m_autonomous_control.control_servo.y = m_initial_turn[i].steer;

        std::chrono::microseconds d(  m_initial_turn[i+1].t - m_initial_turn[i].t );

        std::printf("Servo: %.3f\t%.3f\n", m_autonomous_control.control_servo.x, m_autonomous_control.control_servo.y);
        m_autonomous_control.control_servo_pub_.publish( m_autonomous_control.control_servo );

        std::this_thread::sleep_for( d );
    }
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

#ifdef USE_PLAYER

const PlayerCc::PlayerClient *RobotClass::playerClient() const {  return m_playerClient; }
const PlayerCc::RangerProxy *RobotClass::laserProxy() const { return m_laserProxy; }
const PlayerCc::Position2dProxy *RobotClass::position2dProxy() const { return m_position2dProxy; }

#endif
