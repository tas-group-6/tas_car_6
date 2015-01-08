#include "RobotClass.h"

RobotClass::RobotClass() : m_index_initial(0), m_index_left(0), m_index_right(0), m_turn_counter(0), m_state(STATE_INIT)
{    
    //Linear Velocity
    //m_autonomous_control.control_servo.x = 1550;

    //Angular Velocity
    //double a = RAD_TO_DEG * atan( m_goal.x / m_goal.y * CAR_LENGTH );
    //m_autonomous_control.control_servo.y = DEG_TO_SERVO_SIGNAL( a );


    std::string filename1;
    std::string filename2;
    std::string filename3;

    std::string path = "/home/tas_group_06/catkin_ws/src/slalom/log_files/" ;
    //std::string path = "/home/martin/TAS_ws/tas_car_6/trunk/Martin/slalom/log_files/";
    //std::string path = "/home/martin/Programming/TAS/tas_car_6/trunk/Martin/slalom/log_files/";

    std::stringstream ss1; ss1 << path << "init_turn_left.txt";
    filename1 = ss1.str();

    std::stringstream ss2; ss2 << path << "turn_left.txt";
    filename2 = ss2.str();

    std::stringstream ss3; ss3 << path << "turn_right.txt";
    filename3 = ss3.str();

    bool result;

    result = loadInstructionVector(m_initial_turn, filename1);
    if(!result) throw ErrorType("Could not load initial turn");


    result =  loadInstructionVector(m_turn_left, filename2);
    if(!result) throw ErrorType("Could not load left turn");


    result =  loadInstructionVector(m_turn_right, filename3);
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

bool RobotClass::update( ServoInstructionType& instruction )
{
#ifdef USE_PLAYER

    m_playerClient->Read();

#endif

    static int counter = 0;
    //std::printf("-----------------------------------\n");
    std::cout << "Update Call: " << counter++ << std::endl;


    std::chrono::microseconds d;

    switch( m_state )
    {
    case STATE_INIT:
        std::cout << "STATE_INIT: " << m_index_initial<<std::endl;
        instruction.steer =  m_initial_turn[m_index_initial].steer;
        instruction.velocity = m_initial_turn[m_index_initial].velocity;

        d = std::chrono::microseconds(  m_initial_turn[m_index_initial+1].t - m_initial_turn[m_index_initial].t );
        std::this_thread::sleep_for( d );

        m_index_initial++;

        if( m_index_initial >= m_initial_turn.size() - 1)
        {
            m_state = STATE_RIGHT;
            m_index_initial = 0;
        }

        break;

    case STATE_LEFT:
        std::cout <<"STATE_LEFT: " << m_index_left << std::endl;
        instruction.steer =  m_turn_left[m_index_left].steer;
        instruction.velocity = m_turn_left[m_index_left].velocity;

        d = std::chrono::microseconds(  m_turn_left[m_index_left+1].t - m_turn_left[m_index_left].t );
        std::this_thread::sleep_for( d );

        m_index_left++;

        if( m_index_left >= m_turn_left.size() - 1)
        {
            m_turn_counter++;
            m_state = m_turn_counter < m_turn_counter_max ? STATE_RIGHT  : STATE_FINAL;
            m_index_left = 0;
        }

        break;

    case STATE_RIGHT:
        std::cout <<"STATE_RIGHT: " << m_index_right << std::endl;

        instruction.steer =  m_turn_right[m_index_right].steer;
        instruction.velocity = m_turn_right[m_index_right].velocity;

        d = std::chrono::microseconds(  m_turn_right[m_index_right+1].t - m_turn_right[m_index_right].t );
        std::this_thread::sleep_for( d );

        m_index_right++;

        if( m_index_right >= m_turn_right.size() - 1)
        {
            m_turn_counter++;
            m_state = m_turn_counter < m_turn_counter_max ? STATE_LEFT : STATE_FINAL;
            m_index_right = 0;
        }

        break;

    case STATE_FINAL:
        std::cout <<"STATE_FINAL"<< std::endl;
        return false;
        break;
    default:
        break;
    }


    return true;


    //  for (int i = 0; i < m_initial_turn.size() -1; ++i)
    //  {
    //    m_autonomous_control.control_servo.x = m_initial_turn[i].velocity;
    //    m_autonomous_control.control_servo.y = m_initial_turn[i].steer;
    //    std::chrono::microseconds d(  m_initial_turn[i+1].t - m_initial_turn[i].t );
    //    std::printf("Servo: %.3f\t%.3f\n", m_autonomous_control.control_servo.x, m_autonomous_control.control_servo.y);
    //    m_autonomous_control.control_servo_pub_.publish( m_autonomous_control.control_servo );
    //    std::this_thread::sleep_for( d );
    //  }
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
