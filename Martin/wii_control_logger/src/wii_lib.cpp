#include "wii_lib.h"

wii_lib::wii_lib()
{
    /*Initilaization of publishers, subscribers and messages*/
    // wii_servo_pub_ = nh_.advertise<geometry_msgs::Vector3>("servo", 1);

    //    wii_communication_pub = nh_.advertise<std_msgs::Int16MultiArray>("wii_communication",1);

    wii_sub_ = nh_.subscribe<wiimote::State>("wiimote/state",100,&wii_lib::wiiStateCallback,this);

    msg_Initialization(wii_state_);


    controlMode.data = 0;
    emergencyBrake.data = 1;

    ////////////
    // Logger //
    ////////////

    m_logging_enabled = false;

    m_ofs.open("wiimote_log_file.txt", std::ios::out);
    if(!m_ofs.is_open()) throw std::string("Output file stream not open");


    m_wii_state_old.resize(11);
    m_time_start = std::chrono::steady_clock::now();
}

wii_lib::~wii_lib()
{
    m_ofs.close();
}

void wii_lib::wiiStateCallback(const wiimote::State::ConstPtr& wiiState)
{   
    //Button A pressed and changed
    if( wiiState.get()->buttons[ WII_BUTTON_A] &&
            wiiState.get()->buttons[WII_BUTTON_A] != m_wii_state_old[WII_BUTTON_A])
    {
        std::cout << "- Button A -" << std::endl;
        m_logging_enabled = !m_logging_enabled;
    }


    /*check if C button is pressed*/
    if(wiiState.get()->nunchuk_buttons[WII_BUTTON_NUNCHUK_C]==1)
    {
        controlMode.data = 1; /*setting controlMode flag to 1*/

        if(wiiState.get()->nunchuk_buttons[WII_BUTTON_NUNCHUK_Z]==1)
        {
            emergencyBrake.data = 1; /*setting emergencyBrake flag to 1*/
        }
        else
        {
            emergencyBrake.data = 0; /*setting emergencyBrake flag to 0*/
        }
    }
    else
    {
        controlMode.data = 0;/*setting controlMode flag to 0*/

        /*check if Z button is pressed*/
        if(wiiState.get()->nunchuk_buttons[WII_BUTTON_NUNCHUK_Z]==1)
        {
            emergencyBrake.data = 1; /*setting emergencyBrake flag to 1*/

            servo.x = 1500;
            servo.y = 1500;
        }
        else
        {
            emergencyBrake.data = 0; /*setting emergencyBrake flag to 0*/

            if(wiiState.get()->nunchuk_joystick_zeroed[1]>=0)
            {
                SCALE_FAKTOR_THROTTLE = 200; /*scale factor for driving forward*/
            }
            else
            {
                SCALE_FAKTOR_THROTTLE = 300; /*scale factor for driving reverse*/
            }

            /* mapping analog nunchuk state to servo command*/
            servo.x = 1500 + SCALE_FAKTOR_THROTTLE*wiiState.get()->nunchuk_joystick_zeroed[1];
            servo.y = 1500 + SCALE_FAKTOR_STEERING*wiiState.get()->nunchuk_joystick_zeroed[0];

            if(m_logging_enabled)
            {

                std::chrono::steady_clock::time_point time_cur = std::chrono::steady_clock::now();

                double t =  std::chrono::duration_cast<std::chrono::milliseconds>(time_cur - m_time_start).count();

                m_ofs << t << "\t" << servo.x << "\t" << servo.y << std::endl;
                std::cout << t << "\t" << servo.x << "\t" << servo.y << std::endl;
            }
        }

        //wii_servo_pub_.publish(servo); /*publish servo messages to arduino*/
    }

    wii_state_.data[0] = controlMode.data;
    wii_state_.data[1] = emergencyBrake.data;


    m_wii_state_old[WII_BUTTON_A] = wiiState.get()->buttons[WII_BUTTON_A]; //Save current state
    //std::cout << m_wii_state_old[WII_BUTTON_A] << std::endl;
}

void wii_lib::msg_Initialization(std_msgs::Int16MultiArray &msg)
{
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = 2;
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "flag";
    msg.data.clear();
    msg.data.resize(2);
}

