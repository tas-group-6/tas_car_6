#include <iostream>

#include "control/control.h"

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <fstream>
#include <sstream>


using namespace std;



#define __USE_BSD
#include <math.h>


//#define DAYNAMIC_VEL_CONTROL
#ifdef DAYNAMIC_VEL_CONTROL
    //#define AVG_RANG_VEL_CONTROL
    #define MIN_RANG_VEL_CONTROL
    //#define CORNER_DETECTION
#endif


#ifdef DAYNAMIC_VEL_CONTROL

#define GRAD_TO_RAD         0.01745329252
#define RAD_TO_GRAD			57.2957795131

#define FRONT_RES               20   // [Grad]
#define SIDE_RES                20   // [Grad]

#define FRONT_BOUND             0.6    // [Meter]
#define SIDE_BOUND              0.2 + 0.2    // [Meter]

// *********** BREAK Parameters **************** //
#define SIDE_BREAK				0.2 + 0.3  // [Meter]
#define	FRONT_BREAK				0.2  // [Meter]

// *********** Lin Vel control Parameters ****** //
#define MAX_RANGE				2  // [Meter]
#define MIN_RANGE				0.4	 // [Meter]
#define MIN_VEL					1550 // [PWM]
#define MAX_VEL					1570 // [PWM]

#define MIN_VEL_B				1500 // [PWM]
#define MAX_VEL_B				1400 // [PWM]

// (RANG - Min_RANGE) / (MAX_RANGE-MIN_RANGE) * (MAX_VEL - MIN_VEL) + DEFAULT_VEL

// *********** Corner Detection ****** //
#define ALPHA_CORNER			30 // [Grad]

#define MAX_CORN_APP_VEL		1560 // [PWM]
#define MIN_CORN_APP_VEL		1550 // [PWM]

#define DELTA_YAW				10 // [Grad]

#define MAX_CORN_LEAVE_VEL      1565 // [PWM]
#define MIN_CORN_LEAVE_VEL      1550 // [PWM]

#define CP_POS_TOL				0.2 // [Meter]

#define MAX_CORN_EXIT_VEL		15 // [PWM]

#endif



// free space identifier [0: No free space, 5: All free space]
enum FREE_SPACE { SPACE_DEFAULT, NO_FREE_SPACE, CORRIDOR, ALL_FREE};
FREE_SPACE space = SPACE_DEFAULT; // default state

// corner identifier
enum CORNER { NO_CORNER, APP_CORNER, LEAVE_CORNER, EXIT_CORNER};
CORNER corner = NO_CORNER; // default state

int vel_gain = 0;
int vel_gain_break = 0;
int vel_corner = 0;

#ifdef CORNER_DETECTION

struct CORNER_PIONT {
    geometry_msgs::Pose goe_msg_pos_cp;
    float alpha_correct;
    float pos_leave;
    float pos_exit;
};
CORNER_PIONT corner_piont;
std::vector<CORNER_PIONT> corner_pionts;

int CP_count = 1;
int CP_FILE_READ = 0;
#endif

#ifdef DAYNAMIC_VEL_CONTROL
//Callback function, called every time when laser masage is recieved
void laserCb(const sensor_msgs::LaserScan & msg)
{
    //ROS_INFO("I heard: []");

    const unsigned int N = msg.ranges.size();
    //const double angel_res =  (msg.angle_max - msg.angle_min)/N;
    const double angel_res =  msg.angle_increment;

    const unsigned int front_spann = (int)((FRONT_RES * GRAD_TO_RAD)/angel_res);
    const unsigned int side_spann = (int)((SIDE_RES * GRAD_TO_RAD)/angel_res);
    double distance_f, distance_l, distance_r;
    double distance_f_min, distance_l_min, distance_r_min;

    int i = 0;

        // ####### Front #####//
    // Scanning Front
#ifdef AVG_RANG_VEL_CONTROL
    distance_f = 0.0;
    // Approximation: Calc the avg from all meassurments in a given agualr range
    for (i=0; i < front_spann; i++)
    {
        distance_f += msg.ranges[i + N/2 - front_spann/2];
    }
    distance_f = distance_f/(front_spann);
#endif


#ifdef MIN_RANG_VEL_CONTROL
        // Find Min
        distance_f_min = 100.0;
        for (i=0; i < front_spann; i++)
    {
            if(distance_f_min > msg.ranges[i + N/2 - front_spann/2])
        distance_f_min = msg.ranges[i + N/2 - front_spann/2];
    }
#endif

        // ##################//



        // ####### Left Side #####//
    // Scanning Side
#ifdef AVG_RANG_VEL_CONTROL
    distance_l = 0.0;
    // Approximation: Calc the avg from all meassurments in a given agualr range
    for (i=0; i < side_spann; i++)
    {
        distance_l += msg.ranges[N - i];
    }
    distance_l = distance_l/(side_spann);
#endif

#ifdef MIN_RANG_VEL_CONTROL
        // Find Min
        distance_l_min = 100.0;
        for (i=0; i < side_spann; i++)
    {
            if(distance_l_min > msg.ranges[N -i])
        distance_l_min = msg.ranges[N - i];
    }
        // ##################### //
#endif


        // ####### Right Side #####//
#ifdef AVG_RANG_VEL_CONTROL
    distance_r = 0.0;
    // Approximation: Calc the avg from all meassurments in a given agualr range
    for (i=0; i < side_spann; i++)
    {
        distance_r += msg.ranges[i];
    }
    distance_r = distance_r/(side_spann);
#endif

#ifdef MIN_RANG_VEL_CONTROL
        // Find min
        distance_r_min = 100.0;
        for (i=0; i < side_spann; i++)
    {
            if(distance_r_min > msg.ranges[i])
        distance_r_min = msg.ranges[i];
    }
#endif

#ifdef MIN_RANG_VEL_CONTROL
        distance_f = distance_f_min;
        distance_l = distance_l_min;
        distance_r = distance_r_min;
#endif


// ################## Set Global Flags ############# //

    // Set Global Flags
        if(distance_f > FRONT_BOUND/*(distance_l >= SIDE_BOUND) && (distance_r >= SIDE_BOUND)*/)
    {
        // Object is deteced on the front of the car
        space = CORRIDOR;
                vel_gain = ((distance_f - MIN_RANGE) / (MAX_RANGE-MIN_RANGE)) * (MAX_VEL - MIN_VEL);
        }

        // ######## Corner Detection ####### //


    /*if((distance_l >= SIDE_BOUND) && (distance_r >= SIDE_BOUND))
    {
        // if a object is detected on the sides of the car
        space = CORRIDOR;
                vel_gain = ((distance_l - MIN_RANGE) / (MAX_RANGE-MIN_RANGE)) * (MAX_VEL - MIN_VEL);
        if ((((distance_r - MIN_RANGE) / (MAX_RANGE-MIN_RANGE)) * (MAX_VEL - MIN_VEL)) <
            vel_gain)
        {
            // choos the smaller velocity
            vel_gain =(((distance_r - MIN_RANGE) / (MAX_RANGE-MIN_RANGE)) * (MAX_VEL - MIN_VEL));
        }
        if (distance_f < FRONT_BOUND)

            // +  a objected is deteced in front of the car
            space = CORNER;
    }*/

    // Emergancy Break
    if (distance_f < FRONT_BREAK/*distance_l < SIDE_BREAK || distance_r < SIDE_BREAK ||*/ )
    {
       space = NO_FREE_SPACE;
             vel_gain_break = ((distance_f - FRONT_BREAK) / (FRONT_BREAK)) * (MIN_VEL_B - MAX_VEL_B);

    }

    // No object deteced
    if ((distance_l > (SIDE_BOUND * 10) || distance_r > (SIDE_BOUND * 10)) && (distance_f > (SIDE_BOUND * 10)) )
    {
        space = ALL_FREE;
    }


}

// ######################## Corner Detection ################## //

#ifdef CORNER_DETECTION
//Callback function, called every time when amcl position is published
void positionCb(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
    if(CP_FILE_READ)
    {
        CORNER_PIONT cp_read;
        float delta_y, delta_x, delta_yaw, alpha;
        geometry_msgs::Pose pos_amcl;
        float x_leave, y_leave;
        float x_exit, y_exit;
        float amcl_yaw, cp_yaw;


        // read amcl pos + orientation + Cov + Stamp
        pos_amcl.position = msg.pose.pose.position;
        pos_amcl.orientation = msg.pose.pose.orientation;

        // read CP values;
        cp_read = corner_pionts.at(CP_count);

        // calc alpha(pos_amcl, pos_cp)
        delta_x = abs(pos_amcl.position.x - cp_read.goe_msg_pos_cp.position.x);
        delta_y = abs(pos_amcl.position.y - cp_read.goe_msg_pos_cp.position.y);
        alpha = abs((CP_count%2) * 90 - atan(delta_y/delta_x) * RAD_TO_GRAD);
        alpha = alpha + cp_read.alpha_correct;

        // ###### Set global Corner Flag ####### //
        switch(corner)
        {
                case NO_CORNER:
                        if(alpha>ALPHA_CORNER)
                        {
                            corner = APP_CORNER;
                        }
                break;

                case APP_CORNER:
                    vel_corner = (alpha - ALPHA_CORNER)/alpha * (MIN_CORN_APP_VEL - MAX_CORN_APP_VEL) + 10;

                    if(CP_count == 1)
                    {
                        int y_leave;
                        y_leave = cp_read.pos_leave;
                        if(pos_amcl.position.y >= y_leave)
                        corner = LEAVE_CORNER;
                    }
                    if(CP_count == 2)
                    {
                        x_leave = cp_read.pos_leave;
                        if(pos_amcl.position.x <= x_leave)
                        corner = LEAVE_CORNER;
                    }
                    if(CP_count == 3)
                    {
                        y_leave = cp_read.pos_leave;
                        if(pos_amcl.position.y <= y_leave)
                        corner = LEAVE_CORNER;
                    }
                    if(CP_count == 4)
                    {
                        x_leave = cp_read.pos_leave;
                        if(pos_amcl.position.x >= x_leave)
                        corner = LEAVE_CORNER;
                    }

                break;

                case LEAVE_CORNER:
                    // calc delat abs(orientation(amcl orientation - cp orientation))
                    amcl_yaw = tf::getYaw( pos_amcl.orientation);
                    cp_yaw = tf::getYaw(cp_read.goe_msg_pos_cp.orientation);
                    delta_yaw = abs(amcl_yaw-cp_yaw);
                    vel_corner = (90 - delta_yaw)/90 * (MAX_CORN_LEAVE_VEL - MIN_CORN_LEAVE_VEL);
                    if(delta_yaw < DELTA_YAW)
                        corner = EXIT_CORNER;
                break;

                case EXIT_CORNER:
                    vel_corner = MAX_CORN_EXIT_VEL;

                    if(CP_count == 1)
                    {
                        x_exit = cp_read.pos_exit;
                        if(pos_amcl.position.x <= x_exit + CP_POS_TOL)
                        corner = NO_CORNER;
                        CP_count = 2;
                        break;
                    }
                    if(CP_count == 2)
                    {
                        y_exit = cp_read.pos_exit;
                        if(pos_amcl.position.y <= y_exit + CP_POS_TOL)
                        corner = NO_CORNER;
                        CP_count = 3;
                        break;
                    }
                    if(CP_count == 3)
                    {
                        x_exit = cp_read.pos_exit;
                        if(pos_amcl.position.x >= x_exit - CP_POS_TOL)
                        corner = NO_CORNER;
                        CP_count = 4;
                        break;
                    }
                    if(CP_count == 4)
                    {
                        y_exit = cp_read.pos_exit;
                        if(pos_amcl.position.y >= y_exit - CP_POS_TOL)
                        corner = NO_CORNER;
                        CP_count = 1;
                        break;
                    }
                break;


             default:
                    corner = NO_CORNER;
              break;

   }
 }

}
#endif

#endif


int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_control");
    control autonomous_control;
    #ifdef DAYNAMIC_VEL_CONTROL
        ros::NodeHandle n;
        ros::Subscriber subScan = n.subscribe("/scan", 10, laserCb);
    #ifdef CORNER_DETECTION
        ros::Subscriber subAmcl = n.subscribe("/amcl_pose", 10, positionCb);
        ifstream ifs ("/home/tas_group_06/Documents/Cornerpionts.txt");
    #endif
    #endif



    vel_gain = 0;


#ifdef CORNER_DETECTION
    // reade corner pionts from text file to global var.

  if(!ifs.is_open()) return -1;

    while (!ifs.eof())
    {
        ifs>> corner_piont.goe_msg_pos_cp.position.x>>
              corner_piont.goe_msg_pos_cp.position.y>>
              corner_piont.goe_msg_pos_cp.position.z;

        ifs>>	corner_piont.goe_msg_pos_cp.orientation.x>>
                            corner_piont.goe_msg_pos_cp.orientation.y>>
                            corner_piont.goe_msg_pos_cp.orientation.z>>
                            corner_piont.goe_msg_pos_cp.orientation.w;

                ifs>>corner_piont.alpha_correct;

                ifs>>corner_piont.pos_leave;

                ifs>>corner_piont.pos_exit;

        corner_pionts.push_back(corner_piont);
    }
    // SET GLOBAL FLAG CP_PIONT_READ
    CP_count = 1;
    CP_FILE_READ = 1;


#endif




    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        if(autonomous_control.control_Mode.data==0)
        {
            ROS_INFO("Manually Control!");
        }
        else
        {
            if(autonomous_control.control_Brake.data==1)
            {
                autonomous_control.control_servo.x=1500;
                autonomous_control.control_servo.y=1500;
            }
            else
            {
                ROS_INFO("Automatic Control!");
                std::cout << "Lin Vel CMD" << autonomous_control.cmd_linearVelocity << std::endl;
                std::cout << "Lin Odom Vel " << autonomous_control.odom_linearVelocity  << std::endl;

                switch(space)
                {
                    case SPACE_DEFAULT:
                        // Default velocity control
                        if(autonomous_control.cmd_linearVelocity>0)
                        {
                            autonomous_control.control_servo.x = 1560;
                            /*pwm = (log(autonomous_control.cmd_linearVelocity +1)) * 35 + 1520;
                            //if (pwm<1570)
                            //  autonomous_control.control_servo.x = (int) pwm;
                            //elsevel_gain_break
                            //  autonomous_control.control_servo.x = 1570;
                            */
                        }
                        else if(autonomous_control.cmd_linearVelocity<0)
                        {
                            autonomous_control.control_servo.x = 1400;
                        }
                        else
                        {
                         autonomous_control.control_servo.x = 1500;
                        }

                        break;

                    case NO_FREE_SPACE:
                    // Stopp Car
                                                // Eventualy non linear function
                        autonomous_control.control_servo.x = 1500 + vel_gain_break;
                        break;

                    /*case CORNER:
                        // Default velocity control
                        if(autonomous_control.cmd_linearVelocity>0)
                        {
                            autonomous_control.control_servo.x = 1560;
                        }
                        else if(autonomous_control.cmd_linearVelocity<0)
                        {
                            autonomous_control.control_servo.x = 1400;
                        }
                        else
                        {
                        autonomous_control.control_servo.x = 1500;
                        }

                        break;*/

                    case CORRIDOR:
                        // Linear Velocity control
                        if(autonomous_control.cmd_linearVelocity>0)
                        //if(1)
                        {
                                                    if(corner == APP_CORNER || corner == LEAVE_CORNER || corner == EXIT_CORNER)
                                                    {
                                                        // AMCL Position Vel control
                                                        // Eventualy non linear function
                                                        autonomous_control.control_servo.x = 1550 + vel_corner;
                                                    }
                                                    else
                                                    {
                                                        // LASER_RANG Vel control
                                                        // Eventualy non linear function
                                                        autonomous_control.control_servo.x = 1550 + vel_gain;
                                                    }
                        }
                        else if(autonomous_control.cmd_linearVelocity<0)
                        {
                            autonomous_control.control_servo.x = 1400;
                        }
                        else
                        {
                        autonomous_control.control_servo.x = 1500;
                        }

                        break;


                    case ALL_FREE:
                        // Default velocity control
                        if(autonomous_control.cmd_linearVelocity>0)
                        {
                          if(corner == APP_CORNER || corner == LEAVE_CORNER || corner == EXIT_CORNER)
                                                    {
                                                        autonomous_control.control_servo.x = 1550 + vel_corner;
                                                    }
                                                    else
                                                    {
                                                        autonomous_control.control_servo.x = 1570;
                                                    }
                        }
                        else if(autonomous_control.cmd_linearVelocity<0)
                        {
                            autonomous_control.control_servo.x = 1400;
                        }
                        else
                        {
                                                        autonomous_control.control_servo.x = 1500;
                        }

                        break;

                    default:

                        space = SPACE_DEFAULT;
                        break;

                }

                autonomous_control.control_servo.y = autonomous_control.cmd_steeringAngle;
            }

            autonomous_control.control_servo_pub_.publish(autonomous_control.control_servo);

        }

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;

}
