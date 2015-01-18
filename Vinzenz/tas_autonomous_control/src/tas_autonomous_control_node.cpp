/*---------------------------------------------------------------------------
*   Project: Technik Autonomer Systeme 
*   TUM 
*----------------------------------------------------------------------------
*   Filename: 		tas_autonomous_control.cpp
*   Created on: 	17.01.2015
* 	Modified on: 	----------
*   Author: 		Vinzenz Dallabetta 
*	Modified by: 	----------
*----------------------------------------------------------------------------*/	
/*-----------------------------------------------------------------------------
*
*	Description: 
	
	Includes functions: 
		- for gathering the laserscan datas and calculating the free space 
		  in front of the car
		- for gathering the amcl position datas
		- corner detection based on the amcl position 
		- calculating a velocity gain with respect of the free space
		- calculating a velocity gain while drive around a corner
		
*-----------------------------------------------------------------------------*/


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

#define GRAD_TO_RAD         0.01745329252
#define RAD_TO_GRAD			57.2957795131


// ********************* select velocity control ********************************* //
// By activating 'DYNAMIC_VEL_CONTROL' the velocity of the car is controlled 
// either, directly from the Laserscanner or by the		
// position of the car.
//
// AVG_RANG_VEL_CONTROL:	Taking the mean of the laserscan data  											
// MIN_RANG_VEL_CONTROL:	Taking the minimum of the laserscan data 
// 
// CORNER_DETECTION:		activates the position based corner	detetction 
//							of the car	  
// CORNER_DET_ANGLE:		corner detetction is done by the relative position
// CORNER_DET_POSE:			corner detetction is done by the absolutely position
// ********************************************************************************* //

#define DYNAMIC_VEL_CONTROL
#ifdef DYNAMIC_VEL_CONTROL
    //#define AVG_RANG_VEL_CONTROL
    #define MIN_RANG_VEL_CONTROL
    #define CORNER_DETECTION
    //#define CORNER_DET_ANGLE
    #define CORNER_DET_POSE
#endif




// ********************* adjust dynamic velocity control ************************* //
// Several parameters to adjust the performance of the dynamic velocity control
// ******************************************************************************* //

#ifdef DYNAMIC_VEL_CONTROL

// +++++++++++++++++ front scan parameters +++++++++++++++++ //
#define FRONT_RES               20		// [Degree]
#define FRONT_BOUND             0.3		// [Meter]


// +++++++++++++++++ BREAK parameters ++++++++++++++++++++++ //
#define	FRONT_BREAK				0.2		// [Meter]

#define MIN_VEL_B				1500	// [PWM]
#define MAX_VEL_B				1200	// [PWM]

// vel = (RANG - FRONT_BREAK) / (FRONT_BREAK) * (MIN_VEL_B - MAX_VEL_B) + DEFAULT_VEL


// +++++++++ linear velocity control parameters ++++++++++++ //
#define MAX_RANGE				2		// [Meter]
#define MIN_RANGE				0.4		// [Meter]
#define MIN_VEL					1550	// [PWM]
#define MAX_VEL					1565	// [PWM]

// vel = (RANG - Min_RANGE) / (MAX_RANGE-MIN_RANGE) * (MAX_VEL - MIN_VEL) + DEFAULT_VEL


// ++++++++++++++++ corner detection +++++++++++++++++++++++ //
#define ALPHA_CORNER			35		// [Degree]
#define CORNER_BREAK			1.5		// Meter

#define MAX_CORN_APP_VEL		1560	// [PWM]
#define MIN_CORN_APP_VEL		1550	// [PWM]

// vel = (alpha - ALPHA_CORNER)/alpha * (MIN_CORN_APP_VEL - MAX_CORN_APP_VEL) + 10;

#define DELTA_YAW				   7	// [Degree]

#define MAX_CORN_LEAVE_VEL      1565	// [PWM]
#define MIN_CORN_LEAVE_VEL      1550	// [PWM]

// vel_corner = (90 - yaw)/90 * (MAX_CORN_LEAVE_VEL - MIN_CORN_LEAVE_VEL);

#define CP_POS_TOL				0.1		// [Meter]

#define MAX_CORN_EXIT_VEL		15		// [PWM]

 // vel_corner = MAX_CORN_EXIT_VEL;

// ++++++++++++++++ start position +++++++++++++++++++++++ //
#define CORNER_START_2          3		// [Int Corner]
#define CORNER_START_1			2		// [Int Corner]
#define START_POS				2		// [Int Pose]

#endif




// ################################################################################ //
//									variables										//
// ################################################################################ //

// free space identifier 
enum FREE_SPACE { SPACE_DEFAULT, NO_FREE_SPACE, CORRIDOR, ALL_FREE};
FREE_SPACE space = SPACE_DEFAULT; // default state
FREE_SPACE next_space = SPACE_DEFAULT;

// corner identifier
enum CORNER { NO_CORNER, APP_CORNER, LEAVE_CORNER, EXIT_CORNER};
CORNER corner = NO_CORNER; // default state
CONNER next_corner_state = NO_CORNER;


struct VELOCITY {
	int vel_gain = 0;
	int vel_gain_break = 0;
	int vel_corner = 0;
};
VELOCITY velocity;

struct LASERSCAN {
	const unsigned int N;
	const double angel_res;
	const unsigned int front_spann;
	double distance_f;
	double distance_f_min;
};
LASERSCAN laserscan;

#ifdef CORNER_DETECTION

struct CORNER_PIONT {
    geometry_msgs::Pose goe_msg_pos_cp;
    float alpha_correct;
    float pos_leave;
    float pos_exit;
};
CORNER_PIONT corner_piont;

std::vector<CORNER_PIONT> corner_pionts;

CORNER_PIONT cp_read;

struct AMCL_CB {
	float delta_y;
	float delta_x;
	float delta_yaw;
	float alpha;
    geometry_msgs::Pose pos_amcl;
    float x_leave;
	float y_leave;
    float x_exit;
	float y_exit;
    float amcl_yaw:
	float cp_yaw;
};
AMCL_CB amcl_cb;

int CP_count = 1;
int next_CP_count = 1;
int CP_FILE_READ = 0;
#endif






#ifdef DYNAMIC_VEL_CONTROL
/*----------------------------------------------------------------------------------------------------
 * functionname:	laserCb(const sensor_msgs::LaserScan)

 * description:		Callback function, called every time when new message laser is recieved. Calcualte
					the range to the next objects and sets according to that a global flag for the 
					velocity control. Furthermore a velocity gain factor is calculated based on the 
					range and the global flag

 * return par:		void
 *--------------------------------------------------------------------------------------------------*/
void laserCb(const sensor_msgs::LaserScan & msg)
{
	int i = 0;

    laserscan.N = msg.ranges.size();
    laserscan.angel_res =  msg.angle_increment;
	laserscan.front_spann = (int)((FRONT_RES * GRAD_TO_RAD)/laserscan.angel_res);
   

    // ++++++++++++++++++++++++++++ scanning front ++++++++++++++++++++++++++++++++++++++++++ //
	#ifdef AVG_RANG_VEL_CONTROL
	laserscan.distance_f = 0.0;
    // Approximation: calculate the mean from all meassurments in a given angular range
    for (i=0; i < laserscan.front_spann; i++)
    {
        laserscan.distance_f += msg.ranges[i + laserscan.N/2 - laserscan.front_spann/2];
    }
    laserscan.distance_f = laserscan.distance_f/(laserscan.front_spann);
	#endif


	#ifdef MIN_RANG_VEL_CONTROL
    // Find the minimum in the given angular range
    laserscan.distance_f_min = 100.0;
    for (i=0; i < front_spann; i++)
    {
		if(laserscan.distance_f_min > msg.ranges[i + laserscan.N/2 - laserscan.front_spann/2])
			laserscan.distance_f_min = msg.ranges[i + laserscan.N/2 - laserscan.front_spann/2];
    }
	#endif
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //


	#ifdef MIN_RANG_VEL_CONTROL
        laserscan.distance_f = laserscan.distance_f_min;
	#endif


	// ++++++++++++++++++++++++++++ set global flags +++++++++++++++++++++++++++++++++++++++++ //
	// corridor 
    if(laserscan.distance_f > FRONT_BOUND)
    {
		// Object is deteced on the front of the car
        next_space = CORRIDOR;
        velocity.vel_gain = ((laserscan.distance_f - MIN_RANGE) / (MAX_RANGE-MIN_RANGE)) * (MAX_VEL - MIN_VEL);
	}

    // emergency brake
    if (laserscan.distance_f < FRONT_BREAK)
    {
       next_space = NO_FREE_SPACE;
	   if(space != NO_FREE_SPACE && next_space == NO_FREE_SPACE)
	   {
		   velocity.vel_gain_break = -500;
	   }
	   else
		   velocity.vel_gain_break = ((laserscan.distance_f - FRONT_BREAK) / (FRONT_BREAK))* 5 * (MIN_VEL_B - MAX_VEL_B);
	}

    // no object deteced
    if (distance_l > (SIDE_BOUND * 10))
    {
        next_space = ALL_FREE;
    }


	space = next_space;
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //

}





// ########################################## Corner Detection ############################### //

#ifdef CORNER_DETECTION
/*----------------------------------------------------------------------------------------------------
 * functionname:	positionCb(const geometry_msgs::PoseWithCovarianceStamped)

 * description:		Callback function, called every time when new amcl position data are published.
					Based on the position of the car the velocity of the car is calculated, while
					the car is driving around a corner

 * return par:		void
 *--------------------------------------------------------------------------------------------------*/
void positionCb(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
    if(CP_FILE_READ)
    {

		// +++++++++++++++++++++ read amcl pos + orientation + Cov + Stamp +++++++++++++++++++ //
        amcl_cb.pos_amcl.position = msg.pose.pose.position;
        amcl_cb.pos_amcl.orientation = msg.pose.pose.orientation;

		// +++++++++++++++++++++++++++++++ read corner point values ++++++++++++++++++++++++++ //
        cp_read = corner_pionts.at(CP_count - 1);

		// ++++++++++++++++++++++++++++ calc alpha(pos_amcl, pos_cp) +++++++++++++++++++++++++ //
        amcl_cb.delta_x = abs(amcl_cb.pos_amcl.position.x - cp_read.goe_msg_pos_cp.position.x);
        amcl_cb.delta_y = abs(amcl_cb.pos_amcl.position.y - cp_read.goe_msg_pos_cp.position.y);

        if(CP_count == 1)
        {
            amcl_cb.alpha = abs(atan(amcl_cb.delta_x/amcl_cb.delta_y) * RAD_TO_GRAD);
        }
        if(CP_count == 2)
        {
            amcl_cb.alpha = abs(atan(amcl_cb.delta_y/amcl_cb.delta_x) * RAD_TO_GRAD);
        }
        if(CP_count == 3)
        {
            amcl_cb.alpha = abs(atan(amcl_cb.delta_x/amcl_cb.delta_y) * RAD_TO_GRAD);
        }
        if(CP_count == 4)
        {
            amcl_cb.alpha = abs(atan(amcl_cb.delta_y/amcl_cb.delta_x) * RAD_TO_GRAD);
        }

		// ++++++++++++++++++++ compensate the rotation of the map +++++++++++++++++++++++++ //
        amcl_cb.alpha = amcl_cb.alpha + cp_read.alpha_correct * GRAD_TO_RAD;

				
		// +++++++++++++++++++++ set global corner flag  +++++++++++++++++++++++++++++++++++ //		
        switch(corner)
        {
                case NO_CORNER:
					// ##### angle based corner detection #### //
					#ifdef CORNER_DET_ANGLE
					if(amcl_cb.alpha>ALPHA_CORNER)
                    {
						velocity.vel_corner = -550;
						next_corner_state = APP_CORNER;
                    }
					#endif

					// ##### position based corner detection #### //
					#ifdef CORNER_DET_POSE
					if(CP_count == 1)
					{
						if(amcl_cb.pos_amcl.position.y >= cp_read.pos_leave - CORNER_BREAK)
						{
							// brake if enter corner
							velocity.vel_corner = -550;
							next_corner_state = APP_CORNER;
						}
					}
					
					if(CP_count == 2)
					{
						if(amcl_cb.pos_amcl.position.x <= cp_read.pos_leave; + CORNER_BREAK)
						{
							// brake if enter corner
							velocity.vel_corner = -550;
							next_corner_state = APP_CORNER;
						}
					}

					if(CP_count == 3)
					{
						if(amcl_cb.pos_amcl.position.y <= cp_read.pos_leave + CORNER_BREAK)
						{
							// brake if enter corner  
							velocity.vel_corner = -550;
							next_corner_state = APP_CORNER;
						}
					}

					if(CP_count == 4)
					{
						if(amcl_cb.pos_amcl.position.x >= cp_read.pos_leave - CORNER_BREAK)
						{
							// brake if enter corner
							velocity.vel_corner = -550;
							next_corner_state = APP_CORNER;
						}
					}
					#endif
					break;

                case APP_CORNER:

					// velocity based on angle alpha
                    velocity.vel_corner = (amcl_cb.alpha - ALPHA_CORNER)/amcl_cb.alpha * (MIN_CORN_APP_VEL - MAX_CORN_APP_VEL) + 10;

                    if(CP_count == 1)
                    {
						// enter leaving area
						amcl_cb.y_leave = cp_read.pos_leave;
                        if(amcl_cb.pos_amcl.position.y >= amcl_cb.y_leave)
							next_corner_state = LEAVE_CORNER;
                    }

                    if(CP_count == 2)
                    {
						// enter leaving area
						amcl_cb.x_leave = cp_read.pos_leave;
                        if(amcl_cb.pos_amcl.position.x <= amcl_cb.x_leave)
							next_corner_state = LEAVE_CORNER;
                    }

                    if(CP_count == 3)
                    {
						// enter leaving area
						amcl_cb.y_leave = cp_read.pos_leave;
                        if(amcl_cb.pos_amcl.position.y <= amcl_cb.y_leave)
                        next_corner_state = LEAVE_CORNER;
                    }

                    if(CP_count == 4)
                    {
						// enter leaving area
						amcl_cb.x_leave = cp_read.pos_leave;
                        if(amcl_cb.pos_amcl.position.x >= amcl_cb.x_leave)
                        next_corner_state = LEAVE_CORNER;
                    }

					break;

                case LEAVE_CORNER:

                    // calculate the difference of the car's orientation to the corner piont
                    amcl_cb.amcl_yaw = tf::getYaw(amcl_cb.pos_amcl.orientation);
                    amcl_cb.cp_yaw = tf::getYaw(cp_read.goe_msg_pos_cp.orientation);
                    amcl_cb.delta_yaw = abs(amcl_cb.amcl_yaw - amcl_cb.cp_yaw);

					// calculate the velocity based on the orientation difference
                    velocity.vel_corner = (90 - amcl_cb.delta_yaw)/90 * (MAX_CORN_LEAVE_VEL - MIN_CORN_LEAVE_VEL);

					// entering exit area 
                    if(amcl_cb.delta_yaw < DELTA_YAW)
                        next_corner_state  = EXIT_CORNER;
					break;

                case EXIT_CORNER:

					// fix exit velocity
                    velocity.vel_corner = MAX_CORN_EXIT_VEL;

                    if(CP_count == 1)
                    {
                        if(amcl_cb.pos_amcl.position.x <= cp_read.pos_exit + CP_POS_TOL)
						{
							// exit corner 
							next_corner_state = NO_CORNER;
							next_CP_count = 2;
						}
                    }

                    if(CP_count == 2)
                    {
                        if(amcl_cb.pos_amcl.position.y <= cp_read.pos_exit + CP_POS_TOL)
						{
							// exit corner
							next_corner_state = NO_CORNER;
							next_CP_count = 3;
						}
                    }

                    if(CP_count == 3)
                    {
                        if(amcl_cb.pos_amcl.position.x >= cp_read.pos_exit - CP_POS_TOL)
						{
							// exit corner
							next_corner_state = NO_CORNER;
							next_CP_count = 4;
						}
                    }

                    if(CP_count == 4)
                    {
						if(amcl_cb.pos_amcl.position.y >= cp_read.pos_exit - CP_POS_TOL)
						{
							// exit corner
							next_corner_state = NO_CORNER;
							next_CP_count = 1;
						}
                    }
	
					CP_count = next_CP_count;
					break;
				
				default:
				 next_corner_state = NO_CORNER;
				 break;

   }

   corner = next_corner_state;
 }

}
#endif

#endif


/*----------------------------------------------------------------------------------------------------
 * functionname:	main(int argc, char** argv)

 * description:		implements two subscriber. One for the laserscan datas and one for the amcl 
					position datas. Read in the recorded corner pionts from a structured txt file
					("Cornerpionts.txt"). Translats the trajectory given from the local planner into 
					PWM signals for the arduino board. Modifies the velocity of the trajectory via 
					two state machines (lase range based, position based corner detection).

 * return par:		void
 *--------------------------------------------------------------------------------------------------*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_control");
    control autonomous_control;

    #ifdef DYNAMIC_VEL_CONTROL
        ros::NodeHandle n;
		// +++++++++++++++++++ subscribe for the laserscan datas  +++++++++++++++++++++++++++++++++ //
        ros::Subscriber subScan = n.subscribe("/scan", 10, laserCb);
    #ifdef CORNER_DETECTION
		// +++++++++++++++++++ subscribe for the laserscan datas  +++++++++++++++++++++++++++++++++ //
        ros::Subscriber subAmcl = n.subscribe("/amcl_pose", 10, positionCb);
        ifstream ifs ("/home/tas_group_06/Documents/Cornerpionts.txt");
    #endif
    #endif

    velocity.vel_gain = 0;

	#ifdef CORNER_DETECTION
	// +++++++++++++++++++ read in the recorded corner pionts  +++++++++++++++++++++++++++++++++ //
	if(!ifs.is_open()) return -1;

    while (!ifs.eof())
    {
        ifs>>corner_piont.goe_msg_pos_cp.position.x>>
             corner_piont.goe_msg_pos_cp.position.y>>
             corner_piont.goe_msg_pos_cp.position.z;

        ifs>>corner_piont.goe_msg_pos_cp.orientation.x>>
             corner_piont.goe_msg_pos_cp.orientation.y>>
             corner_piont.goe_msg_pos_cp.orientation.z>>
             corner_piont.goe_msg_pos_cp.orientation.w;

        ifs>>corner_piont.alpha_correct;
		ifs>>corner_piont.pos_leave;
        ifs>>corner_piont.pos_exit;

        corner_pionts.push_back(corner_piont);
    }

	if(START_POS == 1){
		CP_count = CORNER_START_1;
		next_CP_count = CP_count;
	}

	if(START_POS == 2){
		CP_count = CORNER_START_2;
		next_CP_count = CP_count;
	}

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

                        // default velocity control
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

                        break;

                    case NO_FREE_SPACE:

						// emergency brake
						autonomous_control.control_servo.x = 1500 + velocity.vel_gain_break;
                        std::cout << "Lin Vel break" << velocity.vel_gain_break << std::endl;

                        break;

                    case CORRIDOR:

                        // Linear Velocity control
                        if(autonomous_control.cmd_linearVelocity>0)
                        {
                            std::cout << "state" << space << std::endl;

							// if car is in a corner -> corner velocity control
                            if(corner == APP_CORNER || corner == LEAVE_CORNER || corner == EXIT_CORNER)
                            {
								autonomous_control.control_servo.x = 1550 + velocity.vel_corner;

                                std::cout << "corner" << CP_count << std::endl;
                                std::cout << "state" << corner << std::endl;
                                std::cout << "Lin Vel corner gain" << velocity.vel_corner << std::endl;
                             }
                             else
                             {
								 // laser based velocity control
								 autonomous_control.control_servo.x = 1550 + velocity.vel_gain;
								 std::cout << "Lin Vel gain" << velocity.vel_gain << std::endl;
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

                        // Linear Velocity control
                        if(autonomous_control.cmd_linearVelocity>0)
                        {
                            std::cout << "state" << space << std::endl;

							// if car is in a corner -> corner velocity control
                            if(corner == APP_CORNER || corner == LEAVE_CORNER || corner == EXIT_CORNER)
                            {
								autonomous_control.control_servo.x = 1550 + velocity.vel_corner;

                                std::cout << "corner" << CP_count << std::endl;
                                std::cout << "state" << corner << std::endl;
                                std::cout << "Lin Vel corner gain" << velocity.vel_corner << std::endl;
                             }

                             else
                             {
								 autonomous_control.control_servo.x = 1570;
								 std::cout << "Lin Vel gain" << velocity.vel_gain << std::endl;
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
