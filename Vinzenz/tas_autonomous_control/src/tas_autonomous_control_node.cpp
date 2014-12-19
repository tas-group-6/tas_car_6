#include "control/control.h"

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>


#define __USE_BSD
#include <math.h>


#define DAYNAMIC_VEL_CONTROL


#ifdef DAYNAMIC_VEL_CONTROL

#define GRAD_TO_RAD                     0.01745329252

#define FRONT_RES                       20   // [Grad]
#define SIDE_RES                        20   // [Grad]

#define FRONT_BOUND                     1.5    // [Meter]
#define SIDE_BOUND                      0.2 + 0.2    // [Meter]

// *********** BREAK Parameters **************** //
#define SIDE_BREAK						0.2 + 0.3  // [Meter]
#define	FRONT_BREAK						0.2  // [Meter]

// *********** Lin Vel control Parameters ****** //
#define MAX_RANGE						2  // [Meter]
#define MIN_RANGE						0.6	 // [Meter]
#define MIN_VEL							1550 // [PWM]
#define MAX_VEL							1570 // [PWM]

#define MIN_VEL_B						1500 // [PWM]
#define MAX_VEL_B						1400 // [PWM]

// (RANG - Min_RANGE) / (MAX_RANGE-MIN_RANGE) * (MAX_VEL - MIN_VEL) + DEFAULT_VEL


#endif



// free space identifier [0: No free space, 5: All free space]
enum FREE_SPACE { SPACE_DEFAULT, NO_FREE_SPACE, CORNER, NO_CORNER, ALL_FREE};
FREE_SPACE space = SPACE_DEFAULT; // default state

int vel_gain = 0;
int vel_gain_break = 0;

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

    int i = 0;

    // Scanning Front
    distance_f = 0.0;
	// Approximation: Calc the avg from all meassurments in a given agualr range
    for (i=0; i < front_spann*2; i++)
    {
        distance_f += msg.ranges[i + N/2 - front_spann];
    }
    distance_f = distance_f/(front_spann*2);

    // Scanning Side
    distance_l = 0.0;
	// Approximation: Calc the avg from all meassurments in a given agualr range
    for (i=0; i < side_spann; i++)
    {
        distance_l += msg.ranges[i];
    }
    distance_l = distance_l/(side_spann);

    distance_r = 0.0;
	// Approximation: Calc the avg from all meassurments in a given agualr range
    for (i=0; i < side_spann; i++)
    {
        distance_r += msg.ranges[i];
    }
    distance_r = distance_r/(side_spann);


    // Set Global Flags
    if((distance_l >= SIDE_BOUND) && (distance_r <= SIDE_BOUND))
    {
		// if a object is detected on the sides of the car
        space = NO_CORNER;
		vel_gain = ((distance_l - Min_RANGE) / (MAX_RANGE-MIN_RANGE)) * (MAX_VEL - MIN_VEL);
		if ((((distance_r - Min_RANGE) / (MAX_RANGE-MIN_RANGE)) * (MAX_VEL - MIN_VEL)) < 
			vel_gain)
		{
			// choos the smaller velocity
			vel_gain =(((distance_r - Min_RANGE) / (MAX_RANGE-MIN_RANGE)) * (MAX_VEL - MIN_VEL));
		}
        if (distance_f < SIDE_BOUND)

			// +  a objected is deteced in front of the car
            space = CORNER;
    }

	// Emergancy Break
    if (distance_l < SIDE_BREAK || distance_r < SIDE_BREAK || distance_f < FRONT_BREAK)
    {
       space = NO_FREE_SPACE;
			 vel_gain_break = ((distance_f - FRONT_BREAK) / (FRONT_BREAK)) * (MIN_VEL - MAX_VEL);
			 
    }

	// No object deteced
    if ((distance_l > (SIDE_BOUND * 10) || distance_r > (SIDE_BOUND * 10)) && (distance_f > (SIDE_BOUND * 10)) )
    {
        space = ALL_FREE;
    }


}
#endif


int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_control");
    control autonomous_control;

	int vel_gain = 0;

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

                switch(space)
                {
                    case SPACE_DEFAULT:
                        // Default velocity control
                        if(autonomous_control.cmd_linearVelocity>0)
                        {
                            autonomous_control.control_servo.x = 1555;
                        }
                        else if(autonomous_control.cmd_linearVelocity<0)
                        {
                            autonomous_control.control_servo.x = 1300;
                        }
                        else
                        {
                         autonomous_control.control_servo.x = 1500;
                        }

                        break;

                    case NO_FREE_SPACE:
                    // Stopp Car
                        autonomous_control.control_servo.x = 1500 + vel_gain_break;
                        break;

                    case CORNER:
                        // Default velocity control
                        if(autonomous_control.cmd_linearVelocity>0)
                        {
                            autonomous_control.control_servo.x = 1550;
                        }
                        else if(autonomous_control.cmd_linearVelocity<0)
                        {
                            autonomous_control.control_servo.x = 1300;
                        }
                        else
                        {
                        autonomous_control.control_servo.x = 1500;
                        }

                        break;

                    case NO_CORNER:
                        // Linear Velocity control
                        if(autonomous_control.cmd_linearVelocity>0)
                        {
                            autonomous_control.control_servo.x = 1550 + vel_gain;
                        }
                        else if(autonomous_control.cmd_linearVelocity<0)
                        {
                            autonomous_control.control_servo.x = 1300;
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
                            autonomous_control.control_servo.x = 1570;
                        }
                        else if(autonomous_control.cmd_linearVelocity<0)
                        {
                            autonomous_control.control_servo.x = 1300;
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
