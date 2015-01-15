/**
 * This node sends fixed goals to move base via ROS Action API and receives feedback via callback functions.
 */

#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * Callback function
 */
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

/**
 * Callback function, called once when the goal becomes active
 */
void activeCb() {
    ROS_INFO("Goal just went active");
}

/**
 * Callback function, called every time feedback is received for the goal
 */
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    ROS_INFO("[X]:%f [Y]:%f [W]: %f [Z]: %f", feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w, feedback->base_position.pose.orientation.z);
}

/**
 * Main function
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_navigation_goals"); // init and set name
    std::vector<geometry_msgs::Pose> waypoints; // vector of goals, with position and orientation


// Load waypoints from text file
    ifstream ifs;
    ifs.open("/home/tas_group_06/Documents/Waypoints.txt");
    if(!ifs.is_open()) return -1;

    geometry_msgs::Pose waypoint1;

    string s1;


    while (!ifs.eof())
    {

        ifs>> waypoint1.position.x>>
              waypoint1.position.y>>
              waypoint1.position.z;

        ifs>>waypoint1.orientation.x>>
            waypoint1.orientation.y>>
            waypoint1.orientation.z>>
            waypoint1.orientation.w;

        waypoints.push_back(waypoint1);


    }







    MoveBaseClient ac("move_base", true); // action client to spin a thread by default

    while (!ac.waitForServer(ros::Duration(5.0))) { // wait for the action server to come up
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map"; // set target pose frame of coordinates

    ros::Rate goal_rate(1);


//Send goals in a loop
    while (ros::ok())
    {
			 // loop over all goal points, point by point
        for(int i = 0; i < waypoints.size(); ++i)
			 {
            goal.target_pose.header.stamp = ros::Time::now(); // set current time
            goal.target_pose.pose = waypoints.at(i);
            ROS_INFO("Sending goal");
            ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // send goal and register callback handler
            ac.waitForResult(); // wait for goal result

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("The base moved to %d goal", i);
            } else {
                ROS_INFO("The base failed to move to %d goal for some reason", i);
        }

            goal_rate.sleep();
    }
}
    return 0;
}
