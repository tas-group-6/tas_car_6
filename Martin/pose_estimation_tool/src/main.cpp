//Standart
#include <iostream>


//ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

geometry_msgs::Pose p[8];


void fill_posearray()
{
  p[0].position.x  = 20.0;  p[0].position.y  = 10.0;  p[0].position.z  = 0.0;
  p[0].orientation.x = 1.0; p[0].orientation.y = 1.0; p[0].orientation.z = 1.0; p[0].orientation.w = 0.0;

  p[1].position.x  = 20.0;  p[1].position.y  = 10.0;  p[1].position.z  = 0.0;
  p[1].orientation.x = 1.0; p[1].orientation.y = 1.0; p[1].orientation.z = 1.0; p[1].orientation.w = 0.0;

  p[2].position.x  = 20.0;  p[2].position.y  = 10.0;  p[2].position.z  = 0.0;
  p[2].orientation.x = 1.0; p[2].orientation.y = 1.0; p[2].orientation.z = 1.0; p[2].orientation.w = 0.0;

  p[3].position.x  = 20.0;  p[3].position.y  = 10.0;  p[3].position.z  = 0.0;
  p[3].orientation.x = 1.0; p[3].orientation.y = 1.0; p[3].orientation.z = 1.0; p[3].orientation.w = 0.0;

  p[4].position.x  = 20.0;  p[4].position.y  = 10.0;  p[4].position.z  = 0.0;
  p[4].orientation.x = 1.0; p[4].orientation.y = 1.0; p[4].orientation.z = 1.0; p[4].orientation.w = 0.0;

  p[5].position.x  = 20.0;  p[5].position.y  = 10.0;  p[5].position.z  = 0.0;
  p[5].orientation.x = 1.0; p[5].orientation.y = 1.0; p[5].orientation.z = 1.0; p[5].orientation.w = 0.0;

  p[6].position.x  = 20.0;  p[6].position.y  = 10.0;  p[6].position.z  = 0.0;
  p[6].orientation.x = 1.0; p[6].orientation.y = 1.0; p[6].orientation.z = 1.0; p[6].orientation.w = 0.0;

  p[7].position.x  = 20.0;  p[7].position.y  = 10.0;  p[7].position.z  = 0.0;
  p[7].orientation.x = 1.0; p[7].orientation.y = 1.0; p[7].orientation.z = 1.0; p[7].orientation.w = 0.0;
}

void draw_help()
{
  std::cout << "              N               " << std::endl;
  std::cout << "   +--------+   +--------+    " << std::endl;
  std::cout << "   |0  =>  1|   |4  <=  5|    " << std::endl;
  std::cout << " W |        |   |        | O  " << std::endl;
  std::cout << "   |3  <=  2|   |7  =>  6|    " << std::endl;
  std::cout << "   +--------+   +--------+    " << std::endl;
  std::cout << "              S               " << std::endl;
}

int main(int argc, char **argv)
{
  fill_posearray();
  draw_help();

  ros::init(argc, argv, "pose_esitmation_tool");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_estimation", 1);

  geometry_msgs::PoseWithCovarianceStamped pose_estimation;

  std::cout << "Choose initial position: [0-8]" << std::endl;

  int input;
  std::cin >> input;

  if( input > 7 || input < 0){ std::cout << "Wrong input" << std::endl; return -1;}


  pose_estimation.header.frame_id = "/map";
  pose_estimation.header.stamp = ros::Time::now();

  pose_estimation.pose.pose = p[input];
  pub.publish( pose_estimation );


  std::printf("Initial Pose Published\n");
  std::printf("Pose       : %.3f\t%.3f\t%.3f\n", p[input].position.x, p[input].position.y, p[input].position.z);
  std::printf("Orientation: %.3f\t%.3f\t%.3f\t%.3f\n", p[input].orientation.x, p[input].orientation.y , p[input].orientation.z, p[input].orientation.w);


  return 1;
}
