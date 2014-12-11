#pragma once

//ROS includes
#include <geometry_msgs/Point.h>

//Std includes
#include <vector>

#define PI 3.14159265359
#define RAD_TO_DEG 180.0/PI // 360 / 2Pi
#define DEG_TO_RAD PI/180.0 // 360 / 2Pi

#define SAFE_DELETE( p ) { if( (p) ) { delete (p); (p) = 0; } }




/// Simple sturcture to throw custom exceptions with some message
struct ErrorType
{
  std::string msg;

  ErrorType( std::string s) : msg(s) {}

  void what() { printf("%s\n", msg.c_str()); }
};



/**
 * @brief Colors for the pucks and goals
 */
enum ColorType
{
  Red,
  Green,
  Blue,
  Yellow,
  UnknownColor
};

/**
 * @brief The ObstacleEnum enum provides names for the different obstacles
 */
enum ObstacleEnum
{
  Puck,
  Map_Marker,
  Pioneer,
  UnknownObstacle
};


/**
 * @brief The SensorReadingType struct contains the combined sensor readings
 */
struct SensorReadingType
{
  double distance; ///< in meter
  double angle;    ///< in radians
  ColorType color;

  SensorReadingType() :  color(UnknownColor) { }

};

/**
 * \brief Represents the pose of the robot in the world
 */
struct PoseType
{
  geometry_msgs::Point position;///< in meter
  double rotation;  ///< in radians
};


/**
 * \brief Represents a puck in the world
 */
struct PuckType
{
  geometry_msgs::Point position; ///< in meter
  double radius;    ///< in meter
  ColorType color;

  PuckType() : color(UnknownColor) { }
};

/**
 * \brief Represents a line in the world
 *

 */
struct LineType
{
  geometry_msgs::Point lineStart;    ///< in meter
  geometry_msgs::Point lineEnd;
  ColorType color;

  LineType() : color(UnknownColor) { }
};

/**
 * \brief Represents a goal in the world

 * Position is the center of the GoalType
 * Size is the half size of the goal.
 */
struct GoalType
{
  geometry_msgs::Point position;    ///< in meter
  geometry_msgs::Point size;        ///< halfsize in meter
  ColorType color;

  GoalType() : color(UnknownColor) { }
};




/**
 * \brief Datatype for the map.
 *
 * All values are in meters!\n
 * x-axis points from west to east.\n
 * y-axis points from south to north.\n
 * The angle of the pose is measured against the negativ y-axis.\n
 * => Phi = 0 <-> Robot is looking at (0, -1).\n
 */
struct MapType
{
  //Dynamic Part
  PoseType robot;
  PoseType enemy;

  std::vector<PuckType> pucks;

  //Static Part
  GoalType goal_robot;
  GoalType goal_enemy;

  std::vector<PuckType> map_marker;
  std::vector<LineType> lines;  
};


/**
 * \brief Represents an obstacle in the world
 .\n
 * This type is in principle just a puck, but it has a variable for the type of the obstacle.\n
 * This makes it easier to add obstacles to the map.\n
 * The type of the obstacles is defined by the radius.
 */
struct ObstacleType
{
  geometry_msgs::Point position; ///< in meter
  double radius;    ///< in meter
  ColorType color;
  ObstacleEnum type;

  ObstacleType() :  color(UnknownColor), type(UnknownObstacle) { }
};
