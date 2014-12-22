#pragma once


///Enables the use of player to simulate laser scans
//#define USE_PLAYER

#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <wiimote/State.h>


#ifdef USE_PLAYER
#include <libplayerc++/playerc++.h>
#endif

#include <string.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>



#include "DataType.h"



class RobotClass
{
public:
  RobotClass();
  ~RobotClass();

  bool update(ServoInstructionType& instruction);

  bool FillScanMessage(sensor_msgs::LaserScan& scan) const;

private:
  bool loadInstructionVector( std::vector<ServoInstructionType>& inst_vector, std::string filename);


private:

  enum StateType
  {
    STATE_INIT,
    STATE_LEFT,
    STATE_RIGHT,
    STATE_FINAL
  };


  std::vector<ServoInstructionType> m_initial_turn;
  std::vector<ServoInstructionType> m_turn_left;
  std::vector<ServoInstructionType> m_turn_right;

  unsigned int m_index_initial;
  unsigned int m_index_left;
  unsigned int m_index_right;
  unsigned int m_turn_counter;
  const unsigned int m_turn_counter_max = 4;

  StateType m_state;


#ifdef USE_PLAYER
public:

  // Accessor Methods for the player stuff

  const PlayerCc::PlayerClient *playerClient() const;
  const PlayerCc::RangerProxy *laserProxy() const;
  const PlayerCc::Position2dProxy *position2dProxy() const;



private:

  //The player stuff

  PlayerCc::PlayerClient *m_playerClient;
  PlayerCc::RangerProxy* m_laserProxy;
  PlayerCc::Position2dProxy* m_position2dProxy;

#endif



};
