#ifdef MAPCLASS
#pragma once

#include "DataType.h"



#define MAP_A 5.0 / 3.0
#define MAP_B 3.0

class MapClass
{
public:
    MapClass();
    ~MapClass();


    /**
     * @brief SetGoalColor sets the color of both goals
     * @param robotGoal The color of the own goal
     * @param enemyGoal The color of the hostile goal
     */
    void SetGoalColor( const Qt::GlobalColor& robotGoal, const Qt::GlobalColor& enemyGoal);


    /**
      * @brief SetRobotPose takes a pose and stores it in the map
      * @param p  PoseType
      *
      * When the new position for the robot is calculated the result can be stored in the map via this function
      */
    void SetRobotPose( const PoseType& p);



    /**
     * @brief addObstacles Adds these obstacles to the map
     * @param obstacles all detected obstacles
     *
     * Only pucks and the enemy robot are added to the map.\n
     * The map marker and the goal positions are known  from the beginning.\n
     * The position of the own robot is done by this class in the update method.\n
     * The goal color is set from outside of this class.
     */
    void addObstacles( const std::vector<ObstacleType>* obstacles);




    /**
     * @brief getMap Accessor Method for Map
     * @return a constant pointer to the map
     */
    const MapType* getMap() const;


private:

    /// Fills the static part of the map, i.e. map_marker and goal position
    void FillMap();

    /// Holds the known facts of the map and serves as a reference.
    MapType* m_map_model;


};

#endif
