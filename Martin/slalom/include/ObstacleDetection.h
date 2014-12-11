#pragma once

#include <vector>
#include <math.h>


#include <libplayerc++/playerc++.h>

#include "DataType.h"
#include "MathHelper.h"

#include "mapclass.h"
#include "RobotClass.h"

const bool SIMULATION = true;

const double RADIUS_PUCK_MIN = 0.00;         ///< Blue / Yellow pucks Radius Min, in meter for real robot only!
const double RADIUS_PUCK_MAX = 0.05;         ///< Blue / Yellow pucks Radius Max, in meter for real robot only!

const double RADIUS_PUCK_SIM_MIN = 0.035;    ///< Map Marker Radius Min, in meter for simulation only;
const double RADIUS_PUCK_SIM_MAX = 0.07;    ///< Map Marker Radius Max, in meter for simulation only;

const double RADIUS_MAPMARKER_MIN = 0.05;    ///< Map Marker Radius Min, in meter for real robot only;
const double RADIUS_MAPMARKER_MAX = 0.08;    ///< Map Marker Radius Max, in meter for real robot only;

const double RADIUS_MAPMARKER_SIM_MIN = 0.07;    ///< Map Marker Radius Min, in meter for simulation only;
const double RADIUS_MAPMARKER_SIM_MAX = 0.13;    ///< Map Marker Radius Max, in meter for simulation only;

//@{
/**
* The radius of the pioneer, in meter\n
* In the scan only the Sick scanner and the PC can be seen.\n
* The poineer fits into a square of 15x30 cm.\n
* So for the type of the obstacle it is enough to test if the radius is
* in this range.\n
*/
const double RADIUS_PIONEER_MIN = 0.15;     /// Real robot
const double RADIUS_PIONEER_SIM_MIN = 0.25; /// Simulation

const double RADIUS_PIONEER_MAX = 0.30;     /// Real robot
const double RADIUS_PIONEER_SIM_MAX = 0.40; /// Simulation
//@}

/// The maximal difference in the range until two neighboured laser scans count as seperated.\n
/// In meter.
const double CONNECTED_THRESHOLD = 0.1;

/**
 * @brief The ObstacleDetection class takes the LaserProxy and creates obstacles
 *
 * A Color for the obstacles is added when it is available. Otherwise the obstacles stay 'transparent'
 * Different types of obstacles are determined by the radius of the object. Only the visible part is used!
 * For example the pucks are larger at the bottom.
 */
class ObstacleDetection
{
public:
    ObstacleDetection(RobotClass* robot, MapClass* map);
    ~ObstacleDetection();


    /**
     * @brief GetObstacles Accessor method to known obstacles
     * @return Pointer to vector of ObstacleType
     *
     * All obstacles are returned as ObstacleType.\n
     * When an obstacle is detected by the laser, but not identified by the camera
     * its color is transparent.\n
     * The Type of the obstacles is determined by the radius.\n
     */
    const std::vector<ObstacleType>* GetObstacles() const;


    /**
     * @brief GetSensorReading Accessor method to the scans
     * @return Pointer to vector of SensorReadingType
     *
     * The Vector contains the ranges and angles to the known obstacles.\n
     * Should only be neccessary for the localization
     */
    const std::vector<SensorReadingType>* GetSensorReadings() const;

    //public Q_SLOTS:
    /**
     * @brief update Call this function to process the current laser scan
     * @param laser Pointer to the Player LaserProxy
     *
     * Fills the vectors m_sensor_readings and m_obstacles. Both vectors are cleared first!
     */
    void update();


private:

    /** This vector indicates if two neighboured laser scans are connected.\n
     *  When the ranges are seperated, i.e. a Puck stands infront of a wall, then the vector
     *  is false at that position, otherwise true.
    */
    std::vector<bool> m_connected;


    std::vector<SensorReadingType> m_sensor_readings; ///< Simplified Sensor Readings
    std::vector<ObstacleType> m_obstacles; ///< all detected obstacles


    //PlayerCc::LaserProxy* e_laser;
    //PlayerCc::RangerProxy* e_laser;
    //LocalizationAndMapping* e_lam;

    RobotClass* e_robot;
    MapClass* e_map;


    MathHelper mh;
};

