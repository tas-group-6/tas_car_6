#ifdef OSVSTACL
#include "ObstacleDetection.h"

ObstacleDetection::ObstacleDetection(RobotClass* robot, MapClass* map) :
    e_robot(robot), e_map(map)
{
    m_connected.reserve(200);

    m_sensor_readings.reserve(25);
    m_obstacles.reserve(25);

}

ObstacleDetection::~ObstacleDetection(){}


void ObstacleDetection::update()
{
    SensorReadingType scan;
    ObstacleType obstacle;
    PoseType  cur_pose;
    geometry_msgs::Point view;

    int index1 = 0;
    int index2 = -1;
    int center;

    double radius, opening_angle, range, phi;

    const unsigned int N = e_robot->laserProxy()->GetRangeCount(); // The ammount of scans

    m_sensor_readings.clear();
    m_obstacles.clear();

    m_connected.resize(N); //When size is the same, nothing happens here

    //Test if connected, assign true or false
    for (unsigned int i = 0; i < N-1; i++)
        m_connected[i] = fabs( e_robot->laserProxy()->GetRange(i) - e_robot->laserProxy()->GetRange( i+1 ) ) <=  CONNECTED_THRESHOLD ;

    while (index1 < N)
    {
        //Index2 is the end of the last intervall -> next intervall starts at the following index
        index1 = index2+1;

        if( index1 >= N) break;

        // increase index 1 until next connected part is found
        // Due to noise background many neighbouring scans can be disconnected
        if( !m_connected[index1] ) { index1++; }

        if( index1 >= N) break;


        //Find connected scans
        index2 = index1;
        while ( m_connected[index2]) index2++;

        if( index2 >= N) break;


        //Inter division -> when intervall is even the smaller laser scan is used
        center = ( index2 - index1 ) / 2 + index1;

        range = e_robot->laserProxy()->GetRange( center );

        // Because of resolution ommit all scans which are further away than 3.25m
        if (range > RANGE_MAX_OBSTACLES) continue;

        //Full angle of triangle
        opening_angle = (index2 - index1) * e_robot->laserProxy()->GetAngularRes();

        phi = e_robot->laserProxy()->GetAngularRes() * center;

        //double test = opening_angle * RAD_TO_DEG; //For debugging total angle in degree

        //  __r___
        //  |    /
        //d |   /
        //  |a /
        //  | /
        //  |/
        //  a = 0.5 * opening_angle;
        //  d = distance of centered scan
        //  r = radius

        //TODO: Correct false approximation of 90 degree between d and r
        radius = tan( 0.5 * opening_angle ) * range;


        //Get pose of robot in the map to determine the position of the  obstacle
        //cur_pose.position = e_robot->getPosition();
        //cur_pose.rotation = e_robot->getRotationYaw();
        cur_pose = e_robot->Pose();

        //double test2 = cur_pose.rotation * RAD_TO_DEG; //For debugging rotation in degree

        //Set view direction for unrotated robot. Rotate this vector with the current rotation
        //When the robot is pointing to the right, the first laser scan value is in his frame again on the rgiht -> (-1,0 ) * range is
        //the position of the first point of the cloud
        view.setX(0.0);  view.setY(1.0);
        view = mh.Transform_Rotation( view , phi );
        view = mh.Transform_Rotation( view , -cur_pose.rotation );
        view.setX( -1.0 * view.x()); //Because of qt


        //double phi2 = phi * RAD_TO_DEG;  //For debugging angle of scan in degree


        /***********************/
        //Create scan element;
        /***********************/

        //TODO: Correct spatial extent of obstacle

        // All objects are round -> smallest distance at center
        scan.distance = range;

        //TODO: Add color detection
        scan.color = Qt::transparent;
        scan.angle =  e_robot->laserProxy()->GetAngularRes() * center;


        /***********************/
        //Create obstacle element
        /***********************/

        obstacle.position = cur_pose.position + view * scan.distance;
        obstacle.radius = radius;

        //TODO: Add color detection
        obstacle.color = Qt::transparent;

        // If this is a simulation use obstacle sizes for stage
        if (SIMULATION)
        {
            if (radius >= RADIUS_PUCK_SIM_MIN && radius < RADIUS_PUCK_SIM_MAX)
                obstacle.type = ObstacleEnum::Puck;
            else if ( radius >= RADIUS_MAPMARKER_SIM_MIN && radius < RADIUS_MAPMARKER_SIM_MAX)
                obstacle.type = ObstacleEnum::Map_Marker;
            else if ( radius >= RADIUS_PIONEER_SIM_MIN && radius < RADIUS_PIONEER_SIM_MAX)
                obstacle.type = ObstacleEnum::Pioneer;
            else
                obstacle.type = ObstacleEnum::UnknownObstacle;

        }

        //Otherwise use real measurable sizes
        else
        {
            if (radius >= RADIUS_PUCK_MIN && radius < RADIUS_PUCK_MAX)
                obstacle.type = ObstacleEnum::Puck;
            else if ( radius >= RADIUS_MAPMARKER_MIN && radius < RADIUS_MAPMARKER_MAX)
                obstacle.type = ObstacleEnum::Map_Marker;
            else if ( radius >= RADIUS_PIONEER_MIN && radius < RADIUS_PIONEER_MAX)
                obstacle.type = ObstacleEnum::Pioneer;
            else
                obstacle.type = ObstacleEnum::UnknownObstacle;
        }


        //Append to sensorreadings when the obstacle is a map marker
        if (obstacle.type == ObstacleEnum::Map_Marker) m_sensor_readings.push_back(scan);

        //And append the obstacle to the list of identified obstacles
        m_obstacles.push_back(obstacle);




    }//end while


    //Add known obstacles to the map
    e_map->addObstacles( &m_obstacles );

}


const std::vector<ObstacleType>* ObstacleDetection::GetObstacles() const{  return &m_obstacles;}
const std::vector<SensorReadingType>* ObstacleDetection::GetSensorReadings() const { return &m_sensor_readings;}


















#endif
