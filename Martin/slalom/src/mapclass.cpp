#ifdef TEDSLKFJ
#include "mapclass.h"

MapClass::MapClass()
{
    //Create the Model of the map and define the known parts
    m_map_model = new MapType();

    FillMap();

}


MapClass::~MapClass()
{
    delete m_map_model;
}




void MapClass::FillMap()
{
    m_map_model->pucks.reserve(6);

    m_map_model->map_marker.resize(MAP_MARKER_COUNT);

    m_map_model->map_marker[ 0].position = QPointF(0.00*MAP_A,MAP_B);
    m_map_model->map_marker[ 1].position = QPointF(0.25*MAP_A,MAP_B);
    m_map_model->map_marker[ 2].position = QPointF(1.00*MAP_A,MAP_B);
    m_map_model->map_marker[ 3].position = QPointF(1.50*MAP_A,MAP_B);
    m_map_model->map_marker[ 4].position = QPointF(2.00*MAP_A,MAP_B);
    m_map_model->map_marker[ 5].position = QPointF(2.75*MAP_A,MAP_B);
    m_map_model->map_marker[ 6].position = QPointF(3.00*MAP_A,MAP_B);


    m_map_model->map_marker[7].position = QPointF(3.00*MAP_A,0.0);
    m_map_model->map_marker[8].position = QPointF(2.75*MAP_A,0.0);
    m_map_model->map_marker[9].position = QPointF(2.00*MAP_A,0.0);
    m_map_model->map_marker[10].position = QPointF(1.50*MAP_A,0.0);
    m_map_model->map_marker[11].position = QPointF(1.00*MAP_A,0.0);
    m_map_model->map_marker[12].position = QPointF(0.25*MAP_A,0.0);
    m_map_model->map_marker[13].position = QPointF(0.00*MAP_A,0.0);

    for (int var = 0; var < m_map_model->map_marker.size(); ++var)
        m_map_model->map_marker[var].color = Qt::green;

    m_map_model->lines.resize(9);
    m_map_model->lines[0].line = QLineF(
                m_map_model->map_marker[1].position,
            m_map_model->map_marker[12].position );
    m_map_model->lines[0].color = Qt::red;


    m_map_model->lines[1].line = QLineF(
                m_map_model->map_marker[2].position,
            m_map_model->map_marker[11].position );
    m_map_model->lines[1].color = Qt::blue;


    m_map_model->lines[2].line = QLineF(
                m_map_model->map_marker[3].position,
            m_map_model->map_marker[10].position );
    m_map_model->lines[2].color = Qt::red;


    m_map_model->lines[3].line = QLineF(
                m_map_model->map_marker[4].position,
            m_map_model->map_marker[9].position );
    m_map_model->lines[3].color = Qt::blue;


    m_map_model->lines[4].line = QLineF(
                m_map_model->map_marker[5].position,
            m_map_model->map_marker[8].position );
    m_map_model->lines[4].color = Qt::red;



    m_map_model->lines[5].line = QLineF(
                m_map_model->map_marker[0].position,
            m_map_model->map_marker[6].position );
    m_map_model->lines[5].color = Qt::black;

    m_map_model->lines[6].line = QLineF(
                m_map_model->map_marker[7].position,
            m_map_model->map_marker[13].position );
    m_map_model->lines[6].color = Qt::black;

    m_map_model->lines[7].line = QLineF(
                m_map_model->map_marker[0].position,
            m_map_model->map_marker[13].position );
    m_map_model->lines[7].color = Qt::black;

    m_map_model->lines[8].line = QLineF(
                m_map_model->map_marker[7].position,
            m_map_model->map_marker[6].position );
    m_map_model->lines[8].color = Qt::black;


    m_map_model->goal_robot.size = QRectF(- MAP_A / 8.0, - MAP_B / 6.0, + MAP_A / 4.0, + MAP_B / 3.0);
    m_map_model->goal_robot.position = QPointF( 3.0/8.0 * MAP_A , 0.5 * MAP_B);


    m_map_model->goal_enemy.size = QRectF(- MAP_A / 8.0, - MAP_B / 6.0, + MAP_A / 4.0, + MAP_B / 3.0);
    m_map_model->goal_enemy.position = QPointF( 2.625 * MAP_A, 0.5 * MAP_B);


}



void MapClass::addObstacles(const std::vector<ObstacleType> *obstacles)
{
    PuckType p;

    m_map_model->pucks.clear();



    for (unsigned int i = 0; i < obstacles->size(); ++i)
    {
        switch (obstacles->at(i).type)
        {
        case ObstacleEnum::Puck:

            p.position = obstacles->at(i).position;
            p.color = obstacles->at(i).color;
            p.radius = obstacles->at(i).radius;
            m_map_model->pucks.push_back(p);

            break;

        case ObstacleEnum::Pioneer:
            m_map_model->enemy.position = obstacles->at(i).position;

            //TODO: No Roation of obstacle available
            m_map_model->enemy.rotation =  0.0;

            break;

        case ObstacleEnum::Map_Marker:
            //Nothing needs to be done here. The map_marker are hardcoded
            break;


        case ObstacleEnum::UnknownObstacle:

            //TODO: Add element in map for unkown obstacle

            break;

        default:
            break;
        }
    }

}


void MapClass::SetGoalColor(const Qt::GlobalColor &robotGoal, const Qt::GlobalColor &enemyGoal)
{
    m_map_model->goal_enemy.color = enemyGoal;
    m_map_model->goal_robot.color = robotGoal;
}

const MapType* MapClass::getMap() const { return m_map_model;}


void MapClass::SetRobotPose(const PoseType &p) { m_map_model->robot = p;}
#endif
