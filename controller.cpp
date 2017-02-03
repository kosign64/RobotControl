#include "controller.h"
#include "robotfinder.h"
#include <QDebug>

Controller::Controller(QObject *parent) : QObject(parent),
    m_goal{0, 0},
    m_robotToControl(1)
{
    m_engine = new Engine("Fuzzy");
    m_goalDistance = new InputVariable("GoalDistance");
    m_goalAngle = new InputVariable("GoalAngle");
    m_obstacleDistance = new InputVariable("ObstacleDistance");
    m_obstacleAngle = new InputVariable("ObstacleAngle");
    m_goalDistance->setEnabled(true);
    m_goalAngle->setEnabled(true);
    m_obstacleDistance->setEnabled(true);
    m_obstacleAngle->setEnabled(true);
    m_goalDistance->setRange(0, 800);
    m_obstacleDistance->setRange(0, 800);
    m_goalAngle->setRange(-3.14, 3.14);
    m_obstacleAngle->setRange(-3.14, 3.14);
    m_goalDistance->addTerm(new ZShape("LOW", 0, 50));
}

void Controller::controlAction()
{
    if((m_goal.x == 0) && (m_goal.y == 0)) return;
    Robot2D &robot = RobotFinder::getRobotByNumber(m_robots, m_robotToControl);

    dumbController(robot);
    //fuzzyController(robot);
}

void Controller::dumbController(const Robot2D &robot)
{
    RobotData data;
    RobotDataVector vec;
    data.number = m_robotToControl;
    double desiredAngle = atan2(robot.center.y - m_goal.y,
            robot.center.x - m_goal.x);
    desiredAngle += M_PI;

    if(sqrt(pow(robot.center.y - m_goal.y, 2) +
            pow(robot.center.x - m_goal.x, 2)) < 25)
    {
        data.cByte = STOP_BYTE;
        vec << data;
        emit sendRobotData(vec);
    }
    else
    {
        if(((fabs(robot.angle - desiredAngle) * 180 / M_PI) < 15) ||
           ((fabs(robot.angle + 2 * M_PI - desiredAngle) * 180 / M_PI) < 15) ||
           ((fabs(robot.angle - (desiredAngle + 2 * M_PI)) * 180 / M_PI) < 15))
        {
            data.cByte = FORWARD_BYTE;
            vec << data;
            emit sendRobotData(vec);
        }
        else if(robot.angle > desiredAngle)
        {
            if(fabs(robot.angle - desiredAngle) > M_PI)
            {
                data.cByte = RIGHT_BYTE;
                vec << data;
                emit sendRobotData(vec);
            }
            else
            {
                data.cByte = LEFT_BYTE;
                vec << data;
                emit sendRobotData(vec);
            }
        }
        else
        {
            if(fabs(robot.angle - desiredAngle) > M_PI)
            {
                data.cByte = LEFT_BYTE;
                vec << data;
                emit sendRobotData(vec);
            }
            else
            {
                data.cByte = RIGHT_BYTE;
                vec << data;
                emit sendRobotData(vec);
            }
        }
    }
}

void Controller::fuzzyController(const Robot2D &robot)
{
    Robot2D &obstacle = RobotFinder::getRobotByNumber(m_robots, 2 - (m_robotToControl - 1));
    double goalAngle = angleToPoint(robot, m_goal);
    double obstacleAngle = angleToPoint(robot, obstacle.center);

    qDebug() << goalAngle * 180 / M_PI << obstacleAngle * 180 / M_PI;
}

double Controller::angleToPoint(const Robot2D &robot, const Point2D &point)
{
    double angle;
    if(robot.center.x > point.x)
    {
        angle = atan2(robot.center.y - point.y,
                      robot.center.x - point.x);
    }
    else
    {
        angle = atan2(point.y - robot.center.y,
                      point.x - robot.center.x);
    }

    angle -= robot.angle;

    if(robot.center.x > point.x)
    {
        angle = M_PI + angle;
    }
    if(angle < 0)
    {
        angle = 2 * M_PI + angle;
    }
    if(angle > M_PI) angle = angle - 2 * M_PI;

    return angle;
}
