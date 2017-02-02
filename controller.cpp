#include "controller.h"
#include "robotfinder.h"
#include <QDebug>

Controller::Controller(QObject *parent) : QObject(parent),
    m_goal{0, 0},
    m_robotToControl(1)
{

}

void Controller::controlAction()
{
    if((m_goal.x == 0) && (m_goal.y == 0)) return;
    RobotData data;
    RobotDataVector vec;
    data.number = m_robotToControl;
    Robot2D &robot = RobotFinder::getRobotByNumber(m_robots, m_robotToControl);

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
