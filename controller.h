#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QObject>
#include "defines.h"

class Controller : public QObject
{
    Q_OBJECT
public:
    explicit Controller(QObject *parent = 0);

private:
    Point2D m_goal;
    RobotVector m_robots;
    void controlAction();
    int m_robotToControl;

signals:
    void sendRobotData(RobotDataVector data);

public slots:
    void getGoal(Point2D goal) {m_goal = goal; controlAction();}
    void getRobots(RobotVector robots) {m_robots = robots; controlAction();}
};

#endif // CONTROLLER_H
