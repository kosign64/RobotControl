#ifndef ROBOTFINDER_H
#define ROBOTFINDER_H

#include <QObject>
#include "defines.h"

class RobotFinder : public QObject
{
    Q_OBJECT
public:
    explicit RobotFinder(QObject *parent = 0);
    static Robot2D &getRobotByNumber(RobotVector &vector, int number);

private:

    enum State
    {
        RUN_ROBOT,
        WAIT,
        CHECK_ROBOT
    };

    RobotVector robotsPrev;
    RobotVector robotsStart;
    bool findCorrespondence;
    bool start;
    State state;

    bool findRobotsFromPoints(RobotVector &robotVector, const PointVector &pointVector);

    static double length(const Point2D &p1, const Point2D &p2);
    static Point2D center(Point2D &p1, Point2D &p2);
    static void findMovedRobot(const RobotVector &start,
                              const RobotVector &current,
                              int &movedStart,
                              int &movedCurrent);

signals:
    void sendRobots(RobotVector robots);
    void sendRobotData(RobotDataVector data);

public slots:
    void getPoints(PointVector &pointVector);
};

#endif // ROBOTFINDER_H
