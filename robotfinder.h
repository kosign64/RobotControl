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
    static double length(const Point2D &p1, const Point2D &p2);

private:

    enum State
    {
        RUN_ROBOT,
        WAIT,
        CHECK_ROBOT
    };

    RobotVector robotsPrev_;
    RobotVector robotsStart_;
    bool start_;
    State state_;

    bool findRobotsFromPoints(RobotVector &robotVector, const PointVector &pointVector) const;

    static Point2D center(const Point2D &p1, const Point2D &p2);
    static void findMovedRobot(const RobotVector &start,
                              const RobotVector &current,
                              int &movedStart,
                              int &movedCurrent);

signals:
    void sendRobots(RobotVector robots);
    void sendRobotData(RobotDataVector data);

public slots:
    void setPoints(PointVector &pointVector);
};

#endif // ROBOTFINDER_H
