#include "robotfinder.h"
#include <cmath>
#include <cassert>
#include <QDebug>
#include <unistd.h>

RobotFinder::RobotFinder(QObject *parent) : QObject(parent),
    findCorrespondence(true),
    start(true),
    state(RUN_ROBOT)
{

}

void RobotFinder::getPoints(PointVector &pointVector)
{
    RobotVector robots;
    findRobotsFromPoints(robots, pointVector);
    if(robots.isEmpty()) return;

//    static int robotNumber = robots.size() + 1;
//    static int checkingRobotNumber = 1;
//    static const int stopTime = 3500;
//    static const int waitTime = 5000;
//    if(checkingRobotNumber < robotNumber)
//    {
//        RobotData data;
//        RobotDataVector vec;
//        switch (state)
//        {
//        case RUN_ROBOT:
//            if(start)
//            {
//                robotsStart = robots;
//                start = false;
//            }
//            static int stopIteration = 0;
//            data.number = checkingRobotNumber;
//            data.cByte = FORWARD_BYTE;
//            vec.append(data);
//            emit sendRobotData(vec);
//            if(++stopIteration > stopTime)
//            {
//                stopIteration = 0;
//                state = WAIT;
//            }
//            break;

//        case WAIT:
//            static int waitIteration = 0;
//            data.number = checkingRobotNumber;
//            data.cByte = STOP_BYTE;
//            vec.append(data);
//            emit sendRobotData(vec);
//            if(++waitIteration > waitTime)
//            {
//                waitIteration = 0;
//                state = CHECK_ROBOT;
//            }
//            break;

//        case CHECK_ROBOT:
//            int movedStart, movedCurrent;
//            findMovedRobot(robotsStart, robots,
//                           movedStart, movedCurrent);
//            Robot2D &startRob = robotsStart[movedStart];
//            Robot2D &currentRob = robots[movedCurrent];
//            Robot2D &otherRob = getRobotByNumber(robots,
//                                              checkingRobotNumber);
//            otherRob.number = currentRob.number;
//            currentRob.number = checkingRobotNumber;
//            Point2D estimated1;
//            Point2D estimated2;
//            estimated1.x = startRob.center.x + 20 * cos(startRob.angle);
//            estimated1.y = startRob.center.y + 20 * sin(startRob.angle);
//            estimated2.x = startRob.center.x + 20 * cos(startRob.angle + M_PI);
//            estimated2.y = startRob.center.y + 20 * sin(startRob.angle + M_PI);
//            if(length(estimated1, currentRob.center) <
//                    length(estimated2, currentRob.center))
//            {
//                currentRob.angle = startRob.angle;
//            }
//            else
//            {
//                currentRob.angle = startRob.angle + M_PI;
//            }
//            start = true;
//            checkingRobotNumber++;
//            state = RUN_ROBOT;
//            break;
//        }
//    }
//    else
//    {
//        emit sendRobots(robots);
//    }
//    robotsPrev = robots;
//}



    if(findCorrespondence)
    {
        static int n = 0;
        static int l = 1;
        static int robotNumber = robots.size() + 1;
        if(l < robotNumber)
        {
            // Start RobotMovement
            RobotData data;
            data.number = l;
            data.cByte = _BV(G) | _BV(DL) | _BV(VL1) | _BV(DR) | _BV(VR1);
            RobotDataVector vec;
            vec.append(data);
            emit sendRobotData(vec);
            if(++n > 3500)
            {
                // Stop Movement
                data.number = l;
                data.cByte = _BV(G);
                vec[0] = data;
                emit sendRobotData(vec);

                int movedStart, movedCurrent;
                findMovedRobot(robotsStart, robots,
                               movedStart, movedCurrent);
                Robot2D &startRob = robotsStart[movedStart];
                Robot2D &currentRob = robots[movedCurrent];
                Robot2D &otherRob = getRobotByNumber(robots,
                                                  l);
                otherRob.number = currentRob.number;
                currentRob.number = l;
                Point2D estimated1;
                Point2D estimated2;
                estimated1.x = startRob.center.x + 20 * cos(startRob.angle);
                estimated1.y = startRob.center.y + 20 * sin(startRob.angle);
                estimated2.x = startRob.center.x + 20 * cos(startRob.angle + M_PI);
                estimated2.y = startRob.center.y + 20 * sin(startRob.angle + M_PI);
                if(length(estimated1, currentRob.center) <
                        length(estimated2, currentRob.center))
                {
                    currentRob.angle = startRob.angle;
                }
                else
                {
                    currentRob.angle = startRob.angle + M_PI;
                }
                robotsStart = robots;
                n = 0;
                l++;
            }
        }
        else
        {
            findCorrespondence = false;
        }
    }
    else
    {
        emit sendRobots(robots);
    }
    robotsPrev = robots;
    start = false;
}

void RobotFinder::findRobotsFromPoints(RobotVector &robotVector, const PointVector &pointVector)
{
    PointVector pointsRaw = pointVector;
    for(int i = 0; i < pointsRaw.size(); ++i)
    {
        pointsRaw[i].x = (pointsRaw[i].x - X_OFFSET) / X_SCALE;
        pointsRaw[i].y -= Y_OFFSET;
    }

    // Get rid of non-robot points
    PointVector points;
    for(int i = 0; i < (pointsRaw.size() - 1); ++i)
    {
        bool newPoint = true;
        for(int j = i + 1; j < pointsRaw.size(); ++j)
        {
            if(length(pointsRaw[i], pointsRaw[j]) < 10)
            {
                newPoint = false;
            }
        }
        if(newPoint)
        {
            points.append(pointsRaw[i]);
        }
    }
    if(!pointsRaw.empty())
    {
        points.append(pointsRaw[pointsRaw.size() - 1]);
    }

    // Find Robots
    robotVector.clear();
    int robotNum = 0;
    for(int i = 0; i < (points.size() - 1); ++i)
    {
        for(int j = i + 1; j < points.size(); ++j)
        {
            if(length(points[i], points[j]) < 22)
            {
                Robot2D robot;
                if(points[i].x < points[j].x)
                {
                    robot.points[0] = points[j];
                    robot.points[1] = points[i];
                }
                else
                {
                    robot.points[0] = points[i];
                    robot.points[1] = points[j];
                }
                double angle = atan2(robot.points[0].y - robot.points[1].y,
                                     robot.points[0].x - robot.points[1].x);
                robot.center = center(robot.points[0], robot.points[1]);
                robot.angle = angle + M_PI / 2.;
                if(start)
                {
                    robotNum++;
                    robot.number = robotNum;
                }
                else
                {
                    double minDist = 10000;
                    int nearest = -1;
                    for(int k = 0; k < robotsPrev.size(); ++k)
                    {
                        double dist = length(robot.center,
                                             robotsPrev[k].center);
                        if(dist < minDist)
                        {
                            minDist = dist;
                            nearest = k;
                        }
                    }
                    robot.number = robotsPrev[nearest].number;

                    //Angle

                    bool dontCare = false;
                    double angle;

                    if((robotsPrev[nearest].angle > (3.75 * M_PI / 2.) ||
                        (robotsPrev[nearest].angle < M_PI / 4.)) &&
                            (fabs(robotsPrev[nearest].angle - robot.angle) > (0.3 * M_PI)))
                    {
                        if(robotsPrev[nearest].angle > (3.75 * M_PI / 2.))
                        {
                            if(robot.angle < (M_PI / 4.))
                            {
                                angle = robot.angle;
                                dontCare = true;
                            }
                        }
                        else
                        {
                            if(robot.angle > (M_PI / 2.))
                            {
                                angle = robot.angle + M_PI;
                                dontCare = true;
                            }
                        }
                    }
                    if(!dontCare)
                    {
                        if(fabs(robot.angle - robotsPrev[nearest].angle) <
                                fabs((robot.angle + M_PI * 0.9) - robotsPrev[nearest].angle))
                        {
                            angle = robot.angle;
                        }
                        else
                        {
                            angle = robot.angle + M_PI;
                        }
                    }
                    robot.angle = angle;
                }
                robotVector.append(robot);
            }
        }
    }
    if(!start)
    {
        if(robotVector.size() != robotsStart.size())
        {
            return;
        }
    }
}

void RobotFinder::findMovedRobot(const RobotVector &start,
                                       const RobotVector &current,
                                       int &movedStart,
                                       int &movedCurrent)
{
    assert(start.size() == current.size());
    QVector<double> minDists;
    QVector<int> currentIndex;
    minDists.resize(start.size());
    currentIndex.resize(start.size());
    for(int i = 0; i < start.size(); ++i)
    {
        minDists[i] = 10000;
        for(int j = 0; j < current.size(); ++j)
        {
            double dist = length(start[i].center, current[j].center);
            //qDebug() << dist;
            if(dist < minDists[i])
            {
                currentIndex[i] = j;
                minDists[i] = dist;
            }
        }
    }
    //qDebug() << minDists[0] << minDists[1];
    double maxDist = -1;
    for(int i = 0; i < minDists.size(); ++i)
    {
        if(minDists[i] > maxDist)
        {
            maxDist = minDists[i];
            movedStart = i;
        }
    }
    movedCurrent = currentIndex[movedStart];
}

double RobotFinder::length(const Point2D &p1, const Point2D &p2)
{
    return sqrt(pow(p1.x - p2.x, 2) +
                pow(p1.y - p2.y, 2));
}

Point2D RobotFinder::center(Point2D &p1, Point2D &p2)
{
    Point2D p;
    p.x = (p1.x + p2.x) / 2;
    p.y = (p1.y + p2.y) / 2;
    return p;
}

Robot2D &RobotFinder::getRobotByNumber(RobotVector &vector, int number)
{
    for(int i = 0; i < vector.size(); ++i)
    {
        if(vector[i].number == number) return vector[i];
    }

    qDebug() << "ERROR!!!" << number << vector[0].number << vector[1].number;
    return vector[0];
}
