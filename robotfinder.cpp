#include "robotfinder.h"
#include <cmath>
#include <cassert>
#include <iostream>
#include <chrono>
#include <QDebug>

using namespace std::chrono;

RobotFinder::RobotFinder(QObject *parent) : QObject(parent),
    start_(true),
    state_(RUN_ROBOT)
{

}

void RobotFinder::setPoints(PointVector &pointVector)
{
    RobotVector robots;
    if(!findRobotsFromPoints(robots, pointVector)) return;
    if(robots.isEmpty()) return;

    // because robot numbers start from 1
    static int numberOfRobots = robots.size() + 1;
    static int checkingRobotNumber = 1;
    // Initialization
    // Check robots' numbers and directions
    if(checkingRobotNumber < numberOfRobots)
    {
        RobotData data;
        RobotDataVector vec;
        static auto begin = high_resolution_clock::now();
        high_resolution_clock::time_point end;
        double time;
        switch (state_)
        {
        case RUN_ROBOT:
            if(start_)
            {
                robotsStart_ = robots;
            }
            data.number = checkingRobotNumber;
            data.cByte = FORWARD_BYTE;
            vec.append(data);
            emit sendRobotData(vec);
            end = high_resolution_clock::now();
            time = duration_cast<milliseconds>(end - begin).count();
            // Drive robot for at least 400 ms
            if(time > 400)
            {
                begin = high_resolution_clock::now();
                state_ = WAIT;
            }
            break;

        case WAIT:
            data.number = checkingRobotNumber;
            data.cByte = STOP_BYTE;
            vec.append(data);
            emit sendRobotData(vec);
            end = high_resolution_clock::now();
            time = duration_cast<milliseconds>(end - begin).count();
            if(time > 600)
            {
                state_ = CHECK_ROBOT;
            }
            break;

        case CHECK_ROBOT:
            int movedStart, movedCurrent;
            findMovedRobot(robotsStart_, robots,
                           movedStart, movedCurrent);
            Robot2D &startRob = robotsStart_[movedStart];
            Robot2D &currentRob = robots[movedCurrent];
            // Robot, that already has checkingRobotNumber
            Robot2D &otherRob = getRobotByNumber(robots,
                                              checkingRobotNumber);
            otherRob.number = currentRob.number;
            currentRob.number = checkingRobotNumber;
            Point2D estimated1;
            Point2D estimated2;
            // Find robot direction
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
            robotsStart_ = robots;
            checkingRobotNumber++;
            state_ = RUN_ROBOT;
            begin = high_resolution_clock::now();
            break;
        }
    }
    else
    {
        emit sendRobots(robots);
    }
    robotsPrev_ = robots;
    start_ = false;
}

bool RobotFinder::findRobotsFromPoints(RobotVector &robotVector, const PointVector &pointVector) const
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
                break;
            }
        }
        if(newPoint)
        {
            points.append(pointsRaw[i]);
        }
    }
    if(!pointsRaw.empty())
    {
        points.append(pointsRaw.back());
    }

    // Find Robots
    robotVector.clear();
    int robotNum = 0;
    for(int i = 0; i < (points.size() - 1); ++i)
    {
        for(int j = i + 1; j < points.size(); ++j)
        {
            // Check if two points belong to one robot
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
                robot.angle = angle + M_PI_2;
                if(start_)
                {
                    robotNum++;
                    robot.number = robotNum;
                }
                else
                {
                    double minDist = 10000;
                    int nearest;
                    for(int k = 0; k < robotsPrev_.size(); ++k)
                    {
                        double dist = length(robot.center,
                                             robotsPrev_[k].center);
                        if(dist < minDist)
                        {
                            minDist = dist;
                            nearest = k;
                        }
                    }
                    robot.number = robotsPrev_[nearest].number;

                    //Angle
                    bool needFix = true;
                    double angle;

                    if((robotsPrev_[nearest].angle > (3.75 * M_PI_2) ||
                        (robotsPrev_[nearest].angle < M_PI_4)) &&
                            (fabs(robotsPrev_[nearest].angle - robot.angle) > (0.3 * M_PI)))
                    {
                        if(robotsPrev_[nearest].angle > (3.75 * M_PI_2))
                        {
                            if(robot.angle < (M_PI_4))
                            {
                                angle = robot.angle;
                                needFix = false;
                            }
                        }
                        else
                        {
                            if(robot.angle > (M_PI_2))
                            {
                                angle = robot.angle + M_PI;
                                needFix = false;
                            }
                        }
                    }
                    if(needFix)
                    {
                        if(fabs(robot.angle - robotsPrev_[nearest].angle) <
                                fabs((robot.angle + M_PI * 0.9) - robotsPrev_[nearest].angle))
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
    if(!start_)
    {
        if(robotVector.size() != robotsStart_.size())
        {
            return false;
        }
    }

    return true;
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
            if(dist < minDists[i])
            {
                currentIndex[i] = j;
                minDists[i] = dist;
            }
        }
    }
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

Point2D RobotFinder::center(const Point2D &p1, const Point2D &p2)
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

    std::cerr << "Error: Can't get robot by ID" << std::endl;
    exit(-1);
}
