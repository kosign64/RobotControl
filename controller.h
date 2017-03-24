#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QObject>
#include <fl/Headers.h>
#include "defines.h"

using namespace fl;

class NeuralNet;

/*
 * Angle > 0 - to the right
 * Angle < 0 - to the left
 *
 */

enum ControllerType
{
    DUMB,
    FUZZY,
    NEURAL_NET
};

struct ControlData
{
    double goalAngle;
    double obstacleAngle;
    double goalDistance;
    double obstacleDistance;
};

class Controller : public QObject
{
    Q_OBJECT
public:
    explicit Controller(QObject *parent = 0);
    ~Controller();
    static double angleToPoint(const Robot2D &robot, const Point2D &point);

private:
    Point2D m_goal;
    RobotVector m_robots;
    int m_robotToControl;

    // Fuzzy Controller
    Engine *m_engine;
    InputVariable *m_goalDistance;
    InputVariable *m_obstacleDistance;
    InputVariable *m_goalAngle;
    InputVariable *m_obstacleAngle;
    OutputVariable *m_leftSpeed;
    OutputVariable *m_rightSpeed;
    RuleBlock *m_ruleBlock;

    // Neural net Controller
    NeuralNet *m_net;

    void controlAction();
    void dumbController(const Robot2D &robot);
    void fuzzyController(const Robot2D &robot);
    void neuralNetController(const Robot2D &robot);
    static double radians(double degrees) {return degrees * M_PI / 180.;}
    static double degrees(double radians) {return radians * 180. / M_PI;}

signals:
    void sendRobotData(RobotDataVector data);
    void sendControlData(ControlData data);

public slots:
    void getGoal(Point2D goal) {m_goal = goal; controlAction();}
    void getRobots(RobotVector robots) {m_robots = robots; controlAction();}
};

#endif // CONTROLLER_H
