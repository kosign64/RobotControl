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
    Point2D goal_;
    RobotVector robots_;
    int robotToControl_;

    // Fuzzy Controller
    Engine *engine_;
    InputVariable *goalDistance_;
    InputVariable *obstacleDistance_;
    InputVariable *goalAngle_;
    InputVariable *obstacleAngle_;
    OutputVariable *leftSpeed_;
    OutputVariable *rightSpeed_;
    RuleBlock *ruleBlock_;

    // Neural net Controller
    NeuralNet *net_;

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
    void setGoal(Point2D goal) {goal_ = goal; controlAction();}
    void setRobots(RobotVector robots) {robots_ = robots; controlAction();}
};

#endif // CONTROLLER_H
