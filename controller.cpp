#include "controller.h"
#include "robotfinder.h"
#include <neuralnet.h>
#include <QDebug>

#define GOAL_ANGLE        0
#define GOAL_DISTANCE     1
#define OBSTACLE_ANGLE    2
#define OBSTACLE_DISTANCE 3
#define OUTPUT            4

enum Directions
{
    STOP          = 0,
    FORWARD       = 1,
    FORWARD_LIGHT = 2,
    LEFTER        = 3,
    RIGHTER       = 4,
    LEFT          = 5,
    RIGHT         = 6,
    BACKWARD      = 7
};

typedef vector<double> DoubleVector;

Controller::Controller(QObject *parent) : QObject(parent),
    goal_{0, 0},
    robotToControl_(1),
    net_(nullptr)
{
    engine_ = new Engine("Fuzzy");
    goalDistance_ = new InputVariable("GoalDistance");
    goalAngle_ = new InputVariable("GoalAngle");
    obstacleDistance_ = new InputVariable("ObstacleDistance");
    obstacleAngle_ = new InputVariable("ObstacleAngle");
    goalDistance_->setEnabled(true);
    goalAngle_->setEnabled(true);
    obstacleDistance_->setEnabled(true);
    obstacleAngle_->setEnabled(true);
    goalDistance_->setRange(0, 800);
    obstacleDistance_->setRange(0, 800);
    goalAngle_->setRange(-M_PI, M_PI);
    obstacleAngle_->setRange(-M_PI, M_PI);
    goalDistance_->addTerm(new Ramp("LOW", 55, 20));
    goalDistance_->addTerm(new Ramp("HIGH", 20, 55));
    obstacleDistance_->addTerm(new Ramp("LOW", 170, 120));
    obstacleDistance_->addTerm(new Ramp("HIGH", 120, 170));
    goalAngle_->addTerm(new Ramp("RIGHT_BEHIND", radians(120), radians(160)));
    goalAngle_->addTerm(new Trapezoid("RIGHT", 0, radians(40),
                                       radians(120), radians(160)));
    goalAngle_->addTerm(new Triangle("CENTER", radians(-40), 0, radians(40)));
    goalAngle_->addTerm(new Trapezoid("LEFT", radians(-160),
                                       radians(-120), radians(-40), 0));
    goalAngle_->addTerm(new Ramp("LEFT_BEHIND", radians(-120), radians(-160)));
    obstacleAngle_->addTerm(new Ramp("RIGHT", radians(0), radians(5)));
    obstacleAngle_->addTerm(new Triangle("CENTER", radians(-5), 0, radians(5)));
    obstacleAngle_->addTerm(new Ramp("LEFT", radians(0), radians(-5)));
    engine_->addInputVariable(obstacleAngle_);
    engine_->addInputVariable(obstacleDistance_);
    engine_->addInputVariable(goalAngle_);
    engine_->addInputVariable(goalDistance_);

    leftSpeed_ = new OutputVariable("LeftSpeed");
    rightSpeed_ = new OutputVariable("RightSpeed");
    leftSpeed_->setEnabled(true);
    rightSpeed_->setEnabled(true);
    leftSpeed_->setRange(-50, 100);
    rightSpeed_->setRange(-50, 100);

    leftSpeed_->addTerm(new Ramp("BACK", 0, -25));
    leftSpeed_->addTerm(new Triangle("LOW",-25, 0, 25));
    leftSpeed_->addTerm(new Triangle("MEDIUM", 25, 50, 75));
    leftSpeed_->addTerm(new Ramp("HIGH", 50, 75));
    rightSpeed_->addTerm(new Ramp("BACK", 0, -25));
    rightSpeed_->addTerm(new Triangle("LOW",-25, 0, 25));
    rightSpeed_->addTerm(new Triangle("MEDIUM", 25, 50, 75));
    rightSpeed_->addTerm(new Ramp("HIGH", 50, 75));
    engine_->addOutputVariable(leftSpeed_);
    engine_->addOutputVariable(rightSpeed_);

    ruleBlock_ = new RuleBlock;
    ruleBlock_->addRule(Rule::parse("if GoalDistance is LOW then "
                                     "LeftSpeed is LOW and RightSpeed is LOW",
                                     engine_));
    ruleBlock_->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is HIGH and "
                                     "GoalAngle is CENTER then "
                                     "LeftSpeed is HIGH and "
                                     "RightSpeed is HIGH", engine_));
    ruleBlock_->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is any and "
                                     "GoalAngle is LEFT_BEHIND then "
                                     "LeftSpeed is BACK and "
                                     "RightSpeed is MEDIUM", engine_));
    ruleBlock_->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is any and "
                                     "GoalAngle is RIGHT_BEHIND then "
                                     "LeftSpeed is MEDIUM and "
                                     "RightSpeed is BACK", engine_));
    ruleBlock_->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is HIGH and "
                                     "GoalAngle is LEFT then "
                                     "LeftSpeed is MEDIUM and "
                                     "RightSpeed is HIGH", engine_));
    ruleBlock_->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is HIGH and "
                                     "GoalAngle is RIGHT then "
                                     "LeftSpeed is HIGH and "
                                     "RightSpeed is MEDIUM", engine_));

    ruleBlock_->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is LOW and "
                                     "ObstacleAngle is CENTER and "
                                     "GoalAngle is CENTER then "
                                     "LeftSpeed is MEDIUM and "
                                     "RightSpeed is HIGH", engine_));
    ruleBlock_->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is LOW and "
                                     "ObstacleAngle is RIGHT and "
                                     "GoalAngle is CENTER then "
                                     "LeftSpeed is MEDIUM and "
                                     "RightSpeed is HIGH", engine_));
    ruleBlock_->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is LOW and "
                                     "ObstacleAngle is LEFT and "
                                     "GoalAngle is CENTER then "
                                     "LeftSpeed is HIGH and "
                                     "RightSpeed is MEDIUM", engine_));

    ruleBlock_->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is LOW and "
                                     "ObstacleAngle is CENTER and "
                                     "GoalAngle is LEFT then "
                                     "LeftSpeed is MEDIUM and "
                                     "RightSpeed is HIGH", engine_));
    ruleBlock_->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is LOW and "
                                     "ObstacleAngle is RIGHT and "
                                     "GoalAngle is LEFT then "
                                     "LeftSpeed is MEDIUM and "
                                     "RightSpeed is HIGH", engine_));
    ruleBlock_->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is LOW and "
                                     "ObstacleAngle is LEFT and "
                                     "GoalAngle is LEFT then "
                                     "LeftSpeed is HIGH and "
                                     "RightSpeed is MEDIUM", engine_));

    ruleBlock_->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is LOW and "
                                     "ObstacleAngle is CENTER and "
                                     "GoalAngle is RIGHT then "
                                     "LeftSpeed is HIGH and "
                                     "RightSpeed is MEDIUM", engine_));
    ruleBlock_->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is LOW and "
                                     "ObstacleAngle is RIGHT and "
                                     "GoalAngle is RIGHT then "
                                     "LeftSpeed is MEDIUM and "
                                     "RightSpeed is HIGH", engine_));
    ruleBlock_->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is LOW and "
                                     "ObstacleAngle is LEFT and "
                                     "GoalAngle is RIGHT then "
                                     "LeftSpeed is HIGH and "
                                     "RightSpeed is MEDIUM", engine_));

    engine_->addRuleBlock(ruleBlock_);
    engine_->configure("Minimum", "Maximum", "Minimum", "Maximum", "Centroid");
    std::string status;
    if(!engine_->isReady(&status))
    {
        qDebug() << QString::fromStdString("The following errors "
                                           "were encountered:\n" + status);
    }
    else
    {
        qDebug() << "Engine is Ready...";
    }

    net_ = new NeuralNet;
    net_->loadNet("../RobotControl/signX6.net");
}

Controller::~Controller()
{
    if(net_)
    {
        delete net_;
    }
}

void Controller::controlAction()
{
    if((goal_.x == 0) && (goal_.y == 0)) return;
    Robot2D &robot = RobotFinder::getRobotByNumber(robots_, robotToControl_);

    //dumbController(robot);
    //fuzzyController(robot);
    neuralNetController(robot);
}

void Controller::dumbController(const Robot2D &robot)
{
    RobotData data;
    RobotDataVector vec;
    data.number = robotToControl_;
    double desiredAngle = atan2(robot.center.y - goal_.y,
            robot.center.x - goal_.x);
    desiredAngle += M_PI;

    if(sqrt(pow(robot.center.y - goal_.y, 2) +
            pow(robot.center.x - goal_.x, 2)) < 25)
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
    Robot2D &obstacle = RobotFinder::getRobotByNumber(robots_, 2 - (robotToControl_ - 1));
    double goalAngle = angleToPoint(robot, goal_);
    double obstacleAngle = angleToPoint(robot, obstacle.center);
    double goalDistance = RobotFinder::length(robot.center, goal_);
    double obstacleDistance = RobotFinder::length(robot.center, obstacle.center);

    ControlData data;
    data.goalAngle = goalAngle;
    data.goalDistance = goalDistance;
    data.obstacleAngle = obstacleAngle;
    data.obstacleDistance = obstacleDistance;

    emit sendControlData(data);



//    m_goalDistance->setInputValue(goalDistance);
//    m_goalAngle->setInputValue(goalAngle);
//    m_obstacleDistance->setInputValue(obstacleDistance);
//    m_obstacleAngle->setInputValue(obstacleAngle);
//    m_engine->process();

//    QString debug;

//    debug += "Obstacle Distance: " + QString::fromStdString(m_obstacleDistance->fuzzyInputValue()) + '\n';
//    debug += "Goal Distance: " + QString::fromStdString(m_goalDistance->fuzzyInputValue()) + '\n';
//    debug += "Obstacle Angle: " + QString::fromStdString(m_obstacleAngle->fuzzyInputValue()) + " " +
//            QString::number(obstacleAngle) + '\n';
//    debug += "Goal Angle: " + QString::fromStdString(m_goalAngle->fuzzyInputValue()) + " " +
//            QString::number(goalAngle) + '\n';
    
//    double leftSpeed = m_leftSpeed->getOutputValue();
//    double rightSpeed = m_rightSpeed->getOutputValue();

//    debug += "Left Speed: " + QString::fromStdString(m_leftSpeed->fuzzyOutputValue()) + " "
//            + QString::number(leftSpeed) + '\n';
//    debug += "Right Speed: " + QString::fromStdString(m_rightSpeed->fuzzyOutputValue()) + " " +
//            QString::number(rightSpeed) + '\n';

//    qDebug().noquote() << debug;

//    if(obstacleDistance < 120)
//    {
//        qDebug() << "TOO CLOSE!!!";
//        if(goalDistance < 30)
//        {
//            rightSpeed = 0;
//            leftSpeed = 0;
//        }
//        else if(obstacleAngle > 0 && obstacleAngle < radians(35))
//        {
//            rightSpeed = 0;
//            leftSpeed = -30;
//        }
//        else if(obstacleAngle <= 0 && obstacleAngle > radians(-35))
//        {
//            rightSpeed = -30;
//            leftSpeed = 0;
//        }
//        else
//        {
//            rightSpeed = 70;
//            leftSpeed = 70;
//        }
//    }

//    //qDebug() << degrees(goalAngle) << degrees(obstacleAngle) << leftSpeed << rightSpeed;

//    RobotData data;
//    RobotDataVector vec;
//    data.number = m_robotToControl;

//    data.cByte = _BV(G);

//    if((leftSpeed > 50) && (leftSpeed <= 75))
//    {
//        data.cByte |= _BV(VL0) | _BV(DL);
//    }
//    else if(leftSpeed > 75)
//    {
//        data.cByte |= _BV(VL1) | _BV(DL);
//    }
//    else if(leftSpeed < -10)
//    {
//        data.cByte |= _BV(VL0);
//    }

//    if((rightSpeed > 50) && (rightSpeed <= 75))
//    {
//        data.cByte |= _BV(VR0) | _BV(DR);
//    }
//    else if(rightSpeed > 75)
//    {
//        data.cByte |= _BV(VR1) | _BV(DR);
//    }
//    else if(rightSpeed < -10)
//    {
//        data.cByte |= _BV(VR0);
//    }

//    vec << data;
//    emit sendRobotData(vec);
}

void Controller::neuralNetController(const Robot2D &robot)
{
    Robot2D &obstacle = RobotFinder::getRobotByNumber(robots_, 2 - (robotToControl_ - 1));
    double goalAngle = angleToPoint(robot, goal_);
    double obstacleAngle = angleToPoint(robot, obstacle.center);
    double goalDistance = RobotFinder::length(robot.center, goal_);
    double obstacleDistance = RobotFinder::length(robot.center, obstacle.center);

    DoubleVector input;
    DoubleVector output;
    input.resize(4);
    output.resize(8);

    input[GOAL_ANGLE] = goalAngle / M_PI;
    input[GOAL_DISTANCE] = goalDistance / 600.;
    input[OBSTACLE_ANGLE] = obstacleAngle / M_PI;
    input[OBSTACLE_DISTANCE] = obstacleDistance / 600.;

//    input[GOAL_ANGLE] = -0.705985;
//    input[GOAL_DISTANCE] = 0.140099;
//    input[OBSTACLE_ANGLE] = -0.751981;
//    input[OBSTACLE_DISTANCE] = 0.620716;

    qDebug() << input[GOAL_ANGLE] << input[GOAL_DISTANCE] << input[OBSTACLE_ANGLE] <<
                input[OBSTACLE_DISTANCE];

    RobotData data;
    RobotDataVector vec;
    data.number = robotToControl_;

    if(goalDistance < 25)
    {
        data.cByte = STOP_BYTE;
    }
    else
    {
        net_->feedForward(input);
        net_->getResults(output);

        qDebug() << output[0] << output[1] << output[2] << output[3] << output[4]
                << output[5] << output[6] << output[7];

        vector<double>::iterator largest = max_element(output.begin(),
                                                       output.end());
        int j = distance(output.begin(), largest);
        switch (j) {
        case STOP:
            qDebug() << "STOP";
            data.cByte = STOP_BYTE;
            break;

        case FORWARD:
            qDebug() << "FORWARD";
            data.cByte = FORWARD_BYTE;
            break;

        case FORWARD_LIGHT:
            qDebug() << "FORWARD_LIGHT";
            data.cByte = FORWARD_LIGHT_BYTE;
            break;

        case LEFT:
            qDebug() << "LEFT";
            data.cByte = LEFT_BYTE;
            break;

        case RIGHT:
            qDebug() << "RIGHT";
            data.cByte = RIGHT_BYTE;
            break;

        case LEFTER:
            qDebug() << "LEFTER";
            data.cByte = LEFTER_BYTE;
            break;

        case RIGHTER:
            qDebug() << "RIGHTER";
            data.cByte = RIGHTER_BYTE;
            break;

        case BACKWARD:
            qDebug() << "BACKWARD";
            data.cByte = BACKWARD_BYTE;
            break;
        default:
            break;
        }
    }

    vec << data;
    emit sendRobotData(vec);
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
