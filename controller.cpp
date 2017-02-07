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
    m_goalAngle->setRange(-M_PI, M_PI);
    m_obstacleAngle->setRange(-M_PI, M_PI);
    m_goalDistance->addTerm(new Ramp("LOW", 55, 20));
    m_goalDistance->addTerm(new Ramp("HIGH", 20, 55));
    m_obstacleDistance->addTerm(new Ramp("LOW", 150, 100));
    m_obstacleDistance->addTerm(new Ramp("HIGH", 100, 150));
    m_goalAngle->addTerm(new Ramp("RIGHT_BEHIND", radians(120), radians(160)));
    m_goalAngle->addTerm(new Trapezoid("RIGHT", 0, radians(40),
                                       radians(120), radians(160)));
    m_goalAngle->addTerm(new Triangle("CENTER", radians(-40), 0, radians(40)));
    m_goalAngle->addTerm(new Trapezoid("LEFT", radians(-160),
                                       radians(-120), radians(-40), 0));
    m_goalAngle->addTerm(new Ramp("LEFT_BEHIND", radians(-120), radians(-160)));
    m_obstacleAngle->addTerm(new Ramp("RIGHT", radians(0), radians(20)));
    m_obstacleAngle->addTerm(new Triangle("CENTER", radians(-20), 0, radians(20)));
    m_obstacleAngle->addTerm(new Ramp("LEFT", radians(0), radians(-20)));
    m_engine->addInputVariable(m_obstacleAngle);
    m_engine->addInputVariable(m_obstacleDistance);
    m_engine->addInputVariable(m_goalAngle);
    m_engine->addInputVariable(m_goalDistance);

    m_leftSpeed = new OutputVariable("LeftSpeed");
    m_rightSpeed = new OutputVariable("RightSpeed");
    m_leftSpeed->setEnabled(true);
    m_rightSpeed->setEnabled(true);
    m_leftSpeed->setRange(-50, 100);
    m_rightSpeed->setRange(-50, 100);

    m_leftSpeed->addTerm(new Ramp("BACK", 0, -25));
    m_leftSpeed->addTerm(new Triangle("LOW",-25, 0, 25));
    m_leftSpeed->addTerm(new Triangle("MEDIUM", 25, 50, 75));
    m_leftSpeed->addTerm(new Ramp("HIGH", 50, 75));
    m_rightSpeed->addTerm(new Ramp("BACK", 0, -25));
    m_rightSpeed->addTerm(new Triangle("LOW",-25, 0, 25));
    m_rightSpeed->addTerm(new Triangle("MEDIUM", 25, 50, 75));
    m_rightSpeed->addTerm(new Ramp("HIGH", 50, 75));
    m_engine->addOutputVariable(m_leftSpeed);
    m_engine->addOutputVariable(m_rightSpeed);

    m_ruleBlock = new RuleBlock;
    m_ruleBlock->addRule(Rule::parse("if GoalDistance is LOW then "
                                     "LeftSpeed is LOW and RightSpeed is LOW",
                                     m_engine));
    m_ruleBlock->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is HIGH and "
                                     "GoalAngle is CENTER then "
                                     "LeftSpeed is HIGH and "
                                     "RightSpeed is HIGH", m_engine));
    m_ruleBlock->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is any and "
                                     "GoalAngle is LEFT_BEHIND then "
                                     "LeftSpeed is BACK and "
                                     "RightSpeed is MEDIUM", m_engine));
    m_ruleBlock->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is any and "
                                     "GoalAngle is RIGHT_BEHIND then "
                                     "LeftSpeed is MEDIUM and "
                                     "RightSpeed is BACK", m_engine));
    m_ruleBlock->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is HIGH and "
                                     "GoalAngle is LEFT then "
                                     "LeftSpeed is MEDIUM and "
                                     "RightSpeed is HIGH", m_engine));
    m_ruleBlock->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is HIGH and "
                                     "GoalAngle is RIGHT then "
                                     "LeftSpeed is HIGH and "
                                     "RightSpeed is MEDIUM", m_engine));

    m_ruleBlock->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is LOW and "
                                     "ObstacleAngle is CENTER and "
                                     "GoalAngle is CENTER then "
                                     "LeftSpeed is MEDIUM and "
                                     "RightSpeed is HIGH", m_engine));
    m_ruleBlock->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is LOW and "
                                     "ObstacleAngle is RIGHT and "
                                     "GoalAngle is CENTER then "
                                     "LeftSpeed is MEDIUM and "
                                     "RightSpeed is HIGH", m_engine));
    m_ruleBlock->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is LOW and "
                                     "ObstacleAngle is LEFT and "
                                     "GoalAngle is CENTER then "
                                     "LeftSpeed is HIGH and "
                                     "RightSpeed is MEDIUM", m_engine));

    m_ruleBlock->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is LOW and "
                                     "ObstacleAngle is CENTER and "
                                     "GoalAngle is LEFT then "
                                     "LeftSpeed is MEDIUM and "
                                     "RightSpeed is HIGH", m_engine));
    m_ruleBlock->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is LOW and "
                                     "ObstacleAngle is RIGHT and "
                                     "GoalAngle is LEFT then "
                                     "LeftSpeed is MEDIUM and "
                                     "RightSpeed is HIGH", m_engine));
    m_ruleBlock->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is LOW and "
                                     "ObstacleAngle is LEFT and "
                                     "GoalAngle is LEFT then "
                                     "LeftSpeed is HIGH and "
                                     "RightSpeed is MEDIUM", m_engine));

    m_ruleBlock->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is LOW and "
                                     "ObstacleAngle is CENTER and "
                                     "GoalAngle is RIGHT then "
                                     "LeftSpeed is HIGH and "
                                     "RightSpeed is MEDIUM", m_engine));
    m_ruleBlock->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is LOW and "
                                     "ObstacleAngle is RIGHT and "
                                     "GoalAngle is RIGHT then "
                                     "LeftSpeed is MEDIUM and "
                                     "RightSpeed is HIGH", m_engine));
    m_ruleBlock->addRule(Rule::parse("if GoalDistance is HIGH and "
                                     "ObstacleDistance is LOW and "
                                     "ObstacleAngle is LEFT and "
                                     "GoalAngle is RIGHT then "
                                     "LeftSpeed is HIGH and "
                                     "RightSpeed is MEDIUM", m_engine));

    m_engine->addRuleBlock(m_ruleBlock);
    m_engine->configure("Minimum", "Maximum", "Minimum", "Maximum", "Centroid");
    std::string status;
    if(!m_engine->isReady(&status))
    {
        qDebug() << QString::fromStdString("The following errors "
                                           "were encountered:\n" + status);
    }
    else
    {
        qDebug() << "Engine is Ready...";
    }
}

void Controller::controlAction()
{
    if((m_goal.x == 0) && (m_goal.y == 0)) return;
    Robot2D &robot = RobotFinder::getRobotByNumber(m_robots, m_robotToControl);

    //dumbController(robot);
    fuzzyController(robot);
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
    double goalDistance = RobotFinder::length(robot.center, m_goal);
    double obstacleDistance = RobotFinder::length(robot.center, obstacle.center);

    m_goalDistance->setInputValue(goalDistance);
    m_goalAngle->setInputValue(goalAngle);
    m_obstacleDistance->setInputValue(obstacleDistance);
    m_obstacleAngle->setInputValue(obstacleAngle);
    m_engine->process();

    qDebug() << "Obstacle Distance:" << QString::fromStdString(m_obstacleDistance->fuzzyInputValue());
    qDebug() << "Goal Distance:" << QString::fromStdString(m_goalDistance->fuzzyInputValue());
    qDebug() << "Obstacle Angle:" << QString::fromStdString(m_obstacleAngle->fuzzyInputValue()) << obstacleAngle;
    qDebug() << "Goal Angle:" <<  QString::fromStdString(m_goalAngle->fuzzyInputValue()) << goalAngle;

    double leftSpeed = m_leftSpeed->getOutputValue();
    double rightSpeed = m_rightSpeed->getOutputValue();

    qDebug() << degrees(goalAngle) << degrees(obstacleAngle) << leftSpeed << rightSpeed;

    RobotData data;
    RobotDataVector vec;
    data.number = m_robotToControl;

    data.cByte = _BV(G);

    if((leftSpeed > 50) && (leftSpeed <= 80))
    {
        data.cByte |= _BV(VL0) | _BV(DL);
    }
    else if(leftSpeed > 80)
    {
        data.cByte |= _BV(VL1) | _BV(DL);
    }
    else if(leftSpeed < -10)
    {
        data.cByte |= _BV(VL0);
    }

    if((rightSpeed > 50) && (rightSpeed <= 75))
    {
        data.cByte |= _BV(VR0) | _BV(DR);
    }
    else if(rightSpeed > 80)
    {
        data.cByte |= _BV(VR1) | _BV(DR);
    }
    else if(rightSpeed < -10)
    {
        data.cByte |= _BV(VR0);
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
