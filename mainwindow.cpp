#include "mainwindow.h"
#include "socket.h"
#include <QLayout>
#include <QPushButton>
#include <QKeyEvent>
#include <QLineEdit>
#include "playfield.h"
#include "robotfinder.h"
#include <QDebug>
#include <cmath>
#include <unistd.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      upPressed(false),
      downPressed(false),
      leftPressed(false),
      rightPressed(false),
      start(true),
      robotNumber(1)
{
    qRegisterMetaType<PointVector>("PointVector");
    mainWidget = new QWidget(this);
    mainLayout = new QVBoxLayout;
    ipAddressEdit = new QLineEdit(mainWidget);
    connectButton = new QPushButton("Connect");
    socket = new Socket(this);
    field = new PlayField(this);
    robotFinder = new RobotFinder(this);

    ipAddressEdit->setInputMask("000.000.000.000");
    ipAddressEdit->setText("10.139.0.104");
    ipAddressEdit->setAlignment(Qt::AlignCenter);

    connect(socket, &Socket::sendPoints, robotFinder, &RobotFinder::getPoints);
    connect(connectButton, &QPushButton::clicked, this, &MainWindow::onConnectClick);
//    connect(robotFinder, &RobotFinder::sendRobots,
//            this, &MainWindow::getRobots);
    connect(robotFinder, &RobotFinder::sendRobots,
            field, &PlayField::getRobots);
    connect(robotFinder, &RobotFinder::sendRobotData,
            socket, &Socket::getRobotData);
    connect(this, &MainWindow::sendWheels, socket,
            &Socket::getRobotData);

    mainLayout->addWidget(ipAddressEdit);
    mainLayout->addWidget(connectButton);
    mainLayout->addWidget(field);
    mainWidget->setLayout(mainLayout);
    setCentralWidget(mainWidget);
    setFocus();
}

MainWindow::~MainWindow()
{

}

//=============================================================================

void MainWindow::onConnectClick()
{
    QString address = ipAddressEdit->text();
    if(socket->connectToHost(address))
    {
        connectButton->setEnabled(false);
        setFocus();
    }
}

void MainWindow::getRobots(QVector<Robot2D> robs)
{
    // x: 100 - 700
    // y: 80  - 400
    robots = robs;
    static double prevAngle;
    static Point2D a[4];
    static int k = 0;
    a[0].x = 200;
    a[0].y = 100;
    a[1].x = 200;
    a[1].y = 400;
    a[2].x = 620;
    a[2].y = 400;
    a[3].x = 620;
    a[3].y = 100;
    if(start)
    {
        static int n = 0;
        if(robots.isEmpty()) return;
        static double angle = robots[0].angle;
        static int startX = robots[0].center.x;
        static int startY = robots[0].center.y;
        socket->setDir(FORWARD);
        if(++n > 20)
        {
            double currentX = robots[0].center.x;
            double currentY = robots[0].center.y;
            double estimatedX1 = startX + 30 * cos(angle);
            double estimatedY1 = startY + 30 * sin(angle);
            double estimatedX2 = startX + 30 * cos(angle + M_PI);
            double estimatedY2 = startY + 30 * sin(angle + M_PI);
            if(sqrt(pow(estimatedX1 - currentX, 2) + pow(estimatedY1 - currentY, 2)) <
               sqrt(pow(estimatedX2 - currentX, 2) + pow(estimatedY2 - currentY, 2)))
            {
                prevAngle = angle;
            }
            else
            {
                prevAngle = angle + M_PI;
            }
            start = false;
            socket->setDir(STOP);
            qDebug() << angle * 180. / M_PI << prevAngle * 180. / M_PI;
        }
    }
    else
    {
        static Dir dir = FORWARD;
        static bool wasHere = false;
        foreach(Robot2D robot, robots)
        {
            bool dontCare = false;
            //qDebug() << "Raw:" << robot.angle * 180. / M_PI;
            double angle;

            //if(prevAngle < (M_PI / 2.) && robot.angle >)

            //qDebug() << (prevAngle < M_PI / 4.) << prevAngle << robot.angle << (abs(prevAngle - robot.angle) > (0.7 * M_PI));

            if((prevAngle > (3.75 * M_PI / 2.) || (prevAngle < M_PI / 4.)) &&
                    (fabs(prevAngle - robot.angle) > (0.3 * M_PI)))
            {
                //qDebug() << "Done3";
                if(prevAngle > (3.75 * M_PI / 2.))
                {
                    if(robot.angle < (M_PI / 4.))
                    {
                        //qDebug() << "Done2!";
                        angle = robot.angle;
                        dontCare = true;
                    }
                }
                else
                {
                    if(robot.angle > (M_PI / 2.))
                    {
                        //qDebug() << "Done!";
                        angle = robot.angle + M_PI;
                        dontCare = true;
                    }
                }
            }
            if(!dontCare)
            {
                if(fabs(robot.angle - prevAngle) < fabs((robot.angle + M_PI * 0.9) - prevAngle))
                {
                    //qDebug() << robot.angle * 180. / M_PI << prevAngle * 180. / M_PI;
                    angle = robot.angle;
                }
                else
                {
                    //qDebug() << "Here" << robot.angle * 180. / M_PI << prevAngle * 180. / M_PI;
                    angle = robot.angle + M_PI;
                }
            }
            robot.angle = angle;
            qDebug() << "X:" << robot.center.x << "Y:" << robot.center.y <<
                        "Angle:" << robot.angle * 180. / M_PI <<
                        prevAngle * 180. / M_PI;
            prevAngle = angle;

            Point2D desiredPos = a[k];

            double desiredAngle = atan2(robot.center.y - desiredPos.y,
                    robot.center.x - desiredPos.x);
//            if(desiredAngle < 0)
//                desiredAngle += M_PI;
            desiredAngle += M_PI;
            qDebug() << "Desited Angle" << desiredAngle * 180. / M_PI;

            if(sqrt(pow(robot.center.y - desiredPos.y, 2) +
                    pow(robot.center.x - desiredPos.x, 2)) < 25)
            {
                socket->setDir(STOP);
                k++;
                if(k > 3) k = 0;
                desiredPos = a[k];
            }
            else
            {
                qDebug() << "Abs" << fabs(robot.angle - desiredAngle) * 180. / M_PI;
                if(((fabs(robot.angle - desiredAngle) * 180 / M_PI) < 15) ||
        ((fabs(robot.angle + 2 * M_PI - desiredAngle) * 180 / M_PI) < 15) ||
        ((fabs(robot.angle - (desiredAngle + 2 * M_PI)) * 180 / M_PI) < 15))
                {
                    qDebug() << "Forward";
                    socket->setDir(FORWARD);
                    //socket->setDir(STOP);
                }
                else if(robot.angle > desiredAngle)
                {
                    qDebug() << "Right";
                    if(fabs(robot.angle - desiredAngle) > M_PI)
                    {
                        socket->setDir(RIGHT);
                    }
                    else
                    {
                        socket->setDir(LEFT);
                    }
                }
                else
                {
                    qDebug() << "Left";
                    if(fabs(robot.angle - desiredAngle) > M_PI)
                    {
                        socket->setDir(LEFT);
                    }
                    else
                    {
                        socket->setDir(RIGHT);
                    }
                }
            }
        }
    }
}

void MainWindow::keyPressEvent(QKeyEvent *ev)
{
    RobotData data;
    data.number = robotNumber;
    RobotDataVector vec;
    switch(ev->key())
    {
    case Qt::Key_W:
        upPressed = true;
        break;

    case Qt::Key_A:
        leftPressed = true;
        break;

    case Qt::Key_D:
        rightPressed = true;
        break;

    case Qt::Key_S:
        downPressed = true;
        break;

    case Qt::Key_Escape:
        close();
        break;

    case Qt::Key_1:
        robotNumber = 1;
        break;

    case Qt::Key_2:
        robotNumber = 2;
        break;
    }
    if(upPressed)
    {
        //socket->setDir(FORWARD);
        data.cByte = _BV(G) | _BV(DL) | _BV(VL1) | _BV(DR) | _BV(VR1);
    }
    else if(leftPressed)
    {
        data.cByte = _BV(G) | _BV(VL0) | _BV(DL) /*| _BV(VR0)*/;
    }
    else if(rightPressed)
    {
        data.cByte = _BV(G) | _BV(DR) /*| _BV(VL0)*/ | _BV(VR0);
    }
    else if(downPressed)
    {
        data.cByte = _BV(G) | _BV(VL1) | _BV(VR1);
    }
    else
    {
        //socket->setDir(STOP);
        data.cByte = 64;
    }

    vec.append(data);

    emit sendWheels(vec);
}

void MainWindow::keyReleaseEvent(QKeyEvent *ev)
{
    switch(ev->key())
    {
    case Qt::Key_W:
        upPressed = false;
        break;

    case Qt::Key_A:
        leftPressed = false;
        break;

    case Qt::Key_D:
        rightPressed = false;
        break;

    case Qt::Key_S:
        downPressed = false;
        break;
    }
    if(upPressed)
    {
        socket->setDir(FORWARD);
    }
    else if(leftPressed)
    {
        socket->setDir(LEFT);
    }
    else if(rightPressed)
    {
        socket->setDir(RIGHT);
    }
    else if(downPressed)
    {
        socket->setDir(BACKWARD);
    }
    else
    {
        socket->setDir(STOP);
    }
}
