#include "mainwindow.h"
#include "socket.h"
#include <QLayout>
#include <QPushButton>
#include <QKeyEvent>
#include <QLineEdit>
#include "playfield.h"
#include "robotfinder.h"
#include "controller.h"
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
    controller = new Controller(this);

    ipAddressEdit->setInputMask("000.000.000.000");
    ipAddressEdit->setText("10.139.0.104");
    ipAddressEdit->setAlignment(Qt::AlignCenter);

    connect(socket, &Socket::sendPoints, robotFinder,
            &RobotFinder::getPoints);
    connect(connectButton, &QPushButton::clicked, this,
            &MainWindow::onConnectClick);
    connect(robotFinder, &RobotFinder::sendRobots,
            field, &PlayField::getRobots);
    connect(robotFinder, &RobotFinder::sendRobotData,
            socket, &Socket::getRobotData);
    connect(this, &MainWindow::sendWheels, socket,
            &Socket::getRobotData);
    connect(field, &PlayField::sendGoal, controller,
            &Controller::getGoal);
    connect(robotFinder, &RobotFinder::sendRobots, controller,
            &Controller::getRobots);
    connect(controller, &Controller::sendRobotData,
            socket, &Socket::getRobotData);

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
        data.cByte = FORWARD_BYTE;
    }
    else if(leftPressed)
    {
        data.cByte = LEFT_BYTE;
    }
    else if(rightPressed)
    {
        data.cByte = RIGHT_BYTE;
    }
    else if(downPressed)
    {
        data.cByte = BACKWARD_BYTE;
    }
    else
    {
        data.cByte = STOP_BYTE;
    }

    vec.append(data);

    emit sendWheels(vec);
}

void MainWindow::keyReleaseEvent(QKeyEvent *ev)
{
    RobotData data;
    data.number = robotNumber;
    RobotDataVector vec;
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
        data.cByte = FORWARD_BYTE;
    }
    else if(leftPressed)
    {
        data.cByte = LEFT_BYTE;
    }
    else if(rightPressed)
    {
        data.cByte = RIGHT_BYTE;
    }
    else if(downPressed)
    {
        data.cByte = BACKWARD_BYTE;
    }
    else
    {
        data.cByte = STOP_BYTE;
    }

    vec.append(data);

    emit sendWheels(vec);
}
