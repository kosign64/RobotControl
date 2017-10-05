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

enum Directions
{
    STOP = 0,
    FORWARD = 1,
    FORWARD_LIGHT = 2,
    LEFTER = 3,
    RIGHTER = 4,
    LEFT = 5,
    RIGHT = 6,
    BACKWARD = 7
};

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      upPressed_(false),
      upLightPressed_(false),
      downPressed_(false),
      leftPressed_(false),
      rightPressed_(false),
      start_(true),
      saveData_(false),
      robotNumber_(1)
{
    qRegisterMetaType<PointVector>("PointVector");
    mainWidget_ = new QWidget(this);
    mainLayout_ = new QVBoxLayout;
    ipAddressEdit_ = new QLineEdit(mainWidget_);
    connectButton_ = new QPushButton("Connect");
    socket_ = new Socket(this);
    field_ = new PlayField(this);
    robotFinder_ = new RobotFinder(this);
    controller_ = new Controller(this);

    ipAddressEdit_->setInputMask("000.000.000.000");
    ipAddressEdit_->setText("10.139.0.104");
    ipAddressEdit_->setAlignment(Qt::AlignCenter);

    connect(socket_, &Socket::sendPoints, robotFinder_,
            &RobotFinder::getPoints);
    connect(connectButton_, &QPushButton::clicked, this,
            &MainWindow::onConnectClick);
    connect(robotFinder_, &RobotFinder::sendRobots,
            field_, &PlayField::getRobots);
    connect(robotFinder_, &RobotFinder::sendRobotData,
            socket_, &Socket::getRobotData);
    connect(this, &MainWindow::sendWheels, socket_,
            &Socket::getRobotData);
    connect(field_, &PlayField::sendGoal, controller_,
            &Controller::getGoal);
    connect(robotFinder_, &RobotFinder::sendRobots, controller_,
            &Controller::getRobots);
    connect(controller_, &Controller::sendRobotData,
            socket_, &Socket::getRobotData);
    connect(this, &MainWindow::sendRobotNumber, field_,
            &PlayField::getRobotNumber);
    connect(controller_, &Controller::sendControlData,
            this, &MainWindow::getControlData);

    mainLayout_->addWidget(ipAddressEdit_);
    mainLayout_->addWidget(connectButton_);
    mainLayout_->addWidget(field_);
    mainWidget_->setLayout(mainLayout_);
    setCentralWidget(mainWidget_);
    setFocus();

    filename_ = "trainData.data";
    file_.setFileName(filename_);
    if(!file_.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        exit(-1);
    }
    fileStream_.setDevice(&file_);
}

MainWindow::~MainWindow()
{
    file_.close();
}

//=============================================================================

void MainWindow::onConnectClick()
{
    QString address = ipAddressEdit_->text();
    if(socket_->connectToHost(address))
    {
        connectButton_->setEnabled(false);
        setFocus();
    }
}

void MainWindow::keyPressEvent(QKeyEvent *ev)
{
    RobotData data;
    RobotDataVector vec;
    switch(ev->key())
    {
    case Qt::Key_W:
        upPressed_ = true;
        break;

    case Qt::Key_E:
        upLightPressed_ = true;
        break;

    case Qt::Key_A:
        leftPressed_ = true;
        break;

    case Qt::Key_D:
        rightPressed_ = true;
        break;

    case Qt::Key_S:
        downPressed_ = true;
        break;

    case Qt::Key_Escape:
        close();
        break;

    case Qt::Key_1:
        robotNumber_ = 1;
        break;

    case Qt::Key_2:
        robotNumber_ = 2;
        break;

    case Qt::Key_3:
        robotNumber_ = 3;
        break;

    case Qt::Key_N:
        saveData_ = !saveData_;
        break;

    case Qt::Key_P:
        field_->invertDrawPath();
        break;
    }
    if(upPressed_ && !leftPressed_ && !rightPressed_)
    {
        data.cByte = FORWARD_BYTE;
    }
    else if(upLightPressed_)
    {
        data.cByte = FORWARD_LIGHT_BYTE;
    }
    else if(upPressed_ && leftPressed_)
    {
        data.cByte = LEFTER_BYTE;
    }
    else if(upPressed_ && rightPressed_)
    {
        data.cByte = RIGHTER_BYTE;
    }
    else if(leftPressed_)
    {
        data.cByte = LEFT_BYTE;
    }
    else if(rightPressed_)
    {
        data.cByte = RIGHT_BYTE;
    }
    else if(downPressed_)
    {
        data.cByte = BACKWARD_BYTE;
    }
    else
    {
        data.cByte = STOP_BYTE;
    }
    data.number = robotNumber_;

    vec.append(data);

    emit sendRobotNumber(robotNumber_);
    emit sendWheels(vec);
}

void MainWindow::keyReleaseEvent(QKeyEvent *ev)
{
    RobotData data;
    data.number = robotNumber_;
    RobotDataVector vec;
    switch(ev->key())
    {
    case Qt::Key_W:
        upPressed_ = false;
        break;

    case Qt::Key_E:
        upLightPressed_ = false;
        break;

    case Qt::Key_A:
        leftPressed_ = false;
        break;

    case Qt::Key_D:
        rightPressed_ = false;
        break;

    case Qt::Key_S:
        downPressed_ = false;
        break;
    }
    if(upPressed_ && !leftPressed_ && !rightPressed_)
    {
        data.cByte = FORWARD_BYTE;
    }
    else if(upLightPressed_)
    {
        data.cByte = FORWARD_LIGHT_BYTE;
    }
    else if(upPressed_ && leftPressed_)
    {
        data.cByte = LEFTER_BYTE;
    }
    else if(upPressed_ && rightPressed_)
    {
        data.cByte = RIGHTER_BYTE;
    }
    else if(leftPressed_)
    {
        data.cByte = LEFT_BYTE;
    }
    else if(rightPressed_)
    {
        data.cByte = RIGHT_BYTE;
    }
    else if(downPressed_)
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

void MainWindow::getControlData(ControlData data)
{
    static QString prevString;
    QString saveString;
    QTextStream stringStream(&saveString);
    if(saveData_ && ((upPressed_ || leftPressed_ || rightPressed_ || downPressed_) ||
                    data.goalDistance < 20))
    {
        stringStream << data.goalAngle << ";" << data.goalDistance <<
                      ";" << data.obstacleAngle << ";" << data.obstacleDistance <<
                      ";";
        if(upPressed_ && !leftPressed_ && !rightPressed_)
        {
            stringStream << FORWARD;
        }
        else if(upLightPressed_)
        {
            stringStream << FORWARD_LIGHT;
        }
        else if(upPressed_ && leftPressed_)
        {
            stringStream << LEFTER;
        }
        else if(upPressed_ && rightPressed_)
        {
            stringStream << RIGHTER;
        }
        else if(leftPressed_)
        {
            stringStream << LEFT;
        }
        else if(rightPressed_)
        {
            stringStream << RIGHT;
        }
        else if(downPressed_)
        {
            stringStream << BACKWARD;
        }
        else
        {
            stringStream << STOP;
        }

        stringStream << endl;

        if(QString::compare(saveString, prevString) != 0)
        {
            qDebug() << saveString << prevString << "Save data";
            fileStream_ << saveString;
        }

        prevString = saveString;
    }
}
