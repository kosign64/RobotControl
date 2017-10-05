#include "socket.h"
#include "defines.h"
#include <QTcpSocket>

Socket::Socket(QObject *parent) : QObject(parent),
    connected_(false),
    pointSize_(255)
{
    socket_ = new QTcpSocket(this);
    RobotData data;
    data.number = 0;
    data.cByte = _BV(G);
    robotVector_.append(data);
    connect(socket_, &QTcpSocket::readyRead, this, &Socket::readyRead);
    connect(socket_, &QTcpSocket::disconnected, this,
            &Socket::disconnected);
    connect(socket_, SIGNAL(error(QAbstractSocket::SocketError)), this,
            SLOT(getError(QAbstractSocket::SocketError)));
}

Socket::~Socket()
{

}

//=============================================================================

void Socket::readyRead()
{
    if((size_t)socket_->bytesAvailable() >= sizeof(uint16_t))
    {
        socket_->read((char*)&pointSize_, sizeof(uint16_t));
    }
    if((size_t)socket_->bytesAvailable() >= ((sizeof(Point2D)) * pointSize_))
    {
        points_.resize(pointSize_);
        for(int i = 0; i < pointSize_; ++i)
        {
            Point2D point;
            socket_->read((char*)&point, sizeof(point));
            points_[i] = point;
        }

        sendWheels();
        pointSize_ = 255;
        emit sendPoints(points_);
    }
}

void Socket::disconnected()
{
    qDebug() << "Disconnected";
}

void Socket::getError(QAbstractSocket::SocketError error)
{
    qDebug() << "Error:" << error;
}

//=============================================================================

void Socket::sendWheels()
{
    for(int i = 0; i < robotVector_.size(); ++i)
    {
        RobotData &data = robotVector_[i];
        socket_->write((char*)&data, sizeof(data));
    }
    socket_->flush();
}

bool Socket::connectToHost(const QString &address)
{
    if(!connected_)
    {
        socket_->connectToHost(address, 3336);
        if(socket_->waitForConnected(15000))
        {
            connected_ = true;
            RobotData data;
            data.number = 1;
            data.cByte = 0;
            socket_->write((char*)&data, sizeof(data));
            socket_->flush();
            qDebug() << "Connected to " << address;
            return true;
        }
        qDebug() << "Error: Can not connect to " << address;
        return false;
    }
    return true;
}
