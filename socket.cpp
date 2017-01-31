#include "socket.h"
#include "defines.h"
#include <QTcpSocket>

Socket::Socket(QObject *parent) : QObject(parent),
    connected(false),
    dir(STOP),
    pointSize(255)
{
    socket = new QTcpSocket(this);
    RobotData data;
    data.number = 0;
    data.cByte = _BV(G);
    robotVector.append(data);
    connect(socket, &QTcpSocket::readyRead, this, &Socket::readyRead);
    connect(socket, &QTcpSocket::disconnected, this,
            &Socket::disconnected);
    connect(socket, SIGNAL(error(QAbstractSocket::SocketError)), this,
            SLOT(getError(QAbstractSocket::SocketError)));
}

Socket::~Socket()
{

}

//=============================================================================

void Socket::readyRead()
{
    if(socket->bytesAvailable() >= sizeof(uint16_t))
    {
        socket->read((char*)&pointSize, sizeof(uint16_t));
    }
    if(socket->bytesAvailable() >= ((sizeof(Point2D)) * pointSize))
    {
        points.resize(pointSize);
        for(int i = 0; i < pointSize; ++i)
        {
            Point2D point;
            socket->read((char*)&point, sizeof(point));
            points[i] = point;
        }

        sendWheels();
        pointSize = 255;
        emit sendPoints(points);
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
    for(int i = 0; i < robotVector.size(); ++i)
    {
        RobotData &data = robotVector[i];
        //qDebug() << data.number << data.cByte;
        socket->write((char*)&data, sizeof(data));
    }
    socket->flush();
}

bool Socket::connectToHost(const QString &address)
{
    if(!connected)
    {
        socket->connectToHost(address, 3336);
        if(socket->waitForConnected(2000))
        {
            connected = true;
            RobotData data;
            data.number = 1;
            data.cByte = 0;
            socket->write((char*)&data, sizeof(data));
            socket->flush();
            qDebug() << "Connected to " << address;
            return true;
        }
        qDebug() << "Error: Can not connect to " << address;
        return false;
    }
    return true;
}
