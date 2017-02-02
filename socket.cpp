#include "socket.h"
#include "defines.h"
#include <QTcpSocket>

Socket::Socket(QObject *parent) : QObject(parent),
    m_connected(false),
    m_pointSize(255)
{
    m_socket = new QTcpSocket(this);
    RobotData data;
    data.number = 0;
    data.cByte = _BV(G);
    m_robotVector.append(data);
    connect(m_socket, &QTcpSocket::readyRead, this, &Socket::readyRead);
    connect(m_socket, &QTcpSocket::disconnected, this,
            &Socket::disconnected);
    connect(m_socket, SIGNAL(error(QAbstractSocket::SocketError)), this,
            SLOT(getError(QAbstractSocket::SocketError)));
}

Socket::~Socket()
{

}

//=============================================================================

void Socket::readyRead()
{
    if((size_t)m_socket->bytesAvailable() >= sizeof(uint16_t))
    {
        m_socket->read((char*)&m_pointSize, sizeof(uint16_t));
    }
    if((size_t)m_socket->bytesAvailable() >= ((sizeof(Point2D)) * m_pointSize))
    {
        m_points.resize(m_pointSize);
        for(int i = 0; i < m_pointSize; ++i)
        {
            Point2D point;
            m_socket->read((char*)&point, sizeof(point));
            m_points[i] = point;
        }

        sendWheels();
        m_pointSize = 255;
        emit sendPoints(m_points);
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
    for(int i = 0; i < m_robotVector.size(); ++i)
    {
        RobotData &data = m_robotVector[i];
        m_socket->write((char*)&data, sizeof(data));
    }
    m_socket->flush();
}

bool Socket::connectToHost(const QString &address)
{
    if(!m_connected)
    {
        m_socket->connectToHost(address, 3336);
        if(m_socket->waitForConnected(2000))
        {
            m_connected = true;
            RobotData data;
            data.number = 1;
            data.cByte = 0;
            m_socket->write((char*)&data, sizeof(data));
            m_socket->flush();
            qDebug() << "Connected to " << address;
            return true;
        }
        qDebug() << "Error: Can not connect to " << address;
        return false;
    }
    return true;
}
