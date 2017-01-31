#ifndef SOCKET_H
#define SOCKET_H

#include <QObject>
#include <QAbstractSocket>
#include <stdint-gcc.h>
#include <QDebug>
#include "defines.h"

class QTcpSocket;

class Socket : public QObject
{
    Q_OBJECT
public:
    explicit Socket(QObject *parent = 0);
    ~Socket();
    void sendWheels();
    bool connectToHost(const QString &address = "127.0.0.1");
    void setDir(Dir d) {dir = d;}
    PointVector getPoints() {return points;}

private:
    QTcpSocket *socket;
    bool connected;
    Dir dir;
    uint16_t pointSize;
    PointVector points;
    RobotDataVector robotVector;

signals:
    void sendPoints(PointVector &p);

public slots:
    void readyRead();
    void disconnected();
    void getError(QAbstractSocket::SocketError error);
    void getRobotData(const RobotDataVector &data) {robotVector = data;}
};

#endif // SOCKET_H
