#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "defines.h"

class QVBoxLayout;
class QLineEdit;
class QPushButton;
class Socket;
class PlayField;
class RobotFinder;
class Controller;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    void keyPressEvent(QKeyEvent *ev);
    void keyReleaseEvent(QKeyEvent *ev);

private:
    QWidget *mainWidget;
    QVBoxLayout *mainLayout;
    QLineEdit *ipAddressEdit;
    QPushButton *connectButton;
    Socket *socket;
    PlayField *field;
    RobotFinder *robotFinder;
    QVector <Robot2D> robots;
    Controller *controller;

    bool upPressed;
    bool downPressed;
    bool leftPressed;
    bool rightPressed;

    bool start;

    int robotNumber;

signals:
    void sendWheels(RobotDataVector vec);

private slots:
    void onConnectClick();

public slots:

};

#endif // MAINWINDOW_H
