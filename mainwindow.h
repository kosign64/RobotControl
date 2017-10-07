#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "defines.h"
#include "controller.h"
#include <QFile>
#include <QTextStream>

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
    QWidget *mainWidget_;
    QVBoxLayout *mainLayout_;
    QLineEdit *ipAddressEdit_;
    QPushButton *connectButton_;
    QPushButton *ignoreButton_;
    Socket *socket_;
    PlayField *field_;
    RobotFinder *robotFinder_;
    QVector <Robot2D> robots_;
    Controller *controller_;

    bool upPressed_;
    bool upLightPressed_;
    bool downPressed_;
    bool leftPressed_;
    bool rightPressed_;

    bool ignoreNumber_;

    bool start_;
    bool saveData_;

    int robotNumber_;

    QString filename_;
    QFile file_;
    QTextStream fileStream_;

signals:
    void sendWheels(RobotDataVector vec);
    void sendRobotNumber(int number);

private slots:
    void onConnectClick();
    void onIgnoreClick();
    void getControlData(ControlData data);

public slots:

};

#endif // MAINWINDOW_H
