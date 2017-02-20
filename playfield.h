#ifndef PLAYFIELD_H
#define PLAYFIELD_H

#include <QWidget>
#include "defines.h"

class PlayField : public QWidget
{
    Q_OBJECT
public:
    explicit PlayField(QWidget *parent = 0);
    ~PlayField();
    QSize sizeHint() const {return QSize(800, 600);}

protected:
    void paintEvent(QPaintEvent *);
    void timerEvent(QTimerEvent *);
    void mousePressEvent(QMouseEvent *ev);

private:
    RobotVector m_robots;
    double m_scaleFactor;
    Point2D m_origin;
    Point2D m_goal;
    int m_keyboardControlNumber;

signals:
    void sendGoal(Point2D goal);

public slots:
    void getRobots(RobotVector rob) {m_robots = rob;}
    void getRobotNumber(int number) {m_keyboardControlNumber = number;}
};

#endif // PLAYFIELD_H
