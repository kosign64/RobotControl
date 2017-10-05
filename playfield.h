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
    void invertDrawPath() {drawPath_ = !drawPath_;}

protected:
    void paintEvent(QPaintEvent *);
    void timerEvent(QTimerEvent *);
    void mousePressEvent(QMouseEvent *ev);

private:
    RobotVector robots_;
    double scaleFactor_;
    Point2D origin_;
    Point2D goal_;
    int keyboardControlNumber_;
    bool drawPath_;
    PointVector path_;

signals:
    void sendGoal(Point2D goal);

public slots:
    void getRobots(RobotVector rob) {robots_ = rob;}
    void getRobotNumber(int number) {keyboardControlNumber_ = number;}
};

#endif // PLAYFIELD_H
