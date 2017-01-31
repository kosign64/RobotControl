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
    QSize sizeHint() const {return QSize(320, 240);}

protected:
    void paintEvent(QPaintEvent *);
    void timerEvent(QTimerEvent *);

private:
    RobotVector robots;
    PointVector points;

public slots:
    void getRobots(RobotVector rob) {robots = rob;}
};

#endif // PLAYFIELD_H
