#include "playfield.h"
#include <QPainter>
#include <cmath>
#include <QDebug>

#define ACTUAL_HEIGHT 574.f

#define RECT_X0     (692 - X_OFFSET)
#define RECT_Y0     (23 - Y_OFFSET)
#define RECT_WIDTH  ((3970 - X_OFFSET) - RECT_X0)
#define RECT_HEIGHT ((553 - Y_OFFSET) - RECT_Y0)

#define ROBOT_RADIUS 60
#define POINT_RADIUS 10

PlayField::PlayField(QWidget *parent) : QWidget(parent)
{
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    startTimer(1000 / 40);
}

PlayField::~PlayField()
{

}

void PlayField::paintEvent(QPaintEvent *)
{
    double scaleFactor;
    int robotRadius;
    int pointRadius;
    int propWidth;
    int propHeight;
    Point2D origin;
    Point2D boardOrigin;
    int boardWidth;
    int boardHeight;
    int letterSize;


    if((this->width() / RATIO) > this->height())
    {
        propWidth = this->height() * RATIO;
        propHeight = this->height();
        scaleFactor = this->height() / ACTUAL_HEIGHT;
        origin.x = this->width() / 2 - propWidth / 2;
        origin.y = 0;
    }
    else
    {
        propWidth = this->width();
        propHeight = this->width() / RATIO;
        scaleFactor = this->width() / ACTUAL_WIDTH;
        origin.x = 0;
        origin.y = this->height() / 2 - propHeight / 2;
    }
    boardOrigin.x = (RECT_X0 / X_SCALE) * scaleFactor + origin.x;
    boardOrigin.y = RECT_Y0 * scaleFactor + origin.y;
    boardWidth = RECT_WIDTH * scaleFactor / X_SCALE;
    boardHeight = RECT_HEIGHT * scaleFactor;
    robotRadius = ROBOT_RADIUS * scaleFactor;
    pointRadius = POINT_RADIUS * scaleFactor;
    letterSize = propHeight / 15;

    // Draw board
    QPainter painter(this);
    painter.setBrush(QBrush(Qt::red));
    painter.setPen(Qt::NoPen);
    painter.drawRect(origin.x, origin.y,
                     propWidth, propHeight);
    painter.setBrush(QBrush(Qt::black));
    painter.drawRect(boardOrigin.x,
                     boardOrigin.y,
                     boardWidth,
                     boardHeight);
    // Draw robots
    foreach(Robot2D robot, robots)
    {
        robot.center.x = robot.center.x * scaleFactor + origin.x;
        robot.center.y = robot.center.y * scaleFactor + origin.y;

        // Draw robot circle
        painter.setBrush(QBrush(Qt::blue));
        painter.setPen(QPen(Qt::blue));
        painter.drawEllipse(robot.center.x - robotRadius / 2,
                            robot.center.y - robotRadius / 2,
                            robotRadius,
                            robotRadius);
        // Draw robot direction
        painter.setBrush(QBrush(Qt::white));
        painter.setPen(QPen(Qt::white));
        painter.drawLine(robot.center.x, robot.center.y,
                         robotRadius / 2 * cos(robot.angle) + robot.center.x,
                         robotRadius / 2 * sin(robot.angle) + robot.center.y);

        // Draw robot points
        for(size_t i = 0; i < (sizeof(robot.points) / sizeof(robot.points[0])); ++i)
        {
            Point2D point = robot.points[i];
            point.x = point.x * scaleFactor + origin.x;
            point.y = point.y * scaleFactor + origin.y;
            painter.drawEllipse(point.x - pointRadius / 2,
                                point.y - pointRadius / 2,
                                pointRadius,
                                pointRadius);
        }

        // Draw robot number
        painter.setPen(QPen(Qt::red));
        painter.setFont(QFont("Serif", letterSize));
        painter.drawText(robot.center.x - robotRadius / 4,
                         robot.center.y - robotRadius / 1.5,
                         QString::number(robot.number));
    }
}

void PlayField::timerEvent(QTimerEvent *)
{
    update();
}
