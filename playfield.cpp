#include "playfield.h"
#include <QPainter>
#include <QMouseEvent>
#include <cmath>
#include <QDebug>

#define ACTUAL_HEIGHT 574.f

#define RECT_X0     (692 - X_OFFSET)
#define RECT_Y0     (23 - Y_OFFSET)
#define RECT_WIDTH  ((3970 - X_OFFSET) - RECT_X0)
#define RECT_HEIGHT ((553 - Y_OFFSET) - RECT_Y0)

#define ROBOT_RADIUS 60
#define POINT_RADIUS 12

PlayField::PlayField(QWidget *parent) : QWidget(parent),
    goal_{0, 0},
    keyboardControlNumber_(1),
    drawPath_(false)
{
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    startTimer(1000 / 40);
}

PlayField::~PlayField()
{

}

void PlayField::paintEvent(QPaintEvent *)
{
    int robotRadius;
    int pointRadius;
    int propWidth;
    int propHeight;
    Point2D boardOrigin;
    int boardWidth;
    int boardHeight;
    int letterSize;

    if(!drawPath_)
    {
        path_.clear();
    }

    if((this->width() / RATIO) > this->height())
    {
        propWidth = this->height() * RATIO;
        propHeight = this->height();
        scaleFactor_ = this->height() / ACTUAL_HEIGHT;
        origin_.x = this->width() / 2 - propWidth / 2;
        origin_.y = 0;
    }
    else
    {
        propWidth = this->width();
        propHeight = this->width() / RATIO;
        scaleFactor_ = this->width() / ACTUAL_WIDTH;
        origin_.x = 0;
        origin_.y = this->height() / 2 - propHeight / 2;
    }
    boardOrigin.x = (RECT_X0 / X_SCALE) * scaleFactor_ + origin_.x;
    boardOrigin.y = RECT_Y0 * scaleFactor_ + origin_.y;
    boardWidth = RECT_WIDTH * scaleFactor_ / X_SCALE;
    boardHeight = RECT_HEIGHT * scaleFactor_;
    robotRadius = ROBOT_RADIUS * scaleFactor_;
    pointRadius = POINT_RADIUS * scaleFactor_;
    letterSize = propHeight / 15;

    // Draw board
    QPainter painter(this);
    QPen pen;
    painter.setBrush(QBrush(Qt::red));
    painter.setPen(Qt::NoPen);
    painter.drawRect(origin_.x, origin_.y,
                     propWidth, propHeight);
    painter.setBrush(QBrush(Qt::black));
    painter.drawRect(boardOrigin.x,
                     boardOrigin.y,
                     boardWidth,
                     boardHeight);
    // Draw robots
    foreach(Robot2D robot, robots_)
    {
        if(robot.number == 1 && drawPath_)
        {
            path_.push_back(robot.center);
        }

        if(drawPath_)
        {
            painter.save();
            painter.setPen(Qt::NoPen);
            for(int i = 0; i < path_.size(); ++i)
            {
                Point2D point = path_[i];
                point.x = point.x * scaleFactor_ + origin_.x;
                point.y = point.y * scaleFactor_ + origin_.y;
                painter.drawEllipse(point.x - pointRadius / 2,
                                    point.y - pointRadius / 2,
                                    pointRadius,
                                    pointRadius);
            }
            painter.restore();
        }

        robot.center.x = robot.center.x * scaleFactor_ + origin_.x;
        robot.center.y = robot.center.y * scaleFactor_ + origin_.y;

        // Draw robot circle
        painter.setBrush(QBrush(Qt::blue));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(robot.center.x - robotRadius / 2,
                            robot.center.y - robotRadius / 2,
                            robotRadius,
                            robotRadius);
        // Draw robot direction
//        painter.setBrush(QBrush(Qt::white));
//        pen.setColor(Qt::white);
//        pen.setWidthF(propHeight / 150);
//        painter.setPen(pen);
//        painter.drawLine(robot.center.x, robot.center.y,
//                         robotRadius / 2 * cos(robot.angle) + robot.center.x,
//                         robotRadius / 2 * sin(robot.angle) + robot.center.y);
        painter.save();
        painter.setBrush(QBrush(Qt::yellow));
        painter.translate(robot.center.x, robot.center.y);
        painter.rotate(robot.angle * 180. / M_PI - 90);
        QPointF triangle[3] = {QPointF(0, robotRadius / 2), QPointF(-robotRadius / 2, 0),
                              QPointF(robotRadius / 2, 0)};
        painter.drawPolygon(triangle, 3);
        painter.restore();

        painter.setBrush(QBrush(Qt::white));
        pen.setColor(Qt::white);
        painter.setPen(pen);

        // Draw robot points
        for(size_t i = 0; i < (sizeof(robot.points) / sizeof(robot.points[0])); ++i)
        {
            Point2D point = robot.points[i];
            point.x = point.x * scaleFactor_ + origin_.x;
            point.y = point.y * scaleFactor_ + origin_.y;
            painter.drawEllipse(point.x - pointRadius / 2,
                                point.y - pointRadius / 2,
                                pointRadius,
                                pointRadius);
        }

        // Draw robot number
        if(robot.number == keyboardControlNumber_)
        {
            pen.setColor(Qt::green);
        }
        else
        {
            pen.setColor(Qt::red);
        }
        painter.setPen(pen);
        painter.setFont(QFont("Serif", letterSize));
        painter.drawText(robot.center.x - robotRadius / 4,
                         robot.center.y - robotRadius / 1.5,
                         QString::number(robot.number));
    }

    painter.setPen(Qt::NoPen);
    // Draw goal
    if(goal_.x != 0 && goal_.y != 0)
    {
        painter.setBrush(QBrush(Qt::red));
        Point2D goal{(uint16_t)(goal_.x * scaleFactor_ + origin_.x),
                    (uint16_t)(goal_.y * scaleFactor_ + origin_.y)};
        painter.drawEllipse(goal.x - pointRadius / 2,
                            goal.y - pointRadius / 2,
                            pointRadius,
                            pointRadius);
    }
}

void PlayField::timerEvent(QTimerEvent *)
{
    update();
}

void PlayField::mousePressEvent(QMouseEvent *ev)
{
    if(ev->button() == Qt::LeftButton)
    {
        goal_.x = (ev->x() - origin_.x) / scaleFactor_;
        goal_.y = (ev->y() - origin_.y) / scaleFactor_;
        emit sendGoal(goal_);
    }
}
