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
#define POINT_RADIUS 10

PlayField::PlayField(QWidget *parent) : QWidget(parent),
    m_goal{0, 0}
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


    if((this->width() / RATIO) > this->height())
    {
        propWidth = this->height() * RATIO;
        propHeight = this->height();
        m_scaleFactor = this->height() / ACTUAL_HEIGHT;
        m_origin.x = this->width() / 2 - propWidth / 2;
        m_origin.y = 0;
    }
    else
    {
        propWidth = this->width();
        propHeight = this->width() / RATIO;
        m_scaleFactor = this->width() / ACTUAL_WIDTH;
        m_origin.x = 0;
        m_origin.y = this->height() / 2 - propHeight / 2;
    }
    boardOrigin.x = (RECT_X0 / X_SCALE) * m_scaleFactor + m_origin.x;
    boardOrigin.y = RECT_Y0 * m_scaleFactor + m_origin.y;
    boardWidth = RECT_WIDTH * m_scaleFactor / X_SCALE;
    boardHeight = RECT_HEIGHT * m_scaleFactor;
    robotRadius = ROBOT_RADIUS * m_scaleFactor;
    pointRadius = POINT_RADIUS * m_scaleFactor;
    letterSize = propHeight / 15;

    // Draw board
    QPainter painter(this);
    QPen pen;
    painter.setBrush(QBrush(Qt::red));
    painter.setPen(Qt::NoPen);
    painter.drawRect(m_origin.x, m_origin.y,
                     propWidth, propHeight);
    painter.setBrush(QBrush(Qt::black));
    painter.drawRect(boardOrigin.x,
                     boardOrigin.y,
                     boardWidth,
                     boardHeight);
    // Draw robots
    foreach(Robot2D robot, m_robots)
    {
        robot.center.x = robot.center.x * m_scaleFactor + m_origin.x;
        robot.center.y = robot.center.y * m_scaleFactor + m_origin.y;

        // Draw robot circle
        painter.setBrush(QBrush(Qt::blue));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(robot.center.x - robotRadius / 2,
                            robot.center.y - robotRadius / 2,
                            robotRadius,
                            robotRadius);
        // Draw robot direction
        painter.setBrush(QBrush(Qt::white));
        pen.setColor(Qt::white);
        pen.setWidthF(propHeight / 150);
        painter.setPen(pen);
        painter.drawLine(robot.center.x, robot.center.y,
                         robotRadius / 2 * cos(robot.angle) + robot.center.x,
                         robotRadius / 2 * sin(robot.angle) + robot.center.y);

        // Draw robot points
        for(size_t i = 0; i < (sizeof(robot.points) / sizeof(robot.points[0])); ++i)
        {
            Point2D point = robot.points[i];
            point.x = point.x * m_scaleFactor + m_origin.x;
            point.y = point.y * m_scaleFactor + m_origin.y;
            painter.drawEllipse(point.x - pointRadius / 2,
                                point.y - pointRadius / 2,
                                pointRadius,
                                pointRadius);
        }

        // Draw robot number
        pen.setColor(Qt::red);
        painter.setPen(pen);
        painter.setFont(QFont("Serif", letterSize));
        painter.drawText(robot.center.x - robotRadius / 4,
                         robot.center.y - robotRadius / 1.5,
                         QString::number(robot.number));
    }

    // Draw goal
    if(m_goal.x != 0 && m_goal.y != 0)
    {
        painter.setBrush(QBrush(Qt::red));
        Point2D goal{(uint16_t)(m_goal.x * m_scaleFactor + m_origin.x),
                    (uint16_t)(m_goal.y * m_scaleFactor + m_origin.y)};
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
        m_goal.x = (ev->x() - m_origin.x) / m_scaleFactor;
        m_goal.y = (ev->y() - m_origin.y) / m_scaleFactor;
        emit sendGoal(m_goal);
    }
}
