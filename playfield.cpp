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

inline static float length(Point2D &p1, Point2D &p2)
{
    return sqrt(pow(p1.x - p2.x, 2) +
                pow(p1.y - p2.y, 2));
}

inline static Point2D center(Point2D &p1, Point2D &p2)
{
    Point2D p;
    p.x = (p1.x + p2.x) / 2;
    p.y = (p1.y + p2.y) / 2;
    return p;
}

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
    QPainter painter(this);
    painter.setBrush(QBrush(Qt::red));
    painter.setPen(Qt::NoPen);

    // Draw everything
    if((this->width() / RATIO) > this->height())
    {
        int propWidth = this->height() * RATIO;
        painter.drawRect(this->width() / 2 - propWidth / 2, 0,
                         propWidth, this->height());
        float scaleFactor = this->height() / ACTUAL_HEIGHT;
        painter.setBrush(QBrush(Qt::black));
        painter.drawRect(RECT_X0 / X_SCALE * scaleFactor +
                         this->width() / 2 - propWidth / 2,
                         RECT_Y0 * scaleFactor,
                         RECT_WIDTH * scaleFactor / X_SCALE,
                         RECT_HEIGHT * scaleFactor);
        foreach(Robot2D robot, robots)
        {
            painter.setBrush(QBrush(Qt::blue));
            painter.setPen(QPen(Qt::blue));
            painter.drawEllipse(robot.center.x * scaleFactor +
                                this->width() / 2 - propWidth / 2 - ROBOT_RADIUS * scaleFactor / 2,
                                robot.center.y * scaleFactor - ROBOT_RADIUS * scaleFactor / 2,
                                ROBOT_RADIUS * scaleFactor,
                                ROBOT_RADIUS * scaleFactor);
            painter.setBrush(QBrush(Qt::white));
            painter.setPen(QPen(Qt::white));
            painter.drawLine(robot.center.x * scaleFactor +
                             this->width() / 2 - propWidth / 2, robot.center.y * scaleFactor,
                             (30 * cos(robot.angle) + robot.center.x) * scaleFactor +
                             this->width() / 2 - propWidth / 2,
                             (30 * sin(robot.angle) + robot.center.y) * scaleFactor);
            painter.setBrush(QBrush(Qt::white));
            for(size_t i = 0; i < (sizeof(robot.points) / sizeof(robot.points[0])); ++i)
            {
                Point2D point = robot.points[i];
                painter.drawEllipse(point.x * scaleFactor +
                                    this->width() / 2 - propWidth / 2 -
                                    POINT_RADIUS * scaleFactor / 2,
                                    point.y * scaleFactor -
                                    POINT_RADIUS * scaleFactor / 2,
                                    POINT_RADIUS * scaleFactor,
                                    POINT_RADIUS * scaleFactor);
            }
            painter.setPen(QPen(Qt::red));
            painter.setFont(QFont("Serif", this->height() / 15));
            painter.drawText(robot.center.x * scaleFactor +
                             this->width() / 2 - propWidth / 2,
                             robot.center.y * scaleFactor,
                             QString::number(robot.number));
        }
    }
    else
    {
        int propHeight = this->width() / RATIO;
        painter.drawRect(0, this->height() / 2 - propHeight / 2,
                         this->width(), propHeight);
        float scaleFactor = this->width() / ACTUAL_WIDTH;
        painter.setBrush(QBrush(Qt::black));
        painter.drawRect((RECT_X0 / X_SCALE) * scaleFactor,
                         RECT_Y0 * scaleFactor + this->height() / 2 -
                         propHeight / 2,
                         RECT_WIDTH * scaleFactor / X_SCALE,
                         RECT_HEIGHT * scaleFactor);
        foreach(Robot2D robot, robots)
        {
            painter.setBrush(QBrush(Qt::blue));
            painter.setPen(QPen(Qt::blue));
            painter.drawEllipse(robot.center.x * scaleFactor - ROBOT_RADIUS * scaleFactor / 2,
                                robot.center.y * scaleFactor +
                                this->height() / 2 - propHeight / 2 - ROBOT_RADIUS * scaleFactor / 2,
                                ROBOT_RADIUS * scaleFactor,
                                ROBOT_RADIUS * scaleFactor);
            painter.setBrush(QBrush(Qt::white));
            painter.setPen(QPen(Qt::white));
            painter.drawLine(robot.center.x * scaleFactor, robot.center.y * scaleFactor +
                             this->height() / 2 - propHeight / 2,
                             (30 * cos(robot.angle) + robot.center.x) * scaleFactor,
                             (30 * sin(robot.angle) + robot.center.y) * scaleFactor +
                             this->height() / 2 - propHeight / 2);
            painter.setBrush(QBrush(Qt::white));
            for(size_t i = 0; i < (sizeof(robot.points) / sizeof(robot.points[0])); ++i)
            {
                Point2D point = robot.points[i];
                painter.drawEllipse(point.x * scaleFactor -
                                    POINT_RADIUS * scaleFactor / 2,
                                    point.y * scaleFactor +
                                    this->height() / 2 - propHeight / 2 -
                                    POINT_RADIUS * scaleFactor / 2,
                                    POINT_RADIUS * scaleFactor,
                                    POINT_RADIUS * scaleFactor);
            }
            painter.setPen(QPen(Qt::red));
            painter.setFont(QFont("Serif", 20));
            painter.drawText(robot.center.x * scaleFactor,
                             robot.center.y * scaleFactor +
                             this->height() / 2 - propHeight / 2,
                             QString::number(robot.number));
        }
    }
}

void PlayField::timerEvent(QTimerEvent *)
{
    update();
}
