#ifndef DEFINES
#define DEFINES

#include <stdint-gcc.h>
#include <QVector>

#define _BV(b) (1 << (b))

#define G   6
#define DR  5
#define VR1 4
#define VR0 3
#define DL  2
#define VL1 1
#define VL0 0

#define FORWARD_BYTE (_BV(G) | _BV(DL) | _BV(VL1) | _BV(DR) | _BV(VR1))
#define RIGHTER_BYTE (_BV(G) | _BV(DL) | _BV(VL1) | _BV(DR) | _BV(VR0))
#define LEFTER_BYTE (_BV(G) | _BV(DL) | _BV(VL0) | _BV(DR) | _BV(VR1))
#define STOP_BYTE (_BV(G))
#define LEFT_BYTE (_BV(G) | _BV(VR0) | _BV(DR))
#define RIGHT_BYTE (_BV(G) | _BV(DL) | _BV(VL0))
#define BACKWARD_BYTE (_BV(G) | _BV(VL1) | _BV(VR1))

#define RATIO (4 / 3.f)

// width = 4132 - 386 = 3746
// height = 575 - 1 = 574 => width = 765 => coef = 4.35
#define X_OFFSET 386
#define Y_OFFSET 1
#define ACTUAL_WIDTH (574 * RATIO)
#define X_SCALE (3746 / ACTUAL_WIDTH)

enum Dir
{
    STOP,
    LEFT,
    RIGHT,
    FORWARD,
    BACKWARD
};

struct Point2D
{
    uint16_t x;
    uint16_t y;
};

struct Robot2D
{
    Point2D center;
    Point2D points[2];
    double angle;
    int number;
};

struct RobotData
{
    uint8_t number;
    uint8_t cByte;
};

typedef QVector<Point2D>   PointVector;
typedef QVector<Robot2D>   RobotVector;
typedef QVector<RobotData> RobotDataVector;

#endif // DEFINES
