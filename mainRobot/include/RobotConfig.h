#include "math.h"

#define WHEEL_DIAMETER 0.1
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * M_PI)
#define TICKS_PER_REVOLUTION 360
#define TICKS_PER_METER (TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE)

#define IMU_KP 1
#define IMU_KI 0
#define IMU_KD 0

#define LEFT_KP 1
#define LEFT_KI 0
#define LEFT_KD 0

#define RIGHT_KP 1
#define RIGHT_KI 0
#define RIGHT_KD 0