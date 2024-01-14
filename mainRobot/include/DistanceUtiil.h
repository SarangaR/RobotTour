#ifndef DistanceUtil_h
#define DistanceUtil_h

#include "RobotConfig.h"

#define METER 0
#define INCH 1
#define FOOT 2
#define CM 3
#define MM 4

#define METERS_TO_INCHES 39.3701
#define METERS_TO_FEET 3.28084
#define METERS_TO_CM 100
#define METERS_TO_MM 1000

class DistanceUtil {
    public:
        static double encoderTicksToDistance(int ticks, int units) {
            double meters = (double) ticks / TICKS_PER_METER;
            switch (units) {
                case METER:
                    return meters;
                case INCH:
                    return meters * METERS_TO_INCHES;
                case FOOT:
                    return meters * METERS_TO_FEET;
                case CM:
                    return meters * METERS_TO_CM;
                case MM:
                    return meters * METERS_TO_MM;
                default:
                    return meters;
            }
        };

        static double distanceToEncoderTicks(double distance, int units) {
            double meters = 0;
            switch (units) {
                case METER:
                    meters = distance;
                    break;
                case INCH:
                    meters = distance / METERS_TO_INCHES;
                    break;
                case FOOT:
                    meters = distance / METERS_TO_FEET;
                    break;
                case CM:
                    meters = distance / METERS_TO_CM;
                    break;
                case MM:
                    meters = distance / METERS_TO_MM;
                    break;
                default:
                    meters = distance;
            };
            return meters * TICKS_PER_METER;
        }
};
#endif