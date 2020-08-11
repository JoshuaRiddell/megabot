#include <base_controller/speed_curve.h>
#include <algorithm>

SpeedCurve::SpeedCurve(float acceleration, float maxSpeed, float loopPeriod)
    : acceleration(acceleration), maxSpeed(maxSpeed), loopPeriod(loopPeriod), currentSpeed(0), targetSpeed(0)
{
    timeToMaxSpeed = maxSpeed / acceleration;
    distanceTomaxSpeed = 0.5 * acceleration * timeToMaxSpeed * timeToMaxSpeed;
    speedDistanceGradient = maxSpeed / distanceTomaxSpeed;
}

void SpeedCurve::setTargetDistance(float distance)
{
    targetSpeed = std::min(maxSpeed, distance * speedDistanceGradient);
}

float SpeedCurve::getNextSpeed()
{
    if (targetSpeed > currentSpeed)
    {
        currentSpeed = std::min(targetSpeed, currentSpeed + acceleration * loopPeriod);
    }
    else
    {
        currentSpeed = std::max(targetSpeed, currentSpeed - acceleration * loopPeriod);
    }

    return currentSpeed;
}
