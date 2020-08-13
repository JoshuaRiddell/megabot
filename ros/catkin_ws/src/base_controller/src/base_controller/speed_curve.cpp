#include <base_controller/speed_curve.h>
#include <algorithm>
#include <math.h>

SpeedCurve::SpeedCurve()
    : acceleration(0.1), minSpeed(0.05), maxSpeed(0.5), loopPeriod(0.1), currentSpeed(0), targetSpeed(0)
{
    calculateSpeedDistanceGradient();
}

void SpeedCurve::setAcceleration(double acceleration)
{
    this->acceleration = acceleration;
    calculateSpeedDistanceGradient();
}

void SpeedCurve::setMinSpeed(double minSpeed)
{
    this->minSpeed = minSpeed;
    calculateSpeedDistanceGradient();
}

void SpeedCurve::setMaxSpeed(double maxSpeed)
{
    this->maxSpeed = maxSpeed;
    calculateSpeedDistanceGradient();
}

void SpeedCurve::setLoopPeriod(double loopPeriod)
{
    this->loopPeriod = loopPeriod;
}

void SpeedCurve::setTargetDistance(double distance)
{
    targetSpeed = std::min(maxSpeed, sqrt(2 * acceleration * distance));
}

double SpeedCurve::getNextSpeed()
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
