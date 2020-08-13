#include <base_controller/speed_curve.h>
#include <algorithm>

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

void SpeedCurve::calculateSpeedDistanceGradient()
{
    timeToMaxSpeed = maxSpeed / acceleration;
    distanceTomaxSpeed = 0.5 * acceleration * timeToMaxSpeed * timeToMaxSpeed;
    speedDistanceGradient = maxSpeed / distanceTomaxSpeed;
}

void SpeedCurve::setTargetDistance(double distance)
{
    targetSpeed = std::min(maxSpeed, distance * speedDistanceGradient);
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
