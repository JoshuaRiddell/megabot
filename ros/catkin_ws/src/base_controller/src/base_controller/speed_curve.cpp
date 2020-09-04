#include <base_controller/speed_curve.h>
#include <algorithm>
#include <math.h>

SpeedCurve::SpeedCurve()
    : acceleration(0.1), minSpeed(0.05), maxSpeed(0.5), loopPeriod(0.1), currentSpeed(0), targetSpeed(0)
{
    reset();
}

void SpeedCurve::reset() {
    currentSpeed = 0;
}

void SpeedCurve::setAcceleration(double acceleration)
{
    this->acceleration = acceleration;
}

void SpeedCurve::setMinSpeed(double minSpeed)
{
    this->minSpeed = minSpeed;
}

void SpeedCurve::setMaxSpeed(double maxSpeed)
{
    this->maxSpeed = maxSpeed;
}

void SpeedCurve::setLoopPeriod(double loopPeriod)
{
    this->loopPeriod = loopPeriod;
}

void SpeedCurve::setTargetDistance(double distance)
{   
    double speedMagnitude = sqrt(2 * acceleration * fabs(distance));
    targetSpeed = std::copysign(speedMagnitude, distance);
}

double SpeedCurve::getNextSpeed()
{
    if (targetSpeed > currentSpeed)
    {
        currentSpeed = currentSpeed + acceleration * loopPeriod;
    }
    else
    {
        currentSpeed = currentSpeed - acceleration * loopPeriod;
    }

    if (fabs(targetSpeed - currentSpeed) < acceleration * loopPeriod) {
        currentSpeed = targetSpeed;
    }

    if (fabs(currentSpeed) > maxSpeed) {
        currentSpeed = std::copysign(maxSpeed, currentSpeed);
    }

    return currentSpeed;
}
