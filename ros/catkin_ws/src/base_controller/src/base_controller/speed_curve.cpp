#include <base_controller/speed_curve.h>
#include <algorithm>
#include <math.h>

#include <ros/ros.h>

SpeedCurve::SpeedCurve()
    : acceleration(0.1), distanceCoefficient(0.1), maxSpeed(0.5), loopPeriod(0.1), currentSpeed(0), targetSpeed(0)
{
    reset();
}

void SpeedCurve::reset() {
    setCurrentSpeed(0);
}

void SpeedCurve::setAcceleration(double acceleration)
{
    this->acceleration = acceleration;
}

void SpeedCurve::setDistanceCoefficient(double coefficient) {
    this->distanceCoefficient = coefficient;
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
    double speedMagnitude = 2 * distanceCoefficient * fabs(distance);
    targetSpeed = std::copysign(speedMagnitude, distance);
}

void SpeedCurve::setCurrentSpeed(double speed) {
    this->currentSpeed = speed;
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

    ROS_INFO("speed:%f", currentSpeed);
    ROS_INFO("target:%f", targetSpeed);

    return currentSpeed;
}
