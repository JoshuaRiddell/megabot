#include <base_controller/speed_curve.h>
#include <algorithm>
#include <math.h>

#include <ros/ros.h>

SpeedCurve::SpeedCurve()
    : acceleration(0.1), distanceCoefficient(0.1), minSpeed(0.1), maxSpeed(0.5), loopPeriod(0.1), currentSpeed(0), targetSpeed(0)
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

void SpeedCurve::setMinSpeed(double minSpeed) {
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
    double speedMagnitude = minSpeed + 2 * distanceCoefficient * fabs(distance);
    speedMagnitude = std::min(speedMagnitude, maxSpeed);
    targetSpeed = std::copysign(speedMagnitude, distance);
}

void SpeedCurve::setCurrentSpeed(double speed) {
    this->currentSpeed = speed;
}

double SpeedCurve::getNextSpeed()
{
    if (targetSpeedReachableInSingleStep()) {
        currentSpeed = targetSpeed;
    } else {
        if (speedIncreaseRequried())
        {
            incrementSpeed();
        }
        else
        {
            decrementSpeed();
        }
    }

    return currentSpeed;
}

bool SpeedCurve::targetSpeedReachableInSingleStep() {
    return fabs(targetSpeed - currentSpeed) < acceleration * loopPeriod;
}

bool SpeedCurve::speedIncreaseRequried() {
    return targetSpeed > currentSpeed;
}

void SpeedCurve::incrementSpeed() {
    currentSpeed = currentSpeed + acceleration * loopPeriod;
}

void SpeedCurve::decrementSpeed() {
    currentSpeed = currentSpeed - acceleration * loopPeriod;
}

