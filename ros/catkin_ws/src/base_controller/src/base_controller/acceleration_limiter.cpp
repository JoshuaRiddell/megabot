#include <base_controller/acceleration_limiter.h>

AccelerationLimiter::AccelerationLimiter()
    : maxAcceleration(0.1), loopPeriod(0.1)
{
    reset();
}

void AccelerationLimiter::reset() {
    currentVelocity = tf2::Vector3(0,0,0);
    targetVelocity = tf2::Vector3(0,0,0);
}

void AccelerationLimiter::setMaxAcceleration(double maxAcceleration) {
    this->maxAcceleration = maxAcceleration;
}

void AccelerationLimiter::setLoopPeriod(double loopPeriod) {
    this->loopPeriod = loopPeriod;
}

void AccelerationLimiter::setTargetVelocity(tf2::Vector3 velocity)
{
    targetVelocity = velocity;
}

void AccelerationLimiter::setTargetSpeedDirection(double speed, tf2::Vector3 direction)
{
    targetVelocity = direction.normalize() * speed;
}

tf2::Vector3 AccelerationLimiter::getNextVelocity()
{
    tf2::Vector3 velocityIncrement;
    tf2::Vector3 velocityDifference = targetVelocity - currentVelocity;

    if (velocityDifference.length() > maxAcceleration * loopPeriod) {
        velocityIncrement = velocityDifference.normalize() * maxAcceleration * loopPeriod;
    } else {
        velocityIncrement = velocityDifference;
    }
    currentVelocity += velocityIncrement;
    return currentVelocity;
}
