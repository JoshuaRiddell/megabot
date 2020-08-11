class SpeedCurve
{
public:
    SpeedCurve(float acceleration, float maxSpeed, float loopPeriod);
    void setTargetDistance(float distance);
    float getNextSpeed();

private:
    float currentSpeed;
    float targetSpeed;
    float loopPeriod;

    float acceleration;
    float maxSpeed;
    float timeToMaxSpeed;
    float distanceTomaxSpeed;
    float speedDistanceGradient;
};