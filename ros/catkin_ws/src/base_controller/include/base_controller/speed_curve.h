class SpeedCurve
{
public:
    SpeedCurve();
    void setAcceleration(double acceleration);
    void setMinSpeed(double minSpeed);
    void setMaxSpeed(double maxSpeed);
    void setLoopPeriod(double loopPeriod);

    void setTargetDistance(double distance);
    double getNextSpeed();

private:
    void calculateSpeedDistanceGradient();

    double currentSpeed;
    double targetSpeed;
    double loopPeriod;

    double acceleration;
    double minSpeed;
    double maxSpeed;
    double timeToMaxSpeed;
    double distanceTomaxSpeed;
    double speedDistanceGradient;
};