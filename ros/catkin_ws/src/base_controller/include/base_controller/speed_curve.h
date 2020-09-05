class SpeedCurve
{
public:
    SpeedCurve();
    void setAcceleration(double acceleration);
    void setDistanceCoefficient(double coefficient);
    void setMaxSpeed(double maxSpeed);
    void setLoopPeriod(double loopPeriod);

    void setTargetDistance(double distance);
    void setCurrentSpeed(double speed);
    double getNextSpeed();
    void reset();

private:
    double currentSpeed;
    double targetSpeed;
    double loopPeriod;

    double acceleration;
    double distanceCoefficient;
    double maxSpeed;
};