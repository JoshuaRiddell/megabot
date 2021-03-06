class SpeedCurve
{
public:
    SpeedCurve();
    void setAcceleration(double acceleration);
    void setDistanceCoefficient(double coefficient);
    void setDistanceDeadband(double distanceDeadband);
    void setMinSpeed(double minSpeed);
    void setMaxSpeed(double maxSpeed);
    void setLoopPeriod(double loopPeriod);

    void setTargetDistance(double distance);
    void setCurrentSpeed(double speed);
    double getNextSpeed();
    void reset();

private:
    bool inDeadbandRange();
    bool targetSpeedReachableInSingleStep();
    bool speedIncreaseRequried();
    void incrementSpeed();
    void decrementSpeed();

    double currentDistance;
    double currentSpeed;
    double targetSpeed;
    double loopPeriod;

    double acceleration;
    double distanceCoefficient;
    double distanceDeadband;
    double minSpeed;
    double maxSpeed;
};