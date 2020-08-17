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
    double currentSpeed;
    double targetSpeed;
    double loopPeriod;

    double acceleration;
    double minSpeed;
    double maxSpeed;
};