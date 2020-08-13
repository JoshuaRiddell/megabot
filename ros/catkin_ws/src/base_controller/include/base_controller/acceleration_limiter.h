#include <tf2/LinearMath/Vector3.h>

class AccelerationLimiter {
public:
    AccelerationLimiter();
    void setMaxAcceleration(double maxAcceleration);
    void setLoopPeriod(double loopPeriod);

    void setTargetVelocity(tf2::Vector3 velocity);
    void setTargetSpeedDirection(double speed, tf2::Vector3 direction);
    tf2::Vector3 getNextVelocity();
    
    tf2::Vector3 currentVelocity;
    tf2::Vector3 targetVelocity;
    double maxAcceleration;
    double loopPeriod;
};
