#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

#include <base_controller/speed_curve.h>
#include <base_controller/acceleration_limiter.h>

#include <base_controller/BaseControllerConfig.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <ecl/ipc.hpp>

class GotoAction {
public:
    GotoAction();

    void setConfig(base_controller::BaseControllerConfig &config);

    void setRobotFrame(std::string robotFrameMsg);
    void setTargetFrame(std::string targetFrameMsg);
    void setGoalPoint(geometry_msgs::Point pointMsg);
    void setDistanceThreshold(double distanceThreshold);
    void setGoalRotation(geometry_msgs::Quaternion angleMsg);
    void setRotationThreshold(double rotationThreshold);

    void resetControllers();
    void resetPosition();
    void disable();
    void enable();
    void stopRobot();
    void publishNextCmdVel();

    bool isReachedGoal();
    void loopDelay();

private:
    tf2::Transform getTransform(std::string startFrame, std::string endFrame);
    void updateCmdVel();
    void updateTranslationVelocity(tf2::Transform robotTransform);
    void updateRotationVelocity(tf2::Transform robotTransform);
    void publishCmdVel();
    void resetCmdVel();

    ros::NodeHandle nh;

    ros::Publisher cmdVelPub;

    std::string robotFrame;
    std::string targetFrame;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    geometry_msgs::Twist cmdVel;

    SpeedCurve translationSpeedCurve;
    SpeedCurve rotationSpeedCurve;
    AccelerationLimiter accelerationLimiter;

    tf2::Vector3 goalPoint;
    tf2::Quaternion goalRotation;

    bool enabled;
    bool hasReachedTranslationGoal;
    bool hasReachedRotationGoal;

    double distanceThreshold;
    double rotationThreshold;

    ros::Rate loopRate;

    ecl::Semaphore semaphore;
};

