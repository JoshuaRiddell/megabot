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

class GotoAction {
public:
    void setConfig(base_controller::BaseControllerConfig &config);

protected:
    GotoAction(std::string name);

    void resetControllers();
    void setRobotFrame(std_msgs::String robotFrameMsg);
    void setTargetFrame(std_msgs::String targetFrameMsg);
    void setGoalPoint(geometry_msgs::Point pointMsg);
    void setGoalRotation(geometry_msgs::Quaternion angleMsg);

    void publishNextCmdVel();
    void loopDelay();
    void stopRobot();

    bool isRosPreempted();
    bool isReachedGoal();

    ros::NodeHandle nh;

private:
    tf2::Transform getTransform(std::string startFrame, std::string endFrame);
    void updateCmdVel();
    void updateTranslationVelocity(tf2::Transform robotTransform);
    void updateRotationVelocity(tf2::Transform robotTransform);
    void publishCmdVel();
    void resetCmdVel();


    ros::Publisher cmdVelPub;

    std::string robotFrame;
    std::string targetFrame;

    std::string actionName;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    geometry_msgs::Twist cmdVel;

    SpeedCurve translationSpeedCurve;
    SpeedCurve rotationSpeedCurve;
    AccelerationLimiter accelerationLimiter;

    tf2::Vector3 goalPoint;
    tf2::Quaternion goalRotation;

    bool hasReachedTranslationGoal;
    bool hasReachedRotationGoal;

    const double distanceThreshold = 0.01;
    const double rotationThreshold = 0.05;

    ros::Rate loopRate;
};

