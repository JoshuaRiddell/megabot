#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <base_controller/GotoPointAction.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <base_controller/speed_curve.h>
#include <base_controller/acceleration_limiter.h>

class GotoAction {
public:
    tf2::Transform getRobotTransform(std::string target_frame, std::string reference_frame);

protected:
    GotoAction(std::string name);
    void publishVelocity(tf2::Vector3 translation, double rotation);

    ros::NodeHandle nh;

private:
    std::string actionName;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
};

class GotoPointAction : public GotoAction {
public:
    GotoPointAction(std::string actionName);
    ~GotoPointAction();

    void executeCallback(const base_controller::GotoPointGoalConstPtr &goal);

private:
    actionlib::SimpleActionServer<base_controller::GotoPointAction> actionServer;
    base_controller::GotoPointFeedback feedback;
    base_controller::GotoPointResult result;

    SpeedCurve translationSpeedCurve;
    AccelerationLimiter accelerationLimiter;
};
