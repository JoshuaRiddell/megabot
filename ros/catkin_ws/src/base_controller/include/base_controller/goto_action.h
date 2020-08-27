#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

#include <base_controller/speed_curve.h>
#include <base_controller/acceleration_limiter.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

class GotoAction {
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

private:
    tf2::Transform getTransform(std::string startFrame, std::string endFrame);
    void updateCmdVel();
    void publishCmdVel();
    void resetCmdVel();


    ros::Publisher cmdVelPub;

    std::string robotFrame;
    std::string targetFrame;

    // void publishVelocity(tf2::Vector3 translation, double rotation);

    ros::NodeHandle nh;
    std::string actionName;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    geometry_msgs::Twist cmdVel;

    SpeedCurve translationSpeedCurve;
    SpeedCurve rotationSpeedCurve;
    AccelerationLimiter accelerationLimiter;

    tf2::Vector3 goalPoint;
    tf2::Quaternion goalRotation;

    ros::Rate loopRate;
};

