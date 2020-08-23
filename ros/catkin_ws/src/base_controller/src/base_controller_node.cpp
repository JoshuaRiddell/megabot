#include <base_controller.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <tf2/LinearMath/Vector3.h>
#include <actionlib/server/simple_action_server.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher cmdVelPub, resetOdomPub;

GotoAction::GotoAction(std::string actionName)
    : tfListener(tfBuffer),
      actionName(actionName)
{
}

tf2::Transform GotoAction::getRobotTransform(std::string targetFrame, std::string referenceFrame)
{
    geometry_msgs::TransformStamped robotTransformMsg;
    robotTransformMsg = tfBuffer.lookupTransform(targetFrame, referenceFrame, ros::Time::now(), ros::Duration(1.0));
    tf2::Transform robotTransform;
    tf2::fromMsg(robotTransformMsg.transform, robotTransform);
    return robotTransform;
}

void GotoAction::publishVelocity(tf2::Vector3 translation, double rotation)
{
    geometry_msgs::Twist cmdVel;
    cmdVel.linear.x = translation.getX();
    cmdVel.linear.y = translation.getY();
    cmdVel.linear.z = 0;
    cmdVel.angular.x = 0;
    cmdVel.angular.y = 0;
    cmdVel.angular.z = rotation;
    cmdVelPub.publish(cmdVel);
}

GotoPointAction::GotoPointAction(std::string actionName)
    : GotoAction(actionName),
      actionServer(nh, actionName, boost::bind(&GotoPointAction::executeCallback, this, _1), false)
{
    actionServer.start();
}

GotoPointAction::~GotoPointAction()
{
}

void GotoPointAction::executeCallback(const base_controller::GotoPointGoalConstPtr &goal)
{
    ros::Rate rate(10);

    translationSpeedCurve.setAcceleration(0.1);
    translationSpeedCurve.setMinSpeed(0.05);
    translationSpeedCurve.setMaxSpeed(0.5);
    translationSpeedCurve.setLoopPeriod(0.1);

    accelerationLimiter.setMaxAcceleration(0.1);
    accelerationLimiter.setLoopPeriod(0.1);

    while (true)
    {
        if (actionServer.isPreemptRequested() || !ros::ok())
        {
            actionServer.setPreempted();
            break;
        }

        tf2::Transform robotTransform = getRobotTransform(goal->target_frame.data,
                                                          goal->reference_frame.data);

        tf2::Vector3 goalPoint;
        tf2::fromMsg(goal->point, goalPoint);

        tf2::Vector3 displacement = goalPoint - robotTransform.getOrigin();
        displacement.setZ(0);

        double distance = displacement.length();
        translationSpeedCurve.setTargetDistance(distance);
        double speed = translationSpeedCurve.getNextSpeed();

        accelerationLimiter.setTargetSpeedDirection(speed, displacement);

        tf2::Vector3 velocity = accelerationLimiter.getNextVelocity();
        tf2::Transform robotRotation;
        robotRotation.setOrigin(tf2::Vector3(0, 0, 0));
        robotRotation.setRotation(robotTransform.getRotation().inverse());
        velocity = robotRotation * velocity;

        if (fabs(distance) < 0.01)
        {
            actionServer.setSucceeded();
            break;
        }

        publishVelocity(velocity, 0);
    }

    publishVelocity(tf2::Vector3(0,0,0), 0);
}

ResetOdomAction::ResetOdomAction(std::string actionName)
: actionServer(nh, actionName, boost::bind(&ResetOdomAction::executeCallback, this, _1), false) {
    actionServer.start();
}

void ResetOdomAction::executeCallback(const base_controller::ResetOdomGoalConstPtr &goal) {
    std_msgs::Empty msg;
    resetOdomPub.publish(msg);
    ros::Duration(0.5).sleep();
    actionServer.setSucceeded();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle nh;

    cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    resetOdomPub = nh.advertise<std_msgs::Empty>("reset_odom", 1, false);

    GotoPointAction gotoPointAction("goto_point");
    // ResetOdomAction resetOdom("reset_odom");

    ros::spin();

    return 0;
}
