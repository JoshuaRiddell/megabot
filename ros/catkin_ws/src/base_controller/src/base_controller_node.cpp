#include <base_controller.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <tf2/LinearMath/Vector3.h>
#include <actionlib/server/simple_action_server.h>
#include <base_controller/GotoPointAction.h>
#include <base_controller/speed_curve.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher cmdVelPub;

GotoPointAction::GotoPointAction(std::string name)
    : actionServer(nh, name, boost::bind(&GotoPointAction::executeCallback, this, _1), false),
      actionName(name),
      tfListener(tfBuffer)
{
    actionServer.start();
}

GotoPointAction::~GotoPointAction()
{
}

void GotoPointAction::executeCallback(const base_controller::GotoPointGoalConstPtr &goal)
{
    ros::Rate rate(10);
    bool success = true;
    double currentVelocity = 0;
    SpeedCurve speedCurve(0.1, 0.5, 0.1);

    while (true)
    {
        if (actionServer.isPreemptRequested() || !ros::ok())
        {
            actionServer.setPreempted();
            success = false;
            break;
        }


        geometry_msgs::TransformStamped robotTransformMsg;
        robotTransformMsg = tfBuffer.lookupTransform(goal->target_frame.data, goal->reference_frame.data, ros::Time::now(), ros::Duration(1.0));

        tf2::Transform robotTransform;
        tf2::fromMsg(robotTransformMsg.transform, robotTransform);


        double robotY = robotTransform.getOrigin().getY();
        double targetY = goal->point.y;

        double distance = targetY - robotY;
        speedCurve.setTargetDistance(distance);
        double speed = speedCurve.getNextSpeed();

        geometry_msgs::Twist cmdVel;
        cmdVel.angular.z = 0;
        cmdVel.linear.x = speed;
        cmdVel.linear.y = 0;
        cmdVelPub.publish(cmdVel);
    }

    if (success)
    {
        actionServer.setSucceeded();
    }

    geometry_msgs::Twist cmdVel;
    cmdVel.angular.z = 0;
    cmdVel.linear.x = 0;
    cmdVel.linear.y = 0;
    cmdVelPub.publish(cmdVel);
}

// calculate angular difference between robot and goal
// w = min(maxSpeed, diff * angular_speed_factor)

// calculate linear distance between robot and goal
void publishCmdVel(const sensor_msgs::Joy &msg);

void actionExecute()
{
    geometry_msgs::Twist cmdVel;
    tf2::Vector3 linearVelocity, goalDirection;
    float linearDistance, linearSpeed;
    float angularDistance, angularSpeed;

    // get target position

    SpeedCurve linearSpeedCurve(0.1, 0.5, 0.02), angularSpeedCurve(0.05, 0.2, 0.02);

    // get robot position on frame from TF
    // calculate angular distance from target
    // calculate linear distance from target
    // calculate goal direction
    linearSpeedCurve.setTargetDistance(linearDistance);
    angularSpeedCurve.setTargetDistance(angularDistance);
    linearSpeed = linearSpeedCurve.getNextSpeed();
    angularSpeed = angularSpeedCurve.getNextSpeed();
    linearVelocity = goalDirection * linearSpeed;

    cmdVel.angular.z = angularSpeed;
    cmdVel.linear.x = linearVelocity.getX();
    cmdVel.linear.y = linearVelocity.getY();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle nh;

    cmdVelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);

    GotoPointAction gotoPointAction("goto_point");

    ros::spin();

    return 0;
}
