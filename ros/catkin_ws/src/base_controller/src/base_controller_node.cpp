#include <base_controller.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <math.h>
#include <tf2/LinearMath/Vector3.h>
#include <actionlib/server/simple_action_server.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher cmdVelPub, resetOdomPub;

GotoPointAction::GotoPointAction(std::string actionName)
    : GotoAction(actionName),
      actionServer(nh, actionName, boost::bind(&GotoPointAction::executeCallback, this, _1), false)
{
    actionServer.start();
}

void GotoPointAction::executeCallback(const base_controller::GotoPointGoalConstPtr &goal) {
    resetControllers();
    setRobotFrame(goal->reference_frame);
    setTargetFrame(goal->target_frame);
    setGoalPoint(goal->point);

    while (true) {
        publishNextCmdVel();

        if (isRosPreempted() || actionServer.isPreemptRequested()) {
            actionServer.setPreempted();
            break;
        }

        if (isReachedGoal()) {
            actionServer.setSucceeded();
            break;
        }

        loopDelay();
    }

    stopRobot();
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

    if (fabs(distance) < 0.02)
    {
        actionServer.setSucceeded();
        break;
    }

    publishVelocity(velocity, 0);
}

publishVelocity(tf2::Vector3(0,0,0), 0);

GotoPoseAction::GotoPoseAction(std::string actionName)
    : GotoAction(actionName),
      actionServer(nh, actionName, boost::bind(&GotoPoseAction::executeCallback, this, _1), false)
{
    actionServer.start();
}

GotoPoseAction::~GotoPoseAction()
{
}

void GotoPoseAction::executeCallback(const base_controller::GotoPoseGoalConstPtr &goal)
{
    ros::Rate rate(20);

    translationSpeedCurve.setAcceleration(0.1);
    translationSpeedCurve.setMinSpeed(0.03);
    translationSpeedCurve.setMaxSpeed(0.5);
    translationSpeedCurve.setLoopPeriod(0.05);

    accelerationLimiter.setMaxAcceleration(0.5);
    accelerationLimiter.setLoopPeriod(0.05);

    rotationSpeedCurve.setAcceleration(0.1);
    rotationSpeedCurve.setMinSpeed(0.05);
    rotationSpeedCurve.setMaxSpeed(0.03);
    rotationSpeedCurve.setLoopPeriod(0.05);

    accelerationLimiter.reset();
    translationSpeedCurve.reset();

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

        double goalRotation = goal->rotation;

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

        double rotationDifference = robotTransform.getRotation().getAngle();

        velocity = robotRotation * velocity;

        if (fabs(distance) < 0.02)
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







// void callback(goal) {
//     resetControllers();

//     while (true) {
//         updateTranslationGoal(goal->point);
//         updateRotationGoal(goal->rotation);
//         publishCmdVel();
//         loopDelay();

//         if (shouldExit()) {
//             break;
//         }
//     }

//     stopRobot();
// }







int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle nh;

    // cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    resetOdomPub = nh.advertise<std_msgs::Empty>("reset_odom", 1, false);

    GotoPointAction gotoPointAction("goto_point");
    GotoPoseAction gotoPoseAction("goto_pose");
    ResetOdomAction resetOdom("reset_odom");

    ros::spin();

    return 0;
}
