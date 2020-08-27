#include <base_controller/goto_action.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define LOOP_RATE 20

GotoAction::GotoAction(std::string actionName)
    : tfListener(tfBuffer),
      actionName(actionName),
      loopRate(LOOP_RATE)
{
    double loopPeriod = 1. / LOOP_RATE;

    translationSpeedCurve.setAcceleration(0.1);
    translationSpeedCurve.setMinSpeed(0.03);
    translationSpeedCurve.setMaxSpeed(0.5);
    translationSpeedCurve.setLoopPeriod(loopPeriod);

    accelerationLimiter.setMaxAcceleration(0.5);
    accelerationLimiter.setLoopPeriod(loopPeriod);

    rotationSpeedCurve.setAcceleration(0.1);
    rotationSpeedCurve.setMinSpeed(0.05);
    rotationSpeedCurve.setMaxSpeed(0.03);
    rotationSpeedCurve.setLoopPeriod(loopPeriod);

    cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmdVel", 1, true);
}

void GotoAction::resetControllers()
{
    translationSpeedCurve.reset();
    accelerationLimiter.reset();
    rotationSpeedCurve.reset();
    resetCmdVel();
}

void GotoAction::setRobotFrame(std_msgs::String robotFrameMsg)
{
    robotFrame = robotFrameMsg.data;
}

void GotoAction::setTargetFrame(std_msgs::String targetFrameMsg)
{
    targetFrame = targetFrameMsg.data;
}

void GotoAction::setGoalPoint(geometry_msgs::Point pointMsg)
{
    tf2::fromMsg(pointMsg, goalPoint);
}

void GotoAction::setGoalRotation(geometry_msgs::Quaternion angleMsg)
{
    tf2::fromMsg(angleMsg, goalRotation);
}

void GotoAction::publishNextCmdVel()
{
    updateCmdVel();
    publishCmdVel();
}

void GotoAction::updateCmdVel() {
    tf2::Transform robotTransform = getTransform(targetFrame, robotFrame);
    updateTranslationVelocity(robotTransform);
    updateRotationVelocity(robotTransform);
}

void GotoAction::updateTranslationVelocity(tf2::Transform robotTransform) {
    tf2::Vector3 displacement = goalPoint - robotTransform.getOrigin();
    displacement.setZ(0);

    double distance = displacement.length();

    if (fabs(distance) < distanceThreshold) {
        hasReachedTranslationGoal = true;
        cmdVel.linear.x = 0;
        cmdVel.linear.y = 0;
        cmdVel.linear.z = 0;
        return;
    } else {
        hasReachedTranslationGoal = false;
    }

    translationSpeedCurve.setTargetDistance(distance);
    double speed = translationSpeedCurve.getNextSpeed();

    accelerationLimiter.setTargetSpeedDirection(speed, displacement);
    tf2::Vector3 velocity = accelerationLimiter.getNextVelocity();

    tf2::Transform robotRotation;
    robotRotation.setOrigin(tf2::Vector3(0,0,0));
    robotRotation.setRotation(robotTransform.getRotation().inverse());
    velocity = robotTransform * velocity;

    cmdVel.linear.x = velocity.getX();
    cmdVel.linear.y = velocity.getY();
    cmdVel.linear.z = velocity.getZ();
}

void GotoAction::updateRotationVelocity(tf2::Transform robotTransform) {
    tf2::Quaternion angularDisplacement = goalRotation - robotTransform.getRotation();
    double angularDistance = angularDisplacement.getAngle() - M_PI;

    if (fabs(angularDistance) < rotationThreshold) {
        hasReachedRotationGoal = true;
        cmdVel.angular.z = 0;
        return;
    } else {
        hasReachedRotationGoal = false;
        rotationSpeedCurve.setTargetDistance(angularDistance);
        double angularSpeed = rotationSpeedCurve.getNextSpeed();
        cmdVel.angular.z = angularSpeed;
    }
}

void GotoAction::publishCmdVel() {
    cmdVelPub.publish(cmdVel);
}

void GotoAction::loopDelay()
{
    loopRate.sleep();
}

void GotoAction::stopRobot()
{
    resetCmdVel();
    publishCmdVel();
}

void GotoAction::resetCmdVel() {
    cmdVel.linear.x = 0;
    cmdVel.linear.y = 0;
    cmdVel.linear.z = 0;
    cmdVel.angular.x = 0;
    cmdVel.angular.y = 0;
    cmdVel.angular.z = 0;
}

bool GotoAction::isRosPreempted()
{
    return !ros::ok();
}

bool GotoAction::isReachedGoal()
{
    return hasReachedTranslationGoal && hasReachedRotationGoal;
}

tf2::Transform GotoAction::getTransform(std::string targetFrame, std::string referenceFrame)
{
    geometry_msgs::TransformStamped robotTransformMsg;
    robotTransformMsg = tfBuffer.lookupTransform(targetFrame, referenceFrame, ros::Time::now(), ros::Duration(1.0));
    tf2::Transform robotTransform;
    tf2::fromMsg(robotTransformMsg.transform, robotTransform);
    return robotTransform;
}
