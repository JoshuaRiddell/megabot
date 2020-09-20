#include <base_controller/goto_action.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define LOOP_RATE 20

ecl::Semaphore semaphore("goto_action");

GotoAction::GotoAction()
    : tfListener(tfBuffer),
      loopRate(LOOP_RATE),
      distanceThreshold(0.01),
      rotationThreshold(0.05)
{
    double loopPeriod = 1. / LOOP_RATE;

    translationSpeedCurve.setAcceleration(0.1);
    translationSpeedCurve.setMinSpeed(0.1);
    translationSpeedCurve.setMaxSpeed(0.5);
    translationSpeedCurve.setLoopPeriod(loopPeriod);

    accelerationLimiter.setMaxAcceleration(0.5);
    accelerationLimiter.setLoopPeriod(loopPeriod);

    rotationSpeedCurve.setAcceleration(0.1);
    rotationSpeedCurve.setMinSpeed(0.1);
    rotationSpeedCurve.setMaxSpeed(0.5);
    rotationSpeedCurve.setLoopPeriod(loopPeriod);

    cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

    resetPosition();
}

void GotoAction::resetPosition() {
    robotFrame = "base_footprint";
    targetFrame = "odom";
    goalPoint = tf2::Vector3(1.2, 0.2, 0);
    goalRotation.setEuler(0, 0, M_PI_2);
}

void GotoAction::setConfig(base_controller::BaseControllerConfig &config) {
    translationSpeedCurve.setAcceleration(config.translation_speed_curve_acceleration);
    translationSpeedCurve.setMinSpeed(config.translation_speed_curve_min_speed);
    translationSpeedCurve.setMaxSpeed(config.translation_speed_curve_max_speed);
    translationSpeedCurve.setDistanceCoefficient(config.translation_speed_curve_distance_coefficient);
    translationSpeedCurve.setDistanceDeadband(config.translation_speed_curve_distance_deadband);

    accelerationLimiter.setMaxAcceleration(config.translation_acceleration_limiter_max_acceleration);

    rotationSpeedCurve.setAcceleration(config.angle_speed_curve_acceleration);
    rotationSpeedCurve.setMinSpeed(config.angle_speed_curve_min_speed);
    rotationSpeedCurve.setMaxSpeed(config.angle_speed_curve_max_speed);
    rotationSpeedCurve.setDistanceCoefficient(config.angle_speed_curve_distance_coefficient);
    rotationSpeedCurve.setDistanceDeadband(config.angle_speed_curve_distance_deadband);
}

void GotoAction::resetControllers()
{
    translationSpeedCurve.reset();
    accelerationLimiter.reset();
    rotationSpeedCurve.reset();
    resetCmdVel();
}

void GotoAction::disable() {
    enabled = false;
}

void GotoAction::enable() {
    enabled = true;
}

void GotoAction::setRobotFrame(std::string robotFrameMsg)
{
    robotFrame = robotFrameMsg;
}

void GotoAction::setTargetFrame(std::string targetFrameMsg)
{
    targetFrame = targetFrameMsg;
}

void GotoAction::setGoalPoint(geometry_msgs::Point pointMsg)
{
    semaphore.lock();
    tf2::fromMsg(pointMsg, goalPoint);
    hasReachedTranslationGoal = false;
    semaphore.unlock();
}

void GotoAction::setDistanceThreshold(double distanceThreshold) {
    semaphore.lock();
    this->distanceThreshold = distanceThreshold;
    hasReachedTranslationGoal = false;
    semaphore.unlock();
}

void GotoAction::setGoalRotation(geometry_msgs::Quaternion angleMsg)
{
    semaphore.lock();
    tf2::fromMsg(angleMsg, goalRotation);
    hasReachedRotationGoal = false;
    semaphore.unlock();
}

void GotoAction::setRotationThreshold(double rotationThreshold) {
    semaphore.lock();
    this->rotationThreshold = rotationThreshold;
    hasReachedRotationGoal = false;
    semaphore.unlock();
}

void GotoAction::publishNextCmdVel()
{
    semaphore.lock();
    if (enabled) {
        updateCmdVel();
        publishCmdVel();
    } else {
        stopRobot();
    }
    semaphore.unlock();
}

void GotoAction::updateCmdVel() {
    try {
        tf2::Transform robotTransform = getTransform(targetFrame, robotFrame);
        updateTranslationVelocity(robotTransform);
        updateRotationVelocity(robotTransform);
    } catch (std::exception &e) {
        ROS_ERROR("%s", e.what());
    }
}

void GotoAction::updateTranslationVelocity(tf2::Transform robotTransform) {
    tf2::Vector3 displacement = goalPoint - robotTransform.getOrigin();
    displacement.setZ(0);

    double distance = displacement.length();

    hasReachedTranslationGoal = fabs(distance) < distanceThreshold;

    translationSpeedCurve.setTargetDistance(distance);
    double speed = translationSpeedCurve.getNextSpeed();

    accelerationLimiter.setTargetSpeedDirection(speed, displacement);
    tf2::Vector3 velocity = accelerationLimiter.getNextVelocity();

    tf2::Transform robotRotation;
    robotRotation.setOrigin(tf2::Vector3(0,0,0));
    robotRotation.setRotation(robotTransform.getRotation().inverse());
    velocity = robotRotation * velocity;

    cmdVel.linear.x = velocity.getX();
    cmdVel.linear.y = velocity.getY();
    cmdVel.linear.z = velocity.getZ();
}

void GotoAction::updateRotationVelocity(tf2::Transform robotTransform) {
    tf2::Quaternion angularDisplacement = goalRotation * robotTransform.getRotation().inverse();

    double roll, pitch, yaw;
    tf2::Matrix3x3(angularDisplacement).getRPY(roll, pitch, yaw);
    double angularDistance = yaw;

    hasReachedRotationGoal = fabs(angularDistance) < rotationThreshold;

    rotationSpeedCurve.setTargetDistance(angularDistance);
    double angularSpeed = rotationSpeedCurve.getNextSpeed();
    cmdVel.angular.z = angularSpeed;
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
