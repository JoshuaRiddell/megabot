#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <dynamic_reconfigure/server.h>
#include <lifter_controller/LifterConfig.h>
#include <lifter_controller.h>

ros::Publisher stepperPub, servoLeftPub, servoRightPub;
int liftState, grabberLeftState, grabberRightState;
lifter_controller::LifterConfig lifterConfig;

void openServoLeft();
void openServoRight();
void closeServoLeft();
void closeServoRight();
void lifterUp();
void lifterDown();

void reconfigureCallback(lifter_controller::LifterConfig &config, uint32_t level)
{
    lifterConfig = config;
}

void openServoLeft()
{
    std_msgs::Int16 servo;
    servo.data = lifterConfig.servo_left_open;
    servoLeftPub.publish(servo);
}

void openServoRight()
{
    std_msgs::Int16 servo;
    servo.data = lifterConfig.servo_right_open;
    servoRightPub.publish(servo);
}

void closeServoLeft()
{
    std_msgs::Int16 servo;
    servo.data = lifterConfig.servo_left_closed;
    servoLeftPub.publish(servo);
}

void closeServoRight()
{
    std_msgs::Int16 servo;
    servo.data = lifterConfig.servo_right_closed;
    servoRightPub.publish(servo);
}

void lifterUp()
{
    std_msgs::Int16 stepper;
    stepper.data = 1;
    stepperPub.publish(stepper);
}

void lifterDown()
{
    std_msgs::Int16 stepper;
    stepper.data = -1;
    stepperPub.publish(stepper);
}

void lifterLiftCallback(const std_msgs::Int8 &msg)
{
    if (msg.data == 1)
    {
        lifterUp();
    }
    else if (msg.data == -1)
    {
        lifterDown();
    }
}

void lifterGrabberLeftCallback(const std_msgs::Int8 &msg)
{
    if (msg.data == 1)
    {
        openServoLeft();
    }
    else if (msg.data == -1)
    {
        closeServoLeft();
    }
}

void lifterGrabberRightCallback(const std_msgs::Int8 &msg)
{
    if (msg.data == 1)
    {
        openServoRight();
    }
    else if (msg.data == -1)
    {
        closeServoRight();
    }
}

void liftStatusCallback(const std_msgs::Int16 &msg)
{
    liftState = msg.data;
}

void grabberLeftStatusCallback(const std_msgs::Int8 &msg)
{
    grabberLeftState = msg.data;
}

void grabberRightStatusCallback(const std_msgs::Int8 &msg)
{
    grabberRightState = msg.data;
}

LiftAction::LiftAction(std::string actionName)
    : actionName(actionName),
     actionServer(nh, actionName, boost::bind(&LiftAction::executeCallback, this, _1), false)
{
    actionServer.start();
}

void LiftAction::executeCallback(const lifter_controller::LiftGoalConstPtr &goal)
{
    int updateFrequency = 10;
    int timeoutTime = 10;
    int timeoutN = timeoutTime * updateFrequency;
    ros::Rate rate(updateFrequency);
    int requiredState = liftState;

    if (goal->position == goal->UP)
    {
        lifterUp();
        requiredState = goal->UP;
    }
    else if (goal->position == goal->DOWN)
    {
        lifterDown();
        requiredState = goal->DOWN;
    }

    int i = 0;
    while (true) {
        if (actionServer.isPreemptRequested() || !ros::ok())
        {
            actionServer.setPreempted();
            return;
        }

        if (i == timeoutN) {
            actionServer.setPreempted();
            return;
        }

        if (liftState == goal->position) {
            break;
        }

        rate.sleep();

        ++i;
    }

    actionServer.setSucceeded();
}

GrabAction::GrabAction(std::string actionName)
    : actionName(actionName),
    actionServer(nh, actionName, boost::bind(&GrabAction::executeCallback, this, _1), false)
{
    actionServer.start();
}

GrabAction::~GrabAction() {

}

void GrabAction::executeCallback(const lifter_controller::GrabGoalConstPtr &goal)
{
    if (goal->grabber_frame == "left_grabber") {
        if (goal->position == goal->OPEN) {
            openServoLeft();
        } else if (goal->position == goal->CLOSE) {
            closeServoLeft();
        }
    } else if (goal->grabber_frame == "right_grabber") {
        if (goal->position == goal->OPEN) {
            openServoRight();
        } else if (goal->position == goal->CLOSE) {
            closeServoRight();
        }
    }

    ros::Duration(0.5).sleep();
    actionServer.setSucceeded();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lifter_controller");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<lifter_controller::LifterConfig> server;
    dynamic_reconfigure::Server<lifter_controller::LifterConfig>::CallbackType serverCallback;
    serverCallback = boost::bind(&reconfigureCallback, _1, _2);
    server.setCallback(serverCallback);

    LiftAction liftAction("lift");
    GrabAction grabAction("grab");

    ros::Subscriber joySub = nh.subscribe("lifter/lift", 1, &lifterLiftCallback);
    ros::Subscriber grabberLeftSub = nh.subscribe("lifter/grabber_left", 1, &lifterGrabberLeftCallback);
    ros::Subscriber grabberRightSub = nh.subscribe("lifter/grabber_right", 1, &lifterGrabberRightCallback);

    ros::Subscriber liftStatusSub = nh.subscribe("lifter/stepper/status", 1, &liftStatusCallback);

    stepperPub = nh.advertise<std_msgs::Int16>("lifter/stepper", 1, true);
    servoLeftPub = nh.advertise<std_msgs::Int16>("lifter/servo_left", 1, true);
    servoRightPub = nh.advertise<std_msgs::Int16>("lifter/servo_right", 1, true);

    ros::spin();

    return 0;
}
