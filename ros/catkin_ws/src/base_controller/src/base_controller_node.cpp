#include <base_controller.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <math.h>
#include <tf2/LinearMath/Vector3.h>
#include <actionlib/server/simple_action_server.h>

#include <dynamic_reconfigure/server.h>
#include <base_controller/BaseControllerConfig.h>

GotoAction *gotoActionPtr;

GotoPointAction::GotoPointAction(std::string actionName)
    : actionServer(nh, actionName, boost::bind(&GotoPointAction::executeCallback, this, _1), false)
{
    actionServer.start();
}

void GotoPointAction::executeCallback(const base_controller::GotoPointGoalConstPtr &goal) {
    ros::Rate delay(10);

    gotoActionPtr->disable();
    gotoActionPtr->setRobotFrame(goal->reference_frame);
    gotoActionPtr->setTargetFrame(goal->target_frame);
    gotoActionPtr->setGoalPoint(goal->point);
    gotoActionPtr->setDistanceThreshold(goal->distance_threshold);
    gotoActionPtr->enable();

    while (true) {
        if (!ros::ok() || actionServer.isPreemptRequested()) {
            actionServer.setPreempted();
            break;
        }

        if (gotoActionPtr->isReachedGoal()) {
            actionServer.setSucceeded();
            break;
        }
        
        delay.sleep();
    }
}

GotoPoseAction::GotoPoseAction(std::string actionName)
    : actionServer(nh, actionName, boost::bind(&GotoPoseAction::executeCallback, this, _1), false)
{
    actionServer.start();
}

void GotoPoseAction::executeCallback(const base_controller::GotoPoseGoalConstPtr &goal)
{
    ros::Rate delay(10);

    gotoActionPtr->disable();
    gotoActionPtr->setRobotFrame(goal->reference_frame);
    gotoActionPtr->setTargetFrame(goal->target_frame);
    gotoActionPtr->setGoalPoint(goal->point);
    gotoActionPtr->setGoalRotation(goal->rotation);
    gotoActionPtr->setDistanceThreshold(goal->distance_threshold);
    gotoActionPtr->setRotationThreshold(goal->rotation_threshold);
    gotoActionPtr->enable();

    while (true) {
        if (!ros::ok() || actionServer.isPreemptRequested()) {
            actionServer.setPreempted();
            break;
        }

        if (gotoActionPtr->isReachedGoal()) {
            actionServer.setSucceeded();
            break;
        }
        
        delay.sleep();
    }
}

void reconfigureCallback(base_controller::BaseControllerConfig &config, uint32_t level)
{
    gotoActionPtr->setConfig(config);
}

ResetOdomAction::ResetOdomAction(std::string actionName)
: actionServer(nh, actionName, boost::bind(&ResetOdomAction::executeCallback, this, _1), false) {
    actionServer.start();
    resetOdomPub = nh.advertise<std_msgs::Empty>("reset_odom", 1, false);
}

void ResetOdomAction::executeCallback(const base_controller::ResetOdomGoalConstPtr &goal) {
    gotoActionPtr->disable();
    gotoActionPtr->resetPosition();

    std_msgs::Empty msg;
    resetOdomPub.publish(msg);
    ros::Duration(0.5).sleep();

    gotoActionPtr->enable();
    actionServer.setSucceeded();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle nh;

    GotoAction gotoAction;
    gotoActionPtr = &gotoAction;

    GotoPointAction gotoPointAction("goto_point");
    GotoPoseAction gotoPoseAction("goto_pose");
    ResetOdomAction resetOdom("reset_odom");

    dynamic_reconfigure::Server<base_controller::BaseControllerConfig> server;
    dynamic_reconfigure::Server<base_controller::BaseControllerConfig>::CallbackType serverCallback;
    serverCallback = boost::bind(&reconfigureCallback, _1, _2);
    server.setCallback(serverCallback);

    gotoAction.resetControllers();
    gotoAction.disable();

    while (ros::ok()) {
        ros::spinOnce();
        gotoAction.publishNextCmdVel();
        gotoAction.loopDelay();
    }

    gotoAction.stopRobot();

    return 0;
}
