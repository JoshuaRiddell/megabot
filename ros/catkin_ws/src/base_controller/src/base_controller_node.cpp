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
#include <base_controller/reset_odom_action.h>

GotoAction gotoAction;

GotoPointAction::GotoPointAction(std::string actionName)
    : actionServer(nh, actionName, boost::bind(&GotoPointAction::executeCallback, this, _1), false)
{
    actionServer.start();
}

void GotoPointAction::executeCallback(const base_controller::GotoPointGoalConstPtr &goal) {
    ros::Rate delay(10);

    gotoAction.setRobotFrame(goal->reference_frame);
    gotoAction.setTargetFrame(goal->target_frame);
    gotoAction.setGoalPoint(goal->point);

    while (true) {
        if (!ros::ok() || actionServer.isPreemptRequested()) {
            actionServer.setPreempted();
            break;
        }

        if (gotoAction.isReachedGoal()) {
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

    gotoAction.setRobotFrame(goal->reference_frame);
    gotoAction.setTargetFrame(goal->target_frame);
    gotoAction.setGoalPoint(goal->point);
    gotoAction.setGoalRotation(goal->rotation);

    while (true) {
        if (!ros::ok() || actionServer.isPreemptRequested()) {
            actionServer.setPreempted();
            break;
        }

        if (gotoAction.isReachedGoal()) {
            actionServer.setSucceeded();
            break;
        }
        
        delay.sleep();
    }
}

void reconfigureCallback(base_controller::BaseControllerConfig &config, uint32_t level)
{
    gotoAction.setConfig(config);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle nh;

    GotoPointAction gotoPointAction("goto_point");
    GotoPoseAction gotoPoseAction("goto_pose");
    ResetOdomAction resetOdom("reset_odom");

    dynamic_reconfigure::Server<base_controller::BaseControllerConfig> server;
    dynamic_reconfigure::Server<base_controller::BaseControllerConfig>::CallbackType serverCallback;
    serverCallback = boost::bind(&reconfigureCallback, _1, _2);
    server.setCallback(serverCallback);

    gotoAction.resetControllers();

    while (ros::ok()) {
        ros::spinOnce();
        gotoAction.publishNextCmdVel();
        gotoAction.loopDelay();
    }

    gotoAction.stopRobot();

    return 0;
}
