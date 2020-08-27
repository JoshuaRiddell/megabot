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

GotoPoseAction::GotoPoseAction(std::string actionName)
    : GotoAction(actionName),
      actionServer(nh, actionName, boost::bind(&GotoPoseAction::executeCallback, this, _1), false)
{
    actionServer.start();
}

void GotoPoseAction::executeCallback(const base_controller::GotoPoseGoalConstPtr &goal)
{
    resetControllers();
    setRobotFrame(goal->reference_frame);
    setTargetFrame(goal->target_frame);
    setGoalPoint(goal->point);
    setGoalRotation(goal->rotation);

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

    resetOdomPub = nh.advertise<std_msgs::Empty>("reset_odom", 1, false);

    GotoPointAction gotoPointAction("goto_point");
    GotoPoseAction gotoPoseAction("goto_pose");
    ResetOdomAction resetOdom("reset_odom");

    ros::spin();

    return 0;
}
