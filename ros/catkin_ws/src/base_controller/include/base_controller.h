#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <base_controller/GotoPointAction.h>
#include <base_controller/GotoPoseAction.h>
#include <base_controller/goto_action.h>

#include <tf2/LinearMath/Transform.h>

class GotoPointAction {
public:
    GotoPointAction(std::string actionName);

    void executeCallback(const base_controller::GotoPointGoalConstPtr &goal);

private:
    ros::NodeHandle nh;

    actionlib::SimpleActionServer<base_controller::GotoPointAction> actionServer;
    base_controller::GotoPointFeedback feedback;
    base_controller::GotoPointResult result;
};

class GotoPoseAction {
public:
    GotoPoseAction(std::string actionName);

    void executeCallback(const base_controller::GotoPoseGoalConstPtr &goal);

private:
    ros::NodeHandle nh;

    actionlib::SimpleActionServer<base_controller::GotoPoseAction> actionServer;
    base_controller::GotoPoseFeedback feedback;
    base_controller::GotoPoseResult result;
};

