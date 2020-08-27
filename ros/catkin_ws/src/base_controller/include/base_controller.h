#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <base_controller/GotoPointAction.h>
#include <base_controller/GotoPoseAction.h>
#include <base_controller/ResetOdomAction.h>
#include <base_controller/goto_action.h>

#include <tf2/LinearMath/Transform.h>






class GotoPointAction : public GotoAction {
public:
    GotoPointAction(std::string actionName);

    void executeCallback(const base_controller::GotoPointGoalConstPtr &goal);

private:
    actionlib::SimpleActionServer<base_controller::GotoPointAction> actionServer;
    base_controller::GotoPointFeedback feedback;
    base_controller::GotoPointResult result;
};

class GotoPoseAction : public GotoAction {
public:
    GotoPoseAction(std::string actionName);

    void executeCallback(const base_controller::GotoPoseGoalConstPtr &goal);

private:
    actionlib::SimpleActionServer<base_controller::GotoPoseAction> actionServer;
    base_controller::GotoPoseFeedback feedback;
    base_controller::GotoPoseResult result;
};

class ResetOdomAction {
public:
    ResetOdomAction(std::string actionName);

private:
    void executeCallback(const base_controller::ResetOdomGoalConstPtr &goal);

    ros::NodeHandle nh;
    actionlib::SimpleActionServer<base_controller::ResetOdomAction> actionServer;
};
