#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <lifter_controller/LiftAction.h>
#include <lifter_controller/GrabAction.h>

class LiftAction {
public:
    LiftAction(std::string actionName);

    void executeCallback(const lifter_controller::LiftGoalConstPtr &goal);

protected:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<lifter_controller::LiftAction> actionServer;
    std::string actionName;
};

class GrabAction {
public:
    GrabAction(std::string actionName);
    ~GrabAction();

    void executeCallback(const lifter_controller::GrabGoalConstPtr &goal);

protected:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<lifter_controller::GrabAction> actionServer;
    std::string actionName;
};
