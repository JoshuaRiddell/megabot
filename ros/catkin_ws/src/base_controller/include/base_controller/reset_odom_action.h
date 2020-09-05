#include <string>
#include <ros/ros.h>
#include <base_controller/ResetOdomAction.h>
#include <actionlib/server/simple_action_server.h>

class ResetOdomAction {
public:
    ResetOdomAction(std::string actionName);

private:
    void executeCallback(const base_controller::ResetOdomGoalConstPtr &goal);

    ros::NodeHandle nh;
    actionlib::SimpleActionServer<base_controller::ResetOdomAction> actionServer;

    ros::Publisher resetOdomPub;
};
