#include <string>
#include <ros/ros.h>
#include <base_controller/StepperPowerAction.h>
#include <actionlib/server/simple_action_server.h>

class StepperPowerAction {
public:
    StepperPowerAction(std::string actionName);

private:
    void executeCallback(const base_controller::StepperPowerGoalConstPtr &goal);

    ros::NodeHandle nh;
    actionlib::SimpleActionServer<base_controller::StepperPowerAction> actionServer;

    ros::Publisher stepperPowerPub;
};
