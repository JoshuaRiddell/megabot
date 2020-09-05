#include <base_controller/stepper_power_action.h>
#include <std_msgs/Bool.h>

StepperPowerAction::StepperPowerAction(std::string actionName)
: actionServer(nh, actionName, boost::bind(&StepperPowerAction::executeCallback, this, _1), false) {
    actionServer.start();
    stepperPowerPub = nh.advertise<std_msgs::Bool>("stepper_power", 1, false);
}

void StepperPowerAction::executeCallback(const base_controller::StepperPowerGoalConstPtr &goal) {
    std_msgs::Bool msg;
    msg.data = goal->powerOn;
    stepperPowerPub.publish(msg);

    ros::Duration(0.5).sleep();

    actionServer.setSucceeded();
}
