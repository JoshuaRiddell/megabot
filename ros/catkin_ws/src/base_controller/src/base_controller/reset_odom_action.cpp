#include <base_controller/reset_odom_action.h>

ResetOdomAction::ResetOdomAction(std::string actionName)
: actionServer(nh, actionName, boost::bind(&ResetOdomAction::executeCallback, this, _1), false) {
    actionServer.start();
    resetOdomPub = nh.advertise<std_msgs::Empty>("reset_odom", 1, false);
}

void ResetOdomAction::executeCallback(const base_controller::ResetOdomGoalConstPtr &goal) {
    std_msgs::Empty msg;
    resetOdomPub.publish(msg);
    ros::Duration(0.5).sleep();
    actionServer.setSucceeded();
}
