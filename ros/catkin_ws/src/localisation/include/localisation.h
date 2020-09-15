#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <localisation/LineCalibrationAction.h>

class LineCalibrationAction {
public:
    LineCalibrationAction(std::string actionName);

    void executeCallback(const localisation::LineCalibrationGoalConstPtr &goal);

private:
    ros::NodeHandle nh;

    actionlib::SimpleActionServer<localisation::LineCalibrationAction> actionServer;
    localisation::LineCalibrationFeedback feedback;
    localisation::LineCalibrationResult result;
};
