#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <base_controller/GotoPointAction.h>
#include <tf2_ros/transform_listener.h>

class GotoPointAction {
public:
    GotoPointAction(std::string name);
    ~GotoPointAction();

    void executeCallback(const base_controller::GotoPointGoalConstPtr &goal);

private:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<base_controller::GotoPointAction> actionServer;
    std::string actionName;
    base_controller::GotoPointFeedback feedback;
    base_controller::GotoPointResult result;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
};
