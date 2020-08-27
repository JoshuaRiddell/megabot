class ResetOdomAction {
public:
    ResetOdomAction(std::string actionName);

private:
    void executeCallback(const base_controller::ResetOdomGoalConstPtr &goal);

    ros::NodeHandle nh;
    actionlib::SimpleActionServer<base_controller::ResetOdomAction> actionServer;
};
