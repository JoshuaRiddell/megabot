#include <ros/ros.h>
#include <ball_mapper/ball_tracker.h>
#include <geometry_msgs/PointStamped.h>


void ball_callback(const geometry_msgs::PointStamped& msg) {

}

int main(int argc, char **argv)
{
    // node setup
    ros::init(argc, argv, "ball_mapper");
    ros::NodeHandle n;

    // ball location subscriber
    ros::Subscriber ball_sub = n.subscribe("ball", 20, ball_callback);

    ros::spin();
}

