#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

tf2::Transform increment;
const int rate = 50;

void cmdVelCallback(const geometry_msgs::Twist& msg) {

    tf2::Quaternion q;

    q.setRPY(
        msg.angular.x / rate,
        msg.angular.y / rate,
        msg.angular.z / rate
    );
    increment.setRotation(q);

    increment.setOrigin(tf2::Vector3(
        msg.linear.x / rate,
        msg.linear.y / rate,
        msg.linear.z / rate
    ));
}

int main(int argc, char **argv)
{
    tf2::Quaternion q;
    ros::init(argc, argv, "odometry");
    tf2_ros::TransformBroadcaster br;

    ros::NodeHandle n;
    ros::Rate r(rate);

    // setup initial increment transform
    q.setRPY(0,0,0);
    increment.setOrigin(tf2::Vector3(0,0,0));
    increment.setRotation(q);

    ros::Subscriber sub = n.subscribe("cmd_vel", 1, cmdVelCallback);

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_footprint";

    // setup initial state transform
    tf2::Transform state;
    q.setRPY(0, 0, M_PI_2);
    state.setOrigin(tf2::Vector3(1.2, 0.265, 0));
    state.setRotation(q);

    while (ros::ok())
    {
        ros::spinOnce();

        state *= increment;

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.transform = tf2::toMsg(state);

        br.sendTransform(transformStamped);
        r.sleep();
    }

    return 0;
};
