#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

tf2::Transform increment;

void cmd_vel_callback(const geometry_msgs::Twist& msg) {

    tf2::Quaternion q;

    q.setRPY(
        msg.angular.x / 20.,
        msg.angular.y / 20.,
        msg.angular.z / 20.
    );
    increment.setRotation(q);

    increment.setOrigin(tf2::Vector3(
        msg.linear.x / 20.,
        msg.linear.y / 20.,
        msg.linear.z / 20.
    ));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localisation");
    tf2_ros::TransformBroadcaster br;

    ros::NodeHandle n;
    ros::Rate r(20);

    ros::Subscriber sub = n.subscribe("cmd_vel", 1, cmd_vel_callback);

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "base_link";

    // setup initial state transform
    tf2::Transform state;
    tf2::Quaternion q;
    q.setRPY(0, 0, M_PI_2);
    state.setOrigin(tf2::Vector3(1.2, 0.265, 0));
    state.setRotation(q);

    // setup initial increment transform
    q.setRPY(0,0,0);
    increment.setOrigin(tf2::Vector3(0,0,0));
    increment.setRotation(q);

    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();

        state *= increment;

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.transform = tf2::toMsg(state);

        br.sendTransform(transformStamped);
    }

    return 0;
};