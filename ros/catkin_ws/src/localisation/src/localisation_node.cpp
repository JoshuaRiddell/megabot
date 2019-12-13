#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localisation_node");
    tf2_ros::TransformBroadcaster br;

    ros::NodeHandle n;
    ros::Rate r(10);

    while (ros::ok())
    {
        ros::spinOnce();

        geometry_msgs::TransformStamped transformStamped;

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);
    }

    return 0;
};