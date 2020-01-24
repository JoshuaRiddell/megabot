#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

tf2::Transform increment;
std::vector<tf2::Quaternion> map_line_angles;
std::vector<tf2::Vector3> map_line_origins;

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

void line_callback(const geometry_msgs::PoseStamped &msg) {
    tf2::Quaternion line_angle;
    tf2::convert(msg.pose.orientation, line_angle);

    auto map_line_angle_it = map_line_angles.begin();
    double minimum_angle = 999;
    int closest_index = -1;

    for (; map_line_angle_it != map_line_angles.end(); ++map_line_angle_it) {
        double angle = map_line_angle_it->angleShortestPath(line_angle);
        
        // check if the other direction is a shorter angle
        if (M_PI - angle < angle) {
            angle = M_PI - angle;
        }

        if (angle < minimum_angle) {
            closest_index = map_line_angle_it - map_line_angles.begin();
            minimum_angle = angle;
        }
    }

    if (minimum_angle > M_PI_4) {
        return;
    }

    tf2::Quaternion map_line_angle = map_line_angles[closest_index];
    tf2::Vector3 map_line_origin = map_line_origins[closest_index];

    

    std::cout << closest_index << ", " << minimum_angle << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localisation");
    tf2_ros::TransformBroadcaster br;

    ros::NodeHandle n;
    ros::Rate r(20);

    ros::Subscriber sub = n.subscribe("/cmd_vel", 1, cmd_vel_callback);
    ros::Subscriber line_sub = n.subscribe("line", 1, line_callback);

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "base_link";

    // load in lines on the map
    tf2::Quaternion q_tmp;
    q_tmp.setRPY(0, 0, M_PI/2);
    map_line_angles.push_back(q_tmp);
    q_tmp.setRPY(0, 0, -M_PI/4);
    map_line_angles.push_back(q_tmp);
    q_tmp.setRPY(0, 0, -3*M_PI/4);
    map_line_angles.push_back(q_tmp);

    map_line_origins.push_back(tf2::Vector3(1.2, 0, 0));
    map_line_origins.push_back(tf2::Vector3(0, 2.4, 0));
    map_line_origins.push_back(tf2::Vector3(2.4, 0, 0));

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

        state *= increment;

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.transform = tf2::toMsg(state);

        br.sendTransform(transformStamped);
        r.sleep();
    }

    return 0;
};
