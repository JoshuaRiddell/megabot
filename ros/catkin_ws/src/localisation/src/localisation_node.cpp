#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

tf2::Transform state;
tf2::Transform increment;
std::vector<tf2::Quaternion> map_line_quats;
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
    tf2::Quaternion line_quat;
    tf2::convert(msg.pose.orientation, line_quat);

    tf2::Vector3 line_position;
    tf2::convert(msg.pose.position, line_position);

    auto map_line_angle_it = map_line_quats.begin();
    double minimum_angle = 999;
    int closest_index = -1;

    for (; map_line_angle_it != map_line_quats.end(); ++map_line_angle_it) {
        double angle = map_line_angle_it->angleShortestPath(line_quat);
        
        // check if the other direction is a shorter angle
        if (M_PI - angle < angle) {
            angle = M_PI - angle;
        }

        if (angle < minimum_angle) {
            closest_index = map_line_angle_it - map_line_quats.begin();
            minimum_angle = angle;
        }
    }

    tf2::Quaternion map_line_quat = map_line_quats[closest_index];
    tf2::Vector3 map_line_origin = map_line_origins[closest_index];

    // work in y/x gradients since most lines are close to vertical
    double _, map_angle, line_angle;

    tf2::Matrix3x3(map_line_quat).getRPY(_, _, map_angle);
    double m1_inv = tan(M_PI_2 - map_angle);
    double xc1 = map_line_origin.getX() - map_line_origin.getY() * m1_inv;

    tf2::Matrix3x3(line_quat).getRPY(_, _, line_angle);
    double m2_inv = tan(M_PI_2 - line_angle);
    double xc2 = line_position.getX() - line_position.getY() * m2_inv;

    double y = (xc2 - xc1) / (m1_inv - m2_inv);
    double x = y * m1_inv + xc1;

    double angle_diff = map_angle - line_angle;
    if (angle_diff > M_PI_2) {
        angle_diff = M_PI - angle_diff;
    }
    
    tf2::Quaternion q;
    q.setRPY(0,0,angle_diff);

    tf2::Vector3 tmp = state.getOrigin() - tf2::Vector3(x,y,0);
    tf2::Matrix3x3 rot;
    rot.setRPY(0,0,angle_diff);
    tf2::Vector3 tmp2 = rot * tmp;
    
    tf2::Transform inc;
    tf2::Vector3 diff = tmp2 - tmp;
    inc.setOrigin(diff);
    inc.setRotation(q);

    std::cout << closest_index << "," << angle_diff <<
            // "; x1=y*" << m1_inv << "+" << xc1 <<
            "; x2=y*" << m2_inv << "+" << xc2 <<
            "; " << diff.getX() << "," << diff.getY() <<
            "; " << state.getOrigin().getX() << "," << state.getOrigin().getY();
            // "; int: " << x << "," << y << 
            // std::endl;

    // if the angle difference is too large this method becomes unstable
    if (minimum_angle > M_PI_4) {
        std::cout << std::endl;
        return;
    }

    // if minimum angle is too small we're already close enough
    if (minimum_angle < 0.05) {
        std::cout << std::endl;
        return;
    }

    state *= inc;

    std::cout << 
            "; " << state.getOrigin().getX() << "," << state.getOrigin().getY() <<
            std::endl;


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
    map_line_quats.push_back(q_tmp);
    q_tmp.setRPY(0, 0, -M_PI/4);
    map_line_quats.push_back(q_tmp);
    q_tmp.setRPY(0, 0, -3*M_PI/4);
    map_line_quats.push_back(q_tmp);

    map_line_origins.push_back(tf2::Vector3(1.2, 0, 0));
    map_line_origins.push_back(tf2::Vector3(0, 2.4, 0));
    map_line_origins.push_back(tf2::Vector3(2.4, 0, 0));

    // setup initial state transform
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
