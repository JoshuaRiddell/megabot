#include <ros/ros.h>
#include <ball_mapper/ball_tracker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_extended_msgs/PointArray.h>
#include <visualization_msgs/Marker.h>

std::list<BallTracker> ball_trackers;
ros::Publisher ball_map_pub, ball_map_vis_pub;

void ball_callback(const geometry_msgs::PointStamped& msg) {
    // try and add this point to existing trackers and delete expired trackers
    bool tracker_found = false;
    for (auto ball_tracker = ball_trackers.begin(); ball_tracker != ball_trackers.end(); ++ball_tracker) {
        // attempt to add sample
        if (!tracker_found && ball_tracker->add_sample(msg)) {
            tracker_found = true;
        }

        // delete if expired
        // if (ball_tracker->expired()) {
        //     ball_trackers.erase(ball_tracker);
        // }
    }

    // if no tracker matched, add a new one to the list
    if (!tracker_found) {
        ball_trackers.push_back(BallTracker(msg));
    }

    std::cout << ball_trackers.size() << std::endl;

    // publish valid tracker locations
    visualization_msgs::Marker vis_msg;
    vis_msg.header.stamp = ros::Time::now();
    vis_msg.header.frame_id = msg.header.frame_id;
    vis_msg.ns = "ball_map_vis";
    vis_msg.id = 0;
    vis_msg.type = vis_msg.SPHERE_LIST;
    vis_msg.action = vis_msg.MODIFY;
    vis_msg.pose.orientation.w = 1.0;
    vis_msg.scale.x = 0.066;
    vis_msg.scale.y = 0.066;
    vis_msg.scale.z = 0.066;
    vis_msg.frame_locked = false;
    vis_msg.color.a = 1.0;
    vis_msg.color.g = 1.0;

    geometry_extended_msgs::PointArray out_msg;
    out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = msg.header.frame_id;
    for (auto ball_tracker = ball_trackers.begin(); ball_tracker != ball_trackers.end(); ++ball_tracker) {
        if (ball_tracker->is_valid()) {
            out_msg.points.push_back(ball_tracker->get_location());
            vis_msg.points.push_back(ball_tracker->get_location());
        }
    }
    ball_map_pub.publish(out_msg);
    ball_map_vis_pub.publish(vis_msg);
}

int main(int argc, char **argv)
{
    // node setup
    ros::init(argc, argv, "ball_mapper");
    ros::NodeHandle n;

    // ball location subscriber
    ros::Subscriber ball_sub = n.subscribe("ball", 20, ball_callback);

    // point array publisher
    ball_map_pub = n.advertise<geometry_extended_msgs::PointArray>("ball_map", 5);
    ball_map_vis_pub = n.advertise<visualization_msgs::Marker>("ball_map_vis", 5);

    ros::spin();
}

