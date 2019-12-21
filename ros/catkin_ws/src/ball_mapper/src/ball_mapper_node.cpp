#include <ros/ros.h>
#include <ball_mapper/ball_tracker.h>
#include <visualization_msgs/Marker.h>
#include <ball_msgs/BallView.h>
#include <ball_msgs/BallMap.h>
#include <geometry_extended_msgs/PointArray.h>

std::list<BallTracker> ball_trackers;
ros::Publisher ball_map_pub, ball_map_vis_pub;
std::vector<tf2::Vector3> camera_view;

void convert_msg_to_tf(ball_msgs::BallView msg, ros::Time &stamp,
                        std::vector<tf2::Vector3> &camera_view,
                        std::vector<tf2::Vector3> &locations) {
    stamp = msg.header.stamp;

    for (int i = 0; i < msg.camera_view.size(); ++i) {
        tf2::Vector3 v(
            msg.camera_view[i].x,
            msg.camera_view[i].y,
            msg.camera_view[i].z
        );
        camera_view.push_back(v);
    }

    for (int i = 0; i < msg.locations.size(); ++i) {
        tf2::Vector3 v(
            msg.locations[i].x,
            msg.locations[i].y,
            msg.locations[i].z
        );
        locations.push_back(v);
    }
}

void ball_view_callback(const ball_msgs::BallView& msg) {
    ros::Time stamp;
    std::vector<tf2::Vector3> camera_view, locations;

    convert_msg_to_tf(msg, stamp, camera_view, locations);


    auto ball_tracker = ball_trackers.begin();
    while (ball_tracker != ball_trackers.end()) {
        // update tracker with new locations
        if (ball_tracker->update(stamp, locations, camera_view)) {
            ball_tracker = ball_trackers.erase(ball_tracker);
        } else {
            ball_tracker++;
        }
    }

    for (int i = 0; i < locations.size(); ++i) {
        ball_trackers.push_back(BallTracker(stamp, locations[i]));
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

    ball_msgs::BallMap out_msg;
    out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = msg.header.frame_id;
    for (auto ball_tracker = ball_trackers.begin(); ball_tracker != ball_trackers.end(); ++ball_tracker) {
        if (ball_tracker->location_valid()) {

            tf2::Vector3 loc = ball_tracker->get_location();
            geometry_msgs::Point p;
            p.x = loc.x();
            p.y = loc.y();
            p.z = loc.z();

            out_msg.locations.push_back(p);
            vis_msg.points.push_back(p);
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
    ros::Subscriber ball_view_sub = n.subscribe("ball_view", 20, ball_view_callback);

    // point array publisher
    ball_map_pub = n.advertise<ball_msgs::BallMap>("ball_map", 5);
    ball_map_vis_pub = n.advertise<visualization_msgs::Marker>("ball_map_vis", 5);

    ros::spin();
}

