#include <ros/ros.h>
#include <ball_mapper/ball_tracker.h>
#include <visualization_msgs/Marker.h>
#include <ball_msgs/BallView.h>
#include <ball_msgs/BallMap.h>
#include <ball_mapper/ClosestBall.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

std::list<BallTracker> ball_trackers;
ros::Publisher ball_map_pub, ball_map_vis_pub;
std::vector<tf2::Vector3> camera_view;
std::string referenceFrame = "map";
tf2_ros::Buffer tfBuffer;

tf2::Vector3 getLocation(std::string startFrame, std::string endFrame);

void convert_msg_to_tf(ball_msgs::BallView msg, ros::Time &stamp,
                       std::vector<tf2::Vector3> &camera_view,
                       std::vector<tf2::Vector3> &locations)
{
    stamp = msg.header.stamp;

    for (int i = 0; i < msg.camera_view.size(); ++i)
    {
        tf2::Vector3 v(
            msg.camera_view[i].x,
            msg.camera_view[i].y,
            msg.camera_view[i].z);
        camera_view.push_back(v);
    }

    for (int i = 0; i < msg.locations.size(); ++i)
    {
        tf2::Vector3 v(
            msg.locations[i].x,
            msg.locations[i].y,
            msg.locations[i].z);
        locations.push_back(v);
    }
}

void ball_view_callback(const ball_msgs::BallView &msg)
{
    ros::Time stamp;
    std::vector<tf2::Vector3> camera_view, locations;

    convert_msg_to_tf(msg, stamp, camera_view, locations);

    auto ball_tracker = ball_trackers.begin();
    while (ball_tracker != ball_trackers.end())
    {
        // update tracker with new locations
        if (ball_tracker->update(stamp, locations, camera_view))
        {
            ball_tracker = ball_trackers.erase(ball_tracker);
        }
        else
        {
            ball_tracker++;
        }
    }

    for (int i = 0; i < locations.size(); ++i)
    {
        ball_trackers.push_back(BallTracker(stamp, locations[i]));
    }

    referenceFrame = msg.header.frame_id;

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
    for (auto ball_tracker = ball_trackers.begin(); ball_tracker != ball_trackers.end(); ++ball_tracker)
    {
        if (ball_tracker->location_valid())
        {

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

double getDistance(tf2::Vector3 a, tf2::Vector3 b) {
    tf2::Vector3 displacement = b - a;
    return displacement.length();
}

bool closestBall(ball_mapper::ClosestBall::Request &request, ball_mapper::ClosestBall::Response &response)
{
    double minimumDistance = request.distance_limit;
    double oppositeGrabberDistanceThreshold = 0.03;

    tf2::Vector3 grabberLeftLocation = getLocation(referenceFrame, "left_grabber");
    tf2::Vector3 grabberRightLocation = getLocation(referenceFrame, "right_grabber");
    tf2::Vector3 targetLocation = getLocation(referenceFrame, request.grabber_frame);

    tf2::Vector3 closestBallLocation;

    for (auto ball_tracker = ball_trackers.begin(); ball_tracker != ball_trackers.end(); ++ball_tracker)
    {
        if (ball_tracker->location_valid())
        {
            tf2::Vector3 ballLocation = ball_tracker->get_location();
            double ballDistance = getDistance(ballLocation, targetLocation);

            if (request.grabber_frame != "left_grabber") {
                double grabberLeftDistance = getDistance(ballLocation, grabberLeftLocation);
                if (grabberLeftDistance < oppositeGrabberDistanceThreshold) {
                    continue;
                }
            }
            
            if (request.grabber_frame != "right_grabber") {
                double grabberRightDistance = getDistance(ballLocation, grabberRightLocation);
                if (grabberRightDistance < oppositeGrabberDistanceThreshold) {
                    continue;
                }
            }

            if (ballDistance < minimumDistance)
            {
                closestBallLocation = ballLocation;
                minimumDistance = ballDistance;
            }
        }
    }

    if (minimumDistance == request.distance_limit)
    {
        return false;
    }
    else
    {
        response.ball_frame = referenceFrame;
        geometry_msgs::Point ballLocationMessage;
        tf2::toMsg(closestBallLocation, ballLocationMessage);
        response.position = ballLocationMessage;
        response.distance = minimumDistance;
        return true;
    }
}

tf2::Vector3 getLocation(std::string startFrame, std::string endFrame)
{
    geometry_msgs::TransformStamped requestTransformStamped;
    requestTransformStamped = tfBuffer.lookupTransform(startFrame, endFrame, ros::Time::now(), ros::Duration(1.0));

    tf2::Transform requestTransform;
    tf2::fromMsg(requestTransformStamped.transform, requestTransform);

    return requestTransform.getOrigin();
}

int main(int argc, char **argv)
{
    // node setup
    ros::init(argc, argv, "ball_mapper");
    ros::NodeHandle n;

    // ball location subscriber
    ros::Subscriber ball_view_sub = n.subscribe("camera/balls", 20, ball_view_callback);

    ros::ServiceServer closestBallService = n.advertiseService("ball_map/closest", closestBall);

    // point array publisher
    ball_map_pub = n.advertise<ball_msgs::BallMap>("ball_map", 5);
    ball_map_vis_pub = n.advertise<visualization_msgs::Marker>("ball_map_vis", 5);

    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::spin();
}
