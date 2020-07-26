#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

std::vector<tf2::Quaternion> mapLineAngles;
std::vector<tf2::Vector3> mapLineEnds;
tf2::Transform mapToOdom;
double movingAverageConstant = 0.2;

void lineEndpointCallback(const geometry_msgs::PoseStamped &msg) {
    tf2::Quaternion measuredRotation;
    tf2::Vector3 measuredEndpoint;

    tf2::convert(msg.pose.orientation, measuredRotation);
    tf2::convert(msg.pose.position, measuredEndpoint);

    double matchedThreshold = 0.2;

    int matchedIndex = -1;
    for (int i = 0; i < mapLineEnds.size(); ++i) {
        double distance = tf2::tf2Distance(measuredEndpoint, mapLineEnds.at(i));

        if (distance < matchedThreshold) {
            matchedIndex = i;
            break;
        }
    }

    if (matchedIndex < 0) {
        ROS_INFO("distance filtered");
        return;
    }

    double matchedAngleThreshold = 0.15;
    tf2::Quaternion matchedRotation = mapLineAngles.at(matchedIndex);

    if (matchedRotation.angleShortestPath(measuredRotation) > M_PI_2) {
        tf2::Quaternion flip;
        flip.setRPY(0,0,-M_PI);
        matchedRotation *= flip;
    }

    tf2::Quaternion angleDifference = matchedRotation * measuredRotation.inverse();

    ROS_INFO("%f", angleDifference.getAngle());

    if (angleDifference.getAngle() > matchedAngleThreshold) {
        return;
    }


    tf2::Vector3 correctionOrigin = mapLineEnds.at(matchedIndex) - measuredEndpoint;
    tf2::Quaternion correctionRotation = angleDifference;
    
    tf2::Vector3 currentOrigin = mapToOdom.getOrigin();
    tf2::Quaternion currentRotation = mapToOdom.getRotation();

    double c = movingAverageConstant;
    double invC = 1-c;

    currentOrigin.setX(correctionOrigin.x() * c + invC * currentOrigin.x());
    currentOrigin.setY(correctionOrigin.y() * c + invC * currentOrigin.y());
    currentOrigin.setZ(correctionOrigin.z() * c + invC * currentOrigin.z());
    
    double currentYaw = currentRotation.getAngle();
    double correctionYaw = correctionRotation.getAngle();
    currentYaw = correctionYaw * c + invC * currentYaw;
    currentRotation.setRPY(0, 0, currentYaw);

    mapToOdom.setOrigin(currentOrigin);
    mapToOdom.setRotation(currentRotation);
}

void addLine(double x, double y, double angle) {
    tf2::Quaternion qTmp;
    mapLineEnds.push_back(tf2::Vector3(x, y, 0));

    qTmp.setRPY(0, 0, angle);
    mapLineAngles.push_back(qTmp);
}

void loadMapLines() {
    // single vertical
    addLine(1.2, 0, M_PI/2);
    addLine(1.2, 1.2, M_PI/2);

    // left diagonal
    addLine(0.71, 1.2, 2.094);
    addLine(0.16, 2.143, 2.094);

    // right diagonal
    addLine(2.24, 2.143, 1.0472);
    addLine(1.69, 1.2, 1.0472);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localisation");

    ros::NodeHandle nh;
    ros::Rate r(10);

    tf2_ros::TransformBroadcaster transformBroadcaster;

    ros::Subscriber line_sub = nh.subscribe("line_endpoint", 1, lineEndpointCallback);

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "odom";

    loadMapLines();
    
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    mapToOdom.setOrigin(tf2::Vector3(0, 0, 0));
    mapToOdom.setRotation(q);

    while (ros::ok())
    {
        ros::spinOnce();

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.transform = tf2::toMsg(mapToOdom);

        transformBroadcaster.sendTransform(transformStamped);
        r.sleep();
    }

    return 0;
};
