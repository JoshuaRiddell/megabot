#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <localisation/SetPosition.h>
#include <dynamic_reconfigure/server.h>
#include <localisation/LocalisationConfig.h>
#include <localisation.h>

#include <tf2_ros/transform_broadcaster.h>

localisation::LocalisationConfig currentConfig;
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformBroadcaster *transformBroadcasterPtr;
std::vector<tf2::Quaternion> mapLineAngles;
std::vector<tf2::Vector3> mapLineEnds;
tf2::Transform mapToOdom;
geometry_msgs::Twist currentVelocity;

double currentAngleDifference;
double currentDistance;

tf2::Transform getTransform(std::string startFrame, std::string endFrame, ros::Time time);

void pubTransform(std::string from, std::string to, ros::Time stamp, tf2::Transform transform) {
    geometry_msgs::TransformStamped tfMsg;
    tfMsg.header.stamp = stamp;
    tfMsg.header.frame_id = from;
    tfMsg.child_frame_id = to;
    tfMsg.transform = tf2::toMsg(transform);
    transformBroadcasterPtr->sendTransform(tfMsg);
}

void currentVelocityCallback(const geometry_msgs::Twist &msg) {
    currentVelocity = msg;
}

void lineEndpointCallback(const geometry_msgs::PoseStamped &msg) {
    tf2::Vector3 translationVelocity;
    translationVelocity.setX(currentVelocity.linear.x);
    translationVelocity.setY(currentVelocity.linear.y);
    if (translationVelocity.length() > currentConfig.max_translation_velocity) {
        ROS_INFO("translation speed too high");
        return;
    }
    if (fabs(currentVelocity.angular.z) > currentConfig.max_rotation_velocity) {
        ROS_INFO("angular speed too high");
        return;
    }

    tf2::Quaternion measuredRotation;
    tf2::Vector3 measuredEndpoint;

    tf2::convert(msg.pose.orientation, measuredRotation);
    tf2::convert(msg.pose.position, measuredEndpoint);

    double matchedThreshold = currentConfig.matched_distance_threshold;

    int matchedIndex = -1;
    for (int i = 0; i < mapLineEnds.size(); ++i) {
        double distance = tf2::tf2Distance(measuredEndpoint, mapLineEnds.at(i));

        if (distance < matchedThreshold) {
            matchedIndex = i;
            currentDistance = distance;
            break;
        }
    }

    ROS_INFO("distance diff: %f", currentDistance);

    if (matchedIndex < 0) {
        ROS_INFO("distance filtered");
        return;
    }

    double matchedAngleThreshold = tf2Radians(currentConfig.matched_angle_threshold_deg);
    tf2::Vector3 matchedOrigin = mapLineEnds.at(matchedIndex);
    tf2::Quaternion matchedRotation = mapLineAngles.at(matchedIndex);

    if (matchedRotation.angleShortestPath(measuredRotation) > M_PI_2) {
        tf2::Quaternion flip;
        flip.setRPY(0,0,-M_PI);
        matchedRotation *= flip;
    }

    tf2::Quaternion angleDifference = matchedRotation * measuredRotation.inverse();

    currentAngleDifference = angleDifference.getAngle();
    ROS_INFO("angle diff: %f", currentAngleDifference);

    if (angleDifference.getAngle() > matchedAngleThreshold) {
        ROS_INFO("angle filtered");
        return;
    }

    double c = currentConfig.moving_average_constant;
    double invC = 1 - c;

    tf2::Quaternion endpointTargetRotation = measuredRotation.slerp(matchedRotation, c);
    tf2::Vector3 endpointTargetOrigin;
    endpointTargetOrigin.setX(measuredEndpoint.x()*c + matchedOrigin.x()*invC);
    endpointTargetOrigin.setY(measuredEndpoint.y()*c + matchedOrigin.y()*invC);
    endpointTargetOrigin.setZ(measuredEndpoint.z()*c + matchedOrigin.z()*invC);
    tf2::Transform endpointTargetTransform;
    endpointTargetTransform.setOrigin(endpointTargetOrigin);
    endpointTargetTransform.setRotation(endpointTargetRotation);

    tf2::Transform measuredEndpointTransform;
    measuredEndpointTransform.setOrigin(measuredEndpoint);
    measuredEndpointTransform.setRotation(measuredRotation);

    tf2::Transform odomFootprintTransform = getTransform("odom", "base_footprint", msg.header.stamp);
    tf2::Transform mapFootprintTransform = getTransform("map", "base_footprint", msg.header.stamp);
    tf2::Transform footprintEndpointTransform = mapFootprintTransform.inverseTimes(measuredEndpointTransform);
    tf2::Transform newFootprint = endpointTargetTransform * footprintEndpointTransform.inverse();
    mapToOdom = newFootprint * odomFootprintTransform.inverse();

    tf2::Transform matchedTransform;
    matchedTransform.setRotation(matchedRotation);
    matchedTransform.setOrigin(matchedOrigin);
    pubTransform("map", "matched", msg.header.stamp, matchedTransform);

    // pubTransform("map", "measured", msg.header.stamp, measuredEndpointTransform);
    // pubTransform("map", "endpoint_target", msg.header.stamp, endpointTargetTransform);
    // pubTransform("map", "new_footprint", msg.header.stamp, newFootprint);
    // pubTransform("endpoint_target", "new_footprint2", msg.header.stamp, footprintEndpointTransform.inverse());
    // pubTransform("map", "footprint", msg.header.stamp, mapFootprintTransform);
    // pubTransform("base_footprint", "test_endpoint", msg.header.stamp, footprintEndpointTransform);
    // pubTransform("test_odom", "test_footprint", msg.header.stamp, odomFootprintTransform);
    // pubTransform("map", "test_odom", msg.header.stamp, mapToOdom);
}

void clearLines() {
    mapLineEnds.clear();
    mapLineAngles.clear();
}

void addLine(double x, double y, double angleDeg) {
    tf2::Quaternion qTmp;
    mapLineEnds.push_back(tf2::Vector3(x, y, 0));

    qTmp.setRPY(0, 0, tf2Radians(angleDeg));
    mapLineAngles.push_back(qTmp);
}

bool setLocationCallback(localisation::SetPosition::Request &request,
                        localisation::SetPosition::Response &response) {
    tf2::Vector3 requestedPosition;
    tf2::Quaternion requestedRotation;

    tf2::fromMsg(request.position, requestedPosition);
    tf2::fromMsg(request.rotation, requestedRotation);

    tf2::Transform requestedTransform;
    requestedTransform.setOrigin(requestedPosition);
    requestedTransform.setRotation(requestedRotation);

    tf2::Transform currentTransform = getTransform("odom", "base_footprint", ros::Time::now());

    mapToOdom = requestedTransform * currentTransform.inverse();

    return true;
}

tf2::Transform getTransform(std::string startFrame, std::string endFrame, ros::Time time)
{
    geometry_msgs::TransformStamped requestTransformStamped;
    requestTransformStamped = tfBuffer.lookupTransform(startFrame, endFrame, time, ros::Duration(1.0));

    tf2::Transform requestTransform;
    tf2::fromMsg(requestTransformStamped.transform, requestTransform);

    return requestTransform;
}

void reconfigureCallback(localisation::LocalisationConfig &config, uint32_t level) {
    currentConfig = config;

    clearLines();
    addLine(config.line_x, config.line_y, config.line_angle_deg);
}

LineCalibrationAction::LineCalibrationAction(std::string actionName)
    : actionServer(nh, actionName, boost::bind(&LineCalibrationAction::executeCallback, this, _1), false)
{
    actionServer.start();
}

void LineCalibrationAction::executeCallback(const localisation::LineCalibrationGoalConstPtr &goal)
{
    int pollFrequency = 10;
    int timeoutMaxSec = 10;
    int timeoutMaxN = timeoutMaxSec * pollFrequency;
    ros::Rate delay(pollFrequency);
    int timeoutCounter = 0;

    currentDistance = INFINITY;
    currentAngleDifference = INFINITY;
    ros::Subscriber line_sub = nh.subscribe("line_endpoint", 1, lineEndpointCallback);

    while (true) {
        if (!ros::ok() || actionServer.isPreemptRequested()) {
            actionServer.setPreempted();
            break;
        }

        if (currentDistance < 0.005 && currentAngleDifference < tf2Radians(0.01)) {
            actionServer.setSucceeded();
            break;
        }

        if (timeoutCounter > timeoutMaxN) {
            actionServer.setAborted();
            break;
        }

        delay.sleep();
        timeoutCounter++;
    }

    line_sub.shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localisation");

    dynamic_reconfigure::Server<localisation::LocalisationConfig> server;
    dynamic_reconfigure::Server<localisation::LocalisationConfig>::CallbackType serverCallback;
    serverCallback = boost::bind(&reconfigureCallback, _1, _2);
    server.setCallback(serverCallback);

    tf2_ros::TransformBroadcaster transformBroadcaster;
    transformBroadcasterPtr = &transformBroadcaster;

    LineCalibrationAction("line_calibration");

    ros::NodeHandle nh;
    ros::Rate r(10);

    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::ServiceServer setPositionService = nh.advertiseService("set_location", setLocationCallback);
    ros::Subscriber currentVelocitySub = nh.subscribe("cmd_vel", 1, currentVelocityCallback);

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "odom";

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
