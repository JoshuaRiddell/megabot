#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <urdf/model.h>

#include <image_transport/image_transport.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <ball_locator/ImageAnalysisConfig.h>

#include <ball_msgs/BallView.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <coordinate_transformer/coordinate_transformer.h>
#include <ball_locator/ball_locator.h>

image_transport::CameraPublisher debugImagePub;
tf2_ros::Buffer tfBuffer;
ros::Publisher ballCentresPub, cameraViewPolyPub, ballCentresVisPub;

BallLocator ballLocator;
CoordinateTransformer coordinateTransformer;

ros::Time currentTimestamp;
std::string currentFrame;

void setupProjectionPlane();
void dynamicReconfigureCallback(ball_locator::ImageAnalysisConfig &config, uint32_t level);
void cameraInfoCallback(const sensor_msgs::CameraInfo& msg);
void cameraImageCallback(const sensor_msgs::ImageConstPtr& msg);

void publishBallLocationsInImage(cv::Mat &img);
void updateDebugImageState();
void publishDebugImage(cv::Mat &debug);
void publishCentres(const cv::Mat &img, const centres_t &centres);
tf2::Transform getCameraTransform();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_locator");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    dynamic_reconfigure::Server<ball_locator::ImageAnalysisConfig> server;
    server.setCallback(dynamicReconfigureCallback);

    cameraViewPolyPub = nh.advertise<geometry_msgs::PolygonStamped>("camera/view", 10);
    ballCentresPub = nh.advertise<ball_msgs::BallView>("camera/balls", 10);
    ballCentresVisPub = nh.advertise<geometry_msgs::PointStamped>("camera/ball_vis", 10);

    image_transport::Subscriber cameraImageSub = it.subscribe("camera/image_rect_color", 1, cameraImageCallback);
    ros::Subscriber cameraInfoSub = nh.subscribe("camera/camera_info", 1, cameraInfoCallback);
    debugImagePub = it.advertiseCamera("ball/debug", 1);

    tf2_ros::TransformListener tfListener(tfBuffer);

    setupProjectionPlane();

    ros::spin();

    return 0;
};

void setupProjectionPlane() {
    urdf::Model tennisBallModel;
    tennisBallModel.initParam("/ball_description");
    urdf::GeometryConstSharedPtr g = tennisBallModel.getLink("ball")->collision->geometry;
    urdf::Sphere const &ball_geometry = dynamic_cast<const urdf::Sphere &>(*g);
    coordinateTransformer.setGroundPlaneOffset(ball_geometry.radius);
}

void dynamicReconfigureCallback(ball_locator::ImageAnalysisConfig &config, uint32_t level) {
    ballLocator.setHsvLowThreshold(config.h_low, config.s_low, config.v_low);
    ballLocator.setHsvHighThreshold(config.h_high, config.s_high, config.v_high);
    ballLocator.setAreaBounds(config.area_low, config.area_high);
    ballLocator.setCircularityBounds(config.circularity_low, config.circularity_high);
}

void cameraInfoCallback(const sensor_msgs::CameraInfo& msg) {
    double fx = msg.K[0];
    double cx = msg.K[2];
    double fy = msg.K[4];
    double cy = msg.K[5];

    coordinateTransformer.setCameraParameters(cx, cy, fx, fy);
}

void cameraImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    currentTimestamp = msg->header.stamp;
    currentFrame = msg->header.frame_id;

    try {
        publishBallLocationsInImage(img);
    } catch (std::exception &e) {
        ROS_ERROR("%s", e.what());
    }
}

void publishBallLocationsInImage(cv::Mat &img) {
    updateDebugImageState();
    centres_t centres = ballLocator.getBallImageCentres(img);

    if (ballLocator.generateDebugImage) {
        publishDebugImage(ballLocator.debugImage);
    }
    publishCentres(img, centres);
}

void updateDebugImageState() {
    if (debugImagePub.getNumSubscribers() == 0) {
        ballLocator.generateDebugImage = false;
    } else {
        ballLocator.generateDebugImage = true;
    }
}

void publishDebugImage(cv::Mat &debug) {
    std_msgs::Header img_msg_header;
    img_msg_header.stamp = currentTimestamp;
    img_msg_header.frame_id = "camera_link";

    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header = img_msg_header;
    cam_info_msg.height = debug.rows;
    cam_info_msg.width = debug.cols;

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(img_msg_header, "bgr8", debug).toImageMsg();
    debugImagePub.publish(*img_msg, cam_info_msg);
}

void publishCentres(const cv::Mat &img, const centres_t &centres) {
    tf2::Transform cameraTransform = getCameraTransform();
    coordinateTransformer.setCameraTransform(cameraTransform);

    ball_msgs::BallView ballCentresMessage;
    ballCentresMessage.header.frame_id = "map";
    ballCentresMessage.header.stamp = currentTimestamp;

    for (int i = 0; i < centres.size(); ++i) {
        tf2::Vector3 ballCoordinate = coordinateTransformer.projectImagePointToGroundPlane(centres.at(i));
        geometry_msgs::Point point;
        tf2::toMsg(ballCoordinate, point);
        ballCentresMessage.locations.push_back(point);
    }

    std::vector<tf2::Vector3> viewBounds = coordinateTransformer.getCameraBoundsOnPlane(img);
    for (int i = 0; i < viewBounds.size(); ++i) {
        geometry_msgs::Point point;
        tf2::toMsg(viewBounds.at(i), point);
        ballCentresMessage.camera_view.push_back(point);
    }

    ballCentresPub.publish(ballCentresMessage);

    if (cameraViewPolyPub.getNumSubscribers() > 0) {
        geometry_msgs::PolygonStamped viewMsg;
        viewMsg.header.stamp = currentTimestamp;
        viewMsg.header.frame_id = "map";
        
        for (int i = 0; i < viewBounds.size(); ++i) {
            geometry_msgs::Point point;
            geometry_msgs::Point32 point32;
            tf2::toMsg(viewBounds.at(i), point);
            point32.x = point.x;
            point32.y = point.y;
            point32.z = point.z;
            viewMsg.polygon.points.push_back(point32);
        }

        cameraViewPolyPub.publish(viewMsg);
    }

    if (ballCentresVisPub.getNumSubscribers() > 0) {
        geometry_msgs::PointStamped point;
        point.header.stamp = currentTimestamp;
        point.header.frame_id = "map";

        for (int i = 0; i < ballCentresMessage.locations.size(); ++i) {
            point.point = ballCentresMessage.locations.at(i);
            ballCentresVisPub.publish(point);
        }
    }
}

tf2::Transform getCameraTransform() {
    geometry_msgs::TransformStamped cameraTransformStamped;
    cameraTransformStamped = tfBuffer.lookupTransform("map", currentFrame, currentTimestamp, ros::Duration(1.0));

    tf2::Transform cameraTransform;
    tf2::fromMsg(cameraTransformStamped.transform, cameraTransform);

    return cameraTransform;
}

