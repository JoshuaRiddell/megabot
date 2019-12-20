#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>

#include <opencv2/core/mat.hpp>

#include <dynamic_reconfigure/server.h>
#include <ball_locator/ImageAnalysisConfig.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <urdf/model.h>

// publisher for debug images
image_transport::CameraPublisher debug_pub;

// transform buffer
tf2_ros::Buffer tf_buffer;

// publisher for ball position messages
ros::Publisher ball_pub;

// image processing parameters
cv::Scalar hsv_low(0, 0, 0), hsv_high(255, 255, 255);
double area_low = 0;
double area_high = 1000000;
double circularity_low = 0.;
double circularity_high = 1.;

// camera parameter cache
double fx, fy, cx, cy;

// tennis ball model
double ball_radius;

/*
 * Dynamic reconfigure callback. Sets image processing parameters.
 */
void config_callback(ball_locator::ImageAnalysisConfig &config, uint32_t level) {
    hsv_low.val[0] = config.h_low;
    hsv_low.val[1] = config.s_low;
    hsv_low.val[2] = config.v_low;
    
    hsv_high.val[0] = config.h_high;
    hsv_high.val[1] = config.s_high;
    hsv_high.val[2] = config.v_high;

    area_low = config.area_low;
    area_high = config.area_high;

    circularity_low = config.circularity_low;
    circularity_high = config.circularity_high;
}

/*
 * Saves camera parameters used when calculating where balls are located in space
 */
void camera_info_callback(const sensor_msgs::CameraInfo& msg) {
    fx = msg.K[0];
    cx = msg.K[2];
    fy = msg.K[4];
    cy = msg.K[5];
}

/*
 * Get centrepoint of balls in the passed image. Also render a debug image
 * if it is not empty.
 */
void get_ball_centres(std::vector<cv::Point> &centres, cv::Mat &img, cv::Mat &debug) {
    cv::Mat hsv, bin;

    // get binary image using hsv thresholding
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, hsv_low, hsv_high, bin);

    // get contours from binary image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    // filter contours based on area
    for( int i = 0; i< contours.size(); i++ )
    {
        // filter area
        double area = cv::contourArea(contours[i]);
        if (area < area_low)
            continue;
        if (area > area_high)
            continue;

        // filter circularity
        double perimeter = cv::arcLength(contours[i], true);
        double circularity = 4*area*M_PI/perimeter/perimeter;
        if (circularity < circularity_low)
            continue;
        if (circularity > circularity_high)
            continue;

        // get circle centres
        cv::RotatedRect ellipse;
        ellipse = cv::fitEllipse(contours[i]);
        cv::Point centre = ellipse.center;

        // record all centres
        centres.push_back(centre);

        if (!debug.empty()) {
            cv::Scalar color = cv::Scalar( 255, 255, 255 );
            cv::drawContours(debug, contours, i, color, 2, 8);
            cv::circle(debug, centre, 3, color, 3);
        }
    }
}

/*
 * Convert a centrepoint of a ball on an image to a location in 3D space using
 * transformation pass via rotation and origin.
 */
void centre_to_point(tf2::Quaternion rotation, tf2::Vector3 origin,
                     cv::Point &centre, geometry_msgs::Point &point) {
    // unit vector from camera origin to tennis ball point
    tf2::Vector3 v(
        1.0,
        -(centre.x - cx)/fx,
        -(centre.y - cy)/fy
    );

    // rotate vector into map coordinate frame
    v = tf2::quatRotate(rotation, v);

    // intersect vector in map coordinate frame with ground plane at tennis ball height
    double factor = -(origin.getZ() - ball_radius) / v.getZ();
    double dx = factor * v.getX();
    double dy = factor * v.getY();

    // export point at the intersection point (where ball is located)
    point.x = origin.getX() + dx;
    point.y = origin.getY() + dy;
    point.z = ball_radius;
}

/*
 * Publish a debug image via ros.
 */
void publish_debug_image(cv::Mat &debug) {
    std_msgs::Header img_msg_header;
    img_msg_header.stamp = ros::Time::now();
    img_msg_header.frame_id = "camera_link";

    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header = img_msg_header;

    cam_info_msg.height = debug.rows;
    cam_info_msg.width = debug.cols;
    // distortion
    cam_info_msg.distortion_model = "plumb_bob";
    // binning
    cam_info_msg.binning_x = 0;
    cam_info_msg.binning_y = 0;

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(img_msg_header, "bgr8", debug).toImageMsg();
    debug_pub.publish(*img_msg, cam_info_msg);
}

/*
 * Callback fired when new images are ready from the camera. Outputs a
 * debug image stream and a stream of ball positions in space found in
 * the image.
 */
void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    // try and get transform for the camera
    geometry_msgs::TransformStamped transformStamped;
    try {
        // get the transform from camera to map
        transformStamped = tf_buffer.lookupTransform("map",
            msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
    }
    catch (tf2::TransformException ex){
        // transform not available - not much point in continuing analysis
        ROS_ERROR("%s", ex.what());
        return;
    }
    
    // extract image from msg
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

    // check if we should output a debug video
    cv::Mat debug;
    if (debug_pub.getNumSubscribers() > 0) {
        debug = img;
    }

    // get contours of possible balls from image
    std::vector<cv::Point> centres;
    get_ball_centres(centres, img, debug);

    // exit if no centres were found
    if (centres.size() == 0) {
        return;
    }

    // get origin and rotation of camera to transform ball locations
    tf2::Quaternion rotation;
    tf2::fromMsg(transformStamped.transform.rotation, rotation);
    tf2::Vector3 origin(
        transformStamped.transform.translation.x,
        transformStamped.transform.translation.y,
        transformStamped.transform.translation.z
    );

    // convert all the centre points on the image to map frame, and publish
    geometry_msgs::PointStamped p;
    p.header.frame_id = "map";
    p.header.stamp = msg->header.stamp;
    for (int i = 0; i < centres.size(); ++i) {
        centre_to_point(rotation, origin, centres[i], p.point);
        ball_pub.publish(p);
    }

    // publish debug image if needed
    if (!debug.empty()) {
        publish_debug_image(debug);
    }
}

int main(int argc, char **argv)
{
    // node setup
    ros::init(argc, argv, "ball_locator");
    ros::NodeHandle n;

    // dynamic reconfigure setup
    dynamic_reconfigure::Server<ball_locator::ImageAnalysisConfig> server;
    server.setCallback(config_callback);
    
    // publisher for ball positions
    ball_pub = n.advertise<geometry_msgs::PointStamped>("ball", 10);

    // subscriber for camera image
    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub = it.subscribe("camera/image_raw", 1, image_callback);
    ros::Subscriber camera_info_sub = n.subscribe("camera/camera_info", 1, camera_info_callback);

    // publisher for rendered image output stream
    debug_pub = it.advertiseCamera("ball/debug", 1);

    // tf listener for camera position
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // get tennis ball diameter from URDF model
    urdf::Model tennis_ball_model;
    tennis_ball_model.initParam("/ball_description");
    urdf::GeometryConstSharedPtr g = tennis_ball_model.getLink("ball")->collision->geometry;
    urdf::Sphere const &ball_geometry = dynamic_cast<const urdf::Sphere &>(*g);
    ball_radius = ball_geometry.radius;

    // main loop
    ros::spin();

    return 0;
};
