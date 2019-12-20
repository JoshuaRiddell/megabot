#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <dynamic_reconfigure/server.h>
#include <ball_locator/ImageAnalysisConfig.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

tf2_ros::Buffer tf_buffer;
ros::Publisher ball_pub;
cv::Scalar hsv_low(0, 0, 0), hsv_high(255, 255, 255);
double area_low = 0;
double area_high = 1000000;
double circularity_low = 0.;
double circularity_high = 1.;
double fx, fy, cx, cy;

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

void camera_info_callback(const sensor_msgs::CameraInfo& msg) {
    fx = msg.K[0];
    cx = msg.K[2];
    fy = msg.K[4];
    cy = msg.K[5];
}

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    // try and get transform for the camera
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tf_buffer.lookupTransform("map", msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
    }
    catch (tf2::TransformException ex){
        ROS_ERROR("%s", ex.what());
        return;
    }

    // extract image from rosmsg
    cv::Mat hsv, bin;
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

    // get binary image using hsv thresholding
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, hsv_low, hsv_high, bin);

    // get contours from binary image
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(bin, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> filtered_centres;
    cv::Mat drawing = img;
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
        filtered_centres.push_back(centre);

        cv::Scalar color = cv::Scalar( 255, 255, 255 );
        cv::drawContours( drawing, contours, i, color, 2, 8);
        cv::circle(drawing, centre, 3, color, 3);
    }

    cv::Mat view;
    cv::resize(drawing, view, cv::Size(960, 540));
    cv::imshow("view", view);
    cv::waitKey(30);

    if (filtered_centres.size() == 0) {
        return;
    }

    tf2::Quaternion rotation;
    tf2::fromMsg(transformStamped.transform.rotation, rotation);
    tf2::Vector3 o(
        transformStamped.transform.translation.x,
        transformStamped.transform.translation.y,
        transformStamped.transform.translation.z
    );

    for (int i = 0; i < filtered_centres.size(); ++i) {
        // unit vector from camera origin to tennis ball point
        tf2::Vector3 v(
            1.0,
            -(filtered_centres[i].x - cx)/fx,
            -(filtered_centres[i].y - cy)/fy
        );

        // rotate vector into map coordinate frame
        v = tf2::quatRotate(rotation, v);

        // intersect vector in map coordinate frame with ground plane
        double factor = -(o.getZ() - 0.033) / v.getZ();
        double dx = factor * v.getX();
        double dy = factor * v.getY();

        geometry_msgs::PointStamped p;
        p.header.frame_id = "map";
        p.header.stamp = msg->header.stamp;
        p.point.x = o.getX() + dx;
        p.point.y = o.getY() + dy;
        p.point.z = 0.033;
        ball_pub.publish(p);
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

    // tf setup
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // main loop
    ros::spin();

    cv::destroyWindow("view");

    return 0;
};
