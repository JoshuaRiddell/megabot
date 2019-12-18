#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <dynamic_reconfigure/server.h>
#include <ball_locator/ImageAnalysisConfig.h>

ros::Publisher ball_pub;
cv::Scalar hsv_low(0, 0, 0), hsv_high(255, 255, 255);
double area_low = 0;
double area_high = 1000000;
double circularity_low = 0.;
double circularity_high = 1.;

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

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
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

    geometry_msgs::Point p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    ball_pub.publish(p);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_locator");
    ros::NodeHandle n;

    dynamic_reconfigure::Server<ball_locator::ImageAnalysisConfig> server;
    server.setCallback(config_callback);

    ball_pub = n.advertise<geometry_msgs::Point>("ball", 10);

    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, image_callback);

    ros::spin();

    cv::destroyWindow("view");

    return 0;
};
