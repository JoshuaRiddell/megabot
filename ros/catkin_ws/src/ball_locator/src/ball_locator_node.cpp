#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>

ros::Publisher ball_pub;

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

    cv::imshow("view", img);
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

    cv::namedWindow("view");
    // cv::startWindowThread();

    ball_pub = n.advertise<geometry_msgs::Point>("ball", 10);

    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, image_callback);

    ros::spin();

    cv::destroyWindow("view");

    return 0;
};
