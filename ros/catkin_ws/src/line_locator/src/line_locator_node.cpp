#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/mat.hpp>

#include <dynamic_reconfigure/server.h>
#include <line_locator/ImageAnalysisConfig.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

// publisher for debug images
image_transport::CameraPublisher debug_pub;

// raw line point publisher
ros::Publisher line_pub;

// transform buffer
tf2_ros::Buffer tf_buffer;

// image processing parameters
cv::Scalar hsv_low(0, 0, 0), hsv_high(255, 255, 255);
double area_low = 0;
double area_high = 1000000;
double circularity_low = 0.;
double circularity_high = 1.;

// camera parameter cache
double fx, fy, cx, cy;

/*
 * Dynamic reconfigure callback. Sets image processing parameters.
 */
void config_callback(line_locator::ImageAnalysisConfig &config, uint32_t level) {
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

void draw_point(cv::Mat &img, cv::Point coordinate, cv::Scalar color) {
    if (coordinate.y > img.rows / 10 &&
            coordinate.y < img.rows - img.rows / 10 &&
            coordinate.x > img.cols / 10 &&
            coordinate.x < img.cols - img.cols / 10) {
        cv::circle(img, coordinate, 10, color, -1);
    }
}

cv::Point get_mean_point(cv::Mat &img, std::vector<cv::Point> coordinates) {
    double xSum, ySum;
    int n = 0;
    xSum = ySum = 0;

    for (auto coordinate = coordinates.begin(); coordinate != coordinates.end(); ++coordinate) {
        if (coordinate->y > img.rows / 10 &&
                coordinate->y < img.rows - img.rows / 10 &&
                coordinate->x > img.cols / 10 &&
                coordinate->x < img.cols - img.cols / 10) {
            xSum += coordinate->x;
            ySum += coordinate->y;
            ++n;
        }
    }

    if (n == 0) {
        return cv::Point(-1, -1);
    } else {
        return cv::Point(xSum / n, ySum / n);
    }
}

/*
 * Get centrepoint of balls in the passed image. Also render a debug image
 * if it is not empty.
 */
void get_line_points(std::vector<std::vector<cv::Point>> &line_points, cv::Mat &img, cv::Mat &debug) {
    cv::Mat hsv, bin;

    // get binary image using hsv thresholding
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, hsv_low, hsv_high, bin);

    // get contours from binary image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    // filter contours based on area
    int n = 0;
    for( int i = 0; i< contours.size(); i++ )
    {
        std::vector<cv::Point> contour = contours[i];

        // filter area
        double area = cv::contourArea(contour);
        if (area < area_low)
            continue;
        if (area > area_high)
            continue;

        // filter circularity
        double perimeter = cv::arcLength(contour, true);
        double circularity = 4*area*M_PI/perimeter/perimeter;
        if (circularity < circularity_low)
            continue;
        if (circularity > circularity_high)
            continue;

        // fit line to points
        cv::Vec4f line;
        cv::fitLine(contour, line, cv::DIST_L2, 1.0, 0.01, 0.01);

        cv::Point north, south, east, west;
        north = south = east = west = contour[0];
        for (auto point = contour.begin(); point != contour.end(); ++point) {
            double x, y;
            x = point->x;
            y = point->y;

            if (y < north.y) {
                north = *point;
            } else if (y > south.y) {
                south = *point;
            }

            if (x < west.x) {
                west = *point;
            } else if (x > east.x) {
                east = *point;
            }
        }

        std::vector<cv::Point> endpointCoordinates;
        endpointCoordinates.push_back(north);
        endpointCoordinates.push_back(south);
        endpointCoordinates.push_back(east);
        endpointCoordinates.push_back(west);
        cv::Point endpoint = get_mean_point(img, endpointCoordinates);

        ROS_INFO("x:%d y:%d", endpoint.x, endpoint.y);

        // calculate points on the image extremeties for the line
        float vx, vy, x, y;

        vx = line[0];
        vy = line[1];
        x = line[2];
        y = line[3];

        cv::Point start, end;
        int cols = img.cols;
        int rows = img.rows;

        if (abs(vx) > abs(vy)) {
            // use horizontal method
            float m = vy/vx;
            start = cv::Point(0, (int)round(y - x*m));
            end = cv::Point(cols-1, (int)round(y + (cols-x)*m));
        } else {
            // use vertical method
            float m = vx/vy;
            start = cv::Point((int)round(x - y*m), 0);
            end = cv::Point((int)round(x + (rows-y)*m), rows-1);
        }

        // add start and end to line list
        std::vector<cv::Point> points;
        points.push_back(start);
        points.push_back(end);
        line_points.push_back(points);

        // draw debug image if required
        if (!debug.empty()) {
            cv::Scalar white = cv::Scalar( 255, 255, 255 );
            cv::Scalar blue = cv::Scalar( 255, 0, 0 );
            cv::drawContours(debug, contours, i, white, 2, 8);
            cv::line(debug, start, end, white, 2, 8);
            cv::circle(debug, endpoint, 10, blue, -1);
        }
    }
}

/*
 * Convert a centrepoint of a ball on an image to a location in 3D space using
 * transformation pass via rotation and origin.
 */
void image_point_to_world(tf2::Quaternion rotation, tf2::Vector3 origin,
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
    double factor = -(origin.getZ()) / v.getZ();
    double dx = factor * v.getX();
    double dy = factor * v.getY();

    // export point at the intersection point (where ball is located)
    point.x = origin.getX() + dx;
    point.y = origin.getY() + dy;
    point.z = 0;
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

    // get lines from image
    std::vector<std::vector<cv::Point>> lines;
    get_line_points(lines, img, debug);

    // get origin and rotation of camera to transform ball locations
    tf2::Quaternion rotation;
    tf2::fromMsg(transformStamped.transform.rotation, rotation);
    tf2::Vector3 origin(
        transformStamped.transform.translation.x,
        transformStamped.transform.translation.y,
        transformStamped.transform.translation.z
    );

    // convert all the centre points on the image to map frame, and publish
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = msg->header.stamp;

    auto line = lines.begin();
    for (int i = 0; line != lines.end(); ++line) {
        geometry_msgs::Point start, end;

        image_point_to_world(rotation, origin, line->at(0), start);
        image_point_to_world(rotation, origin, line->at(1), end);

        p.pose.position = start;
        
        double angle = atan2(end.y-start.y, end.x-start.x);
        tf2::Quaternion q;
        q.setRPY(0, 0, angle);
        p.pose.orientation = tf2::toMsg(q);
        
        line_pub.publish(p);
    }

    // publish debug image if needed
    if (!debug.empty()) {
        publish_debug_image(debug);
    }
}

int main(int argc, char **argv)
{
    // node setup
    ros::init(argc, argv, "line_locator");
    ros::NodeHandle n;

    // dynamic reconfigure setup
    dynamic_reconfigure::Server<line_locator::ImageAnalysisConfig> server;
    server.setCallback(config_callback);
    
    // publisher for ball positions
    line_pub = n.advertise<geometry_msgs::PoseStamped>("line", 10);

    // subscriber for camera image
    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub = it.subscribe("camera/image_rect_color", 1, image_callback);
    ros::Subscriber camera_info_sub = n.subscribe("camera/camera_info", 1, camera_info_callback);

    // publisher for rendered image output stream
    debug_pub = it.advertiseCamera("line/debug", 1);

    // tf listener for camera position
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // main loop
    ros::spin();

    return 0;
};
