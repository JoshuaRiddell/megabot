#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>

ros::Publisher cmdVelPub;
ros::Publisher lifterPub;
ros::Publisher grabberLeftPub;
ros::Publisher grabberRightPub;

enum {
    AXIS_LEFT_HORIZONTAL,
    AXIS_LEFT_VERTICAL,
    AXIS_RIGHT_HORIZONTAL,
    AXIS_RIGHT_VERTICAL,
    DPAD_LEFT_HORIZONTAL,
    DPAD_LEFT_VERTICAL,
};

enum {
    BUTTON_1,
    BUTTON_2,
    BUTTON_3,
    BUTTON_4,
    BUTTON_LEFT_UPPER,
    BUTTON_RIGHT_UPPER,
    BUTTON_LEFT_LOWER,
    BUTTON_RIGHT_LOWER,
    BUTTON_SELECT,
    BUTTON_START,
    BUTTON_LEFT_JOY,
    BUTTON_RIGHT_JOY,
};

void publishCmdVel(const sensor_msgs::Joy& msg);
void publishLifter(const sensor_msgs::Joy& msg);
void publishGrabberLeft(const sensor_msgs::Joy& msg);
void publishGrabberRight(const sensor_msgs::Joy& msg);

void joystickCallback(const sensor_msgs::Joy& msg) {
    publishCmdVel(msg);
    publishLifter(msg);
    publishGrabberLeft(msg);
    publishGrabberRight(msg);
}

void publishCmdVel(const sensor_msgs::Joy& msg) {
    geometry_msgs::Twist cmdVel;

    cmdVel.linear.x = msg.axes[AXIS_LEFT_VERTICAL] * 0.25;
    cmdVel.linear.y = msg.axes[AXIS_LEFT_HORIZONTAL] * 0.25;
    cmdVel.angular.z = msg.axes[AXIS_RIGHT_HORIZONTAL] * 0.6;

    cmdVelPub.publish(cmdVel);
}

void publishLifter(const sensor_msgs::Joy& msg) {
    std_msgs::Int8 lift;

    if (msg.axes[DPAD_LEFT_VERTICAL] == 1.0) {
        lift.data = 1;
        lifterPub.publish(lift);
    } else if (msg.axes[DPAD_LEFT_VERTICAL] == -1.0) {
        lift.data = -1;
        lifterPub.publish(lift);
    }
}

void publishGrabberLeft(const sensor_msgs::Joy& msg) {
    std_msgs::Int8 grab;

    if (msg.buttons[BUTTON_LEFT_UPPER] == 1) {
        grab.data = 1;
        grabberLeftPub.publish(grab);
    } else if (msg.buttons[BUTTON_LEFT_LOWER] == 1) {
        grab.data = -1;
        grabberLeftPub.publish(grab);
    }
}

void publishGrabberRight(const sensor_msgs::Joy& msg) {
    std_msgs::Int8 grab;

    if (msg.buttons[BUTTON_RIGHT_UPPER] == 1) {
        grab.data = 1;
        grabberRightPub.publish(grab);
    } else if (msg.buttons[BUTTON_RIGHT_LOWER] == 1) {
        grab.data = -1;
        grabberRightPub.publish(grab);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "megabot_joystick");
    ros::NodeHandle nh;

    ros::Subscriber joySub = nh.subscribe("/joy", 1, &joystickCallback);

    cmdVelPub = nh.advertise<geometry_msgs::Twist>("/megabot/cmd_vel", 1, true);
    lifterPub = nh.advertise<std_msgs::Int8>("/megabot/lifter/lift", 1, true);
    grabberLeftPub = nh.advertise<std_msgs::Int8>("/megabot/lifter/grabber_left", 1, true);
    grabberRightPub = nh.advertise<std_msgs::Int8>("/megabot/lifter/grabber_right", 1, true);

    ros::spin();

    return 0;
}

