#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <dynamic_reconfigure/server.h>
#include <lifter_controller/LifterConfig.h>

ros::Publisher stepperPub, servoLeftPub, servoRightPub;
lifter_controller::LifterConfig lifterConfig;

void reconfigureCallback(lifter_controller::LifterConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %d %d, %d %d, %d", 
              config.servo_left_open, config.servo_left_closed,
              config.servo_right_open, config.servo_right_closed,
              config.speed);
    lifterConfig = config;
}

void lifterLiftCallback(const std_msgs::Int8 &msg) {
    if (msg.data == 1) {
        std_msgs::Int16 stepper;
        stepper.data = 1;
        stepperPub.publish(stepper);
    } else if (msg.data == -1) {
        std_msgs::Int16 stepper;
        stepper.data = -1;
        stepperPub.publish(stepper);
    }
}

void lifterGrabberLeftCallback(const std_msgs::Int8 &msg) {
    if (msg.data == 1) {
        std_msgs::Int16 servo;
        servo.data = lifterConfig.servo_left_open;
        servoLeftPub.publish(servo);
    } else if (msg.data == -1) {
        std_msgs::Int16 servo;
        servo.data = lifterConfig.servo_left_closed;
        servoLeftPub.publish(servo);
    }
}

void lifterGrabberRightCallback(const std_msgs::Int8 &msg) {
    if (msg.data == 1) {
        std_msgs::Int16 servo;
        servo.data = lifterConfig.servo_right_open;
        servoRightPub.publish(servo);
    } else if (msg.data == -1) {
        std_msgs::Int16 servo;
        servo.data = lifterConfig.servo_right_closed;
        servoRightPub.publish(servo);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lifter_controller");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<lifter_controller::LifterConfig> server;
    dynamic_reconfigure::Server<lifter_controller::LifterConfig>::CallbackType serverCallback;
    serverCallback = boost::bind(&reconfigureCallback, _1, _2);
    server.setCallback(serverCallback);

    ros::Subscriber joySub = nh.subscribe("lifter/lift", 1, &lifterLiftCallback);
    ros::Subscriber grabberLeftSub = nh.subscribe("lifter/grabber_left", 1, &lifterGrabberLeftCallback);
    ros::Subscriber grabberRightSub = nh.subscribe("lifter/grabber_right", 1, &lifterGrabberRightCallback);

    stepperPub = nh.advertise<std_msgs::Int16>("lifter/stepper", 1, true);
    servoLeftPub = nh.advertise<std_msgs::Int16>("lifter/servo_left", 1, true);
    servoRightPub = nh.advertise<std_msgs::Int16>("lifter/servo_right", 1, true);

    ros::spin();

    return 0;
}

