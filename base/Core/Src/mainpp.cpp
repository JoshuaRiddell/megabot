#include <mainpp.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_conf.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

extern "C" {
#include "omni_bot/omni_bot.h"
}

ros::NodeHandle nh;
std_msgs::Float64 motorMsg;
geometry_msgs::Twist commandEchoMsg;
geometry_msgs::TransformStamped odomTransform;
tf::TransformBroadcaster broadcaster;

void cmdVelCallback(const geometry_msgs::Twist& msg);
void resetOdomCallback(const std_msgs::Empty& msg);
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCallback);
ros::Subscriber<std_msgs::Empty> resetOdomSub("reset_odom", &resetOdomCallback);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup() {
	pin_setup();
	timer_setup();
	motion_setup();
	kinematics_setup();

	enable_steppers();

	nh.initNode();
	nh.subscribe(cmdVelSub);
	nh.subscribe(resetOdomSub);
	broadcaster.init(nh);

	resetOdomCallback(std_msgs::Empty());
}

#define LOOP_DIVISOR 10

void loop() {
	uint32_t loopCount = 0;

	while (true) {
		nh.spinOnce();
		HAL_Delay(10);

		++loopCount;

		if (loopCount % LOOP_DIVISOR == 0) {
			position_t position = get_position();
			odomTransform.header.frame_id = "odom";
			odomTransform.child_frame_id = "base_footprint";
			odomTransform.transform.translation.x = position.x;
			odomTransform.transform.translation.y = position.y;
			odomTransform.transform.translation.z = 0.;
			odomTransform.transform.rotation = tf::createQuaternionFromYaw(-position.theta + M_PI/2);
			odomTransform.header.stamp = nh.now();
			broadcaster.sendTransform(odomTransform);
		}
	}
}

void cmdVelCallback(const geometry_msgs::Twist& msg) {
	move_robot(-msg.linear.y*314, msg.linear.x*314, msg.angular.z*72);
}

void resetOdomCallback(const std_msgs::Empty& msg) {
	position_t initialPosition;

	initialPosition.x = 1.2;
	initialPosition.y = 0.2;
	initialPosition.theta = 0;

	set_position(initialPosition);
}
