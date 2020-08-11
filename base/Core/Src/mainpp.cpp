#include <mainpp.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_conf.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

extern "C" {
#include "omni_bot/omni_bot.h"
}

ros::NodeHandle nh;

void cmdVelCallback(const geometry_msgs::Twist& msg);
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCallback);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup() {
	stepper_setup();
	motion_setup();
	enable_steppers();

	nh.initNode();
	nh.subscribe(cmdVelSub);
}

void loop() {
	nh.spinOnce();
	HAL_Delay(50);
}

void cmdVelCallback(const geometry_msgs::Twist& msg) {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	move_robot(msg.linear.x*314, msg.linear.y*314, msg.angular.z*72);
}
