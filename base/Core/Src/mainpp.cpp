#include <mainpp.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_conf.h>
#include <ros.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle nh;

void ledCallback(const std_msgs::UInt8& msg);
ros::Subscriber<std_msgs::UInt8> ledSub("led", &ledCallback);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup() {
	nh.initNode();
	nh.subscribe(ledSub);
}

void loop() {
	nh.spinOnce();
	HAL_Delay(10);
}

void ledCallback(const std_msgs::UInt8& msg) {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
}
