#include <mainpp.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_conf.h>
#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;

void ledCallback(const std_msgs::Empty& msg);
ros::Subscriber<std_msgs::Empty> ledSub("led", &ledCallback);

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
	HAL_Delay(50);
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
}

void ledCallback(const std_msgs::Empty& msg) {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
}
