#include <ros.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>

#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

enum {
  STEPPER_DOWN = -1,
  STEPPER_MOVING = 0,
  STEPPER_UP = 1,
};

Servo leftGripper;
Servo rightGripper;

#define LOOP_DELAY_TIME 10
#define ONE_HZ_DECIMATION (1000/LOOP_DELAY_TIME)

#define LEFT_GRIPPER_PIN 9
#define LEFT_GRIPPER_START 90
#define RIGHT_GRIPPER_PIN 10
#define RIGHT_GRIPPER_START 90

#define STEPPER_DIR_PIN A1
#define STEPPER_STEP_PIN A2
#define STEPPER_UP_END_PIN 11
#define STEPPER_DOWN_END_PIN 12
#define STEPPER_ENABLE_PIN A3
int currentStepperEndPin = STEPPER_DOWN_END_PIN;
int currentStepperDirection = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setupGrippers();
void setupStepper();
void setupLcd();

void leftGripperCb(const std_msgs::Int16& msg);
void rightGripperCb(const std_msgs::Int16& msg);
void stepperCb(const std_msgs::Int16& msg);
void displayCb(const std_msgs::String& msg);

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> leftGripperSub("lifter/servo_left", leftGripperCb);
ros::Subscriber<std_msgs::Int16> rightGripperSub("lifter/servo_right", rightGripperCb);
ros::Subscriber<std_msgs::Int16> stepperSub("lifter/stepper", stepperCb);
ros::Subscriber<std_msgs::String> displaySub("display", displayCb);

std_msgs::Int16 stepperStatusMsg;
ros::Publisher stepperStatusPub("lifter/stepper/status", &stepperStatusMsg);

void leftGripperCb(const std_msgs::Int16& msg) {
  leftGripper.write(msg.data);
}

void rightGripperCb(const std_msgs::Int16& msg) {
  rightGripper.write(msg.data);
}

void stepperCb(const std_msgs::Int16& msg) {
  setStepperDirection(msg.data);
}

void setStepperDirection(int direction) {
  switch (direction) {
    case STEPPER_DOWN:
      digitalWrite(STEPPER_DIR_PIN, HIGH);
      currentStepperEndPin = STEPPER_DOWN_END_PIN;
      currentStepperDirection = STEPPER_DOWN;
      break;
    case STEPPER_UP:
      digitalWrite(STEPPER_DIR_PIN, LOW);
      currentStepperEndPin = STEPPER_UP_END_PIN;
      currentStepperDirection = STEPPER_UP;
      break;
  }
}

void displayCb(const std_msgs::String& msg) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(msg.data);
}

void toggleStep() {
  static const int stepDelay = 60;
  
  if (!digitalRead(currentStepperEndPin)) {
    for (int i = 0; i < LOOP_DELAY_TIME * 1000 / stepDelay / 2; ++i) {
      digitalWrite(STEPPER_STEP_PIN, !digitalRead(STEPPER_STEP_PIN));
      delayMicroseconds(stepDelay);
      digitalWrite(STEPPER_STEP_PIN, !digitalRead(STEPPER_STEP_PIN));
      delayMicroseconds(stepDelay);
    }
  } else {
    delay(LOOP_DELAY_TIME);
  }
}

void setup() {
  nh.initNode();
  setupGrippers();
  setupStepper();
  setupLcd();
}

void setupGrippers() {
  leftGripper.attach(LEFT_GRIPPER_PIN);
  rightGripper.attach(RIGHT_GRIPPER_PIN);

  leftGripper.write(LEFT_GRIPPER_START);
  rightGripper.write(RIGHT_GRIPPER_START);
  
  nh.subscribe(leftGripperSub);
  nh.subscribe(rightGripperSub);
}

void setupStepper() {
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  pinMode(STEPPER_UP_END_PIN, INPUT);
  pinMode(STEPPER_DOWN_END_PIN, INPUT);

  digitalWrite(STEPPER_ENABLE_PIN, LOW);
  setStepperDirection(STEPPER_UP);

  nh.subscribe(stepperSub);
  nh.advertise(stepperStatusPub);
  stepperStatusMsg.data = STEPPER_MOVING;
}

void setupLcd() {
  lcd.init();
  lcd.backlight();
  
  nh.subscribe(displaySub);
}

void publishStepperStatus() {
  if (digitalRead(currentStepperEndPin)) {
      stepperStatusMsg.data = currentStepperDirection;
      stepperStatusPub.publish(&stepperStatusMsg);
  } else {
      stepperStatusMsg.data = STEPPER_MOVING;
      stepperStatusPub.publish(&stepperStatusMsg);
  }
}

void loop() {
  static uint32_t loopCounter = 0;
  
  nh.spinOnce();
  toggleStep();

  if (loopCounter % ONE_HZ_DECIMATION) {
    publishStepperStatus();
  }

  loopCounter++;
}
