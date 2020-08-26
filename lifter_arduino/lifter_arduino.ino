#include <ros.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

#include <TimerOne.h>

enum {
  STEPPER_DOWN = -1,
  STEPPER_MOVING = 0,
  STEPPER_UP = 1,
};

Servo leftGripper;
Servo rightGripper;

const int leftGripperPin = 9;
const int leftGripperStart = 90;
const int rightGripperPin = 10;
const int rightGripperStart = 90;

const int stepperDirPin = A1;
const int stepperStepPin = A2;
const int stepperUpEndPin = 11;
const int stepperDownEndPin = 12;
const int stepperEnablePin = A3;
int currentStepperEndPin = stepperDownEndPin;
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
      digitalWrite(stepperDirPin, HIGH);
      currentStepperEndPin = stepperDownEndPin;
      currentStepperDirection = STEPPER_DOWN;
      break;
    case STEPPER_UP:
      digitalWrite(stepperDirPin, LOW);
      currentStepperEndPin = stepperUpEndPin;
      currentStepperDirection = STEPPER_UP;
      break;
  }
}

void displayCb(const std_msgs::String& msg) {
  lcd.setCursor(0, 0);
  lcd.print(msg.data);
}

void toggleStep() {
  if (!digitalRead(currentStepperEndPin)) {
    for (int i = 0; i < 2; ++i) {
      digitalWrite(stepperStepPin, !digitalRead(stepperStepPin));
      delayMicroseconds(100);
      digitalWrite(stepperStepPin, !digitalRead(stepperStepPin));
      delayMicroseconds(100);
    }

    if (stepperStatusMsg.data != STEPPER_MOVING) {
      stepperStatusMsg.data = STEPPER_MOVING;
      stepperStatusPub.publish(&stepperStatusMsg);
    }
  } else {
    if (stepperStatusMsg.data != currentStepperDirection) {
      stepperStatusMsg.data = currentStepperDirection;
      stepperStatusPub.publish(&stepperStatusMsg);
    }

    delay(10);
  }
}

void setup() {
  nh.initNode();
  setupGrippers();
  setupStepper();
  setupLcd();
}

void setupGrippers() {
  leftGripper.attach(leftGripperPin);
  rightGripper.attach(rightGripperPin);

  leftGripper.write(leftGripperStart);
  rightGripper.write(rightGripperStart);
  
  nh.subscribe(leftGripperSub);
  nh.subscribe(rightGripperSub);
}

void setupStepper() {
  pinMode(stepperDirPin, OUTPUT);
  pinMode(stepperStepPin, OUTPUT);
  pinMode(stepperEnablePin, OUTPUT);
  pinMode(stepperUpEndPin, INPUT);
  pinMode(stepperDownEndPin, INPUT);

  digitalWrite(stepperEnablePin, LOW);
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

void loop() {
  nh.spinOnce();
  toggleStep();
}
