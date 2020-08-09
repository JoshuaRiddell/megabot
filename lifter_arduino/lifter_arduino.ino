#include <ros.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

#include <TimerOne.h>

enum {
  STEPPER_DOWN = -1,
  STEPPER_STOP = 0,
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
int currentEndPin = stepperDownEndPin;

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

void leftGripperCb(const std_msgs::Int16& msg) {
  leftGripper.write(msg.data);
}

void rightGripperCb(const std_msgs::Int16& msg) {
  rightGripper.write(msg.data);
}

void stepperCb(const std_msgs::Int16& msg) {
  switch (msg.data) {
    case STEPPER_DOWN:
      digitalWrite(stepperDirPin, HIGH);
      currentEndPin = stepperDownEndPin;
      break;
    case STEPPER_STOP:
      break;
    case STEPPER_UP:
      digitalWrite(stepperDirPin, LOW);
      currentEndPin = stepperUpEndPin;
      break;
  }
}

void displayCb(const std_msgs::String& msg) {
  lcd.setCursor(0, 0);
  lcd.print(msg.data);
}

void toggleStep() {
  if (!digitalRead(currentEndPin)) {
    for (int i = 0; i < 2; ++i) {
      digitalWrite(stepperStepPin, !digitalRead(stepperStepPin));
      delayMicroseconds(100);
      digitalWrite(stepperStepPin, !digitalRead(stepperStepPin));
      delayMicroseconds(100);
    }
  } else {
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

  digitalWrite(stepperDirPin, LOW);
  digitalWrite(stepperStepPin, LOW);
  digitalWrite(stepperEnablePin, LOW);
  currentEndPin = stepperUpEndPin;

  nh.subscribe(stepperSub);
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
