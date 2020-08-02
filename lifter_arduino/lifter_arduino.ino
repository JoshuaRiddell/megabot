#include <ros.h>
#include <Servo.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>

#include <TimerOne.h>

// servo left and right uint8 message
// up down uint8 message
// emission up down uint8 message
// string display message

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

const int stepperDirPin = 7;
const int stepperStepPin = 8;
const int stepperUpEndPin = 4;
const int stepperDownEndPin = 5;
int currentEndPin = stepperDownEndPin;

void setupGrippers();
void setupStepper();

void leftGripperCb(const std_msgs::Int16& msg);
void rightGripperCb(const std_msgs::Int16& msg);
void stepperCb(const std_msgs::Int8& msg);

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16> leftGripperSub("lifter/gripper_left", leftGripperCb);
ros::Subscriber<std_msgs::Int16> rightGripperSub("lifter/gripper_right", rightGripperCb);
ros::Subscriber<std_msgs::Int8> stepperSub("lifter/stepper", stepperCb);

void leftGripperCb(const std_msgs::Int16& msg) {
  leftGripper.write(msg.data);
}

void rightGripperCb(const std_msgs::Int16& msg) {
  rightGripper.write(msg.data);
}

void stepperCb(const std_msgs::Int8& msg) {
  switch (msg.data) {
    case STEPPER_DOWN:
      digitalWrite(stepperDirPin, HIGH);
      currentEndPin = stepperDownEndPin;
      Timer1.start();
      break;
    case STEPPER_STOP:
      Timer1.stop();
      break;
    case STEPPER_UP:
      digitalWrite(stepperDirPin, LOW);
      currentEndPin = stepperUpEndPin;
      Timer1.start();
      break;
  }
}

void toggleStep() {
  if (!digitalRead(currentEndPin)) {
    digitalWrite(stepperStepPin, !digitalRead(stepperStepPin));
  }
}

void setup() {
  nh.initNode();
  setupGrippers();
  setupStepper();

  Timer1.initialize(100);
  Timer1.attachInterrupt(toggleStep);
  Timer1.stop();
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
  pinMode(stepperUpEndPin, INPUT);
  pinMode(stepperDownEndPin, INPUT);

  digitalWrite(stepperDirPin, LOW);
  digitalWrite(stepperStepPin, LOW);

  nh.subscribe(stepperSub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
