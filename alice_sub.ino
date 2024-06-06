#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

#include <Braccio.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

ros::NodeHandle nh;

void servoCallback(const std_msgs::Float32MultiArray& cmd) {
  // Extract the servo angles from the received message
  int m1 = cmd.data[0];  // Base
  int m2 = cmd.data[1];  // Shoulder
  int m3 = cmd.data[2];  // Elbow
  int m4 = cmd.data[3];  // Wrist vertical
  int m5 = cmd.data[4];  // Wrist rotation
  int m6 = cmd.data[5];  // Gripper

  // Constrain the servo angles to their allowed ranges
  m1 = constrain(m1, 0, 180);
  m2 = constrain(m2, 15, 165);
  m3 = constrain(m3, 0, 180);
  m4 = constrain(m4, 0, 180);
  m5 = constrain(m5, 0, 180);
  m6 = constrain(m6, 10, 73);

  // Move the servos to the desired positions
  Braccio.ServoMovement(20, m1, m2, m3, m4, m5, m6);
}

ros::Subscriber<std_msgs::Float32MultiArray> servo_sub("servo_commands", servoCallback);

void setup() {
  
  Braccio.begin();
  nh.initNode();
  nh.subscribe(servo_sub);
  
}

void loop() {
  nh.spinOnce();
  delay(100);

  // Step Delay: a milliseconds delay between the movement of each servo. Allowed values from 10 to 30 msec.
  // Base degrees. Allowed values from 0 to 180 degrees
  // Shoulder degrees. Allowed values from 15 to 165 degrees
  // Elbow degrees. Allowed values from 0 to 180 degrees
  // Wrist vertical degrees. Allowed values from 0 to 180 degrees
  // Wrist rotation degrees. Allowed values from 0 to 180 degrees
  // Gripper degrees. Allowed values from 10 to 73 degrees. 10: the tongue is open, 73: the gripper is closed.
  
//  // the arm is aligned upwards and the gripper is closed
//  delay(2000);
//  //Braccio.ServoMovement(100, 45, 90, 90, 90, 90, 0);
//  Braccio.ServoMovement(100, 45, 90, 90, 90, 180, 0);
}
