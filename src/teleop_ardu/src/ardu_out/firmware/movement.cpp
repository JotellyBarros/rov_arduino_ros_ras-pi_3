#include <Arduino.h>
#include <geometry_msgs/Twist.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

#define BRAKE 0          //Stop
#define CW 1             //Forward
#define CCW 2            //Reverse
#define CS_THRESHOLD 15  //Definition of safety current (Check: "1.3 Monster Shield Example").

// if you add or remove items from the above enum, update this value
#define AXIS_COUNT 26

// if you add or remove items from the above enum, update this value
#define BUTTON_COUNT 17

//MOTOR 1
#define MOTOR_1 0
#define EN_PIN_1 A0        //Driver Board EN
#define PWM_MOTOR_1 5      //Driver Board PWM Pin Top
#define MOTOR_1_A1_PIN 22  //Driver Board IN1 Pin Top
#define MOTOR_1_B1_PIN 23  //Driver Board IN2 Pin Top

//MOTOR 2
#define MOTOR_2 1
#define EN_PIN_2 A1        //Driver Board EN
#define PWM_MOTOR_2 6      //Driver Board PWM Pin Left
#define MOTOR_2_A1_PIN 24  //Driver Board IN1 Pin Left
#define MOTOR_2_B1_PIN 25  //Driver Board IN2 Pin Left

//MOTOR 3
#define MOTOR_3 2
#define EN_PIN_3 A2        //Driver Board EN
#define PWM_MOTOR_3 7      //Driver Board PWM Pin right
#define MOTOR_3_A1_PIN 26  //Driver Board IN1 Pin right
#define MOTOR_3_B1_PIN 27  //Driver Board IN2 Pin right

short usSpeed = 70;  //default motor speed
unsigned short usMotor_Status = BRAKE;

ros::NodeHandle nh;
char log_msg[200];
double max_linear_vel = usSpeed;
double max_angular_vel = usSpeed;

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm);

enum AxisId {
  AXIS_LEFT_STICK_HORIZONTAL,   // 0
  AXIS_LEFT_STICK_VERTICAL,     // 1
  AXIS_RIGHT_STICK_HORIZONTAL,  // 2
  AXIS_RIGHT_STICK_VERTICAL,    // 3
  NA4,
  NA5,
  NA6,
  NA7,
  AXIS_DPAD_UP,     // 8
  AXIS_DPAD_RIGHT,  // 9
  AXIS_DPAD_DOWN,   // 10
  // who knows what the left value should be...
  AXIS_DPAD_LEFT,      // 11
  AXIS_LEFT_TRIGGER,   // 12
  AXIS_RIGHT_TRIGGER,  // 13
  AXIS_LEFT_BUMPER,    // 14
  AXIS_RIGHT_BUMPER,   // 15
  AXIS_TRIANGLE,       // 16
  AXIS_CIRCLE,         // 17
  AXIS_X,              // 18
  AXIS_SQUARE,         // 19
  NA20,
  NA21,
  NA22,
  // X is left/right
  AXIS_ACCEL_X,  // 23 note: left is positive, right is negative
  // Y is front/back
  AXIS_ACCEL_Y,  // 24 note: back is positive, forward is negative
  // Z is up/down
  AXIS_ACCEL_Z,  // 25 note: can't tell what sign is what
};

enum ButtonId {
  BUTTON_SELECT,          //  0
  BUTTON_LEFT_JOYSTICK,   //  1
  BUTTON_RIGHT_JOYSTICK,  //  2
  BUTTON_START,           //  3
  BUTTON_DPAD_UP,         //  4
  BUTTON_DPAD_RIGHT,      //  5
  BUTTON_DPAD_DOWN,       //  6
  BUTTON_DPAD_LEFT,       //  7
  BUTTON_LEFT_TRIGGER,    //  8
  BUTTON_RIGHT_TRIGGER,   //  9
  BUTTON_LEFT_BUMPER,     // 10
  BUTTON_RIGHT_BUMPER,    // 11
  BUTTON_TRIANGLE,        // 12
  BUTTON_CIRCLE,          // 13
  BUTTON_X,               // 14
  BUTTON_SQUARE,          // 15
  BUTTON_PS3,             // 16
};

void joydata(const sensor_msgs::Joy& joy) {
  geometry_msgs::Twist vel_axis_left, vel_axis_right;

  if (joy.buttons[BUTTON_DPAD_UP] == 1) {
    // nh.loginfo("BUTTON_DPAD_UP");
    // AXIS_LEFT_STICK_VERTICAL
    vel_axis_left.linear.x = max_linear_vel * joy.axes[AXIS_LEFT_STICK_VERTICAL];
    dtostrf(vel_axis_left.linear.x, 1, 4, log_msg);

    if (vel_axis_left.linear.x > 0) {
      motorGo(MOTOR_1, CW, vel_axis_left.linear.x);
      nh.loginfo(log_msg);
    } else if (vel_axis_left.linear.x < 0) {
      motorGo(MOTOR_1, CCW, vel_axis_left.linear.x * -1);
      nh.loginfo(log_msg);
    } else if (vel_axis_left.linear.x != 0) {
      nh.loginfo("AXIS_LEFT_STICK_VERTICAL");
    }

    // AXIS_RIGHT_STICK_HORIZONTAL
    vel_axis_right.linear.x = max_linear_vel * joy.axes[AXIS_RIGHT_STICK_HORIZONTAL];
    dtostrf(vel_axis_right.linear.x, 1, 4, log_msg);
    nh.loginfo(log_msg);
    if (vel_axis_right.linear.x > 0) {
      motorGo(MOTOR_2, CCW, vel_axis_right.linear.x);
      motorGo(MOTOR_3, CW, vel_axis_right.linear.x);
      nh.loginfo(log_msg);
    } else if (vel_axis_right.linear.x < 0) {
      motorGo(MOTOR_2, CW, vel_axis_right.linear.x * -1);
      motorGo(MOTOR_3, CCW, vel_axis_right.linear.x * -1);
      nh.loginfo(log_msg);
    } else if (vel_axis_right.linear.x != 0) {
      nh.loginfo("AXIS_RIGHT_STICK_HORIZONTAL");
    }

    // AXIS_RIGHT_STICK_VERTICAL
    vel_axis_right.linear.z = max_linear_vel * joy.axes[AXIS_RIGHT_STICK_VERTICAL];
    dtostrf(vel_axis_right.linear.z, 1, 4, log_msg);
    if (vel_axis_right.linear.z > 0) {
      motorGo(MOTOR_2, CW, vel_axis_right.linear.z);
      motorGo(MOTOR_3, CW, vel_axis_right.linear.z);
      nh.loginfo(log_msg);
    } else if (vel_axis_right.linear.z < 0) {
      motorGo(MOTOR_2, CCW, vel_axis_right.linear.z * -1);
      motorGo(MOTOR_3, CCW, vel_axis_right.linear.z * -1);
      nh.loginfo(log_msg);
    } else if (vel_axis_right.linear.z != 0) {
      nh.loginfo("AXIS_RIGHT_STICK_VERTICAL");
    }
  }

  // BUTTON_X
  if (joy.buttons[BUTTON_RIGHT_JOYSTICK] == 1) {
    motorGo(MOTOR_1, BRAKE, 0);
    motorGo(MOTOR_2, BRAKE, 0);
    motorGo(MOTOR_3, BRAKE, 0);
    nh.loginfo("BUTTON_RIGHT_JOYSTICK");
  }
}

ros::Subscriber<sensor_msgs::Joy> sub_input_joy("joy", joydata);

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)  //Function that controls the variables: motor(0 ou 1), direction (cw ou ccw) e pwm (entra 0 e 255);
{
  if (motor == MOTOR_1) {
    if (direct == CW) {
      nh.loginfo("Motor Up");
      digitalWrite(MOTOR_1_A1_PIN, LOW);
      digitalWrite(MOTOR_1_B1_PIN, HIGH);
    } else if (direct == CCW) {
      nh.loginfo("Motor Down");
      digitalWrite(MOTOR_1_A1_PIN, HIGH);
      digitalWrite(MOTOR_1_B1_PIN, LOW);
    } else {
      nh.loginfo("Motor Stop");
      digitalWrite(MOTOR_1_A1_PIN, LOW);
      digitalWrite(MOTOR_1_B1_PIN, LOW);
    }
    analogWrite(PWM_MOTOR_1, pwm);
  }

  if (motor == MOTOR_2) {
    if (direct == CW) {
      nh.loginfo("Motor Up");
      digitalWrite(MOTOR_2_A1_PIN, LOW);
      digitalWrite(MOTOR_2_B1_PIN, HIGH);
    } else if (direct == CCW) {
      nh.loginfo("Motor Down");
      digitalWrite(MOTOR_2_A1_PIN, HIGH);
      digitalWrite(MOTOR_2_B1_PIN, LOW);
    } else {
      nh.loginfo("Motor Stop");
      digitalWrite(MOTOR_2_A1_PIN, LOW);
      digitalWrite(MOTOR_2_B1_PIN, LOW);
    }
    analogWrite(PWM_MOTOR_2, pwm);
  }

  if (motor == MOTOR_3) {
    if (direct == CW) {
      nh.loginfo("Motor Up");
      digitalWrite(MOTOR_3_A1_PIN, LOW);
      digitalWrite(MOTOR_3_B1_PIN, HIGH);
    } else if (direct == CCW) {
      nh.loginfo("Motor Down");
      digitalWrite(MOTOR_3_A1_PIN, HIGH);
      digitalWrite(MOTOR_3_B1_PIN, LOW);
    } else {
      nh.loginfo("Motor Stop");
      digitalWrite(MOTOR_3_A1_PIN, LOW);
      digitalWrite(MOTOR_3_B1_PIN, LOW);
    }
    analogWrite(PWM_MOTOR_3, pwm);
  }
}

void setup() {
  nh.initNode();
  // setupPins();

  // pinMode MOTOR_1
  pinMode(EN_PIN_1, OUTPUT);
  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(MOTOR_1_A1_PIN, OUTPUT);
  pinMode(MOTOR_1_B1_PIN, OUTPUT);

  // pinMode MOTOR_2
  pinMode(EN_PIN_2, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);
  pinMode(MOTOR_2_A1_PIN, OUTPUT);
  pinMode(MOTOR_2_B1_PIN, OUTPUT);

  // pinMode MOTOR_3
  pinMode(EN_PIN_3, OUTPUT);
  pinMode(PWM_MOTOR_3, OUTPUT);
  pinMode(MOTOR_3_A1_PIN, OUTPUT);
  pinMode(MOTOR_3_B1_PIN, OUTPUT);

  // Active pin_out
  digitalWrite(EN_PIN_1, HIGH);
  digitalWrite(EN_PIN_2, HIGH);
  digitalWrite(EN_PIN_3, HIGH);

  nh.loginfo("Enter number for control option:");
  nh.loginfo("1. STOP");
  nh.loginfo("2. FORWARD");
  nh.loginfo("3. REVERSE");
  nh.loginfo("4. READ CURRENT");
  nh.loginfo("+. INCREASE SPEED");
  nh.loginfo("-. DECREASE SPEED");

  nh.subscribe(sub_input_joy);
}

void loop() {
  nh.spinOnce();
  // delay(1);
}