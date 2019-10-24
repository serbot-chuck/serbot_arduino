#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <AFMotor.h>

#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define MOTOR3_PWM 6
#define MOTOR4_PWM 5 

//left 1 2
//right 3 4

AF_DCMotor motor_1(1);
AF_DCMotor motor_2(2);
AF_DCMotor motor_3(3);
AF_DCMotor motor_4(4);

double w_r=0, w_l=0;
 
//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.04, wheel_sep = 0.1;

ros::NodeHandle nh;
std_msgs::String str_msg;

double speed_ang=0, speed_lin=0;

ros::Publisher pulse_width("pulse_width", &str_msg);

void messageCb(const geometry_msgs::Twist& msg){
  speed_lin = msg.linear.x;
  speed_ang = msg.angular.z;
//  w_l = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
//  w_r = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));

  w_l = 100*speed_lin + 50*speed_ang;
  w_r = 100*speed_lin - 50*speed_ang;

//  std_msgs::String str;
//  str.data = w_l.c_str();
//  pub_pulse.publish(&str);
}

void motorCb(const std_msgs::String& stop_msgs) {
   motor_1.setSpeed(0);
   motor_2.setSpeed(0);
   motor_3.setSpeed(0);
   motor_4.setSpeed(0);

   motor_1.run(RELEASE);
   motor_2.run(RELEASE);
   motor_3.run(RELEASE);
   motor_4.run(RELEASE);
   
    delay(1000);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
ros::Subscriber<std_msgs::String> sub_stop("fullbodydetect/stop", &motorCb );

void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

void setup() {
  // 문제발생시 테스트를 위해 시리얼모니터를 시작합니다.
  
  Motors_init();

  nh.initNode();

  nh.subscribe(sub);
  nh.subscribe(sub_stop);
  nh.advertise(pulse_width);
}


void Motors_init() {
  //핀을 초기화합니다.
  //L9110S 모터드라이버의 핀들을 출력으로 변경합니다.
   Serial.begin(57600);
   
   motor_1.setSpeed(100);
   motor_2.setSpeed(100);
   motor_3.setSpeed(100);
   motor_4.setSpeed(100);

   motor_1.run(RELEASE);
   motor_2.run(RELEASE);
   motor_3.run(RELEASE);
   motor_4.run(RELEASE);
}

void loop() {
   
  MotorL(w_l * 10);
  MotorR(w_r * 10);

  nh.spinOnce();
}

void MotorL(int Pulse_Width1) {
  if(Pulse_Width1 > 0) {
    
    analogWrite(MOTOR3_PWM, Pulse_Width1);
    motor_3.run(BACKWARD);

    analogWrite(MOTOR4_PWM, Pulse_Width1);
    motor_4.run(BACKWARD);
  }
  if(Pulse_Width1 < 0) {
    Pulse_Width1 = abs(Pulse_Width1);

    analogWrite(MOTOR3_PWM, Pulse_Width1);
    motor_3.run(FORWARD);

    analogWrite(MOTOR4_PWM, Pulse_Width1);
    motor_4.run(FORWARD);
  }
  if(Pulse_Width1 == 0) {

    analogWrite(MOTOR3_PWM, Pulse_Width1);
    motor_3.run(RELEASE);

    analogWrite(MOTOR4_PWM, Pulse_Width1);
    motor_4.run(RELEASE);
  }
}

void MotorR(int Pulse_Width2) {
if(Pulse_Width2 > 0) {
    analogWrite(MOTOR1_PWM, Pulse_Width2);
    motor_1.run(FORWARD);

    analogWrite(MOTOR2_PWM, Pulse_Width2);
    motor_2.run(FORWARD);
  }
  if(Pulse_Width2 < 0) {
    Pulse_Width2 = abs(Pulse_Width2);

    analogWrite(MOTOR1_PWM, Pulse_Width2);
    motor_1.run(BACKWARD);

    analogWrite(MOTOR2_PWM, Pulse_Width2);
    motor_2.run(BACKWARD);
  }
  if(Pulse_Width2 == 0) {
    analogWrite(MOTOR1_PWM, Pulse_Width2);
    motor_1.run(RELEASE);

    analogWrite(MOTOR2_PWM, Pulse_Width2);
    motor_2.run(RELEASE);
  }
}
