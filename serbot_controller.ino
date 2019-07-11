#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Servo.h>
/* L9110s 모터드라이버
   오른쪽모터
   L9110s A_1A D6
   L9110s A_1B D11
   왼쪽모터
   L9110s B_1A D3
   L9110s B_1B D5
*/
int A_1A = 6;
int A_1B = 11;
int B_1A = 3;
int B_1B = 5;

double w_r=0, w_l=0;
 
//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.0325, wheel_sep = 0.295;

ros::NodeHandle nh;

double speed_ang=0, speed_lin=0;

int motorASpeed = 150; // 모터A 속도 (0~255)
int motorBSpeed = 150; // 모터B 속도 (0~255)

void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

void setup() {
  Motors_init();

  nh.initNode();

  nh.subscribe(sub);
}

void Motors_init() {
  // 문제발생시 테스트를 위해 시리얼모니터를 시작합니다.
  Serial.begin(9600);
  //핀을 초기화합니다.
  //L9110S 모터드라이버의 핀들을 출력으로 변경합니다.
  pinMode(A_1A, OUTPUT);
  pinMode(A_1B, OUTPUT);
  pinMode(B_1A, OUTPUT);
  pinMode(B_1B, OUTPUT);
  digitalWrite(A_1A, LOW);
  digitalWrite(A_1B, LOW);
  digitalWrite(B_1A, LOW);
  digitalWrite(B_1B, LOW);
}

void loop() {
  MotorL(w_l * 10);
  MotorR(w_r * 10);

  nh.spinOnce();
}

void MotorL(int Pulse_Width1) {
  if(Pulse_Width1 > 0) {
    analogWrite(B_1A, Pulse_Width1);
    analogWrite(B_1B, 0);
    digitalWrite(B_1A, HIGH);
    digitalWrite(B_1B, LOW);
  }
  if(Pulse_Width1 < 0) {
    Pulse_Width1 = abs(Pulse_Width1);

    analogWrite(B_1A, Pulse_Width1);
    analogWrite(B_1B, 0);
    digitalWrite(B_1A, LOW);
    digitalWrite(B_1B, HIGH);
  }
  if(Pulse_Width1 == 0) {
    analogWrite(B_1A, Pulse_Width1);
    analogWrite(B_1B, 0);
    digitalWrite(B_1A, LOW);
    digitalWrite(B_1B, LOW);
  }
}

void MotorR(int Pulse_Width2) {
if(Pulse_Width2 > 0) {
    analogWrite(A_1A, 0);
    analogWrite(A_1B, Pulse_Width2);
    digitalWrite(A_1A, HIGH);
    digitalWrite(A_1B, LOW);
  }
  if(Pulse_Width2 < 0) {
    Pulse_Width2 = abs(Pulse_Width2);

    analogWrite(A_1A, 0);
    analogWrite(A_1B, Pulse_Width2);
    digitalWrite(A_1A, LOW);
    digitalWrite(A_1B, HIGH);
  }
  if(Pulse_Width2 == 0) {
    analogWrite(A_1A, Pulse_Width2);
    analogWrite(A_1B, 0);
    digitalWrite(A_1A, LOW);
    digitalWrite(A_1B, LOW);
  }
}
