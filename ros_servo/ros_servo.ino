/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

//라이브러리 해더 포함 
#include <Arduino.h>
#include <Servo.h> 
#include <ros.h>
//여러 데이터 받기 위함.
#include <std_msgs/UInt16MultiArray.h>

//ros 노드 핸들 
ros::NodeHandle  nh;

//서보 제어용 
//x가 y보다 크면 + step
//x가 y보다 작으면 - step
#define getStep(x, y) (x>y?SERVO_POSITION_STEP:-SERVO_POSITION_STEP)

/* 
 * x값이 0보가 크면 양수 x
 * x값이 0보다 작으면 -를 제거한 양수 x 
 * Ex:
 * getAbs(-12)  : 12
 * getAbs(12)   : 12
 */
#define getAbs(x) (x>=0?x:-(x))

// 서보 초기 위치 
#define POWER_ON_POS 110
// 서보 이동 폭
#define SERVO_POSITION_STEP 10

// 서보 팔(손, 팔, 어깨가 한 세트)의 갯수 
#define MAX_ARM_COUNT 1

// 서보 팔의 인덱스
enum{
 ARM_LEFT = 0,
 ARM_RIGHT,
};

//서보 클래스
Servo servoFinger[MAX_ARM_COUNT];
Servo servoArm[MAX_ARM_COUNT];
Servo servoShoulder[MAX_ARM_COUNT];

// 서보 도착 위치 
int goalServoPosFinger[MAX_ARM_COUNT]   = {POWER_ON_POS};
int goalServoPosArm[MAX_ARM_COUNT]      = {POWER_ON_POS};
int goalServoPosShoulder[MAX_ARM_COUNT] = {POWER_ON_POS};

// 서보 현재 위치 
int CurrentServoPosFinger[MAX_ARM_COUNT]   = {POWER_ON_POS};
int CurrentServoPosArm[MAX_ARM_COUNT]      = {POWER_ON_POS};
int CurrentServoPosShoulder[MAX_ARM_COUNT] = {POWER_ON_POS};

// 서보 핀 
int servoFingerPin[MAX_ARM_COUNT]  = {PB6};
int servoArmPin[MAX_ARM_COUNT]     = {PB7};
int servoShoulderPin[MAX_ARM_COUNT]  = {PB8};

// LED pin
int ledPin = PC13;



/*
 * ROS 콜백 함수 
 * 서보 데이터 받기
 */
void servo_cb( const std_msgs::UInt16MultiArray& cmd_msg){
  goalServoPosFinger[ARM_LEFT]   = cmd_msg.data[0];   
  goalServoPosArm[ARM_LEFT]      = cmd_msg.data[1];      
  goalServoPosShoulder[ARM_LEFT] = cmd_msg.data[2];   
  digitalWrite(ledPin, HIGH-digitalRead(ledPin));  
}


// 노드 servo 메시지 입력과 콜백 함수와 연결 
ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo", servo_cb);


/*
 * 초기 설정 
 */
void setup()
{
  //LED 핀 설정
  pinMode(ledPin, OUTPUT);

  // 노드 설정
  nh.initNode();
  nh.subscribe(sub);


  //서보 PIN 설정
  for(int i=0;i<MAX_ARM_COUNT;i++)
  {
    servoFinger[i].attach(servoFingerPin[i]);
    servoArm[i].attach(servoArmPin[i]); 
    servoShoulder[i].attach(servoShoulderPin[i]);
  }  
}


/*
 * 현재 서보 위치에서 지정 위치로 이동
 * 한번 호출시 SERVO_POSITION_STEP만큼 움직임
 */
void SetServoPos(Servo* servo, int* cPos, int* rPos)
{
 if(*cPos != *rPos)
 {
  *cPos += getStep(*rPos, *cPos);
  if(getAbs(*rPos - *cPos) <= SERVO_POSITION_STEP)
    *cPos = *rPos;
 }
 servo->write(*cPos);
}


/*
 * 서보 위치 제어 함수
 */
void servoProc()
{ 
  for(int i=0;i<MAX_ARM_COUNT;i++)
  {
    SetServoPos(&servoFinger[i], &CurrentServoPosFinger[i], &goalServoPosFinger[i]);
    SetServoPos(&servoShoulder[i], &CurrentServoPosShoulder[i], &goalServoPosShoulder[i]);
    SetServoPos(&servoArm[i], &CurrentServoPosArm[i], &goalServoPosArm[i]);
  }
}


/*
 * 루프
 */
void loop()
{
  //ROS 콜백 함수 
  nh.spinOnce();
  
  //서보 위치 제어 함수
  servoProc();  
  delay(1);  
}
