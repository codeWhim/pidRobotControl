/*
 * Chotu V2 Controller Layer
 * 
 * Code By Abdul Samad
 * customscrypts.com
 * 
 */
 
#include <AutoPID.h>

// Can Range from 0 to 255
#define robot_speed_min 200
#define robot_speed_max 255

#define KP 0.012
#define KI 0.03
#define KD 0.0001

#define motor_ENA 4
#define motor_IN1 5
#define motor_IN2 6
#define motor_IN3 7
#define motor_IN4 8
#define motor_ENB 9

#define encoder_L_C1 3 // Base Motor # 1
#define encoder_R_C1 2 // Base Motor # 2

#define Front 1
#define Rear 2
#define Right 3
#define Left 4

#define Stop 0
#define Start 1
#define CW 2
#define CCW 3

double countLeft=0;
double countRight=0;
double stepsOverall=0;
double stepsMotion=0;
double pwmLeft,pwmRight;
int currentMotion=0;

AutoPID leftPID(&countLeft, &countRight, &pwmLeft, robot_speed_min, robot_speed_max, KP, KI, KD);
AutoPID rightPID(&countRight, &countLeft, &pwmRight, robot_speed_min, robot_speed_max, KP, KI, KD);

void setup() {
  pinMode(motor_ENA,OUTPUT);
  pinMode(motor_IN1,OUTPUT);
  pinMode(motor_IN2,OUTPUT);
  pinMode(motor_IN3,OUTPUT);
  pinMode(motor_IN4,OUTPUT);
  pinMode(motor_ENB,OUTPUT);

  pinMode(encoder_L_C1,INPUT);
  pinMode(encoder_R_C1,INPUT);

  attachInterrupt(digitalPinToInterrupt(encoder_L_C1), encoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_R_C1), encoderRight, RISING);

  // If input is more than value mentioned below or above setpoint, PWM will be set to min or max respectively
  leftPID.setBangBang(20);
  rightPID.setBangBang(20);
  
  // PID update interval
  leftPID.setTimeStep(4);
  rightPID.setTimeStep(4);

  Serial.begin(9600);
}

void loop() {

  moveForward(10000);
  printStatus();
  
}

String motionText(int motionInteger){
  switch(motionInteger){
    case Stop:
    return "Stop";
    break;
    case Front:
    return "Front";
    break;
    case Rear:
    return "Rear";
    break;
    case Left:
    return "Left";
    break;
    case Right:
    return "Right";
    break;
  }
}

void printStatus(){
  Serial.print(motionText(currentMotion));
  Serial.print("; Count; L:");
  Serial.print(countLeft);
  Serial.print(", R:");
  Serial.print(countRight);
  Serial.print(", Steps; Overall:");
  Serial.print(stepsOverall);
  Serial.print(", Steps; Motion:");
  Serial.print(stepsMotion);
  Serial.print(", PWM; L:");
  Serial.print(pwmLeft);
  Serial.print(", R:");
  Serial.print(pwmRight);
  Serial.println();
}

void encoderLeft(){
  countLeft++;
  stepsOverall = (countLeft + countRight)/2;
}
void encoderRight(){
  countRight++;
  stepsOverall = (countLeft + countRight)/2;
}

void turnLeft(double steps){
  moveDirection(Left, steps);
}
void turnRight(double steps){
  moveDirection(Right, steps);
}
void moveForward(double steps){
  moveDirection(Front, steps);
}
void moveBackward(double steps){
  moveDirection(Rear, steps);
}

void stopMotion(){
  motor(Left,Stop,Stop);
  motor(Right,Stop,Stop);
  currentMotion = Stop;
}

void resetMotionSteps(){
  stepsMotion = stepsOverall;
}

void moveDirection(int Direction, double steps){
  if(stepsOverall-stepsMotion<steps){
    if(currentMotion==Direction){
      
      rightPID.run();
      leftPID.run();
      
      switch(Direction){
        case Front:
        motor(Right, pwmRight, CCW);
        motor(Left, pwmLeft, CW);
        break;
        
        case Rear:
        motor(Right, pwmRight, CW);
        motor(Left, pwmLeft, CCW);
        break;
        
        case Right:
        motor(Right, pwmRight, CW);
        motor(Left, pwmLeft, CW);
        break;
        
        case Left:
        motor(Right, pwmRight, CCW);
        motor(Left, pwmLeft, CCW);
        break;
      }  
      
      
    }else{
      currentMotion=Direction;
      resetMotionSteps();
    }
  }else{
    stopMotion();
  }
}

void motor(int LeftRight, int pwm, int CWCCW){
      switch(LeftRight){
        case Right:
        switch(CWCCW){
          case CW:
          digitalWrite(motor_IN1,HIGH);
          digitalWrite(motor_IN2,LOW);
          break;
          case CCW:
          digitalWrite(motor_IN1,LOW);
          digitalWrite(motor_IN2,HIGH);
          break;
        }
          //digitalWrite(motor_ENA,HIGH);
          analogWrite(motor_ENA,pwm);
        break;
        case Left:
        switch(CWCCW){
          case CW:
          digitalWrite(motor_IN3,HIGH);
          digitalWrite(motor_IN4,LOW);
          break;
          case CCW:
          digitalWrite(motor_IN3,LOW);
          digitalWrite(motor_IN4,HIGH);
          break;
        }
          //digitalWrite(motor_ENB,HIGH);
          analogWrite(motor_ENB,pwm);
        break;
      }
}
