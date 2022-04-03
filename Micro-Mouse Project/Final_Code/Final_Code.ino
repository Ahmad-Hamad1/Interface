#include <PIDController.h>
#include <PID_v1.h>
#include <NewPing.h>


#define __Kp 260 // Proportional constant
#define __Ki 2.7 // Integral Constant
#define __Kd 2000 // Derivative Constant

int encoderPin1 = 13; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 3; //Encoder Otput 'B' must connected with intreput pin of arduino.

int encoderPin3 = A3; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin4 = 2; //Encoder Otput 'B' must connected with intreput pin of arduino.

int forwardCycles = 260; // required cycles to move forward
int turnCycles = 75; // required cycles to turn

/*************FOR PIDs*************/
// Pid controller object
volatile long int encoder_countR = 0;
int motor_pwm_value = 255; // after PID computation data is stored in this variable.

volatile long int encoder_countL = 0;
int motor_pwm_value2 = 255; // after PID computation data is stored in this variable.
/***********************************/


/***********FOR ULTRA SONIC*********/
const int leftEchoPin = 11;
const int leftTrigPin = 12;

const int rightEchoPin = 5;
const int rightTrigPin = A0;
const int frontTrigPin = 4;

const int rightIr = A5;
const int leftIr = A4;
const int frontIr = 4;

NewPing sonarR(rightTrigPin, rightEchoPin, 300);
NewPing sonarL(leftTrigPin, leftEchoPin, 300);


volatile int leftDistance;
volatile int rightDistance;
volatile int forwardDistance;
volatile int prevTime = 0;
/***********************************/


/***********FOR MOTORS*************/
// Motor Left connections
const int enL = 9;
const int in2 = A2; // replaced with 6
const int in1 = A1; // replaced with 2

// Motor Right connections
const int enR = 10;
const int in3 = 7;
const int in4 = 8;
/***********************************/


double rpmL = 0.0;
double rpmR = 0.0;

// Float for the desired RPM
double rpm_setL = 0;
double rpm_setR = 0;

// Byte for PWM of each Motor
double pwmL = 0;
double pwmR = 0;

// PID controller for each Motor
double kp = 2, ki = 5, kd = 1;

double error = 0, desired = 0, dPWM = 0;
volatile double currentError = 0;


PID pidL(&rpmL, &pwmL, &rpm_setL, kp, ki, kd, DIRECT);
PID pidR(&rpmR, &pwmR, &rpm_setR, kp, ki, kd, DIRECT);
PID pidNew(&error, &dPWM, &desired, kp, ki, kd, DIRECT);
const int buzzer = 6;

void setup() {
  Serial.begin(9600);

  pinMode(encoderPin1, INPUT);
  pinMode(encoderPin2, INPUT);
  pinMode(encoderPin3, INPUT);
  pinMode(encoderPin4, INPUT);

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(digitalPinToInterrupt(encoderPin2), updateEncoderR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin4), updateEncoderL, RISING);

  //
  pinMode(leftTrigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(leftEchoPin, INPUT); // Sets the echoPin as an INPUT

  pinMode(rightTrigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(rightEchoPin, INPUT); // Sets the echoPin as an INPUT



  // Motor control pins are outputs
  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  pidL.SetMode(AUTOMATIC);
  pidR.SetMode(AUTOMATIC);
  pidNew.SetMode(AUTOMATIC);
  //go_forward_initial();
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  pinMode(leftIr, INPUT);
  pinMode(rightIr, INPUT);
  pinMode(frontIr, INPUT);
  pinMode(buzzer, OUTPUT);
  pidNew.SetOutputLimits(0, 20);
  pidL.SetOutputLimits(0, 75);
  pidR.SetOutputLimits(0, 75);
}

int baseSpeed = 120;
void loop() {
  delay(2000);
  //forward2(80);
  //go_forward();
  //exit(0);
  //solve();
  //playTone();
  leftHand();
  //playTone();
  //exit(0);
  //Serial.println(wallRight());
}

int x = 0;
int y = 0;
int rightCount = 0;
void leftHand() {

  while (1) {
    if (rightCount == 3) {
      digitalWrite(in2, LOW);
      digitalWrite(in1, HIGH);
      digitalWrite(in4, HIGH);
      digitalWrite(in3, LOW);
      analogWrite(enL, 80);
      analogWrite(enR,80);
      for (int i = 0; i < 50; i++) {
        analogWrite(6, 100);
        delay(200);
        analogWrite(6, 0);
        delay(200);
      }
      delay(3000);
      stopAll();
      delay(1000);
      exit(0);
    }
    if (digitalRead(leftIr) == 1) {
      rightCount = 0;
      turn_left(90, 80);
      forward2(85);
    } else if (digitalRead(frontIr) == 1) {
      rightCount = 0;
      forward2(85);
    } else if (digitalRead(rightIr) == 1) {
      rightCount++;
      turn_right(90, 85);
      forward2(85);
    } else {
      rightCount = 0;
      turn_left(90, 85);
      turn_left(90, 85);
    }
  }
}

void solve() {
  while (1) {
    forward2(75);
    error = 0;
    if (digitalRead(rightIr) == 1) {
      turn_right(90, 75);
    } else if (digitalRead(leftIr) == 1) {
      turn_left(90, 75);
    } else {
      turn_right(90, 75);
      delay(500);
      turn_right(90, 75);
    }
    delay(1000);
  }
}

void forward2(int mspeed) {
  encoder_countL = 0;  //  reset counter A to zero
  encoder_countR = 0;  //  reset counter B to zero
  // Set Motor A forward
  //Serial.println(wallFront());
  int type;
  if (digitalRead(rightIr) == 1 && digitalRead(leftIr) == 0) { // digitalRead(rightIr) == 1 && digitalRead(leftIr) == 0
    type = 0;
  }  else if (digitalRead(leftIr) == 1 && digitalRead(rightIr) == 0) { //digitalRead(leftIr) == 1 && digitalRead(rightIr) == 0
    type = 1;
  } else if (digitalRead(leftIr) == 1 && digitalRead(rightIr) == 1) { //digitalRead(leftIr) == 1 && digitalRead(rightIr) == 1
    type = 2;
  } else {
    type = 3;
  }
  digitalWrite(in2, HIGH);
  digitalWrite(in1, LOW);
  // Set Motor B forward
  digitalWrite(in4, HIGH);
  digitalWrite(in3, LOW);
  encoder_countR = encoder_countL = 0;
  // Go forward until step value is reached
  while (digitalRead(frontIr) == 1 && (encoder_countR + encoder_countL) / 2   <= 280) {
    myPID(type);
    //slow M2, accelerate M1 speed
    //Serial.println("Eror: " + String(error) + "  dPwm: " + String(dPWM));
    if (currentError > 0) {
      analogWrite(enL, mspeed + (int) (20 / 2));
      analogWrite(enR, mspeed - (int) (20 / 2));
    }
    //slow M1, accelerate M2 speed
    else if (currentError < 0) {
      analogWrite(enL, mspeed - (int) (20 / 2));
      analogWrite(enR, mspeed + (int) (20 / 2));
    }
    else {
      analogWrite(enL, 82);
      analogWrite(enR, 75);
    }
    delay(0);
    //}
  }
  stopAll();
  delay(1000);
  // Stop when done
  //delay(1000);
}

void myPID(int type) {
  int rightWall = wallRight();
  int leftWall = wallLeft();
  if (type == 0) { // digitalRead(rightIr) == 1 && digitalRead(leftIr) == 0
    if (leftWall < 12)
      currentError = 1;
    else
      currentError = -1;
    return;
  }  else if (type == 1) { //digitalRead(leftIr) == 1 && digitalRead(rightIr) == 0
    if (rightWall < 12)
      currentError = -1;
    else
      currentError = 1;
    return;
  } else if (type == 2) { //digitalRead(leftIr) == 1 && digitalRead(rightIr) == 1
    currentError = 0;
    return;
  }
  error = rightWall - leftWall;
  currentError = error;
  error = abs(error);
  //pidNew.Compute();
}

void myPID2() {
  error = encoder_countR - encoder_countL;
  pidNew.Compute();
}

void updateEncoderL() {
  //  if (digitalRead(encoderPin3) == HIGH)
  encoder_countR++;
  //  else
  //    encoder_countL--;
}


void updateEncoderR() {
  //  if (digitalRead(encoderPin1) == HIGH)
  //    encoder_countR--;
  //  else
  encoder_countL++;
}


void go_forward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  encoder_countR = encoder_countL = 0;
  while ((encoder_countR + encoder_countL) / 2   < 280) {
    rpmL = encoder_countL;
    rpmR = encoder_countR;
    rpm_setL = 280;
    rpm_setR = 280;
    PIdMotors();
  }
  stopAll();
}

void turn_right(int steps, int mspeed) {
  encoder_countL = 0;  //  reset counter A to zero
  encoder_countR = 0;  //  reset counter B to zero
  // Set Motor A forward
  digitalWrite(in2, HIGH);
  digitalWrite(in1, LOW);
  // Set Motor B forward
  digitalWrite(in4, LOW);
  digitalWrite(in3, HIGH);
  error = 0;
  // Go forward until step value is reached
  while (steps > (encoder_countL + encoder_countR) / 2) {
    myPID2();
    //slow M2, accelerate M1 speed
    if (error >= 0) {
      if (steps > encoder_countL) {
        analogWrite(enL, mspeed + (dPWM / 2));
      }
      if (steps > encoder_countR) {
        analogWrite(enR, mspeed - (dPWM / 2));
      }
    }
    //slow M1, accelerate M2 speed
    if (error < 0) {
      if (steps > encoder_countL) {
        analogWrite(enL, mspeed - (dPWM / 2));
      }
      if (steps > encoder_countR) {
        analogWrite(enR, mspeed + (dPWM / 2));
      }
    }
    delay(0);
  }
  // Stop when done
  stopAll();
  delay(1000);
}

void turn_left(int steps, int mspeed) {
  encoder_countL = 0;  //  reset counter A to zero
  encoder_countR = 0;  //  reset counter B to zero
  // Set Motor A forward
  digitalWrite(in2, LOW);
  digitalWrite(in1, HIGH);
  // Set Motor B forward
  digitalWrite(in4, HIGH);
  digitalWrite(in3, LOW);
  error = 0;
  // Go forward until step value is reached
  while (steps > (encoder_countL + encoder_countR) / 2) {
    myPID2();
    //slow M2, accelerate M1 speed
    if (error >= 0) {
      if (steps > encoder_countL) {
        analogWrite(enL, mspeed + (dPWM / 2));
      }
      if (steps > encoder_countR) {
        analogWrite(enR, mspeed - (dPWM / 2));
      }
    }
    //slow M1, accelerate M2 speed
    if (error < 0) {
      if (steps > encoder_countL) {
        analogWrite(enL, mspeed - (dPWM / 2));
      }
      if (steps > encoder_countR) {
        analogWrite(enR, mspeed + (dPWM / 2));
      }
    }
    delay(0);
  }
  // Stop when done
  stopAll();
  delay(1000);
}

void stopAll() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enL, 0);
  analogWrite(enR, 0);
}

void PIdMotors() {
  pidL.Compute();
  Serial.println(pwmL);
  analogWrite(enL, pwmL);

  pidR.Compute();
  analogWrite(enR, pwmR);
}
int wallLeft() {
  return sonarL.ping_cm();
}

int wallRight() {
  return sonarR.ping_cm();
}
