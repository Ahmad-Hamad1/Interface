#include <StackArray.h>
#include <PIDController.h>
#include <PID_v1.h>
#include <NewPing.h>

#define __Kp 260 // Proportional constant
#define __Ki 2.7 // Integral Constant
#define __Kd 2000 // Derivative Constant

//#define int byte

int encoderPin1 = 13; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 3; //Encoder Otput 'B' must connected with intreput pin of arduino.

int encoderPin3 = A3; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin4 = 2; //Encoder Otput 'B' must connected with intreput pin of arduino.

int forwardCycles = 260; // required cycles to move forward
int turnCycles = 75; // required cycles to turn

/*************FOR PIDs*************/
// Pid controller object
volatile int encoder_countR = 0;
int motor_pwm_value = 255; // after PID computation data is stored in this variable.

volatile int encoder_countL = 0;
int motor_pwm_value2 = 255; // after PID computation data is stored in this variable.
/***********************************/


/***********FOR ULTRA SONIC*********/
const int leftEchoPin = 11;
const int leftTrigPin = 12;

const int rightEchoPin = 5;
const int rightTrigPin = A0;

const int frontEchoPin = 6;
const int frontTrigPin = 4;

// Ir sensors pins
const int rightIr = A5;
const int leftIr = A4;
const int frontIr = 4;

NewPing sonarF(frontTrigPin, frontEchoPin, 300);
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

double error = 0, desired = 0, dPWM = 0, currentError = 0;


PID pidL(&rpmL, &pwmL, &rpm_setL, kp, ki, kd, DIRECT);
PID pidR(&rpmR, &pwmR, &rpm_setR, kp, ki, kd, DIRECT);
PID pidNew(&error, &dPWM, &desired, kp, ki, kd, DIRECT);


const int MAZE_SIZE_Y = 8;
const int MAZE_SIZE_X = 8;
int wall[MAZE_SIZE_X][MAZE_SIZE_Y][4] = {
  0
};  // 0 : north, 1 : east, 2 : south, 3 : west

int m[27];
int arr[MAZE_SIZE_X][MAZE_SIZE_Y];
int color[MAZE_SIZE_X][MAZE_SIZE_Y] = {0};

struct pair {
  int first;
  int second;
};

pair temp;
pair front;
StackArray <pair> st;

const int mspeed = 90;


// startup point entry (runs once).
void
setup () {
  // start serial communication.
  Serial.begin (9600);
  for (int y = 0; y < MAZE_SIZE_Y / 2; y += 1) {
    int distance = MAZE_SIZE_X / 2 + MAZE_SIZE_Y / 2 - 2 - y;
    for (int x = 0; x < MAZE_SIZE_X / 2; x += 1) {
      arr[x][y] = distance;
      arr[x][MAZE_SIZE_Y - 1 - y] = distance;
      arr[MAZE_SIZE_X - 1 - x][MAZE_SIZE_Y - 1 - y] = distance;
      arr[MAZE_SIZE_X - 1 - x][y] = distance;
      distance -= 1;
    }
  }
  m['N' - 'A'] = 0;
  m['E' - 'A'] = 1;
  m['S' - 'A'] = 2;
  m['W' - 'A'] = 3;
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

  pinMode(frontTrigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(frontEchoPin, INPUT); // Sets the echoPin as an INPUT

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
  pidNew.SetOutputLimits(0, 20);
}

void loop() {
  delay(2000);
  //Serial.println("Right : " + String(wallRight()) + "Left : " + String(wallLeft()));
  floodFill();

}

int getXY(int x, int y, char direction, char turn, int flag) {
  if (direction == 'N') {
    if (turn == 'R') {
      x++;
    } else if (turn == 'L') {
      x--;
    } else if (turn == 'F') {
      y++;
    } else {
      y--;
    }
  }

  if (direction == 'S') {
    if (turn == 'R') {
      x--;
    } else if (turn == 'L') {
      x++;
    } else if (turn == 'F') {
      y--;
    } else {
      y++;
    }
  }

  if (direction == 'E') {
    if (turn == 'R') {
      y--;
    } else if (turn == 'L') {
      y++;
    } else if (turn == 'F') {
      x++;
    } else {
      x--;
    }
  }

  if (direction == 'W') {
    if (turn == 'R') {
      y++;
    } else if (turn == 'L') {
      y--;
    } else if (turn == 'F') {
      x--;
    } else {
      x++;
    }
  }
  return flag == 0 ? x : y;
}
char getNewDirection(char direction, char turn) {
  if (direction == 'N') {
    if (turn == 'R') {
      return 'E';
    } else if (turn == 'L') {
      return 'W';
    } else if (turn == 'F') {
      return 'N';
    } else {
      return 'S';
    }
  }

  if (direction == 'S') {
    if (turn == 'R') {
      return 'W';
    } else if (turn == 'L') {
      return 'E';
    } else if (turn == 'F') {
      return 'S';
    } else {
      return 'N';
    }
  }

  if (direction == 'E') {
    if (turn == 'R') {
      return 'S';
    } else if (turn == 'L') {
      return 'N';
    } else if (turn == 'F') {
      return 'E';
    } else {
      return 'W';
    }
  }

  if (direction == 'W') {
    if (turn == 'R') {
      return 'N';
    } else if (turn == 'L') {
      return 'S';
    } else if (turn == 'F') {
      return 'W';
    } else {
      return 'E';
    }
  }
  return 'r';
}

char inverseDirection(char direction) {
  if (direction == 'S') {
    return 'N';
  }
  if (direction == 'N') {
    return 'S';
  }
  if (direction == 'W') {
    return 'E';
  }
  if (direction == 'E') {
    return 'W';
  }
}

bool checkCenter(int x, int y) {
  if (x == MAZE_SIZE_X / 2 && y == MAZE_SIZE_Y / 2) return false;
  if (x == MAZE_SIZE_X / 2 && y == MAZE_SIZE_Y / 2 - 1) return false;
  if (x == MAZE_SIZE_X / 2 - 1 && y == MAZE_SIZE_Y / 2) return false;
  if (x == MAZE_SIZE_X / 2 - 1 && y == MAZE_SIZE_Y / 2 - 1) return false;
  return x > -1 && x < MAZE_SIZE_X && y > -1 && y < MAZE_SIZE_Y;
}

bool checkValid(int x, int y) {
  return x > -1 && x < MAZE_SIZE_X && y > -1 && y < MAZE_SIZE_Y;
}



void floodFill() {
  int x = 0, y = 0, direction = 'N', turn = 'F', prev = 100, centers = 0;
  int back = 0;
  while (true) {
    if (!checkCenter(x, y))
      break;
    int mmin = prev;
    if (digitalRead(rightIr) == 1) {
      mmin = min(mmin, arr[getXY(x, y, direction, 'R', 0)]
                 [getXY(x, y, direction, 'R', 1)]);
    } else {
      wall[x][y][m[getNewDirection(direction, 'R') - 'A']] = 1;
      if (checkValid(getXY(x, y, direction, 'R', 0),
                     getXY(x, y, direction, 'R', 1)))
        wall[getXY(x, y, direction, 'R', 0)][getXY(x, y, direction, 'R', 1)]
        [m[inverseDirection(getNewDirection(direction, 'R'))] - 'A'] = 1;
    }
    // left
    if (digitalRead(leftIr) == 1) {
      mmin = min(mmin, arr[getXY(x, y, direction, 'L', 0)]
                 [getXY(x, y, direction, 'L', 1)]);
    } else {
      wall[x][y][m[getNewDirection(direction, 'L')] - 'A'] = 1;
      if (checkValid(getXY(x, y, direction, 'L', 0),
                     getXY(x, y, direction, 'L', 1)))
        wall[getXY(x, y, direction, 'L', 0)][getXY(x, y, direction, 'L', 1)]
        [m[inverseDirection(getNewDirection(direction, 'L'))] - 'A'] = 1;
    }
    // forward
    if (digitalRead(frontIr) == 1) {
      mmin = min(mmin, arr[getXY(x, y, direction, 'F', 0)]
                 [getXY(x, y, direction, 'F', 1)]);
    } else {
      wall[x][y][m[getNewDirection(direction, 'F')] - 'A'] = 1;
      if (checkValid(getXY(x, y, direction, 'F', 0),
                     getXY(x, y, direction, 'F', 1)))
        wall[getXY(x, y, direction, 'F', 0)][getXY(x, y, direction, 'F', 1)]
        [m[inverseDirection(getNewDirection(direction, 'F'))] - 'A'] = 1;
    }
    // backward
    /* if (!wallBackward()) {
       mmin = min(
           mmin,
           arr[getXY(x, y, direction, 'B', 0)][getXY(x, y, direction, 'B',
      1)]);
      }*/
    if (arr[x][y] != mmin + 1) {
      arr[x][y] = mmin + 1;
      if (checkCenter(x + 1, y) /*&& !wall[x][y][m['E']]*/) {
        temp.first = x + 1;
        temp.second = y;
        st.push(temp);
      }
      if (checkCenter(x - 1, y) /*&& !wall[x][y][m['W']]*/) {
        temp.first = x - 1;
        temp.second = y;
        st.push(temp);
      }
      if (checkCenter(x, y + 1) /*&& !wall[x][y][m['N']]*/) {
        temp.first = x;
        temp.second = y + 1;
        st.push(temp);
      }
      if (checkCenter(x, y - 1) /*&& !wall[x][y][m['S']]*/) {
        temp.first = x;
        temp.second = y - 1;
        st.push(temp);
      }
      while (!st.isEmpty()) {
        temp = st.pop();
        int xx = temp.first, yy = temp.second;
        int mmmin = 100;
        if (checkValid(xx + 1, yy) && !wall[xx][yy][m['E' - 'A']])
          mmmin = min(mmmin, arr[xx + 1][yy]);
        if (checkValid(xx - 1, yy) && !wall[xx][yy][m['W' - 'A']])
          mmmin = min(mmmin, arr[xx - 1][yy]);
        if (checkValid(xx, yy + 1) && !wall[xx][yy][m['N' - 'A']])
          mmmin = min(mmmin, arr[xx][yy + 1]);
        if (checkValid(xx, yy - 1) && !wall[xx][yy][m['S' - 'A']])
          mmmin = min(mmmin, arr[xx][yy - 1]);
        if (arr[xx][yy] != mmmin + 1) {
          arr[xx][yy] = mmmin + 1;
          if (checkCenter(xx + 1, yy) /*&& !wall[xx][yy][m['E' - 'A']]*/) {
            temp.first = xx + 1;
            temp.second = yy;
            st.push(temp);
          }
          if (checkCenter(xx - 1, yy) /*&& !wall[xx][yy][m['W' - 'A']]*/) {
            temp.first = xx - 1;
            temp.second = yy;
            st.push(temp);
          }
          if (checkCenter(xx, yy + 1) /*&& !wall[xx][yy][m['N' - 'A']]*/) {
            temp.first = xx;
            temp.second = yy + 1;
            st.push(temp);
          }
          if (checkCenter(xx, yy - 1) /*&& !wall[xx][yy][m['S' - 'A']]*/) {
            temp.first = xx;
            temp.second = yy - 1;
            st.push(temp);
          }
        }
      }
    }
    int fmin = 100;
    if (digitalRead(rightIr) == 1) {
      fmin = min(fmin, arr[getXY(x, y, direction, 'R', 0)]
                 [getXY(x, y, direction, 'R', 1)]);
    }
    // left
    if (digitalRead(leftIr) == 1) {
      fmin = min(fmin, arr[getXY(x, y, direction, 'L', 0)]
                 [getXY(x, y, direction, 'L', 1)]);
    }
    // forward
    if (digitalRead(frontIr) == 1) {
      fmin = min(fmin, arr[getXY(x, y, direction, 'F', 0)]
                 [getXY(x, y, direction, 'F', 1)]);
    }
    if (digitalRead(frontIr) == 1 && fmin == arr[getXY(x, y, direction, 'F', 0)]
        [getXY(x, y, direction, 'F', 1)]) {
      go_forward();
      prev = arr[x][y];
      x = getXY(x, y, direction, 'F', 0);
      y = getXY(x, y, direction, 'F', 1);
      direction = getNewDirection(direction, 'F');
      turn = 'F';
      continue;
    }
    if (digitalRead(rightIr) == 1 && fmin == arr[getXY(x, y, direction, 'R', 0)]
        [getXY(x, y, direction, 'R', 1)]) {
      back = 0;
      turn_right(90, mspeed);
      go_forward();
      prev = arr[x][y];
      x = getXY(x, y, direction, 'R', 0);
      y = getXY(x, y, direction, 'R', 1);
      direction = getNewDirection(direction, 'R');
      turn = 'R';
      continue;
    }
    if (digitalRead(leftIr) == 1 && fmin == arr[getXY(x, y, direction, 'L', 0)]
        [getXY(x, y, direction, 'L', 1)]) {
      back = 0;
      turn_left(90, mspeed);
      go_forward();
      prev = arr[x][y];
      x = getXY(x, y, direction, 'L', 0);
      y = getXY(x, y, direction, 'L', 1);
      direction = getNewDirection(direction, 'L');
      turn = 'L';
      continue;
    }
    turn_left(90, mspeed);
    turn_left(90, mspeed);
    go_forward();
    prev = arr[x][y];
    x = getXY(x, y, direction, 'B', 0);
    y = getXY(x, y, direction, 'B', 1);
    direction = getNewDirection(direction, 'B');
    turn = 'B';
    back = 1;
    continue;
  }
}


void forward2() {
  encoder_countL = 0;  //  reset counter A to zero
  encoder_countR = 0;  //  reset counter B to zero
  // Set Motor A forward
  //Serial.println(wallFront());
  if (wallFront() > 6) {
    digitalWrite(in2, HIGH);
    digitalWrite(in1, LOW);
    // Set Motor B forward
    digitalWrite(in4, HIGH);
    digitalWrite(in3, LOW);
  }

  // Go forward until step value is reached
  while (1) {
    myPID();
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
      analogWrite(enL, mspeed);
      analogWrite(enR, mspeed);
    }
    if (wallFront() < 6) {
      break;
    }
    delay(0);
  }
  // Stop when done
  stopAll();
  delay(1000);
}

void myPID() {
  int rightWall = wallRight();
  int leftWall = wallLeft();
  if (digitalRead(rightIr) == 1 && digitalRead(leftIr) == 0) {
    if (leftWall < 12)
      currentError = 1;
    else
      currentError = -1;
    return;
  }  else if (digitalRead(leftIr) == 1 && digitalRead(rightIr) == 0) {
    if (rightWall < 12)
      currentError = -1;
    else
      currentError = 1;
    return;
  } else if (digitalRead(leftIr) == 1 && digitalRead(rightIr) == 1) {
    rightWall = 0, leftWall = 0;
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
  encoder_countL = 0;  //  reset counter A to zero
  encoder_countR = 0;  //  reset counter B to zero
  // Set Motor A forward
  //Serial.println(wallFront());
  myPID();
  int temp = currentError;
  if ((encoder_countR + encoder_countL) / 2   <= 280) {
    digitalWrite(in2, HIGH);
    digitalWrite(in1, LOW);
    // Set Motor B forward
    digitalWrite(in4, HIGH);
    digitalWrite(in3, LOW);
  }
  encoder_countR = encoder_countL = 0;
  // Go forward until step value is reached
  while (digitalRead(frontIr) == 1 && (encoder_countR + encoder_countL) / 2   <= 280) {
    if (temp != 0 )
      myPID();
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
      analogWrite(enL, mspeed);
      analogWrite(enR, mspeed);
    }
    delay(0);
    //}
  }
  stopAll();
  delay(1000);
  // Stop when done
  //delay(1000);
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

int wallFront() {
  return sonarF.ping_cm();
}
