#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <SparkFun_TB6612.h>

#define AIN1 9
#define BIN1 11
#define AIN2 10
#define BIN2 12
#define PWMA 5
#define PWMB 6
#define STBY A2

const int offsetA = -1;
const int offsetB = -1;

Motor rightMotor = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor leftMotor = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

unsigned long lastMs = 0;
unsigned long currentMs = 0;
const int TICKS_PER_REVOLUTION = 515;
const double RADIUS = 33.5;
const double WHEELBASE = 177;

double speed_req = 0;
double angular_speed_req = 0; // Desired angular speed for the robot, in rad/s

double speed_req_left = 0;
double speed_act_left = 0;
double speed_adj_left = 0;

double speed_req_right = 0;
double speed_act_right = 0;
double speed_adj_right = 0;

const double MAX_SPEED = 330; // Max speed in m/s
const double MIN_SPEED = 20;
const double PWM_TO_SPEED_RATIO = 0.4;

int PWM_leftMotor = 0;
int PWM_rightMotor = 0;

double kp = 0.7, ki = 0, kd = 0.03;

PID PID_leftMotor(&speed_act_left, &speed_adj_left, &speed_req_left, kp, ki, kd, DIRECT);
PID PID_rightMotor(&speed_act_right, &speed_adj_right, &speed_req_right, kp, ki, kd, DIRECT);

volatile float encoder_left_count = 0;
volatile float encoder_right_count = 0;

const int PIN_LEFT_ENCODER = 2;
const int PIN_RIGHT_ENCODER = 3;

#define LOOPTIME 100
#define NO_COMM_LOOPS_MAX 10;
unsigned int noCommLoops = 0;

void receiveEvent(int howMany) {
  while (Wire.available()) {
    char c = Wire.read();
  }
}

void encoderLeftMotor() { encoder_left_count++; }

void encoderRightMotor() { encoder_right_count++; }

void onTwistCommand(float linear_mm_s, float angular_r_s) {
  noCommLoops = 0;
  speed_req_left = linear_mm_s - angular_r_s * (WHEELBASE / 2);
  speed_req_right = linear_mm_s + angular_r_s * (WHEELBASE / 2);
}

void setup() {
  PID_leftMotor.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
  PID_rightMotor.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);

  leftMotor.standby();
  rightMotor.standby();

  pinMode(PIN_LEFT_ENCODER, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENCODER, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENCODER), encoderLeftMotor, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENCODER), encoderRightMotor, FALLING);

  Wire.begin(4);
  Wire.onReceive(receiveEvent);

  Serial.begin(9600);
  Serial.println("start...");
}

template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

void processMessage() {
  while (Serial.available() > 0) {

    char c = Serial.peek();
    // ignore newlines
    if (c == 'S') {
      Serial.read();
      float linear = Serial.parseFloat();
      float angular = Serial.parseFloat();

      Serial.println(F("OK"));
      onTwistCommand(linear, angular);
    } else {
      Serial.read();
    }
  }
}

unsigned long loopDuration = 0;

void loop() {
  if (Serial.available() > 0) {
    processMessage();
  }

  currentMs = millis();
  loopDuration = currentMs - lastMs;

  if (loopDuration >= LOOPTIME) {
    lastMs = currentMs;

    PID_leftMotor.SetSampleTime(loopDuration);
    PID_rightMotor.SetSampleTime(loopDuration);

    if (encoder_left_count < 5) {
      speed_act_left = 0;
    } else {
      speed_act_left = ((encoder_left_count / TICKS_PER_REVOLUTION) * 2 * PI) * (1000 / loopDuration) * RADIUS;
      if (speed_req_left < 0) {
        speed_act_left = -speed_act_left;
      }
    }

    if (encoder_right_count < 5) {
      speed_act_right = 0;
    } else {
      speed_act_right = ((encoder_right_count / TICKS_PER_REVOLUTION) * 2 * PI) * (1000 / loopDuration) * RADIUS;
      if (speed_req_right < 0) {
        speed_act_right = -speed_act_right;
      }
    }

    encoder_left_count = 0;
    encoder_right_count = 0;

    PID_leftMotor.Compute();
    PID_rightMotor.Compute();

    PWM_leftMotor = constrain(((speed_req_left + sgn(speed_req_left) * MIN_SPEED) * PWM_TO_SPEED_RATIO) + (speed_adj_left * PWM_TO_SPEED_RATIO), -255, 255);
    PWM_rightMotor = constrain(((speed_req_right + sgn(speed_req_right) * MIN_SPEED) * PWM_TO_SPEED_RATIO) + (speed_adj_right * PWM_TO_SPEED_RATIO), -255, 255);

    if (speed_req_left == 0) {
      leftMotor.brake();
      leftMotor.standby();
    } else {
      leftMotor.drive(PWM_leftMotor);
    }

    if (speed_req_right == 0) {
      rightMotor.brake();
      rightMotor.standby();
    } else {
      rightMotor.drive(PWM_rightMotor);
    }

    Serial.print(speed_act_left, 4);
    Serial.print(F(","));
    Serial.print(speed_act_right, 4);
    Serial.print(F(","));
    Serial.print(speed_adj_left, 4);
    Serial.print(F(","));
    Serial.print(speed_adj_right, 4);
    Serial.print(F(","));
    Serial.print(PWM_leftMotor);
    Serial.print(F(","));
    Serial.println(PWM_rightMotor);

    noCommLoops++;
    if (noCommLoops == 65535) {
      noCommLoops = NO_COMM_LOOPS_MAX;
    }
  }
}