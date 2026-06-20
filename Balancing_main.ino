#include <Wire.h>
#include <MPU6050.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
MPU6050 mpu;

// Motor Pins
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33
#define ENA 14
#define ENB 32

// Gamepad Controls
#define FORWARD 'F'
#define BACKWARD 'B'
#define LEFT 'L'
#define RIGHT 'R'
#define CROSS 'X'

// Kalman Variables
float angle = 0, bias = 0;
float P[2][2] = {{1,0},{0,1}};
float Q_angle = 0.001;
float Q_bias = 0.003;
float R_measure = 0.03;

// PID
float Kp = 12;
float Ki = 95;
float Kd = 0.6;

float setpoint = 0;
float error, lastError = 0;
float integral = 0;
float output = 0;

// Time
unsigned long lastTime;
float dt;

// Control offsets
float moveOffset = 0;
float turnOffset = 0;

// 🔥 NEW: Command timing
unsigned long lastCommandTime = 0;
unsigned long commandTimeout = 200;

// MPU raw
int16_t ax, ay, az, gx, gy, gz;

// Kalman Filter
float kalmanUpdate(float newAngle, float newRate, float dt) {
  float rate = newRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float S = P[0][0] + R_measure;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  float y = newAngle - angle;
  angle += K[0] * y;
  bias += K[1] * y;

  float P00 = P[0][0], P01 = P[0][1];

  P[0][0] -= K[0] * P00;
  P[0][1] -= K[0] * P01;
  P[1][0] -= K[1] * P00;
  P[1][1] -= K[1] * P01;

  return angle;
}

// Motor Control
void setMotor(int leftSpeed, int rightSpeed) {

  if (leftSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    leftSpeed = -leftSpeed;
  }

  if (rightSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    rightSpeed = -rightSpeed;
  }

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  ledcWrite(ENA, leftSpeed);
  ledcWrite(ENB, rightSpeed);
}

// 🔥 UPDATED Bluetooth Control
void handleBluetooth() {

  if (SerialBT.available()) {
    char cmd = SerialBT.read();

    lastCommandTime = millis(); // update on every input

    switch (cmd) {
      case FORWARD:
        moveOffset = -3;
        break;
      case BACKWARD:
        moveOffset = 3;
        break;
      case LEFT:
        turnOffset = -50;
        break;
      case RIGHT:
        turnOffset = 50;
        break;
      case CROSS:
        moveOffset = 0;
        turnOffset = 0;
        break;
    }
  }

  // 🔥 AUTO STOP when button released
  if (millis() - lastCommandTime > commandTimeout) {
    moveOffset = 0;
    turnOffset = 0;
  }
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_BALANCER");

  Wire.begin();
  mpu.initialize();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, 1000, 8);
  ledcAttach(ENB, 1000, 8);

  lastTime = millis();
}

void loop() {

  handleBluetooth();

  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accAngle = atan2(ay, az) * 180 / PI;
  float gyroRate = gx / 131.0;

  float filteredAngle = kalmanUpdate(accAngle, gyroRate, dt);

  float target = setpoint + moveOffset;

  error = target - filteredAngle;
  integral += error * dt;
  float derivative = (error - lastError) / dt;

  output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  int leftMotor = output + turnOffset;
  int rightMotor = output - turnOffset;

  setMotor(leftMotor, rightMotor);

  Serial.print("Angle: ");
  Serial.print(filteredAngle);
  Serial.print(" | Output: ");
  Serial.print(output);
  Serial.print(" | MoveOffset: ");
  Serial.print(moveOffset);
  Serial.print(" | TurnOffset: ");
  Serial.println(turnOffset);

  delay(5);
}
