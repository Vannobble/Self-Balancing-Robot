#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define MIN_ABS_SPEED 30

MPU6050 mpu;
Quaternion q;
VectorFloat gravity;
float ypr[3];

bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

// PID parameters
double Kp = 30, Ki = 150, Kd = 1.5;
double setpoint = 0;
double input, output;

// PID internal variables
double error, lastInput = 0;
double ITerm = 0;
unsigned long lastTime = 0;

// PID config
unsigned long sampleTime = 10; // ms
double outMin = -255, outMax = 255;
bool inAuto = true;
int controllerDirection = 1; // 1 = DIRECT

// Motor pins
const int enA = 5, in1 = 6, in2 = 7;
const int enB = 10, in3 = 8, in4 = 9;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  TWBR = 24;  // I2C speed tweak

  // Motor setup
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); pinMode(enA, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT); pinMode(enB, OUTPUT);

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // Gyro offsets (customize for best stability)
  mpu.setXGyroOffset(30);
  mpu.setYGyroOffset(-50);
  mpu.setZGyroOffset(20);
  mpu.setZAccelOffset(1600);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("MPU6050 ready!");
    initializePID();
    lastTime = millis();
  } else {
    Serial.print("DMP Initialization failed: ");
    Serial.println(devStatus);
    while (1);
  }
}

void loop() {
  if (!dmpReady) return;

  // Cek FIFO overflow, reset FIFO jika penuh
  if (mpu.getFIFOCount() == 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO overflow! Resetting FIFO.");
    return;  // skip loop kali ini supaya tidak baca data corrupt
  }

  // Ambil data sensor hanya jika ada paket baru
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    input = ypr[2] * 180 / M_PI; // roll dalam derajat

    // Reset PID jika robot jatuh (roll > 45 derajat)
    if (abs(input) > 45) {
      ITerm = 0;
      output = 0;
      lastInput = input;
      analogWrite(enA, 0);
      analogWrite(enB, 0);
      Serial.println("Robot jatuh, reset PID output.");
      return; // skip PID dan motor control supaya motor berhenti
    }

    computePID();
    setMotor(output);

    Serial.print("Roll: "); Serial.print(input);
    Serial.print("\tOutput: "); Serial.println(output);
  }
}

void computePID() {
  if (!inAuto) return;

  unsigned long now = millis();
  unsigned long timeChange = now - lastTime;
  if (timeChange >= sampleTime) {
    error = setpoint - input;

    // Integral term
    ITerm += (Ki * error * timeChange / 1000.0);
    ITerm = constrain(ITerm, outMin, outMax);

    // Derivative on measurement
    double dInput = (input - lastInput) / (timeChange / 1000.0);

    double result = (Kp * error) + ITerm - (Kd * dInput);
    result *= controllerDirection;
    output = constrain(result, outMin, outMax);

    lastInput = input;
    lastTime = now;
  }
}

void setMotor(double pidOutput) {
  int pwm = abs(pidOutput);

  if (pwm < MIN_ABS_SPEED) pwm = 0;
  if (pwm > 255) pwm = 255;

  if (pidOutput > 0) {
    // Move backward
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  } else {
    // Move forward
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  }

  analogWrite(enA, pwm);
  analogWrite(enB, pwm);
}

// Optional helper functions
void initializePID() {
  lastInput = input;
  ITerm = output;
  ITerm = constrain(ITerm, outMin, outMax);
}
