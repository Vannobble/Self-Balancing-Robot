#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

Quaternion q;
VectorFloat gravity;
float ypr[3];

bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

double input, output;
double setpoint = 0 ;
double Kp = 0;  // Diatur dari serial

// Motor pins
const int enA = 5, in1 = 6, in2 = 7;
const int enB = 10, in3 = 8, in4 = 9;

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  TWBR = 24; // Tingkatkan kecepatan I2C

  // Motor setup
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); pinMode(enA, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT); pinMode(enB, OUTPUT);

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // Kalibrasi sensor
  mpu.setXGyroOffset(30);
  mpu.setYGyroOffset(-50);
  mpu.setZGyroOffset(20);
  mpu.setZAccelOffset(1600);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("MPU6050 siap untuk tuning Ziegler–Nichols!");
    Serial.println("Ketik: SET 10  (untuk set Kp = 10)");
  } else {
    Serial.print("DMP gagal: "); Serial.println(devStatus);
    while (1); // Stop
  }

  inputString.reserve(20);
}

void loop() {
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    if (isnan(ypr[2])) {
      Serial.println("MPU error: ypr[2] NaN. Motor dimatikan.");
      setMotor(0);
      return;
    }

    input = ypr[2] * 180 / M_PI;

    if (abs(input) > 90) {
      Serial.println("Robot jatuh atau data tidak valid. Motor dimatikan.");
      setMotor(0);
      return;
    }

    double error = setpoint - input;
    output = Kp * error;

    setMotor(output);

    // Output ke serial plotter
    Serial.print(input);
    Serial.print(",");
    Serial.println(output);
  }

  // Cek input serial
  if (stringComplete) {
    processSerialCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void setMotor(double pidOutput) {
  int pwm = abs(pidOutput);
  if (pwm > 255) pwm = 255;

  if (pidOutput > 0) {
    // Mundur
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  } else {
    // Maju
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  }

  analogWrite(enA, pwm);
  analogWrite(enB, pwm);
}

// ==================== Serial Input Processing ====================
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void processSerialCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.startsWith("SET")) {
    cmd.remove(0, 3);
    cmd.trim();
    double val = cmd.toFloat();
    Kp = val;
    Serial.print("Kp di-set ke: ");
    Serial.println(Kp);
  } else {
    Serial.println("Perintah tidak dikenali. Gunakan: SET <nilai>");
  }
}
