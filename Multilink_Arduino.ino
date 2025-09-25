#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>

// === Pin Motor Driver TB6612FNG ===
#define AIN1 9
#define AIN2 10
#define PWMA 11
#define BIN1 8
#define BIN2 7
#define PWMB 6

// === Pin Encoder ===
#define ENCODER_LEFT_A 2  // Interrupt 0
#define ENCODER_LEFT_B 4
#define ENCODER_RIGHT_A 3 // Interrupt 1
#define ENCODER_RIGHT_B 5

// === Pin Ultrasonik ===
#define TRIG_LEFT 44
#define ECHO_LEFT 42
#define TRIG_MIDDLE 40
#define ECHO_MIDDLE 38
#define TRIG_RIGHT 36
#define ECHO_RIGHT 34

// === Variabel encoder ===
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;
long prevEncoderLeftCount = 0;
long prevEncoderRightCount = 0;
unsigned long lastEncoderTime = 0;
float leftSpeed = 0;
float rightSpeed = 0;

// === Variabel ultrasonik ===
long duration;
int distanceLeft, distanceMiddle, distanceRight;

// === MPU6050 DMP ===
MPU6050 mpu;
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3]; // yaw, pitch, roll

// Raw sensor
int16_t ax, ay, az;
int16_t gx, gy, gz;

// === Variabel kontrol motor ===
int motorSpeed = 100;
int turnSpeed  = 80;

void setup() {
  Serial.begin(115200);

  // Setup motor driver
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);

  // Setup encoder
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), updateEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), updateEncoderRight, CHANGE);

  // Setup ultrasonik
  pinMode(TRIG_LEFT, OUTPUT); pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_MIDDLE, OUTPUT); pinMode(ECHO_MIDDLE, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  // Inisialisasi MPU6050 DMP
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // (opsional) kalibrasi offset
  mpu.setXAccelOffset(-1684);
  mpu.setYAccelOffset(-1929);
  mpu.setZAccelOffset(1131);
  mpu.setXGyroOffset(43);
  mpu.setYGyroOffset(-29);
  mpu.setZGyroOffset(12);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("MPU6050 DMP ready!");
  } else {
    Serial.print("DMP init failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }

  stopMotors();
  Serial.println("DDMR Ready! Waiting commands from Raspberry Pi...");
}

void loop() {
  // Baca sensor ultrasonik
  distanceLeft   = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  distanceMiddle = readUltrasonic(TRIG_MIDDLE, ECHO_MIDDLE);
  distanceRight  = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  // Hitung kecepatan encoder
  calculateSpeed();

  // Baca IMU raw
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Hitung pitch & roll dari DMP
  float pitch = 0, roll = 0;
  if (dmpReady) {
    fifoCount = mpu.getFIFOCount();
    if (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      pitch = ypr[1] * 180 / M_PI;
      roll  = ypr[2] * 180 / M_PI;
    }
  }

  // Terima perintah dari Raspberry Pi
  readPiCommands();

  // Kirim data sensor ke Raspberry Pi
  sendSensorData(pitch, roll);


}

// === Baca sensor ultrasonik ===
int readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH, 20000);
  if (duration == 0) return 200;
  return duration * 0.034 / 2;
}

// === Interrupt encoder ===
void updateEncoderLeft() {
  if (digitalRead(ENCODER_LEFT_A) == digitalRead(ENCODER_LEFT_B)) encoderLeftCount++;
  else encoderLeftCount--;
}
void updateEncoderRight() {
  if (digitalRead(ENCODER_RIGHT_A) == digitalRead(ENCODER_RIGHT_B)) encoderRightCount++;
  else encoderRightCount--;
}

// === Hitung kecepatan motor ===
void calculateSpeed() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastEncoderTime;

  if (deltaTime >= 100) {
    long deltaLeft  = encoderLeftCount  - prevEncoderLeftCount;
    long deltaRight = encoderRightCount - prevEncoderRightCount;

    leftSpeed  = (deltaLeft  / 20.0) / (deltaTime / 1000.0) * 60.0;
    rightSpeed = (deltaRight / 20.0) / (deltaTime / 1000.0) * 60.0;

    prevEncoderLeftCount  = encoderLeftCount;
    prevEncoderRightCount = encoderRightCount;
    lastEncoderTime = currentTime;
  }
}

// === Kirim data sensor ke Raspberry Pi (CSV sesuai format) ===
void sendSensorData(float pitch, float roll) {
  Serial.print(distanceLeft); Serial.print(",");
  Serial.print(distanceMiddle); Serial.print(",");
  Serial.print(distanceRight); Serial.print(",");
  Serial.print((leftSpeed != 0 || rightSpeed != 0) ? 1 : 0); Serial.print(",");
  Serial.print(motorSpeed); Serial.print(",");
  Serial.print(ax/16384.0, 2); Serial.print(",");
  Serial.print(ay/16384.0, 2); Serial.print(",");
  Serial.print(az/16384.0, 2); Serial.print(",");
  Serial.print(gx/131.0, 2); Serial.print(",");
  Serial.print(gy/131.0, 2); Serial.print(",");
  Serial.print(gz/131.0, 2); Serial.print(",");
  Serial.print(pitch, 2); Serial.print(",");
  Serial.println(roll, 2);
}

// === Terima perintah dari Raspberry Pi ===
void readPiCommands() {
  if (Serial.available() > 0) {
    String incoming = Serial.readStringUntil('\n');
    incoming.trim();

    if (incoming == "up") {
      moveForward();
      Serial.println("Reply: ok move forward");
    } else if (incoming == "down") {
      moveBackward();
      Serial.println("Reply: ok move backward");
    } else if (incoming == "left") {
      turnLeft();
      Serial.println("Reply: ok move left");
    } else if (incoming == "right") {
      turnRight();
      Serial.println("Reply: ok move right");
    } else if (incoming == "stop") {
      stopMotors();
      Serial.println("Reply: ok stopping");
    }
  }
}

// === Kontrol Motor ===
void moveForward() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, motorSpeed);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, motorSpeed);
}
void moveBackward() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, motorSpeed);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, motorSpeed);
}
void turnLeft() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, turnSpeed);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); analogWrite(PWMB, turnSpeed);
}
void turnRight() {
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); analogWrite(PWMA, turnSpeed);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, turnSpeed);
}
void stopMotors() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, 0);
}