#include <Arduino.h>
/*
VCC = 3.3v
int = 2
scl = a5
sda = a4
*/

#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>

MPU6050 mpu(Wire);

float ax, ay, az;
float gx, gy, gz;
float angx, angy, angz;
float temp;
const float g = 9.81;
const float MAX_STEP = 2.0; // Max degrees per loop iteration

// Servo centre positions
const float PITCH_CENTRE = 90.0;
const float ROLL_CENTRE = 90.0;
const float YAW_CENTRE = 90.0;

float lastPitchPos = PITCH_CENTRE;
float lastRollPos = ROLL_CENTRE;
float lastYawPos = YAW_CENTRE;

// Sensitivity (increase for more response, decrease for less)
const float PITCH_GAIN = 3.0;
const float ROLL_GAIN = 2.0;
const float YAW_GAIN = 4.0;

// Smoothing (0.0 - 1.0)
const float SMOOTHING = 0.15;

float smoothPitch = 0;
float smoothRoll = 0;
float smoothYaw = 0;

int val;
int prevVal;

int valax;
int valay;
int valaz;

Servo servo_pitch;
Servo servo_roll;
Servo servo_yaw;

void sweepServo(Servo &s) {
  int positions[] = {0, 90, 180, 90};
  for (int pos : positions) {
    s.write(pos);
    delay(1000);
  }
}

void setup()
{
  Wire.begin();
  Serial.begin(38400);
  Serial.println("Initialize MPU");
  byte status = mpu.begin();
  Serial.println(status == 0 ? "Connected" : "Connection failed");
  mpu.calcOffsets();

  // Attaching servos to pins
  servo_pitch.attach(9);
  servo_roll.attach(10);
  servo_yaw.attach(11);

  // Center servos on startup
  servo_pitch.write(PITCH_CENTRE);
  servo_roll.write(ROLL_CENTRE);
  servo_yaw.write(YAW_CENTRE);

  delay(500);
}

void loop()
{
  mpu.update( );
  angx = mpu.getAngleX();
  angy = mpu.getAngleY();
  angz = mpu.getAngleZ();
  ax = mpu.getAccX() * g;
  ay = mpu.getAccY() * g;
  az = mpu.getAccZ() * g;
  gx = mpu.getGyroX();
  gy = mpu.getGyroY();
  gz = mpu.getGyroZ();

  // Smoothing MPU outputs
  smoothPitch = smoothPitch * (1- SMOOTHING) + angx * SMOOTHING;
  smoothRoll = smoothRoll * (1- SMOOTHING) + angy * SMOOTHING;
  smoothYaw = smoothYaw * (1- SMOOTHING) + angz * SMOOTHING;

  // Calculating servo positions
  float pitchPos = constrain(PITCH_CENTRE - (smoothPitch * PITCH_GAIN), 0, 180);
  float rollPos = constrain(ROLL_CENTRE - (smoothRoll * ROLL_GAIN), 0, 180);
  float yawPos = constrain(YAW_CENTRE - (smoothYaw * YAW_GAIN), 0, 180);

  pitchPos = lastPitchPos + constrain(pitchPos - lastPitchPos, -MAX_STEP, MAX_STEP);
  rollPos = lastRollPos + constrain(rollPos - lastRollPos, -MAX_STEP, MAX_STEP);
  yawPos = lastYawPos + constrain(yawPos - lastYawPos, -MAX_STEP, MAX_STEP);

  lastPitchPos = pitchPos;
  lastRollPos = rollPos;
  lastYawPos = yawPos;

  // Writing to servos
  servo_pitch.write(pitchPos);
  servo_roll.write(rollPos);
  servo_yaw.write(yawPos);


  // ## Teleplot output ##

  // MPU raw reads
  
  Serial.print(">Pitch:");
  Serial.println(angx);
  Serial.print(">Roll:");
  Serial.println(angy);
  Serial.print(">Yaw:");
  Serial.println(angz);
  

  // Servo writes
  Serial.print(">ServoPitch(9):");
  Serial.println(pitchPos);
  Serial.print(">ServoRoll(10):");
  Serial.println(rollPos);
  Serial.print(">ServoYaw(11):");
  Serial.println(yawPos);
  // Servo writes
  /*
  Serial.print(">SmoothPitch:");
  Serial.println(smoothPitch);
  Serial.print(">SmoothRoll:");
  Serial.println(smoothRoll);
  Serial.print(">SmoothYaw:");
  Serial.println(smoothYaw);
  */

  /*
  Serial.print(">AccX:"); // Extra spaces to clear any leftover characters
  Serial.println(ax);
  Serial.print(">AccY:"); // Extra spaces to clear any leftover characters
  Serial.println(ay);
  Serial.print(">AccZ:"); // Extra spaces to clear any leftover characters
  Serial.println(az);
  Serial.print(">GyroX:"); // Extra spaces to clear any leftover characters
  Serial.println(gx);
  Serial.print(">GyroY:"); // Extra spaces to clear any leftover characters
  Serial.println(gy);
  Serial.print(">GyroZ:"); // Extra spaces to clear any leftover characters
  Serial.println(gz); 
  */

  delay(10);
}