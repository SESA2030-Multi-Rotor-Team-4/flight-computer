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
const int g = 9.81;

// Servo centre positions
const int PITCH_CENTRE = 90;
const int ROLL_CENTRE = 90;
const int YAW_CENTRE = 90;

// Sensitivity (increase for more response, decrease for less)
const float PITCH_GAIN = 1.0;
const float ROLL_GAIN = 1.0;
const float YAW_GAIN = 1.0;

// Smoothing (0.0 - 1.0, higher = smoother but slower response)
const float SMOOTHING = 0;

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
  int pitchPos = constrain(PITCH_CENTRE - (smoothPitch * PITCH_GAIN), 0, 180);
  int rollPos = constrain(ROLL_CENTRE - (smoothRoll * ROLL_GAIN), 0, 180);
  int yawPos = constrain(YAW_CENTRE - (smoothYaw * YAW_GAIN), 0, 180);

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
  Serial.print(">ServoPitch:");
  Serial.println(pitchPos);
  Serial.print(">ServoRoll:");
  Serial.println(rollPos);
  Serial.print(">ServoYaw:");
  Serial.println(yawPos);

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