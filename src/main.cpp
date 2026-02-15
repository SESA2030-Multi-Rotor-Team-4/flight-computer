#include <Arduino.h>
#include <avr/wdt.h>
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

uint8_t resetReason __attribute__((section(".noinit")));

void getResetReason() {
  resetReason = MCUSR;  // Save reset flags
  MCUSR = 0;            // Clear so bootloader doesn't get confused
  wdt_disable();        // Disable WDT immediately in case it caused the reset
}

float ax, ay, az;
float gx, gy, gz;
float angx, angy, angz;
float temp;
const float g = 9.81;
const float MAX_STEP = 180.0; // Max degrees per loop iteration

// Servo centre positions
const float PITCH_CENTRE = 90.0;
const float ROLL_CENTRE = 90.0;
const float YAW_CENTRE = 90.0;

float lastPitchPos = PITCH_CENTRE;
float lastRollPos = ROLL_CENTRE;
float lastYawPos = YAW_CENTRE;

// Sensitivity (increase for more response, decrease for less) (P term in PID)
const float PITCH_GAIN = 2.0;
const float ROLL_GAIN = 2.0;
const float YAW_GAIN = 2.0;

// Damping gain — higher = more overshoot suppression (D term in PID)
const float PITCH_DAMP = 0.5;
const float ROLL_DAMP = 0.5;
const float YAW_DAMP = 0.5;

// Smoothing (0.0 - 1.0)
const float SMOOTHING = 0.25;

// Deadband - ignore angle changes smaller than this (degrees)
const float DEADBAND = 1.5;
const float GYRO_DEADBAND = 0.5; // degrees/sec — ignore gyro noise below this


float smoothPitch = 0;
float smoothRoll = 0;
float smoothYaw = 0;

float smoothGx = 0, smoothGy = 0, smoothGz = 0;
const float GYRO_SMOOTHING = 0.15; // Filter gyro noise

int val;
int prevVal;

int valax;
int valay;
int valaz;

Servo servo_pitch;
Servo servo_roll;
Servo servo_yaw;

bool mpuSettled = false;
const float SETTLE_THRESHOLD = 1; // Degrees - readings must be within this range of 0
const int SETTLE_COUNT = 50;        // Number of consecutive stable readings required
int stableReadings = 0;

void setup()
{
  getResetReason();  // Must be called FIRST in setup
  Wire.begin();
  Serial.begin(38400);

  if (resetReason & (1 << WDRF)) {
    Serial.println("*** WARNING: WATCHDOG RESET - Arduino crashed! ***");
  } else if (resetReason & (1 << BORF)) {
    Serial.println("*** WARNING: Brown-out reset - check power supply ***");
  } else if (resetReason & (1 << EXTRF)) {
    Serial.println("Normal external reset (button press)");
  } else if (resetReason & (1 << PORF)) {
    Serial.println("Normal power-on reset");

    wdt_enable(WDTO_2S);
  }

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
  delay(1000);
  servo_roll.write(ROLL_CENTRE);
  delay(1000);
  servo_yaw.write(YAW_CENTRE);
  delay(1000);
  if (servo_pitch.read() == 90) {
    Serial.println("Pitch axis centred");
  }
  if (servo_roll.read() == 90) {
    Serial.println("Roll axis centred");
  }
  if (servo_roll.read() == 90) {
    Serial.println("Yaw axis centred");
  }

  delay(1000);
}

void loop()
{
  wdt_reset();  // "Pet" the watchdog — must be called every loop iteration

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

  if (!mpuSettled) {
    if (abs(angx) < SETTLE_THRESHOLD && abs(angy) < SETTLE_THRESHOLD && abs(angz) < SETTLE_THRESHOLD) {
      stableReadings++;
      if (stableReadings >= SETTLE_COUNT) {
        mpuSettled = true;
        Serial.println("MPU settled - servos enabled");
      }
    } else {
      stableReadings = 0; // Reset counter if readings are unstable
    }
    Serial.print(">Settling:");
    Serial.println(stableReadings);
    delay(10);
    return; // Skips the rest of the loop until settled
  }

  // Smoothing MPU outputs
  smoothPitch = smoothPitch * (1- SMOOTHING) + angx * SMOOTHING;
  smoothRoll = smoothRoll * (1- SMOOTHING) + angy * SMOOTHING;
  smoothYaw = smoothYaw * (1- SMOOTHING) + angz * SMOOTHING;

  smoothGx = smoothGx * (1.0 - GYRO_SMOOTHING) + gx * GYRO_SMOOTHING;
  smoothGy = smoothGy * (1.0 - GYRO_SMOOTHING) + gy * GYRO_SMOOTHING;
  smoothGz = smoothGz * (1.0 - GYRO_SMOOTHING) + gz * GYRO_SMOOTHING;

    // Apply deadband - treat small angles as zero
  float effectivePitch = (abs(smoothPitch) < DEADBAND) ? 0 : smoothPitch;
  float effectiveRoll  = (abs(smoothRoll)  < DEADBAND) ? 0 : smoothRoll;
  float effectiveYaw   = (abs(smoothYaw)   < DEADBAND) ? 0 : smoothYaw;

  float dampGx = (abs(smoothGx) < GYRO_DEADBAND) ? 0 : smoothGx;
  float dampGy = (abs(smoothGy) < GYRO_DEADBAND) ? 0 : smoothGy;
  float dampGz = (abs(smoothGz) < GYRO_DEADBAND) ? 0 : smoothGz;

  // Calculating servo positions
  float pitchPos = constrain(PITCH_CENTRE - (effectivePitch * PITCH_GAIN) + (dampGx * PITCH_DAMP), 0, 180);
  float rollPos  = constrain(ROLL_CENTRE  - (effectiveRoll  * ROLL_GAIN)  + (dampGy * ROLL_DAMP),  0, 180);
  float yawPos   = constrain(YAW_CENTRE   - (effectiveYaw   * YAW_GAIN)  + (dampGz * YAW_DAMP),   0, 180);

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
  Serial.println(-(pitchPos - PITCH_CENTRE));
  Serial.print(">ServoRoll(10):");
  Serial.println(-(rollPos - ROLL_CENTRE));
  Serial.print(">ServoYaw(11):");
  Serial.println(-(yawPos - YAW_CENTRE));
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

  delay(1);
}