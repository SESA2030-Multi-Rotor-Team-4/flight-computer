#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <SoftwareSerial.h>
#include <MPU6050_light.h>
#include <Servo.h>

/*
MPU Wiring
VCC = 3.3v
int = 2
scl = a5
sda = a4

MSP Data Reader (Serial Monitor Version)
WIRING:
  - FC TX  ---> Arduino Pin 2 (Soft RX)
  - FC RX  <--- Arduino Pin 3 (Soft TX) [Use Voltage Divider!]
  - FC GND <--- Arduino GND
*/

// ======== PIN ASSIGNMENT ========
// Create a virtual serial port on Pins 2 and 3
// RX = Pin 2, TX = Pin 3
SoftwareSerial droneSerial(2, 3);
// Ultrasound Sensor pins
constexpr uint8_t TRIG_PIN = 6;
constexpr uint8_t ECHO_PIN = 7;
// SD Card pin
constexpr uint8_t SD_CS = 4;
// Servo pins
constexpr uint8_t PITCH_PIN = 9;
constexpr uint8_t ROLL_PIN = 10;
constexpr uint8_t YAW_PIN = 11;

// ======== MSP ========
// MSP Request for Attitude (Command 108)
static const byte MSP_REQ_ATTITUDE[] PROGMEM = {0x24, 0x4D, 0x3C, 0x00, 0x6C, 0x6C};
constexpr uint8_t MSP_REQ_LEN = 6;
constexpr uint8_t MSP_REPLY_LEN = 12;
constexpr uint32_t MSP_PERIOD_MS = 100;

int16_t mspRoll, mspPitch, mspYaw;
uint32_t lastMspMs = 0;

// ======== MPU ========
MPU6050 mpu(Wire);
constexpr float G_MPS2 = 9.81f; // gravity if we need it

float ax, ay, az;
float gx, gy, gz;
float angx, angy, angz;

// ======== Servo Constants ========
constexpr float PITCH_CENTRE = 90.0f;
constexpr float ROLL_CENTRE = 90.0f;
constexpr float YAW_CENTRE = 90.0f;
constexpr float MAX_STEP = 180.0f;

// ======== PID & Filter Gains ========
// Sensitivity (increase for more response, decrease for less) (P term in PID)
constexpr float PITCH_GAIN = 2.0f;
constexpr float ROLL_GAIN = 2.5f;
constexpr float YAW_GAIN = 1.0f;
// Damping gain — higher = more overshoot suppression (D term in PID)
constexpr float PITCH_DAMP = 0.0f;
constexpr float ROLL_DAMP = 0.0f;
constexpr float YAW_DAMP = 0.0f;
// Smoothing (0.0 - 1.0)
constexpr float SMOOTHING = 0.3f;
constexpr float GYRO_SMOOTHING = 0.15f;
// Deadband - ignore angle changes smaller than this (degrees)
constexpr float DEADBAND = 1.5f;
constexpr float GYRO_DEADBAND = 0.5f;

// ======== Servo State ========
Servo servo_pitch, servo_roll, servo_yaw;
float lastPitchPos, lastRollPos, lastYawPos;
float smoothPitch = 0, smoothRoll = 0, smoothYaw = 0;
float smoothGx = 0, smoothGy = 0, smoothGz = 0;

// ======== MPU Settling ========
bool mpuSettled = false;
constexpr float SETTLE_THRESHOLD = 1.0f;
constexpr uint8_t SETTLE_COUNT = 50; // Wraps above 255, do not increase above 255
uint8_t stableReadings = 0;

// ======== SD Card ========
SdFat sd;
SdFile logFile;
constexpr uint16_t LOG_HZ = 50;
constexpr uint32_t LOG_PERIOD_MS = 1000UL / LOG_HZ;
constexpr uint32_t FLUSH_EVERY_MS = 1000;
uint32_t lastLogMs = 0;
uint32_t lastFlushMs = 0;
bool sdReady = false;

bool mpuSettling()
{
  if (mpuSettled)
    return false;

  if (fabsf(angx) < SETTLE_THRESHOLD &&
      fabsf(angy) < SETTLE_THRESHOLD &&
      fabsf(angz) < SETTLE_THRESHOLD)
  {
    if (++stableReadings >= SETTLE_COUNT)
    {
      mpuSettled = true;
      mpu.calcOffsets();
      smoothPitch = angx;
      smoothRoll = angy;
      smoothYaw = angz;
      lastPitchPos = constrain(PITCH_CENTRE - angx * PITCH_GAIN, 0.0f, 180.0f);
      lastRollPos = constrain(ROLL_CENTRE - angy * ROLL_GAIN, 0.0f, 180.0f);
      lastYawPos = constrain(YAW_CENTRE - angz * YAW_GAIN, 0.0f, 180.0f);
      Serial.println(F("MPU settled"));
      return false;
    }
  }
  else
  {
    stableReadings = 0;
  }
  return true;
}

// ——————————————————————————————————————————
// Helper: apply deadband
// ——————————————————————————————————————————
static inline float deadband(float v, float db)
{
  return (fabsf(v) < db) ? 0.0f : v;
}

// ——————————————————————————————————————————
// Filter + servo position calculation
// ——————————————————————————————————————————
void filterAndActuate()
{
  // EMA smoothing
  smoothPitch += SMOOTHING * (angx - smoothPitch);
  smoothRoll += SMOOTHING * (angy - smoothRoll);
  smoothYaw += SMOOTHING * (angz - smoothYaw);
  smoothGx += GYRO_SMOOTHING * (gx - smoothGx);
  smoothGy += GYRO_SMOOTHING * (gy - smoothGy);
  smoothGz += GYRO_SMOOTHING * (gz - smoothGz);

  const float ep = deadband(smoothPitch, DEADBAND);
  const float er = deadband(smoothRoll, DEADBAND);
  const float ey = deadband(smoothYaw, DEADBAND);
  const float dg = deadband(smoothGx, GYRO_DEADBAND);
  const float dr = deadband(smoothGy, GYRO_DEADBAND);
  const float dy = deadband(smoothGz, GYRO_DEADBAND);

  float pPos = constrain(PITCH_CENTRE - ep * PITCH_GAIN - dg * PITCH_DAMP, 0.0f, 180.0f);
  float rPos = constrain(ROLL_CENTRE - er * ROLL_GAIN - dr * ROLL_DAMP, 0.0f, 180.0f);
  float yPos = constrain(YAW_CENTRE - ey * YAW_GAIN - dy * YAW_DAMP, 0.0f, 180.0f);

  // Rate-limit
  lastPitchPos += constrain(pPos - lastPitchPos, -MAX_STEP, MAX_STEP);
  lastRollPos += constrain(rPos - lastRollPos, -MAX_STEP, MAX_STEP);
  lastYawPos += constrain(yPos - lastYawPos, -MAX_STEP, MAX_STEP);

  servo_pitch.write((int)lastPitchPos);
  servo_roll.write((int)lastRollPos);
  servo_yaw.write((int)lastYawPos);
}

// ======== Ultrasonic State Machine ========
enum UltraState : uint8_t
{
  US_IDLE,
  US_TRIGGER,
  US_WAIT_ECHO
};

UltraState usState = US_IDLE;
uint32_t usTriggerUs = 0;                   // when we sent the trigger
uint32_t usEchoStartUs = 0;                 // when echo pin went HIGH
float lastDistanceM = NAN;                  // most recent valid reading
constexpr uint32_t US_TIMEOUT_US = 15000UL; // same 15 ms max

void ultrasonicUpdate()
{
  switch (usState)
  {
  case US_IDLE:
    // Nothing to do — call ultrasonicStart() to begin a measurement
    break;

  case US_TRIGGER:
    // We set TRIG high for 10 µs, then go low and move on
    // (This part is so fast we can do it in one pass)
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    usTriggerUs = micros();
    usState = US_WAIT_ECHO;
    break;

  case US_WAIT_ECHO:
  {
    uint32_t now_us = micros();
    bool echoHigh = digitalRead(ECHO_PIN);

    if (usEchoStartUs == 0)
    {
      // Waiting for echo to go HIGH (pulse start)
      if (echoHigh)
      {
        usEchoStartUs = now_us;
      }
      else if (now_us - usTriggerUs > US_TIMEOUT_US)
      {
        // Echo never started — no object in range
        lastDistanceM = NAN;
        usState = US_IDLE;
      }
    }
    else
    {
      // Echo is running — waiting for it to go LOW (pulse end)
      if (!echoHigh)
      {
        uint32_t duration = now_us - usEchoStartUs;
        lastDistanceM = duration * 0.0001715f;
        usEchoStartUs = 0;
        usState = US_IDLE;
      }
      else if (now_us - usEchoStartUs > US_TIMEOUT_US)
      {
        // Echo stuck HIGH — out of range
        lastDistanceM = NAN;
        usEchoStartUs = 0;
        usState = US_IDLE;
      }
    }
    break;
  }
  }
}

// Call this to kick off a new measurement
void ultrasonicStart()
{
  if (usState == US_IDLE)
  {
    usState = US_TRIGGER;
    usEchoStartUs = 0;
  }
}

// ======== MSP State Machine ========
enum MspState : uint8_t
{
  MSP_IDLE,
  MSP_SENT,
  MSP_PARSE
};

MspState mspState = MSP_IDLE;
uint32_t mspSentMs = 0;
uint8_t mspBuf[MSP_REPLY_LEN]; // 12-byte reply buffer
uint8_t mspBufIdx = 0;
constexpr uint32_t MSP_TIMEOUT_MS = 50; // tighter than 100 ms

void mspUpdate()
{
  switch (mspState)
  {
  case MSP_IDLE:
    break;

  case MSP_SENT:
  {
    // Collect bytes as they arrive — no waiting
    while (droneSerial.available() && mspBufIdx < MSP_REPLY_LEN)
    {
      mspBuf[mspBufIdx++] = droneSerial.read();
    }

    if (mspBufIdx >= MSP_REPLY_LEN)
    {
      // All bytes received — move to parse
      mspState = MSP_PARSE;
    }
    else if (millis() - mspSentMs > MSP_TIMEOUT_MS)
    {
      // Timeout — discard partial data
      Serial.println(F("MSP timeout"));
      mspState = MSP_IDLE;
    }
    break;
  }

  case MSP_PARSE:
  {
    // Validate header: $, M, >
    if (mspBuf[0] != '$' || mspBuf[1] != 'M' || mspBuf[2] != '>')
    {
      Serial.println(F("MSP bad header"));
      mspState = MSP_IDLE;
      break;
    }

    // Bytes 3 = payload size, 4 = command
    // Bytes 5..10 = roll(2), pitch(2), yaw(2)
    // Byte 11 = checksum

    mspRoll = (int16_t)((uint16_t)mspBuf[6] << 8 | mspBuf[5]);
    mspPitch = (int16_t)((uint16_t)mspBuf[8] << 8 | mspBuf[7]);
    mspYaw = (int16_t)((uint16_t)mspBuf[10] << 8 | mspBuf[9]);

    // Optional: verify checksum
    // XOR of bytes 3..10 should equal byte 11
    uint8_t crc = 0;
    for (uint8_t i = 3; i < 11; i++)
      crc ^= mspBuf[i];

    if (crc != mspBuf[11])
    {
      Serial.println(F("MSP CRC fail"));
    }

    mspState = MSP_IDLE;
    break;
  }
  }
}

void mspStart()
{
  if (mspState != MSP_IDLE)
    return;

  // Flush stale data
  while (droneSerial.available())
    droneSerial.read();

  // Send request
  for (uint8_t i = 0; i < MSP_REQ_LEN; i++)
    droneSerial.write(pgm_read_byte(&MSP_REQ_ATTITUDE[i]));

  mspBufIdx = 0;
  mspSentMs = millis();
  mspState = MSP_SENT;
}

// ——————————————————————————————————————————
// Setup
// ——————————————————————————————————————————
void setup()
{
  // Open connection to Drone (Virtual Port)
  // Note: SoftwareSerial at 115200 can be a bit unstable.
  // If this fails, try lowering FC baud rate to 57600 in Cleanflight.
  droneSerial.begin(115200);

  // Assigning pins for ultrasound sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Wire.begin();
  Wire.setClock(400000UL); // Fast-mode I²C — MPU6050 supports it

  Serial.begin(115200);
  Serial.println(F("Arduino Ready. Connecting to Drone..."));

  // ======== MPU Setup ========
  Serial.println(F("Init MPU"));
  if (mpu.begin() != 0)
  {
    Serial.println(F("MPU fail"));
    while (1)
      ;
  }
  mpu.calcOffsets();

  // ======== Servo setup ========

  // Attaching servos to pins
  servo_pitch.attach(PITCH_PIN);
  servo_roll.attach(ROLL_PIN);
  servo_yaw.attach(YAW_PIN);

  lastPitchPos = PITCH_CENTRE;
  lastRollPos = ROLL_CENTRE;
  lastYawPos = YAW_CENTRE;

  // Center servos on startup
  servo_pitch.write(PITCH_CENTRE);
  servo_roll.write(ROLL_CENTRE);
  servo_yaw.write(YAW_CENTRE);
  delay(1000);

  // ======== SD Card setup ========
  if (sd.begin(SD_CS))
  {
    logFile.open("log.csv", O_WRITE | O_CREAT | O_AT_END);
    sdReady = logFile.isOpen();
  }
  if (!sdReady)
    Serial.println(F("SD failed — logging disabled"));

  if (sdReady && logFile.fileSize() == 0)
  {
    logFile.println(F("t_ms,distance_m,ax_mps2,ay_mps2,az_mps2,gx_rps,gy_rps,gz_rps"));
    logFile.flush();
  }

  lastLogMs = millis();
  lastFlushMs = lastLogMs;

  delay(1000);
}

void loop()
{
  uint32_t now = millis();
  mpu.update();
  angx = mpu.getAngleX();
  angy = mpu.getAngleY();
  angz = mpu.getAngleZ();
  ax = mpu.getAccX() * G_MPS2;
  ay = mpu.getAccY() * G_MPS2;
  az = mpu.getAccZ() * G_MPS2;
  gx = mpu.getGyroX();
  gy = mpu.getGyroY();
  gz = mpu.getGyroZ();

  if (mpuSettling())
    return; // No data gathered until settled

  filterAndActuate(); // Filters all MPU readings and writes to servos

  // ---- Logging (rate-limited) ----
  if (now - lastLogMs >= LOG_PERIOD_MS)
  {
    lastLogMs += LOG_PERIOD_MS;

    float d = lastDistanceM;

    ultrasonicStart();

    // Write to both Serial and SD without snprintf (saves ~128 bytes RAM + ~1.5 KB flash)
    Print *out[2] = {&Serial, nullptr};
    uint8_t outCount = 1;
    if (sdReady)
    {
      out[1] = &logFile;
      outCount = 2;
    }
    for (uint8_t i = 0; i < outCount; i++)
    {
      out[i]->print(now);
      out[i]->print(',');
      if (isnan(d))
        out[i]->print(F("NaN"));
      else
        out[i]->print(d, 4);
      out[i]->print(',');
      out[i]->print(ax, 3);
      out[i]->print(',');
      out[i]->print(ay, 3);
      out[i]->print(',');
      out[i]->print(az, 3);
      out[i]->print(',');
      out[i]->print(gx, 3);
      out[i]->print(',');
      out[i]->print(gy, 3);
      out[i]->print(',');
      out[i]->println(gz, 3);
    }
  }

  if (sdReady && now - lastFlushMs >= FLUSH_EVERY_MS)
  {
    lastFlushMs = now;
    logFile.flush();
  }
  ultrasonicUpdate();

  // ---- MSP from FC (rate-limited) ----
  if (now - lastMspMs >= MSP_PERIOD_MS)
  {
    lastMspMs += MSP_PERIOD_MS; // use += to prevent drift
    mspStart();
  }
  mspUpdate();

  // ---- Teleplot (rate-limited to avoid flooding serial) ----
  // Only output teleplot every ~100 ms (10 Hz) instead of every loop
  static uint32_t lastTeleMs = 0;
  if (now - lastTeleMs >= 100)
  {
    lastTeleMs = now;
    Serial.print(F(">Pitch:"));
    Serial.println(angx, 1);
    Serial.print(F(">Roll:"));
    Serial.println(angy, 1);
    Serial.print(F(">Yaw:"));
    Serial.println(angz, 1);
    Serial.print(F(">SP:"));
    Serial.println(-(lastPitchPos - PITCH_CENTRE), 1);
    Serial.print(F(">SR:"));
    Serial.println(-(lastRollPos - ROLL_CENTRE), 1);
    Serial.print(F(">SY:"));
    Serial.println(-(lastYawPos - YAW_CENTRE), 1);
  }
}