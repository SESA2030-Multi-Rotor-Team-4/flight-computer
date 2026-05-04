#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <MPU6050_light.h>
#include <Servo.h>

/*
This sketch is split into small modules for sensing, filtering, servo control, 
mode handling, and logging.
*/

// Pin assignments for the hardware connections.
// SD Card pin
constexpr uint8_t SD_CS = 4;
// Mode control pin from RC/receiver PWM input
constexpr uint8_t MODE_PIN = 2;
// Servo pins
constexpr uint8_t PITCH_PIN = 6;
constexpr uint8_t ROLL_PIN = 10;
constexpr uint8_t YAW_PIN = 9;

// IMU state and raw sensor values.
MPU6050 mpu(Wire);
constexpr float G_MPS2 = 9.81f; // gravity if we need it

float rawAx, rawAy, rawAz;
float rawGx, rawGy, rawGz;
float rawAngx, rawAngy, rawAngz;

float ax, ay, az;
float gx, gy, gz;
float angx, angy, angz;

// Filter settings and shared low-pass filters for each sensor axis.
constexpr float MPU_FILTER_Q = 0.7071f;
constexpr float ANGLE_FILTER_CUTOFF_HZ = 12.0f;
constexpr float GYRO_FILTER_CUTOFF_HZ = 18.0f;
constexpr float ACCEL_FILTER_CUTOFF_HZ = 15.0f;

constexpr uint32_t MODE_PULSE_TIMEOUT_US = 25000UL;
constexpr uint16_t MODE_STABILISING_THRESHOLD_US = 1500;

enum class GimbalMode : uint8_t
{
  Locked,
  Stabilising
};

// Simple biquad low-pass filter used by the sensor processing module.
struct BiquadLowPass
{
  float b0 = 0.0f;
  float b1 = 0.0f;
  float b2 = 0.0f;
  float a1 = 0.0f;
  float a2 = 0.0f;
  float x1 = 0.0f;
  float x2 = 0.0f;
  float y1 = 0.0f;
  float y2 = 0.0f;
  float cutoffHz = 0.0f;
  float q = MPU_FILTER_Q;
  float sampleRateHz = 200.0f;
  bool configured = false;

  void configure(float newCutoffHz, float newSampleRateHz, float newQ)
  {
    if (newCutoffHz <= 0.0f || newSampleRateHz <= 0.0f || newQ <= 0.0f)
      return;

    cutoffHz = newCutoffHz;
    sampleRateHz = newSampleRateHz;
    q = newQ;

    const float w0 = 2.0f * PI * cutoffHz / sampleRateHz;
    const float cosW0 = cosf(w0);
    const float sinW0 = sinf(w0);
    const float alpha = sinW0 / (2.0f * q);
    const float a0 = 1.0f + alpha;

    b0 = ((1.0f - cosW0) * 0.5f) / a0;
    b1 = (1.0f - cosW0) / a0;
    b2 = ((1.0f - cosW0) * 0.5f) / a0;
    a1 = (-2.0f * cosW0) / a0;
    a2 = (1.0f - alpha) / a0;
    configured = true;
  }

  void reset(float value)
  {
    x1 = value;
    x2 = value;
    y1 = value;
    y2 = value;
  }

  float update(float input, float dtSeconds)
  {
    if (dtSeconds <= 0.0f)
      return input;

    const float currentFs = 1.0f / dtSeconds;
    if (!configured || fabsf(currentFs - sampleRateHz) > sampleRateHz * 0.05f)
      configure(cutoffHz > 0.0f ? cutoffHz : 1.0f, currentFs, q > 0.0f ? q : MPU_FILTER_Q);

    const float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1;
    x1 = input;
    y2 = y1;
    y1 = output;
    return output;
  }
};

BiquadLowPass angleXFilter;
BiquadLowPass angleYFilter;
BiquadLowPass angleZFilter;
BiquadLowPass accelXFilter;
BiquadLowPass accelYFilter;
BiquadLowPass accelZFilter;
BiquadLowPass gyroXFilter;
BiquadLowPass gyroYFilter;
BiquadLowPass gyroZFilter;

// Servo center positions and rate limits.
constexpr float PITCH_CENTRE = 20.0f;
constexpr float ROLL_CENTRE = 45.0f;
constexpr float YAW_CENTRE = 40.0f;
constexpr float MAX_STEP = 180.0f;

// Servo angle limits to keep the gimbal inside a safe range.
constexpr float PITCH_MIN = 0.0f;      // prevent forward tilt beyond this
constexpr float PITCH_MAX = 120.0f;     // prevent backward tilt beyond this
constexpr float ROLL_MIN = 0.0f;      // prevent left roll beyond this
constexpr float ROLL_MAX = 180.0f;     // prevent right roll beyond this
constexpr float YAW_MIN = 0.0f;        // prevent left yaw beyond this
constexpr float YAW_MAX = 180.0f;      // prevent right yaw beyond this

// Control gains for the stabilization response.
constexpr float PITCH_GAIN = 2.0f;
constexpr float ROLL_GAIN = 2.0f;
constexpr float YAW_GAIN = 1.2f;
// Damping from gyro rate feedback.
constexpr float PITCH_DAMP = 0.05f;
constexpr float ROLL_DAMP = 0.05f;
constexpr float YAW_DAMP = 0.05f;
// Deadbands to ignore small noise around zero.
constexpr float DEADBAND = 1.0f;
constexpr float GYRO_DEADBAND = 10.0f;

// Servo objects and current output positions.
Servo servo_pitch, servo_roll, servo_yaw;
float lastPitchPos, lastRollPos, lastYawPos;
float lockedPitchPos = PITCH_CENTRE;
float lockedRollPos = ROLL_CENTRE;
float lockedYawPos = YAW_CENTRE;

// Startup settling state for the IMU and filters.
bool mpuSettled = false;
constexpr float SETTLE_THRESHOLD = 1.0f;
constexpr uint8_t SETTLE_COUNT = 50; // Wraps above 255, do not increase above 255
uint8_t stableReadings = 0;
bool filtersJustReset = false;
uint32_t lastFilterUpdateUs = 0;
GimbalMode gimbalMode = GimbalMode::Stabilising;
GimbalMode lastGimbalMode = GimbalMode::Stabilising;
uint32_t lastModeReadMs = 0;

// Read the current gimbal mode from the PWM control input.
GimbalMode readGimbalMode()
{
  const uint32_t nowMs = millis();
  if (nowMs - lastModeReadMs < 20)
    return gimbalMode;

  lastModeReadMs = nowMs;
  const unsigned long pulseWidthUs = pulseIn(MODE_PIN, HIGH, MODE_PULSE_TIMEOUT_US);
  if (pulseWidthUs == 0)
    return GimbalMode::Stabilising;

  return (pulseWidthUs >= MODE_STABILISING_THRESHOLD_US) ? GimbalMode::Stabilising : GimbalMode::Locked;
}

// Update mode state and reset filters when re-entering stabilisation.
void handleModeTransition(GimbalMode newMode)
{
  if (newMode == gimbalMode)
    return;

  lastGimbalMode = gimbalMode;
  gimbalMode = newMode;

  if (gimbalMode == GimbalMode::Stabilising)
  {
    angleXFilter.reset(rawAngx);
    angleYFilter.reset(rawAngy);
    angleZFilter.reset(rawAngz);
    accelXFilter.reset(rawAx);
    accelYFilter.reset(rawAy);
    accelZFilter.reset(rawAz);
    gyroXFilter.reset(rawGx);
    gyroYFilter.reset(rawGy);
    gyroZFilter.reset(rawGz);

    angx = rawAngx;
    angy = rawAngy;
    angz = rawAngz;
    ax = rawAx;
    ay = rawAy;
    az = rawAz;
    gx = rawGx;
    gy = rawGy;
    gz = rawGz;

    lastFilterUpdateUs = micros();
    filtersJustReset = true;
  }

  Serial.print(F("Gimbal mode: "));
  Serial.println(gimbalMode == GimbalMode::Stabilising ? F("stabilising") : F("locked"));
}

// SD logging state and timing.
SdFat sd;
SdFile logFile;
constexpr uint16_t LOG_HZ = 50;
constexpr uint32_t LOG_PERIOD_MS = 1000UL / LOG_HZ;
constexpr uint32_t FLUSH_EVERY_MS = 1000;
uint32_t lastLogMs = 0;
uint32_t lastFlushMs = 0;
bool sdReady = false;

// Wait for the IMU to settle before enabling normal control.
bool mpuSettling()
{
  if (mpuSettled)
    return false;

  if (fabsf(rawAngx) < SETTLE_THRESHOLD &&
      fabsf(rawAngy) < SETTLE_THRESHOLD &&
      fabsf(rawAngz) < SETTLE_THRESHOLD)
  {
    if (++stableReadings >= SETTLE_COUNT)
    {
      mpuSettled = true;
      mpu.calcOffsets();

      angleXFilter.reset(rawAngx);
      angleYFilter.reset(rawAngy);
      angleZFilter.reset(rawAngz);
      accelXFilter.reset(rawAx);
      accelYFilter.reset(rawAy);
      accelZFilter.reset(rawAz);
      gyroXFilter.reset(rawGx);
      gyroYFilter.reset(rawGy);
      gyroZFilter.reset(rawGz);

      angx = rawAngx;
      angy = rawAngy;
      angz = rawAngz;
      ax = rawAx;
      ay = rawAy;
      az = rawAz;
      gx = rawGx;
      gy = rawGy;
      gz = rawGz;

      lastPitchPos = constrain(PITCH_CENTRE - angx * PITCH_GAIN, PITCH_MIN, PITCH_MAX);
      lastRollPos = constrain(ROLL_CENTRE - angy * ROLL_GAIN, ROLL_MIN, ROLL_MAX);
      lastYawPos = constrain(YAW_CENTRE - angz * YAW_GAIN, YAW_MIN, YAW_MAX);
      filtersJustReset = true;
      lastFilterUpdateUs = micros();
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

// Update filtered sensor outputs using the current time step.
void updateMpuFilters(uint32_t nowUs)
{
  if (lastFilterUpdateUs == 0)
  {
    lastFilterUpdateUs = nowUs;
    return;
  }

  const float dtSeconds = (nowUs - lastFilterUpdateUs) * 1.0e-6f;
  lastFilterUpdateUs = nowUs;

  angx = angleXFilter.update(rawAngx, dtSeconds);
  angy = angleYFilter.update(rawAngy, dtSeconds);
  angz = angleZFilter.update(rawAngz, dtSeconds);
  ax = accelXFilter.update(rawAx, dtSeconds);
  ay = accelYFilter.update(rawAy, dtSeconds);
  az = accelZFilter.update(rawAz, dtSeconds);
  gx = gyroXFilter.update(rawGx, dtSeconds);
  gy = gyroYFilter.update(rawGy, dtSeconds);
  gz = gyroZFilter.update(rawGz, dtSeconds);
}

// Return zero for small values so noise does not drive the servos.
static inline float deadband(float v, float db)
{
  return (fabsf(v) < db) ? 0.0f : v;
}

// Convert filtered IMU data into stabilizing servo commands.
void filterAndActuate()
{
  const float ep = deadband(angx, DEADBAND);
  const float er = deadband(angy, DEADBAND);
  const float ey = deadband(angz, DEADBAND);
  const float dg = deadband(gx, GYRO_DEADBAND);
  const float dr = deadband(gy, GYRO_DEADBAND);
  const float dy = deadband(gz, GYRO_DEADBAND);

  float pPos = constrain(PITCH_CENTRE - ep * PITCH_GAIN - dg * PITCH_DAMP, PITCH_MIN, PITCH_MAX);
  float rPos = constrain(ROLL_CENTRE - er * ROLL_GAIN - dr * ROLL_DAMP, ROLL_MIN, ROLL_MAX);
  float yPos = constrain(YAW_CENTRE - ey * YAW_GAIN - dy * YAW_DAMP, YAW_MIN, YAW_MAX);

  // Rate-limit
  lastPitchPos += constrain(pPos - lastPitchPos, -MAX_STEP, MAX_STEP);
  lastRollPos += constrain(rPos - lastRollPos, -MAX_STEP, MAX_STEP);
  lastYawPos += constrain(yPos - lastYawPos, -MAX_STEP, MAX_STEP);

  servo_yaw.write((int)lastYawPos);
  servo_roll.write((int)lastRollPos);
  servo_pitch.write((int)lastPitchPos);
}

// Hold the servos at the stored locked position.
void holdLockedPosition()
{
  lastPitchPos += constrain(lockedPitchPos - lastPitchPos, -MAX_STEP, MAX_STEP);
  lastRollPos += constrain(lockedRollPos - lastRollPos, -MAX_STEP, MAX_STEP);
  lastYawPos += constrain(lockedYawPos - lastYawPos, -MAX_STEP, MAX_STEP);

  servo_yaw.write((int)lastYawPos);
  servo_roll.write((int)lastRollPos);
  servo_pitch.write((int)lastPitchPos);
}

// Initialize hardware, sensors, filters, servos, and logging.
void setup()
{

  pinMode(MODE_PIN, INPUT);

  Wire.begin();
  Wire.setClock(400000UL); // Fast-mode I²C — MPU6050 supports it

  Serial.begin(115200);
  Serial.println(F("Arduino Ready. Connecting to MPU..."));

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
  lastFilterUpdateUs = micros();

  // Center servos on startup
  servo_pitch.write(PITCH_CENTRE);
  servo_roll.write(ROLL_CENTRE);
  servo_yaw.write(YAW_CENTRE);
  lockedPitchPos = PITCH_CENTRE;
  lockedRollPos = ROLL_CENTRE;
  lockedYawPos = YAW_CENTRE;
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
    logFile.println(F("t_ms,f_ax_mps2,f_ay_mps2,f_az_mps2,f_gx_rps,f_gy_rps,f_gz_rps"));
    logFile.flush();
  }

  lastLogMs = millis();
  lastFlushMs = lastLogMs;

  delay(1000);
}

// Main control loop that reads sensors, updates mode, drives servos, and logs data.
void loop()
{
  uint32_t now = millis();
  mpu.update();
  rawAngx = mpu.getAngleX();
  rawAngy = mpu.getAngleY();
  rawAngz = mpu.getAngleZ();
  rawAx = mpu.getAccX() * G_MPS2;
  rawAy = mpu.getAccY() * G_MPS2;
  rawAz = mpu.getAccZ() * G_MPS2;
  rawGx = mpu.getGyroX();
  rawGy = mpu.getGyroY();
  rawGz = mpu.getGyroZ();

  if (mpuSettling())
    return; // No data gathered until settled

  handleModeTransition(readGimbalMode());

  updateMpuFilters(micros());

  if (gimbalMode == GimbalMode::Stabilising)
  {
    if (filtersJustReset)
    {
      filtersJustReset = false;
    }
    else
    {
      filterAndActuate(); // Filters all MPU readings and writes to servos
    }
  }
  else
  {
    holdLockedPosition();
  }

  // ---- Logging (rate-limited) ----
  if (now - lastLogMs >= LOG_PERIOD_MS)
  {
    lastLogMs += LOG_PERIOD_MS;

    // Write post-filter MPU outputs to SD without snprintf (saves ~128 bytes RAM + ~1.5 KB flash)
    if (sdReady)
    {
      const float filteredAx = ax;
      const float filteredAy = ay;
      const float filteredAz = az;
      const float filteredGx = gx;
      const float filteredGy = gy;
      const float filteredGz = gz;

      logFile.print(now);
      logFile.print(',');
      logFile.print(filteredAx, 3);
      logFile.print(',');
      logFile.print(filteredAy, 3);
      logFile.print(',');
      logFile.print(filteredAz, 3);
      logFile.print(',');
      logFile.print(filteredGx, 3);
      logFile.print(',');
      logFile.print(filteredGy, 3);
      logFile.print(',');
      logFile.println(filteredGz, 3);
    }
  }

  if (sdReady && now - lastFlushMs >= FLUSH_EVERY_MS)
  {
    lastFlushMs = now;
    logFile.flush();
  }

  // ---- Teleplot (rate-limited to avoid flooding serial) ----
  // Outputs every 5 ms (~200 Hz)
  static uint32_t lastTeleMs = 0;
  if (now - lastTeleMs >= 5)
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