// Polar obstacle scanner: MPU6050 (bearing) + VL53L0X (range) on one I2C bus.
// Streams CSV over serial: t_ms,yaw_deg,pitch_deg,roll_deg,range_mm,ok
// Sweep the sensor left/right; the host bins (yaw, range) into a polar map.
//
// Libraries (Arduino Library Manager):
//   Adafruit MPU6050, Adafruit VL53L0X, Adafruit Unified Sensor, Adafruit BusIO
//
// Wiring (ESP32 devkit, both sensors on the same bus, 3V3):
//   SDA -> GPIO21, SCL -> GPIO22, VCC -> 3V3, GND -> GND
//   MPU6050 = 0x68, VL53L0X = 0x29 (no address conflict)
//   VL53L0X XSHUT/GPIO1 unconnected.

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_VL53L0X.h>

// ---- tuning knobs ----------------------------------------------------------
#define SDA_PIN            21
#define SCL_PIN            22
#define I2C_HZ             400000
#define LOOP_HZ            100      // gyro integration rate
#define GYRO_CAL_SAMPLES   600      // hold still during boot calibration
#define ZUPT_DPS           1.2f     // below this the head is "still": don't integrate yaw
#define YAW_SCALE          1.0f     // spin exactly 360 deg; if it reads 352, set 360/352
#define COMP_ALPHA         0.98f    // complementary filter for pitch/roll
#define RANGE_TIMING_MS    250       // VL53L0X continuous-mode inter-measurement period
#define RANGE_BUDGET_US    33000    // per-shot integration time; longer = more range,
                                    // slower. Keep RANGE_TIMING_MS >= this / 1000.

// ---- mount: which raw MPU axis is the ToF bore, +Y (left), +Z (up)? --------
// Each entry picks a raw MPU axis and a sign: 1=+X 2=+Y 3=+Z, negate for -.
// Measured from run1.csv: with the bore pointing up, gravity sat on raw +X
// (+1.00,-0.01,-0.05), so the bore IS the MPU's +X axis.
// Send 'm' over serial to print the raw accel and read your own mount off it:
// hold the bore straight up, whichever axis reads ~+9.8 is MOUNT_FWD.
#define MOUNT_FWD           1      // ToF +X (bore / ahead)
#define MOUNT_LEFT          2      // ToF +Y (left)
#define MOUNT_UP            3      // ToF +Z (up)
// ---------------------------------------------------------------------------

Adafruit_MPU6050 mpu;
Adafruit_VL53L0X tof;

float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
float gxBias = 0, gyBias = 0, gzBias = 0;
uint32_t lastMicros = 0;
uint16_t rangeMm = 0;
uint8_t rangeOk = 0;

static const float RAD2DEG = 57.2957795f;

// Remap raw MPU axes into the ToF's frame (+X bore, +Y left, +Z up) using the
// MOUNT_* knobs above. Doing it once here means every formula downstream --
// yaw integration, the complementary filter, the CSV -- stays in the ToF frame.
// If yaw runs backwards after this, set YAW_SCALE negative.
static inline float pickAxis(int sel, float x, float y, float z) {
  switch (sel) {
    case  1: return  x;   case -1: return -x;
    case  2: return  y;   case -2: return -y;
    case  3: return  z;   default: return (sel == -3) ? -z : 0.0f;
  }
}

static inline void mountRotate(float &x, float &y, float &z) {
  float ox = x, oy = y, oz = z;
  x = pickAxis(MOUNT_FWD,  ox, oy, oz);
  y = pickAxis(MOUNT_LEFT, ox, oy, oz);
  z = pickAxis(MOUNT_UP,   ox, oy, oz);
}

static float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

static void calibrateGyro() {
  Serial.println(F("# calibrating gyro - hold still"));
  double sx = 0, sy = 0, sz = 0;
  sensors_event_t a, g, t;
  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    mpu.getEvent(&a, &g, &t);
    float gx = g.gyro.x, gy = g.gyro.y, gz = g.gyro.z;
    mountRotate(gx, gy, gz);   // bias lives in the same frame the loop uses
    sx += gx; sy += gy; sz += gz;
    delay(3);
  }
  gxBias = sx / GYRO_CAL_SAMPLES;
  gyBias = sy / GYRO_CAL_SAMPLES;
  gzBias = sz / GYRO_CAL_SAMPLES;
  yaw = pitch = roll = 0.0f;
  Serial.print(F("# gyro bias rad/s: "));
  Serial.print(gxBias, 5); Serial.print(',');
  Serial.print(gyBias, 5); Serial.print(',');
  Serial.println(gzBias, 5);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Wire.begin(SDA_PIN, SCL_PIN, I2C_HZ);

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println(F("# ERROR: MPU6050 not found at 0x68"));
    while (1) delay(1000);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  if (!tof.begin(0x29, false, &Wire)) {
    Serial.println(F("# ERROR: VL53L0X not found at 0x29"));
    while (1) delay(1000);
  }
  // Default profile caps out around 30 cm: it wants 0.25 MCPS of return signal
  // and uses short VCSEL pulses, so weak far returns are binned as status 4.
  // Long range drops the limit to 0.1 MCPS and lengthens the pulses -> ~2 m.
  tof.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  // Longer budget = more photons integrated per shot = the 2 m returns survive.
  tof.setMeasurementTimingBudgetMicroSeconds(200000);
  tof.startRangeContinuous(RANGE_TIMING_MS);

  calibrateGyro();
  Serial.println(F("# t_ms,yaw_deg,pitch_deg,roll_deg,range_mm,ok"));
  lastMicros = micros();
}

void loop() {
  // 'z' zeroes the heading (point straight ahead first), 'c' recalibrates,
  // 'm' prints raw accel so you can read your mount off it (see MOUNT_* above).
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') { yaw = 0.0f; Serial.println(F("# yaw zeroed")); }
    else if (c == 'c') calibrateGyro();
    else if (c == 'm') {
      sensors_event_t ma, mg, mt;
      mpu.getEvent(&ma, &mg, &mt);
      Serial.print(F("# raw accel x,y,z m/s^2: "));
      Serial.print(ma.acceleration.x, 2); Serial.print(',');
      Serial.print(ma.acceleration.y, 2); Serial.print(',');
      Serial.println(ma.acceleration.z, 2);
      Serial.println(F("# bore straight up: axis reading ~+9.8 is MOUNT_FWD"));
    }
  }

  uint32_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  if (dt < 1.0f / LOOP_HZ) return;
  lastMicros = now;

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  float rgx = g.gyro.x, rgy = g.gyro.y, rgz = g.gyro.z;
  float rax = a.acceleration.x, ray = a.acceleration.y, raz = a.acceleration.z;
  mountRotate(rgx, rgy, rgz);   // into the ToF's frame
  mountRotate(rax, ray, raz);

  float gx = (rgx - gxBias) * RAD2DEG;   // deg/s
  float gy = (rgy - gyBias) * RAD2DEG;
  float gz = (rgz - gzBias) * RAD2DEG;

  // Zero-velocity update: without a magnetometer, yaw only comes from
  // integrating gz, so freeze it while stationary or it walks away.
  if (fabsf(gz) > ZUPT_DPS) yaw = wrap180(yaw + gz * dt * YAW_SCALE);

  // Gravity gives absolute pitch/roll; blend to kill gyro drift on those axes.
  // Sign: at rest the accel reads +9.8 along whichever axis points up, so bore
  // straight up puts +g on ToF +X and must read pitch = +90 -- that is the
  // elevation the host feeds to sin() for the Z coordinate. Rotation about
  // +Y (left) by the right-hand rule tips the bore down, so gy is subtracted.
  float accPitch = atan2f(rax, sqrtf(ray * ray + raz * raz)) * RAD2DEG;
  float accRoll  = atan2f(ray, raz) * RAD2DEG;
  pitch = COMP_ALPHA * (pitch - gy * dt) + (1.0f - COMP_ALPHA) * accPitch;
  roll  = COMP_ALPHA * (roll  + gx * dt) + (1.0f - COMP_ALPHA) * accRoll;

  // Non-blocking range read: keeps the integration loop at a steady dt.
  if (tof.isRangeComplete()) {
    uint16_t r = tof.readRange();
    rangeOk = (tof.readRangeStatus() == 0) ? 1 : 0;   // 4 = out of range
    if (rangeOk) rangeMm = r;
  }

  Serial.print(millis());       Serial.print(',');
  Serial.print(yaw, 2);         Serial.print(',');
  Serial.print(pitch, 2);       Serial.print(',');
  Serial.print(roll, 2);        Serial.print(',');
  Serial.print(rangeMm);        Serial.print(',');
  Serial.println(rangeOk);
}
