// Polar obstacle scanner: MPU6050 (bearing) + HC-SR04 (range).
// Same CSV as polar_scanner.ino: t_ms,yaw_deg,pitch_deg,roll_deg,range_mm,ok
// Sweep the sensor left/right; the host bins (yaw, range) into a polar map.
//
// Libraries (Arduino Library Manager):
//   Adafruit MPU6050, Adafruit Unified Sensor, Adafruit BusIO
//
// Wiring (ESP32 devkit):
//   MPU6050: SDA -> GPIO21, SCL -> GPIO22, VCC -> 3V3, GND -> GND (0x68)
//   HC-SR04: VCC -> 5V (VIN), GND -> GND, TRIG -> GPIO5, ECHO -> GPIO18
//   ECHO is a 5 V push-pull output and the ESP32 is 3.3 V only: put a divider
//   on it (1k from ECHO to the pin, 2k from the pin to GND) or you cook GPIO18.

#include <Wire.h>
#include <Adafruit_MPU6050.h>

// ---- tuning knobs ----------------------------------------------------------
#define SDA_PIN            21
#define SCL_PIN            22
#define I2C_HZ             400000
#define LOOP_HZ            100      // gyro integration rate
#define GYRO_CAL_SAMPLES   600      // hold still during boot calibration
#define ZUPT_DPS           1.2f     // below this the head is "still": don't integrate yaw
#define YAW_SCALE          1.0f     // spin exactly 360 deg; if it reads 352, set 360/352
#define COMP_ALPHA         0.98f    // complementary filter for pitch/roll
#define TRIG_PIN           5
#define ECHO_PIN           18
#define RANGE_PERIOD_MS    500      // 2 measurements/s
#define ECHO_RISE_US       5000     // trigger -> echo rising; a live sensor is
                                    // under 1 ms. Timing out here means no pulse
                                    // ever came back: wiring, or no 5 V.
#define ECHO_HIGH_US       30000    // echo high time ceiling. Past RANGE_MAX_MM
                                    // (23 ms) so nothing valid is cut, but under
                                    // the ~38 ms the module parks high on "no
                                    // object", so we bail early instead of
                                    // blocking the gyro loop for 38 ms.
#define SOUND_MM_PER_US    0.1715f  // 343 m/s / 2 (there and back), at ~20 C
                                    // calibration knob: raise if it reads short
#define RANGE_MIN_MM       20       // HC-SR04 blind zone
#define RANGE_MAX_MM       2000

// ---- mount: which raw MPU axis is the sensor bore, +Y (left), +Z (up)? -----
// Each entry picks a raw MPU axis and a sign: 1=+X 2=+Y 3=+Z, negate for -.
// The HC-SR04 and the MPU6050 face the same way here, and "the way a breakout
// board faces" is its normal: the MPU's raw +Z. So the bore is raw +Z, not the
// raw +X the VL53L0X rig used (there the ToF sat on the front edge, normal
// along the MPU's in-plane X). Tipping fwd from +X to +Z about the left axis
// carries the old up (+Z) forward and the old fwd (+X) down, hence UP = -X.
// Right-handed check: fwd x left = +Z_raw x +Y_raw = -X_raw = up. Good.
//
// Roll ambiguity: mounting the board 180 deg rolled is equally plausible and
// indistinguishable on paper -- it gives MOUNT_LEFT -2, MOUNT_UP 1. Send 'm'
// to settle it against your actual rig.
#define MOUNT_FWD           3      // sensor +X (bore / ahead)  <- raw +Z
#define MOUNT_LEFT          2      // sensor +Y (left)          <- raw +Y
#define MOUNT_UP           -1      // sensor +Z (up)            <- raw -X
// ---------------------------------------------------------------------------

Adafruit_MPU6050 mpu;

float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
float gxBias = 0, gyBias = 0, gzBias = 0;
uint32_t lastMicros = 0;
uint32_t lastPingMs = 0;
uint32_t echoSilentPings = 0;
uint16_t rangeMm = 0;
uint8_t rangeOk = 0;

static const float RAD2DEG = 57.2957795f;

// Remap raw MPU axes into the sensor's frame (+X bore, +Y left, +Z up) using
// the MOUNT_* knobs above. Doing it once here means every formula downstream --
// yaw integration, the complementary filter, the CSV -- stays in that frame.
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

// ponytail: blocking wait, up to ~35 ms once per 500 ms. Move to an ECHO
// interrupt + micros() timestamps if that hiccup ever shows up in the yaw.
//
// Hand-rolled instead of pulseIn() because the ESP32 core's pulseIn shares ONE
// timeout across its wait-for-low, wait-for-high and wait-for-low again
// (wiring_pulse.c: start_cycle_count is captured once), so a single number has
// to cover both the rise latency and the pulse -- and every failure returns a
// bare 0. Splitting them tells "sensor never answered" (wiring / no 5 V) apart
// from "answered, nothing in range", which is the distinction worth debugging.
// Returns echo high time in us, 0 = no rise, UINT32_MAX = stuck high.
static uint32_t echoPulseUs() {
  // ECHO must be idle low before triggering. After a stuck-high abort the pin
  // is still high from the last shot, and the wait-for-rise below would fall
  // straight through and time the tail of that old pulse as a fresh reading.
  uint32_t idle = micros();
  while (digitalRead(ECHO_PIN) == HIGH) {
    if (micros() - idle > ECHO_HIGH_US) return UINT32_MAX;   // never settles
  }

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  uint32_t t0 = micros();
  while (digitalRead(ECHO_PIN) == LOW) {
    if (micros() - t0 > ECHO_RISE_US) return 0;
  }
  uint32_t rise = micros();
  while (digitalRead(ECHO_PIN) == HIGH) {
    if (micros() - rise > ECHO_HIGH_US) return UINT32_MAX;
  }
  return micros() - rise;
}

static void pingRange() {
  uint32_t us = echoPulseUs();
  if (us == 0) {
    // Distinct from an out-of-range reading: nothing drove ECHO at all.
    if (echoSilentPings++ % 8 == 0)
      Serial.println(F("# WARN: no echo rise - check ECHO wiring, divider, 5 V on VCC"));
    rangeOk = 0;
    return;
  }
  if (us == UINT32_MAX) { rangeOk = 0; return; }   // stuck high = nothing in range

  float mm = us * SOUND_MM_PER_US;
  rangeOk = (mm >= RANGE_MIN_MM && mm <= RANGE_MAX_MM) ? 1 : 0;
  if (rangeOk) rangeMm = (uint16_t)mm;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN, I2C_HZ);

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println(F("# ERROR: MPU6050 not found at 0x68"));
    while (1) delay(1000);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // Probe the HC-SR04 once before the stream starts: a wiring fault otherwise
  // shows up only as an unbroken run of ok=0 rows, which reads like "nothing in
  // range" instead of "sensor not connected".
  uint32_t probe = echoPulseUs();
  if (probe == 0)
    Serial.println(F("# ERROR: HC-SR04 silent. ECHO -> GPIO18 via 1k/2k divider, VCC -> 5V"));
  else if (probe == UINT32_MAX)
    Serial.println(F("# WARN: HC-SR04 echo stuck high - nothing in range?"));
  else {
    Serial.print(F("# HC-SR04 ok, "));
    Serial.print((uint16_t)(probe * SOUND_MM_PER_US));
    Serial.println(F(" mm"));
  }

  calibrateGyro();
  Serial.println(F("# t_ms,yaw_deg,pitch_deg,roll_deg,range_mm,ok"));
  lastMicros = micros();
}

void loop() {
  // 'z' zeroes the heading (point straight ahead first), 'c' recalibrates,
  // 'm' prints raw accel so you can read your mount off it (see MOUNT_* above),
  // 'p' pings once and prints the raw echo time -- use it to sanity-check the
  // wiring against a wall at a known distance.
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') { yaw = 0.0f; Serial.println(F("# yaw zeroed")); }
    else if (c == 'c') calibrateGyro();
    else if (c == 'p') {
      uint32_t us = echoPulseUs();
      Serial.print(F("# echo us: "));
      if (us == 0) Serial.println(F("0 (no rise - ECHO not driven)"));
      else if (us == UINT32_MAX) Serial.println(F("stuck high (no object)"));
      else { Serial.print(us); Serial.print(F(" -> ")); Serial.print((uint16_t)(us * SOUND_MM_PER_US)); Serial.println(F(" mm")); }
    }
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
  mountRotate(rgx, rgy, rgz);   // into the sensor's frame
  mountRotate(rax, ray, raz);

  float gx = (rgx - gxBias) * RAD2DEG;   // deg/s
  float gy = (rgy - gyBias) * RAD2DEG;
  float gz = (rgz - gzBias) * RAD2DEG;

  // Zero-velocity update: without a magnetometer, yaw only comes from
  // integrating gz, so freeze it while stationary or it walks away.
  if (fabsf(gz) > ZUPT_DPS) yaw = wrap180(yaw + gz * dt * YAW_SCALE);

  // Gravity gives absolute pitch/roll; blend to kill gyro drift on those axes.
  // Sign: at rest the accel reads +9.8 along whichever axis points up, so bore
  // straight up puts +g on +X and must read pitch = +90 -- that is the
  // elevation the host feeds to sin() for the Z coordinate. Rotation about
  // +Y (left) by the right-hand rule tips the bore down, so gy is subtracted.
  float accPitch = atan2f(rax, sqrtf(ray * ray + raz * raz)) * RAD2DEG;
  float accRoll  = atan2f(ray, raz) * RAD2DEG;
  pitch = COMP_ALPHA * (pitch - gy * dt) + (1.0f - COMP_ALPHA) * accPitch;
  roll  = COMP_ALPHA * (roll  + gx * dt) + (1.0f - COMP_ALPHA) * accRoll;

  if (millis() - lastPingMs >= RANGE_PERIOD_MS) {
    lastPingMs = millis();
    pingRange();
    lastMicros = micros();   // don't charge the ping's blocking time to dt
  }

  Serial.print(millis());       Serial.print(',');
  Serial.print(yaw, 2);         Serial.print(',');
  Serial.print(pitch, 2);       Serial.print(',');
  Serial.print(roll, 2);        Serial.print(',');
  Serial.print(rangeMm);        Serial.print(',');
  Serial.println(rangeOk);
}
