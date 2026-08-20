/*
  CAN bus:
    500 kbit/s, standard 11-bit IDs

  Sensor ECU -> Raspberry Pi:
    0x200 = range + servo + status
    0x201 = acceleration
    0x202 = gyro
    0x203 = roll/pitch/yaw
    0x204 = left/center/right scan

  Battery ECU -> Sensor ECU:
    0x100 = battery status, used for OLED #2

  Raspberry Pi -> Sensor ECU:
    0x400 = safety-filtered drive command, mode and E-stop status
            used for OLED #1 and MAX7219 direction arrows
    0x401 = browser vehicle lock/unlock command
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MD_MAX72xx.h>
#include <ESP32Servo.h>
#include <driver/twai.h>
#include <math.h>
#include <stdint.h>

// =====================================================
// USER SETTINGS
// =====================================================

const bool ENABLE_SERVO_SCAN = true;

bool vehicleUnlocked = false;
const uint8_t AUTHORIZED_RFID_UID[4] = {0x2D, 0xB9, 0xEC, 0x44};

// Change these if either OLED uses address 0x3D.
const uint8_t OLED1_ADDRESS = 0x3C;
const uint8_t OLED2_ADDRESS = 0x3C;

const uint8_t MPU_ADDRESS_PRIMARY = 0x68;
const uint8_t MPU_ADDRESS_SECONDARY = 0x69;

// =====================================================
// PIN DEFINITIONS
// =====================================================

const int SERVO_PIN = 14;

const int I2C1_SDA = 21;
const int I2C1_SCL = 22;

// OLED #2 uses its dedicated secondary I2C pins.
const int I2C2_SDA = 26;
const int I2C2_SCL = 25;

const int RFID_SS_PIN = 19;
const int RFID_SCK_PIN = 33;
const int RFID_MOSI_PIN = 13;
const int RFID_MISO_PIN = 27;
const int RFID_RST_PIN = 32;

const int ULTRASONIC_TRIG_PIN = 15;
const int ULTRASONIC_ECHO_PIN = 4;

const int MATRIX_DIN_PIN = 17;
const int MATRIX_CLK_PIN = 23;
const int MATRIX_CS_PIN = 16;

// SN65HVD230 CAN transceiver.
const gpio_num_t CAN_TX_PIN = GPIO_NUM_5;
const gpio_num_t CAN_RX_PIN = GPIO_NUM_18;

// =====================================================
// CAN PROTOCOL
// =====================================================

const uint32_t BATTERY_STATUS_CAN_ID = 0x100;

const uint32_t SENSOR_STATUS_CAN_ID      = 0x200;
const uint32_t SENSOR_ACCEL_CAN_ID       = 0x201;
const uint32_t SENSOR_GYRO_CAN_ID        = 0x202;
const uint32_t SENSOR_ORIENTATION_CAN_ID = 0x203;
const uint32_t SENSOR_SCAN_CAN_ID        = 0x204;
const uint32_t PI_DRIVE_COMMAND_CAN_ID = 0x400;
const uint32_t PI_LOCK_COMMAND_CAN_ID  = 0x401;

// Sensor telemetry is transmitted at 10 Hz.
// The Pi declares an ECU stale after 500 ms
const unsigned long CAN_TX_INTERVAL_MS = 100;

// =====================================================
// OLED CONFIGURATION
// =====================================================

const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 64;
const int OLED_RESET_PIN = -1;

TwoWire I2C_BUS_1 = TwoWire(0);
TwoWire I2C_BUS_2 = TwoWire(1);

Adafruit_SSD1306 oled1(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &I2C_BUS_1,
  OLED_RESET_PIN
);

Adafruit_SSD1306 oled2(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &I2C_BUS_2,
  OLED_RESET_PIN
);

// =====================================================
// LED MATRIX CONFIGURATION
// =====================================================

#define MATRIX_HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MATRIX_DEVICE_COUNT 1

MD_MAX72XX matrix(
  MATRIX_HARDWARE_TYPE,
  MATRIX_DIN_PIN,
  MATRIX_CLK_PIN,
  MATRIX_CS_PIN,
  MATRIX_DEVICE_COUNT
);

// =====================================================
// RFID CONFIGURATION
// =====================================================

MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);

bool rfidWorking = false;
unsigned long lastRfidReadTime = 0;
const unsigned long RFID_SCAN_COOLDOWN_MS = 900;

// =====================================================
// SERVO / ULTRASONIC SCAN CONFIGURATION
// =====================================================

Servo sensorServo;

const int SERVO_CENTER_DEG = 90;
const int SERVO_SCAN_LIMIT_DEG = 60;
const int SERVO_MIN_PULSE_US = 500;
const int SERVO_MAX_PULSE_US = 2400;

const unsigned long SERVO_POSITION_DWELL_MS = 700;
const unsigned long ULTRASONIC_TIMEOUT_US = 30000;

const int SERVO_SCAN_POSITIONS[] = {
  -SERVO_SCAN_LIMIT_DEG,
  0,
  SERVO_SCAN_LIMIT_DEG,
  0
};
const int SERVO_SCAN_POSITION_COUNT =
  sizeof(SERVO_SCAN_POSITIONS) / sizeof(SERVO_SCAN_POSITIONS[0]);

// Start centered. Index 1 corresponds to the CENTER entry above.
int servoScanPositionIndex = 1;
int servoScanAngleDeg = 0;

float latestInstantDistanceCm = NAN;
float scanLeftCm = NAN;
float scanCenterCm = NAN;
float scanRightCm = NAN;

// =====================================================
// MPU CONFIGURATION
// =====================================================

bool mpuWorking = false;
uint8_t detectedMpuAddress = 0;

float latestAxG = NAN;
float latestAyG = NAN;
float latestAzG = NAN;

float latestGxDps = NAN;
float latestGyDps = NAN;
float latestGzDps = NAN;

float latestRollDeg = 0.0f;
float latestPitchDeg = 0.0f;
float latestYawDeg = 0.0f;

bool imuFilterInitialized = false;
unsigned long previousImuMicros = 0;

const unsigned long IMU_UPDATE_INTERVAL_MS = 20; // 50 Hz.

// MPU full-scale settings used below:
// accel +/-2 g  -> 16384 LSB/g
// gyro  +/-250 dps -> 131 LSB/(deg/s)
const float ACCEL_LSB_PER_G = 16384.0f;
const float GYRO_LSB_PER_DPS = 131.0f;
const float COMPLEMENTARY_ALPHA = 0.98f;

// =====================================================
// DEVICE / CAN STATUS
// =====================================================

bool oled1Working = false;
bool oled2Working = false;
bool canWorking = false;

// Battery ECU data shown on OLED #2.
bool batteryMessageReceived = false;
float canBatteryVoltageV = NAN;
float canBatteryTemperatureC = NAN;
float canBatteryCurrentMa = NAN;
uint16_t batteryCanMessageCounter = 0;
unsigned long lastBatteryCanMessageTime = 0;
const unsigned long BATTERY_CAN_TIMEOUT_MS = 2000;

// One burst counter is shared by 0x200-0x204 so a receiver can associate
// the five frames from the same telemetry update.
uint16_t sensorCanCounter = 0;
uint32_t canTxFailureCount = 0;

// Latest safety-filtered Raspberry Pi drive command (CAN 0x400).
bool piDriveCommandReceived = false;
int8_t piDriveLeft = 0;
int8_t piDriveRight = 0;
uint8_t piDriveMode = 1;       // 0=MANUAL, 1=ASSISTED, 2=AUTONOMOUS
uint8_t piSpeedLimit = 0;
bool piEstopActive = false;
uint8_t piSafetyFlags = 0;
uint16_t piDriveCounter = 0;
unsigned long lastPiDriveCommandTime = 0;
const unsigned long PI_COMMAND_TIMEOUT_MS = 500;

// Latest Raspberry Pi browser lock command (CAN 0x401).
bool piLockCommandReceived = false;
uint16_t piLockCommandCounter = 0;
unsigned long lastPiLockCommandTime = 0;

// =====================================================
// TIMING
// =====================================================

unsigned long previousCanTxTime = 0;
unsigned long previousServoScanTime = 0;
unsigned long previousDisplayTime = 0;
unsigned long previousSerialStatusTime = 0;

const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 250;
const unsigned long SERIAL_STATUS_INTERVAL_MS = 1000;

// =====================================================
// GENERAL HELPERS
// =====================================================

float clampFloat(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

float normalizeAngle180(float angleDeg) {
  while (angleDeg >= 180.0f) angleDeg -= 360.0f;
  while (angleDeg < -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

uint16_t centimetersToMillimetersU16(float distanceCm) {
  if (isnan(distanceCm) || distanceCm < 0.0f) {
    return UINT16_MAX;
  }

  float mm = clampFloat(distanceCm * 10.0f, 0.0f, 65534.0f);
  return static_cast<uint16_t>(lroundf(mm));
}

int16_t floatToScaledI16(float value, float scale) {
  if (isnan(value)) {
    return INT16_MIN;
  }

  float scaled = value * scale;
  scaled = clampFloat(scaled, -32767.0f, 32767.0f);
  return static_cast<int16_t>(lroundf(scaled));
}

void putU16LE(uint8_t *data, int offset, uint16_t value) {
  data[offset] = static_cast<uint8_t>(value & 0xFF);
  data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void putI16LE(uint8_t *data, int offset, int16_t value) {
  uint16_t raw = static_cast<uint16_t>(value);
  putU16LE(data, offset, raw);
}


int8_t decodeI8(uint8_t value) {
  return static_cast<int8_t>(value);
}

bool piCommandFresh() {
  return piDriveCommandReceived &&
         (millis() - lastPiDriveCommandTime <= PI_COMMAND_TIMEOUT_MS);
}

bool servoScanShouldBeActive() {
  return ENABLE_SERVO_SCAN &&
         piCommandFresh() &&
         piDriveMode != 0;
}

const char *driveModeName() {
  switch (piDriveMode) {
    case 0: return "MANUAL";
    case 2: return "AUTO";
    default: return "ASSISTED";
  }
}

int displayedDriveSpeedPercent() {
  if (!piCommandFresh() || !vehicleUnlocked || piEstopActive) {
    return 0;
  }

  int speed = (abs(static_cast<int>(piDriveLeft)) + abs(static_cast<int>(piDriveRight))) / 2;
  return constrain(speed, 0, 100);
}

const char *vehicleStatusName() {
  if (piEstopActive) {
    return "E-BRAKE";
  }

  bool batteryFresh = batteryMessageReceived && (millis() - lastBatteryCanMessageTime <= BATTERY_CAN_TIMEOUT_MS);

  if (!canWorking || !rfidWorking || !piCommandFresh() ||
      !batteryFresh || piSafetyFlags != 0 || !mpuWorking) {
    return "ERROR";
  }

  return "NORMAL";
}

int estimateBatteryPercent(float packVoltageV) {
  if (isnan(packVoltageV)) {
    return -1;
  }

  // Approximate 2S LiPo state-of-charge from resting pack voltage.
  // Under load the displayed percentage may temporarily sag.
  const float perCell = packVoltageV / 2.0f;

  struct SocPoint { float v; int pct; };
  static const SocPoint table[] = {
    {4.20f, 100}, {4.15f, 95}, {4.11f, 90}, {4.08f, 85},
    {4.02f, 80},  {3.98f, 75}, {3.95f, 70}, {3.91f, 65},
    {3.87f, 60},  {3.85f, 55}, {3.84f, 50}, {3.82f, 45},
    {3.80f, 40},  {3.79f, 35}, {3.77f, 30}, {3.75f, 25},
    {3.73f, 20},  {3.71f, 15}, {3.69f, 10}, {3.61f, 5},
    {3.20f, 0}
  };

  if (perCell >= table[0].v) return 100;
  if (perCell <= table[20].v) return 0;

  for (size_t i = 0; i < 20; i++) {
    if (perCell <= table[i].v && perCell >= table[i + 1].v) {
      float fraction = (perCell - table[i + 1].v) /
                       (table[i].v - table[i + 1].v);
      float pct = table[i + 1].pct +
                  fraction * (table[i].pct - table[i + 1].pct);
      return constrain(static_cast<int>(lroundf(pct)), 0, 100);
    }
  }

  return 0;
}

// =====================================================
// I2C HELPERS
// =====================================================

bool pingI2CDevice(TwoWire &bus, uint8_t address) {
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

void scanI2CBus(TwoWire &bus, const char *busName) {
  Serial.println();
  Serial.print("Scanning ");
  Serial.println(busName);

  int deviceCount = 0;

  for (uint8_t address = 1; address < 127; address++) {
    bus.beginTransmission(address);
    uint8_t error = bus.endTransmission();

    if (error == 0) {
      Serial.print("  Found device at 0x");
      if (address < 0x10) Serial.print("0");
      Serial.println(address, HEX);
      deviceCount++;
    }
  }

  if (deviceCount == 0) {
    Serial.println("  No I2C devices found.");
  } else {
    Serial.print("  Total devices found: ");
    Serial.println(deviceCount);
  }
}

bool writeI2CRegister(
  TwoWire &bus,
  uint8_t deviceAddress,
  uint8_t registerAddress,
  uint8_t value
) {
  bus.beginTransmission(deviceAddress);
  bus.write(registerAddress);
  bus.write(value);
  return bus.endTransmission() == 0;
}

bool readI2CRegister(
  TwoWire &bus,
  uint8_t deviceAddress,
  uint8_t registerAddress,
  uint8_t &result
) {
  bus.beginTransmission(deviceAddress);
  bus.write(registerAddress);

  if (bus.endTransmission(false) != 0) {
    return false;
  }

  uint8_t bytesReceived = bus.requestFrom(
    deviceAddress,
    static_cast<uint8_t>(1)
  );

  if (bytesReceived != 1) {
    return false;
  }

  result = bus.read();
  return true;
}

// =====================================================
// MPU
// =====================================================

bool detectMPU() {
  uint8_t whoAmI = 0;

  const uint8_t possibleAddresses[] = {
    MPU_ADDRESS_PRIMARY,
    MPU_ADDRESS_SECONDARY
  };

  for (uint8_t address : possibleAddresses) {
    if (!pingI2CDevice(I2C_BUS_1, address)) {
      continue;
    }

    if (!readI2CRegister(I2C_BUS_1, address, 0x75, whoAmI)) {
      continue;
    }

    Serial.print("MPU WHO_AM_I at 0x");
    Serial.print(address, HEX);
    Serial.print(": 0x");
    Serial.println(whoAmI, HEX);

    if (
      whoAmI == 0x71 ||
      whoAmI == 0x73 ||
      whoAmI == 0x70 ||
      whoAmI == 0x68
    ) {
      detectedMpuAddress = address;
      return true;
    }
  }

  detectedMpuAddress = 0;
  return false;
}

bool initializeMPU() {
  if (!detectMPU()) {
    return false;
  }

  // Wake device.
  if (!writeI2CRegister(I2C_BUS_1, detectedMpuAddress, 0x6B, 0x00)) {
    return false;
  }
  delay(100);

  // DLPF enabled, approximately 44 Hz gyro bandwidth.
  writeI2CRegister(I2C_BUS_1, detectedMpuAddress, 0x1A, 0x03);

  // Gyro +/-250 deg/s.
  writeI2CRegister(I2C_BUS_1, detectedMpuAddress, 0x1B, 0x00);

  // Accelerometer +/-2 g.
  writeI2CRegister(I2C_BUS_1, detectedMpuAddress, 0x1C, 0x00);

  // Accel DLPF where supported by MPU6500/9250.
  writeI2CRegister(I2C_BUS_1, detectedMpuAddress, 0x1D, 0x03);

  previousImuMicros = micros();
  imuFilterInitialized = false;
  return true;
}

int16_t readBigEndianI16(TwoWire &bus) {
  uint8_t highByte = static_cast<uint8_t>(bus.read());
  uint8_t lowByte = static_cast<uint8_t>(bus.read());

  return static_cast<int16_t>(
    (static_cast<uint16_t>(highByte) << 8) |
    static_cast<uint16_t>(lowByte)
  );
}

bool readMPUData() {
  if (!mpuWorking) {
    return false;
  }

  // ACCEL_XOUT_H through GYRO_ZOUT_L = 14 bytes.
  I2C_BUS_1.beginTransmission(detectedMpuAddress);
  I2C_BUS_1.write(0x3B);

  if (I2C_BUS_1.endTransmission(false) != 0) {
    return false;
  }

  uint8_t bytesReceived = I2C_BUS_1.requestFrom(
    detectedMpuAddress,
    static_cast<uint8_t>(14)
  );

  if (bytesReceived != 14) {
    return false;
  }

  int16_t axRaw = readBigEndianI16(I2C_BUS_1);
  int16_t ayRaw = readBigEndianI16(I2C_BUS_1);
  int16_t azRaw = readBigEndianI16(I2C_BUS_1);

  // Temperature is not used by the vehicle app.
  (void)readBigEndianI16(I2C_BUS_1);

  int16_t gxRaw = readBigEndianI16(I2C_BUS_1);
  int16_t gyRaw = readBigEndianI16(I2C_BUS_1);
  int16_t gzRaw = readBigEndianI16(I2C_BUS_1);

  latestAxG = axRaw / ACCEL_LSB_PER_G;
  latestAyG = ayRaw / ACCEL_LSB_PER_G;
  latestAzG = azRaw / ACCEL_LSB_PER_G;

  latestGxDps = gxRaw / GYRO_LSB_PER_DPS;
  latestGyDps = gyRaw / GYRO_LSB_PER_DPS;
  latestGzDps = gzRaw / GYRO_LSB_PER_DPS;

  float accelRollDeg = atan2f(
    latestAyG,
    latestAzG
  ) * 180.0f / PI;

  float accelPitchDeg = atan2f(
    -latestAxG,
    sqrtf(latestAyG * latestAyG + latestAzG * latestAzG)
  ) * 180.0f / PI;

  unsigned long currentMicros = micros();
  float dt = (currentMicros - previousImuMicros) / 1000000.0f;
  previousImuMicros = currentMicros;

  // Protect the filter from a giant step after a pause/debugger stop.
  if (dt <= 0.0f || dt > 0.25f) {
    dt = IMU_UPDATE_INTERVAL_MS / 1000.0f;
  }

  if (!imuFilterInitialized) {
    latestRollDeg = accelRollDeg;
    latestPitchDeg = accelPitchDeg;
    latestYawDeg = 0.0f;
    imuFilterInitialized = true;
  } else {
    latestRollDeg =
      COMPLEMENTARY_ALPHA * (latestRollDeg + latestGxDps * dt) +
      (1.0f - COMPLEMENTARY_ALPHA) * accelRollDeg;

    latestPitchDeg =
      COMPLEMENTARY_ALPHA * (latestPitchDeg + latestGyDps * dt) +
      (1.0f - COMPLEMENTARY_ALPHA) * accelPitchDeg;

    // MPU6500 has no usable magnetometer heading in this configuration,
    // so yaw is gyro-integrated and can slowly drift over time.
    latestYawDeg = normalizeAngle180(
      latestYawDeg + latestGzDps * dt
    );
  }

  return true;
}

void updateIMU() {
  static unsigned long lastImuUpdateMs = 0;

  if (millis() - lastImuUpdateMs < IMU_UPDATE_INTERVAL_MS) {
    return;
  }

  lastImuUpdateMs = millis();

  if (!readMPUData()) {
    latestAxG = NAN;
    latestAyG = NAN;
    latestAzG = NAN;
    latestGxDps = NAN;
    latestGyDps = NAN;
    latestGzDps = NAN;
  }
}

// =====================================================
// ULTRASONIC / SERVO SCAN
// =====================================================

float readUltrasonicDistanceCm() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(3);

  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long echoDuration = pulseIn(
    ULTRASONIC_ECHO_PIN,
    HIGH,
    ULTRASONIC_TIMEOUT_US
  );

  if (echoDuration == 0) {
    return NAN;
  }

  return echoDuration * 0.0343f / 2.0f;
}

void initializeServoScan() {
  if (!ENABLE_SERVO_SCAN) {
    servoScanAngleDeg = 0;
    Serial.println("Ultrasonic servo scan disabled.");
    return;
  }

  sensorServo.setPeriodHertz(50);
  sensorServo.attach(
    SERVO_PIN,
    SERVO_MIN_PULSE_US,
    SERVO_MAX_PULSE_US
  );

  // Always boot centered. The servo will only begin sweeping after a fresh
  // ASSISTED or AUTONOMOUS drive command is received from the Raspberry Pi.
  servoScanPositionIndex = 1;
  servoScanAngleDeg = 0;
  sensorServo.write(SERVO_CENTER_DEG);
  delay(700);

  Serial.println("Ultrasonic servo initialized on GPIO 14.");
  Serial.println("  MANUAL: centered");
  Serial.println("  ASSISTED/AUTONOMOUS: scanning");
}

void storeScanReading(int angleDeg, float distanceCm) {
  latestInstantDistanceCm = distanceCm;

  if (isnan(distanceCm)) {
    return;
  }

  if (!ENABLE_SERVO_SCAN) {
    scanCenterCm = distanceCm;
    return;
  }

  // The scan positions pass exactly through -60, 0, and +60 degrees.
  if (angleDeg == -SERVO_SCAN_LIMIT_DEG) {
    scanLeftCm = distanceCm;
  } else if (angleDeg == 0) {
    scanCenterCm = distanceCm;
  } else if (angleDeg == SERVO_SCAN_LIMIT_DEG) {
    scanRightCm = distanceCm;
  }
}

void updateServoScan() {
  static bool wasScanning = false;

  bool shouldScan = servoScanShouldBeActive();

  // MANUAL mode, stale Pi command, or globally disabled scan:
  // keep the servo centered and continue measuring straight ahead.
  if (!shouldScan) {
    if (ENABLE_SERVO_SCAN &&
        (wasScanning || servoScanAngleDeg != 0)) {
      servoScanPositionIndex = 1;
      servoScanAngleDeg = 0;
      sensorServo.write(SERVO_CENTER_DEG);

      // Left/right values came from an earlier scan and should not be treated
      // as current while the vehicle is in centered MANUAL operation.
      scanLeftCm = NAN;
      scanRightCm = NAN;

      // Give the servo a full dwell period to settle at center before taking
      // the next forward range measurement.
      previousServoScanTime = millis();
    }

    wasScanning = false;

    if (millis() - previousServoScanTime < SERVO_POSITION_DWELL_MS) {
      return;
    }

    previousServoScanTime = millis();

    float distanceCm = readUltrasonicDistanceCm();
    storeScanReading(0, distanceCm);
    return;
  }

  if (!wasScanning) {
    wasScanning = true;
    servoScanPositionIndex = 1;
    servoScanAngleDeg = 0;
    sensorServo.write(SERVO_CENTER_DEG);
    previousServoScanTime = millis();
    return;
  }

  if (millis() - previousServoScanTime < SERVO_POSITION_DWELL_MS) {
    return;
  }

  previousServoScanTime = millis();

  // The servo has been sitting still at this position for the full dwell interval, so take the ultrasonic reading before commanding another move.
  float distanceCm = readUltrasonicDistanceCm();
  storeScanReading(servoScanAngleDeg, distanceCm);

  // Advance through CENTER -> RIGHT -> CENTER -> LEFT -> CENTER -> repeat.
  servoScanPositionIndex =
    (servoScanPositionIndex + 1) % SERVO_SCAN_POSITION_COUNT;

  servoScanAngleDeg = SERVO_SCAN_POSITIONS[servoScanPositionIndex];

  // Only one new target command is issued per dwell interval.
  sensorServo.write(SERVO_CENTER_DEG + servoScanAngleDeg);
}

// =====================================================
// RFID
// =====================================================

bool authorizedUidPresent() {
  if (rfid.uid.size != 4) {
    return false;
  }

  MFRC522::PICC_Type cardType = rfid.PICC_GetType(rfid.uid.sak);
  if (cardType != MFRC522::PICC_TYPE_MIFARE_1K) {
    return false;
  }

  for (byte i = 0; i < 4; i++) {
    if (rfid.uid.uidByte[i] != AUTHORIZED_RFID_UID[i]) {
      return false;
    }
  }

  return true;
}

void initializeRFID() {

  SPI.begin(
    RFID_SCK_PIN,
    RFID_MISO_PIN,
    RFID_MOSI_PIN,
    RFID_SS_PIN
  );

  rfid.PCD_Init(RFID_SS_PIN, RFID_RST_PIN);
  delay(50);

  byte version = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  rfidWorking = (version != 0x00 && version != 0xFF);

  Serial.print("MFRC522 version register: 0x");
  if (version < 0x10) Serial.print("0");
  Serial.println(version, HEX);

  if (rfidWorking) {
    Serial.println("RFID reader online. Authorized UID: 2D:B9:EC:44");
  } else {
    Serial.println("ERROR: MFRC522 communication failed. Vehicle remains locked.");
  }
}

void updateRFID() {
  if (!rfidWorking) {
    return;
  }

  if (millis() - lastRfidReadTime < RFID_SCAN_COOLDOWN_MS) {
    return;
  }

  if (!rfid.PICC_IsNewCardPresent()) {
    return;
  }

  if (!rfid.PICC_ReadCardSerial()) {
    return;
  }

  lastRfidReadTime = millis();

  Serial.print("RFID UID: ");
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) Serial.print("0");
    Serial.print(rfid.uid.uidByte[i], HEX);
    if (i + 1 < rfid.uid.size) Serial.print(":");
  }
  Serial.println();

  if (authorizedUidPresent()) {
    vehicleUnlocked = !vehicleUnlocked;
    Serial.println(vehicleUnlocked ?
      "AUTHORIZED RFID -> VEHICLE UNLOCKED" :
      "AUTHORIZED RFID -> VEHICLE LOCKED");
  } else {
    Serial.println("UNAUTHORIZED RFID -> ACCESS DENIED");
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// =====================================================
// LED MATRIX DIRECTION INDICATOR
// =====================================================

const uint8_t ARROW_UP[8] = {
  0b00011000,
  0b00111100,
  0b01111110,
  0b11011011,
  0b00011000,
  0b00011000,
  0b00011000,
  0b00011000
};

const uint8_t ARROW_DOWN[8] = {
  0b00011000,
  0b00011000,
  0b00011000,
  0b00011000,
  0b11011011,
  0b01111110,
  0b00111100,
  0b00011000
};

const uint8_t ARROW_LEFT[8] = {
  0b00001000,
  0b00001100,
  0b00000110,
  0b11111111,
  0b11111111,
  0b00000110,
  0b00001100,
  0b00001000
};

const uint8_t ARROW_RIGHT[8] = {
  0b00010000,
  0b00110000,
  0b01100000,
  0b11111111,
  0b11111111,
  0b01100000,
  0b00110000,
  0b00010000
};

void initializeMatrix() {
  matrix.begin();
  matrix.control(MD_MAX72XX::INTENSITY, 1);
  matrix.clear();
}

void drawMatrixBitmap(const uint8_t bitmap[8]) {
  matrix.clear();

  for (uint8_t row = 0; row < 8; row++) {
    for (uint8_t column = 0; column < 8; column++) {
      bool on = (bitmap[row] & (1 << (7 - column))) != 0;
      matrix.setPoint(row, column, on);
    }
  }
}

void updateMatrixDirection() {
  enum Direction { DIR_NONE, DIR_FORWARD, DIR_REVERSE, DIR_LEFT, DIR_RIGHT };
  static Direction lastDirection = DIR_NONE;
  static bool lastBlinkOn = false;

  Direction direction = DIR_NONE;

  if (piCommandFresh() && vehicleUnlocked && !piEstopActive) {
    int left = static_cast<int>(piDriveLeft);
    int right = static_cast<int>(piDriveRight);
    int turnDelta = right - left;
    int average = (left + right) / 2;

    // A/left command produces right > left. D/right produces left > right.
    if (turnDelta > 10) {
      direction = DIR_LEFT;
    } else if (turnDelta < -10) {
      direction = DIR_RIGHT;
    } else if (average > 5) {
      direction = DIR_FORWARD;
    } else if (average < -5) {
      direction = DIR_REVERSE;
    }
  }

  bool isTurn = (direction == DIR_LEFT || direction == DIR_RIGHT);
  bool blinkOn = !isTurn || ((millis() / 350UL) % 2UL == 0UL);

  if (direction == lastDirection && blinkOn == lastBlinkOn) {
    return;
  }

  lastDirection = direction;
  lastBlinkOn = blinkOn;

  if (!blinkOn || direction == DIR_NONE) {
    matrix.clear();
    return;
  }

  switch (direction) {
    case DIR_FORWARD: drawMatrixBitmap(ARROW_UP); break;
    case DIR_REVERSE: drawMatrixBitmap(ARROW_DOWN); break;
    case DIR_LEFT:    drawMatrixBitmap(ARROW_LEFT); break;
    case DIR_RIGHT:   drawMatrixBitmap(ARROW_RIGHT); break;
    default:          matrix.clear(); break;
  }
}

// =====================================================
// OLED FUNCTIONS
// =====================================================

void showStartupScreen(
  Adafruit_SSD1306 &display,
  const char *title,
  const char *busPins
) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(title);

  display.setTextSize(1);
  display.println();
  display.println("Sensor ECU");
  display.println("Display online");
  display.println(busPins);

  display.display();
}

void drawCenteredHeader(
  Adafruit_SSD1306 &display,
  const String &text
) {
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

  int16_t x = (SCREEN_WIDTH - static_cast<int16_t>(w)) / 2;
  if (x < 0) x = 0;

  // y=0 through y=15 maps to the yellow band on the common
  // 0.96-inch yellow/blue 128x64 OLED modules.
  display.setCursor(x, 0);
  display.print(text);
}

void updateOled1() {
  if (!oled1Working) {
    return;
  }

  oled1.clearDisplay();

  // Yellow header: current drive mode fills the top band.
  drawCenteredHeader(oled1, String(driveModeName()));

  // Blue area: three evenly spaced status rows.
  oled1.setTextSize(1);
  oled1.setTextColor(SSD1306_WHITE);

  oled1.setCursor(0, 22);
  oled1.print("Speed: ");
  oled1.print(displayedDriveSpeedPercent());
  oled1.print("%");

  oled1.setCursor(0, 37);
  oled1.print("Mode:  ");
  oled1.print(vehicleStatusName());

  oled1.setCursor(0, 52);
  oled1.print("RFID:  ");
  oled1.print(vehicleUnlocked ? "UNLOCKED" : "LOCKED");

  oled1.display();
}

void updateOled2() {
  if (!oled2Working) {
    return;
  }

  oled2.clearDisplay();

  bool batteryFresh = batteryMessageReceived &&
                      (millis() - lastBatteryCanMessageTime <= BATTERY_CAN_TIMEOUT_MS);

  int batteryPercent = -1;
  if (batteryFresh) {
    batteryPercent = estimateBatteryPercent(canBatteryVoltageV);
  }

  // Yellow header: battery percentage fills the top band.
  String batteryHeader = "BAT ";
  if (batteryPercent < 0) {
    batteryHeader += "--%";
  } else {
    batteryHeader += String(batteryPercent);
    batteryHeader += "%";
  }
  drawCenteredHeader(oled2, batteryHeader);

  // Blue area: voltage, current and temperature.
  oled2.setTextSize(1);
  oled2.setTextColor(SSD1306_WHITE);

  oled2.setCursor(0, 22);
  oled2.print("Voltage: ");
  if (!batteryFresh || isnan(canBatteryVoltageV)) {
    oled2.print("N/A");
  } else {
    oled2.print(canBatteryVoltageV, 2);
    oled2.print(" V");
  }

  oled2.setCursor(0, 37);
  oled2.print("Current: ");
  if (!batteryFresh || isnan(canBatteryCurrentMa)) {
    oled2.print("N/A");
  } else {
    oled2.print(canBatteryCurrentMa / 1000.0f, 2);
    oled2.print(" A");
  }

  oled2.setCursor(0, 52);
  oled2.print("Temp:    ");
  if (!batteryFresh || isnan(canBatteryTemperatureC)) {
    oled2.print("N/A");
  } else {
    oled2.print(canBatteryTemperatureC, 1);
    oled2.print(" C");
  }

  oled2.display();
}

void updateDisplays() {
  if (millis() - previousDisplayTime < DISPLAY_UPDATE_INTERVAL_MS) {
    return;
  }

  previousDisplayTime = millis();
  updateOled1();
  updateOled2();
}

// =====================================================
// CAN / TWAI
// =====================================================

void initializeCan() {
  twai_general_config_t generalConfig = TWAI_GENERAL_CONFIG_DEFAULT(
    CAN_TX_PIN,
    CAN_RX_PIN,
    TWAI_MODE_NORMAL
  );

  // Five Sensor ECU frames are sent per telemetry burst.
  // A larger queue gives the burst room if the bus is briefly busy.
  generalConfig.tx_queue_len = 20;

  twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t installResult = twai_driver_install(
    &generalConfig,
    &timingConfig,
    &filterConfig
  );

  if (installResult != ESP_OK) {
    Serial.print("CAN driver install failed. Error: ");
    Serial.println(static_cast<int>(installResult));
    canWorking = false;
    return;
  }

  esp_err_t startResult = twai_start();

  if (startResult != ESP_OK) {
    Serial.print("CAN driver start failed. Error: ");
    Serial.println(static_cast<int>(startResult));
    twai_driver_uninstall();
    canWorking = false;
    return;
  }

  canWorking = true;

  Serial.println("CAN/TWAI started at 500 kbps.");
  Serial.print("CAN TX GPIO: ");
  Serial.println(static_cast<int>(CAN_TX_PIN));
  Serial.print("CAN RX GPIO: ");
  Serial.println(static_cast<int>(CAN_RX_PIN));
  Serial.println("TX IDs: 0x200 0x201 0x202 0x203 0x204");
  Serial.println("RX Battery ID for OLED: 0x100");
}

bool sendCanFrame(uint32_t identifier, const uint8_t data[8]) {
  if (!canWorking) {
    return false;
  }

  twai_message_t message = {};
  message.identifier = identifier;
  message.extd = 0;
  message.rtr = 0;
  message.data_length_code = 8;

  for (int i = 0; i < 8; i++) {
    message.data[i] = data[i];
  }

  esp_err_t result = twai_transmit(&message, 0);

  if (result != ESP_OK) {
    canTxFailureCount++;
    return false;
  }

  return true;
}

void sendSensorStatusFrame(uint16_t counter) {
  uint8_t data[8] = {0};

  putU16LE(data, 0, centimetersToMillimetersU16(scanCenterCm));

  int16_t servoDeciDeg = ENABLE_SERVO_SCAN
    ? static_cast<int16_t>(servoScanAngleDeg * 10)
    : 0;

  putI16LE(data, 2, servoDeciDeg);

  // Byte 4 flags.
  if (vehicleUnlocked) {
    data[4] |= 0x01;
  }

  // Byte 5 sensor status.
  // bit0 = center ultrasonic reading valid
  // bit1 = MPU connected
  // bit2 = servo scan enabled
  // bit3 = CAN initialized
  // bit4 = RFID reader connected
  // bit7 = Sensor ECU is authoritative for vehicle lock state
  if (!isnan(scanCenterCm)) data[5] |= 0x01;
  if (mpuWorking) data[5] |= 0x02;
  if (ENABLE_SERVO_SCAN) data[5] |= 0x04;
  if (canWorking) data[5] |= 0x08;
  if (rfidWorking) data[5] |= 0x10;

  data[5] |= 0x80;

  putU16LE(data, 6, counter);
  sendCanFrame(SENSOR_STATUS_CAN_ID, data);
}

void sendSensorAccelFrame(uint16_t counter) {
  uint8_t data[8] = {0};

  // milli-g
  putI16LE(data, 0, floatToScaledI16(latestAxG, 1000.0f));
  putI16LE(data, 2, floatToScaledI16(latestAyG, 1000.0f));
  putI16LE(data, 4, floatToScaledI16(latestAzG, 1000.0f));
  putU16LE(data, 6, counter);

  sendCanFrame(SENSOR_ACCEL_CAN_ID, data);
}

void sendSensorGyroFrame(uint16_t counter) {
  uint8_t data[8] = {0};

  // 0.01 deg/s
  putI16LE(data, 0, floatToScaledI16(latestGxDps, 100.0f));
  putI16LE(data, 2, floatToScaledI16(latestGyDps, 100.0f));
  putI16LE(data, 4, floatToScaledI16(latestGzDps, 100.0f));
  putU16LE(data, 6, counter);

  sendCanFrame(SENSOR_GYRO_CAN_ID, data);
}

void sendSensorOrientationFrame(uint16_t counter) {
  uint8_t data[8] = {0};

  // 0.01 degree.
  // Yaw is intentionally normalized to -180..+180 before packing because
  // signed int16 at 0.01 degree cannot directly represent 327.68..359.99.
  putI16LE(data, 0, floatToScaledI16(latestRollDeg, 100.0f));
  putI16LE(data, 2, floatToScaledI16(latestPitchDeg, 100.0f));
  putI16LE(
    data,
    4,
    floatToScaledI16(normalizeAngle180(latestYawDeg), 100.0f)
  );
  putU16LE(data, 6, counter);

  sendCanFrame(SENSOR_ORIENTATION_CAN_ID, data);
}

void sendSensorScanFrame(uint16_t counter) {
  uint8_t data[8] = {0};

  putU16LE(data, 0, centimetersToMillimetersU16(scanLeftCm));
  putU16LE(data, 2, centimetersToMillimetersU16(scanCenterCm));
  putU16LE(data, 4, centimetersToMillimetersU16(scanRightCm));
  putU16LE(data, 6, counter);

  sendCanFrame(SENSOR_SCAN_CAN_ID, data);
}

void sendSensorCanBurst() {
  if (!canWorking) {
    return;
  }

  if (millis() - previousCanTxTime < CAN_TX_INTERVAL_MS) {
    return;
  }

  previousCanTxTime = millis();

  uint16_t counter = sensorCanCounter;

  sendSensorStatusFrame(counter);
  sendSensorAccelFrame(counter);
  sendSensorGyroFrame(counter);
  sendSensorOrientationFrame(counter);
  sendSensorScanFrame(counter);

  sensorCanCounter++;
}

void processCanMessages() {
  if (!canWorking) {
    return;
  }

  twai_message_t message;

  // Drain the receive queue without blocking the sensor loop.
  while (twai_receive(&message, 0) == ESP_OK) {
    if (message.extd || message.rtr || message.data_length_code != 8) {
      continue;
    }

    if (message.identifier == BATTERY_STATUS_CAN_ID) {
      uint16_t voltageMv =
        static_cast<uint16_t>(message.data[0]) |
        (static_cast<uint16_t>(message.data[1]) << 8);

      int16_t temperatureCentiC = static_cast<int16_t>(
        static_cast<uint16_t>(message.data[2]) |
        (static_cast<uint16_t>(message.data[3]) << 8)
      );

      int16_t currentMa = static_cast<int16_t>(
        static_cast<uint16_t>(message.data[4]) |
        (static_cast<uint16_t>(message.data[5]) << 8)
      );

      batteryCanMessageCounter =
        static_cast<uint16_t>(message.data[6]) |
        (static_cast<uint16_t>(message.data[7]) << 8);

      canBatteryVoltageV = (voltageMv == UINT16_MAX)
        ? NAN
        : voltageMv / 1000.0f;

      canBatteryTemperatureC = (temperatureCentiC == INT16_MIN)
        ? NAN
        : temperatureCentiC / 100.0f;

      canBatteryCurrentMa = (currentMa == INT16_MIN)
        ? NAN
        : static_cast<float>(currentMa);

      batteryMessageReceived = true;
      lastBatteryCanMessageTime = millis();
      continue;
    }

    if (message.identifier == PI_DRIVE_COMMAND_CAN_ID) {
      piDriveLeft = decodeI8(message.data[0]);
      piDriveRight = decodeI8(message.data[1]);
      piDriveMode = message.data[2];
      piSpeedLimit = message.data[3];
      piEstopActive = (message.data[4] & 0x01) != 0;
      piSafetyFlags = message.data[5];
      piDriveCounter =
        static_cast<uint16_t>(message.data[6]) |
        (static_cast<uint16_t>(message.data[7]) << 8);

      piDriveCommandReceived = true;
      lastPiDriveCommandTime = millis();
      continue;
    }

    if (message.identifier == PI_LOCK_COMMAND_CAN_ID) {
      // byte 0: 0 = LOCK, 1 = UNLOCK
      // byte 1: source, 1 = browser
      // bytes 2-5: reserved
      // bytes 6-7: rolling uint16 command counter
      if (message.data[0] <= 1) {
        bool requestedUnlocked = (message.data[0] == 1);

        piLockCommandCounter =
          static_cast<uint16_t>(message.data[6]) |
          (static_cast<uint16_t>(message.data[7]) << 8);

        piLockCommandReceived = true;
        lastPiLockCommandTime = millis();

        if (vehicleUnlocked != requestedUnlocked) {
          vehicleUnlocked = requestedUnlocked;
          Serial.println(vehicleUnlocked ?
            "BROWSER COMMAND -> VEHICLE UNLOCKED" :
            "BROWSER COMMAND -> VEHICLE LOCKED");
        }
      }

      continue;
    }
  }
}

// =====================================================
// SERIAL STATUS
// =====================================================

void printSerialStatus() {
  if (millis() - previousSerialStatusTime < SERIAL_STATUS_INTERVAL_MS) {
    return;
  }

  previousSerialStatusTime = millis();

  Serial.println("------------------------------");

  Serial.print("Center range: ");
  if (isnan(scanCenterCm)) Serial.println("N/A");
  else {
    Serial.print(scanCenterCm, 1);
    Serial.println(" cm");
  }

  Serial.print("Scan L/C/R: ");
  if (isnan(scanLeftCm)) Serial.print("N/A");
  else Serial.print(scanLeftCm, 1);

  Serial.print(" / ");
  if (isnan(scanCenterCm)) Serial.print("N/A");
  else Serial.print(scanCenterCm, 1);

  Serial.print(" / ");
  if (isnan(scanRightCm)) Serial.println("N/A");
  else {
    Serial.print(scanRightCm, 1);
    Serial.println(" cm");
  }

  Serial.print("Servo angle: ");
  Serial.print(servoScanAngleDeg);
  Serial.println(" deg");

  Serial.print("Servo behavior: ");
  if (!ENABLE_SERVO_SCAN) {
    Serial.println("DISABLED");
  } else if (servoScanShouldBeActive()) {
    Serial.println("SCANNING");
  } else {
    Serial.println("CENTERED");
  }

  Serial.print("IMU: ");
  Serial.println(mpuWorking ? "ONLINE" : "FAILED");

  if (mpuWorking && !isnan(latestAxG)) {
    Serial.print("  Accel g: ");
    Serial.print(latestAxG, 3);
    Serial.print(", ");
    Serial.print(latestAyG, 3);
    Serial.print(", ");
    Serial.println(latestAzG, 3);

    Serial.print("  Gyro dps: ");
    Serial.print(latestGxDps, 2);
    Serial.print(", ");
    Serial.print(latestGyDps, 2);
    Serial.print(", ");
    Serial.println(latestGzDps, 2);

    Serial.print("  R/P/Y: ");
    Serial.print(latestRollDeg, 1);
    Serial.print(", ");
    Serial.print(latestPitchDeg, 1);
    Serial.print(", ");
    Serial.println(latestYawDeg, 1);
  }

  Serial.print("Sensor CAN burst #: ");
  Serial.println(sensorCanCounter);

  Serial.print("CAN TX failures: ");
  Serial.println(canTxFailureCount);

  Serial.print("Battery ECU: ");
  bool batteryFresh = batteryMessageReceived &&
                      (millis() - lastBatteryCanMessageTime <= BATTERY_CAN_TIMEOUT_MS);
  Serial.println(batteryFresh ? "ONLINE" : "WAITING");

  Serial.print("RFID: ");
  if (!rfidWorking) Serial.println("READER FAILED / LOCKED");
  else Serial.println(vehicleUnlocked ? "UNLOCKED" : "LOCKED");

  Serial.print("Pi command: ");
  Serial.println(piCommandFresh() ? "ONLINE" : "STALE");
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("================================");
  Serial.println("LGARI SENSOR ECU");
  Serial.println("================================");

  // Start both independent ESP32 I2C controllers.
  I2C_BUS_1.begin(I2C1_SDA, I2C1_SCL, 100000);
  I2C_BUS_2.begin(I2C2_SDA, I2C2_SCL, 100000);

  Serial.println("I2C Bus 1: SDA 21 / SCL 22");
  Serial.println("I2C Bus 2: SDA 26 / SCL 25");

  scanI2CBus(I2C_BUS_1, "I2C Bus 1");
  scanI2CBus(I2C_BUS_2, "I2C Bus 2");

  // OLED #1
  oled1Working = oled1.begin(
    SSD1306_SWITCHCAPVCC,
    OLED1_ADDRESS
  );

  if (oled1Working) {
    Serial.println("OLED #1 initialized.");
    showStartupScreen(oled1, "OLED 1", "SDA21 SCL22");
  } else {
    Serial.println("ERROR: OLED #1 failed.");
  }

  // OLED #2
  oled2Working = oled2.begin(
    SSD1306_SWITCHCAPVCC,
    OLED2_ADDRESS
  );

  if (oled2Working) {
    Serial.println("OLED #2 initialized.");
    showStartupScreen(oled2, "OLED 2", "SDA26 SCL25");
  } else {
    Serial.println("ERROR: OLED #2 failed.");
  }

  // IMU
  mpuWorking = initializeMPU();

  if (mpuWorking) {
    Serial.print("MPU initialized at 0x");
    Serial.println(detectedMpuAddress, HEX);
  } else {
    Serial.println("ERROR: MPU not detected/configured.");
  }

  // Ultrasonic
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  // Matrix direction indicator
  initializeMatrix();
  Serial.println("MAX7219 direction matrix initialized.");

  // Ultrasonic scan servo
  initializeServoScan();

  // Vehicle is locked by default. RFID and browser commands may both change the same state.
  Serial.println("RFID wiring:");
  Serial.println("  SDA / SS -> GPIO 19");
  Serial.println("  SCK      -> GPIO 33");
  Serial.println("  MOSI     -> GPIO 13");
  Serial.println("  MISO     -> GPIO 27");
  Serial.println("  IRQ      -> NOT CONNECTED");
  Serial.println("  RST      -> GPIO 32");
  Serial.println();

  initializeRFID();

  // CAN
  initializeCan();

  // Prime sensor values before normal CAN traffic begins
  if (mpuWorking) {
    readMPUData();
  }

  float firstDistance = readUltrasonicDistanceCm();

  // Servo boots centered, so the first ultrasonic sample is the forward
  // center measurement regardless of the eventual drive mode.
  storeScanReading(0, firstDistance);

  previousServoScanTime = millis();
  previousCanTxTime = millis();

  Serial.println();
  Serial.println("Setup complete.");
  Serial.println("Sensor ECU telemetry active.");
  Serial.println();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  processCanMessages();
  updateRFID();

  updateIMU();
  updateServoScan();
  updateMatrixDirection();

  sendSensorCanBurst();

  updateDisplays();
  printSerialStatus();
}