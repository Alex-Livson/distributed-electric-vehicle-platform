/*
  CAN bus:
    500 kbit/s, standard 11-bit IDs

  Raspberry Pi -> Motor ECU, CAN 0x400:
    byte 0: left motor command, signed int8, -100..100 %
    byte 1: right motor command, signed int8, -100..100 %
    byte 2: drive mode
            0 = MANUAL
            1 = ASSISTED
            2 = AUTONOMOUS
    byte 3: speed limit, uint8, 0..100 %
    byte 4: flags
            bit0 = E-stop active
            bit1 = vehicle unlocked
    byte 5: safety flags
            bit0 = wall intervention active
            bit1 = current intervention active
            bit2 = overcurrent fault active
    byte 6-7: rolling uint16 command counter, little-endian

  Motor ECU -> Raspberry Pi, CAN 0x300:
    byte 0: applied left output, signed int8 %
    byte 1: applied right output, signed int8 %
    byte 2: motor driver enabled, 0/1
    byte 3: Motor ECU fault flags
            bit0 = Pi command timeout / no command
            bit1 = E-stop active
            bit2 = vehicle locked
            bit3 = overcurrent fault
            bit4 = CAN driver unavailable
    byte 4-5: rolling uint16 status counter, little-endian
    byte 6-7: reserved
*/

#include <Arduino.h>
#include <driver/twai.h>
#include <stdint.h>

// ============================================================
// PIN ASSIGNMENTS
// ============================================================

// Status outputs
const int LED_PIN    = 16;
const int BUZZER_PIN = 17;

// TB6612FNG
// Motor A = LEFT motor
const int PWMA_PIN = 13;
const int AIN2_PIN = 14;
const int AIN1_PIN = 27;
const int STBY_PIN = 26;

// Motor B = RIGHT motor
const int BIN1_PIN = 25;
const int BIN2_PIN = 33;
const int PWMB_PIN = 32;

// SN65HVD230 CAN transceiver
const gpio_num_t CAN_TX_PIN = GPIO_NUM_23;
const gpio_num_t CAN_RX_PIN = GPIO_NUM_22;

// ============================================================
// CAN PROTOCOL
// ============================================================

const uint32_t MOTOR_STATUS_CAN_ID     = 0x300;
const uint32_t PI_DRIVE_COMMAND_CAN_ID = 0x400;

const unsigned long MOTOR_STATUS_INTERVAL_MS = 100;

const unsigned long PI_COMMAND_TIMEOUT_MS = 500;

// ============================================================
// PWM / MOTOR SETTINGS
// ============================================================

const int MOTOR_PWM_FREQUENCY = 20000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX = 255;

// Normal commands are ramped to reduce abrupt starts and reversals.
// 10 percentage points every 20 ms:
//   0 -> 50% in about 100 ms
//   0 -> 100% in about 200 ms
// Safety stops are always immediate.
const unsigned long MOTOR_RAMP_INTERVAL_MS = 20;
const int MOTOR_RAMP_STEP_PERCENT = 10;

// ============================================================
// CAN / COMMAND STATE
// ============================================================

bool canWorking = false;

bool piCommandReceived = false;
unsigned long lastPiCommandTime = 0;

int8_t requestedLeftPercent = 0;
int8_t requestedRightPercent = 0;

int appliedLeftPercent = 0;
int appliedRightPercent = 0;

uint8_t piDriveMode = 1;
uint8_t piSpeedLimit = 0;

bool piEstopActive = false;
bool vehicleUnlocked = false;

bool wallIntervention = false;
bool currentIntervention = false;
bool overcurrentFault = false;

uint16_t piCommandCounter = 0;
uint16_t motorStatusCounter = 0;

bool motorDriverEnabled = false;

unsigned long previousMotorRampTime = 0;
unsigned long previousStatusTxTime = 0;
unsigned long previousSerialTime = 0;

bool previousEstopActive = false;
bool previousOvercurrentFault = false;

bool buzzerActive = false;
unsigned long buzzerStartTime = 0;
unsigned long buzzerDurationMs = 0;

// ============================================================
// SMALL HELPERS
// ============================================================

int clampPercent(int value) {
  if (value > 100) return 100;
  if (value < -100) return -100;
  return value;
}

int decodeI8(uint8_t value) {
  return (value > 127)
    ? static_cast<int>(value) - 256
    : static_cast<int>(value);
}

uint8_t encodeI8(int value) {
  value = clampPercent(value);
  return static_cast<uint8_t>(static_cast<int8_t>(value));
}

void putU16LE(uint8_t data[8], int offset, uint16_t value) {
  data[offset]     = static_cast<uint8_t>(value & 0xFF);
  data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

bool piCommandFresh() {
  if (!piCommandReceived) {
    return false;
  }

  return (millis() - lastPiCommandTime) <= PI_COMMAND_TIMEOUT_MS;
}

const char* driveModeName() {
  switch (piDriveMode) {
    case 0: return "MANUAL";
    case 1: return "ASSISTED";
    case 2: return "AUTONOMOUS";
    default: return "UNKNOWN";
  }
}

// ============================================================
// BUZZER
// ============================================================
//
// Active buzzer:
//   GPIO17 HIGH = buzzer ON
//   GPIO17 LOW  = buzzer OFF
//
// No PWM/tone generation is needed because the active buzzer contains
// its own oscillator.

void silenceBuzzer() {
  buzzerActive = false;
  buzzerDurationMs = 0;

  digitalWrite(BUZZER_PIN, LOW);
}

void startBuzzer(unsigned long durationMs) {
  buzzerActive = true;
  buzzerStartTime = millis();
  buzzerDurationMs = durationMs;

  digitalWrite(BUZZER_PIN, HIGH);
}

void updateBuzzer() {
  if (!buzzerActive) {
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  if (millis() - buzzerStartTime >= buzzerDurationMs) {
    silenceBuzzer();
  }
}

// ============================================================
// TB6612FNG MOTOR OUTPUT
// ============================================================

int percentToPwm(int percent) {
  int magnitude = abs(clampPercent(percent));
  return (magnitude * PWM_MAX) / 100;
}

void writeMotorA(int percent) {
  percent = clampPercent(percent);
  int pwm = percentToPwm(percent);

  if (percent > 0) {
    // Preserve the verified bench-test Motor A forward direction.
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
    ledcWrite(PWMA_PIN, pwm);
  }
  else if (percent < 0) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
    ledcWrite(PWMA_PIN, pwm);
  }
  else {
    ledcWrite(PWMA_PIN, 0);
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, LOW);
  }
}

void writeMotorB(int percent) {
  percent = clampPercent(percent);
  int pwm = percentToPwm(percent);

  if (percent > 0) {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
    ledcWrite(PWMB_PIN, pwm);
  }
  else if (percent < 0) {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
    ledcWrite(PWMB_PIN, pwm);
  }
  else {
    ledcWrite(PWMB_PIN, 0);
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, LOW);
  }
}

void setMotorDriverEnabled(bool enabled) {
  motorDriverEnabled = enabled;
  digitalWrite(STBY_PIN, enabled ? HIGH : LOW);
}

void applyMotorOutputs() {
  if (!motorDriverEnabled) {
    writeMotorA(0);
    writeMotorB(0);
    return;
  }

  writeMotorA(appliedLeftPercent);
  writeMotorB(appliedRightPercent);
}

void hardStopMotors() {
  appliedLeftPercent = 0;
  appliedRightPercent = 0;

  writeMotorA(0);
  writeMotorB(0);

  setMotorDriverEnabled(false);
}

int stepToward(int currentValue, int targetValue, int maxStep) {
  if (currentValue < targetValue) {
    currentValue += maxStep;
    if (currentValue > targetValue) {
      currentValue = targetValue;
    }
  }
  else if (currentValue > targetValue) {
    currentValue -= maxStep;
    if (currentValue < targetValue) {
      currentValue = targetValue;
    }
  }

  return currentValue;
}

bool hardStopRequired() {
  if (!canWorking) {
    return true;
  }

  if (!piCommandFresh()) {
    return true;
  }

  if (piEstopActive) {
    return true;
  }

  if (!vehicleUnlocked) {
    return true;
  }

  if (overcurrentFault) {
    return true;
  }

  return false;
}

void updateMotorControl() {
  // Safety conditions never ramp down. They stop immediately and disable STBY.
  if (hardStopRequired()) {
    hardStopMotors();
    return;
  }

  // Pi already safety-filters the commands, but locally enforce its speed-limit
  // field as a second sanity check before touching the motor driver.
  int localLimit = constrain(static_cast<int>(piSpeedLimit), 0, 100);

  int targetLeft = constrain(
    static_cast<int>(requestedLeftPercent),
    -localLimit,
    localLimit
  );

  int targetRight = constrain(
    static_cast<int>(requestedRightPercent),
    -localLimit,
    localLimit
  );

  setMotorDriverEnabled(true);

  if (millis() - previousMotorRampTime < MOTOR_RAMP_INTERVAL_MS) {
    return;
  }

  previousMotorRampTime = millis();

  appliedLeftPercent = stepToward(
    appliedLeftPercent,
    targetLeft,
    MOTOR_RAMP_STEP_PERCENT
  );

  appliedRightPercent = stepToward(
    appliedRightPercent,
    targetRight,
    MOTOR_RAMP_STEP_PERCENT
  );

  applyMotorOutputs();
}

// ============================================================
// STATUS LED
// ============================================================

void updateStatusLed() {
  unsigned long t = millis();

  if (!canWorking) {
    // Very fast blink = CAN initialization problem.
    digitalWrite(LED_PIN, ((t / 100) % 2) ? HIGH : LOW);
    return;
  }

  if (!piCommandFresh()) {
    // Fast blink = Pi command lost.
    digitalWrite(LED_PIN, ((t / 150) % 2) ? HIGH : LOW);
    return;
  }

  if (piEstopActive || overcurrentFault) {
    // Fast warning blink.
    digitalWrite(LED_PIN, ((t / 100) % 2) ? HIGH : LOW);
    return;
  }

  if (!vehicleUnlocked) {
    // Slow blink = healthy CAN, vehicle intentionally locked.
    digitalWrite(LED_PIN, ((t / 500) % 2) ? HIGH : LOW);
    return;
  }

  // Solid = CAN healthy, command fresh, vehicle unlocked.
  digitalWrite(LED_PIN, HIGH);
}

// ============================================================
// CAN / TWAI
// ============================================================

void initializeCan() {
  twai_general_config_t generalConfig = TWAI_GENERAL_CONFIG_DEFAULT(
    CAN_TX_PIN,
    CAN_RX_PIN,
    TWAI_MODE_NORMAL
  );

  generalConfig.tx_queue_len = 10;
  generalConfig.rx_queue_len = 20;

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
  Serial.println("RX: 0x400 Pi drive command");
  Serial.println("TX: 0x300 Motor ECU status");
}

void processCanMessages() {
  if (!canWorking) {
    return;
  }

  twai_message_t message;

  while (twai_receive(&message, 0) == ESP_OK) {
    if (message.extd || message.rtr) {
      continue;
    }

    if (message.identifier != PI_DRIVE_COMMAND_CAN_ID) {
      continue;
    }

    if (message.data_length_code != 8) {
      continue;
    }

    bool oldEstop = piEstopActive;
    bool oldOvercurrentFault = overcurrentFault;

    requestedLeftPercent = static_cast<int8_t>(
      clampPercent(decodeI8(message.data[0]))
    );

    requestedRightPercent = static_cast<int8_t>(
      clampPercent(decodeI8(message.data[1]))
    );

    piDriveMode = message.data[2];
    piSpeedLimit = constrain(static_cast<int>(message.data[3]), 0, 100);

    piEstopActive = (message.data[4] & 0x01) != 0;
    vehicleUnlocked = (message.data[4] & 0x02) != 0;

    wallIntervention = (message.data[5] & 0x01) != 0;
    currentIntervention = (message.data[5] & 0x02) != 0;
    overcurrentFault = (message.data[5] & 0x04) != 0;

    piCommandCounter =
      static_cast<uint16_t>(message.data[6]) |
      (static_cast<uint16_t>(message.data[7]) << 8);

    piCommandReceived = true;
    lastPiCommandTime = millis();

    // Short local audible warning when a hard Pi safety condition first appears.
    if (!oldEstop && piEstopActive) {
      startBuzzer(250);
    }

    if (!oldOvercurrentFault && overcurrentFault) {
      startBuzzer(250);
    }
  }
}

uint8_t motorFaultFlags() {
  uint8_t flags = 0;

  if (!piCommandFresh()) flags |= 0x01;
  if (piEstopActive) flags |= 0x02;
  if (!vehicleUnlocked) flags |= 0x04;
  if (overcurrentFault) flags |= 0x08;
  if (!canWorking) flags |= 0x10;

  return flags;
}

bool sendMotorStatusFrame() {
  if (!canWorking) {
    return false;
  }

  uint8_t data[8] = {0};

  data[0] = encodeI8(appliedLeftPercent);
  data[1] = encodeI8(appliedRightPercent);
  data[2] = motorDriverEnabled ? 1 : 0;
  data[3] = motorFaultFlags();

  putU16LE(data, 4, motorStatusCounter);

  twai_message_t message = {};
  message.identifier = MOTOR_STATUS_CAN_ID;
  message.extd = 0;
  message.rtr = 0;
  message.data_length_code = 8;

  for (int i = 0; i < 8; i++) {
    message.data[i] = data[i];
  }

  esp_err_t result = twai_transmit(&message, 0);

  if (result == ESP_OK) {
    motorStatusCounter++;
    return true;
  }

  return false;
}

void updateMotorStatusCan() {
  if (millis() - previousStatusTxTime < MOTOR_STATUS_INTERVAL_MS) {
    return;
  }

  previousStatusTxTime = millis();
  sendMotorStatusFrame();
}

// ============================================================
// SERIAL DIAGNOSTICS
// ============================================================

void printSerialStatus() {
  if (millis() - previousSerialTime < 1000) {
    return;
  }

  previousSerialTime = millis();

  Serial.println();
  Serial.println("----- MOTOR ECU STATUS -----");

  Serial.print("CAN: ");
  Serial.println(canWorking ? "ONLINE" : "ERROR");

  Serial.print("Pi command: ");
  if (!piCommandReceived) {
    Serial.println("NEVER RECEIVED");
  } else {
    Serial.print(piCommandFresh() ? "FRESH" : "TIMEOUT");
    Serial.print("  age=");
    Serial.print(millis() - lastPiCommandTime);
    Serial.println(" ms");
  }

  Serial.print("Mode: ");
  Serial.println(driveModeName());

  Serial.print("Unlocked: ");
  Serial.println(vehicleUnlocked ? "YES" : "NO");

  Serial.print("E-stop: ");
  Serial.println(piEstopActive ? "ACTIVE" : "CLEAR");

  Serial.print("Wall intervention: ");
  Serial.println(wallIntervention ? "ACTIVE" : "OFF");

  Serial.print("Current intervention: ");
  Serial.println(currentIntervention ? "ACTIVE" : "OFF");

  Serial.print("Overcurrent fault: ");
  Serial.println(overcurrentFault ? "ACTIVE" : "CLEAR");

  Serial.print("Requested L/R: ");
  Serial.print(static_cast<int>(requestedLeftPercent));
  Serial.print("% / ");
  Serial.print(static_cast<int>(requestedRightPercent));
  Serial.println("%");

  Serial.print("Applied L/R: ");
  Serial.print(appliedLeftPercent);
  Serial.print("% / ");
  Serial.print(appliedRightPercent);
  Serial.println("%");

  Serial.print("Driver: ");
  Serial.println(motorDriverEnabled ? "ENABLED" : "STANDBY");

  Serial.print("Pi command counter: ");
  Serial.println(piCommandCounter);

  Serial.print("Motor status counter: ");
  Serial.println(motorStatusCounter);

  Serial.print("Fault flags: 0x");
  Serial.println(motorFaultFlags(), HEX);
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LED_PIN, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);

  ledcAttach(PWMA_PIN, MOTOR_PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(PWMB_PIN, MOTOR_PWM_FREQUENCY, PWM_RESOLUTION);

  // Safe boot state. The Motor ECU never moves until it has:
  //   1. working CAN,
  //   2. a fresh Pi command,
  //   3. vehicle unlocked,
  //   4. no E-stop,
  //   5. no overcurrent fault.
  digitalWrite(LED_PIN, LOW);
  silenceBuzzer();
  hardStopMotors();

  Serial.println();
  Serial.println("================================");
  Serial.println("       LGARI MOTOR ECU");
  Serial.println("================================");
  Serial.println("Vehicle CAN firmware");
  Serial.println();

  Serial.println("Motor driver wiring:");
  Serial.println("  LEFT / Motor A:");
  Serial.println("    PWMA -> GPIO 13");
  Serial.println("    AIN2 -> GPIO 14");
  Serial.println("    AIN1 -> GPIO 27");
  Serial.println("  STBY   -> GPIO 26");
  Serial.println("  RIGHT / Motor B:");
  Serial.println("    BIN1 -> GPIO 25");
  Serial.println("    BIN2 -> GPIO 33");
  Serial.println("    PWMB -> GPIO 32");
  Serial.println();

  Serial.println("Status outputs:");
  Serial.println("  LED    -> GPIO 16");
  Serial.println("  Buzzer -> GPIO 17");
  Serial.println();

  Serial.println("CAN wiring:");
  Serial.println("  TX -> GPIO 23");
  Serial.println("  RX -> GPIO 22");
  Serial.println();

  initializeCan();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  processCanMessages();

  updateMotorControl();
  updateMotorStatusCan();

  updateStatusLed();
  updateBuzzer();

  printSerialStatus();
}