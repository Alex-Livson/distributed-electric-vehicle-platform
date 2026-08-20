#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA228.h>
#include <math.h>
#include "driver/twai.h"

// ============================================================
// BATTERY ECU SETTINGS
// ============================================================

// Enable or disable the INA228.
const bool ENABLE_CURRENT_SENSOR = true;

const bool VBUS_CONNECTED = true;

// ESP32 pin assignments
const int STATUS_LED_PIN = 13;
const int THERMISTOR_PIN = 36;

// PCB wiring:
// SDA -> GPIO 21
// SCL -> GPIO 3

const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 3;

const gpio_num_t CAN_TX_PIN = GPIO_NUM_18;
const gpio_num_t CAN_RX_PIN = GPIO_NUM_19;

// ============================================================
// CAN PROTOCOL
// ============================================================

// Battery ECU -> Raspberry Pi / Sensor ECU
// 0x100, 8 data bytes:
//   [0..1] pack voltage in mV, uint16 little-endian
//   [2..3] battery temperature in 0.01 C, int16 little-endian
//   [4..5] pack current in mA, int16 little-endian
//   [6..7] rolling message counter, uint16 little-endian
const uint32_t BATTERY_STATUS_CAN_ID = 0x100;

// Raspberry Pi -> all ECUs
// 0x400, 8 data bytes:
//   byte 0: left motor command, signed int8
//   byte 1: right motor command, signed int8
//   byte 2: drive mode
//   byte 3: speed limit
//   byte 4 flags:
//     bit0 = E-stop active
//     bit1 = vehicle unlocked
//   byte 5 safety flags:
//     bit0 = wall intervention
//     bit1 = current intervention
//     bit2 = overcurrent fault
//   bytes 6-7: rolling command counter
const uint32_t PI_DRIVE_COMMAND_CAN_ID = 0x400;

// Match the Motor ECU's Pi command timeout.
const unsigned long PI_COMMAND_TIMEOUT_MS = 500;

const float SERIES_RESISTOR_OHMS = 10000.0;
const float NOMINAL_RESISTANCE_OHMS = 10000.0;
const float NOMINAL_TEMPERATURE_C = 25.0;
const float BETA_COEFFICIENT = 3950.0;

const float ADC_MAX = 4095.0;
const float ADC_REFERENCE_VOLTAGE = 3.3;
const int THERMISTOR_SAMPLES = 20;

// False matches: 3.3V -> 10k resistor -> thermistor -> GND.
const bool THERMISTOR_CONNECTED_TO_3V3 = false;

// ============================================================
// INA228 CURRENT SENSOR SETTINGS
// ============================================================

Adafruit_INA228 ina228;
bool ina228Available = false;

const float INA228_SHUNT_RESISTANCE_OHMS = 0.002;

const float INA228_MAX_EXPECTED_CURRENT_A = 10.0;

const uint8_t INA228_I2C_ADDRESS = 0x40;

// Latest values retained for the CAN message.
float latestBatteryTemperatureC = NAN;
float latestBusVoltageV = NAN;
float latestCurrentMa = NAN;
float latestPowerMw = NAN;

// ============================================================
// TIMING / CAN STATE
// ============================================================

unsigned long previousSensorTime = 0;
const unsigned long SENSOR_INTERVAL_MS = 500;

unsigned long previousCanTime = 0;
const unsigned long CAN_INTERVAL_MS = 100;

uint16_t canMessageCounter = 0;
bool canAvailable = false;

bool piCommandReceived = false;
unsigned long lastPiCommandTime = 0;

bool piEstopActive = false;
bool vehicleUnlocked = false;
bool overcurrentFault = false;

uint16_t piCommandCounter = 0;

// ============================================================
// SMALL HELPERS
// ============================================================

bool piCommandFresh()
{
  if (!piCommandReceived)
  {
    return false;
  }

  return (millis() - lastPiCommandTime) <= PI_COMMAND_TIMEOUT_MS;
}

// ============================================================
// THERMISTOR FUNCTIONS
// ============================================================

float readAverageAdc()
{
  uint32_t adcTotal = 0;

  for (int i = 0; i < THERMISTOR_SAMPLES; i++)
  {
    adcTotal += analogRead(THERMISTOR_PIN);
    delay(2);
  }

  return static_cast<float>(adcTotal) / THERMISTOR_SAMPLES;
}

float calculateThermistorResistance(float adcReading)
{
  if (adcReading <= 0.0 || adcReading >= ADC_MAX)
  {
    return NAN;
  }

  if (THERMISTOR_CONNECTED_TO_3V3)
  {
    // 3.3V -> thermistor -> ADC -> fixed resistor -> GND
    return SERIES_RESISTOR_OHMS * ((ADC_MAX / adcReading) - 1.0);
  }

  // 3.3V -> fixed resistor -> ADC -> thermistor -> GND
  return SERIES_RESISTOR_OHMS * (adcReading / (ADC_MAX - adcReading));
}

float resistanceToCelsius(float resistance)
{
  if (isnan(resistance) || resistance <= 0.0)
  {
    return NAN;
  }

  float steinhart = resistance / NOMINAL_RESISTANCE_OHMS;
  steinhart = log(steinhart);
  steinhart /= BETA_COEFFICIENT;
  steinhart += 1.0 / (NOMINAL_TEMPERATURE_C + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;

  return steinhart;
}

void readThermistor()
{
  float adcReading = readAverageAdc();
  float voltage = adcReading * ADC_REFERENCE_VOLTAGE / ADC_MAX;
  float resistance = calculateThermistorResistance(adcReading);
  float temperatureC = resistanceToCelsius(resistance);
  latestBatteryTemperatureC = temperatureC;

  Serial.println("---------- Battery ECU ----------");

  Serial.print("Thermistor ADC: ");
  Serial.println(adcReading, 1);

  Serial.print("Thermistor voltage: ");
  Serial.print(voltage, 3);
  Serial.println(" V");

  Serial.print("Thermistor resistance: ");
  if (isnan(resistance))
  {
    Serial.println("Invalid reading");
  }
  else
  {
    Serial.print(resistance, 0);
    Serial.println(" ohms");
  }

  Serial.print("Battery temperature: ");
  if (isnan(temperatureC))
  {
    Serial.println("Sensor disconnected or wired incorrectly");
  }
  else
  {
    Serial.print(temperatureC, 1);
    Serial.println(" C");
  }
}

// ============================================================
// INA228 FUNCTIONS
// ============================================================

void initializeCurrentSensor()
{
  if (!ENABLE_CURRENT_SENSOR)
  {
    Serial.println("INA228 current sensor: DISABLED");
    return;
  }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);

  Serial.print("INA228 I2C SDA: GPIO ");
  Serial.println(I2C_SDA_PIN);
  Serial.print("INA228 I2C SCL: GPIO ");
  Serial.println(I2C_SCL_PIN);

  if (!ina228.begin(INA228_I2C_ADDRESS, &Wire))
  {
    Serial.println("INA228 not detected.");
    Serial.println("Check VCC, GND, SDA, SCL, and I2C address.");
    ina228Available = false;
    return;
  }

  // Configure calibration for the R002 onboard shunt and 10 A range.
  ina228.setShunt(
    INA228_SHUNT_RESISTANCE_OHMS,
    INA228_MAX_EXPECTED_CURRENT_A
  );

  // Average 16 samples to reduce measurement noise.
  ina228.setAveragingCount(INA228_COUNT_16);

  // Moderate conversion times provide stable readings without being too slow.
  ina228.setVoltageConversionTime(INA228_TIME_1052_us);
  ina228.setCurrentConversionTime(INA228_TIME_1052_us);

  ina228Available = true;

  Serial.println("INA228 current sensor: ENABLED and detected");
  Serial.print("INA228 I2C address: 0x");
  Serial.println(INA228_I2C_ADDRESS, HEX);

  Serial.print("Configured shunt resistance: ");
  Serial.print(INA228_SHUNT_RESISTANCE_OHMS, 4);
  Serial.println(" ohms");

  Serial.print("Configured maximum current: ");
  Serial.print(INA228_MAX_EXPECTED_CURRENT_A, 1);
  Serial.println(" A");

  if (!VBUS_CONNECTED)
  {
    Serial.println("VBUS is marked disconnected.");
    Serial.println("Current and shunt voltage can be tested.");
    Serial.println("Bus voltage and power will not be trusted.");
  }
}

void readCurrentSensor()
{
  if (!ENABLE_CURRENT_SENSOR)
  {
    latestCurrentMa = NAN;
    latestBusVoltageV = NAN;
    latestPowerMw = NAN;
    Serial.println("Current sensor: disabled");
    return;
  }

  if (!ina228Available)
  {
    latestCurrentMa = NAN;
    latestBusVoltageV = NAN;
    latestPowerMw = NAN;
    Serial.println("Current sensor: enabled but not detected");
    return;
  }

  float shuntVoltageMv = ina228.getShuntVoltage_mV();
  float currentMa = ina228.getCurrent_mA();
  latestCurrentMa = currentMa;

  Serial.print("Shunt voltage: ");
  Serial.print(shuntVoltageMv, 4);
  Serial.println(" mV");

  Serial.print("Current: ");
  Serial.print(currentMa, 1);
  Serial.println(" mA");

  if (VBUS_CONNECTED)
  {
    float busVoltageV = ina228.getBusVoltage_V();
    float powerMw = ina228.getPower_mW();

    latestBusVoltageV = busVoltageV;
    latestPowerMw = powerMw;

    Serial.print("Bus voltage: ");
    Serial.print(busVoltageV, 3);
    Serial.println(" V");

    Serial.print("Power: ");
    Serial.print(powerMw, 1);
    Serial.println(" mW");
  }
  else
  {
    latestBusVoltageV = NAN;
    latestPowerMw = NAN;
    Serial.println("Bus voltage: unavailable because VBUS is disconnected");
    Serial.println("Power: unavailable because VBUS is disconnected");
  }
}

// ============================================================
// CAN / TWAI FUNCTIONS
// ============================================================

void initializeCan()
{
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

  if (installResult != ESP_OK)
  {
    Serial.print("CAN driver install failed. Error: ");
    Serial.println(static_cast<int>(installResult));
    canAvailable = false;
    return;
  }

  esp_err_t startResult = twai_start();

  if (startResult != ESP_OK)
  {
    Serial.print("CAN driver start failed. Error: ");
    Serial.println(static_cast<int>(startResult));
    twai_driver_uninstall();
    canAvailable = false;
    return;
  }

  canAvailable = true;

  Serial.println("CAN/TWAI: ENABLED");
  Serial.print("CAN TX pin: GPIO ");
  Serial.println(static_cast<int>(CAN_TX_PIN));
  Serial.print("CAN RX pin: GPIO ");
  Serial.println(static_cast<int>(CAN_RX_PIN));
  Serial.println("CAN bitrate: 500 kbps");
  Serial.print("Battery status CAN ID: 0x");
  Serial.println(BATTERY_STATUS_CAN_ID, HEX);
  Serial.println("Listening for Pi drive/status frame ID 0x400.");
}

void processCanMessages()
{
  if (!canAvailable)
  {
    return;
  }

  twai_message_t message;

  // Drain queued CAN traffic without blocking.
  while (twai_receive(&message, 0) == ESP_OK)
  {
    if (message.extd || message.rtr)
    {
      continue;
    }

    if (message.identifier != PI_DRIVE_COMMAND_CAN_ID ||
        message.data_length_code != 8)
    {
      continue;
    }

    piEstopActive = (message.data[4] & 0x01) != 0;
    vehicleUnlocked = (message.data[4] & 0x02) != 0;

    overcurrentFault = (message.data[5] & 0x04) != 0;

    piCommandCounter =
      static_cast<uint16_t>(message.data[6]) |
      (static_cast<uint16_t>(message.data[7]) << 8);

    piCommandReceived = true;
    lastPiCommandTime = millis();
  }
}

int16_t clampToInt16(long value)
{
  if (value > 32767)
  {
    return 32767;
  }

  if (value < -32768)
  {
    return -32768;
  }

  return static_cast<int16_t>(value);
}

void sendBatteryCanMessage()
{
  if (!canAvailable)
  {
    return;
  }

  // Invalid or unavailable measurements are transmitted as sentinel values.
  uint16_t voltageMv = isnan(latestBusVoltageV)
    ? 0xFFFF
    : static_cast<uint16_t>(constrain(
        lroundf(latestBusVoltageV * 1000.0f),
        0L,
        65534L
      ));

  int16_t temperatureCentiC = isnan(latestBatteryTemperatureC)
    ? INT16_MIN
    : clampToInt16(lroundf(latestBatteryTemperatureC * 100.0f));

  int16_t currentMa = isnan(latestCurrentMa)
    ? INT16_MIN
    : clampToInt16(lroundf(latestCurrentMa));

  twai_message_t message = {};
  message.identifier = BATTERY_STATUS_CAN_ID;
  message.extd = 0;
  message.rtr = 0;
  message.data_length_code = 8;

  // Bytes 0-1: bus voltage in mV, unsigned, little-endian
  message.data[0] = static_cast<uint8_t>(voltageMv & 0xFF);
  message.data[1] = static_cast<uint8_t>((voltageMv >> 8) & 0xFF);

  // Bytes 2-3: battery temperature in 0.01 C, signed, little-endian
  uint16_t temperatureRaw = static_cast<uint16_t>(temperatureCentiC);
  message.data[2] = static_cast<uint8_t>(temperatureRaw & 0xFF);
  message.data[3] = static_cast<uint8_t>((temperatureRaw >> 8) & 0xFF);

  // Bytes 4-5: current in mA, signed, little-endian
  uint16_t currentRaw = static_cast<uint16_t>(currentMa);
  message.data[4] = static_cast<uint8_t>(currentRaw & 0xFF);
  message.data[5] = static_cast<uint8_t>((currentRaw >> 8) & 0xFF);

  // Bytes 6-7: rolling message counter, unsigned, little-endian
  message.data[6] = static_cast<uint8_t>(canMessageCounter & 0xFF);
  message.data[7] = static_cast<uint8_t>((canMessageCounter >> 8) & 0xFF);

  esp_err_t transmitResult = twai_transmit(
    &message,
    pdMS_TO_TICKS(20)
  );

  if (transmitResult == ESP_OK)
  {
    Serial.print("CAN TX 0x");
    Serial.print(BATTERY_STATUS_CAN_ID, HEX);
    Serial.print(" | Count: ");
    Serial.print(canMessageCounter);
    Serial.print(" | Voltage: ");

    if (voltageMv == 0xFFFF)
    {
      Serial.print("N/A");
    }
    else
    {
      Serial.print(voltageMv);
      Serial.print(" mV");
    }

    Serial.print(" | Temp: ");

    if (temperatureCentiC == INT16_MIN)
    {
      Serial.print("N/A");
    }
    else
    {
      Serial.print(temperatureCentiC / 100.0f, 2);
      Serial.print(" C");
    }

    Serial.print(" | Current: ");

    if (currentMa == INT16_MIN)
    {
      Serial.println("N/A");
    }
    else
    {
      Serial.print(currentMa);
      Serial.println(" mA");
    }

    canMessageCounter++;
  }
  else
  {
    Serial.print("CAN transmit failed. Error: ");
    Serial.println(static_cast<int>(transmitResult));
    Serial.println("Check that the receiver ECU is powered and connected.");
  }
}

// ============================================================
// STATUS LED
// ============================================================
//
// Matches the Motor ECU status LED behavior:
//
// Very fast blink = CAN initialization / driver problem
// Fast blink      = Pi command missing or stale
// Warning blink   = E-stop or overcurrent fault
// Slow blink      = healthy CAN, vehicle intentionally locked
// Solid ON        = CAN healthy, Pi command fresh, vehicle unlocked

void updateStatusLed()
{
  unsigned long t = millis();

  if (!canAvailable)
  {
    // Very fast blink = CAN initialization problem.
    digitalWrite(STATUS_LED_PIN, ((t / 100) % 2) ? HIGH : LOW);
    return;
  }

  if (!piCommandFresh())
  {
    // Fast blink = Pi command lost / not yet received.
    digitalWrite(STATUS_LED_PIN, ((t / 150) % 2) ? HIGH : LOW);
    return;
  }

  if (piEstopActive || overcurrentFault)
  {
    // Fast warning blink.
    digitalWrite(STATUS_LED_PIN, ((t / 100) % 2) ? HIGH : LOW);
    return;
  }

  if (!vehicleUnlocked)
  {
    // Slow blink = healthy CAN, vehicle intentionally locked.
    digitalWrite(STATUS_LED_PIN, ((t / 500) % 2) ? HIGH : LOW);
    return;
  }

  // Solid = CAN healthy, command fresh, vehicle unlocked.
  digitalWrite(STATUS_LED_PIN, HIGH);
}

// ============================================================
// SERIAL STATUS
// ============================================================

void printCommandStatus()
{
  static unsigned long previousPrintTime = 0;

  if (millis() - previousPrintTime < 1000)
  {
    return;
  }

  previousPrintTime = millis();

  Serial.print("Pi command: ");

  if (!piCommandReceived)
  {
    Serial.println("NEVER RECEIVED");
  }
  else
  {
    Serial.print(piCommandFresh() ? "FRESH" : "TIMEOUT");
    Serial.print(" | count=");
    Serial.print(piCommandCounter);
    Serial.print(" | unlocked=");
    Serial.print(vehicleUnlocked ? "YES" : "NO");
    Serial.print(" | estop=");
    Serial.print(piEstopActive ? "ACTIVE" : "CLEAR");
    Serial.print(" | overcurrent=");
    Serial.println(overcurrentFault ? "ACTIVE" : "CLEAR");
  }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Battery ECU starting...");

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  pinMode(THERMISTOR_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(THERMISTOR_PIN, ADC_11db);

  initializeCurrentSensor();
  initializeCan();

  // Prime telemetry once at startup so the first CAN frame contains
  // real measurements instead of NAN/sentinel startup values.
  readThermistor();
  readCurrentSensor();
  previousSensorTime = millis();

  Serial.println("Battery ECU initialization complete.");
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
  // Receive 0x400 first so the LED state updates quickly.
  processCanMessages();

  updateStatusLed();
  printCommandStatus();

  unsigned long currentTime = millis();

  if (currentTime - previousSensorTime >= SENSOR_INTERVAL_MS)
  {
    previousSensorTime = currentTime;

    readThermistor();
    readCurrentSensor();

    Serial.println();
  }

  if (currentTime - previousCanTime >= CAN_INTERVAL_MS)
  {
    previousCanTime = currentTime;
    sendBatteryCanMessage();
  }
}