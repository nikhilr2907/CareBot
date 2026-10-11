#include <Arduino.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <Wire.h>
#include <string.h>

namespace config {
constexpr uint8_t kSensorCount = 3;
constexpr uint8_t kZonesPerSensor = 16;

constexpr int kStatusLedPin = 2;
constexpr int kSdaPin = 21;
constexpr int kSclPin = 22;
constexpr uint32_t kI2cInitClockHz = 400000UL;
constexpr uint32_t kI2cRunClockHz = 400000UL;

constexpr int kLtc4311EnablePin = 32;
constexpr bool kUseLtc4311Enable = true;

constexpr int kRs485RxPin = 16;
constexpr int kRs485TxPin = 17;
constexpr uint32_t kRs485Baud = 115200;
constexpr uint32_t kRs485SendPeriodMs = 50;

constexpr uint16_t kStopThresholdMm = 200;
constexpr uint16_t kInvalidDistanceMm = 0xFFFF;
constexpr uint32_t kSensorStaleMs = 300;

constexpr uint8_t kFrameStart0 = 0xAA;
constexpr uint8_t kFrameStart1 = 0x55;
constexpr uint8_t kCmdVl53l5Grid = 0x02;
constexpr uint8_t kPayloadLen = 3 + (kSensorCount * kZonesPerSensor * 2);
constexpr uint8_t kFrameLen = kPayloadLen + 5;

constexpr uint8_t kDefaultAddress = 0x29;
constexpr uint8_t kAddresses[kSensorCount] = {0x29, 0x2A, 0x2B};

// Matches the attached VL53L5-array sketch.
constexpr int kLpnPins[kSensorCount] = {14, 27, 25};
constexpr int kInterruptPins[kSensorCount] = {13, 35, 26};

constexpr bool kFanControlEnabled = true;
constexpr uint8_t kFanCount = 2;
constexpr int kFanPins[kFanCount] = {18, 19};
constexpr uint8_t kFanPwmChannels[kFanCount] = {0, 1};
constexpr uint32_t kFanPwmHz = 25000;
constexpr uint8_t kFanPwmBits = 8;
constexpr uint8_t kFanIdleDutyPercent = 70;
constexpr uint8_t kFanStopDutyPercent = 100;

constexpr bool kRunStartupLpnProbe = false;
}  // namespace config

struct SensorState {
  bool online = false;
  bool stop = false;
  uint16_t minDistanceMm = config::kInvalidDistanceMm;
  uint16_t zones[config::kZonesPerSensor] = {0};
  uint32_t frames = 0;
  uint32_t lastUpdateMs = 0;
};

SparkFun_VL53L5CX sensors[config::kSensorCount];
SensorState sensorState[config::kSensorCount];
SemaphoreHandle_t sensorMutex = nullptr;

HardwareSerial Rs485Serial(2);

uint8_t currentFanDutyPercent = config::kFanIdleDutyPercent;

static uint8_t checksumSum(const uint8_t* frame, uint16_t lastPayloadIndex) {
  uint8_t checksum = 0;
  for (uint16_t i = 2; i <= lastPayloadIndex; ++i) {
    checksum = static_cast<uint8_t>(checksum + frame[i]);
  }
  return checksum;
}

static bool zoneValid(uint8_t status, int16_t distanceMm) {
  return (status == 5 || status == 9) && distanceMm > 0;
}

static void fillInvalidZones(uint16_t zones[config::kZonesPerSensor]) {
  for (uint8_t i = 0; i < config::kZonesPerSensor; ++i) {
    zones[i] = config::kInvalidDistanceMm;
  }
}

static void scanI2c(const char* label) {
  Serial.print("I2C scan (");
  Serial.print(label);
  Serial.print("):");
  bool found = false;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(" 0x");
      Serial.print(addr, HEX);
      found = true;
    }
  }
  if (!found) {
    Serial.print(" <nothing>");
  }
  Serial.println();
}

static void setAllLpnLow() {
  for (uint8_t i = 0; i < config::kSensorCount; ++i) {
    pinMode(config::kLpnPins[i], OUTPUT);
    digitalWrite(config::kLpnPins[i], LOW);
  }
}

static void runLpnProbe() {
  if (!config::kRunStartupLpnProbe) {
    return;
  }

  Serial.println("--- LPn probe ---");
  for (uint8_t i = 0; i < config::kSensorCount; ++i) {
    setAllLpnLow();
    delay(50);
    digitalWrite(config::kLpnPins[i], HIGH);
    delay(200);
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" LPn=GPIO");
    Serial.print(config::kLpnPins[i]);
    Serial.print(" ");
    scanI2c("probe");
  }
  setAllLpnLow();
  delay(50);
  Serial.println("--- end LPn probe ---");
}

static bool startSensor(uint8_t index) {
  Serial.print("Bringing up VL53L5CX ");
  Serial.print(index);
  Serial.print(" LPn=GPIO");
  Serial.println(config::kLpnPins[index]);

  digitalWrite(config::kLpnPins[index], HIGH);
  delay(200);

  if (!sensors[index].begin()) {
    Serial.print("  FAIL: begin() failed for sensor ");
    Serial.println(index);
    digitalWrite(config::kLpnPins[index], LOW);
    return false;
  }

  if (config::kAddresses[index] != config::kDefaultAddress &&
      !sensors[index].setAddress(config::kAddresses[index])) {
    Serial.print("  FAIL: setAddress failed for sensor ");
    Serial.println(index);
    digitalWrite(config::kLpnPins[index], LOW);
    return false;
  }

  sensors[index].setResolution(4 * 4);
  sensors[index].setRangingFrequency(30);
  sensors[index].startRanging();

  xSemaphoreTake(sensorMutex, portMAX_DELAY);
  sensorState[index].online = true;
  sensorState[index].lastUpdateMs = millis();
  fillInvalidZones(sensorState[index].zones);
  xSemaphoreGive(sensorMutex);

  Serial.print("  sensor ");
  Serial.print(index);
  Serial.print(" online at 7-bit 0x");
  Serial.println(config::kAddresses[index], HEX);
  return true;
}

static void initSensors() {
  if (config::kUseLtc4311Enable) {
    pinMode(config::kLtc4311EnablePin, OUTPUT);
    digitalWrite(config::kLtc4311EnablePin, HIGH);
  }

  setAllLpnLow();
  for (uint8_t i = 0; i < config::kSensorCount; ++i) {
    pinMode(config::kInterruptPins[i], INPUT);
    fillInvalidZones(sensorState[i].zones);
  }
  delay(50);

  Wire.begin(config::kSdaPin, config::kSclPin);
  Wire.setClock(config::kI2cInitClockHz);
  scanI2c("all sensors in standby");
  runLpnProbe();

  // Wake sensors that need reassigned addresses first. The default-address
  // sensor is woken last so it can safely remain at 0x29.
  for (uint8_t i = 1; i < config::kSensorCount; ++i) {
    startSensor(i);
  }
  startSensor(0);

  Wire.setClock(config::kI2cRunClockHz);
  scanI2c("post-init");
}

static void configureFans() {
  if (!config::kFanControlEnabled) {
    return;
  }

  for (uint8_t i = 0; i < config::kFanCount; ++i) {
    ledcSetup(config::kFanPwmChannels[i], config::kFanPwmHz, config::kFanPwmBits);
    ledcAttachPin(config::kFanPins[i], config::kFanPwmChannels[i]);
  }
}

static void setFanDuty(uint8_t dutyPercent) {
  currentFanDutyPercent = constrain(dutyPercent, 0, 100);

  if (!config::kFanControlEnabled) {
    return;
  }

  uint32_t maxDuty = (1u << config::kFanPwmBits) - 1u;
  uint32_t duty = (maxDuty * currentFanDutyPercent) / 100u;
  for (uint8_t i = 0; i < config::kFanCount; ++i) {
    ledcWrite(config::kFanPwmChannels[i], duty);
  }
}

static void updateSensorState(uint8_t index, VL53L5CX_ResultsData& frame) {
  SensorState local;
  local.online = true;
  local.stop = false;
  local.minDistanceMm = config::kInvalidDistanceMm;
  local.lastUpdateMs = millis();
  fillInvalidZones(local.zones);

  for (uint8_t i = 0; i < config::kZonesPerSensor; ++i) {
    int16_t distance = frame.distance_mm[i];
    if (zoneValid(frame.target_status[i], distance)) {
      local.zones[i] = static_cast<uint16_t>(distance);
      if (local.zones[i] < local.minDistanceMm) {
        local.minDistanceMm = local.zones[i];
      }
      if (local.zones[i] < config::kStopThresholdMm) {
        local.stop = true;
      }
    }
  }

  xSemaphoreTake(sensorMutex, portMAX_DELAY);
  local.frames = sensorState[index].frames + 1;
  sensorState[index] = local;
  xSemaphoreGive(sensorMutex);
}

static void tofTask(void* /*arg*/) {
  VL53L5CX_ResultsData frame;
  for (;;) {
    for (uint8_t i = 0; i < config::kSensorCount; ++i) {
      bool online = false;
      xSemaphoreTake(sensorMutex, portMAX_DELAY);
      online = sensorState[i].online;
      xSemaphoreGive(sensorMutex);

      if (online && sensors[i].isDataReady() && sensors[i].getRangingData(&frame)) {
        updateSensorState(i, frame);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static void sendRs485Frame(const SensorState snapshot[config::kSensorCount]) {
  uint8_t frame[config::kFrameLen] = {0};
  frame[0] = config::kFrameStart0;
  frame[1] = config::kFrameStart1;
  frame[2] = config::kPayloadLen;
  frame[3] = config::kCmdVl53l5Grid;

  uint8_t onlineMask = 0;
  uint8_t stopFlags = 0;
  for (uint8_t i = 0; i < config::kSensorCount; ++i) {
    if (snapshot[i].online) {
      onlineMask |= (1u << i);
    }
    if (snapshot[i].stop) {
      stopFlags |= (1u << i);
    }
  }

  frame[4] = onlineMask;
  frame[5] = stopFlags;
  frame[6] = currentFanDutyPercent;

  uint16_t p = 7;
  for (uint8_t sensor = 0; sensor < config::kSensorCount; ++sensor) {
    for (uint8_t zone = 0; zone < config::kZonesPerSensor; ++zone) {
      uint16_t mm = snapshot[sensor].zones[zone];
      frame[p++] = static_cast<uint8_t>(mm >> 8);
      frame[p++] = static_cast<uint8_t>(mm & 0xFF);
    }
  }

  frame[config::kFrameLen - 1] = checksumSum(frame, config::kFrameLen - 2);
  Rs485Serial.write(frame, sizeof(frame));
}

static void printStatus(const SensorState snapshot[config::kSensorCount],
                        const uint32_t previousFrames[config::kSensorCount],
                        float seconds) {
  Serial.print("[");
  Serial.print(millis() / 1000);
  Serial.print("s] fan=");
  Serial.print(currentFanDutyPercent);
  Serial.print("% ");

  for (uint8_t i = 0; i < config::kSensorCount; ++i) {
    uint32_t delta = snapshot[i].frames - previousFrames[i];
    Serial.print("T");
    Serial.print(i);
    Serial.print(" ");
    if (!snapshot[i].online) {
      Serial.print("offline");
    } else {
      Serial.print(delta / seconds, 1);
      Serial.print("fps min=");
      if (snapshot[i].minDistanceMm == config::kInvalidDistanceMm) {
        Serial.print("---");
      } else {
        Serial.print(snapshot[i].minDistanceMm);
      }
      if (snapshot[i].stop) {
        Serial.print(" STOP");
      }
    }
    Serial.print("  ");
  }
  Serial.println();
}

static void commTask(void* /*arg*/) {
  TickType_t wake = xTaskGetTickCount();
  uint32_t lastReportMs = millis();
  uint32_t framesAtLastReport[config::kSensorCount] = {0};

  for (;;) {
    SensorState snapshot[config::kSensorCount];
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    memcpy(snapshot, sensorState, sizeof(snapshot));
    xSemaphoreGive(sensorMutex);

    bool anyStop = false;
    uint32_t now = millis();
    for (uint8_t i = 0; i < config::kSensorCount; ++i) {
      bool fresh = snapshot[i].online && ((now - snapshot[i].lastUpdateMs) < config::kSensorStaleMs);
      if (!fresh) {
        snapshot[i].stop = false;
      }
      anyStop = anyStop || snapshot[i].stop;
    }

    setFanDuty(anyStop ? config::kFanStopDutyPercent : config::kFanIdleDutyPercent);
    sendRs485Frame(snapshot);

    if ((now - lastReportMs) >= 1000) {
      float seconds = (now - lastReportMs) / 1000.0f;
      printStatus(snapshot, framesAtLastReport, seconds);
      for (uint8_t i = 0; i < config::kSensorCount; ++i) {
        framesAtLastReport[i] = snapshot[i].frames;
      }
      digitalWrite(config::kStatusLedPin, !digitalRead(config::kStatusLedPin));
      lastReportMs = now;
    }

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(config::kRs485SendPeriodMs));
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("\n=== ESP32 sensor node: 3x VL53L5CX + fans ===");

  pinMode(config::kStatusLedPin, OUTPUT);
  digitalWrite(config::kStatusLedPin, LOW);

  sensorMutex = xSemaphoreCreateMutex();
  if (!sensorMutex) {
    Serial.println("FATAL: could not create sensor mutex");
    while (true) {
      delay(1000);
    }
  }

  initSensors();
  configureFans();
  setFanDuty(config::kFanIdleDutyPercent);

  Rs485Serial.begin(config::kRs485Baud,
                    SERIAL_8N1,
                    config::kRs485RxPin,
                    config::kRs485TxPin);
  Serial.print("RS485 Serial2 up at ");
  Serial.print(config::kRs485Baud);
  Serial.print(" baud, RX=GPIO");
  Serial.print(config::kRs485RxPin);
  Serial.print(" TX=GPIO");
  Serial.println(config::kRs485TxPin);

  xTaskCreatePinnedToCore(tofTask, "tofTask", 8192, nullptr, 5, nullptr, 0);
  xTaskCreatePinnedToCore(commTask, "commTask", 6144, nullptr, 4, nullptr, 1);

  Serial.println("Tasks running. Sending RS485 cmd=0x02 frames.");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
