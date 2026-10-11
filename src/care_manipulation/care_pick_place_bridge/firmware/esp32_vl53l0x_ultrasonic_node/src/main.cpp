#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

namespace config {
constexpr uint8_t kSensorCount = 4;

constexpr int kStatusLedPin = 2;
constexpr int kSdaPin = 21;
constexpr int kSclPin = 22;
constexpr uint32_t kI2cClockHz = 100000;

constexpr int kUltrasonicRxPin = 4;
constexpr int kUltrasonicTxPin = 5;
constexpr uint32_t kUltrasonicBaud = 9600;
constexpr uint32_t kUltrasonicTriggerPeriodMs = 100;
constexpr uint32_t kUltrasonicTimeoutMs = 200;

constexpr int kRs485RxPin = 16;
constexpr int kRs485TxPin = 17;
constexpr uint32_t kRs485Baud = 115200;
constexpr uint32_t kRs485SendPeriodMs = 100;

constexpr uint16_t kInvalidDistanceMm = 9999;
constexpr uint16_t kVl53l0xOutOfRangeMm = 8190;
constexpr uint16_t kUltrasonicMinMm = 30;
constexpr uint16_t kUltrasonicMaxMm = 4500;

constexpr uint8_t kFrameStart0 = 0xAA;
constexpr uint8_t kFrameStart1 = 0x55;
constexpr uint8_t kCmdVl53l0xUltrasonic = 0x01;
constexpr uint8_t kPayloadLen = 10;
constexpr uint8_t kFrameLen = kPayloadLen + 5;

constexpr int kXshutPins[kSensorCount] = {32, 33, 25, 26};
constexpr uint8_t kI2cAddresses[kSensorCount] = {0x30, 0x31, 0x32, 0x29};
}  // namespace config

struct TofSensorState {
  VL53L0X sensor;
  bool ready = false;
  bool timeout = false;
  uint16_t distanceMm = config::kInvalidDistanceMm;
  uint32_t lastReadMs = 0;
};

HardwareSerial UltraSerial(1);
HardwareSerial Rs485Serial(2);

TofSensorState tof[config::kSensorCount];
uint8_t nextTofIndex = 0;

uint16_t ultrasonicMm = config::kInvalidDistanceMm;
uint8_t ultrasonicFrame[4] = {0};
uint8_t ultrasonicFrameIndex = 0;
uint32_t lastUltrasonicTriggerMs = 0;
uint32_t lastUltrasonicReceiveMs = 0;
uint32_t lastRs485SendMs = 0;
uint32_t lastStatusPrintMs = 0;

static uint8_t payloadChecksum(const uint8_t* frame, uint8_t lastPayloadIndex) {
  uint8_t checksum = 0;
  for (uint8_t i = 2; i <= lastPayloadIndex; ++i) {
    checksum = static_cast<uint8_t>(checksum + frame[i]);
  }
  return checksum;
}

static uint16_t normalizedTofDistance(uint16_t mm, bool timeout) {
  if (timeout || mm == 0 || mm >= config::kVl53l0xOutOfRangeMm) {
    return config::kInvalidDistanceMm;
  }
  return mm;
}

static void shutdownAllTofSensors() {
  for (uint8_t i = 0; i < config::kSensorCount; ++i) {
    pinMode(config::kXshutPins[i], OUTPUT);
    digitalWrite(config::kXshutPins[i], LOW);
    tof[i].ready = false;
    tof[i].timeout = false;
    tof[i].distanceMm = config::kInvalidDistanceMm;
  }
  delay(100);
}

static void initTofSensors() {
  shutdownAllTofSensors();

  for (uint8_t i = 0; i < config::kSensorCount; ++i) {
    Serial.printf("\n--- Starting VL53L0X %u ---\r\n", i);
    digitalWrite(config::kXshutPins[i], HIGH);
    delay(500);

    tof[i].sensor.setTimeout(500);
    if (!tof[i].sensor.init()) {
      Serial.printf("VL53L0X %u init failed at default address 0x29\r\n", i);
      continue;
    }

    if (config::kI2cAddresses[i] != 0x29) {
      tof[i].sensor.setAddress(config::kI2cAddresses[i]);
      Serial.printf("VL53L0X %u assigned address 0x%02X\r\n", i, config::kI2cAddresses[i]);
    } else {
      Serial.printf("VL53L0X %u kept default address 0x29\r\n", i);
    }

    tof[i].sensor.startContinuous(0);
    tof[i].lastReadMs = millis();
    tof[i].ready = true;
    Serial.printf("VL53L0X %u continuous ranging started\r\n", i);
    delay(50);
  }
}

static void serviceTofRoundRobin(uint32_t now) {
  TofSensorState& current = tof[nextTofIndex];

  if (current.ready && (now - current.lastReadMs) >= 50) {
    uint16_t rawMm = current.sensor.readRangeContinuousMillimeters();
    current.timeout = current.sensor.timeoutOccurred();
    current.distanceMm = normalizedTofDistance(rawMm, current.timeout);
    current.lastReadMs = now;
  }

  nextTofIndex = static_cast<uint8_t>((nextTofIndex + 1) % config::kSensorCount);
}

static void triggerUltrasonic() {
  while (UltraSerial.available()) {
    UltraSerial.read();
  }
  ultrasonicFrameIndex = 0;
  UltraSerial.write(0x55);
}

static void serviceUltrasonicRx() {
  while (UltraSerial.available()) {
    uint8_t byteIn = static_cast<uint8_t>(UltraSerial.read());

    if (ultrasonicFrameIndex == 0 && byteIn != 0xFF) {
      continue;
    }

    ultrasonicFrame[ultrasonicFrameIndex++] = byteIn;
    if (ultrasonicFrameIndex < sizeof(ultrasonicFrame)) {
      continue;
    }

    uint8_t expected = static_cast<uint8_t>(
        ultrasonicFrame[0] + ultrasonicFrame[1] + ultrasonicFrame[2]);
    if (expected == ultrasonicFrame[3]) {
      uint16_t distance = static_cast<uint16_t>(
          (static_cast<uint16_t>(ultrasonicFrame[1]) << 8) | ultrasonicFrame[2]);

      ultrasonicMm = (distance >= config::kUltrasonicMinMm &&
                      distance <= config::kUltrasonicMaxMm)
                         ? distance
                         : config::kInvalidDistanceMm;
      lastUltrasonicReceiveMs = millis();
    }

    ultrasonicFrameIndex = 0;
  }
}

static void sendSensorFrame() {
  uint8_t frame[config::kFrameLen] = {0};
  frame[0] = config::kFrameStart0;
  frame[1] = config::kFrameStart1;
  frame[2] = config::kPayloadLen;
  frame[3] = config::kCmdVl53l0xUltrasonic;

  for (uint8_t i = 0; i < config::kSensorCount; ++i) {
    uint16_t mm = tof[i].ready ? tof[i].distanceMm : config::kInvalidDistanceMm;
    frame[4 + (i * 2)] = static_cast<uint8_t>(mm >> 8);
    frame[5 + (i * 2)] = static_cast<uint8_t>(mm & 0xFF);
  }

  frame[12] = static_cast<uint8_t>(ultrasonicMm >> 8);
  frame[13] = static_cast<uint8_t>(ultrasonicMm & 0xFF);
  frame[14] = payloadChecksum(frame, 13);

  Rs485Serial.write(frame, sizeof(frame));
}

static void printStatus(uint32_t now) {
  if ((now - lastStatusPrintMs) < 500) {
    return;
  }

  digitalWrite(config::kStatusLedPin, !digitalRead(config::kStatusLedPin));
  Serial.printf("--- VL53L0X + ultrasonic node @ %lu ms ---\r\n", now);
  for (uint8_t i = 0; i < config::kSensorCount; ++i) {
    if (!tof[i].ready) {
      Serial.printf(" ToF %u: OFFLINE\r\n", i);
    } else if (tof[i].distanceMm == config::kInvalidDistanceMm) {
      Serial.printf(" ToF %u: OUT_OF_RANGE/TIMEOUT\r\n", i);
    } else {
      Serial.printf(" ToF %u: %u mm\r\n", i, tof[i].distanceMm);
    }
  }

  if (ultrasonicMm == config::kInvalidDistanceMm) {
    Serial.println(" Ultra : TIMEOUT/DISCONNECTED");
  } else {
    Serial.printf(" Ultra : %u mm\r\n", ultrasonicMm);
  }
  Serial.println();
  lastStatusPrintMs = now;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n=== ESP32 sensor node: 4x VL53L0X + 1x DYP-A02 ===");

  pinMode(config::kStatusLedPin, OUTPUT);
  digitalWrite(config::kStatusLedPin, LOW);

  UltraSerial.begin(config::kUltrasonicBaud,
                    SERIAL_8N1,
                    config::kUltrasonicRxPin,
                    config::kUltrasonicTxPin);

  Rs485Serial.begin(config::kRs485Baud,
                    SERIAL_8N1,
                    config::kRs485RxPin,
                    config::kRs485TxPin);

  Wire.begin(config::kSdaPin, config::kSclPin, config::kI2cClockHz);
  delay(100);
  initTofSensors();

  lastUltrasonicReceiveMs = millis();
  Serial.println("\nSetup complete. Sending RS485 cmd=0x01 frames.\r\n");
}

void loop() {
  uint32_t now = millis();

  serviceTofRoundRobin(now);

  if ((now - lastUltrasonicTriggerMs) >= config::kUltrasonicTriggerPeriodMs) {
    triggerUltrasonic();
    lastUltrasonicTriggerMs = now;
  }
  serviceUltrasonicRx();

  if ((now - lastUltrasonicReceiveMs) > config::kUltrasonicTimeoutMs) {
    ultrasonicMm = config::kInvalidDistanceMm;
  }

  if ((now - lastRs485SendMs) >= config::kRs485SendPeriodMs) {
    sendSensorFrame();
    lastRs485SendMs = now;
  }

  printStatus(now);
}
