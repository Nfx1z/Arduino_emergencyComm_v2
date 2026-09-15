// =========================================================================
// SUB-FIELD DEVICE FIRMWARE (rev.5) - TYPE 5 / TYPE 12 FIX
// -------------------------------------------------------------------------
// Important message types:
//   Type 5  = MSG_FIELD_DATA_REQUEST    : HQ -> FIELD DEVICE
//   Type 12 = MSG_SUBFIELD_DATA_REQUEST : FIELD DEVICE -> SUBFIELD
//   Type 13 = MSG_SUBFIELD_DATA_REPLY   : SUBFIELD -> FIELD DEVICE
//
// This sub-field firmware must reply only to type 12.
// Type 5 may be heard directly from HQ because all LoRa modules are in
// transparent mode on the same channel/address. It must be ignored.
// =========================================================================

#include <DHT.h>
#include "LoRa_E220.h"

// =========================================================================
// DEBUG
// =========================================================================
#define DEBUG_ENABLED   1
#define DEBUG_BAUD      115200

#if DEBUG_ENABLED
#define DEBUG_PRINT(x)    Serial.print(x)
#define DEBUG_PRINTLN(x)  Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// =========================================================================
// DEVICE IDENTITY
// =========================================================================
// AREA_ID must match field device.
// SUBFIELD_ID must be unique for every sub-field device.
//
// Device 1: SUBFIELD_ID 1
// Device 2: SUBFIELD_ID 2
//
#define AREA_ID       591
#define SUBFIELD_ID   1

// =========================================================================
// PIN DEFINITIONS - ESP32-C3 MINI
// =========================================================================
#define DHT_PIN     6
#define DHT_TYPE    DHT22
#define MQ2_PIN     2

#define LORA_AUX    5
#define LORA_M0     3
#define LORA_M1     4

// -------------------------------------------------------------------------
// LoRa UART pins (TXD=0, RXD=1). Matches
// LoRaSerial.begin(9600, SERIAL_8N1, LORA_RXD, LORA_TXD) in setup().
// -------------------------------------------------------------------------
#define LORA_TXD    0
#define LORA_RXD    1

// =========================================================================
// WARNING THRESHOLDS
// =========================================================================
#define GAS_LEAK_MQ2_THRESHOLD       3500
#define FIRE_GAS_THRESHOLD           3000
#define FIRE_TEMP_THRESHOLD_C         45.0f

// =========================================================================
// TIMING CONSTANTS
// =========================================================================
#define SENSOR_READ_INTERVAL_MS       2000UL
#define HEARTBEAT_INTERVAL_MS      1800000UL
#define WARNING_RESEND_INTERVAL_MS  60000UL
#define WARNING_ACK_TIMEOUT_MS  3600000UL   // 1 hour

// -------------------------------------------------------------------------
// DATA REPLY RESPONSE SETTINGS
// -------------------------------------------------------------------------
// Reply is sent once, staggered by subfield ID so subfields on the same
// area/channel don't all transmit at once: SUBFIELD_ID * this value.
// e.g. subfield 1 replies 1000ms after the request, subfield 2 at 2000ms.
#define DATA_REPLY_DELAY_PER_SUBFIELD_MS 1000UL

// Parser timeout. If a packet is incomplete for this long, reset parser.
#define PARSER_TIMEOUT_MS             250UL

// Large enough to size-skip any recognized message this device may hear on
// the shared LoRa channel/address (see getMessageSize() below); the actual
// FieldDataResponse struct on the field device is 32 bytes, which fits.
#define MAX_MESSAGE_SIZE              32

// =========================================================================
// MESSAGE STRUCTURES USED BY THIS SUB-FIELD DEVICE
// =========================================================================
#pragma pack(push, 1)

struct FieldDataRequestMessage {
  uint8_t type;
  uint16_t areaId;
};

struct SubFieldDataRequestMessage {
  uint8_t type;
  uint16_t areaId;
};

struct SubFieldDataReplyMessage {
  uint8_t type;
  uint16_t areaId;
  uint8_t subFieldId;
  uint16_t mq2Ppm;
  int16_t temperatureX10;
  int16_t humidityX10;
};

struct WarningMessage {
  uint8_t type;
  uint16_t areaId;
  uint8_t subFieldId;
  uint8_t reason;
};

struct SubFieldHeartbeatMessage {
  uint8_t type;
  uint16_t areaId;
  uint8_t subFieldId;
};

struct WarningAckMessage {
  uint8_t type;      // = 14
  uint16_t areaId;
  uint8_t subFieldId;
};

#pragma pack(pop)

enum MessageType {
  MSG_FIELD_DATA_REQUEST    = 5,
  MSG_WARNING               = 9,
  MSG_SUBFIELD_HEARTBEAT    = 11,
  MSG_SUBFIELD_DATA_REQUEST = 12,
  MSG_SUBFIELD_DATA_REPLY   = 13,
  MSG_WARNING_ACK           = 14,
};

enum WarningReason {
  WARN_GAS_LEAK    = 0,
  WARN_FOREST_FIRE = 1,
};

// =========================================================================
// GLOBAL OBJECTS
// =========================================================================
DHT dht(DHT_PIN, DHT_TYPE);
HardwareSerial LoRaSerial(1);
LoRa_E220 e220(&LoRaSerial, LORA_AUX, LORA_M0, LORA_M1);

// =========================================================================
// SENSOR STATE
// =========================================================================
uint16_t latestMq2Value = 0;
int16_t latestTemperatureX10 = 0;
int16_t latestHumidityX10 = 0;
bool sensorsReady = false;

unsigned long lastSensorReadTime = 0;
unsigned long lastHeartbeatTime = 0;

bool warningActive = false;
uint8_t activeWarningReason = 0;
bool warningAcknowledged = false;
unsigned long lastWarningSendTime = 0;
unsigned long warningAckTime = 0;

// =========================================================================
// RX PARSER STATE
// =========================================================================
uint8_t rxBuffer[MAX_MESSAGE_SIZE];
size_t rxBytesReceived = 0;
size_t rxExpectedSize = 0;
bool rxReadingMessage = false;
unsigned long rxLastByteTime = 0;

// =========================================================================
// DATA REPLY SCHEDULER STATE
// =========================================================================
bool dataReplyPending = false;
unsigned long dataReplySendTime = 0;

// =========================================================================
// FUNCTION PROTOTYPES
// =========================================================================
void setLoRaNormalMode();
void waitLoRaIdle(unsigned long timeoutMs);
void resetRxParser();

void readSensorsIfDue();

void checkHeartbeat();
void sendHeartbeat();

void checkWarningCondition();
void sendWarning(uint8_t reason);

void checkIncomingMessages();
void processCompleteMessage(uint8_t* buffer, size_t len, uint8_t type, uint16_t areaId);

void scheduleDataReply();
void handlePendingDataReply();
void sendDataReply();

size_t getMessageSize(uint8_t type);

// =========================================================================
// LORA CONTROL
// =========================================================================
void setLoRaNormalMode() {
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  delay(5);
  waitLoRaIdle(250);
}

void waitLoRaIdle(unsigned long timeoutMs) {
  // E220 AUX is usually LOW while module is busy.
  // Timeout prevents hang if AUX behavior is different.
  unsigned long start = millis();
  while (digitalRead(LORA_AUX) == LOW) {
    if (millis() - start >= timeoutMs) break;
    delay(1);
  }
}

// =========================================================================
// MESSAGE SIZE FUNCTION
// =========================================================================
// Sizes for every message type this sub-field may hear on the shared
// LoRa channel/address, so the parser can skip them cleanly even though
// this device only acts on type 12 (see processCompleteMessage()).
//
// Type 1  HELP                  = 13 bytes
// Type 2  ACK                   = 3 bytes
// Type 3  ALARM                 = 11 bytes
// Type 4  CANCEL_HELP           = 3 bytes
// Type 5  FIELD_DATA_REQUEST    = 3 bytes
// Type 6  FIELD_DATA_RESPONSE   = 32 bytes (size-skip only; not parsed)
// Type 7  HEARTBEAT             = 5 bytes
// Type 8  MISSING_BEACON        = 4 bytes
// Type 9  WARNING               = sizeof(WarningMessage)
// Type 10 WARNING_FORWARD       = 13 bytes
// Type 11 SUBFIELD_HEARTBEAT    = sizeof(SubFieldHeartbeatMessage)
// Type 12 SUBFIELD_DATA_REQUEST = sizeof(SubFieldDataRequestMessage)
// Type 13 SUBFIELD_DATA_REPLY   = sizeof(SubFieldDataReplyMessage)
// =========================================================================
size_t getMessageSize(uint8_t type) {
  switch (type) {
    case 1:  return 13;
    case 2:  return 3;
    case 3:  return 11;
    case 4:  return 3;
    case 5:  return sizeof(FieldDataRequestMessage);
    case 6:  return 32;
    case 7:  return 5;
    case 8:  return 4;
    case 9:  return sizeof(WarningMessage);
    case 10: return 13;
    case 11: return sizeof(SubFieldHeartbeatMessage);
    case 12: return sizeof(SubFieldDataRequestMessage);
    case 13: return sizeof(SubFieldDataReplyMessage);
    case 14: return sizeof(WarningAckMessage);
    default: return 0;
  }
}

// =========================================================================
// SETUP
// =========================================================================
void setup() {
#if DEBUG_ENABLED
  Serial.begin(DEBUG_BAUD);
  delay(300);
  Serial.println();
  Serial.println("=== SUB-FIELD FIRMWARE rev.5 BOOT ===");
  Serial.print("AREA_ID=");
  Serial.print(AREA_ID);
  Serial.print("  SUBFIELD_ID=");
  Serial.println(SUBFIELD_ID);
#endif

  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  pinMode(LORA_AUX, INPUT);
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);

  analogSetPinAttenuation(MQ2_PIN, ADC_11db);
  dht.begin();

  delay(500);

  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RXD, LORA_TXD);

  DEBUG_PRINTLN("Initializing LoRa E220...");

  if (e220.begin() != 1) {
    DEBUG_PRINTLN("!!! LoRa init FAILED - halting.");
    while (1) {
      delay(1000);
    }
  }

  DEBUG_PRINTLN("LoRa init OK.");
  delay(500);

  // FIX: silently continuing on config failure leaves this module on its
  // factory-default ADDH/ADDL/CHAN, incompatible with the rest of the
  // network (all set to 0x12/0x34/56) - this is exactly what produces the
  // "areaId mismatch (got <garbage>)" symptom on other nodes. Retry a few
  // times, then halt (same pattern as the e220.begin() failure above)
  // rather than run with a broken/mismatched radio config.
  bool loraConfigApplied = false;
  for (int attempt = 1; attempt <= 5 && !loraConfigApplied; attempt++) {
    ResponseStructContainer cfgContainer = e220.getConfiguration();
    if (cfgContainer.status.code == 1) {
      Configuration* cfg = (Configuration*)cfgContainer.data;

      cfg->ADDH = 0x12;
      cfg->ADDL = 0x34;
      cfg->CHAN = 56;

      cfg->SPED.uartBaudRate = UART_BPS_9600;
      cfg->SPED.uartParity = MODE_00_8N1;
      cfg->SPED.airDataRate = AIR_DATA_RATE_010_24;

      cfg->OPTION.subPacketSetting = SPS_032_11;
      cfg->TRANSMISSION_MODE.WORPeriod = WOR_500_000;

      cfg->OPTION.transmissionPower = POWER_22;
      cfg->OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;

      cfg->TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
      cfg->TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;
      cfg->TRANSMISSION_MODE.enableLBT = LBT_DISABLED;

      e220.setConfiguration(*cfg, WRITE_CFG_PWR_DWN_SAVE);
      e220.resetModule();
      cfgContainer.close();

      loraConfigApplied = true;
      DEBUG_PRINTLN("LoRa config applied.");
    } else {
      DEBUG_PRINT("!!! getConfiguration() failed (attempt ");
      DEBUG_PRINT(attempt);
      DEBUG_PRINTLN("/5) - retrying.");
      cfgContainer.close();
      delay(500);
    }
  }
  if (!loraConfigApplied) {
    DEBUG_PRINTLN("!!! LoRa config could not be applied - halting (address/channel would mismatch the rest of the network).");
    while (1) delay(1000);
  }

  delay(500);

  setLoRaNormalMode();

  lastHeartbeatTime = millis();
  lastSensorReadTime = 0;

  resetRxParser();

  DEBUG_PRINTLN("=== Sub-field setup complete ===");
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
  // Check incoming first so data requests are not delayed too much.
  checkIncomingMessages();

  // Handle scheduled data replies.
  handlePendingDataReply();

  // Periodic sensor sampling is disabled here; sendDataReply() calls
  // readSensorsIfDue() itself right before it replies.
  //readSensorsIfDue();

  // Avoid transmitting heartbeat/warning while a data reply sequence is active.
  if (!dataReplyPending) {
    checkHeartbeat();
    checkWarningCondition();
  }

  delay(5);
}

// =========================================================================
// SENSOR SAMPLING
// =========================================================================
void readSensorsIfDue() {
  if (millis() - lastSensorReadTime < SENSOR_READ_INTERVAL_MS) return;

  lastSensorReadTime = millis();

  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  int mq2Raw = analogRead(MQ2_PIN);

  if (isnan(temp) || isnan(hum)) {
    DEBUG_PRINTLN("Sensor read failed - keeping old values.");
    return;
  }

  latestTemperatureX10 = (int16_t)(temp * 10.0f);
  latestHumidityX10 = (int16_t)(hum * 10.0f);
  latestMq2Value = (uint16_t)mq2Raw;
  sensorsReady = true;

  DEBUG_PRINT("Sensors: temp=");
  DEBUG_PRINT(temp);
  DEBUG_PRINT("C, hum=");
  DEBUG_PRINT(hum);
  DEBUG_PRINT("%, mq2=");
  DEBUG_PRINTLN(mq2Raw);
}

// =========================================================================
// HEARTBEAT
// =========================================================================
void checkHeartbeat() {
  if (millis() - lastHeartbeatTime < HEARTBEAT_INTERVAL_MS) return;

  lastHeartbeatTime = millis();
  sendHeartbeat();
}

void sendHeartbeat() {
  setLoRaNormalMode();
  delay(5);

  SubFieldHeartbeatMessage msg;
  msg.type = MSG_SUBFIELD_HEARTBEAT;
  msg.areaId = AREA_ID;
  msg.subFieldId = SUBFIELD_ID;

  LoRaSerial.write((uint8_t*)&msg, sizeof(msg));
  LoRaSerial.flush();
  waitLoRaIdle(800);

  DEBUG_PRINTLN("Sent heartbeat to field device.");
}

// =========================================================================
// WARNING LOGIC
// =========================================================================
void checkWarningCondition() {
  if (!sensorsReady) return;

  float temperature = latestTemperatureX10 / 10.0f;
  bool gasLeak = (latestMq2Value >= GAS_LEAK_MQ2_THRESHOLD);
  bool fireGasOk = (!gasLeak) && (latestMq2Value >= FIRE_GAS_THRESHOLD);
  bool fireTempOk = (temperature >= FIRE_TEMP_THRESHOLD_C);
  bool forestFire = fireGasOk && fireTempOk;

  bool conditionNow = gasLeak || forestFire;
  uint8_t reasonNow = gasLeak ? WARN_GAS_LEAK : WARN_FOREST_FIRE;

  // Reset flag when condition clears
  if (!conditionNow) {
    if (warningActive) DEBUG_PRINTLN("Warning condition cleared.");
    warningActive = false;
    warningAcknowledged = false;
    return;
  }

  if (warningAcknowledged && (millis() - warningAckTime >= WARNING_ACK_TIMEOUT_MS)) {
    DEBUG_PRINTLN("Warning ACK timeout expired – resetting to allow resend.");
    warningAcknowledged = false;
    warningActive = false;   // force a fresh trigger
  }

  // Stop retransmission if already acknowledged by field device
  if (warningAcknowledged) {
    return;
  }

  bool justTriggered = (!warningActive || reasonNow != activeWarningReason);
  bool dueForResend = (millis() - lastWarningSendTime >= WARNING_RESEND_INTERVAL_MS);

  if (justTriggered || dueForResend) {
    DEBUG_PRINT("Sending warning: ");
    DEBUG_PRINTLN(reasonNow == WARN_GAS_LEAK ? "GAS_LEAK" : "FOREST_FIRE");
    sendWarning(reasonNow);
    warningActive = true;
    activeWarningReason = reasonNow;
    lastWarningSendTime = millis();
  }
}

void sendWarning(uint8_t reason) {
  setLoRaNormalMode();
  delay(5);

  WarningMessage msg;
  msg.type = MSG_WARNING;
  msg.areaId = AREA_ID;
  msg.subFieldId = SUBFIELD_ID;
  msg.reason = reason;

  LoRaSerial.write((uint8_t*)&msg, sizeof(msg));
  LoRaSerial.flush();
  waitLoRaIdle(800);

  DEBUG_PRINTLN("Warning message sent.");
}

// =========================================================================
// INCOMING MESSAGE PARSER - STATE MACHINE WITH TIMEOUT
// =========================================================================
void resetRxParser() {
  rxBytesReceived = 0;
  rxExpectedSize = 0;
  rxReadingMessage = false;
}

void checkIncomingMessages() {
  // Reset parser if a partial packet stalls.
  if ((rxBytesReceived > 0 || rxReadingMessage) &&
      (millis() - rxLastByteTime > PARSER_TIMEOUT_MS)) {
    DEBUG_PRINTLN("RX parser timeout - resetting parser.");
    resetRxParser();
  }

  while (LoRaSerial.available() > 0) {
    uint8_t b = LoRaSerial.read();
    rxLastByteTime = millis();

    if (!rxReadingMessage) {
      // Collect header: type + areaId low + areaId high.
      if (rxBytesReceived < 3) {
        rxBuffer[rxBytesReceived++] = b;
      }

      if (rxBytesReceived == 3) {
        uint8_t type = rxBuffer[0];
        uint16_t areaId = (uint16_t)((rxBuffer[2] << 8) | rxBuffer[1]);

        if (areaId != AREA_ID) {
          DEBUG_PRINT("RX: areaId mismatch (got ");
          DEBUG_PRINT(areaId);
          DEBUG_PRINTLN(") - flushing.");
          resetRxParser();
          continue;
        }

        size_t expected = getMessageSize(type);

        if (expected == 0 || expected > MAX_MESSAGE_SIZE) {
          DEBUG_PRINT("RX: unknown/oversized type=");
          DEBUG_PRINT(type);
          DEBUG_PRINTLN(" - flushing.");
          resetRxParser();
          continue;
        }

        rxExpectedSize = expected;

        // Header-only message.
        if (rxExpectedSize == 3) {
          DEBUG_PRINT("RX header-only: type=");
          DEBUG_PRINTLN(type);

          processCompleteMessage(rxBuffer, 3, type, areaId);
          resetRxParser();
          continue;
        }

        // Continue reading payload.
        rxReadingMessage = true;
      }
    } else {
      // Collect payload.
      if (rxBytesReceived < MAX_MESSAGE_SIZE) {
        rxBuffer[rxBytesReceived++] = b;
      } else {
        DEBUG_PRINTLN("RX buffer overflow - resetting parser.");
        resetRxParser();
        continue;
      }

      if (rxBytesReceived == rxExpectedSize) {
        uint8_t type = rxBuffer[0];
        uint16_t areaId = (uint16_t)((rxBuffer[2] << 8) | rxBuffer[1]);

        DEBUG_PRINT("RX: type=");
        DEBUG_PRINT(type);
        DEBUG_PRINT(" (");
        DEBUG_PRINT(rxExpectedSize);
        DEBUG_PRINTLN(" bytes)");

        processCompleteMessage(rxBuffer, rxExpectedSize, type, areaId);
        resetRxParser();
      }
    }
  }
}

// =========================================================================
// PROCESS COMPLETE MESSAGE
// =========================================================================
void processCompleteMessage(uint8_t* buffer, size_t len, uint8_t type, uint16_t areaId) {
  (void)buffer;
  (void)len;
  (void)areaId;

  if (type == MSG_SUBFIELD_DATA_REQUEST) {
    DEBUG_PRINTLN(" -> SUBFIELD_DATA_REQUEST received.");
    scheduleDataReply();
    return;
  }

  if (type == MSG_FIELD_DATA_REQUEST) {
    DEBUG_PRINTLN(" -> FIELD_DATA_REQUEST heard from HQ. Ignored by sub-field.");
    return;
  }

  if (type == MSG_WARNING_ACK) {
    if (len >= sizeof(WarningAckMessage)) {
      WarningAckMessage* ack = (WarningAckMessage*)buffer;
      if (ack->subFieldId == SUBFIELD_ID) {
        DEBUG_PRINTLN(" -> WARNING_ACK for this sub-field. Stopping retransmission.");
        warningAcknowledged = true;
        warningActive = false;
        warningAckTime = millis();   // <-- store the time
      } else {
        DEBUG_PRINTLN(" -> WARNING_ACK for another sub-field, ignored.");
      }
    }
    return;
  }
  
  // Other messages may be heard on the same transparent LoRa channel/address,
  // but this sub-field device does not need to act on them.
  DEBUG_PRINT(" -> ignored type=");
  DEBUG_PRINTLN(type);
}

// =========================================================================
// DATA REPLY SCHEDULER
// =========================================================================
void scheduleDataReply() {
  // If a reply is already scheduled/in flight, ignore extra request copies.
  if (dataReplyPending) {
    DEBUG_PRINTLN("Data reply already pending - ignoring duplicate request.");
    return;
  }

  unsigned long delayMs = (unsigned long)SUBFIELD_ID * DATA_REPLY_DELAY_PER_SUBFIELD_MS;

  dataReplyPending = true;
  dataReplySendTime = millis() + delayMs;

  DEBUG_PRINT("Scheduled data reply in ");
  DEBUG_PRINT(delayMs);
  DEBUG_PRINTLN(" ms.");
}

void handlePendingDataReply() {
  if (!dataReplyPending) return;

  // Wait until scheduled send time.
  if ((long)(millis() - dataReplySendTime) < 0) return;

  sendDataReply();
  dataReplyPending = false;
  DEBUG_PRINTLN("Data reply sent.");
}

// =========================================================================
// SEND DATA REPLY
// =========================================================================
void sendDataReply() {
  // Try to refresh sensors if due.
  readSensorsIfDue();

  setLoRaNormalMode();
  delay(5);

  SubFieldDataReplyMessage msg;
  msg.type = MSG_SUBFIELD_DATA_REPLY;
  msg.areaId = AREA_ID;
  msg.subFieldId = SUBFIELD_ID;
  msg.mq2Ppm = latestMq2Value;
  msg.temperatureX10 = latestTemperatureX10;
  msg.humidityX10 = latestHumidityX10;

  LoRaSerial.write((uint8_t*)&msg, sizeof(msg));
  LoRaSerial.flush();
  waitLoRaIdle(800);

  DEBUG_PRINT("Reply sent: subField=");
  DEBUG_PRINT(msg.subFieldId);
  DEBUG_PRINT(" mq2=");
  DEBUG_PRINT(msg.mq2Ppm);
  DEBUG_PRINT(" temp=");
  DEBUG_PRINT(msg.temperatureX10 / 10.0f);
  DEBUG_PRINT("C hum=");
  DEBUG_PRINT(msg.humidityX10 / 10.0f);
  DEBUG_PRINTLN("%");
}