#include <Arduino.h>
#include "LoRa_E220.h"
#include "esp_sleep.h"
#include "esp_pm.h"

// =========================================================================
// DEBUG — enable serial monitor output (USB)
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
// AREA AND BEACON IDENTITY
// =========================================================================
#define AREA_ID        591
#define BEACON_ORDER     1
#define PREV_BEACON  (BEACON_ORDER - 1)
#define NEXT_BEACON  (BEACON_ORDER + 1)

// =========================================================================
// HARDWARE PINS
// =========================================================================
#define LORA_RX   16
#define LORA_TX   17
#define LORA_M0   12
#define LORA_M1   13
#define LORA_AUX  14

// =========================================================================
// TIMING CONSTANTS (ms)
// =========================================================================
#define HEARTBEAT_INTERVAL       1800000  // 30 min
#define HEARTBEAT_TOLERANCE       300000  // 5 min
#define HEARTBEAT_CHECK_INTERVAL   60000  // 1 min
#define RELAY_DELAY                 100
#define MESSAGE_CLEANUP_INTERVAL  300000  // 5 min
#define IDLE_LOOP_DELAY             100

// =========================================================================
// POWER MANAGEMENT
// =========================================================================
#define POWER_SAVE_MODE true

// =========================================================================
// MEMORY
// =========================================================================
#define MAX_MESSAGE_HISTORY  50
// Largest recognized message on this channel is FIELD_DATA_RESPONSE, sized
// via getMessageSize() below to 32 bytes for this device's purposes.
#define MAX_MESSAGE_SIZE     32
#define PARSER_TIMEOUT_MS    250UL   // FIX: max idle time waiting for rest of a partial packet

// =========================================================================
// MESSAGE TYPES (MUST match field device / sub‑field exactly)
// =========================================================================
enum MessageType {
  MSG_HELP                  = 1,
  MSG_ACK                   = 2,
  MSG_ALARM                 = 3,
  MSG_CANCEL_HELP           = 4,
  MSG_FIELD_DATA_REQUEST    = 5,
  MSG_FIELD_DATA_RESPONSE   = 6,
  MSG_HEARTBEAT             = 7,
  MSG_MISSING_BEACON        = 8,
  MSG_WARNING               = 9,
  MSG_WARNING_FORWARD       = 10,
  MSG_SUBFIELD_HEARTBEAT    = 11,
  MSG_SUBFIELD_DATA_REQUEST = 12,
  MSG_SUBFIELD_DATA_REPLY   = 13,
};

// =========================================================================
// MESSAGE STRUCTURES (packed, exactly as in field device)
// =========================================================================
#pragma pack(push, 1)
struct HelpMessage {
  uint8_t type;
  uint16_t areaId;
  uint8_t disaster;
  uint8_t destruction;
  float latitude;
  float longitude;
};

struct AckMessage {
  uint8_t type;
  uint16_t areaId;
};

struct AlarmMessage {
  uint8_t type;
  uint16_t areaId;
  float latitude;
  float longitude;
};

struct CancelHelpMessage {
  uint8_t type;
  uint16_t areaId;
};

struct FieldDataRequestMessage {
  uint8_t type;
  uint16_t areaId;
};

struct FieldDataResponseMessage {
  uint8_t type;
  uint16_t areaId;
  float latitude;
  float longitude;
  uint8_t payload[21];  // placeholder; real size comes from getMessageSize()
};

struct HeartbeatMessage {
  uint8_t type;
  uint16_t areaId;
  uint8_t senderBeacon;
  uint8_t receiverBeacon;
};

struct MissingBeaconMessage {
  uint8_t type;
  uint16_t areaId;
  uint8_t missingBeacon;
};

struct WarningMessage {
  uint8_t type;
  uint16_t areaId;
  uint8_t subFieldId;
  uint8_t reason;
};

struct WarningForwardMessage {
  uint8_t type;
  uint16_t areaId;
  uint8_t disaster;
  uint8_t subFieldId;
  float latitude;
  float longitude;
};

struct SubFieldHeartbeatMessage {
  uint8_t type;
  uint16_t areaId;
  uint8_t subFieldId;
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
#pragma pack(pop)

#define MAX_SUBFIELDS 3
// =========================================================================
// MESSAGE SIZE FUNCTION
// =========================================================================
size_t getMessageSize(uint8_t type) {
  switch (type) {
    case MSG_HELP:                 return sizeof(HelpMessage);
    case MSG_ACK:                  return sizeof(AckMessage);
    case MSG_ALARM:                return sizeof(AlarmMessage);
    case MSG_CANCEL_HELP:          return sizeof(CancelHelpMessage);
    case MSG_FIELD_DATA_REQUEST:   return sizeof(FieldDataRequestMessage);
    case MSG_FIELD_DATA_RESPONSE:  return 11 + MAX_SUBFIELDS * 7;  // 32 bytes (11 + 3*7)
    case MSG_HEARTBEAT:            return sizeof(HeartbeatMessage);
    case MSG_MISSING_BEACON:       return sizeof(MissingBeaconMessage);
    case MSG_WARNING:              return sizeof(WarningMessage);
    case MSG_WARNING_FORWARD:      return sizeof(WarningForwardMessage);
    case MSG_SUBFIELD_HEARTBEAT:   return sizeof(SubFieldHeartbeatMessage);
    case MSG_SUBFIELD_DATA_REQUEST:return sizeof(SubFieldDataRequestMessage);
    case MSG_SUBFIELD_DATA_REPLY:  return sizeof(SubFieldDataReplyMessage);
    default:                       return 0;
  }
}

// =========================================================================
// RELAY DIRECTION
// =========================================================================
#define RELAY_TO_NEXT 0
#define RELAY_TO_PREV 1
#define RELAY_NONE    2

uint8_t relayDirectionFor(uint8_t type) {
  switch (type) {
    // UT -> MT
    case MSG_HELP:
    case MSG_ALARM:
    case MSG_CANCEL_HELP:
    case MSG_FIELD_DATA_RESPONSE:
    case MSG_MISSING_BEACON:
    case MSG_WARNING_FORWARD:
      return RELAY_TO_NEXT;

    // MT -> UT
    case MSG_ACK:
    case MSG_FIELD_DATA_REQUEST:
      return RELAY_TO_PREV;

    // FIX: MSG_HEARTBEAT is only relevant for a single hop (sender ==
    // PREV_BEACON, receiver == this node's BEACON_ORDER — see
    // updateHeartbeatStatus()). Relaying it further on with senderBeacon/
    // receiverBeacon untouched means the next hop's BEACON_ORDER never
    // matches receiverBeacon, so it's just passed along hop after hop
    // without ever being "useful" again — wasted LoRa airtime. Each node
    // sends its own heartbeat to its own next hop via checkHeartbeatSend(),
    // so a received heartbeat should be consumed locally, not relayed.
    // Local sub-field types (not relayed)
    case MSG_WARNING:
    case MSG_SUBFIELD_HEARTBEAT:
    case MSG_SUBFIELD_DATA_REQUEST:
    case MSG_SUBFIELD_DATA_REPLY:
    case MSG_HEARTBEAT:
    default:
      return RELAY_NONE;
  }
}

const char* messageTypeName(uint8_t type) {
  switch (type) {
    case MSG_HELP:                 return "HELP";
    case MSG_ACK:                  return "ACK";
    case MSG_ALARM:                return "ALARM";
    case MSG_CANCEL_HELP:          return "CANCEL_HELP";
    case MSG_FIELD_DATA_REQUEST:   return "FIELD_DATA_REQUEST";
    case MSG_FIELD_DATA_RESPONSE:  return "FIELD_DATA_RESPONSE";
    case MSG_HEARTBEAT:            return "HEARTBEAT";
    case MSG_MISSING_BEACON:       return "MISSING_BEACON";
    case MSG_WARNING:              return "WARNING";
    case MSG_WARNING_FORWARD:      return "WARNING_FORWARD";
    case MSG_SUBFIELD_HEARTBEAT:   return "SUBFIELD_HEARTBEAT";
    case MSG_SUBFIELD_DATA_REQUEST:return "SUBFIELD_DATA_REQUEST";
    case MSG_SUBFIELD_DATA_REPLY:  return "SUBFIELD_DATA_REPLY";
    default:                       return "UNKNOWN";
  }
}

// =========================================================================
// GLOBALS
// =========================================================================
LoRa_E220 e220(&Serial2, LORA_AUX, LORA_M0, LORA_M1);

struct MessageRecord {
  uint32_t hash;
  unsigned long timestamp;
};
MessageRecord messageHistory[MAX_MESSAGE_HISTORY];
int messageHistoryIndex = 0;

unsigned long prevBeaconLastSeen = 0;
bool prevBeaconActive = false;
bool missingBeaconReported = false;

unsigned long lastHeartbeatSend = 0;
unsigned long lastHeartbeatCheck = 0;
unsigned long lastCleanup = 0;
unsigned long lastMessageActivity = 0;

// Message buffer for partial reads
uint8_t messageBuffer[MAX_MESSAGE_SIZE];
size_t bytesReceived = 0;
size_t expectedSize = 0;
bool readingMessage = false;
unsigned long lastByteTime = 0;   // FIX: parser watchdog timestamp

// =========================================================================
// FUNCTION PROTOTYPES
// FIX: this file was missing this block (unlike field_device_firmware and
// subfield_device_firmware). Without explicit prototypes, functions that
// are called before their definition (e.g. updateHeartbeatStatus(), used
// inside handleMessage() above its own definition) only compile because
// the Arduino IDE/PlatformIO build step auto-generates prototypes for
// .ino-style sketches. If this file is ever compiled as a plain .cpp
// translation unit instead of an Arduino sketch, it would fail to compile
// without this block.
// =========================================================================
uint32_t computeHash(const uint8_t* data, size_t len);
bool isMessageInHistory(uint32_t hash);
void addToMessageHistory(uint32_t hash);
void cleanupOldMessages();

void relayToNext(const uint8_t* buffer, size_t len);
void relayToPrev(const uint8_t* buffer, size_t len);
void handleMessage(uint8_t* buffer, size_t len, uint8_t type, uint16_t areaId);

void updateHeartbeatStatus(uint8_t* buffer);
void checkHeartbeatSend();
void checkHeartbeatStatus();

void resetRxParser();
void checkIncomingMessages();

// =========================================================================
// HASH & HISTORY (dedup)
// =========================================================================
uint32_t computeHash(const uint8_t* data, size_t len) {
  uint32_t hash = 5381;
  for (size_t i = 0; i < len; i++) {
    hash = ((hash << 5) + hash) + data[i];
  }
  return hash;
}

bool isMessageInHistory(uint32_t hash) {
  unsigned long now = millis();
  for (int i = 0; i < MAX_MESSAGE_HISTORY; i++) {
    if (messageHistory[i].hash == hash) {
      if (now - messageHistory[i].timestamp < 60000) { // 1 minute window
        return true;
      }
    }
  }
  return false;
}

void addToMessageHistory(uint32_t hash) {
  messageHistory[messageHistoryIndex].hash = hash;
  messageHistory[messageHistoryIndex].timestamp = millis();
  messageHistoryIndex = (messageHistoryIndex + 1) % MAX_MESSAGE_HISTORY;
  lastMessageActivity = millis();
}

void cleanupOldMessages() {
  if (millis() - lastCleanup < MESSAGE_CLEANUP_INTERVAL) return;
  lastCleanup = millis();
  unsigned long now = millis();
  for (int i = 0; i < MAX_MESSAGE_HISTORY; i++) {
    if (messageHistory[i].hash != 0 && (now - messageHistory[i].timestamp > 600000)) {
      messageHistory[i].hash = 0;
      messageHistory[i].timestamp = 0;
    }
  }
}

// =========================================================================
// MESSAGE RELAY
// =========================================================================
void relayToNext(const uint8_t* buffer, size_t len) {
  delay(RELAY_DELAY);
  Serial2.write(buffer, len);
  DEBUG_PRINT("  -> relayed toward MT (");
  DEBUG_PRINT(len);
  DEBUG_PRINTLN(" bytes)");
}

void relayToPrev(const uint8_t* buffer, size_t len) {
  delay(RELAY_DELAY);
  Serial2.write(buffer, len);
  DEBUG_PRINT("  -> relayed toward UT (");
  DEBUG_PRINT(len);
  DEBUG_PRINTLN(" bytes)");
}

void handleMessage(uint8_t* buffer, size_t len, uint8_t type, uint16_t areaId) {
  // Reset our own heartbeat timer whenever we actually relay a packet
  // (the channel is busy, so postpone our own heartbeat transmission).
  uint8_t dir = relayDirectionFor(type);
  if (dir != RELAY_NONE) {
    lastHeartbeatSend = millis();
  }

  // --- Determine if this packet came from the Field Device (UT) ---
  // These types are sent by the UT (or originate from UT).
  bool fromUT = false;
  switch (type) {
    case MSG_HELP:
    case MSG_ALARM:
    case MSG_CANCEL_HELP:
    case MSG_FIELD_DATA_RESPONSE:
    case MSG_MISSING_BEACON:
    case MSG_WARNING_FORWARD:
    case MSG_SUBFIELD_DATA_REQUEST:   // UT asks sub-fields
    case MSG_HEARTBEAT:               // handled separately below
      fromUT = true;
      break;

    // These types come from Sub-Field devices, NOT the UT
    case MSG_WARNING:
    case MSG_SUBFIELD_HEARTBEAT:
    case MSG_SUBFIELD_DATA_REPLY:
      fromUT = false;
      break;

    default:
      fromUT = false;   // safety
      break;
  }

  // --- Update previous-hop liveness if this is a UT-originated message ---
  if (fromUT) {
    // For heartbeats, we need to check sender/receiver – do it in the dedicated function
    if (type == MSG_HEARTBEAT) {
      updateHeartbeatStatus(buffer);   // this function already validates and updates prevBeaconLastSeen
    } else {
      // Non-heartbeat from UT – we can safely consider UT alive
      prevBeaconLastSeen = millis();
      prevBeaconActive = true;
      missingBeaconReported = false;
    }
  }

  // --- Relay the packet based on direction ---
  if (dir == RELAY_TO_NEXT) {
    relayToNext(buffer, len);
  } else if (dir == RELAY_TO_PREV) {
    relayToPrev(buffer, len);
  } else {
    DEBUG_PRINTLN("  -> local type, not relayed");
  }
}

// =========================================================================
// HEARTBEAT HANDLING
// =========================================================================
void updateHeartbeatStatus(uint8_t* buffer) {
  HeartbeatMessage* msg = (HeartbeatMessage*)buffer;
  if (msg->areaId != AREA_ID) return;
  if (msg->receiverBeacon != BEACON_ORDER) return;
  if (msg->senderBeacon != PREV_BEACON) return;

  prevBeaconLastSeen = millis();
  prevBeaconActive = true;
  missingBeaconReported = false;
  lastHeartbeatSend = millis();  // reset own timer to avoid sending redundant heartbeat
  DEBUG_PRINTLN("  -> received heartbeat from prev beacon, marked alive");
}

void checkHeartbeatSend() {
  if (millis() - lastHeartbeatSend < HEARTBEAT_INTERVAL) return;
  lastHeartbeatSend = millis();

  HeartbeatMessage msg;
  msg.type = MSG_HEARTBEAT;
  msg.areaId = AREA_ID;
  msg.senderBeacon = BEACON_ORDER;
  msg.receiverBeacon = NEXT_BEACON;

  Serial2.write((uint8_t*)&msg, sizeof(msg));
  DEBUG_PRINTLN("Sent own heartbeat to next hop.");
}

void checkHeartbeatStatus() {
  if (millis() - lastHeartbeatCheck < HEARTBEAT_CHECK_INTERVAL) return;
  lastHeartbeatCheck = millis();

  if (!prevBeaconActive) return;

  unsigned long timeout = HEARTBEAT_INTERVAL + HEARTBEAT_TOLERANCE;
  unsigned long sinceLastSeen = millis() - prevBeaconLastSeen;

  DEBUG_PRINT("Heartbeat check: prev beacon last seen ");
  DEBUG_PRINT(sinceLastSeen / 1000UL);
  DEBUG_PRINT("s ago (timeout=");
  DEBUG_PRINT(timeout / 1000UL);
  DEBUG_PRINTLN("s)");

  if (sinceLastSeen > timeout && !missingBeaconReported) {
    MissingBeaconMessage msg;
    msg.type = MSG_MISSING_BEACON;
    msg.areaId = AREA_ID;
    msg.missingBeacon = PREV_BEACON;
    Serial2.write((uint8_t*)&msg, sizeof(msg));
    missingBeaconReported = true;
    prevBeaconActive = false;
    DEBUG_PRINT("!!! Previous beacon (order=");
    DEBUG_PRINT(PREV_BEACON);
    DEBUG_PRINTLN(") reported MISSING to MT");
  }
}

// =========================================================================
// INCOMING MESSAGE PARSER (state machine)
// =========================================================================
void resetRxParser() {   // FIX: parser watchdog reset
  bytesReceived = 0;
  expectedSize = 0;
  readingMessage = false;
}

void checkIncomingMessages() {
  // FIX: if a partial packet stalls (RF interference / packet loss cuts a
  // message mid-way), don't wait forever for bytes that will never come —
  // this device would otherwise stop processing incoming messages entirely
  // until rebooted.
  if ((bytesReceived > 0 || readingMessage) &&
      (millis() - lastByteTime > PARSER_TIMEOUT_MS)) {
    DEBUG_PRINTLN("RX parser timeout - resetting parser.");
    resetRxParser();
  }

  while (Serial2.available()) {
    if (!readingMessage) {
      // Read the first 3 bytes (header)
      if (bytesReceived < 3) {
        messageBuffer[bytesReceived++] = Serial2.read();
        lastByteTime = millis();

        if (bytesReceived == 3) {
          uint8_t type = messageBuffer[0];
          uint16_t areaId = (messageBuffer[2] << 8) | messageBuffer[1];

          // --- Filter by AREA_ID (drop anything that doesn't match) ---
          if (areaId != AREA_ID) {
            DEBUG_PRINT("RX: areaId mismatch (got ");
            DEBUG_PRINT(areaId);
            DEBUG_PRINTLN(") – flushing");
            bytesReceived = 0;
            continue;
          }

          expectedSize = getMessageSize(type);
          if (expectedSize == 0 || expectedSize > MAX_MESSAGE_SIZE) {
            DEBUG_PRINT("Unrecognized/oversized type=");
            DEBUG_PRINT(type);
            DEBUG_PRINTLN(" – flushing");
            bytesReceived = 0;
            continue;
          }

          // If the message is only 3 bytes (e.g., ACK, CANCEL_HELP, etc.)
          if (expectedSize == 3) {
            uint32_t msgHash = computeHash(messageBuffer, expectedSize);
            if (!isMessageInHistory(msgHash)) {
              addToMessageHistory(msgHash);
              handleMessage(messageBuffer, expectedSize, type, areaId);
            } else {
              DEBUG_PRINTLN("  -> duplicate (header only), dropped");
            }
            bytesReceived = 0;
            continue;
          }

          readingMessage = true; // we need more bytes
        }
      } else {
        // Should not happen (header already complete); read and discard.
        Serial2.read();
        lastByteTime = millis();
      }
    } else {
      // Reading the rest of the message
      messageBuffer[bytesReceived++] = Serial2.read();
      lastByteTime = millis();

      if (bytesReceived == expectedSize) {
        uint8_t type = messageBuffer[0];
        uint16_t areaId = (messageBuffer[2] << 8) | messageBuffer[1];

        uint32_t msgHash = computeHash(messageBuffer, expectedSize);
        if (!isMessageInHistory(msgHash)) {
          addToMessageHistory(msgHash);
          DEBUG_PRINT("Packet: type=");
          DEBUG_PRINT(messageTypeName(type));
          DEBUG_PRINT(" (");
          DEBUG_PRINT(expectedSize);
          DEBUG_PRINTLN(" bytes)");

          // FIX: removed the duplicate updateHeartbeatStatus() call that
          // used to happen here. handleMessage() already calls it (via the
          // fromUT / MSG_HEARTBEAT branch) for every heartbeat, so calling
          // it here too just reprocessed the same packet and double-logged
          // it for no functional benefit.
          handleMessage(messageBuffer, expectedSize, type, areaId);
        } else {
          DEBUG_PRINTLN("  -> duplicate, dropped");
        }

        bytesReceived = 0;
        expectedSize = 0;
        readingMessage = false;
      }
    }
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
  Serial.println("=== RELAY NODE (RN) BOOT ===");
  Serial.print("AREA_ID=");
  Serial.print(AREA_ID);
  Serial.print("  BEACON_ORDER=");
  Serial.print(BEACON_ORDER);
  Serial.print("  PREV_BEACON=");
  Serial.print(PREV_BEACON);
  Serial.print("  NEXT_BEACON=");
  Serial.println(NEXT_BEACON);
#endif

  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  pinMode(LORA_AUX, INPUT);
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);

  Serial2.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);
  DEBUG_PRINTLN("Initializing LoRa E220...");
  if (e220.begin() != 1) {
    DEBUG_PRINTLN("!!! LoRa init FAILED – halting.");
    while (1) delay(1000);
  }
  DEBUG_PRINTLN("LoRa init OK.");

  // Configure LoRa module (same as UT/sub-field)
  // FIX: this is the exact failure that produced "RX: areaId mismatch (got
  // 2048)" on a sub-field in the field - when getConfiguration() fails here,
  // the beacon used to just log a warning and keep running on its
  // factory-default ADDH/ADDL/CHAN, which doesn't match the rest of the
  // network (0x12/0x34/56). That's not a benign fallback, it silently
  // breaks this beacon's relay link. Retry a few times, then halt (same
  // pattern as the e220.begin() failure above) instead of running with a
  // mismatched radio config.
  bool loraConfigApplied = false;
  for (int attempt = 1; attempt <= 5 && !loraConfigApplied; attempt++) {
    ResponseStructContainer c = e220.getConfiguration();
    if (c.status.code == 1) {
      Configuration* cfg = (Configuration*)c.data;
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
      c.close();
      e220.resetModule();
      loraConfigApplied = true;
      DEBUG_PRINTLN("LoRa config applied.");
    } else {
      DEBUG_PRINT("!!! getConfiguration() failed (attempt ");
      DEBUG_PRINT(attempt);
      DEBUG_PRINTLN("/5) - retrying.");
      c.close();
      delay(500);
    }
  }
  if (!loraConfigApplied) {
    DEBUG_PRINTLN("!!! LoRa config could not be applied - halting (address/channel would mismatch the rest of the network).");
    while (1) delay(1000);
  }

  for (int i = 0; i < MAX_MESSAGE_HISTORY; i++) {
    messageHistory[i].hash = 0;
    messageHistory[i].timestamp = 0;
  }

  prevBeaconLastSeen = millis();
  prevBeaconActive = true;
  missingBeaconReported = false;
  lastHeartbeatSend = millis();
  lastHeartbeatCheck = millis();
  lastCleanup = millis();
  lastMessageActivity = millis();

  if (POWER_SAVE_MODE) {
    esp_pm_config_esp32_t pm_config;
    pm_config.max_freq_mhz = 80;
    pm_config.min_freq_mhz = 10;
    pm_config.light_sleep_enable = false;
    esp_pm_configure(&pm_config);
  }

  DEBUG_PRINTLN("=== RN setup complete ===");
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
  checkIncomingMessages();
  checkHeartbeatSend();
  checkHeartbeatStatus();
  cleanupOldMessages();

  if (POWER_SAVE_MODE)  delay(IDLE_LOOP_DELAY);
  else                  delay(10);
}