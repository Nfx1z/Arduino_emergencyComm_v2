// =========================================================================
// FIELD DEVICE FIRMWARE — TFT/Touch version (rev. 5)
// -------------------------------------------------------------------------
// Changes from rev. 4:
//  - Replaced simple message polling with a state‑machine parser that reads
//    header (3 bytes), validates areaId & type, then reads the rest of the
//    packet. This handles fragmented packets and prevents mis‑parsing.
//  - Added hash‑based deduplication (60‑second window) to avoid
//    reprocessing the same packet if it echoes back.
//  - All other logic (screen, BLE, GPS, warnings, sub‑field polling, etc.)
//    is unchanged.
// =========================================================================

#include <Arduino_GFX_Library.h>
#include <XPT2046_Touchscreen.h>
#include <TinyGPS++.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "LoRa_E220.h"
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
// DEVICE IDENTITY
// =========================================================================
#define AREA_ID       591
#define NUM_SUBFIELDS 2

// =========================================================================
// PIN DEFINITIONS
// =========================================================================
// --- TFT Display (SPI) ---
#define TFT_SCK   18
#define TFT_MOSI  23
#define TFT_MISO  -1
#define TFT_CS    5
#define TFT_DC    2
#define TFT_RST   4
#define TFT_BL    27

// --- Touchscreen (shares SCK/MOSI with TFT) ---
#define TOUCH_CS  19
#define TOUCH_DO  21

// --- Wake sensor (TTP223 touch pad) ---
#define WAKE_PIN            22
#define WAKE_HOLD_DURATION  5000UL

// --- Piezo buzzer ---
#define BUZZER_PIN               25
#define BUZZER_WARNING_DURATION  10000UL

// --- LoRa E220 control pins ---
#define LORA_RX   16
#define LORA_TX   17
#define LORA_M0   12
#define LORA_M1   13
#define LORA_AUX  14

// --- GPS ---
#define GPS_RX    33
#define GPS_TX    32

// =========================================================================
// TOUCH CALIBRATION
// =========================================================================
#define TOUCH_MIN_X 400
#define TOUCH_MAX_X 3700
#define TOUCH_MIN_Y 400
#define TOUCH_MAX_Y 3700
#define TOUCH_INVERT_X true
#define TOUCH_INVERT_Y true
#define TOUCH_SWAP_XY  false

// =========================================================================
// BLE SERVICE
// =========================================================================
#define BLE_NAME              "Field Device"
#define SERVICE_UUID          "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_WRITE_UUID       "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_NOTIFY_UUID      "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"

// =========================================================================
// TIMING CONSTANTS (ms)
// =========================================================================
#define CANCEL_HELP_HOLD_MS      5000UL
#define WARNING_CONFIRM_HOLD_MS  5000UL
#define ACK_TIMEOUT               60000UL
#define CANCEL_HELP_TIMEOUT      300000UL
#define WARNING_TIMEOUT          600000UL
#define HEARTBEAT_INTERVAL      1800000UL
#define HEARTBEAT_TOLERANCE      300000UL
#define GPS_CHECK_INTERVAL         5000UL
#define INACTIVITY_TIMEOUT        60000UL
#define SCREEN_INACTIVITY_TIMEOUT 60000UL
#define STATUS_DISPLAY_TIME        4000UL

#define MAX_SUBFIELDS                    3
#define SUBFIELD_HEARTBEAT_TIMEOUT       (HEARTBEAT_INTERVAL + HEARTBEAT_TOLERANCE)
#define SUBFIELD_MISSING_DISPLAY_TIME    8000UL

#define FIELD_DATA_COLLECTION_WINDOW_MS  30000UL
#define MOVEMENT_THRESHOLD  5.0

// =========================================================================
// DISASTER / DAMAGE OPTIONS
// =========================================================================
const char* DISASTER_NAMES[] = {
  "Gempa Bumi",
  "Banjir",
  "Tanah Longsor",
  "Kebakaran",
  "Kebocoran Gas",
  "Lain-lainnya",
};
#define DISASTER_KEBAKARAN     3
#define DISASTER_KEBOCORAN_GAS 4
const int NUM_DISASTERS = sizeof(DISASTER_NAMES) / sizeof(DISASTER_NAMES[0]);

const char* DESTRUCTION_NAMES[] = {
  "Ringan",
  "Menengah",
  "Besar",
  "Masif",
  "Unknown",
};
const int NUM_DESTRUCTIONS = sizeof(DESTRUCTION_NAMES) / sizeof(DESTRUCTION_NAMES[0]);

const char* TITLE_DISASTER    = "PILIH JENIS BENCANA";
const char* TITLE_DESTRUCTION = "PILIH TINGKAT DAMPAK";
#define VISIBLE_OPTION_ROWS 3

// =========================================================================
// MESSAGE STRUCTURES
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

struct SubFieldDataEntry {
  uint8_t subFieldId;
  uint16_t mq2Ppm;
  int16_t temperatureX10;
  int16_t humidityX10;
};

struct FieldDataResponseMessage {
  uint8_t type;
  uint16_t areaId;
  float latitude;
  float longitude;
  SubFieldDataEntry subFields[MAX_SUBFIELDS];
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

struct WarningAckMessage {
  uint8_t type;      // = 14
  uint16_t areaId;
  uint8_t subFieldId;
};

#pragma pack(pop)

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
  MSG_WARNING_ACK           = 14,      
};

enum WarningReason {
  WARN_GAS_LEAK    = 0,
  WARN_FOREST_FIRE = 1,
};

// FIX: WarningForwardMessage.disaster is shared by two different meanings:
// (a) a real disaster index into DISASTER_NAMES[] (0..NUM_DISASTERS-1), used
//     by forwardWarningToHQ(), and (b) a "sub-field missing" marker, used by
//     handleSubfieldMissingInPoll(). The old value 0 collided with
//     DISASTER_NAMES[0]="Gempa Bumi", so HQ decoding the field against
//     DISASTER_NAMES could misread "sub-field missing" as an earthquake
//     report. 0xFF is outside the valid disaster-index range and can never
//     collide with a real disaster. FWD_REASON_GAS_LEAK/FOREST_FIRE were
//     defined but never used (forwardWarningToHQ() uses DISASTER_KEBAKARAN /
//     DISASTER_KEBOCORAN_GAS directly) and have been removed.
#define FWD_REASON_SUBFIELD_MISSING  0xFF

// =========================================================================
// SYSTEM STATE
// =========================================================================
enum SystemState {
  STATE_SLEEP,
  STATE_IDLE_TAP,
  STATE_SELECT_DISASTER,
  STATE_SELECT_DESTRUCTION,
  STATE_CONFIRM_REPORT,
  STATE_WAITING_ACK,
  STATE_SEND_SUCCESS,
  STATE_SEND_FAILED,
  STATE_CANCEL_AVAILABLE,
  STATE_WAITING_CANCEL_ACK,
  STATE_CANCEL_CONFIRMED,   // FIX: lets "Pesan Dibatalkan" stay visible until statusDisplayUntil elapses
  STATE_WARNING_CONFIRM,
  STATE_WARNING_SENT,
  STATE_SUBFIELD_MISSING,
};

enum HoldAction {
  HOLD_NONE,
  HOLD_CANCEL_HELP,
  HOLD_WARNING_CANCEL,
  HOLD_WARNING_SEND,
};

// =========================================================================
// COLORS
// =========================================================================
#define COLOR_BG          RGB565_BLACK
#define COLOR_BOX         0x39C7
#define COLOR_BOX_SEL     0x051F
#define COLOR_BORDER      RGB565_WHITE
#define COLOR_TEXT        RGB565_WHITE
#define COLOR_OK          RGB565_GREEN
#define COLOR_KEMBALI     RGB565_RED
#define COLOR_WARNING     0xFD20
#define COLOR_SCROLL_ON   0x7BEF
#define COLOR_SCROLL_OFF  0x2104
#define COLOR_PROGRESS    0xFFE0

// =========================================================================
// GLOBAL OBJECTS
// =========================================================================
Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO);
Arduino_GFX *gfx = new Arduino_ILI9488_18bit(bus, TFT_RST, 1, false);
XPT2046_Touchscreen ts(TOUCH_CS);

TinyGPSPlus gps;
LoRa_E220 e220(&Serial2, LORA_AUX, LORA_M0, LORA_M1);
HardwareSerial SerialGPS(1);

BLEServer* pServer = nullptr;
BLECharacteristic* pCharWrite = nullptr;
BLECharacteristic* pCharNotify = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;
// True only once the phone has actually written the CCCD descriptor to
// subscribe to notifications. sendInitialSync() must not fire before this
// is true, or the very first sync packet gets dropped by the BLE stack
// because nothing is subscribed yet (see NotifyDescriptorCallbacks below).
volatile bool notificationsEnabled = false;
String pendingBLECommand = "";
bool hasBLECommand = false;

// =========================================================================
// LAYOUT (computed in setup)
// =========================================================================
int SCREEN_W, SCREEN_H, ROW_H, SCROLL_COL_W;

// =========================================================================
// SYSTEM / SELECTION STATE
// =========================================================================
SystemState currentState = STATE_SLEEP;
bool screenOn = false;

int selectedDisaster = 0;
int selectedDestruction = 0;
int cursorIndex = 0;
int scrollOffset = 0;

double currentLat = 0.0, currentLon = 0.0;
double lastLat = 0.0, lastLon = 0.0;

bool systemLocked = false;
bool messageFromApp = false;
bool ackReceived = false;
bool alarmSent = false;
bool cancelAckReceived = false;

unsigned long lastMessageTime = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastGPSCheck = 0;
unsigned long lastActivityTime = 0;
unsigned long statusDisplayUntil = 0;
unsigned long ackReceivedTime = 0;
unsigned long connectionEstablishedTime = 0;
bool initialStatusSent = false;

bool wakeHoldActive = false;
unsigned long wakeHoldStart = 0;

bool buzzerActive = false;
unsigned long buzzerOffAt = 0;

// --- pending warning ---
uint8_t pendingWarningSubField = 0;
uint8_t pendingWarningReason = 0;
unsigned long warningReceivedTime = 0;

// --- sub-field heartbeat registry ---
struct SubFieldSlot {
  bool active;
  bool missing;
  uint8_t id;
  unsigned long lastSeen;
};
SubFieldSlot subFields[MAX_SUBFIELDS];
// FIX: was a single uint8_t id, which meant a second sub-field going
// missing while the first one's notice was still on screen silently
// overwrote it - the first missing sub-field was never actually shown/
// acknowledged. Now a bitmask (bit i-1 set = sub-field i is missing in the
// CURRENT notice) so simultaneous notices merge into one combined screen,
// e.g. "Sub-Field 1 & 2 tidak merespon", instead of one replacing the other.
uint16_t missingNoticeMask = 0;
unsigned long missingNoticeUntil = 0;

// --- HQ field-data pull state ---
bool collectingFieldData = false;
unsigned long collectionStartTime = 0;
// FIX: timestamp of when the last collection cycle STARTED (kept even after
// the cycle finishes). Used by startFieldDataCollection() to refuse a new
// HQ request that arrives too soon after the previous one — see
// MIN_FIELD_DATA_REQUEST_INTERVAL_MS below.
unsigned long lastCollectionStartTime = 0;

struct SubFieldCollectSlot {
  bool replied;
  uint16_t mq2Ppm;
  int16_t temperatureX10;
  int16_t humidityX10;
};
SubFieldCollectSlot collectSlots[NUM_SUBFIELDS];

// --- touch tracking ---
bool wasTouched = false;
bool holdRegionActive = false;
unsigned long holdRegionStartTime = 0;
HoldAction holdRegionAction = HOLD_NONE;

// =========================================================================
// MESSAGE DEDUPLICATION & ROBUST PARSER
// =========================================================================
#define MAX_MESSAGE_HISTORY 20
#define DEDUP_WINDOW_MS     60000

// FIX: minimum spacing enforced between two field-data collection cycles
// (see startFieldDataCollection()). Sub-field replies are sent within ~1-2s
// of the request, so as long as two requests are spaced further apart than
// the RX dedup window, a sub-field's reply from cycle N can never hash-match
// (and get wrongly dropped as "duplicate") against its own reply from
// cycle N+1, even if the sensor readings happen to be identical. The extra
// 5s on top of DEDUP_WINDOW_MS is just margin for stagger + processing time.
#define MIN_FIELD_DATA_REQUEST_INTERVAL_MS (DEDUP_WINDOW_MS + 5000UL)

struct MessageRecord {
  uint32_t hash;
  unsigned long timestamp;
};
MessageRecord messageHistory[MAX_MESSAGE_HISTORY];
int messageHistoryIndex = 0;

// FieldDataResponseMessage (type/areaId/lat/lon + MAX_SUBFIELDS entries of
// 7 bytes each) is the largest message this device parses: 11 + 3*7 = 32.
#define MAX_MESSAGE_SIZE 32
#define PARSER_TIMEOUT_MS 250UL   // FIX: max idle time waiting for rest of a partial packet
uint8_t rxBuffer[MAX_MESSAGE_SIZE];
size_t rxBytesReceived = 0;
size_t rxExpectedSize = 0;
bool rxReadingMessage = false;
unsigned long rxLastByteTime = 0;   // FIX: parser watchdog timestamp

// =========================================================================
// FUNCTION PROTOTYPES
// =========================================================================
void setLoRaNormalMode();
void initBLE();
void handleBLEConnection();
void sendBLENotification(String status);
void updateGPS();
bool isGPSReady();
void checkMovementDetection();
void handleWakeSensor();
void handleTouch();
void mapTouch(TS_Point p, int &x, int &y);
void onTouchDown(int x, int y);
void checkHold();
void onTouchUp();
void moveCursor(int direction, const char** options, int count);
void ensureVisible(int idx, int count);
void handleBack();
void handleSelectOK();
void sendHelpMessage();
void sendAlarmMessage();
void handleCancelHelp();
void handleRestart();
void resetRxParser();           // FIX: parser watchdog reset
void checkIncomingMessages();   // state-machine header/payload parser
void processCompleteMessage(uint8_t* buffer, size_t len, uint8_t type, uint16_t areaId);
void handleAckReceived();
void handleWarningReceived(uint8_t subFieldId, uint8_t reason);
void forwardWarningToHQ();
void cancelWarning();
void sendInitialSync();
void checkWarningTimeout();
void registerSubFieldHeartbeat(uint8_t id);
void checkSubFieldHeartbeats();
void triggerSubFieldMissing(uint8_t id);
void showSubfieldMissingNotice(uint8_t id);
void checkHeartbeat();
void resetHeartbeatTimer();
void updateActivity();
void checkAutoReturnToIdle();
void checkCancelWindow();
void updateStateMachine();
void updateScreenPower();
void startBuzzer(unsigned long durationMs);
void updateBuzzer();
void handleAppHelpRequest(String command);
void startFieldDataCollection();
void broadcastSubfieldDataRequest();
void handleSubfieldDataReply(uint8_t subFieldId, uint16_t mq2Ppm, int16_t temperatureX10, int16_t humidityX10);
void sendFieldDataResponseToHQ();
void handleSubfieldMissingInPoll(uint8_t id);
void checkFieldDataCollection();
void drawTitleRow(int row, const char* label, int colW);
void drawOptionRow(int row, const char* label, bool highlighted, bool empty, int colW);
void drawScrollButtons(int count);
void drawBottomRow(const char* leftLabel, const char* rightLabel);
void drawSelectionScreen(const char** options, int count, const char* title);
void drawConfirmScreen();
void drawIdleTapScreen();
void drawWaitingScreen(const char* text);
void drawStatusFull(const char* text);
void drawCancelAvailableScreen();
void drawWarningScreen(uint8_t reason, uint8_t subFieldId);
void drawSubFieldMissingScreen(uint16_t missingMask);
void buildMissingIdsLabel(uint16_t mask, char* out, size_t outSize);
void drawHoldProgress(int x0, int width, unsigned long held, unsigned long total);

// =========================================================================
// HASH & DEDUP FUNCTIONS
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
      if (now - messageHistory[i].timestamp < DEDUP_WINDOW_MS) {
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
}

// =========================================================================
// MESSAGE SIZE FUNCTION
// =========================================================================
size_t getMessageSize(uint8_t type) {
  switch (type) {
    case MSG_HELP:                  return sizeof(HelpMessage);
    case MSG_ACK:                   return sizeof(AckMessage);
    case MSG_ALARM:                 return sizeof(AlarmMessage);
    case MSG_CANCEL_HELP:           return sizeof(CancelHelpMessage);
    case MSG_FIELD_DATA_REQUEST:    return sizeof(FieldDataRequestMessage);
    case MSG_FIELD_DATA_RESPONSE:   return sizeof(FieldDataResponseMessage);
    case MSG_HEARTBEAT:             return sizeof(HeartbeatMessage);
    case MSG_MISSING_BEACON:        return sizeof(MissingBeaconMessage);
    case MSG_WARNING:               return sizeof(WarningMessage);
    case MSG_WARNING_FORWARD:       return sizeof(WarningForwardMessage);
    case MSG_SUBFIELD_HEARTBEAT:    return sizeof(SubFieldHeartbeatMessage);
    case MSG_SUBFIELD_DATA_REQUEST: return sizeof(SubFieldDataRequestMessage);
    case MSG_SUBFIELD_DATA_REPLY:   return sizeof(SubFieldDataReplyMessage);
    case MSG_WARNING_ACK:           return sizeof(WarningAckMessage);
    default:                        return 0;
  }
}

// =========================================================================
// LoRa CONTROL
// =========================================================================
void setLoRaNormalMode() {
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  delay(10);
}

// =========================================================================
// SETUP
// =========================================================================
void setup() {
#if DEBUG_ENABLED
  Serial.begin(DEBUG_BAUD);
  delay(300);
  Serial.println();
  Serial.println("=== FIELD DEVICE (UT) FIRMWARE BOOT ===");
  Serial.print("AREA_ID=");
  Serial.print(AREA_ID);
  Serial.print("  NUM_SUBFIELDS=");
  Serial.println(NUM_SUBFIELDS);
#endif

  delay(500);
  esp_pm_config_esp32_t pm_config = {
    .max_freq_mhz = 80,
    .min_freq_mhz = 10,
    .light_sleep_enable = false
  };
  esp_pm_configure(&pm_config);

  delay(500);

  pinMode(WAKE_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  // FIX: noTone() here (before tone() is ever called) tries to stop a LEDC
  // channel that hasn't been attached yet - ESP32's tone()/noTone() are
  // implemented on top of LEDC, and LEDC only gets configured for this pin
  // on the first real tone() call (see startBuzzer()). That produced the
  // boot-time "E (...) ledc: ledc_set_duty/ledc_update_duty: LEDC is not
  // initialized" log lines. A plain digitalWrite LOW achieves the same
  // "buzzer silent at boot" goal without touching LEDC at all.
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  pinMode(LORA_AUX, INPUT);
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, LOW);

  if (!gfx->begin(10000000)) {
    while (1);
  }
  gfx->fillScreen(COLOR_BG);

  SPI.begin(TFT_SCK, TOUCH_DO, TFT_MOSI, TOUCH_CS);
  ts.begin(SPI);
  ts.setRotation(1);

  SCREEN_W = gfx->width();
  SCREEN_H = gfx->height();
  ROW_H = SCREEN_H / 5;
  SCROLL_COL_W = SCREEN_W / 5;

  for (int i = 0; i < MAX_SUBFIELDS; i++) {
    bool isKnownSubField = (i < NUM_SUBFIELDS);
    subFields[i].active   = isKnownSubField;
    subFields[i].missing  = false;
    subFields[i].id       = isKnownSubField ? (i + 1) : 0;
    subFields[i].lastSeen = millis();
  }

  for (int i = 0; i < NUM_SUBFIELDS; i++) {
    collectSlots[i].replied = false;
    collectSlots[i].mq2Ppm = 0;
    collectSlots[i].temperatureX10 = 0;
    collectSlots[i].humidityX10 = 0;
  }

  // Initialize message history
  for (int i = 0; i < MAX_MESSAGE_HISTORY; i++) {
    messageHistory[i].hash = 0;
    messageHistory[i].timestamp = 0;
  }

  delay(500);
  Serial2.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);
  DEBUG_PRINTLN("Initializing LoRa E220...");
  if (e220.begin() != 1) {
    DEBUG_PRINTLN("!!! LoRa init FAILED – halting.");
    while (1);
  }
  DEBUG_PRINTLN("LoRa init OK.");

  delay(500);
  // FIX: getConfiguration()/setConfiguration() used to just log a warning
  // and continue on failure, leaving the module on its factory-default
  // ADDH/ADDL/CHAN - incompatible with every other node on the network
  // (which are all set to 0x12/0x34/56). That silently breaks this node's
  // comms (received bytes look like garbage / wrong areaId) instead of
  // failing loudly. Now retry a few times, and halt (same pattern as the
  // e220.begin() failure above) if it still won't take.
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
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  delay(500);
  initBLE();

  lastHeartbeatTime = millis();
  lastActivityTime = millis();

  setLoRaNormalMode();
  currentState = STATE_SLEEP;
  DEBUG_PRINTLN("=== UT setup complete ===");
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
  if (hasBLECommand) {
    hasBLECommand = false;
    String command = pendingBLECommand;
    pendingBLECommand = "";
    if (systemLocked && currentState != STATE_CANCEL_AVAILABLE &&
        currentState != STATE_WARNING_CONFIRM) {
      // ignore
    } else {
      if (command.startsWith("HELP,")) {
        handleAppHelpRequest(command);
        updateActivity();
      } else if (command == "CANCEL") {
        if (currentState == STATE_CANCEL_AVAILABLE) {
          handleCancelHelp();
          updateActivity();
        } else if (currentState == STATE_WARNING_CONFIRM) {
          cancelWarning();
          updateActivity();
        }
      } else if (command == "FORWARD_WARNING") {
        // App-initiated equivalent of holding the on-screen "KIRIM" button.
        // Only honored while a warning is actually pending confirmation, so
        // it can't do anything once the state has already moved on (e.g.
        // the touchscreen already cancelled/forwarded it, or it timed out).
        if (currentState == STATE_WARNING_CONFIRM) {
          forwardWarningToHQ();
          updateActivity();
        }
      } else if (command == "RESTART") {
        handleRestart();
      }
    }
  }

  updateGPS();
  checkMovementDetection();
  handleBLEConnection();
  checkHeartbeat();
  checkSubFieldHeartbeats();
  checkIncomingMessages();
  checkFieldDataCollection();
  updateBuzzer();
  handleWakeSensor();
  handleTouch();
  updateScreenPower();

  checkAutoReturnToIdle();
  checkCancelWindow();
  checkWarningTimeout();
  updateStateMachine();

  if (currentState == STATE_SEND_SUCCESS && millis() > statusDisplayUntil) {
    currentState = STATE_CANCEL_AVAILABLE;
    ackReceivedTime = millis();
    drawCancelAvailableScreen();
  }

  if (currentState == STATE_WARNING_SENT && millis() > statusDisplayUntil) {
    handleRestart();
  }

  // FIX: give the "Pesan Dibatalkan" screen its full STATUS_DISPLAY_TIME
  // before returning to idle, instead of overwriting it immediately.
  if (currentState == STATE_CANCEL_CONFIRMED && millis() > statusDisplayUntil) {
    systemLocked = false;
    handleRestart();
  }

  if (currentState == STATE_SEND_FAILED && millis() > statusDisplayUntil) {
    handleRestart();
  }

  if (currentState == STATE_SUBFIELD_MISSING && millis() > missingNoticeUntil) {
    handleRestart();
  }

  delay(10);
}

// =========================================================================
// BLE
// =========================================================================
class MyServerCallbacks : public BLEServerCallbacks {
public:
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    updateActivity();
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    notificationsEnabled = false;
  }
};

// Fires when the phone writes the CCCD (0x2902) descriptor — i.e. the
// moment it actually enables notifications, which happens well after raw
// connect (MTU negotiation + service discovery + the descriptor write
// itself all have to complete first). This is the real "ready to receive"
// signal; a fixed post-connect delay is just a guess and can fire early.
class NotifyDescriptorCallbacks : public BLEDescriptorCallbacks {
public:
  void onWrite(BLEDescriptor* pDescriptor) override {
    uint8_t* value = pDescriptor->getValue();
    notificationsEnabled = (pDescriptor->getLength() > 0) && (value[0] & 0x01);
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
public:
  void onWrite(BLECharacteristic* pCharacteristic) override {
    uint8_t* pData = pCharacteristic->getData();
    size_t length = pCharacteristic->getLength();
    if (length == 0 || length > 127) return;
    char buffer[128];
    memcpy(buffer, pData, length);
    buffer[length] = '\0';
    pendingBLECommand = String(buffer);
    hasBLECommand = true;
  }
};

void initBLE() {
  String name = String(BLE_NAME) + "_" + String(AREA_ID);
  BLEDevice::init(name.c_str());
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharWrite = pService->createCharacteristic(CHAR_WRITE_UUID, BLECharacteristic::PROPERTY_WRITE);
  pCharWrite->setCallbacks(new MyCallbacks());
  pCharNotify = pService->createCharacteristic(CHAR_NOTIFY_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  BLE2902* pNotifyDescriptor = new BLE2902();
  pNotifyDescriptor->setCallbacks(new NotifyDescriptorCallbacks());
  pCharNotify->addDescriptor(pNotifyDescriptor);
  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setMinInterval(800);
  pAdvertising->setMaxInterval(1600);
  BLEDevice::startAdvertising();
  DEBUG_PRINTLN("BLE initialized.");
}

void handleBLEConnection() {
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
    initialStatusSent = false;
  }

  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    connectionEstablishedTime = millis();
    initialStatusSent = false;
  }

  // Primary path: send as soon as the phone has actually subscribed
  // (CCCD write observed). This is normally quick — no more waiting out
  // a fixed 1s guess — and, critically, it can no longer fire before
  // anyone is listening.
  if (deviceConnected && !initialStatusSent && notificationsEnabled) {
    sendInitialSync();
    initialStatusSent = true;
  }

  // Safety net: if a notify() call ever gets dropped by the radio right
  // after subscribing (rare, but BLE links are lossy), don't leave the
  // phone stuck — retry periodically. Still gated on notificationsEnabled
  // so this can never fire before the phone has actually subscribed.
  else if (deviceConnected && !initialStatusSent && notificationsEnabled &&
           (millis() - connectionEstablishedTime > 4000) &&
           (millis() - connectionEstablishedTime) % 1000 < 50) {
    sendInitialSync();
  }
}

// Sent exactly once per BLE connection, ~1s after connect. Unlike the
// plain state-word notifications used for live transitions, this carries
// the full context of an in-progress report or warning (disaster,
// destruction, coordinates, remaining cancel-window time, or the warning's
// sub-field id) so the app can rebuild its screen to match the device
// instead of just guessing "idle" after a reconnect or app relaunch.
void sendInitialSync() {
  if (!systemLocked) {
    sendBLENotification("READY");
    return;
  }

  switch (currentState) {
    case STATE_WAITING_ACK:
    case STATE_SEND_SUCCESS:
    case STATE_CANCEL_AVAILABLE:
    case STATE_WAITING_CANCEL_ACK:
    case STATE_CANCEL_CONFIRMED:
    case STATE_SEND_FAILED: {
      String stateWord;
      switch (currentState) {
        case STATE_WAITING_ACK:        stateWord = "WAITING"; break;
        case STATE_SEND_SUCCESS:
        case STATE_CANCEL_AVAILABLE:   stateWord = "SUCCESS"; break;
        case STATE_WAITING_CANCEL_ACK: stateWord = "CANCEL_WAITING"; break;
        case STATE_CANCEL_CONFIRMED:   stateWord = "CANCELLED"; break;
        case STATE_SEND_FAILED:        stateWord = "FAILED"; break;
        default:                       stateWord = "READY"; break;
      }

      long remainingMs = 0;
      if (currentState == STATE_CANCEL_AVAILABLE) {
        long elapsed = (long)(millis() - ackReceivedTime);
        remainingMs = (long)CANCEL_HELP_TIMEOUT - elapsed;
        if (remainingMs < 0) remainingMs = 0;
      }

      String payload = "SYNC_HELP," + stateWord + "," +
                        String(selectedDisaster) + "," + String(selectedDestruction) + "," +
                        String(currentLat, 6) + "," + String(currentLon, 6) + "," +
                        String(remainingMs);
      sendBLENotification(payload);
      break;
    }
    case STATE_WARNING_CONFIRM: {
      String stateWord = (pendingWarningReason == WARN_FOREST_FIRE) ? "WARNING_FIRE" : "WARNING_GAS";
      String payload = "SYNC_WARNING," + stateWord + "," + String(pendingWarningSubField);
      sendBLENotification(payload);
      break;
    }
    default:
      // systemLocked should only be true in the states handled above; if
      // that ever changes, tell the app explicitly rather than staying
      // silent so it doesn't sit there assuming the device is idle.
      sendBLENotification("SYNC_UNKNOWN");
      break;
  }
}

void sendBLENotification(String status) {
  if (deviceConnected && pCharNotify) {
    pCharNotify->setValue(status.c_str());
    pCharNotify->notify();
  }
}

// =========================================================================
// GPS & MOVEMENT
// =========================================================================
void updateGPS() {
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read()) && gps.location.isValid()) {
      currentLat = gps.location.lat();
      currentLon = gps.location.lng();
    }
  }
}

bool isGPSReady() {
  return gps.location.isValid() &&
         gps.location.age() <= 30000 &&
         gps.satellites.value() >= 4 &&
         gps.hdop.hdop() <= 500;
}

void checkMovementDetection() {
  if (millis() - lastGPSCheck < GPS_CHECK_INTERVAL) return;
  lastGPSCheck = millis();

  if (!isGPSReady()) return;
  if (lastLat == 0.0 || lastLon == 0.0) {
    lastLat = currentLat;
    lastLon = currentLon;
    return;
  }

  double distance = TinyGPSPlus::distanceBetween(lastLat, lastLon, currentLat, currentLon);
  // FIX: the device spends most of its time in STATE_SLEEP (screen off
  // after SCREEN_INACTIVITY_TIMEOUT), so requiring STATE_IDLE_TAP meant the
  // movement/theft alarm almost never fired in the situation it matters
  // most - being moved while nobody is actively using the touchscreen.
  bool alarmableState = (currentState == STATE_IDLE_TAP || currentState == STATE_SLEEP);
  if (distance > MOVEMENT_THRESHOLD && !alarmSent && !systemLocked && alarmableState) {
    DEBUG_PRINTLN("Movement detected, sending alarm.");
    updateActivity();
    sendAlarmMessage();
    alarmSent = true;
  }

  lastLat = currentLat;
  lastLon = currentLon;
}

// =========================================================================
// WAKE SENSOR (TTP223)
// =========================================================================
void handleWakeSensor() {
  if (screenOn) { wakeHoldActive = false; return; }

  bool pressed = digitalRead(WAKE_PIN);
  if (pressed) {
    if (!wakeHoldActive) {
      wakeHoldActive = true;
      wakeHoldStart = millis();
    } else if (millis() - wakeHoldStart >= WAKE_HOLD_DURATION) {
      wakeHoldActive = false;
      screenOn = true;
      digitalWrite(TFT_BL, HIGH);
      currentState = STATE_IDLE_TAP;
      updateActivity();
      drawIdleTapScreen();
      DEBUG_PRINTLN("Wake sensor: screen on.");
    }
  } else {
    wakeHoldActive = false;
  }
}

// =========================================================================
// TOUCH HANDLING
// =========================================================================
void mapTouch(TS_Point p, int &x, int &y) {
  int rawX = p.x;
  int rawY = p.y;
  if (TOUCH_SWAP_XY) { int t = rawX; rawX = rawY; rawY = t; }

  if (TOUCH_INVERT_X) x = map(rawX, TOUCH_MIN_X, TOUCH_MAX_X, SCREEN_W, 0);
  else                x = map(rawX, TOUCH_MIN_X, TOUCH_MAX_X, 0, SCREEN_W);

  if (TOUCH_INVERT_Y) y = map(rawY, TOUCH_MIN_Y, TOUCH_MAX_Y, SCREEN_H, 0);
  else                y = map(rawY, TOUCH_MIN_Y, TOUCH_MAX_Y, 0, SCREEN_H);

  x = constrain(x, 0, SCREEN_W - 1);
  y = constrain(y, 0, SCREEN_H - 1);
}

void handleTouch() {
  if (!screenOn) { wasTouched = false; return; }

  bool touched = ts.touched();

  if (touched && !wasTouched) {
    TS_Point p = ts.getPoint();
    int x, y;
    mapTouch(p, x, y);
    onTouchDown(x, y);
  } else if (touched && wasTouched) {
    checkHold();
  } else if (!touched && wasTouched) {
    onTouchUp();
  }

  wasTouched = touched;
}

void onTouchDown(int x, int y) {
  updateActivity();
  int row = y / ROW_H;

  switch (currentState) {

    case STATE_IDLE_TAP:
      currentState = STATE_SELECT_DISASTER;
      cursorIndex = 0;
      scrollOffset = 0;
      drawSelectionScreen(DISASTER_NAMES, NUM_DISASTERS, TITLE_DISASTER);
      break;

    case STATE_SELECT_DISASTER:
    case STATE_SELECT_DESTRUCTION: {
      const char** options = (currentState == STATE_SELECT_DISASTER) ? DISASTER_NAMES : DESTRUCTION_NAMES;
      int count = (currentState == STATE_SELECT_DISASTER) ? NUM_DISASTERS : NUM_DESTRUCTIONS;

      if (row == 0) {
        // title row, not interactive
      } else if (row < 4) {
        int colW = SCREEN_W - SCROLL_COL_W;
        if (x >= colW) {
          int startY = ROW_H;
          int halfH = (VISIBLE_OPTION_ROWS * ROW_H) / 2;
          int direction = (y < startY + halfH) ? -1 : 1;
          moveCursor(direction, options, count);
        }
      } else {
        if (x < SCREEN_W / 2) handleBack();
        else handleSelectOK();
      }
      break;
    }

    case STATE_CONFIRM_REPORT:
      if (row >= 4) {
        if (x < SCREEN_W / 2) {
          currentState = STATE_SELECT_DESTRUCTION;
          cursorIndex = selectedDestruction;
          scrollOffset = 0;
          ensureVisible(cursorIndex, NUM_DESTRUCTIONS);
          drawSelectionScreen(DESTRUCTION_NAMES, NUM_DESTRUCTIONS, TITLE_DESTRUCTION);
        } else {
          if (!isGPSReady()) {
            drawStatusFull("GPS Belum Siap");
            sendBLENotification("GPS_NOT_READY");
          } else {
            sendHelpMessage();
          }
        }
      }
      break;

    case STATE_CANCEL_AVAILABLE:
      if (row >= 3) {
        holdRegionActive = true;
        holdRegionStartTime = millis();
        holdRegionAction = HOLD_CANCEL_HELP;
      }
      break;

    case STATE_WARNING_CONFIRM:
      if (row >= 4) {
        holdRegionActive = true;
        holdRegionStartTime = millis();
        holdRegionAction = (x < SCREEN_W / 2) ? HOLD_WARNING_CANCEL : HOLD_WARNING_SEND;
      }
      break;

    case STATE_SUBFIELD_MISSING:
      handleRestart();
      break;

    default:
      break;
  }
}

void checkHold() {
  if (!holdRegionActive) return;
  unsigned long held = millis() - holdRegionStartTime;

  if (holdRegionAction == HOLD_CANCEL_HELP && currentState == STATE_CANCEL_AVAILABLE) {
    drawHoldProgress(0, SCREEN_W, held, CANCEL_HELP_HOLD_MS);
    if (held >= CANCEL_HELP_HOLD_MS) {
      holdRegionActive = false;
      handleCancelHelp();
    }
  } else if (holdRegionAction == HOLD_WARNING_CANCEL && currentState == STATE_WARNING_CONFIRM) {
    drawHoldProgress(0, SCREEN_W / 2, held, WARNING_CONFIRM_HOLD_MS);
    if (held >= WARNING_CONFIRM_HOLD_MS) {
      holdRegionActive = false;
      cancelWarning();
    }
  } else if (holdRegionAction == HOLD_WARNING_SEND && currentState == STATE_WARNING_CONFIRM) {
    drawHoldProgress(SCREEN_W / 2, SCREEN_W / 2, held, WARNING_CONFIRM_HOLD_MS);
    if (held >= WARNING_CONFIRM_HOLD_MS) {
      holdRegionActive = false;
      forwardWarningToHQ();
    }
  } else {
    holdRegionActive = false;
  }
}

void onTouchUp() {
  holdRegionActive = false;
  holdRegionAction = HOLD_NONE;
}

void moveCursor(int direction, const char** options, int count) {
  int maxIdx = count - 1;
  int newIdx = constrain(cursorIndex + direction, 0, maxIdx);
  if (newIdx == cursorIndex) return;

  int oldIdx = cursorIndex;
  int oldScrollOffset = scrollOffset;
  cursorIndex = newIdx;
  ensureVisible(cursorIndex, count);

  int colW = SCREEN_W - SCROLL_COL_W;

  if (scrollOffset != oldScrollOffset) {
    for (int i = 0; i < VISIBLE_OPTION_ROWS; i++) {
      int absIdx = scrollOffset + i;
      bool empty = absIdx >= count;
      bool hl = (absIdx == cursorIndex);
      drawOptionRow(i + 1, empty ? "" : options[absIdx], hl, empty, colW);
    }
  } else {
    int oldRow = oldIdx - scrollOffset;
    int newRow = cursorIndex - scrollOffset;
    if (oldRow >= 0 && oldRow < VISIBLE_OPTION_ROWS) drawOptionRow(oldRow + 1, options[oldIdx], false, false, colW);
    if (newRow >= 0 && newRow < VISIBLE_OPTION_ROWS) drawOptionRow(newRow + 1, options[cursorIndex], true, false, colW);
  }

  drawScrollButtons(count);
}

void ensureVisible(int idx, int count) {
  int maxOffset = max(0, count - VISIBLE_OPTION_ROWS);
  if (idx < scrollOffset) scrollOffset = idx;
  else if (idx > scrollOffset + (VISIBLE_OPTION_ROWS - 1)) scrollOffset = idx - (VISIBLE_OPTION_ROWS - 1);
  scrollOffset = constrain(scrollOffset, 0, maxOffset);
}

void handleBack() {
  if (currentState == STATE_SELECT_DISASTER) {
    currentState = STATE_IDLE_TAP;
    drawIdleTapScreen();
  } else if (currentState == STATE_SELECT_DESTRUCTION) {
    currentState = STATE_SELECT_DISASTER;
    cursorIndex = selectedDisaster;
    scrollOffset = 0;
    ensureVisible(cursorIndex, NUM_DISASTERS);
    drawSelectionScreen(DISASTER_NAMES, NUM_DISASTERS, TITLE_DISASTER);
  }
}

void handleSelectOK() {
  if (currentState == STATE_SELECT_DISASTER) {
    selectedDisaster = cursorIndex;
    cursorIndex = 0;
    scrollOffset = 0;
    currentState = STATE_SELECT_DESTRUCTION;
    drawSelectionScreen(DESTRUCTION_NAMES, NUM_DESTRUCTIONS, TITLE_DESTRUCTION);
  } else if (currentState == STATE_SELECT_DESTRUCTION) {
    selectedDestruction = cursorIndex;
    currentState = STATE_CONFIRM_REPORT;
    drawConfirmScreen();
  }
}

// =========================================================================
// MESSAGING
// =========================================================================
void sendHelpMessage() {
  setLoRaNormalMode();
  delay(10);

  HelpMessage msg;
  msg.type = MSG_HELP;
  msg.areaId = AREA_ID;
  msg.disaster = selectedDisaster;
  msg.destruction = selectedDestruction;
  msg.latitude = currentLat;
  msg.longitude = currentLon;
  Serial2.write((uint8_t*)&msg, sizeof(msg));

  currentState = STATE_WAITING_ACK;
  systemLocked = true;
  lastMessageTime = millis();
  ackReceived = false;
  drawWaitingScreen("MENUNGGU ACK...");
  sendBLENotification("WAITING");
  resetHeartbeatTimer();
}

void sendAlarmMessage() {
  setLoRaNormalMode();
  delay(10);

  AlarmMessage msg;
  msg.type = MSG_ALARM;
  msg.areaId = AREA_ID;
  msg.latitude = currentLat;
  msg.longitude = currentLon;
  Serial2.write((uint8_t*)&msg, sizeof(msg));

  currentState = STATE_WAITING_ACK;
  systemLocked = true;
  lastMessageTime = millis();
  ackReceived = false;
  screenOn = true;
  digitalWrite(TFT_BL, HIGH);
  drawWaitingScreen("ALARM TERKIRIM");
  sendBLENotification("ALARM_WAITING");
  resetHeartbeatTimer();
}

void handleCancelHelp() {
  if (currentState != STATE_CANCEL_AVAILABLE) return;

  setLoRaNormalMode();
  delay(10);

  CancelHelpMessage msg;
  msg.type = MSG_CANCEL_HELP;
  msg.areaId = AREA_ID;
  Serial2.write((uint8_t*)&msg, sizeof(msg));

  currentState = STATE_WAITING_CANCEL_ACK;
  systemLocked = true;
  lastMessageTime = millis();
  cancelAckReceived = false;
  drawWaitingScreen("BATAL MENUNGGU...");
  sendBLENotification("CANCEL_WAITING");
  resetHeartbeatTimer();
}

void handleRestart() {
  systemLocked = false;
  messageFromApp = false;
  ackReceived = false;
  cancelAckReceived = false;
  alarmSent = false;
  missingNoticeMask = 0;   // FIX: clear so the next notice starts fresh
  currentState = STATE_IDLE_TAP;
  drawIdleTapScreen();
  sendBLENotification("READY");
}

// =========================================================================
// INCOMING MESSAGE PARSER (state machine)
// =========================================================================
void resetRxParser() {   // FIX: parser watchdog reset
  rxBytesReceived = 0;
  rxExpectedSize = 0;
  rxReadingMessage = false;
}

void checkIncomingMessages() {
  // FIX: if a partial packet stalls (RF interference / packet loss cuts a
  // message mid-way), don't wait forever for bytes that will never come.
  if ((rxBytesReceived > 0 || rxReadingMessage) &&
      (millis() - rxLastByteTime > PARSER_TIMEOUT_MS)) {
    DEBUG_PRINTLN("RX parser timeout - resetting parser.");
    resetRxParser();
  }

  while (Serial2.available()) {
    if (!rxReadingMessage) {
      if (rxBytesReceived < 3) {
        rxBuffer[rxBytesReceived++] = Serial2.read();
        rxLastByteTime = millis();

        while (rxBytesReceived == 3) {
          uint8_t type = rxBuffer[0];
          uint16_t areaId = (rxBuffer[2] << 8) | rxBuffer[1];

          if (areaId != AREA_ID) {
            DEBUG_PRINT("RX: areaId mismatch (got ");
            DEBUG_PRINT(areaId);
            DEBUG_PRINTLN(") - resync (shift 1 byte)");

            // Sliding-window resync: drop only the oldest byte and
            // re-test, instead of discarding the whole 3-byte window.
            // Prevents the parser from staying permanently out of phase
            // after a single corrupted/extra byte.
            rxBuffer[0] = rxBuffer[1];
            rxBuffer[1] = rxBuffer[2];
            rxBytesReceived = 2;
            break; // go back to filling byte 3 from the stream
          }

          rxExpectedSize = getMessageSize(type);
          if (rxExpectedSize == 0 || rxExpectedSize > MAX_MESSAGE_SIZE) {
            DEBUG_PRINT("RX: unknown/oversized type=");
            DEBUG_PRINT(type);
            DEBUG_PRINTLN(" - resync (shift 1 byte)");
            rxBuffer[0] = rxBuffer[1];
            rxBuffer[1] = rxBuffer[2];
            rxBytesReceived = 2;
            break;
          }

          if (rxExpectedSize == 3) {
            uint32_t hash = computeHash(rxBuffer, 3);

            // MSG_SUBFIELD_DATA_REQUEST is broadcast by this device itself
            // (see broadcastSubfieldDataRequest()), so it can echo straight
            // back into our own receiver on the shared LoRa channel. Skip
            // adding it to the dedup history so a genuine repeat broadcast
            // isn't blocked; it's ignored either way in
            // processCompleteMessage() (default case).
            bool bypassDedup = (type == MSG_SUBFIELD_DATA_REQUEST || type == MSG_ACK || type == MSG_WARNING);

            if (bypassDedup || !isMessageInHistory(hash)) {
                if (!bypassDedup) {
                    addToMessageHistory(hash);
                }
                processCompleteMessage(rxBuffer, 3, type, areaId);
            } else {
                DEBUG_PRINTLN("RX: duplicate header-only, dropped");
            }

            rxBytesReceived = 0;
            break;
          }

          rxReadingMessage = true;
          break;
        }
      }
    } else {
      rxBuffer[rxBytesReceived++] = Serial2.read();
      rxLastByteTime = millis();

      if (rxBytesReceived == rxExpectedSize) {
        uint8_t type = rxBuffer[0];
        uint16_t areaId = (rxBuffer[2] << 8) | rxBuffer[1];
        uint32_t hash = computeHash(rxBuffer, rxExpectedSize);

        // Dedup runs normally for all types here, including
        // MSG_SUBFIELD_DATA_REPLY (real retransmits within one cycle are
        // still caught by collectSlots[idx].replied in
        // handleSubfieldDataReply). The cross-cycle false-negative case —
        // two collection cycles close enough together that a sub-field's
        // unchanged reading hashes the same in both — is prevented upstream
        // in startFieldDataCollection() by refusing to start a new cycle
        // within MIN_FIELD_DATA_REQUEST_INTERVAL_MS of the last one, so this
        // generic dedup never sees two genuinely-different-cycle replies
        // close enough together to collide.
        if (!isMessageInHistory(hash)) {
            addToMessageHistory(hash);

            DEBUG_PRINT("RX: type=");
            DEBUG_PRINT(type);
            DEBUG_PRINT(" (");
            DEBUG_PRINT(rxExpectedSize);
            DEBUG_PRINTLN(" bytes)");

            processCompleteMessage(rxBuffer, rxExpectedSize, type, areaId);
        } else {
            DEBUG_PRINTLN("RX: duplicate packet, dropped");
        }
        rxBytesReceived = 0;
        rxExpectedSize = 0;
        rxReadingMessage = false;
      }
    }
  }
}

// =========================================================================
// PROCESS A COMPLETE MESSAGE
// =========================================================================
void processCompleteMessage(uint8_t* buffer, size_t len, uint8_t type, uint16_t areaId) {
  switch (type) {
    case MSG_ACK: {
      DEBUG_PRINTLN(" -> ACK");
      handleAckReceived();
      resetHeartbeatTimer();
      break;
    }

    case MSG_FIELD_DATA_REQUEST: {
      DEBUG_PRINTLN(" -> FIELD_DATA_REQUEST from HQ");
      startFieldDataCollection();
      resetHeartbeatTimer();
      break;
    }

    case MSG_WARNING: {
      WarningMessage* wm = (WarningMessage*)buffer;
      DEBUG_PRINT(" -> WARNING subField=");
      DEBUG_PRINT(wm->subFieldId);
      DEBUG_PRINT(" reason=");
      DEBUG_PRINTLN(wm->reason == WARN_FOREST_FIRE ? "FOREST_FIRE" : "GAS_LEAK");
      handleWarningReceived(wm->subFieldId, wm->reason);
      resetHeartbeatTimer();
      break;
    }

    case MSG_SUBFIELD_HEARTBEAT: {
      SubFieldHeartbeatMessage* shm = (SubFieldHeartbeatMessage*)buffer;
      DEBUG_PRINT(" -> SUBFIELD_HEARTBEAT from subField=");
      DEBUG_PRINTLN(shm->subFieldId);
      registerSubFieldHeartbeat(shm->subFieldId);
      resetHeartbeatTimer();
      break;
    }

    case MSG_SUBFIELD_DATA_REPLY: {
      SubFieldDataReplyMessage* sdm = (SubFieldDataReplyMessage*)buffer;
      DEBUG_PRINT(" -> SUBFIELD_DATA_REPLY subField=");
      DEBUG_PRINT(sdm->subFieldId);
      DEBUG_PRINT(" mq2=");
      DEBUG_PRINT(sdm->mq2Ppm);
      DEBUG_PRINT(" tempX10=");
      DEBUG_PRINT(sdm->temperatureX10);
      DEBUG_PRINT(" humX10=");
      DEBUG_PRINTLN(sdm->humidityX10);
      handleSubfieldDataReply(sdm->subFieldId, sdm->mq2Ppm, sdm->temperatureX10, sdm->humidityX10);
      registerSubFieldHeartbeat(sdm->subFieldId);
      resetHeartbeatTimer();
      break;
    }

    default: {
      // Ignore any other type (e.g. MSG_HEARTBEAT, MSG_MISSING_BEACON,
      // MSG_SUBFIELD_DATA_REQUEST, etc.) — not relevant to this device.
      DEBUG_PRINTLN(" -> ignored (not handled)");
      break;
    }
  }
}

// =========================================================================
// ACK HANDLING
// =========================================================================
void handleAckReceived() {
  if (currentState == STATE_WAITING_ACK) {
    ackReceived = true;
    ackReceivedTime = millis();
    currentState = STATE_SEND_SUCCESS;
    drawStatusFull("Terkirim!");
    sendBLENotification("SUCCESS");
  } else if (currentState == STATE_WAITING_CANCEL_ACK) {
    cancelAckReceived = true;
    // FIX: don't jump to STATE_IDLE_TAP / call handleRestart() here, or the
    // "Pesan Dibatalkan" screen gets overwritten by the idle screen in the
    // same function call and is never actually seen. Move to a dedicated
    // state and let loop() return to idle only after statusDisplayUntil
    // has elapsed (same pattern as STATE_SEND_SUCCESS).
    currentState = STATE_CANCEL_CONFIRMED;
    drawStatusFull("Pesan Dibatalkan");
    sendBLENotification("CANCELLED");
    statusDisplayUntil = millis() + STATUS_DISPLAY_TIME;
  }
}

// =========================================================================
// WARNING HANDLING (with duplicate suppression & critical-flow blocking)
// =========================================================================
void handleWarningReceived(uint8_t subFieldId, uint8_t reason) {
  // --- Do not interrupt critical flows ---
  if (currentState == STATE_WAITING_ACK ||
      currentState == STATE_WAITING_CANCEL_ACK ||
      currentState == STATE_SEND_SUCCESS ||
      currentState == STATE_CANCEL_AVAILABLE ||
      currentState == STATE_SEND_FAILED ||
      currentState == STATE_WARNING_SENT) {
    DEBUG_PRINTLN("WARNING: ignored because system is in critical flow");
    registerSubFieldHeartbeat(subFieldId);
    return;
  }

  // --- Duplicate suppression ---
  if (currentState == STATE_WARNING_CONFIRM &&
      subFieldId == pendingWarningSubField &&
      reason == pendingWarningReason) {
    DEBUG_PRINTLN("WARNING: duplicate from same source – suppressed");
    registerSubFieldHeartbeat(subFieldId);
    return;
  }

  // --- NEW: Send dedicated warning ACK to sub-field ---
  setLoRaNormalMode();
  delay(5);
  WarningAckMessage ack;
  ack.type = MSG_WARNING_ACK;
  ack.areaId = AREA_ID;
  ack.subFieldId = subFieldId;
  Serial2.write((uint8_t*)&ack, sizeof(ack));
  DEBUG_PRINTLN("Sent WARNING_ACK to sub-field");

  // --- Continue normal warning handling ---
  pendingWarningSubField = subFieldId;
  pendingWarningReason = reason;
  warningReceivedTime = millis();
  DEBUG_PRINTLN("WARNING: new warning, starting 10-min auto-forward countdown");

  registerSubFieldHeartbeat(subFieldId);

  currentState = STATE_WARNING_CONFIRM;
  systemLocked = true;
  screenOn = true;
  digitalWrite(TFT_BL, HIGH);
  updateActivity();

  drawWarningScreen(reason, subFieldId);
  startBuzzer(BUZZER_WARNING_DURATION);
  sendBLENotification(reason == WARN_FOREST_FIRE ? "WARNING_FIRE" : "WARNING_GAS");
}

void forwardWarningToHQ() {
  setLoRaNormalMode();
  delay(10);

  WarningForwardMessage msg;
  msg.type = MSG_WARNING_FORWARD;
  msg.areaId = AREA_ID;
  msg.disaster = (pendingWarningReason == WARN_FOREST_FIRE) ? DISASTER_KEBAKARAN : DISASTER_KEBOCORAN_GAS;
  msg.subFieldId = pendingWarningSubField;
  msg.latitude = currentLat;
  msg.longitude = currentLon;
  Serial2.write((uint8_t*)&msg, sizeof(msg));

  systemLocked = false;
  currentState = STATE_WARNING_SENT;
  drawStatusFull("Terkirim ke HQ");
  statusDisplayUntil = millis() + STATUS_DISPLAY_TIME;
  sendBLENotification("WARNING_FORWARDED");
}

void cancelWarning() {
  systemLocked = false;
  currentState = STATE_IDLE_TAP;
  drawIdleTapScreen();
  sendBLENotification("WARNING_DISMISSED");
  updateActivity();
}

void checkWarningTimeout() {
  if (currentState == STATE_WARNING_CONFIRM &&
      (millis() - warningReceivedTime >= WARNING_TIMEOUT)) {
    forwardWarningToHQ();
  }
}

// =========================================================================
// SUB-FIELD HEARTBEAT REGISTRY
// =========================================================================
void registerSubFieldHeartbeat(uint8_t id) {
  int freeSlot = -1;
  for (int i = 0; i < MAX_SUBFIELDS; i++) {
    if (subFields[i].active && subFields[i].id == id) {
      subFields[i].lastSeen = millis();
      if (subFields[i].missing) {
        subFields[i].missing = false;
        sendBLENotification("SUBFIELD_RECOVERED:" + String(id));
      }
      return;
    }
    if (!subFields[i].active && freeSlot == -1) freeSlot = i;
  }
  if (freeSlot != -1) {
    subFields[freeSlot].active = true;
    subFields[freeSlot].missing = false;
    subFields[freeSlot].id = id;
    subFields[freeSlot].lastSeen = millis();
  }
}

void checkSubFieldHeartbeats() {
  for (int i = 0; i < MAX_SUBFIELDS; i++) {
    if (!subFields[i].active || subFields[i].missing) continue;
    if (millis() - subFields[i].lastSeen > SUBFIELD_HEARTBEAT_TIMEOUT) {
      subFields[i].missing = true;
      triggerSubFieldMissing(subFields[i].id);
    }
  }
}

void showSubfieldMissingNotice(uint8_t id) {
  startBuzzer(BUZZER_WARNING_DURATION);
  sendBLENotification("SUBFIELD_MISSING:" + String(id));

  if (id < 1 || id > MAX_SUBFIELDS) return;

  // FIX: if the missing-notice screen is already showing, merge this id
  // into it instead of only redrawing for the latest one - otherwise a
  // second sub-field going missing silently erases the first one's notice.
  bool alreadyShowing = (currentState == STATE_SUBFIELD_MISSING);
  bool safeToInterrupt = (currentState == STATE_IDLE_TAP || currentState == STATE_SLEEP);
  if (!alreadyShowing && !safeToInterrupt) return;

  if (!alreadyShowing) {
    missingNoticeMask = 0;
  }
  missingNoticeMask |= (uint16_t)(1 << (id - 1));

  screenOn = true;
  digitalWrite(TFT_BL, HIGH);
  currentState = STATE_SUBFIELD_MISSING;
  missingNoticeUntil = millis() + SUBFIELD_MISSING_DISPLAY_TIME;
  drawSubFieldMissingScreen(missingNoticeMask);
  updateActivity();
}

void triggerSubFieldMissing(uint8_t id) {
  showSubfieldMissingNotice(id);

  setLoRaNormalMode();
  delay(10);
  // FIX (point 5): MSG_MISSING_BEACON is reserved for beacon-to-beacon
  // liveness reporting between relay nodes (see relay_node_firmware's
  // checkHeartbeatStatus()). A missing SUB-FIELD device is a different
  // event and must use the same protocol as the other "sub-field missing"
  // path (handleSubfieldMissingInPoll()) — MSG_WARNING_FORWARD with
  // FWD_REASON_SUBFIELD_MISSING — so HQ only has to listen for one message
  // type/format for this event, regardless of which detection path fired.
  WarningForwardMessage msg;
  msg.type = MSG_WARNING_FORWARD;
  msg.areaId = AREA_ID;
  msg.disaster = FWD_REASON_SUBFIELD_MISSING;
  msg.subFieldId = id;
  msg.latitude = currentLat;
  msg.longitude = currentLon;
  Serial2.write((uint8_t*)&msg, sizeof(msg));
  DEBUG_PRINT("Heartbeat missing: subField=");
  DEBUG_PRINTLN(id);
}

// =========================================================================
// HQ FIELD-DATA PULL
// =========================================================================
void startFieldDataCollection() {
  if (collectingFieldData) return;

  // FIX: reject a new HQ FIELD_DATA_REQUEST if it arrives too soon after the
  // previous collection cycle started. Without this, two cycles close
  // together (e.g. HQ retrying) can produce identical SUBFIELD_DATA_REPLY
  // bytes (unchanged sensor readings) from the same sub-field within the
  // 60s RX dedup window — the second cycle's reply then gets silently
  // dropped as a "duplicate" before it ever reaches handleSubfieldDataReply(),
  // and that sub-field gets wrongly reported as missing even though it
  // actually replied. See MIN_FIELD_DATA_REQUEST_INTERVAL_MS.
  if (lastCollectionStartTime != 0 &&
      (millis() - lastCollectionStartTime) < MIN_FIELD_DATA_REQUEST_INTERVAL_MS) {
    DEBUG_PRINTLN("FIELD_DATA_REQUEST ignored - too soon after last collection cycle.");
    return;
  }

  for (int i = 0; i < NUM_SUBFIELDS; i++) {
    collectSlots[i].replied = false;
    collectSlots[i].mq2Ppm = 0;
    collectSlots[i].temperatureX10 = 0;
    collectSlots[i].humidityX10 = 0;
  }

  collectingFieldData = true;
  collectionStartTime = millis();
  lastCollectionStartTime = collectionStartTime;
  broadcastSubfieldDataRequest();
}

void broadcastSubfieldDataRequest() {
  setLoRaNormalMode();
  delay(10);

  SubFieldDataRequestMessage msg;
  msg.type = MSG_SUBFIELD_DATA_REQUEST;
  msg.areaId = AREA_ID;
  Serial2.write((uint8_t*)&msg, sizeof(msg));
  DEBUG_PRINTLN("Broadcast SUBFIELD_DATA_REQUEST");
}

void handleSubfieldDataReply(uint8_t subFieldId, uint16_t mq2Ppm, int16_t temperatureX10, int16_t humidityX10) {
  if (subFieldId < 1 || subFieldId > NUM_SUBFIELDS) return;
  int idx = subFieldId - 1;

  if (collectSlots[idx].replied) {
    DEBUG_PRINT(">>> subField=");
    DEBUG_PRINT(subFieldId);
    DEBUG_PRINTLN(" reply already recorded this cycle, ignoring retransmit");
    return;
  }

  DEBUG_PRINT(">>> handleSubfieldDataReply: subField=");
  DEBUG_PRINTLN(subFieldId);
  collectSlots[idx].replied = true;
  collectSlots[idx].mq2Ppm = mq2Ppm;
  collectSlots[idx].temperatureX10 = temperatureX10;
  collectSlots[idx].humidityX10 = humidityX10;
}

void sendFieldDataResponseToHQ() {
  setLoRaNormalMode();
  delay(10);

  FieldDataResponseMessage msg;
  msg.type = MSG_FIELD_DATA_RESPONSE;
  msg.areaId = AREA_ID;
  msg.latitude = currentLat;
  msg.longitude = currentLon;

  for (int i = 0; i < MAX_SUBFIELDS; i++) {
    if (i < NUM_SUBFIELDS) {
      msg.subFields[i].subFieldId = collectSlots[i].replied ? (i + 1) : 0;
      msg.subFields[i].mq2Ppm = collectSlots[i].mq2Ppm;
      msg.subFields[i].temperatureX10 = collectSlots[i].temperatureX10;
      msg.subFields[i].humidityX10 = collectSlots[i].humidityX10;
    } else {
      msg.subFields[i].subFieldId = 0;
      msg.subFields[i].mq2Ppm = 0;
      msg.subFields[i].temperatureX10 = 0;
      msg.subFields[i].humidityX10 = 0;
    }
  }

  Serial2.write((uint8_t*)&msg, sizeof(msg));
  sendBLENotification("FIELD_DATA_SENT");
  DEBUG_PRINTLN("Sent FIELD_DATA_RESPONSE to HQ");
}

void handleSubfieldMissingInPoll(uint8_t id) {
  showSubfieldMissingNotice(id);

  setLoRaNormalMode();
  delay(10);
  WarningForwardMessage msg;
  msg.type = MSG_WARNING_FORWARD;
  msg.areaId = AREA_ID;
  msg.disaster = FWD_REASON_SUBFIELD_MISSING;
  msg.subFieldId = id;
  msg.latitude = currentLat;
  msg.longitude = currentLon;
  Serial2.write((uint8_t*)&msg, sizeof(msg));
  DEBUG_PRINT("Poll missing: subField=");
  DEBUG_PRINTLN(id);
}

void checkFieldDataCollection() {
  if (!collectingFieldData) return;

  unsigned long elapsed = millis() - collectionStartTime;
  if (elapsed < FIELD_DATA_COLLECTION_WINDOW_MS) return;

  sendFieldDataResponseToHQ();

  for (int i = 0; i < NUM_SUBFIELDS; i++) {
    if (!collectSlots[i].replied) {
      handleSubfieldMissingInPoll(i + 1);
    }
  }

  collectingFieldData = false;
}

// =========================================================================
// HEARTBEAT & TIMERS
// =========================================================================
void checkHeartbeat() {
  if (millis() - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
    setLoRaNormalMode();
    delay(10);

    HeartbeatMessage msg;
    msg.type = MSG_HEARTBEAT;
    msg.areaId = AREA_ID;
    msg.senderBeacon = 0;
    msg.receiverBeacon = 1;
    Serial2.write((uint8_t*)&msg, sizeof(msg));
    lastHeartbeatTime = millis();
    DEBUG_PRINTLN("Sent own heartbeat to HQ");
  }
}

void resetHeartbeatTimer() {
  lastHeartbeatTime = millis();
}

void updateActivity() {
  lastActivityTime = millis();
}

void checkAutoReturnToIdle() {
  if (messageFromApp) return;
  if (currentState == STATE_SLEEP || currentState == STATE_IDLE_TAP) return;
  if (currentState == STATE_WAITING_ACK || currentState == STATE_CANCEL_AVAILABLE ||
      currentState == STATE_SEND_SUCCESS || currentState == STATE_WAITING_CANCEL_ACK ||
      currentState == STATE_CANCEL_CONFIRMED ||   // FIX: let cancel-confirmed screen finish its own timer
      currentState == STATE_WARNING_CONFIRM || currentState == STATE_WARNING_SENT ||
      currentState == STATE_SUBFIELD_MISSING) return;

  if (millis() - lastActivityTime > INACTIVITY_TIMEOUT) handleRestart();
}

void checkCancelWindow() {
  if (currentState == STATE_CANCEL_AVAILABLE && (millis() - ackReceivedTime > CANCEL_HELP_TIMEOUT)) {
    handleRestart();
  }
}

void updateStateMachine() {
  if (currentState == STATE_WAITING_ACK) {
    if (millis() - lastMessageTime > ACK_TIMEOUT && !ackReceived) {
      currentState = STATE_SEND_FAILED;
      systemLocked = false;
      drawStatusFull("Gagal Kirim");
      sendBLENotification("FAILED");
      statusDisplayUntil = millis() + STATUS_DISPLAY_TIME;
      return;
    }
  }

  if (currentState == STATE_WAITING_CANCEL_ACK) {
    if (millis() - lastMessageTime > ACK_TIMEOUT && !cancelAckReceived) {
      systemLocked = false;
      currentState = STATE_SEND_FAILED;
      drawStatusFull("Batal Gagal");
      sendBLENotification("CANCEL_FAILED");
      statusDisplayUntil = millis() + STATUS_DISPLAY_TIME;
      return;
    }
  }
}

// =========================================================================
// SCREEN POWER
// =========================================================================
void updateScreenPower() {
  if (currentState == STATE_WAITING_ACK || currentState == STATE_WAITING_CANCEL_ACK ||
      currentState == STATE_CANCEL_AVAILABLE || currentState == STATE_WARNING_CONFIRM ||
      currentState == STATE_CANCEL_CONFIRMED ||   // FIX: keep screen on to show "Pesan Dibatalkan"
      currentState == STATE_SUBFIELD_MISSING || messageFromApp) {
    if (!screenOn) {
      screenOn = true;
      digitalWrite(TFT_BL, HIGH);
    }
    return;
  }

  if (screenOn && (millis() - lastActivityTime >= SCREEN_INACTIVITY_TIMEOUT)) {
    digitalWrite(TFT_BL, LOW);
    screenOn = false;
    currentState = STATE_SLEEP;
  }
}

// =========================================================================
// BUZZER
// =========================================================================
void startBuzzer(unsigned long durationMs) {
  tone(BUZZER_PIN, 2000);
  buzzerActive = true;
  buzzerOffAt = millis() + durationMs;
}

void updateBuzzer() {
  if (buzzerActive && millis() >= buzzerOffAt) {
    noTone(BUZZER_PIN);
    buzzerActive = false;
  }
}

// =========================================================================
// UI DRAWING
// =========================================================================
void drawTitleRow(int row, const char* label, int colW) {
  int y0 = row * ROW_H;
  gfx->fillRect(0, y0, colW, ROW_H, COLOR_KEMBALI);
  gfx->drawRect(2, y0 + 2, colW - 4, ROW_H - 4, COLOR_BORDER);

  gfx->setTextColor(COLOR_TEXT);
  gfx->setTextSize(2);
  int16_t x1, y1;
  uint16_t tw, th;
  gfx->getTextBounds(label, 0, 0, &x1, &y1, &tw, &th);
  int textX = (colW - tw) / 2;
  if (textX < 4) textX = 4;
  int textY = y0 + (ROW_H - th) / 2;
  gfx->setCursor(textX, textY);
  gfx->print(label);
}

void drawOptionRow(int row, const char* label, bool highlighted, bool empty, int colW) {
  int y0 = row * ROW_H;
  uint16_t fillColor = empty ? COLOR_BG : (highlighted ? COLOR_BOX_SEL : COLOR_BOX);

  gfx->fillRect(0, y0, colW, ROW_H, fillColor);
  if (!empty) {
    gfx->drawRect(2, y0 + 2, colW - 4, ROW_H - 4, COLOR_BORDER);

    gfx->setTextColor(COLOR_TEXT);
    gfx->setTextSize(2);
    int16_t x1, y1;
    uint16_t tw, th;
    gfx->getTextBounds(label, 0, 0, &x1, &y1, &tw, &th);
    int textX = (colW - tw) / 2;
    if (textX < 4) textX = 4;
    int textY = y0 + (ROW_H - th) / 2;
    gfx->setCursor(textX, textY);
    gfx->print(label);
  }
}

void drawScrollButtons(int count) {
  int colX  = SCREEN_W - SCROLL_COL_W;
  int startY = ROW_H;
  int totalH = VISIBLE_OPTION_ROWS * ROW_H;
  int halfH = totalH / 2;

  bool canUp   = cursorIndex > 0;
  bool canDown = cursorIndex < (count - 1);

  gfx->fillRect(colX, startY, SCROLL_COL_W, halfH, canUp ? COLOR_SCROLL_ON : COLOR_SCROLL_OFF);
  gfx->fillRect(colX, startY + halfH, SCROLL_COL_W, totalH - halfH, canDown ? COLOR_SCROLL_ON : COLOR_SCROLL_OFF);
  gfx->drawRect(colX, startY, SCROLL_COL_W, totalH, COLOR_BORDER);
  gfx->drawFastHLine(colX, startY + halfH, SCROLL_COL_W, COLOR_BORDER);

  int cx = colX + SCROLL_COL_W / 2;
  int arrowSize = min(SCROLL_COL_W, ROW_H) / 3;

  int cyUp = startY + halfH / 2;
  gfx->fillTriangle(cx - arrowSize, cyUp + arrowSize / 2,
                     cx + arrowSize, cyUp + arrowSize / 2,
                     cx, cyUp - arrowSize / 2, COLOR_TEXT);

  int cyDown = startY + halfH + (totalH - halfH) / 2;
  gfx->fillTriangle(cx - arrowSize, cyDown - arrowSize / 2,
                     cx + arrowSize, cyDown - arrowSize / 2,
                     cx, cyDown + arrowSize / 2, COLOR_TEXT);
}

void drawBottomRow(const char* leftLabel, const char* rightLabel) {
  int y0 = 4 * ROW_H;
  int halfW = SCREEN_W / 2;

  gfx->fillRect(0, y0, halfW, ROW_H, COLOR_KEMBALI);
  gfx->fillRect(halfW, y0, SCREEN_W - halfW, ROW_H, COLOR_OK);
  gfx->drawFastVLine(halfW, y0, ROW_H, RGB565_WHITE);

  gfx->setTextColor(RGB565_WHITE);
  gfx->setTextSize(2);

  int16_t x1, y1; uint16_t tw, th;
  gfx->getTextBounds(leftLabel, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor((halfW - tw) / 2, y0 + (ROW_H - th) / 2);
  gfx->print(leftLabel);

  gfx->getTextBounds(rightLabel, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor(halfW + (halfW - tw) / 2, y0 + (ROW_H - th) / 2);
  gfx->print(rightLabel);
}

void drawSelectionScreen(const char** options, int count, const char* title) {
  gfx->fillScreen(COLOR_BG);
  int colW = SCREEN_W - SCROLL_COL_W;

  drawTitleRow(0, title, SCREEN_W);

  for (int i = 0; i < VISIBLE_OPTION_ROWS; i++) {
    int absIdx = scrollOffset + i;
    bool empty = absIdx >= count;
    bool hl    = (absIdx == cursorIndex);
    drawOptionRow(i + 1, empty ? "" : options[absIdx], hl, empty, colW);
  }
  drawScrollButtons(count);
  drawBottomRow("KEMBALI", "OK");
}

void drawConfirmScreen() {
  gfx->fillScreen(COLOR_BG);

  char line1[48];
  char line2[48];
  snprintf(line1, sizeof(line1), "Bencana: %s", DISASTER_NAMES[selectedDisaster]);
  snprintf(line2, sizeof(line2), "Dampak: %s", DESTRUCTION_NAMES[selectedDestruction]);

  drawTitleRow(0, "KONFIRMASI LAPORAN", SCREEN_W);
  drawOptionRow(1, line1, false, false, SCREEN_W);
  drawOptionRow(2, line2, false, false, SCREEN_W);
  drawOptionRow(3, "Cek lagi, lalu tekan KIRIM", false, false, SCREEN_W);
  drawBottomRow("KEMBALI", "KIRIM");
}

void drawIdleTapScreen() {
  gfx->fillScreen(COLOR_BG);
  drawTitleRow(0, "FIELD DEVICE", SCREEN_W);

  gfx->setTextColor(COLOR_TEXT);
  gfx->setTextSize(2);
  gfx->setCursor(20, ROW_H + 15);
  gfx->print("Area ");
  gfx->print(AREA_ID);

  gfx->setCursor(20, ROW_H + 15 + 30);
  gfx->print(isGPSReady() ? "GPS: SIAP" : "GPS: MENCARI...");

  int y0 = 3 * ROW_H;
  int h  = SCREEN_H - y0;
  gfx->fillRect(10, y0, SCREEN_W - 20, h - 10, COLOR_OK);
  gfx->drawRect(10, y0, SCREEN_W - 20, h - 10, COLOR_BORDER);

  const char* label = "TAP UNTUK LAPOR";
  gfx->setTextColor(RGB565_BLACK);
  gfx->setTextSize(2);
  int16_t x1, y1; uint16_t tw, th;
  gfx->getTextBounds(label, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor((SCREEN_W - tw) / 2, y0 + (h - 10 - th) / 2);
  gfx->print(label);
}

void drawWaitingScreen(const char* text) {
  gfx->fillScreen(COLOR_BG);
  drawTitleRow(0, "FIELD DEVICE", SCREEN_W);

  gfx->setTextColor(COLOR_TEXT);
  gfx->setTextSize(2);
  int16_t x1, y1; uint16_t tw, th;
  gfx->getTextBounds(text, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor((SCREEN_W - tw) / 2, SCREEN_H / 2 - th / 2);
  gfx->print(text);
}

void drawStatusFull(const char* text) {
  drawWaitingScreen(text);
  statusDisplayUntil = millis() + STATUS_DISPLAY_TIME;
}

void drawCancelAvailableScreen() {
  gfx->fillScreen(COLOR_BG);
  drawTitleRow(0, "TERKIRIM", SCREEN_W);

  gfx->setTextColor(COLOR_TEXT);
  gfx->setTextSize(2);
  gfx->setCursor(20, ROW_H * 2);
  gfx->print("Bantuan dalam proses");

  int y0 = 3 * ROW_H;
  int h = ROW_H * 2;
  gfx->fillRect(0, y0, SCREEN_W, h, COLOR_KEMBALI);
  gfx->drawRect(0, y0, SCREEN_W, h, COLOR_BORDER);

  const char* label = "TAHAN 5 DETIK UTK BATALKAN";
  gfx->setTextColor(RGB565_WHITE);
  gfx->setTextSize(2);
  int16_t x1, y1; uint16_t tw, th;
  gfx->getTextBounds(label, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor((SCREEN_W - tw) / 2, y0 + (h - th) / 2);
  gfx->print(label);
}

void drawWarningScreen(uint8_t reason, uint8_t subFieldId) {
  gfx->fillScreen(COLOR_BG);

  int y0 = 0;
  gfx->fillRect(0, y0, SCREEN_W, ROW_H, COLOR_WARNING);
  gfx->drawRect(2, y0 + 2, SCREEN_W - 4, ROW_H - 4, COLOR_BORDER);
  gfx->setTextColor(RGB565_BLACK);
  gfx->setTextSize(2);
  const char* title = "PERINGATAN SUB-FIELD";
  int16_t x1, y1; uint16_t tw, th;
  gfx->getTextBounds(title, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor((SCREEN_W - tw) / 2, y0 + (ROW_H - th) / 2);
  gfx->print(title);

  char line1[40];
  if (reason == WARN_FOREST_FIRE) {
    snprintf(line1, sizeof(line1), "Kemungkinan Kebakaran Hutan");
  } else {
    snprintf(line1, sizeof(line1), "Kemungkinan Kebocoran Gas");
  }
  char line2[32];
  snprintf(line2, sizeof(line2), "Sub-Field #%d", subFieldId);

  drawOptionRow(1, line1, false, false, SCREEN_W);
  drawOptionRow(2, line2, false, false, SCREEN_W);
  drawOptionRow(3, "Cek sumber peringatan!", false, false, SCREEN_W);

  drawBottomRow("BATAL (5s)", "KIRIM (5s)");
}

// Builds "1", "1 & 2", or "1, 2 & 3" from a bitmask (bit i-1 = subfield i).
void buildMissingIdsLabel(uint16_t mask, char* out, size_t outSize) {
  int ids[MAX_SUBFIELDS];
  int count = 0;
  for (int i = 0; i < MAX_SUBFIELDS && count < MAX_SUBFIELDS; i++) {
    if (mask & (1 << i)) ids[count++] = i + 1;
  }

  size_t pos = 0;
  out[0] = '\0';
  for (int i = 0; i < count && pos < outSize; i++) {
    const char* sep = (i == 0) ? "" : (i == count - 1) ? " & " : ", ";
    int written = snprintf(out + pos, outSize - pos, "%s%d", sep, ids[i]);
    if (written < 0) break;
    pos += (size_t)written;
  }
}

void drawSubFieldMissingScreen(uint16_t missingMask) {
  gfx->fillScreen(COLOR_BG);

  int y0 = 0;
  gfx->fillRect(0, y0, SCREEN_W, ROW_H, COLOR_WARNING);
  gfx->drawRect(2, y0 + 2, SCREEN_W - 4, ROW_H - 4, COLOR_BORDER);
  gfx->setTextColor(RGB565_BLACK);
  gfx->setTextSize(2);
  const char* title = "SUB-FIELD HILANG";
  int16_t x1, y1; uint16_t tw, th;
  gfx->getTextBounds(title, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor((SCREEN_W - tw) / 2, y0 + (ROW_H - th) / 2);
  gfx->print(title);

  char idsLabel[24];
  buildMissingIdsLabel(missingMask, idsLabel, sizeof(idsLabel));

  char line1[48];
  snprintf(line1, sizeof(line1), "Sub-Field %s tidak merespon", idsLabel);

  drawOptionRow(1, line1, false, false, SCREEN_W);
  drawOptionRow(2, "Cek koneksi/baterai unit tsb.", false, false, SCREEN_W);
  drawOptionRow(3, "HQ sudah diberi tahu otomatis.", false, false, SCREEN_W);

  int by0 = 4 * ROW_H;
  gfx->fillRect(0, by0, SCREEN_W, ROW_H, COLOR_OK);
  gfx->drawRect(0, by0, SCREEN_W, ROW_H, COLOR_BORDER);

  const char* label = "TAP UNTUK TUTUP";
  gfx->setTextColor(RGB565_BLACK);
  gfx->setTextSize(2);
  gfx->getTextBounds(label, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor((SCREEN_W - tw) / 2, by0 + (ROW_H - th) / 2);
  gfx->print(label);
}

void drawHoldProgress(int x0, int width, unsigned long held, unsigned long total) {
  static unsigned long lastDraw = 0;
  if (millis() - lastDraw < 150) return;
  lastDraw = millis();

  int y0 = 4 * ROW_H;
  int barH = 6;
  int barY = y0 + ROW_H - barH - 4;
  float frac = (float)held / (float)total;
  if (frac > 1.0) frac = 1.0;
  int fillW = (int)(width * frac);

  gfx->fillRect(x0 + 4, barY, width - 8, barH, RGB565_BLACK);
  gfx->fillRect(x0 + 4, barY, fillW - 8 > 0 ? fillW - 8 : 0, barH, COLOR_PROGRESS);
}

// =========================================================================
// APP (BLE) COMMAND
// =========================================================================
void handleAppHelpRequest(String command) {
  int i1 = command.indexOf(',');
  if (i1 == -1) return;
  command = command.substring(i1 + 1);

  int i2 = command.indexOf(',');
  if (i2 == -1) return;
  int d1 = command.substring(0, i2).toInt();
  command = command.substring(i2 + 1);

  int i3 = command.indexOf(',');
  if (i3 == -1) return;
  int d2 = command.substring(0, i3).toInt();
  command = command.substring(i3 + 1);

  int i4 = command.indexOf(',');
  if (i4 == -1) return;
  double lat = command.substring(0, i4).toDouble();
  double lon = command.substring(i4 + 1).toDouble();

  if (d1 >= 0 && d1 < NUM_DISASTERS) selectedDisaster = d1;
  if (d2 >= 0 && d2 < NUM_DESTRUCTIONS) selectedDestruction = d2;
  if (lat != 0.0 && lon != 0.0) {
    currentLat = lat;
    currentLon = lon;
  }

  messageFromApp = true;
  screenOn = true;
  digitalWrite(TFT_BL, HIGH);
  updateActivity();
  sendHelpMessage();
}