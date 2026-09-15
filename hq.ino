// =========================================================================
// MONITORING TERMINAL (MT) — v6.2 (DETAIL LOG TERSTRUKTUR)
// =========================================================================
// - LogEntry menyimpan detail terstruktur (bukan parsing ulang string)
// - Detail log menampilkan label + nilai, wrapping otomatis
// - Navigasi pilihan log pakai panah, tombol DETAIL di bawah
// - Semua fitur lain tetap (dashboard, keypad, alert, Firebase, NTP)
// =========================================================================

#include <Arduino_GFX_Library.h>
#include <XPT2046_Touchscreen.h>
#include "LoRa_E220.h"
#include "esp_pm.h"
#include <WiFi.h>
#include <time.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>  // Library Manager: "ArduinoJson" by Benoit Blanchon, v6.x

// =========================================================================
// DEBUG TOUCH
// =========================================================================
// #define DEBUG_TOUCH

// =========================================================================
// WIFI & NTP (REAL-TIME CLOCK)
// =========================================================================
const char* WIFI_SSID = "11 Pro 5G";
const char* WIFI_PASS = "budiutomo";

const char* NTP_SERVER = "pool.ntp.org";
const long  GMT_OFFSET_SEC = 7 * 3600;   // WIB (GMT+7)
const int   DAYLIGHT_OFFSET_SEC = 0;

struct tm timeinfo;
bool timeSynced = false;
unsigned long lastNtpAttempt = 0;
const unsigned long NTP_RETRY_INTERVAL = 3600000UL;

const char* compileDate = __DATE__;
const char* compileTime = __TIME__;
unsigned long bootTime = 0;

// =========================================================================
// FIREBASE CONFIGURATION
// =========================================================================
const String FIREBASE_URL = "https://peshongsigap-default-rtdb.asia-southeast1.firebasedatabase.app";

// Realtime Database "Secret" (legacy) — Firebase Console > Project Settings >
// Service Accounts > Database secrets. Appending ?auth=<secret> to every
// request makes the firmware bypass database.rules.json entirely (full
// admin access), while requests without it (a normal user hitting the REST
// API directly, or the sigap_user app, which has no Firebase code at all)
// are still blocked by the admin-only rules. Google marks this mechanism
// legacy/deprecated in favor of the Admin SDK, but it's still supported and
// is the practical option for an ESP32 that can't run the Admin SDK.
const String FIREBASE_SECRET = "hQzXcfaPMUk3y9ECyQJUIUhBBmyIH7n5BYF5C5WM";
// const String FIREBASE_SECRET = "HeFZjZJkt5NT1kmnlY8gwjifOTU3qSS1bWXT3R6I";

// =========================================================================
// COLOR DEFINITIONS
// =========================================================================
#define RGB565_BLACK   0x0000
#define RGB565_WHITE   0xFFFF
#define RGB565_RED     0xF800
#define RGB565_GREEN   0x07E0
#define RGB565_BLUE    0x001F
#define RGB565_YELLOW  0xFFE0
#define RGB565_ORANGE  0xFD20
#define RGB565_CYAN    0x07FF

#define COLOR_BG          RGB565_BLACK
#define COLOR_BOX         0x39C7
#define COLOR_BOX_SEL     0x051F
#define COLOR_BORDER      RGB565_WHITE
#define COLOR_TEXT        RGB565_WHITE
#define COLOR_OK          RGB565_GREEN
#define COLOR_KEMBALI     RGB565_RED
#define COLOR_SCROLL_ON   0x7BEF
#define COLOR_SCROLL_OFF  0x2104
#define COLOR_SCROLLBAR_TRACK  0x18E3
#define COLOR_SCROLLBAR_THUMB  0xBDF7
#define COLOR_LABEL       0x8410   // abu-abu untuk label

#define COLOR_LOG_INFO    RGB565_WHITE
#define COLOR_LOG_SUCCESS RGB565_GREEN
#define COLOR_LOG_WARN    RGB565_YELLOW
#define COLOR_LOG_ERROR   RGB565_RED
#define COLOR_LOG_DATA    RGB565_CYAN

// Warna perangkat berdasarkan tipe
#define COLOR_DEV_UT_ONLINE   0x0480
#define COLOR_DEV_UT_STALE    0x6180
#define COLOR_DEV_RN_ONLINE   0x001F
#define COLOR_DEV_RN_STALE    0x318C
#define COLOR_DEV_SF_ONLINE   0xFD20
#define COLOR_DEV_SF_STALE    0x7BEF
#define COLOR_DEV_EMPTY       COLOR_BOX

// =========================================================================
// PIN DEFINITIONS
// =========================================================================
#define TFT_SCK   18
#define TFT_MOSI  23
#define TFT_MISO  -1
#define TFT_CS    5
#define TFT_DC    2
#define TFT_RST   4
#define TFT_BL    27

#define TOUCH_CS  19
#define TOUCH_DO  21

#define WAKE_PIN                    22

#define WAKE_PAGE_SWITCH_TAPS       2
#define WAKE_TAP_WINDOW_MS          500UL
#define WAKE_DEBOUNCE_MS            50UL
#define WAKE_HOLD_DURATION_MS       5000UL

#define BUZZER_PIN          25

#define LORA_RX   16
#define LORA_TX   17
#define LORA_M0   12
#define LORA_M1   13
#define LORA_AUX  14

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
// LAYOUT CONSTANTS
// =========================================================================
#define LOG_LINES_VISIBLE  3      // 3 baris log
#define MAX_LOG_ENTRIES    50
#define SCROLLBAR_W        10

#define KEYPAD_ROWS        5
#define KEYPAD_COLS        3

#define HEARTBEAT_TIMEOUT    2100000UL
#define DEVICE_MAX_ENTRIES   20
#define PRUNE_INTERVAL        60000UL

#define HELP_CANCEL_WINDOW_MS 300000UL   // 5 menit — jendela valid CANCEL setelah HELP

#define DEVICE_PUSH_MIN_INTERVAL_MS 3000UL   // jarak minimum antar-push status device yang sama
#define COMMAND_POLL_INTERVAL_MS    5000UL   // seberapa sering polling /commands dari app admin

// =========================================================================
// DEDUP HASH CONFIGURATION
// =========================================================================
#define MAX_MESSAGE_HISTORY   100
#define DEDUP_WINDOW_MS       60000UL

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

#define MAX_SUBFIELDS 3

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

struct WarningForwardMessage {
  uint8_t type;
  uint16_t areaId;
  uint8_t disaster;
  uint8_t subFieldId;
  float latitude;
  float longitude;
};

struct WarningAckMessage {
  uint8_t type;
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
  MSG_WARNING                = 9,
  MSG_WARNING_FORWARD        = 10,
  MSG_SUBFIELD_HEARTBEAT      = 11,
  MSG_SUBFIELD_DATA_REQUEST   = 12,
  MSG_SUBFIELD_DATA_REPLY     = 13,
  MSG_WARNING_ACK             = 14,
};

size_t messageSize(uint8_t type) {
  switch (type) {
    case MSG_HELP:                 return 13;
    case MSG_ACK:                  return 3;
    case MSG_ALARM:                return 11;
    case MSG_CANCEL_HELP:          return 3;
    case MSG_FIELD_DATA_REQUEST:   return 3;
    case MSG_FIELD_DATA_RESPONSE:  return 11 + MAX_SUBFIELDS * 7;
    case MSG_HEARTBEAT:            return 5;
    case MSG_MISSING_BEACON:       return 4;
    case MSG_WARNING:              return 5;
    case MSG_WARNING_FORWARD:      return 13;
    case MSG_SUBFIELD_HEARTBEAT:   return 4;
    case MSG_SUBFIELD_DATA_REQUEST:return 3;
    case MSG_SUBFIELD_DATA_REPLY:  return 10;
    case MSG_WARNING_ACK:          return sizeof(WarningAckMessage);
    default:                       return 0;
  }
}

// =========================================================================
// NAME LOOKUP
// =========================================================================
const char* DISASTER_NAMES[] = {
  "Gempa Bumi",
  "Banjir",
  "Tanah Longsor",
  "Kebakaran",
  "Kebocoran Gas",
  "Lain-lainnya"
};
const int NUM_DISASTERS = sizeof(DISASTER_NAMES) / sizeof(DISASTER_NAMES[0]);

const char* DESTRUCTION_NAMES[] = {
  "Ringan",
  "Menengah",
  "Besar",
  "Masif",
  "Unknown",
};
const int NUM_DESTRUCTIONS = sizeof(DESTRUCTION_NAMES) / sizeof(DESTRUCTION_NAMES[0]);

#define FWD_REASON_SUBFIELD_MISSING 0xFF

const char* FWD_REASON_NAMES[] = {
  "Sub-Field Hilang", "Kebocoran Gas", "Kebakaran Hutan",
};
#define NUM_FWD_REASONS 3

// =========================================================================
// STATE MACHINE
// =========================================================================
enum SystemState {
  STATE_LOG,
  STATE_KEYPAD,
  STATE_DASHBOARD,
  STATE_ALERT,
  STATE_DETAIL_LOG
};

// =========================================================================
// LOG ENTRY DENGAN DETAIL TERSTRUKTUR — DITEMPATKAN DI SINI
// =========================================================================
#define DETAIL_NONE       0
#define DETAIL_HELP       1
#define DETAIL_ACK        2
#define DETAIL_ALARM      3
#define DETAIL_CANCEL     4
#define DETAIL_DATA       5
#define DETAIL_SUBFIELD   6
#define DETAIL_HEARTBEAT  7
#define DETAIL_BEACON     8
#define DETAIL_WARNING    9
#define DETAIL_SF_MISSING 10
#define DETAIL_REQ_OUT    11
#define DETAIL_ACK_OUT    12
#define DETAIL_OFFLINE    13

struct LogDetail {
  uint8_t  kind = DETAIL_NONE;
  uint16_t areaId = 0;
  int16_t  subFieldId = -1;   // -1 = tidak ada
  String   disaster = "";
  String   destruction = "";
  float    lat = 0, lon = 0;
  bool     hasCoord = false;
  uint16_t ppm = 0;
  int16_t  tempX10 = 0;
  int16_t  humX10 = 0;
  bool     hasSensor = false;
  String   note = "";
  String   time = "";
};

struct LogEntry {
  String message;
  uint16_t color;
  unsigned long timestamp;
  LogDetail detail;
};

// =========================================================================
// GLOBAL OBJECTS
// =========================================================================
Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO);
Arduino_GFX *gfx = new Arduino_ILI9488_18bit(bus, TFT_RST, 1, false);
XPT2046_Touchscreen ts(TOUCH_CS);
LoRa_E220 e220(&Serial2, LORA_AUX, LORA_M0, LORA_M1);

// =========================================================================
// GLOBAL VARIABLES
// =========================================================================
int SCREEN_W, SCREEN_H;
int ROW_H, SCROLL_COL_W;

int alertBtnX, alertBtnY, alertBtnW, alertBtnH;

SystemState currentState = STATE_LOG;
SystemState stateBeforeAlert = STATE_LOG;

LogEntry logEntries[MAX_LOG_ENTRIES];
int logCount = 0;
int logScrollOffset = 0;
bool logFollowingBottom = true;

int selectedLogIndex = -1;

String enteredNumber = "";
int keypadDisplayY0 = 0;
int keypadDisplayH = 0;

bool wakePrevState = false;
unsigned long wakePressStart = 0;
bool wakeHoldFired = false;
unsigned long wakeLastEdgeTime = 0;
unsigned long wakeLastTapTime = 0;
int wakeTapCount = 0;
SystemState pageBeforeKeypad = STATE_LOG;

bool wasTouched = false;
int touchX, touchY;

struct DeviceEntry {
  uint16_t areaId;
  unsigned long lastSeen;
  uint8_t deviceType;
  bool active;
  unsigned long lastPushed = 0;  // last time this device's status was pushed to Firebase
};
DeviceEntry deviceRegistry[DEVICE_MAX_ENTRIES];

// Tracks the Firebase key (the millis() value used as the /emergencies/<key>
// path segment) of the most recent HELP written per areaId, so a later
// CANCEL can PATCH that same node's status instead of only logging locally.
#define HELP_KEY_MAX_ENTRIES 20
struct HelpFirebaseEntry {
  uint16_t areaId;
  unsigned long firebaseKey;
  bool active;
};
HelpFirebaseEntry helpKeyRegistry[HELP_KEY_MAX_ENTRIES];

unsigned long alertDismissAfter = 0;
String alertTitle = "";
String alertBody = "";
uint16_t alertColor = RGB565_RED;

#define SUBFIELD_MISSING_WINDOW_MS      800UL
#define MAX_PENDING_MISSING_SUBFIELDS   8

uint16_t missingAggAreaId = 0;
uint8_t  missingAggSubFields[MAX_PENDING_MISSING_SUBFIELDS];
int      missingAggCount = 0;
bool     missingAggPending = false;
unsigned long missingAggDeadline = 0;

unsigned long lastHeartbeatLog = 0;
#define HEARTBEAT_LOG_INTERVAL 300000UL

unsigned long lastPruneTime = 0;
unsigned long lastCommandPoll = 0;

unsigned long lastClockUpdate = 0;
bool dashboardGridDirty = true;

int statusLine1Y = 0;
int statusLine1H = 0;
int statusLine1X = 2;
int statusLine1W = 0;
bool statusLayoutReady = false;

String lastClockText = "";
String lastStatusLine1 = "";
int lastActiveCount = -1;

uint16_t helpCount = 0;
uint16_t alarmCount = 0;
uint16_t cancelCount = 0;
uint16_t missingBeaconCount = 0;
uint16_t warningCount = 0;

bool buzzerSequenceActive = false;
int buzzerSequenceStep = 0;
unsigned long buzzerSequenceLastTick = 0;

struct MessageRecord {
  uint32_t hash;
  unsigned long timestamp;
};
MessageRecord messageHistory[MAX_MESSAGE_HISTORY];
int messageHistoryIndex = 0;

String lastImportantLog = "";

// =========================================================================
// PROTOTYPES FUNGSI (LogDetail sudah dikenal)
// =========================================================================
void setLoRaNormalMode();
void drawLogScreen();
void drawLogList();
void drawStatusBar();
void drawScrollbarThumb();
void drawKeypadScreen();
void drawKeypadDisplay();
void drawDashboardScreen();
void drawDashboardGrid(int yStart);
void drawDetailLogScreen(int logIndex);
void updateStatusBarClock();
void togglePage();
void toggleKeypad();
void drawAlertScreen();
void handleAlertTouch();
void addLogEntry(const String& msg, uint16_t color);
void addLogEntry(const String& msg, uint16_t color, const LogDetail& d);
void handleTouch();
void mapTouch(TS_Point p, int &x, int &y);
void onTouchDown(int x, int y);
void onTouchUp();
void handleWakeSensor();
void checkIncomingLoRa();
void updateBuzzerSequence();
void stopBuzzerSequence();
void dismissAlert();

uint32_t computeHash(const uint8_t* data, size_t len);
bool isMessageInHistory(uint32_t hash);
void addToMessageHistory(uint32_t hash);
String getTimeString();
String getTimeStringShort();
String pad2(uint16_t value);

void syncNTP();
String authQueryParam();
void sendToFirebase(String path, String jsonData);
void sendFirebasePatch(String path, String jsonData);
void recordHelpFirebaseKey(uint16_t areaId, unsigned long key);
bool findHelpFirebaseKey(uint16_t areaId, unsigned long &outKey);
String jsonEscape(const String& s);
void pushDeviceStatus(uint16_t areaId, uint8_t deviceType, bool active);
void pushLogToFirebase(const LogEntry& entry);
void checkFirebaseCommands();

void handleHelp(HelpMessage* msg);
void handleAck(AckMessage* msg);
void handleAlarm(AlarmMessage* msg);
void handleCancelHelp(CancelHelpMessage* msg);
bool hasRecentHelp(uint16_t areaId);
void handleFieldDataResponse(FieldDataResponseMessage* msg);
void handleHeartbeat(HeartbeatMessage* msg);
void handleMissingBeacon(MissingBeaconMessage* msg);
void handleWarningForward(WarningForwardMessage* msg);
void queueMissingSubfield(uint16_t areaId, uint8_t subFieldId);
void checkMissingSubfieldBatch();

void sendAck(uint16_t areaId);
void sendFieldDataRequest(uint16_t areaId);
void broadcastFieldDataRequest();
void sendRequestFromKeypad(const String& areaIdStr);

int findDevice(uint16_t areaId);
void registerDevice(uint16_t areaId, uint8_t type);
void updateDeviceSeen(uint16_t areaId);
int countActiveDevices();
void pruneInactiveDevices();

void beep(uint16_t freq = 2000, unsigned long duration = 100);
void beepAlert();
void showAlert(const String& title, const String& body, uint16_t color, unsigned long duration = 10000);

// Fungsi navigasi pilihan
void moveSelection(int direction);
void ensureSelectedVisible();

// Fungsi bantu detail log
const char* detailKindName(uint8_t kind);
int printWrapped(const String& text, int x, int y, int firstW, int lineH, int maxLines, int contX, int contW);
int printField(int x, int y, const char* label, const String& value, uint16_t valColor, int colW, int lineH);

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  delay(500);
  
  esp_pm_config_esp32_t pm_config = { .max_freq_mhz = 80, .min_freq_mhz = 10, .light_sleep_enable = false };
  esp_pm_configure(&pm_config);

  pinMode(WAKE_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);
  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  pinMode(LORA_AUX, INPUT);
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  if (!gfx->begin(10000000)) while (1);
  gfx->fillScreen(COLOR_BG);

  SPI.begin(TFT_SCK, TOUCH_DO, TFT_MOSI, TOUCH_CS);
  ts.begin(SPI);
  ts.setRotation(1);

  SCREEN_W = gfx->width();
  SCREEN_H = gfx->height();
  ROW_H = SCREEN_H / 5;
  SCROLL_COL_W = SCREEN_W / 6;

  alertBtnX = 20;
  alertBtnW = SCREEN_W - 40;
  alertBtnH = ROW_H;
  alertBtnY = SCREEN_H - ROW_H - 10;

  Serial2.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);
  if (e220.begin() != 1) while (1);
  setLoRaNormalMode();

  for (int i = 0; i < DEVICE_MAX_ENTRIES; i++) {
    deviceRegistry[i].active = false;
  }

  for (int i = 0; i < MAX_MESSAGE_HISTORY; i++) {
    messageHistory[i].hash = 0;
    messageHistory[i].timestamp = 0;
  }

  // --- WiFi + NTP ---
  Serial.begin(115200);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    Serial.print(".");
    retry++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected. Syncing NTP...");
    syncNTP();
  } else {
    Serial.println("WiFi failed, using compile-time fallback.");
  }

  bootTime = millis();

  if (logCount > 0) {
    selectedLogIndex = logCount - 1;
  }

  drawLogScreen();
  dashboardGridDirty = true;
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
  if (!timeSynced && WiFi.status() == WL_CONNECTED) {
    if (millis() - lastNtpAttempt > NTP_RETRY_INTERVAL) {
      syncNTP();
    }
  }

  if (currentState == STATE_ALERT) {
    handleAlertTouch();
    if (millis() > alertDismissAfter) {
      dismissAlert();
      return;
    }
    updateBuzzerSequence();
    checkIncomingLoRa();
    checkMissingSubfieldBatch();
    delay(10);
    return;
  }
  
  handleWakeSensor();
  handleTouch();
  updateBuzzerSequence();
  checkIncomingLoRa();
  checkMissingSubfieldBatch();
  pruneInactiveDevices();
  checkFirebaseCommands();

  if ((currentState == STATE_LOG || currentState == STATE_DASHBOARD || currentState == STATE_DETAIL_LOG) &&
      millis() - lastClockUpdate >= 1000) {
    lastClockUpdate = millis();
    updateStatusBarClock();
  }

  if (currentState == STATE_DASHBOARD && dashboardGridDirty) {
    dashboardGridDirty = false;
    drawDashboardGrid(2 * ROW_H);
  }
  
  delay(10);
}

// =========================================================================
// DEDUP HASH FUNCTIONS
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
// NTP SYNC
// =========================================================================
void syncNTP() {
  lastNtpAttempt = millis();
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  for (int i = 0; i < 10; i++) {
    if (getLocalTime(&timeinfo)) {
      timeSynced = true;
      Serial.println("NTP sync success!");
      return;
    }
    delay(500);
  }
  timeSynced = false;
  Serial.println("NTP sync failed.");
}

// =========================================================================
// TIME HELPERS
// =========================================================================
String pad2(uint16_t value) {
  if (value < 10) return "0" + String(value);
  return String(value);
}

time_t compileTimeToTimestamp() {
  struct tm t = {0};
  const char* months[] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
  char monthStr[4];
  int day, year;
  sscanf(compileDate, "%s %d %d", monthStr, &day, &year);
  for (int i = 0; i < 12; i++) {
    if (strcmp(monthStr, months[i]) == 0) { t.tm_mon = i; break; }
  }
  t.tm_mday = day;
  t.tm_year = year - 1900;
  int hh, mm, ss;
  sscanf(compileTime, "%d:%d:%d", &hh, &mm, &ss);
  t.tm_hour = hh;
  t.tm_min = mm;
  t.tm_sec = ss;
  t.tm_isdst = 0;
  return mktime(&t);
}

String getTimeString() {
  if (timeSynced && getLocalTime(&timeinfo)) {
    char buf[9];
    strftime(buf, sizeof(buf), "%H:%M:%S", &timeinfo);
    return String(buf);
  }
  time_t base = compileTimeToTimestamp();
  unsigned long elapsed = millis() / 1000;
  time_t now = base + elapsed;
  struct tm* tm_info = localtime(&now);
  char buf[9];
  strftime(buf, sizeof(buf), "%H:%M:%S", tm_info);
  return String(buf);
}

String getTimeStringShort() {
  if (timeSynced && getLocalTime(&timeinfo)) {
    char buf[6];
    strftime(buf, sizeof(buf), "%H:%M", &timeinfo);
    return String(buf);
  }
  time_t base = compileTimeToTimestamp();
  unsigned long elapsed = millis() / 1000;
  time_t now = base + elapsed;
  struct tm* tm_info = localtime(&now);
  char buf[6];
  strftime(buf, sizeof(buf), "%H:%M", tm_info);
  return String(buf);
}

// =========================================================================
// LORA CONFIG
// =========================================================================
void setLoRaNormalMode() {
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  delay(10);
}

// =========================================================================
// WAKE SENSOR
// =========================================================================
void handleWakeSensor() {
  if (currentState == STATE_ALERT) return;

  bool pressed = digitalRead(WAKE_PIN);
  unsigned long now = millis();

  if (pressed && !wakePrevState && (now - wakeLastEdgeTime > WAKE_DEBOUNCE_MS)) {
    wakeLastEdgeTime = now;
    wakePressStart = now;
    wakeHoldFired = false;
  }

  if (pressed && wakePrevState && !wakeHoldFired) {
    if (now - wakePressStart >= WAKE_HOLD_DURATION_MS) {
      wakeHoldFired = true;
      wakeTapCount = 0;
      toggleKeypad();
    }
  }

  if (!pressed && wakePrevState) {
    wakeLastEdgeTime = now;
    if (!wakeHoldFired) {
      if (now - wakeLastTapTime <= WAKE_TAP_WINDOW_MS) {
        wakeTapCount++;
      } else {
        wakeTapCount = 1;
      }
      wakeLastTapTime = now;
      if (wakeTapCount >= WAKE_PAGE_SWITCH_TAPS) {
        wakeTapCount = 0;
        togglePage();
      }
    }
  }

  wakePrevState = pressed;
  if (wakeTapCount > 0 && (now - wakeLastTapTime > WAKE_TAP_WINDOW_MS)) {
    wakeTapCount = 0;
  }
}

// =========================================================================
// TOGGLE PAGE (Double tap) — LOG <-> DASHBOARD
// =========================================================================
void togglePage() {
  if (currentState != STATE_LOG && currentState != STATE_DASHBOARD) return;
  beep();
  if (currentState == STATE_LOG) {
    currentState = STATE_DASHBOARD;
    dashboardGridDirty = true;
    drawDashboardScreen();
  } else {
    currentState = STATE_LOG;
    drawLogScreen();
  }
}

// =========================================================================
// TOGGLE KEYPAD (hold)
// =========================================================================
void toggleKeypad() {
  beep();
  if (currentState == STATE_KEYPAD) {
    currentState = pageBeforeKeypad;
    if (currentState == STATE_DASHBOARD) {
      dashboardGridDirty = true;
      drawDashboardScreen();
    } else {
      drawLogScreen();
    }
  } else if (currentState == STATE_LOG || currentState == STATE_DASHBOARD) {
    pageBeforeKeypad = currentState;
    currentState = STATE_KEYPAD;
    enteredNumber = "";
    drawKeypadScreen();
  }
}

// =========================================================================
// TOUCH HANDLING
// =========================================================================
void mapTouch(TS_Point p, int &x, int &y) {
  int rawX = p.x, rawY = p.y;
  if (TOUCH_SWAP_XY) { int t = rawX; rawX = rawY; rawY = t; }
  if (TOUCH_INVERT_X) x = map(rawX, TOUCH_MIN_X, TOUCH_MAX_X, SCREEN_W, 0);
  else                x = map(rawX, TOUCH_MIN_X, TOUCH_MAX_X, 0, SCREEN_W);
  if (TOUCH_INVERT_Y) y = map(rawY, TOUCH_MIN_Y, TOUCH_MAX_Y, SCREEN_H, 0);
  else                y = map(rawY, TOUCH_MIN_Y, TOUCH_MAX_Y, 0, SCREEN_H);
  x = constrain(x, 0, SCREEN_W - 1);
  y = constrain(y, 0, SCREEN_H - 1);
}

void handleAlertTouch() {
  bool touched = ts.touched();
  if (touched && !wasTouched) {
    TS_Point p = ts.getPoint();
    int x, y;
    mapTouch(p, x, y);
    if (x >= alertBtnX && x <= alertBtnX + alertBtnW &&
        y >= alertBtnY && y <= alertBtnY + alertBtnH) {
      dismissAlert();
    }
  }
  wasTouched = touched;
}

void handleTouch() {
  bool touched = ts.touched();
  if (touched && !wasTouched) {
    TS_Point p = ts.getPoint();
    mapTouch(p, touchX, touchY);
    onTouchDown(touchX, touchY);
  } else if (!touched && wasTouched) {
    onTouchUp();
  }
  wasTouched = touched;
}

void onTouchDown(int x, int y) {
  if (currentState == STATE_ALERT) return;
  
  if (currentState == STATE_LOG) {
    int colW = SCREEN_W - SCROLL_COL_W - SCROLLBAR_W;
    int buttonX = colW + SCROLLBAR_W;
    if (x >= buttonX) {
      int startY = ROW_H;
      int totalH = LOG_LINES_VISIBLE * ROW_H;
      int halfH = totalH / 2;
      if (y >= startY && y < startY + halfH) {
        beep();
        moveSelection(-1);
      } else if (y >= startY + halfH && y < startY + totalH) {
        beep();
        moveSelection(1);
      }
    }
    int btnRow = 1 + LOG_LINES_VISIBLE;
    int btnY = btnRow * ROW_H;
    if (y >= btnY && y < btnY + ROW_H) {
      if (selectedLogIndex >= 0 && selectedLogIndex < logCount) {
        beep();
        currentState = STATE_DETAIL_LOG;
        drawDetailLogScreen(selectedLogIndex);
      } else {
        beep();
      }
    }
  }
  else if (currentState == STATE_KEYPAD) {
    int row = y / ROW_H;
    int col = x / (SCREEN_W / KEYPAD_COLS);
    if (row >= 1 && row <= 3) {
      int digit = (row - 1) * 3 + col + 1;
      if (digit >= 1 && digit <= 9) {
        enteredNumber += String(digit);
        drawKeypadDisplay();
        beep();
      }
    } else if (row == 4) {
      if (col == 0) {
        if (enteredNumber.length() > 0) {
          enteredNumber.remove(enteredNumber.length() - 1);
          drawKeypadDisplay();
          beep();
        }
      } else if (col == 1) {
        enteredNumber += "0";
        drawKeypadDisplay();
        beep();
      } else if (col == 2) {
        if (enteredNumber.length() > 0) {
          logFollowingBottom = true;
          sendRequestFromKeypad(enteredNumber);
          enteredNumber = "";
          currentState = pageBeforeKeypad;
          if (currentState == STATE_DASHBOARD) {
            dashboardGridDirty = true;
            drawDashboardScreen();
          } else {
            drawLogScreen();
          }
          beep();
        }
      }
    }
  }
  else if (currentState == STATE_DASHBOARD) {
    int gridY0 = 2 * ROW_H;
    if (y < gridY0) return;
    int cols = 5;
    int rows = 3;
    int cellW = SCREEN_W / cols;
    int cellH = (SCREEN_H - gridY0) / rows;
    int col = x / cellW;
    int row = (y - gridY0) / cellH;
    int idx = row * cols + col;
    if (idx >= 0 && idx < DEVICE_MAX_ENTRIES && deviceRegistry[idx].active) {
      beep();
      sendFieldDataRequest(deviceRegistry[idx].areaId);
    }
  }
  else if (currentState == STATE_DETAIL_LOG) {
    int btnY = SCREEN_H - ROW_H;
    if (y >= btnY) {
      beep();
      currentState = STATE_LOG;
      drawLogScreen();
    }
  }
}

void onTouchUp() {}

// =========================================================================
// NAVIGASI PILIHAN
// =========================================================================
void moveSelection(int direction) {
  if (logCount == 0) return;
  if (selectedLogIndex < 0 || selectedLogIndex >= logCount) {
    selectedLogIndex = (direction > 0) ? (logCount - 1) : 0;
  } else {
    int newIdx = selectedLogIndex + direction;
    if (newIdx < 0) newIdx = 0;
    if (newIdx >= logCount) newIdx = logCount - 1;
    selectedLogIndex = newIdx;
  }
  ensureSelectedVisible();
  drawLogList();
}

void ensureSelectedVisible() {
  if (selectedLogIndex < 0 || selectedLogIndex >= logCount) return;
  if (selectedLogIndex < logScrollOffset) {
    logScrollOffset = selectedLogIndex;
  } else if (selectedLogIndex >= logScrollOffset + LOG_LINES_VISIBLE) {
    logScrollOffset = selectedLogIndex - LOG_LINES_VISIBLE + 1;
  }
  if (logScrollOffset < 0) logScrollOffset = 0;
  if (logScrollOffset + LOG_LINES_VISIBLE > logCount) {
    logScrollOffset = logCount - LOG_LINES_VISIBLE;
    if (logScrollOffset < 0) logScrollOffset = 0;
  }
  logFollowingBottom = (logScrollOffset + LOG_LINES_VISIBLE >= logCount);
}

// =========================================================================
// DRAW FUNCTIONS
// =========================================================================
void drawLogScreen() {
  gfx->fillScreen(COLOR_BG);
  drawStatusBar();
  drawLogList();
}

void drawStatusBar() {
  int y0 = 0;
  int activeCount = countActiveDevices();
  gfx->fillRect(0, y0, SCREEN_W, ROW_H, COLOR_BOX);
  gfx->drawRect(2, y0 + 2, SCREEN_W - 4, ROW_H - 4, COLOR_BORDER);
  gfx->setTextColor(COLOR_TEXT);
  gfx->setTextSize(2);

  String timeStr = getTimeStringShort();
  if (!timeSynced) {
    timeStr = "NTP...";
  }

  String line1 = String(activeCount) + " AKTIF   " + timeStr;
  String line2 = "H:" + pad2(helpCount) + " A:" + pad2(alarmCount) +
                 " C:" + pad2(cancelCount) + " M:" + pad2(missingBeaconCount) +
                 " W:" + pad2(warningCount);

  int16_t x1, y1; uint16_t tw1, th1, tw2, th2;
  gfx->getTextBounds(line1.c_str(), 0, 0, &x1, &y1, &tw1, &th1);
  gfx->getTextBounds(line2.c_str(), 0, 0, &x1, &y1, &tw2, &th2);

  int gap = 4;
  int blockH = th1 + gap + th2;
  int topY = y0 + (ROW_H - blockH) / 2;

  statusLine1X = 2;
  statusLine1W = SCREEN_W - 4;
  statusLine1Y = topY - 2;
  statusLine1H = th1 + 4;
  statusLayoutReady = true;

  lastStatusLine1 = line1;
  lastClockText = timeStr;
  lastActiveCount = activeCount;

  gfx->setTextColor(COLOR_TEXT, COLOR_BOX);
  gfx->setCursor((SCREEN_W - tw1) / 2, topY);
  gfx->print(line1);
  gfx->setCursor((SCREEN_W - tw2) / 2, topY + th1 + gap);
  gfx->print(line2);
}

void updateStatusBarClock() {
  if (!statusLayoutReady) {
    drawStatusBar();
    return;
  }
  int activeCount = countActiveDevices();
  String timeStr = getTimeStringShort();
  if (!timeSynced) timeStr = "NTP...";
  String line1 = String(activeCount) + " AKTIF   " + timeStr;
  if (line1 == lastStatusLine1 && activeCount == lastActiveCount) return;
  lastStatusLine1 = line1;
  lastClockText = timeStr;
  lastActiveCount = activeCount;
  int16_t x1, y1; uint16_t tw1, th1;
  gfx->setTextSize(2);
  gfx->getTextBounds(line1.c_str(), 0, 0, &x1, &y1, &tw1, &th1);
  gfx->fillRect(statusLine1X, statusLine1Y, statusLine1W, statusLine1H, COLOR_BOX);
  gfx->setTextColor(COLOR_TEXT, COLOR_BOX);
  gfx->setCursor((SCREEN_W - tw1) / 2, statusLine1Y + 2);
  gfx->print(line1);
}

void drawLogList() {
  int colW = SCREEN_W - SCROLL_COL_W - SCROLLBAR_W;
  int startRow = 1;
  int maxWidth = colW - 16;
  for (int i = 0; i < LOG_LINES_VISIBLE; i++) {
    int idx = logScrollOffset + i;
    int rowY = startRow * ROW_H + i * ROW_H;
    bool isSelected = (idx == selectedLogIndex && idx >= 0 && idx < logCount);
    gfx->fillRect(0, rowY, colW, ROW_H, isSelected ? COLOR_BOX_SEL : COLOR_BG);
    if (idx < logCount) {
      if (isSelected) {
        gfx->drawRect(2, rowY + 2, colW - 4, ROW_H - 4, COLOR_BORDER);
      }
      gfx->setTextColor(logEntries[idx].color);
      gfx->setTextSize(2);
      String line = logEntries[idx].message;
      int16_t x1, y1; uint16_t tw, th;
      gfx->getTextBounds(line, 0, 0, &x1, &y1, &tw, &th);
      if (tw <= maxWidth || ROW_H <= 40) {
        gfx->setCursor(8, rowY + (ROW_H / 2) - (th / 2));
        gfx->print(line);
      } else {
        int splitIdx = -1;
        for (int j = line.length() - 1; j > 0; j--) {
          if (line[j] == ' ') {
            String testLine = line.substring(0, j);
            int16_t tx1, ty1; uint16_t ttw, tth;
            gfx->getTextBounds(testLine, 0, 0, &tx1, &ty1, &ttw, &tth);
            if (ttw <= maxWidth) { splitIdx = j; break; }
          }
        }
        if (splitIdx < 0) splitIdx = line.length() / 2;
        String line1 = line.substring(0, splitIdx);
        String line2 = line.substring(splitIdx);
        line2.trim();
        gfx->setCursor(8, rowY + 8);
        gfx->print(line1);
        gfx->setCursor(8, rowY + 28);
        gfx->print(line2);
      }
    }
    gfx->drawFastHLine(0, rowY + ROW_H - 1, colW, COLOR_SCROLL_OFF);
  }
  drawScrollbarThumb();
  int scrollX = colW + SCROLLBAR_W;
  int scrollY = ROW_H;
  int scrollH = LOG_LINES_VISIBLE * ROW_H;
  int halfH = scrollH / 2;
  bool canUp = (selectedLogIndex > 0 && logCount > 0);
  bool canDown = (selectedLogIndex < logCount - 1 && logCount > 0);
  gfx->fillRect(scrollX, scrollY, SCROLL_COL_W, halfH, canUp ? COLOR_SCROLL_ON : COLOR_SCROLL_OFF);
  gfx->drawRect(scrollX, scrollY, SCROLL_COL_W, halfH, COLOR_BORDER);
  gfx->fillRect(scrollX, scrollY + halfH, SCROLL_COL_W, scrollH - halfH, canDown ? COLOR_SCROLL_ON : COLOR_SCROLL_OFF);
  gfx->drawRect(scrollX, scrollY + halfH, SCROLL_COL_W, scrollH - halfH, COLOR_BORDER);

  int cx = scrollX + SCROLL_COL_W / 2;
  int arrowSize = min(SCROLL_COL_W, halfH) / 3;
  int cyUp = scrollY + halfH / 2;
  gfx->fillTriangle(cx - arrowSize, cyUp + arrowSize / 2, cx + arrowSize, cyUp + arrowSize / 2, cx, cyUp - arrowSize / 2, COLOR_TEXT);
  int cyDown = scrollY + halfH + (scrollH - halfH) / 2;
  gfx->fillTriangle(cx - arrowSize, cyDown - arrowSize / 2, cx + arrowSize, cyDown - arrowSize / 2, cx, cyDown + arrowSize / 2, COLOR_TEXT);

  int btnRow = startRow + LOG_LINES_VISIBLE;
  int btnY = btnRow * ROW_H;
  if (btnY + ROW_H <= SCREEN_H) {
    gfx->fillRect(0, btnY, SCREEN_W, ROW_H, COLOR_OK);
    gfx->drawRect(2, btnY + 2, SCREEN_W - 4, ROW_H - 4, COLOR_BORDER);
    gfx->setTextColor(RGB565_BLACK);
    gfx->setTextSize(2);
    const char* label = "DETAIL";
    int16_t x1, y1; uint16_t tw, th;
    gfx->getTextBounds(label, 0, 0, &x1, &y1, &tw, &th);
    gfx->setCursor((SCREEN_W - tw) / 2, btnY + (ROW_H - th) / 2);
    gfx->print(label);
  }
}

void drawScrollbarThumb() {
  int colW = SCREEN_W - SCROLL_COL_W - SCROLLBAR_W;
  int trackX = colW;
  int trackY = ROW_H;
  int trackH = LOG_LINES_VISIBLE * ROW_H;
  gfx->fillRect(trackX, trackY, SCROLLBAR_W, trackH, COLOR_SCROLLBAR_TRACK);
  if (logCount <= 0) return;
  float visibleFraction = (float)LOG_LINES_VISIBLE / (float)logCount;
  if (visibleFraction > 1.0) visibleFraction = 1.0;
  int minThumbH = 10;
  int thumbH = (int)(trackH * visibleFraction);
  if (thumbH < minThumbH) thumbH = minThumbH;
  if (thumbH > trackH) thumbH = trackH;
  int maxOffset = max(0, logCount - LOG_LINES_VISIBLE);
  int thumbY = trackY;
  if (maxOffset > 0) {
    float scrollFraction = (float)logScrollOffset / (float)maxOffset;
    thumbY = trackY + (int)((trackH - thumbH) * scrollFraction);
  }
  int pad = 2;
  int thumbX = trackX + pad;
  int thumbW = SCROLLBAR_W - (pad * 2);
  int radius = thumbW / 2;
  gfx->fillRoundRect(thumbX, thumbY, thumbW, thumbH, radius, COLOR_SCROLLBAR_THUMB);
}

void drawKeypadScreen() {
  gfx->fillScreen(COLOR_BG);
  int keyW = SCREEN_W / 3;
  int keyH = ROW_H;
  keypadDisplayY0 = 0;
  keypadDisplayH = keyH;
  const char* btnLabels[4][3] = {
    {"1", "2", "3"},
    {"4", "5", "6"},
    {"7", "8", "9"},
    {"HAPUS", "0", "KIRIM"}
  };
  for (int row = 0; row < 4; row++) {
    int y = (row + 1) * keyH;
    for (int col = 0; col < 3; col++) {
      int x = col * keyW;
      uint16_t color = (row == 3 && col == 0) ? COLOR_KEMBALI :
                       (row == 3 && col == 2) ? COLOR_OK : COLOR_BOX;
      gfx->fillRect(x + 2, y + 2, keyW - 4, keyH - 4, color);
      gfx->drawRect(x, y, keyW, keyH, COLOR_BORDER);
      gfx->setTextColor(COLOR_TEXT);
      gfx->setTextSize(2);
      const char* label = btnLabels[row][col];
      int16_t x1, y1; uint16_t tw, th;
      gfx->getTextBounds(label, 0, 0, &x1, &y1, &tw, &th);
      gfx->setCursor(x + (keyW - tw) / 2, y + (keyH - th) / 2);
      gfx->print(label);
    }
  }
  drawKeypadDisplay();
}

void drawKeypadDisplay() {
  gfx->fillRect(0, keypadDisplayY0, SCREEN_W, keypadDisplayH, COLOR_BOX);
  gfx->drawRect(0, keypadDisplayY0, SCREEN_W, keypadDisplayH, COLOR_BORDER);
  gfx->setTextColor(COLOR_TEXT);
  gfx->setTextSize(3);
  String display = enteredNumber;
  if (display.length() == 0) display = " ";
  int16_t x1, y1; uint16_t tw, th;
  gfx->getTextBounds(display.c_str(), 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor((SCREEN_W - tw) / 2, keypadDisplayY0 + (keypadDisplayH - th) / 2);
  gfx->print(display);
}

// =========================================================================
// DASHBOARD
// =========================================================================
void drawDashboardScreen() {
  gfx->fillScreen(COLOR_BG);
  drawStatusBar();

  int infoY = ROW_H;
  gfx->fillRect(0, infoY, SCREEN_W, ROW_H, COLOR_BOX);
  gfx->drawRect(2, infoY + 2, SCREEN_W - 4, ROW_H - 4, COLOR_BORDER);
  gfx->setTextColor(COLOR_TEXT);
  gfx->setTextSize(2);
  gfx->setCursor(4, infoY + 4);
  gfx->print("Kejadian terakhir: ");
  if (lastImportantLog.length() > 0) {
    String display = lastImportantLog;
    if (display.length() > 35) display = display.substring(0, 33) + "...";
    gfx->print(display);
  } else {
    gfx->print("Tidak ada");
  }

  dashboardGridDirty = true;
  drawDashboardGrid(2 * ROW_H);
}

void drawDashboardGrid(int yStart) {
  int gridH = SCREEN_H - yStart;
  int cols = 5;
  int rows = 3;
  int cellW = SCREEN_W / cols;
  int cellH = gridH / rows;
  for (int i = 0; i < DEVICE_MAX_ENTRIES; i++) {
    int col = i % cols;
    int row = i / cols;
    if (row >= rows) break;
    int x = col * cellW;
    int y = yStart + row * cellH;
    uint16_t bg = COLOR_DEV_EMPTY;
    String line1 = "-";
    String line2 = "";
    String line3 = "";
    if (deviceRegistry[i].active) {
      unsigned long ageMs = millis() - deviceRegistry[i].lastSeen;
      unsigned long ageS = ageMs / 1000;
      bool stale = ageMs > (HEARTBEAT_TIMEOUT / 2);
      uint8_t type = deviceRegistry[i].deviceType;
      if (type == 0) bg = stale ? COLOR_DEV_UT_STALE : COLOR_DEV_UT_ONLINE;
      else if (type == 1) bg = stale ? COLOR_DEV_RN_STALE : COLOR_DEV_RN_ONLINE;
      else bg = stale ? COLOR_DEV_SF_STALE : COLOR_DEV_SF_ONLINE;
      line1 = String(deviceRegistry[i].areaId);
      if (type == 0) line2 = "UT";
      else if (type == 1) line2 = "RN";
      else line2 = "SF";
      if (ageS < 60) line3 = String(ageS) + "s";
      else line3 = String(ageS / 60) + "m";
    }
    gfx->fillRect(x + 2, y + 2, cellW - 4, cellH - 4, bg);
    gfx->drawRect(x, y, cellW, cellH, COLOR_BORDER);
    gfx->setTextColor(COLOR_TEXT);
    gfx->setTextSize(2);
    int16_t x1, y1; uint16_t tw, th;
    gfx->getTextBounds(line1.c_str(), 0, 0, &x1, &y1, &tw, &th);
    gfx->setCursor(x + (cellW - tw) / 2, y + 8);
    gfx->print(line1);
    if (line2.length() > 0) {
      gfx->setTextSize(1);
      gfx->getTextBounds(line2.c_str(), 0, 0, &x1, &y1, &tw, &th);
      gfx->setCursor(x + 4, y + cellH - 16);
      gfx->print(line2);
    }
    if (line3.length() > 0) {
      gfx->setTextSize(1);
      gfx->getTextBounds(line3.c_str(), 0, 0, &x1, &y1, &tw, &th);
      gfx->setCursor(x + cellW - tw - 4, y + cellH - 16);
      gfx->print(line3);
    }
  }
}

// =========================================================================
// DETAIL LOG — render dari LogDetail
// =========================================================================
const char* detailKindName(uint8_t kind) {
  switch (kind) {
    case DETAIL_HELP:       return "Help";
    case DETAIL_ACK:        return "Ack";
    case DETAIL_ALARM:      return "Alarm";
    case DETAIL_CANCEL:     return "Batal Help";
    case DETAIL_DATA:       return "Data Lapangan";
    case DETAIL_SUBFIELD:   return "Data Sub-Field";
    case DETAIL_HEARTBEAT:  return "Heartbeat";
    case DETAIL_BEACON:     return "Beacon Hilang";
    case DETAIL_WARNING:    return "Warning";
    case DETAIL_SF_MISSING: return "Sub-Field Hilang";
    case DETAIL_REQ_OUT:    return "Request Data";
    case DETAIL_ACK_OUT:    return "Ack Terkirim";
    case DETAIL_OFFLINE:    return "Device Offline";
    default:                return "Informasi";
  }
}

int printWrapped(const String& text, int x, int y, int firstW, int lineH, int maxLines, int contX, int contW) {
  String rest = text;
  bool first = true;
  int lines = 0;
  int16_t x1, y1; uint16_t tw, th;
  while (rest.length() > 0 && lines < maxLines) {
    int curX = first ? x : contX;
    int curW = first ? firstW : contW;
    gfx->getTextBounds(rest.c_str(), 0, 0, &x1, &y1, &tw, &th);
    if (tw <= curW) {
      gfx->setCursor(curX, y); gfx->print(rest);
      y += lineH; break;
    }
    int splitIdx = -1;
    for (int j = rest.length() - 1; j > 0; j--) {
      if (rest.charAt(j) == ' ') {
        String test = rest.substring(0, j);
        gfx->getTextBounds(test.c_str(), 0, 0, &x1, &y1, &tw, &th);
        if (tw <= curW) { splitIdx = j; break; }
      }
    }
    if (splitIdx < 0) splitIdx = rest.length();
    gfx->setCursor(curX, y);
    gfx->print(rest.substring(0, splitIdx));
    y += lineH; lines++;
    rest = rest.substring(splitIdx);
    rest.trim();
    first = false;
  }
  return y;
}

int printField(int x, int y, const char* label, const String& value, uint16_t valColor, int colW, int lineH) {
  int16_t x1, y1; uint16_t lw, lh;
  gfx->setTextSize(2);
  gfx->getTextBounds(label, 0, 0, &x1, &y1, &lw, &lh);
  gfx->setTextColor(COLOR_LABEL);
  gfx->setCursor(x, y);
  gfx->print(label);
  int vx = x + lw + 6;
  gfx->setTextColor(valColor);
  return printWrapped(value, vx, y, colW - (vx - x), lineH, 2, x, colW);
}

void drawDetailLogScreen(int logIndex) {
  if (logIndex < 0 || logIndex >= logCount) {
    currentState = STATE_LOG;
    drawLogScreen();
    return;
  }
  gfx->fillScreen(COLOR_BG);
  drawStatusBar();

  const LogEntry &e = logEntries[logIndex];
  const LogDetail &d = e.detail;

  const int lineH = 26;
  const int colL = 8;
  const int colR = SCREEN_W / 2 + 8;
  const int colWL = SCREEN_W / 2 - 8;
  const int colWR = SCREEN_W - colR - 8;
  const int maxY = SCREEN_H - ROW_H - 4;
  int y = ROW_H + 6;
  int ya, yb;

  gfx->setTextSize(2);

  ya = printField(colL, y, "Waktu:", d.time.length() ? (d.time + " WIB") : "-", COLOR_TEXT, colWL, lineH);
  yb = printField(colR, y, "Pesan:", detailKindName(d.kind), e.color, colWR, lineH);
  y = max(ya, yb);

  String areaStr = d.areaId ? String(d.areaId) : "-";
  String sfStr   = (d.subFieldId >= 0) ? String(d.subFieldId) : "-";
  if (y < maxY && (areaStr != "-" || sfStr != "-")) {
    ya = printField(colL, y, "Area:", areaStr, COLOR_TEXT, colWL, lineH);
    yb = printField(colR, y, "Subfield:", sfStr, COLOR_TEXT, colWR, lineH);
    y = max(ya, yb);
  }

  String disStr = d.disaster.length() ? d.disaster : "-";
  String latStr = d.hasCoord ? String(d.lat, 5) : "-";
  if (y < maxY && (disStr != "-" || latStr != "-")) {
    ya = printField(colL, y, "Bencana:", disStr, COLOR_TEXT, colWL, lineH);
    yb = printField(colR, y, "Lintang:", latStr, COLOR_TEXT, colWR, lineH);
    y = max(ya, yb);
  }

  String desStr = d.destruction.length() ? d.destruction : "-";
  String lonStr = d.hasCoord ? String(d.lon, 5) : "-";
  if (y < maxY && (desStr != "-" || lonStr != "-")) {
    ya = printField(colL, y, "Kerusakan:", desStr, COLOR_TEXT, colWL, lineH);
    yb = printField(colR, y, "Bujur:", lonStr, COLOR_TEXT, colWR, lineH);
    y = max(ya, yb);
  }

  if (y < maxY && d.hasSensor) {
    ya = printField(colL, y, "MQ2:", String(d.ppm) + " ppm", COLOR_TEXT, colWL, lineH);
    yb = printField(colR, y, "Suhu:", String(d.tempX10 / 10.0, 1) + " C", COLOR_TEXT, colWR, lineH);
    y = max(ya, yb);
  }
  if (y < maxY && d.hasSensor) {
    y = printField(colL, y, "Kelembaban:", String(d.humX10 / 10.0, 0) + " %", COLOR_TEXT, colWL, lineH);
  }

  if (y < maxY && d.note.length() > 0) {
    gfx->setTextColor(COLOR_TEXT);
    y = printWrapped(d.note, colL, y, SCREEN_W - 16, lineH, 2, colL, SCREEN_W - 16);
  }

  if (y < maxY && d.kind == DETAIL_NONE) {
    gfx->setTextColor(COLOR_TEXT);
    printWrapped(e.message, colL, y, SCREEN_W - 16, lineH, 3, colL, SCREEN_W - 16);
  }

  int btnY = SCREEN_H - ROW_H;
  gfx->fillRect(0, btnY, SCREEN_W, ROW_H, COLOR_OK);
  gfx->drawRect(2, btnY + 2, SCREEN_W - 4, ROW_H - 4, COLOR_BORDER);
  gfx->setTextColor(RGB565_BLACK);
  gfx->setTextSize(2);
  const char* label = "KEMBALI";
  int16_t x1, y1; uint16_t tw, th;
  gfx->getTextBounds(label, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor((SCREEN_W - tw) / 2, btnY + (ROW_H - th) / 2);
  gfx->print(label);
}

// =========================================================================
// ALERT
// =========================================================================
void drawAlertScreen() {
  gfx->fillScreen(COLOR_BG);
  int titleY = 0;
  gfx->fillRect(0, titleY, SCREEN_W, ROW_H, alertColor);
  gfx->drawRect(2, titleY + 2, SCREEN_W - 4, ROW_H - 4, COLOR_BORDER);
  gfx->setTextColor(RGB565_WHITE);
  gfx->setTextSize(3);
  int16_t x1, y1; uint16_t tw, th;
  gfx->getTextBounds(alertTitle.c_str(), 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor((SCREEN_W - tw) / 2, titleY + (ROW_H - th) / 2);
  gfx->print(alertTitle);
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565_WHITE);
  gfx->getTextBounds(alertBody.c_str(), 0, 0, &x1, &y1, &tw, &th);
  int maxWidth = SCREEN_W - 40;
  if (tw > maxWidth) {
    int splitIdx = -1;
    for (int j = alertBody.length() - 1; j > 0; j--) {
      if (alertBody[j] == ' ') {
        String testLine = alertBody.substring(0, j);
        int16_t tx1, ty1; uint16_t ttw, tth;
        gfx->getTextBounds(testLine, 0, 0, &tx1, &ty1, &ttw, &tth);
        if (ttw <= maxWidth) { splitIdx = j; break; }
      }
    }
    if (splitIdx < 0) splitIdx = alertBody.length() / 2;
    String line1 = alertBody.substring(0, splitIdx);
    String line2 = alertBody.substring(splitIdx);
    line2.trim();
    gfx->setCursor(20, SCREEN_H / 2 - 15);
    gfx->print(line1);
    gfx->setCursor(20, SCREEN_H / 2 + 15);
    gfx->print(line2);
  } else {
    gfx->setCursor((SCREEN_W - tw) / 2, SCREEN_H / 2 - th / 2);
    gfx->print(alertBody);
  }
  gfx->fillRect(alertBtnX, alertBtnY, alertBtnW, alertBtnH, COLOR_OK);
  gfx->drawRect(alertBtnX, alertBtnY, alertBtnW, alertBtnH, COLOR_BORDER);
  gfx->setTextColor(RGB565_WHITE);
  gfx->setTextSize(3);
  const char* hint = "TEKAN UNTUK TUTUP";
  gfx->getTextBounds(hint, 0, 0, &x1, &y1, &tw, &th);
  gfx->setCursor(alertBtnX + (alertBtnW - tw) / 2, alertBtnY + (alertBtnH - th) / 2);
  gfx->print(hint);
}

void showAlert(const String& title, const String& body, uint16_t color, unsigned long duration) {
  if (currentState == STATE_ALERT) {
    alertTitle = title;
    alertBody = body;
    alertColor = color;
    alertDismissAfter = millis() + duration;
    drawAlertScreen();
    stopBuzzerSequence();
    beepAlert();
    return;
  }
  stateBeforeAlert = currentState;
  alertTitle = title;
  alertBody = body;
  alertColor = color;
  alertDismissAfter = millis() + duration;
  currentState = STATE_ALERT;
  drawAlertScreen();
  beepAlert();
}

void dismissAlert() {
  stopBuzzerSequence();
  currentState = stateBeforeAlert;
  if (currentState == STATE_LOG) {
    drawLogScreen();
  } else if (currentState == STATE_KEYPAD) {
    drawKeypadScreen();
  } else if (currentState == STATE_DASHBOARD) {
    dashboardGridDirty = true;
    drawDashboardScreen();
  } else if (currentState == STATE_DETAIL_LOG) {
    currentState = STATE_LOG;
    drawLogScreen();
  }
}

// =========================================================================
// ADD LOG ENTRY (overload)
// =========================================================================
void addLogEntry(const String& msg, uint16_t color) {
  LogDetail dummy;
  addLogEntry(msg, color, dummy);
}

void addLogEntry(const String& msg, uint16_t color, const LogDetail& d) {
  LogEntry entry;
  entry.message = msg;
  entry.color = color;
  entry.timestamp = millis();
  entry.detail = d;
  entry.detail.time = getTimeStringShort();

  if (logCount < MAX_LOG_ENTRIES) {
    logEntries[logCount++] = entry;
  } else {
    for (int i = 0; i < MAX_LOG_ENTRIES - 1; i++) {
      logEntries[i] = logEntries[i + 1];
    }
    logEntries[MAX_LOG_ENTRIES - 1] = entry;
  }
  if (logFollowingBottom) {
    if (logCount > LOG_LINES_VISIBLE) logScrollOffset = logCount - LOG_LINES_VISIBLE;
    else logScrollOffset = 0;
  }
  if (logFollowingBottom && logCount > 0) {
    selectedLogIndex = logCount - 1;
  } else if (logCount == 0) {
    selectedLogIndex = -1;
  }

  if (currentState == STATE_LOG) {
    drawLogList();
    drawStatusBar();
  }
  if (color == COLOR_LOG_WARN || color == COLOR_LOG_ERROR || color == COLOR_LOG_DATA) {
    lastImportantLog = msg;
  }

  pushLogToFirebase(entry);
}

// =========================================================================
// DEVICE REGISTRY
// =========================================================================
int findDevice(uint16_t areaId) {
  for (int i = 0; i < DEVICE_MAX_ENTRIES; i++) {
    if (deviceRegistry[i].active && deviceRegistry[i].areaId == areaId) {
      return i;
    }
  }
  return -1;
}

void registerDevice(uint16_t areaId, uint8_t type) {
  int idx = findDevice(areaId);
  if (idx >= 0) {
    deviceRegistry[idx].lastSeen = millis();
    bool typeChanged = (deviceRegistry[idx].deviceType != type);
    if (typeChanged) {
      deviceRegistry[idx].deviceType = type;
      dashboardGridDirty = true;
    }
    if (typeChanged || millis() - deviceRegistry[idx].lastPushed >= DEVICE_PUSH_MIN_INTERVAL_MS) {
      pushDeviceStatus(areaId, deviceRegistry[idx].deviceType, true);
      deviceRegistry[idx].lastPushed = millis();
    }
    return;
  }
  for (int i = 0; i < DEVICE_MAX_ENTRIES; i++) {
    if (!deviceRegistry[i].active) {
      deviceRegistry[i].areaId = areaId;
      deviceRegistry[i].deviceType = type;
      deviceRegistry[i].active = true;
      deviceRegistry[i].lastSeen = millis();
      dashboardGridDirty = true;
      pushDeviceStatus(areaId, type, true);
      deviceRegistry[i].lastPushed = millis();
      return;
    }
  }
  unsigned long oldest = millis();
  int oldestIdx = 0;
  for (int i = 0; i < DEVICE_MAX_ENTRIES; i++) {
    if (deviceRegistry[i].lastSeen < oldest) {
      oldest = deviceRegistry[i].lastSeen;
      oldestIdx = i;
    }
  }
  deviceRegistry[oldestIdx].areaId = areaId;
  deviceRegistry[oldestIdx].deviceType = type;
  deviceRegistry[oldestIdx].active = true;
  deviceRegistry[oldestIdx].lastSeen = millis();
  dashboardGridDirty = true;
  pushDeviceStatus(areaId, type, true);
  deviceRegistry[oldestIdx].lastPushed = millis();
}

void updateDeviceSeen(uint16_t areaId) {
  int idx = findDevice(areaId);
  if (idx >= 0) {
    deviceRegistry[idx].lastSeen = millis();
    if (millis() - deviceRegistry[idx].lastPushed >= DEVICE_PUSH_MIN_INTERVAL_MS) {
      pushDeviceStatus(areaId, deviceRegistry[idx].deviceType, true);
      deviceRegistry[idx].lastPushed = millis();
    }
  } else {
    registerDevice(areaId, 0);
  }
}

int countActiveDevices() {
  int count = 0;
  unsigned long now = millis();
  for (int i = 0; i < DEVICE_MAX_ENTRIES; i++) {
    if (deviceRegistry[i].active && (now - deviceRegistry[i].lastSeen < HEARTBEAT_TIMEOUT)) {
      count++;
    }
  }
  return count;
}

void pruneInactiveDevices() {
  if (millis() - lastPruneTime < PRUNE_INTERVAL) return;
  lastPruneTime = millis();
  unsigned long now = millis();
  for (int i = 0; i < DEVICE_MAX_ENTRIES; i++) {
    if (deviceRegistry[i].active && (now - deviceRegistry[i].lastSeen > HEARTBEAT_TIMEOUT)) {
      deviceRegistry[i].active = false;
      dashboardGridDirty = true;
      String msg = "Area " + String(deviceRegistry[i].areaId) + " offline";
      LogDetail d; d.kind = DETAIL_OFFLINE; d.areaId = deviceRegistry[i].areaId; d.note = "Heartbeat timeout";
      addLogEntry(msg, COLOR_LOG_WARN, d);
      pushDeviceStatus(deviceRegistry[i].areaId, deviceRegistry[i].deviceType, false);
    }
  }
}

// =========================================================================
// FIREBASE SENDER
// =========================================================================
String authQueryParam() {
  if (FIREBASE_SECRET.length() == 0 || FIREBASE_SECRET == "ISI_DATABASE_SECRET_DI_SINI") {
    return "";
  }
  return "?auth=" + FIREBASE_SECRET;
}

void sendToFirebase(String path, String jsonData) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, cannot send to Firebase.");
    return;
  }
  HTTPClient http;
  String url = FIREBASE_URL + path + ".json" + authQueryParam();
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.PUT(jsonData);
  if (code > 0) {
    Serial.printf("Firebase PUT success, code: %d\n", code);
  } else {
    Serial.printf("Firebase PUT failed, code: %d\n", code);
  }
  http.end();
}

// Partial update — only touches the fields present in jsonData, unlike
// sendToFirebase's PUT which would overwrite the whole node.
void sendFirebasePatch(String path, String jsonData) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, cannot patch Firebase.");
    return;
  }
  HTTPClient http;
  String url = FIREBASE_URL + path + ".json" + authQueryParam();
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.PATCH(jsonData);
  if (code > 0) {
    Serial.printf("Firebase PATCH success, code: %d\n", code);
  } else {
    Serial.printf("Firebase PATCH failed, code: %d\n", code);
  }
  http.end();
}

void recordHelpFirebaseKey(uint16_t areaId, unsigned long key) {
  for (int i = 0; i < HELP_KEY_MAX_ENTRIES; i++) {
    if (helpKeyRegistry[i].active && helpKeyRegistry[i].areaId == areaId) {
      helpKeyRegistry[i].firebaseKey = key;
      return;
    }
  }
  for (int i = 0; i < HELP_KEY_MAX_ENTRIES; i++) {
    if (!helpKeyRegistry[i].active) {
      helpKeyRegistry[i].areaId = areaId;
      helpKeyRegistry[i].firebaseKey = key;
      helpKeyRegistry[i].active = true;
      return;
    }
  }
  // Registry full — reuse slot 0 rather than silently dropping the newest HELP.
  helpKeyRegistry[0].areaId = areaId;
  helpKeyRegistry[0].firebaseKey = key;
  helpKeyRegistry[0].active = true;
}

bool findHelpFirebaseKey(uint16_t areaId, unsigned long &outKey) {
  for (int i = 0; i < HELP_KEY_MAX_ENTRIES; i++) {
    if (helpKeyRegistry[i].active && helpKeyRegistry[i].areaId == areaId) {
      outKey = helpKeyRegistry[i].firebaseKey;
      return true;
    }
  }
  return false;
}

String jsonEscape(const String& s) {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') out += '\\';
    out += c;
  }
  return out;
}

// Mirrors deviceRegistry to Firebase so the admin app can show which areas
// are currently online — this is the "active/monitored areas" board.
void pushDeviceStatus(uint16_t areaId, uint8_t deviceType, bool active) {
  if (WiFi.status() != WL_CONNECTED) return;
  time_t now = time(nullptr);
  if (!timeSynced) now = millis() / 1000 + compileTimeToTimestamp();
  String path = "/devices/" + String(areaId);
  String json = "{";
  json += "\"areaId\":" + String(areaId) + ",";
  json += "\"deviceType\":" + String(deviceType) + ",";
  json += "\"active\":" + String(active ? "true" : "false") + ",";
  json += "\"lastSeen\":" + String(now);
  json += "}";
  sendToFirebase(path, json);
}

// Mirrors every addLogEntry() call to Firebase — a single hook point means
// every event kind (HELP, HEARTBEAT, DATA, WARNING, BEACON HILANG, ...)
// reaches the app automatically, with the same throttling the on-screen
// log already applies (e.g. heartbeat log entries are already rate-limited
// before they ever call addLogEntry).
// Uses HTTP POST (Firebase "push") instead of a millis()-based key so
// entries get a real unique, time-ordered key — a millis()-based key would
// collide across reboots since millis() resets to 0 every boot.
void pushLogToFirebase(const LogEntry& entry) {
  if (WiFi.status() != WL_CONNECTED) return;
  time_t now = time(nullptr);
  if (!timeSynced) now = millis() / 1000 + compileTimeToTimestamp();

  String json = "{";
  json += "\"kind\":" + String(entry.detail.kind) + ",";
  json += "\"areaId\":" + String(entry.detail.areaId) + ",";
  json += "\"message\":\"" + jsonEscape(entry.message) + "\",";
  json += "\"timestamp\":" + String(now);
  json += "}";

  HTTPClient http;
  String url = FIREBASE_URL + "/logs.json" + authQueryParam();
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(json);
  if (code <= 0) {
    Serial.printf("Firebase log POST failed, code: %d\n", code);
  }
  http.end();
}

// Polls /commands for requests the admin app's keypad has written, and
// triggers the same sendFieldDataRequest() the physical keypad uses.
void checkFirebaseCommands() {
  if (millis() - lastCommandPoll < COMMAND_POLL_INTERVAL_MS) return;
  lastCommandPoll = millis();
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  String url = FIREBASE_URL + "/commands.json" + authQueryParam();
  http.begin(url);
  int code = http.GET();
  if (code != 200) {
    http.end();
    return;
  }
  String payload = http.getString();
  http.end();
  if (payload.length() < 3 || payload == "null") return;

  DynamicJsonDocument doc(2048);
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.printf("Gagal parse /commands: %s\n", err.c_str());
    return;
  }

  JsonObject root = doc.as<JsonObject>();
  for (JsonPair kv : root) {
    uint16_t areaId = (uint16_t) String(kv.key().c_str()).toInt();
    if (areaId == 0) continue;
    JsonObject cmd = kv.value().as<JsonObject>();
    bool wantsFieldData = cmd["requestFieldData"] | false;
    if (wantsFieldData) {
      sendFieldDataRequest(areaId);
      sendFirebasePatch("/commands/" + String(areaId), "{\"requestFieldData\":false}");
    }
  }
}

// =========================================================================
// INCOMING LORA
// =========================================================================
void checkIncomingLoRa() {
  if (Serial2.available() < 3) return;
  uint8_t header[3];
  size_t len = Serial2.readBytes(header, 3);
  if (len < 3) return;
  uint8_t type = header[0];
  size_t totalSize = messageSize(type);
  if (totalSize == 0) { while (Serial2.available()) Serial2.read(); return; }
  if (totalSize > 100) { while (Serial2.available()) Serial2.read(); return; }
  if (Serial2.available() < (int)(totalSize - 3)) {
    unsigned long start = millis();
    while (Serial2.available() < (int)(totalSize - 3)) {
      if (millis() - start > 500) { while (Serial2.available()) Serial2.read(); return; }
      delay(1);
    }
  }
  uint8_t buffer[100];
  memcpy(buffer, header, 3);
  Serial2.readBytes(buffer + 3, totalSize - 3);
  uint16_t areaId = (buffer[2] << 8) | buffer[1];
  uint32_t msgHash = computeHash(buffer, totalSize);
  bool isDuplicate = isMessageInHistory(msgHash);
  if (type == MSG_CANCEL_HELP || type == MSG_HELP || type == MSG_ALARM) {
    sendAck(areaId);
  }
  if (isDuplicate) return;
  addToMessageHistory(msgHash);
  switch (type) {
    case MSG_HELP:
      if (totalSize >= sizeof(HelpMessage)) handleHelp((HelpMessage*)buffer);
      break;
    case MSG_ACK:
      if (totalSize >= sizeof(AckMessage)) handleAck((AckMessage*)buffer);
      break;
    case MSG_ALARM:
      if (totalSize >= sizeof(AlarmMessage)) handleAlarm((AlarmMessage*)buffer);
      break;
    case MSG_CANCEL_HELP:
      if (totalSize >= sizeof(CancelHelpMessage)) handleCancelHelp((CancelHelpMessage*)buffer);
      break;
    case MSG_FIELD_DATA_RESPONSE:
      if (totalSize >= sizeof(FieldDataResponseMessage)) handleFieldDataResponse((FieldDataResponseMessage*)buffer);
      break;
    case MSG_HEARTBEAT:
      if (totalSize >= sizeof(HeartbeatMessage)) handleHeartbeat((HeartbeatMessage*)buffer);
      break;
    case MSG_MISSING_BEACON:
      if (totalSize >= sizeof(MissingBeaconMessage)) handleMissingBeacon((MissingBeaconMessage*)buffer);
      break;
    case MSG_WARNING_FORWARD:
      if (totalSize >= sizeof(WarningForwardMessage)) handleWarningForward((WarningForwardMessage*)buffer);
      break;
    case MSG_WARNING_ACK:
      break;
    default:
      break;
  }
}

// =========================================================================
// MESSAGE HANDLERS
// =========================================================================
void handleHelp(HelpMessage* msg) {
  registerDevice(msg->areaId, 0);
  helpCount++;
  dashboardGridDirty = true;
  String disaster = (msg->disaster < NUM_DISASTERS) ? DISASTER_NAMES[msg->disaster] : "Unknown";
  String destr = (msg->destruction < NUM_DESTRUCTIONS) ? DESTRUCTION_NAMES[msg->destruction] : "?";
  String logMsg = getTimeStringShort() + " HELP " + String(msg->areaId) + ": " + disaster + "/" + destr;
  LogDetail d;
  d.kind = DETAIL_HELP;
  d.areaId = msg->areaId;
  d.disaster = disaster;
  d.destruction = destr;
  if (msg->latitude != 0.0 || msg->longitude != 0.0) {
    logMsg += " " + String(msg->latitude, 5) + "," + String(msg->longitude, 5);
    d.lat = msg->latitude; d.lon = msg->longitude; d.hasCoord = true;
  }
  addLogEntry(logMsg, COLOR_LOG_WARN, d);
  showAlert("LAPORAN BENCANA", "Area " + String(msg->areaId) + " - " + disaster, RGB565_ORANGE, 15000);

  time_t now = time(nullptr);
  if (!timeSynced) now = millis() / 1000 + compileTimeToTimestamp();
  unsigned long firebaseKey = millis();
  String path = "/emergencies/" + String(firebaseKey);
  String json = "{";
  json += "\"utId\":\"" + String(msg->areaId) + "\",";
  json += "\"disasterCode\":" + String(msg->disaster) + ",";
  json += "\"destructionCode\":" + String(msg->destruction) + ",";
  json += "\"lat\":" + String(msg->latitude, 6) + ",";
  json += "\"lon\":" + String(msg->longitude, 6) + ",";
  json += "\"timestamp\":" + String(now) + ",";
  json += "\"status\":\"baru\"";
  json += "}";
  sendToFirebase(path, json);
  recordHelpFirebaseKey(msg->areaId, firebaseKey);
}

void handleAck(AckMessage* msg) {
  updateDeviceSeen(msg->areaId);
  LogDetail d; d.kind = DETAIL_ACK; d.areaId = msg->areaId;
  addLogEntry(getTimeStringShort() + " ACK dari " + String(msg->areaId), COLOR_LOG_SUCCESS, d);
}

void handleAlarm(AlarmMessage* msg) {
  registerDevice(msg->areaId, 0);
  alarmCount++;
  dashboardGridDirty = true;
  String logMsg = getTimeStringShort() + " ALARM " + String(msg->areaId) + " gerak";
  LogDetail d;
  d.kind = DETAIL_ALARM; d.areaId = msg->areaId;
  d.note = "Pergerakan terdeteksi";
  if (msg->latitude != 0.0 || msg->longitude != 0.0) {
    logMsg += " " + String(msg->latitude, 5) + "," + String(msg->longitude, 5);
    d.lat = msg->latitude; d.lon = msg->longitude; d.hasCoord = true;
  }
  addLogEntry(logMsg, COLOR_LOG_ERROR, d);
  showAlert("ALARM PERGERAKAN", "Area " + String(msg->areaId) + " bergerak", RGB565_RED, 15000);

  time_t now = time(nullptr);
  if (!timeSynced) now = millis() / 1000 + compileTimeToTimestamp();
  String path = "/emergencies/" + String(millis());
  String json = "{";
  json += "\"utId\":\"" + String(msg->areaId) + "\",";
  json += "\"disasterCode\":3,";
  json += "\"destructionCode\":0,";
  json += "\"lat\":" + String(msg->latitude, 6) + ",";
  json += "\"lon\":" + String(msg->longitude, 6) + ",";
  json += "\"timestamp\":" + String(now) + ",";
  json += "\"status\":\"baru\"";
  json += "}";
  sendToFirebase(path, json);
}

// Cari event TERAKHIR (HELP atau CANCEL) untuk areaId ini di log.
// Return true hanya jika event terakhir itu HELP dan umurnya <= 5 menit.
bool hasRecentHelp(uint16_t areaId) {
  unsigned long now = millis();
  for (int i = logCount - 1; i >= 0; i--) {
    if (logEntries[i].detail.areaId != areaId) continue;
    if (logEntries[i].detail.kind != DETAIL_HELP &&
        logEntries[i].detail.kind != DETAIL_CANCEL) continue;

    // Ini event HELP/CANCEL paling baru untuk area ini.
    if (logEntries[i].detail.kind != DETAIL_HELP) return false; // sudah dibatalkan sebelumnya

    unsigned long age = now - logEntries[i].timestamp;
    return (age <= HELP_CANCEL_WINDOW_MS);
  }
  return false; // tidak ada HELP sama sekali di log
}

void handleCancelHelp(CancelHelpMessage* msg) {
  updateDeviceSeen(msg->areaId);

  if (!hasRecentHelp(msg->areaId)) {
    LogDetail d; d.kind = DETAIL_NONE; d.areaId = msg->areaId;
    d.note = "Cancel ditolak: tidak ada HELP dalam 5 menit terakhir";
    addLogEntry(getTimeStringShort() + " CANCEL DITOLAK " + String(msg->areaId), COLOR_LOG_ERROR, d);
    return;
  }

  cancelCount++;
  dashboardGridDirty = true;
  LogDetail d; d.kind = DETAIL_CANCEL; d.areaId = msg->areaId;
  addLogEntry(getTimeStringShort() + " BATAL HELP " + String(msg->areaId), COLOR_LOG_WARN, d);

  unsigned long key;
  if (findHelpFirebaseKey(msg->areaId, key)) {
    String path = "/emergencies/" + String(key);
    sendFirebasePatch(path, "{\"status\":\"dibatalkan\"}");
    for (int i = 0; i < HELP_KEY_MAX_ENTRIES; i++) {
      if (helpKeyRegistry[i].active && helpKeyRegistry[i].areaId == msg->areaId) {
        helpKeyRegistry[i].active = false;
        break;
      }
    }
  } else {
    Serial.printf("CANCEL diterima untuk area %d tapi key Firebase tidak ditemukan\n", msg->areaId);
  }
}

void handleFieldDataResponse(FieldDataResponseMessage* msg) {
  registerDevice(msg->areaId, 0);
  dashboardGridDirty = true;
  String header = getTimeStringShort() + " DATA " + String(msg->areaId);
  LogDetail dh;
  dh.kind = DETAIL_DATA; dh.areaId = msg->areaId;
  if (msg->latitude != 0.0 || msg->longitude != 0.0) {
    header += " " + String(msg->latitude, 5) + "," + String(msg->longitude, 5);
    dh.lat = msg->latitude; dh.lon = msg->longitude; dh.hasCoord = true;
  }
  addLogEntry(header, COLOR_LOG_DATA, dh);
  for (int i = 0; i < MAX_SUBFIELDS; i++) {
    if (msg->subFields[i].subFieldId == 0) continue;
    float temp = msg->subFields[i].temperatureX10 / 10.0;
    float hum  = msg->subFields[i].humidityX10 / 10.0;
    String line = "SF" + String(msg->subFields[i].subFieldId) + ": " +
                  String(msg->subFields[i].mq2Ppm) + "ppm " + String(temp, 1) + "C " + String(hum, 0) + "%";
    LogDetail ds;
    ds.kind = DETAIL_SUBFIELD;
    ds.areaId = msg->areaId;
    ds.subFieldId = msg->subFields[i].subFieldId;
    ds.ppm = msg->subFields[i].mq2Ppm;
    ds.tempX10 = msg->subFields[i].temperatureX10;
    ds.humX10 = msg->subFields[i].humidityX10;
    ds.hasSensor = true;
    addLogEntry(line, COLOR_LOG_DATA, ds);
  }
}

void handleHeartbeat(HeartbeatMessage* msg) {
  registerDevice(msg->areaId, (msg->senderBeacon == 0) ? 0 : 1);
  dashboardGridDirty = true;
  if (millis() - lastHeartbeatLog > HEARTBEAT_LOG_INTERVAL) {
    LogDetail d; d.kind = DETAIL_HEARTBEAT; d.areaId = msg->areaId;
    d.note = String(countActiveDevices()) + " device aktif";
    addLogEntry(getTimeStringShort() + " HB: " + String(countActiveDevices()) + " aktif", COLOR_LOG_INFO, d);
    lastHeartbeatLog = millis();
  }
}

void handleMissingBeacon(MissingBeaconMessage* msg) {
  registerDevice(msg->areaId, 1);
  missingBeaconCount++;
  dashboardGridDirty = true;
  LogDetail d; d.kind = DETAIL_BEACON; d.areaId = msg->areaId; d.subFieldId = msg->missingBeacon;
  addLogEntry(getTimeStringShort() + " BEACON HILANG " + String(msg->areaId) + " #" + String(msg->missingBeacon), COLOR_LOG_ERROR, d);
  showAlert("BEACON HILANG", "Area " + String(msg->areaId) + " node #" + String(msg->missingBeacon), RGB565_RED, 10000);
}

void handleWarningForward(WarningForwardMessage* msg) {
  registerDevice(msg->areaId, 0);
  dashboardGridDirty = true;
  if (msg->disaster == FWD_REASON_SUBFIELD_MISSING) {
    queueMissingSubfield(msg->areaId, msg->subFieldId);
    return;
  }
  warningCount++;
  String disaster;
  if (msg->disaster < NUM_FWD_REASONS) disaster = FWD_REASON_NAMES[msg->disaster];
  else if (msg->disaster < NUM_DISASTERS) disaster = DISASTER_NAMES[msg->disaster];
  else disaster = "Unknown";
  String logMsg = getTimeStringShort() + " WARNING " + String(msg->areaId) + ": " + disaster;
  LogDetail d;
  d.kind = DETAIL_WARNING; d.areaId = msg->areaId; d.disaster = disaster;
  d.subFieldId = msg->subFieldId;
  if (msg->latitude != 0.0 || msg->longitude != 0.0) {
    logMsg += " " + String(msg->latitude, 5) + "," + String(msg->longitude, 5);
    d.lat = msg->latitude; d.lon = msg->longitude; d.hasCoord = true;
  }
  addLogEntry(logMsg, COLOR_LOG_ERROR, d);
  showAlert("PERINGATAN", "Area " + String(msg->areaId) + " " + disaster, RGB565_RED, 15000);

  time_t now = time(nullptr);
  if (!timeSynced) now = millis() / 1000 + compileTimeToTimestamp();
  String path = "/emergencies/" + String(millis());
  String json = "{";
  json += "\"utId\":\"" + String(msg->areaId) + "\",";
  json += "\"disasterCode\":" + String(msg->disaster) + ",";
  json += "\"destructionCode\":0,";
  json += "\"subFieldId\":" + String(msg->subFieldId) + ",";
  json += "\"lat\":" + String(msg->latitude, 6) + ",";
  json += "\"lon\":" + String(msg->longitude, 6) + ",";
  json += "\"timestamp\":" + String(now) + ",";
  json += "\"status\":\"baru\"";
  json += "}";
  sendToFirebase(path, json);
}

void queueMissingSubfield(uint16_t areaId, uint8_t subFieldId) {
  unsigned long now = millis();
  if (!missingAggPending || areaId != missingAggAreaId) {
    missingAggAreaId = areaId;
    missingAggCount = 0;
    missingAggPending = true;
  }
  bool alreadyQueued = false;
  for (int i = 0; i < missingAggCount; i++) {
    if (missingAggSubFields[i] == subFieldId) { alreadyQueued = true; break; }
  }
  if (!alreadyQueued && missingAggCount < MAX_PENDING_MISSING_SUBFIELDS) {
    missingAggSubFields[missingAggCount++] = subFieldId;
  }
  missingAggDeadline = now + SUBFIELD_MISSING_WINDOW_MS;
  String logMsg = getTimeStringShort() + " SF HILANG " + String(areaId) + " #" + String(subFieldId);
  LogDetail d; d.kind = DETAIL_SF_MISSING; d.areaId = areaId; d.subFieldId = subFieldId;
  addLogEntry(logMsg, COLOR_LOG_WARN, d);
}

void checkMissingSubfieldBatch() {
  if (!missingAggPending) return;
  if (millis() < missingAggDeadline) return;
  String ids = "";
  for (int i = 0; i < missingAggCount; i++) {
    if (i > 0) ids += " & ";
    ids += "#" + String(missingAggSubFields[i]);
  }
  showAlert("SUB-FIELD HILANG", "Area " + String(missingAggAreaId) + " " + ids, RGB565_RED, 15000);
  missingAggPending = false;
  missingAggCount = 0;
}

// =========================================================================
// OUTBOUND
// =========================================================================
void sendAck(uint16_t areaId) {
  setLoRaNormalMode();
  delay(400);
  AckMessage msg;
  msg.type = MSG_ACK;
  msg.areaId = areaId;
  Serial2.write((uint8_t*)&msg, sizeof(msg));
  LogDetail d; d.kind = DETAIL_ACK_OUT; d.areaId = areaId;
  addLogEntry(getTimeStringShort() + " ACK->" + String(areaId), COLOR_LOG_SUCCESS, d);
}

void sendFieldDataRequest(uint16_t areaId) {
  setLoRaNormalMode();
  delay(10);
  FieldDataRequestMessage msg;
  msg.type = MSG_FIELD_DATA_REQUEST;
  msg.areaId = areaId;
  Serial2.write((uint8_t*)&msg, sizeof(msg));
  LogDetail d; d.kind = DETAIL_REQ_OUT; d.areaId = areaId;
  addLogEntry(getTimeStringShort() + " REQ->" + String(areaId), COLOR_LOG_DATA, d);
}

void broadcastFieldDataRequest() {
  int sentCount = 0;
  for (int i = 0; i < DEVICE_MAX_ENTRIES; i++) {
    if (deviceRegistry[i].active) {
      sendFieldDataRequest(deviceRegistry[i].areaId);
      sentCount++;
      delay(100);
    }
  }
  LogDetail d; d.kind = DETAIL_REQ_OUT;
  if (sentCount == 0) {
    d.note = "Tidak ada device";
    addLogEntry("Tidak ada device", COLOR_LOG_WARN, d);
  } else {
    d.note = "Broadcast ke " + String(sentCount) + " device";
    addLogEntry("REQ " + String(sentCount) + " device", COLOR_LOG_DATA, d);
  }
}

void sendRequestFromKeypad(const String& areaIdStr) {
  uint16_t areaId = areaIdStr.toInt();
  if (areaId == 0) {
    LogDetail d; d.kind = DETAIL_NONE; d.note = "Area ID invalid";
    addLogEntry("Area ID invalid", COLOR_LOG_WARN, d);
    return;
  }
  sendFieldDataRequest(areaId);
}

// =========================================================================
// BUZZER
// =========================================================================
void beep(uint16_t freq, unsigned long duration) {
  tone(BUZZER_PIN, freq, duration);
  delay(duration);
  noTone(BUZZER_PIN);
}

// --- beepAlert() tidak berubah, cuma trigger state machine ---
void beepAlert() {
  buzzerSequenceActive = true;
  buzzerSequenceStep = 0;
  buzzerSequenceLastTick = millis();
}

// --- stopBuzzerSequence() ganti noTone() jadi digitalWrite LOW ---
void stopBuzzerSequence() {
  digitalWrite(BUZZER_PIN, LOW);
  buzzerSequenceActive = false;
}

// --- updateBuzzerSequence() ganti tone()/noTone() jadi digitalWrite HIGH/LOW ---
void updateBuzzerSequence() {
  if (!buzzerSequenceActive) return;
  unsigned long now = millis();
  unsigned long elapsed = now - buzzerSequenceLastTick;
  switch (buzzerSequenceStep) {
    case 0:
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerSequenceLastTick = now;
      buzzerSequenceStep = 1;
      break;
    case 1:
      if (elapsed >= 1000) {              // <-- durasi "tuuut" panjang, atur sesuai selera
        digitalWrite(BUZZER_PIN, LOW);
        buzzerSequenceLastTick = now;
        buzzerSequenceStep = 2;
      }
      break;
    case 2:
      if (elapsed >= 500) {               // <-- jeda sebelum tuut berikutnya
        buzzerSequenceLastTick = now;
        buzzerSequenceStep = 0;           // loop lagi ke ON
      }
      break;
  }
}
