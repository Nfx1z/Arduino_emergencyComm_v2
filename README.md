# 🚨 Arduino Emergency Communication v2 - Field Device Firmware

Advanced emergency communication firmware for ESP32-based field devices using **LoRa wireless technology**. Part of the SIGAP (Sistem Informasi Gawat Darurat - Emergency Information System) ecosystem.

> **Version 2.0** - Enhanced from the initial disaster communication system with improved relay routing, heartbeat monitoring, and multi-node support.

## 📋 Overview

This repository contains the embedded firmware for three types of emergency communication nodes:

1. **Field Device (UT)** - End-user device for emergency reporting
2. **HQ Device (MT)** - Central coordination hub receiving and distributing alerts
3. **Beacon/Relay Node (RN)** - Intermediate relay nodes extending communication range

The system uses **LoRa (Long Range)** radio technology to enable emergency communication when traditional networks are unavailable.

## 🏗️ System Architecture

### SIGAP Ecosystem Components

```
Citizens (sigapUser Mobile App)
    ↓
Field Devices (This Repository - ESP32/Arduino with LoRa)
    ↓
Admin Dashboard (Android_sigapAdmin)
    ↓
Government Agencies & Emergency Services
```

### Network Topology

```
HQ Device (MT) — Central Coordinator
        ↑
        |
    Beacon 2 (RN) — Relay/Relay Node
        ↑
        |
    Beacon 1 (RN) — Relay Node
        ↑
        |
Field Device (UT) — Emergency Reporter
```

## 🚀 Features

### Field Device (UT) - `field_device.ino`
- **Emergency Reporting**: Send help requests with GPS coordinates
- **Disaster Classification**: Categorize emergencies (fire, medical, police, etc.)
- **Destruction Level**: Specify severity/damage assessment
- **Message Acknowledgment**: Receive confirmation from HQ
- **Heartbeat Monitoring**: Keep-alive signals with relay nodes
- **Sub-field Support**: Manage multiple sensor-equipped sub-devices
- **Data Requests**: Query sensor data from sub-field devices (MQ2, temperature, humidity)

### HQ Device (MT) - `hq.ino`
- **Central Message Hub**: Receive and process all emergency messages
- **Message Distribution**: Relay critical alerts through beacon network
- **Field Device Management**: Track active field devices and their status
- **Alert Generation**: Issue system-wide warnings and alarms
- **Data Aggregation**: Collect sensor data from sub-field devices
- **Missing Beacon Detection**: Monitor relay node availability

### Beacon/Relay Node (RN) - `beacon.ino`
- **Message Relaying**: Forward messages between Field Devices and HQ
- **Hop Routing**: Support multi-hop communication for extended range
- **Heartbeat Exchange**: Verify connectivity with adjacent nodes
- **Duplicate Detection**: Prevent message flooding through history tracking
- **Power Management**: Low-power operation for extended deployment
- **State Machine Parser**: Robust LoRa packet reception with timeout handling

### Sub-Field Device (SF) - `sub_field.ino`
- **Sensor Integration**: Collect environmental data
  - MQ2 Gas Sensor (smoke/gas detection)
  - Temperature & Humidity sensors
- **Wireless Transmission**: Report sensor data via LoRa
- **Autonomous Operation**: Battery-powered with deep sleep capability
- **Alert Triggers**: Send warnings when thresholds exceeded

## 📱 Technology Stack

- **Microcontroller**: ESP32 (or compatible Arduino boards)
- **Wireless**: LoRa E220 Module (900 MHz / 433 MHz)
- **Architecture**: Clean, modular C++ with message-based protocol
- **Power Management**: Deep sleep and low-power modes support
- **Sensors**: MQ2 gas sensor, DHT/BME temperature/humidity sensors
- **Communication Protocol**: Binary packed structs with type-safe messaging

## ⚙️ Hardware Requirements

### Essential Components
- **ESP32 Development Board** (or Arduino with LoRa shield)
- **LoRa Module**: E220 (900 MHz recommended)
- **Antenna**: LoRa antenna (SMA connector or wire)
- **Power Supply**: USB or battery (3.3V regulated)
- **USB Cable**: For programming and serial debugging

### Optional Sensors (for Sub-field nodes)
- **MQ2 Gas Sensor** - Smoke/gas detection
- **DHT22/BME680** - Temperature & humidity
- **GPS Module** - Location tracking (for field devices)
- **Battery Shield** - Extended operation

### Pin Configuration (ESP32)

```cpp
LoRa E220 Connections:
- RX: GPIO 16 (Serial2.RX)
- TX: GPIO 17 (Serial2.TX)
- M0: GPIO 12 (Mode select 0)
- M1: GPIO 13 (Mode select 1)
- AUX: GPIO 14 (Auxiliary)

Sensor Connections:
- MQ2: A0 (Analog)
- DHT: GPIO 5 (or configurable)
- GPS RX: GPIO 9
- GPS TX: GPIO 10
```

## 📝 Message Types

The firmware supports 13 different message types:

| Type | Size | Direction | Purpose |
|------|------|-----------|---------|
| `MSG_HELP` (1) | 19 bytes | UT → HQ | Emergency help request |
| `MSG_ACK` (2) | 3 bytes | HQ → UT | Acknowledge receipt |
| `MSG_ALARM` (3) | 11 bytes | UT → HQ | Alarm with location |
| `MSG_CANCEL_HELP` (4) | 3 bytes | UT → HQ | Cancel previous help request |
| `MSG_FIELD_DATA_REQUEST` (5) | 3 bytes | HQ → UT | Request field data |
| `MSG_FIELD_DATA_RESPONSE` (6) | 32 bytes | UT → HQ | Field data with sub-field sensor readings |
| `MSG_HEARTBEAT` (7) | 4 bytes | UT ↔ HQ | Keep-alive signal (hop-to-hop) |
| `MSG_MISSING_BEACON` (8) | 4 bytes | RN → HQ | Report missing relay node |
| `MSG_WARNING` (9) | 4 bytes | SF → RN | Local sub-field warning |
| `MSG_WARNING_FORWARD` (10) | 15 bytes | UT → HQ | Forward sub-field warning |
| `MSG_SUBFIELD_HEARTBEAT` (11) | 4 bytes | SF → UT | Sub-field keep-alive |
| `MSG_SUBFIELD_DATA_REQUEST` (12) | 3 bytes | UT → SF | Request sensor data from sub-field |
| `MSG_SUBFIELD_DATA_REPLY` (13) | 10 bytes | SF → UT | Sub-field sensor response |

## 🔧 Installation & Setup

### 1. Prerequisites
```bash
# Arduino IDE / PlatformIO
- ESP32 board support installed
- LoRa library (LoRa_E220)
- Sensor libraries (DHT, MQ2, GPS if used)
```

### 2. Clone Repository
```bash
git clone https://github.com/Nfx1z/Arduino_emergencyComm_v2.git
cd Arduino_emergencyComm_v2
```

### 3. Configure Hardware Parameters

Edit the appropriate `.ino` file for your device type:

```cpp
// AREA_ID - Emergency region identifier
#define AREA_ID 591

// For Beacon nodes: define beacon order (hop number)
#define BEACON_ORDER 1

// Debug output
#define DEBUG_ENABLED 1
#define DEBUG_BAUD 115200
```

### 4. LoRa Configuration

Both UT and HQ devices must use matching LoRa settings:

```cpp
// LoRa Address & Channel (must match across all nodes)
cfg->ADDH = 0x12;           // High address byte
cfg->ADDL = 0x34;           // Low address byte
cfg->CHAN = 56;             // Channel 56 (frequency dependent on region)

// Air Data Rate
cfg->SPED.airDataRate = AIR_DATA_RATE_010_24;  // 2.4 kbps

// Transmission Power
cfg->OPTION.transmissionPower = POWER_22;      // 22 dBm
```

### 5. Upload to Device

**Using Arduino IDE:**
```
1. Tools → Board → ESP32 Dev Module
2. Tools → Port → Select COM port
3. Sketch → Upload
```

**Using PlatformIO:**
```bash
pio run -t upload
```

### 6. Verify Operation

Open Serial Monitor (115200 baud) and observe:

```
=== FIELD DEVICE (UT) BOOT ===
AREA_ID=591
Initializing LoRa E220...
LoRa init OK.
LoRa config applied.
=== UT setup complete ===
```

## 🔌 Firmware Variants

### `field_device.ino` - End-User Device
- Collects emergency reports and GPS data
- Sends alerts to HQ via LoRa
- Manages sub-field sensor devices
- Primary input from users/dispatchers

### `hq.ino` - Central Coordination Hub
- Receives messages from field devices
- Distributes alerts through relay network
- Tracks device status and connectivity
- Can interface with admin dashboard

### `beacon.ino` - Relay/Repeater Node
- Forwards messages hop-by-hop
- Deduplicates messages (prevents flooding)
- Monitors adjacent node health via heartbeats
- Low-power optimized for extended deployment

### `sub_field.ino` - Sensor Node
- Battery-powered environmental monitoring
- Reports gas, temperature, humidity
- Autonomous alert generation
- Deep sleep for extended operation

## 🔐 Security Considerations

- **Area ID Filtering**: Messages from other regions are dropped
- **Address Filtering**: LoRa ADDH/ADDL acts as network ID
- **Hash-based Deduplication**: Prevents replay attacks
- **Message History**: 60-second window prevents loops
- **No Encryption** (v2.0): Consider adding AES-128 for sensitive deployments

## 🔋 Power Management

### Deep Sleep Mode (Sub-field only)
```cpp
#define POWER_SAVE_MODE true
esp_pm_config_esp32_t pm_config;
pm_config.max_freq_mhz = 80;    // Reduced CPU frequency
pm_pm.min_freq_mhz = 10;
esp_pm_configure(&pm_config);
```

### Typical Power Consumption
- **LoRa Transmit**: ~400 mA (peak)
- **LoRa Receive**: ~80 mA
- **Idle/Sleep**: ~2 mA
- **Deep Sleep**: < 0.05 mA

## 🔄 Communication Flow

### Example: Emergency Help Request
```
1. User initiates emergency via sigapUser mobile app
2. Mobile app sends BLE message to Field Device (UT)
3. Field Device constructs MSG_HELP with GPS coordinates
4. Message hops through relay chain:
   UT → Beacon1 → Beacon2 → HQ
5. HQ receives and acknowledges with MSG_ACK
6. ACK relayed back: HQ → Beacon2 → Beacon1 → UT
7. Mobile app receives confirmation
8. HQ dashboard shows incident on Admin interface
```

## 📊 Heartbeat & Monitoring

### Beacon Heartbeat Cycle
- **Interval**: 30 minutes (configurable)
- **Tolerance**: 5 minutes grace period
- **Check Frequency**: Every 1 minute
- **Timeout Detection**: Reports missing beacon after 35 minutes

```cpp
#define HEARTBEAT_INTERVAL       1800000  // 30 min
#define HEARTBEAT_TOLERANCE       300000  // 5 min
#define HEARTBEAT_CHECK_INTERVAL   60000  // 1 min
```

## 🐛 Debugging

### Enable Debug Output
```cpp
#define DEBUG_ENABLED 1
#define DEBUG_BAUD 115200
```

### Common Messages
```
"=== FIELD DEVICE (UT) BOOT ===" — Normal startup
"Packet: type=HELP (19 bytes)" — Received message
"  -> relayed toward MT" — Message forwarded upstream
"!!! Previous beacon MISSING to MT" — Link lost
"RX parser timeout - resetting parser" — Packet corruption recovery
```

## 🚀 Improvements in v2.0

- ✅ **Robust Parser Watchdog**: Timeout detection for stalled packets
- ✅ **Heartbeat Validation**: Proper sender/receiver verification
- ✅ **Function Prototypes**: C++ compliant compilation
- ✅ **Message History**: Enhanced 60-second deduplication window
- ✅ **Sub-field Support**: Full sensor data integration
- ✅ **Error Recovery**: Graceful handling of RF interference

## 📚 Related Repositories

- **[Android_sigapUser](https://github.com/Nfx1z/Android_sigapUser)** - Mobile app for end-users
- **[Android_sigapAdmin](https://github.com/Nfx1z/Android_sigapAdmin)** - Admin dashboard
- **[_DisasterCommunication_LoRa](https://github.com/Nfx1z/_DisasterCommunication_LoRa)** - Previous generation (v1.0)

## 🤝 Contributing

Contributions welcome! Areas for enhancement:
- Encryption support (AES-128)
- Mesh networking optimization
- Battery optimization
- Additional sensor types
- Web dashboard integration

## 📄 License

Part of the SIGAP Emergency Communication System

## ⚠️ Important Notes

- **LoRa Range**: ~500m urban, ~2km line-of-sight (depends on antenna/power)
- **Packet Size**: Max 240 bytes (LoRa limitation)
- **Network Size**: Up to 254 nodes per area (ADDH/ADDL + CHAN)
- **Deployment**: Requires proper antenna placement for optimal coverage
- **Regulations**: Comply with local RF regulations for 900MHz/433MHz operation

## 🆘 Troubleshooting

| Issue | Solution |
|-------|----------|
| "LoRa init FAILED" | Check E220 wiring and power supply |
| "areaId mismatch" | Verify all devices have same AREA_ID |
| "Unrecognized type" | Check message structure size definition |
| No messages received | Verify ADDH, ADDL, CHAN match across nodes |
| High message loss | Reduce transmission power or move relay node |
| Battery drain | Disable DEBUG_ENABLED and use deep sleep mode |

## 📞 Support

For issues and questions:
1. Check serial debug output (115200 baud)
2. Review message flow in README
3. Verify hardware connections
4. Test with debug enabled
5. Open GitHub issue with full debug logs

---

**Part of the SIGAP Emergency Communication System** 🚨

*Last Updated: 2026-09-15*
