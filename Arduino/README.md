# Arduino LoRa Gas & Temperature Sensor Examples
==================================================

This folder contains Arduino sketches for LoRa-based gas detection and temperature monitoring using both **MCCI LMIC library** and **SX12XX library**. The examples are specifically designed for **Arduino Uno**, **Arduino Mega**, and **Arduino Micro** with **Dragino LoRa Shield** and demonstrate how to build low-cost IoT sensor nodes for environmental monitoring.

## Hardware Requirements

### Supported Arduino Boards
- **Arduino Uno** (ATmega328P, 16MHz/5V with 3.3V pin)
- **Arduino Mega 2560** (ATmega2560, 16MHz/5V with 3.3V pin) 
- **Arduino Micro** (ATmega32u4, 16MHz/5V with 3.3V pin)

### LoRa Shield Connection
**Dragino LoRa Shield v1.4** pin mapping for all supported boards:

```
    Arduino Pin      Dragino Shield      Function
       GND    ----------- GND          (Ground)
       3V3    ----------- 3.3V         (Power 3.3V)
       D10    ----------- NSS          (SPI Chip Select)
       D11    ----------- MOSI         (SPI Data In)
       D12    ----------- MISO         (SPI Data Out)  
       D13    ----------- SCK          (SPI Clock)
       D9     ----------- RST          (Reset)
       D2     ----------- DIO0         (Digital I/O 0)
       D6     ----------- DIO1         (Digital I/O 1)
       D7     ----------- DIO2         (Digital I/O 2)
```

**Note:** On Arduino Mega, SPI pins are: D50 (MISO), D51 (MOSI), D52 (SCK), but NSS remains D10.

### Sensor Connections

#### MQ2 Gas Sensor
```
MQ2 Pin         Arduino Pin      Function
  VCC    ----------- 5V          (Power 5V)
  GND    ----------- GND         (Ground)
  AO     ----------- A0          (Analog Output)
  DO     ----------- Not Used    (Digital Output - Optional)
```

#### DS18B20 Temperature Sensor
```
DS18B20 Pin     Arduino Pin      Function
  VCC    ----------- 5V          (Power 5V)
  GND    ----------- GND         (Ground)
  DATA   ----------- D4          (Data with 4.7kΩ pullup to VCC)
```

**Important:** DS18B20 requires a 4.7kΩ pullup resistor between DATA and VCC pins.

#### DHT22 Temperature & Humidity Sensor
```
DHT22 Pin       Arduino Pin      Function
  VCC    ----------- 5V          (Power 5V)
  GND    ----------- GND         (Ground)
  DATA   ----------- D3          (Data pin)
```

## Library Dependencies

### Required Libraries (Install via Arduino Library Manager)

#### For MCCI LMIC Examples
1. **MCCI LoRaWAN LMIC library** - Main LoRaWAN communication library
2. **OneWire** - For DS18B20 communication protocol
3. **DallasTemperature** - High-level DS18B20 temperature sensor library
4. **DHT sensor library** - For DHT22 humidity sensor (if used)

#### For SX12XX Examples
1. **SX12XX-LoRa library** by Stuart Robinson - Professional LoRa library
2. **OneWire** - For DS18B20 communication protocol
3. **DallasTemperature** - High-level DS18B20 temperature sensor library
4. **DHT sensor library** - For DHT22 humidity sensor (if used)
5. **LowPower** - For power management (Arduino AVR boards)

### Installation Steps
1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search and install required libraries based on your chosen examples

## Project Structure

### MCCI LMIC Examples (LoRaWAN/TTN)
```
MCCI_ttn_DS18B20/           - DS18B20 temperature sensor
├── MCCI_ttn_DS18B20.ino    - Main sketch
└── payload.txt             - TTN decoder function

MCCI_ttn_mq2/               - MQ2 gas sensor  
├── MCCI_ttn_mq2.ino        - Main sketch
└── payload.txt             - TTN decoder function

MCCI_ttn_Enhanced_MQ2/      - Enhanced MQ2 with calculations
├── MCCI_ttn_Enhanced_MQ2.ino - Main sketch
└── payload.txt             - TTN decoder function

MCCI_ttn_Enhanced_Raw_MQ2/  - Memory-optimized raw MQ2
├── MCCI_ttn_Enhanced_Raw_MQ2.ino - Main sketch
└── payload.txt             - TTN decoder function

MCCI_ttn_DS18B20_MQ2/       - Combined temperature + gas
├── MCCI_ttn_DS18B20_MQ2.ino - Main sketch
└── payload.txt             - TTN decoder function

MCCI_ttn_Enhanced_Raw_MQ2_DS18B20/ - Combined raw sensors
├── MCCI_ttn_Enhanced_Raw_MQ2_DS18B20.ino - Main sketch
└── payload.txt             - TTN decoder function

MCCI_ttn_abp/               - Basic LoRaWAN ABP example
└── MCCI_ttn_abp.ino        - Standard "Hello World"

MCCI_ttn_abp_custom/        - Interactive message sender
├── MCCI_ttn_abp_custom.ino - Main sketch
└── payload.txt             - TTN decoder function
```

### SX12XX Examples (Professional LoRa)
```
Lora_DS18B20_SX12XXX/       - DS18B20 with professional features
├── Lora_DS18B20_SX12XXX.ino - Main sketch
└── Lora_DS18B20_SX12XXX.h  - Configuration header

Lora_Enhanced_MQ2_SX12XXX/  - Professional MQ2 gas monitoring
├── Lora_Enhanced_MQ2_SX12XXX.ino - Main sketch
└── Lora_Enhanced_MQ2_SX12XXX.h   - Configuration header

Lora_Enhanced_MQ2_DS18B20_SX12XXX/ - Complete environmental station
├── Lora_Enhanced_MQ2_DS18B20_SX12XXX.ino - Main sketch
└── Lora_Enhanced_MQ2_DS18B20_SX12XXX.h   - Configuration header

Lora_DH22_SX12XXX/          - DHT22 temperature & humidity
├── Lora_DH22_SX12XXX.ino   - Main sketch
└── Lora_DH22_SX12XXX.h     - Configuration header

Lora_Custom_Payload_SX12XXX/ - Interactive messaging
├── Lora_Custom_Payload_SX12XXX.ino - Main sketch
└── Lora_Custom_Payload_SX12XXX.h   - Configuration header

Lora_Ping_Pong_SX12XXX/     - Gateway communication test
├── Lora_Ping_Pong_SX12XXX.ino - Main sketch
└── Lora_Ping_Pong_SX12XXX.h   - Configuration header
```

## Configuration Setup

### Regional Frequency Configuration (MCCI LMIC)

You have **two options** to configure the regional frequency:

#### **Option 1: Direct config.h Edit (Recommended)**
Navigate to: **`Arduino/libraries/MCCI_LoRaWAN_LMIC_library/src/lmic/config.h`**

Find this section around line 12-20 and modify:

```cpp
// make sure that we have exactly one target region defined.
#if CFG_LMIC_REGION_MASK == 0
// #define CFG_eu868 1    // ← Comment out default
#define CFG_us915 1       // ← Uncomment your region
#elif (CFG_LMIC_REGION_MASK & (-CFG_LMIC_REGION_MASK)) != CFG_LMIC_REGION_MASK
```

**Available regions:**
- `#define CFG_eu868 1`     // Europe 868MHz (default)
- `#define CFG_us915 1`     // North America 915MHz
- `#define CFG_au915 1`     // Australia 915MHz  
- `#define CFG_as923 1`     // Asia-Pacific 923MHz
- `#define CFG_kr920 1`     // South Korea 920MHz
- `#define CFG_in866 1`     // India 866MHz

#### **Option 2: Project Config File**
Create `lmic_project_config.h` in: **`{Arduino_Libraries}/MCCI_LoRaWAN_LMIC_library/project_config/lmic_project_config.h`**

```cpp
// Regional frequency selection - uncomment ONE only
#define CFG_us915 1     // North America 915MHz
// #define CFG_eu868 1  // Europe 868MHz  
// #define CFG_as923 1  // Asia-Pacific 923MHz

// Radio module selection - uncomment ONE only  
#define CFG_sx1276_radio 1  // For Dragino Shield (RFM95W)
// #define CFG_sx1272_radio 1

// Optional: Disable features to save memory
// #define DISABLE_JOIN
// #define DISABLE_PING
```

### TTN (The Things Network) Configuration
All MCCI LMIC examples use **ABP (Activation By Personalization)**. You need to:

1. **Create TTN Application** at https://console.thethingsnetwork.org
2. **Add Device** with ABP activation
3. **Copy credentials** from TTN console:
   - **DevAddr** (Device Address)
   - **NwkSKey** (Network Session Key) 
   - **AppSKey** (Application Session Key)
4. **Update code** with your credentials:

```cpp
// Replace with your TTN device credentials
static const u4_t DEVADDR = 0x260BE0A2; // ← Your DevAddr
static const PROGMEM u1_t NWKSKEY[16] = { /* Your NwkSKey in MSB format */ };
static const u1_t PROGMEM APPSKEY[16] = { /* Your AppSKey in MSB format */ };
```

### SX12XX Library Configuration
For SX12XX examples, configure frequency and chip type in the header files:

```cpp
// In Lora_*.h files, uncomment your chip family
//#define SX126X          // For newer modules (SX1262, etc.)
#define SX127X           // For Dragino Shield (SX1276/RFM95W) 
//#define SX128X          // For 2.4GHz modules

// Set frequency for your region
const uint32_t DEFAULT_CHANNEL = 865200000;  // EU 865.2MHz
// const uint32_t DEFAULT_CHANNEL = 915000000;  // US 915MHz
```

## Example Sketches

### MCCI LMIC Examples (LoRaWAN/TTN)

#### 1. **`MCCI_ttn_DS18B20.ino`** - Temperature Sensor
**Features:**
- ✅ DS18B20 digital temperature sensor
- ✅ Standard LoRaWAN format
- ✅ 2-byte payload (temperature × 100)
- ✅ Automatic TTN integration

**Payload Structure (2 bytes):**
```
Byte 0-1: Temperature × 100 (signed 16-bit, supports negative temps)
```

#### 2. **`MCCI_ttn_mq2.ino`** - Basic Gas Sensor
**Features:**
- ✅ MQ2 gas sensor raw ADC values
- ✅ Simple gas detection
- ✅ 2-byte payload
- ✅ Basic threshold alerts

**Payload Structure (2 bytes):**
```
Byte 0-1: Raw ADC value (0-1023)
```

#### 3. **`MCCI_ttn_Enhanced_MQ2.ino`** - Advanced Gas Sensor
**Features:**
- ✅ Multiple gas concentration calculations
- ✅ 7 different gas types detected
- ✅ 10-byte detailed payload
- ✅ Safety threshold monitoring

**Gas Detection Capability:**
- LPG (Liquefied Petroleum Gas)
- Methane (CH4)
- Carbon Monoxide (CO)
- Hydrogen (H2)
- Alcohol/Ethanol
- Propane
- Smoke particles

**Payload Structure (10 bytes):**
```
Byte 0-1: Raw ADC value
Byte 2-3: LPG concentration (ppm)
Byte 4-5: Methane concentration (ppm)
Byte 6-7: CO concentration (ppm)
Byte 8-9: Rs/R0 ratio × 1000
```

#### 4. **`MCCI_ttn_Enhanced_Raw_MQ2.ino`** - Memory Optimized Gas Sensor
**Features:**
- ✅ Raw sensor data for TTN-side calculations
- ✅ Memory optimized for Arduino Uno (~30% usage)
- ✅ 6-byte compact payload
- ✅ All gas calculations performed on TTN

**Payload Structure (6 bytes):**
```
Byte 0-1: Raw ADC value (0-1023)
Byte 2-3: Sensor resistance Rs÷10 (ohms)
Byte 4-5: Rs/R0 ratio × 1000
```

#### 5. **`MCCI_ttn_DS18B20_MQ2.ino`** - Combined Sensors
**Features:**
- ✅ DS18B20 + MQ2 in single device
- ✅ Environmental monitoring
- ✅ 4-byte efficient payload
- ✅ Combined alerts

**Payload Structure (4 bytes):**
```
Byte 0-1: Temperature × 100 (signed)
Byte 2-3: MQ2 gas sensor ADC value
```

#### 6. **`MCCI_ttn_Enhanced_Raw_MQ2_DS18B20.ino`** - Combined Raw Sensors
**Features:**
- ✅ Both sensors with raw data approach
- ✅ Memory optimized for Arduino Uno (~30% usage)
- ✅ 8-byte payload
- ✅ Complete environmental monitoring
- ✅ TTN-side calculations

**Payload Structure (8 bytes):**
```
Byte 0-1: MQ2 ADC value
Byte 2-3: MQ2 resistance Rs÷10
Byte 4-5: Rs/R0 ratio × 1000  
Byte 6-7: Temperature × 100 (signed)
```

#### 7. **`MCCI_ttn_abp.ino`** - Basic LoRaWAN Example
**Features:**
- ✅ Standard "Hello World" LoRaWAN transmission
- ✅ Testing TTN connectivity
- ✅ Template for custom applications

#### 8. **`MCCI_ttn_abp_custom.ino`** - Interactive Message Sender
**Features:**
- ✅ Send custom messages via Serial Monitor
- ✅ Interactive testing and debugging
- ✅ Variable payload length (1-64 bytes)
- ✅ Real-time transmission feedback

### SX12XX Examples (Professional LoRa)

#### 1. **`Lora_DS18B20_SX12XXX.ino`** - Professional Temperature Sensor
**Features:**
- ✅ **Advanced power management** with hibernation modes
- ✅ **EEPROM persistence** for packet sequence numbers
- ✅ **ACK support** with SNR reporting
- ✅ **Carrier sense** before transmission (Listen-Before-Talk)
- ✅ **Multi-platform support** (AVR, SAMD, Teensy, ESP8266/32)
- ✅ **Gateway-compatible format** (Congduc Pham style)

#### 2. **`Lora_Enhanced_MQ2_SX12XXX.ino`** - Professional Gas Monitoring
**Features:**
- ✅ **Complete gas analysis** with 7 gas types
- ✅ **Low power optimization** for battery operation
- ✅ **Professional gateway integration**
- ✅ **Real-time safety alerts**
- ✅ **Configurable transmission intervals**

#### 3. **`Lora_Enhanced_MQ2_DS18B20_SX12XXX.ino`** - Complete Environmental Station
**Features:**
- ✅ **Combined sensors** (DS18B20 + MQ2) with error handling
- ✅ **Environmental safety monitoring** with comprehensive alerts
- ✅ **Cross-sensor correlation** and validation
- ✅ **Graceful degradation** (continues if one sensor fails)
- ✅ **Professional data format** for gateway compatibility

**Enhanced Payload Format:**
```
\!TC/23.45/LPG/250.1/CH4/180.3/CO/45.2/H2/120.5/ALC/85.7/ADC/345
```

#### 4. **`Lora_DH22_SX12XXX.ino`** - Humidity and Temperature
**Features:**
- ✅ **DHT22 sensor** for temperature and humidity
- ✅ **Heat index calculation**
- ✅ **Averaged readings** for accuracy
- ✅ **Low power support** with external sensor control

#### 5. **`Lora_Custom_Payload_SX12XXX.ino`** - Interactive Messaging
**Features:**
- ✅ **Real-time message input** via Serial Monitor
- ✅ **Character-by-character echo** and backspace support
- ✅ **Message validation** and feedback
- ✅ **Professional transmission** with detailed status

#### 6. **`Lora_Ping_Pong_SX12XXX.ino`** - Gateway Communication Test
**Features:**
- ✅ **Gateway connectivity testing**
- ✅ **ACK-based ping-pong** communication
- ✅ **Signal quality reporting** (SNR, RSSI)
- ✅ **Automatic retry** mechanisms

## SX12XX Library Features

The **SX12XX library** by Stuart Robinson offers significant advantages for professional sensor applications:

### **Advantages over MCCI LMIC:**
- ✅ **Multi-chip support** (SX126X, SX127X, SX128X) 
- ✅ **Direct LoRa control** without LoRaWAN protocol overhead
- ✅ **Better power management** with precise sleep/wake control
- ✅ **Lower memory usage** - perfect for Arduino Uno
- ✅ **Professional radio management** with carrier sense
- ✅ **Modular design** - easy to add/remove features
- ✅ **Gateway compatibility** with packet formatting
- ✅ **Hardware abstraction** for multiple platforms

### **Supported LoRa Chips:**
| Chip Family | Models | Frequency | Key Features |
|-------------|--------|-----------|--------------|
| **SX127X** | SX1272, SX1276, SX1277, SX1278, SX1279 | 433/868/915 MHz | Most common, Dragino compatible |
| **SX126X** | SX1261, SX1262, SX1268 | 433/868/915 MHz | Latest generation, better efficiency |
| **SX128X** | SX1280, SX1281 | 2.4 GHz | High-speed, ranging capability |

### **When to Use SX12XX vs MCCI LMIC:**

#### **Use SX12XX for:**
- 🔬 **Professional sensor networks** with custom protocols
- 🏠 **Point-to-point** communication systems
- 🔋 **Battery-powered** remote monitoring
- 🎯 **Custom gateway** integration (Congduc Pham style)
- ⚡ **Memory-constrained** Arduino Uno projects
- 🧪 **Research and development** projects

#### **Use MCCI LMIC for:**
- 🌐 **Standard LoRaWAN** networks (TTN, ChirpStack)
- 🏢 **Commercial IoT** deployments  
- 🔐 **Secure communications** with encryption
- 📡 **Multi-operator** compatibility
- 🌍 **Global deployment** with regional compliance

## Advanced Features

### Gateway-Compatible Data Format (SX12XX)
Professional environmental stations use **Congduc Pham's gateway format**:

```cpp
// Standard format: \!SENSOR/value/SENSOR/value...
sprintf(message, "\\!TC/%s/LPG/%s/CH4/%s/CO/%s/H2/%s/ALC/%s/ADC/%d", 
        temp_str, lpg_str, ch4_str, co_str, h2_str, alcohol_str, sensorValue);

// Example output: \!TC/23.45/LPG/250.1/CH4/180.3/CO/45.2/H2/120.5/ALC/85.7/ADC/345
```

**Nomenclature Reference:**
- **TC**: Temperature Celsius  
- **HU**: Humidity (%)
- **HI**: Heat Index
- **LPG**: Liquefied Petroleum Gas (ppm)
- **CH4**: Methane (ppm)
- **CO**: Carbon Monoxide (ppm)  
- **H2**: Hydrogen (ppm)
- **ALC**: Alcohol (ppm)
- **ADC**: Raw ADC value for diagnostics

### EEPROM Persistence (SX12XX)
Maintains packet sequence numbers across power cycles:

```cpp
#ifdef WITH_EEPROM
struct sx1272config {
  uint8_t flag1;     // Validation flag 1 (0x12)
  uint8_t flag2;     // Validation flag 2 (0x34)  
  uint8_t seq;       // Last packet sequence number
};

// Automatic save after each transmission
my_sx1272config.seq = LT.readTXSeqNo();    
EEPROM.put(0, my_sx1272config);
#endif
```

### Environmental Safety Monitoring
Comprehensive safety threshold system with alerts:

```cpp
#ifdef ENVIRONMENTAL_ALERTS
String getEnvironmentalAlert(float tempC, float lpg, float methane, float co, float alcohol) {
  String alerts = "";
  
  // Temperature thresholds
  if (tempC > 50) alerts += "TEMP-HOT ";
  else if (tempC < 0) alerts += "TEMP-FREEZE ";
  else if (tempC > 35) alerts += "TEMP-HIGH ";
  
  // Gas safety thresholds (ppm)
  if (co > 200) alerts += "CO-DANGER ";
  else if (co > 50) alerts += "CO-HIGH ";
  
  if (lpg > 5000) alerts += "LPG-DANGER ";
  else if (lpg > 1000) alerts += "LPG-HIGH ";
  
  return alerts.length() > 0 ? alerts : "NORMAL";
}
#endif
```

### Multi-Platform Low Power Support (SX12XX)
Optimized sleep modes for different microcontrollers:

#### **Arduino AVR (Uno/Nano/Pro Mini):**
```cpp
#include "LowPower.h"
// 8-second sleep cycles (hardware limitation)
LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
```

#### **Teensy (LC/31/32/35/36):**
```cpp
#include <Snooze.h>
SnoozeTimer timer;
timer.setTimer(LOW_POWER_PERIOD*1000);  // Up to 65535ms

#ifdef LOW_POWER_HIBERNATE
  Snooze.hibernate(sleep_config);  // Deepest sleep
#else            
  Snooze.deepSleep(sleep_config);  // Normal deep sleep
#endif
```

#### **SAMD21 (Zero/M0):**
```cpp
#include "RTCZero.h"
RTCZero rtc;

// Precise RTC-based wake-up
rtc.setAlarmTime(17, idlePeriodInMin, 0);
rtc.enableAlarm(rtc.MATCH_HHMMSS);
rtc.standbyMode();
```

#### **ESP8266/ESP32:**
```cpp
// Microsecond precision deep sleep
ESP.deepSleep(LOW_POWER_PERIOD*1000*1000);  // Auto-restart after wake
```

## LoRa Mode Configuration

The examples use **LoRa Mode 1** by default for maximum range and sensitivity:

### **Pre-defined LoRa Modes:**

| Mode | Bandwidth | Spreading Factor | Code Rate | Range | Speed | Power |
|------|-----------|------------------|-----------|-------|--------|-------|
| **1** | 125 kHz | SF12 | CR4/5 | ⭐⭐⭐ Max | 🐌 Slow | 💚 Low |
| **2** | 250 kHz | SF12 | CR4/5 | ⭐⭐⭐ High | 🐌 Slow | 💚 Low |
| **3** | 125 kHz | SF10 | CR4/5 | ⭐⭐ Good | 🚶 Medium | 💛 Medium |
| **4** | 500 kHz | SF12 | CR4/5 | ⭐⭐ Good | 🚶 Medium | 💛 Medium |
| **5** | 250 kHz | SF10 | CR4/5 | ⭐⭐ Good | 🚶 Medium | 💛 Medium |
| **6** | 500 kHz | SF11 | CR4/5 | ⭐⭐ Good | 🏃 Fast | 💛 Medium |
| **7** | 250 kHz | SF9 | CR4/5 | ⭐ Medium | 🏃 Fast | 🧡 High |
| **8** | 500 kHz | SF9 | CR4/5 | ⭐ Medium | 🏃 Fast | 🧡 High |
| **9** | 500 kHz | SF8 | CR4/5 | ⭐ Short | 🏃‍♂️ Fast | 🧡 High |
| **10** | 500 kHz | SF7 | CR4/5 | ⭐ Short | 🏃‍♂️ Very Fast | ❤️ High |

### **Recommended Modes by Application:**

#### **Gas/Temperature Sensors:**
- 🏠 **Indoor monitoring:** Mode 1 (SF12BW125) - Maximum sensitivity
- 🏭 **Industrial areas:** Mode 1 (SF12BW125) - Penetrates obstacles  
- 🌾 **Outdoor long-range:** Mode 1 (SF12BW125) - Up to 15km rural
- 🏘️ **Urban environments:** Mode 3 (SF10BW125) - Good range, faster

#### **Interactive Applications:**
- 🧪 **Development/testing:** Mode 6 (SF11BW500) - Balance of range/speed
- 💬 **Interactive sender:** Mode 8 (SF9BW500) - Fast response

## AES Encryption

### **LoRaWAN Encryption (MCCI LMIC):**
```cpp
// Encryption is automatic in LoRaWAN - handled by LMIC library
// using your NwkSKey and AppSKey from TTN console

// Security features included:
// ✅ AES-128 payload encryption
// ✅ Message integrity (MIC)  
// ✅ Replay protection
// ✅ Network-level security
```

### **Security Levels:**

#### **LoRaWAN (Automatic):**
- 🔐 **Network encryption** (NwkSKey) - Protects routing info
- 🔐 **Application encryption** (AppSKey) - Protects sensor data
- 🔐 **Message authentication** (MIC) - Prevents tampering
- 🔐 **Frame counter** - Prevents replay attacks

#### **When to Use Encryption:**
- 🏢 **Commercial deployments** - Always recommended
- 🏭 **Industrial monitoring** - Critical for security
- 🏠 **Home automation** - Protects privacy
- 🧪 **Development/testing** - Optional, can disable for debugging

## Downlink Communication

Both MCCI LMIC and SX12XX libraries support downlink communication, but use different approaches and protocols.

### **MCCI LMIC Downlink (LoRaWAN Standard)**

MCCI LMIC provides standard LoRaWAN downlink capability through RX windows, allowing TTN or network servers to send commands back to sensor nodes.

#### **Current Implementation Status:**
Most MCCI sketches include **basic downlink detection** but require additional code for complete command processing:

**Basic Detection (Included):**
```cpp
case EV_TXCOMPLETE:
  if (LMIC.dataLen) {
    Serial.print(F("Received "));
    Serial.print(LMIC.dataLen);
    Serial.println(F(" bytes of payload"));
    Serial.print(F("Downlink data: "));
    for (int i = 0; i < LMIC.dataLen; i++) {
      if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
        Serial.print(F("0"));
      }
      Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  break;
```

**Complete Processing (Additional Code Needed):**
```cpp
void processDownlinkCommand() {
  if (LMIC.dataLen == 0) return;
  
  switch (LMIC.frame[LMIC.dataBeg]) {
    case 0x01: // Change transmission interval
      if (LMIC.dataLen >= 2) {
        TX_INTERVAL = LMIC.frame[LMIC.dataBeg + 1] * 60;
        Serial.print(F("TX interval changed to: "));
        Serial.println(TX_INTERVAL);
      }
      break;
      
    case 0x02: // LED control
      if (LMIC.dataLen >= 2) {
        digitalWrite(LED_PIN, LMIC.frame[LMIC.dataBeg + 1] ? HIGH : LOW);
        Serial.print(F("LED set to: "));
        Serial.println(LMIC.frame[LMIC.dataBeg + 1] ? "ON" : "OFF");
      }
      break;
      
    case 0x03: // Request immediate sensor reading
      do_send(&sendjob);
      Serial.println(F("Immediate reading requested"));
      break;
      
    default:
      Serial.print(F("Unknown command: 0x"));
      Serial.println(LMIC.frame[LMIC.dataBeg], HEX);
  }
}
```

#### **LoRaWAN RX Windows:**
- **RX1 Window:** Opens 1 second after transmission (same frequency/SF)
- **RX2 Window:** Opens 2 seconds after transmission (869.525MHz, SF9 for EU)
- **Automatic timing** handled by LMIC library

#### **MCCI LMIC Downlink Features:**
- ✅ **Standard LoRaWAN protocol** compliance
- ✅ **Automatic RX window management**
- ✅ **Network-level encryption** (AES-128)
- ✅ **TTN console integration** for easy command sending
- ✅ **Variable payload length** (up to 51 bytes depending on SF)
- ❌ **Requires additional code** for command processing in most examples

### **SX12XX Downlink (ACK and Ping-Pong)**

The SX12XX library provides robust bidirectional communication through ACK mechanisms and ping-pong protocols, specifically designed for custom gateway communication.

#### **ACK-Based Downlink:**
```cpp
#ifdef WITH_ACK
  p_type = PKT_TYPE_DATA | PKT_FLAG_ACK_REQ;
  PRINTLN_CSTSTR("Will request an ACK");
  
  if (LT.transmitAddressed(message, r_size, p_type, DEFAULT_DEST_ADDR, LT.readDevAddr(), 10000, MAX_DBM, WAIT_TX)) {
    if (LT.readAckStatus()) {
      PRINT_CSTSTR("Received ACK from gateway ");
      PRINT_VALUE("%d", LT.readRXSource());
      PRINT_CSTSTR("SNR of transmitted pkt is ");
      PRINT_VALUE("%d", LT.readPacketSNRinACK());
      
      // Process ACK data for downlink commands
      processACKData();
    }
  }
#endif
```

#### **Ping-Pong Communication:**
```cpp
// From Lora_Ping_Pong_SX12XXX.ino
if (LT.transmitAddressed(message, r_size, p_type, DEFAULT_DEST_ADDR, LT.readDevAddr(), 10000, MAX_DBM, WAIT_TX)) {
  if (LT.readAckStatus()) {
    PRINTLN_CSTSTR("Pong received from gateway!");
    sprintf((char*)message,"SNR at gw=%d   ", LT.readPacketSNRinACK());
    sprintf((char*)message,"gw->SNR=%d:%d", LT.readPacketSNR(), LT.readPacketRSSI());
    
    // Full bidirectional communication established
    processPongResponse();
  }
}
```

#### **Signal Quality Reporting:**
```cpp
// SX12XX provides detailed link quality information
int gateway_snr = LT.readPacketSNRinACK();    // SNR of our packet at gateway
int local_snr = LT.readPacketSNR();           // SNR of gateway response
int local_rssi = LT.readPacketRSSI();         // RSSI of gateway response
uint8_t gateway_addr = LT.readRXSource();     // Gateway address confirmation
```

#### **SX12XX Downlink Features:**
- ✅ **Native ACK support** with data payload
- ✅ **Real-time signal quality** (SNR/RSSI) reporting
- ✅ **Ping-pong communication** for testing and monitoring
- ✅ **Gateway address validation**
- ✅ **Custom protocol flexibility**
- ✅ **Low-latency responses** (no RX window delays)
- ✅ **Immediate feedback** on transmission success
- ❌ **No standard LoRaWAN compatibility**

### **Downlink Comparison**

| Feature | MCCI LMIC | SX12XX Library |
|---------|-----------|----------------|
| **Protocol** | Standard LoRaWAN | Custom LoRa |
| **RX Windows** | RX1/RX2 automatic | ACK-based immediate |
| **Command Processing** | Basic display only* | ACK with signal data |
| **Signal Quality** | Limited | Full SNR/RSSI reporting |
| **Gateway Integration** | TTN/ChirpStack | Congduc Pham gateway |
| **Response Latency** | 1-2 seconds | Immediate |
| **Encryption** | AES-128 automatic | Optional custom |
| **Payload Length** | Up to 51 bytes | Limited by ACK size |
| **Implementation** | Requires additional code | Built-in examples |

*Most current MCCI examples only display received data and require additional code for command processing.

### **Downlink Command Examples**

#### **Device Control Commands:**
- `LED_ON` / `LED_OFF` - Control status LED
- `RESET` - Restart device
- `SLEEP_3600` - Sleep for 1 hour
- `INTERVAL_300` - Change to 5-minute intervals

#### **Sensor Configuration:**
- `CALIBRATE_MQ2` - Recalibrate MQ2 sensor
- `THRESHOLD_CO_100` - Set CO alert threshold to 100ppm
- `SAMPLE_RATE_10` - Take 10 samples for averaging

#### **Network Management:**
- `SET_SF_10` - Change spreading factor to 10
- `SET_POWER_10` - Change TX power to 10dBm
- `SET_FREQ_868300` - Change frequency to 868.3MHz

### **When to Use Each Approach**

#### **Use MCCI LMIC Downlink for:**
- 🌐 **Standard LoRaWAN** deployments with TTN/ChirpStack
- 🏢 **Commercial applications** requiring protocol compliance
- 🔐 **Secure communications** with automatic encryption
- 📱 **TTN console integration** for easy command sending
- 🌍 **Multi-operator compatibility**

#### **Use SX12XX Downlink for:**
- 🏠 **Custom gateway** deployments (Congduc Pham style)
- 📊 **Real-time monitoring** with signal quality feedback
- 🔧 **Development and testing** with immediate feedback
- ⚡ **Low-latency** command/response applications
- 🧪 **Research projects** with custom protocols

### **Downlink Best Practices**

#### **For Both Libraries:**
- ⏰ **Keep commands short** to save airtime
- 🔄 **Implement command acknowledgment** for reliability
- 🛡️ **Validate commands** before execution
- 📝 **Log downlink activity** for debugging
- 🔋 **Balance responsiveness vs battery life**

#### **MCCI LMIC Specific:**
- 📋 **Add command processing** to existing basic detection
- ⏱️ **Account for RX window timing** in power management
- 🔐 **Leverage automatic encryption** for sensitive commands
- 📡 **Use TTN console** for convenient command scheduling

#### **SX12XX Specific:**
- 📊 **Utilize signal quality data** for network optimization
- 🎯 **Implement custom protocols** for specific applications
- ⚡ **Take advantage of immediate responses** for real-time control
- 🔧 **Use ping-pong mode** for connectivity testing

### **Implementation Notes**

- **MCCI LMIC:** Most examples provide foundation for downlink but require additional `processDownlinkCommand()` function
- **SX12XX:** Examples include working ACK and ping-pong communication out of the box
- **Both approaches** can be extended with custom command parsing and device control logic
- **Power consumption** considerations differ between immediate ACK responses and scheduled RX windows

## Gas Concentration Formulas

All gas concentrations are calculated using these formulas (TTN decoder or device-side):

| Gas | Formula | Detection Range |
|-----|---------|-----------------|
| **LPG** | `987.26 × (Rs/R0)^(-2.162)` ppm | 200-10,000 ppm |
| **Methane** | `2217.8 × (Rs/R0)^(-2.827)` ppm | 300-10,000 ppm |
| **CO** | `605.18 × (Rs/R0)^(-3.937)` ppm | 300-10,000 ppm |
| **Hydrogen** | `988.05 × (Rs/R0)^(-1.767)` ppm | 300-5,000 ppm |
| **Alcohol** | `75.906 × (Rs/R0)^(-1.691)` ppm | 100-2,000 ppm |
| **Propane** | `614.56 × (Rs/R0)^(-2.564)` ppm | 200-5,000 ppm |
| **Smoke** | `143.01 × (Rs/R0)^(-2.186)` ppm | Variable |

**Where:**
- **Rs** = Sensor resistance in gas
- **R0** = Sensor resistance in clean air (baseline)

## Safety Thresholds

### Gas Safety Levels
| Gas | Normal | Caution | Warning | Danger |
|-----|--------|---------|---------|--------|
| **CO** | <50 ppm | 50-200 ppm | 200-1000 ppm | >1000 ppm ⚠️ |
| **LPG/Methane** | <1000 ppm | 1000-5000 ppm | 5000-10000 ppm | >10000 ppm ⚠️ |
| **Hydrogen** | <1000 ppm | 1000-10000 ppm | 10000-40000 ppm | >40000 ppm ⚠️ |

### Temperature Safety Levels
| Condition | Range | Action |
|-----------|-------|--------|
| **Normal** | 0-35°C | Continue monitoring |
| **Caution** | 35-50°C or -10-0°C | Monitor closely |
| **Warning** | >50°C or <-10°C | Investigate cause |

### Alert Levels (SX12XX Examples)
| Parameter | Normal | High | Danger | Action |
|-----------|--------|------|--------|--------|
| **Temperature** | 0-35°C | 35-50°C | >50°C or <0°C | Monitor/Investigate |
| **CO** | <50 ppm | 50-200 ppm | >200 ppm | Ventilate/Evacuate |
| **LPG/Methane** | <1000 ppm | 1000-5000 ppm | >5000 ppm | Check sources/Ventilate |
| **Alcohol** | <1000 ppm | >1000 ppm | N/A | Monitor activity |

## TTN Payload Decoders

Each MCCI LMIC example includes comprehensive TTN payload decoder functions:

### **Decoder Features:**
- ✅ **Gas concentration calculations** for all detectable gases
- ✅ **Safety level assessment** (Normal/Caution/Warning/Danger)
- ✅ **Environmental type detection** (Kitchen/Garage/Industrial)
- ✅ **Air quality indexing** (1-4 scale)
- ✅ **Action recommendations** (Monitor/Ventilate/Evacuate)
- ✅ **Sensor diagnostics** and calibration status
- ✅ **Historical data compatibility** for trend analysis

### **Example TTN v3 Decoder (Raw MQ2):**
```javascript
function decodeUplink(input) {
  var bytes = input.bytes;
  var raw_adc = (bytes[0] << 8) | bytes[1];
  var sensor_resistance = ((bytes[2] << 8) | bytes[3]) * 10;
  var rs_r0_ratio = ((bytes[4] << 8) | bytes[5]) / 1000.0;
  
  // Calculate all gas concentrations
  var lpg_ppm = Math.round(987.26 * Math.pow(rs_r0_ratio, -2.162));
  var methane_ppm = Math.round(2217.8 * Math.pow(rs_r0_ratio, -2.827));
  var co_ppm = Math.round(605.18 * Math.pow(rs_r0_ratio, -3.937));
  
  return {
    data: {
      sensor: {
        raw_adc_value: raw_adc,
        resistance_ohms: sensor_resistance,
        rs_r0_ratio: rs_r0_ratio
      },
      concentrations: {
        lpg_ppm: lpg_ppm,
        methane_ppm: methane_ppm,
        co_ppm: co_ppm
      },
      safety: {
        level: co_ppm > 200 ? "DANGER" : lpg_ppm > 1000 ? "WARNING" : "SAFE"
      }
    }
  };
}
```

## Memory Usage Optimization

### Arduino Uno Memory Comparison
| Approach | Memory Used | Available Space | Recommended For |
|----------|------------|-----------------|-----------------|
| **Raw sensors (TTN calc)** | ~30-35% | 20-23KB free | **Most projects** |
| **Device calculations** | ~70% | 9KB free | Simple monitoring |
| **SX12XX professional** | ~45-55% | 15-18KB free | **Advanced features** |

### **Memory Optimization Benefits:**
- ✅ **More memory** for additional sensors
- ✅ **Room for LCD displays** or SD card logging  
- ✅ **Stable operation** with memory headroom
- ✅ **Future expansion** capability
- ✅ **Flexible gas formulas** - update on TTN without reprogramming

### **Choosing the Right Approach:**

#### **For Arduino Uno Projects:**
1. **Raw sensor data** (MCCI_ttn_Enhanced_Raw_*) - **Recommended**
2. **SX12XX library** - For professional features
3. **Avoid device calculations** - Use TTN decoders instead

#### **For Arduino Mega Projects:**
- Any approach works well
- Can combine multiple sensors
- Room for complex algorithms

## Sensor Calibration

### MQ2 Gas Sensors
1. **Preheat Period:** 24-48 hours in clean air for stable readings
2. **Warmup:** 20-second warmup in each power cycle
3. **Baseline (R0):** Resistance measured in clean air
4. **Regular Recalibration:** Monthly in known clean environment
5. **Temperature Compensation:** Consider ambient temperature effects

```cpp
// Auto-calibration example
int initial_adc = analogRead(MQ2_PIN);
float initial_Rs = calculateResistance(initial_adc);
R0_LPG = R0_METHANE = R0_CO = initial_Rs; // Rough baseline
```

### DS18B20 Temperature Sensor
- ✅ **Factory calibrated** - no user calibration needed
- ✅ **±0.5°C accuracy** from -10°C to +85°C
- ✅ **Automatic error detection** for disconnected sensors

### DHT22 Temperature & Humidity Sensor
- ✅ **Factory calibrated** - no user calibration needed
- ✅ **±0.5°C temperature accuracy**
- ✅ **±2-5% humidity accuracy**
- ✅ **Built-in checksum validation**

## Power Consumption

### Typical Current Draw
| Component | Active | Sleep | Notes |
|-----------|--------|-------|-------|
| **Arduino Uno** | ~20mA | ~0.03mA | With low-power library |
| **Dragino Shield** | ~120mA (TX) | ~1.5μA | During transmission |
| **MQ2 Gas Sensor** | ~150mA | N/A | Continuous heating |
| **DS18B20** | ~1.5mA | ~1μA | During conversion |
| **DHT22** | ~2.5mA | ~15μA | During measurement |

### Battery Life Estimation
| Configuration | Estimated Battery Life | Power Source |
|---------------|----------------------|--------------|
| **Gas sensors (MQ2)** | 1-3 days | USB/Wall adapter recommended |
| **Temperature only** | 6-12 months | Battery feasible |
| **Temperature + humidity** | 4-8 months | Battery feasible |
| **Professional SX12XX** | 8-15 months | Optimized for battery |

### **Power Optimization Tips:**
- 🔋 **External power** recommended for gas sensors
- 💤 **Low power modes** essential for battery operation
- ⏰ **Longer intervals** save significant power
- 🔌 **DHT22 power control** can be switched on/off

## Environmental Applications

### Indoor Air Quality Monitoring
- **Kitchen safety:** LPG leak detection with immediate alerts
- **Garage monitoring:** CO from vehicles and generators
- **Industrial spaces:** Multiple gas monitoring with safety systems
- **Home automation:** Temperature-based HVAC control

### Safety Systems
- **Gas leak alarms:** Immediate alerts for dangerous levels
- **HVAC integration:** Automatic ventilation control
- **Emergency response:** Remote monitoring capabilities
- **Data logging:** Historical trend analysis and compliance

### Agricultural Applications
- **Greenhouse monitoring:** Temperature, humidity, and CO₂ levels
- **Livestock areas:** Methane and ammonia monitoring
- **Storage facilities:** Gas buildup prevention
- **Cold storage:** Temperature monitoring with alerts

### Research and Development
- **Environmental studies:** Long-term air quality data collection
- **Indoor air quality research:** Correlation studies
- **Sensor network testing:** Multi-node deployments
- **Gateway development:** Custom protocol implementation

## Gateway Integration

### Low-Cost LoRa Gateway (SX12XX Examples)
The SX12XX examples are compatible with **Congduc Pham's Low-Cost LoRa Gateway**:

```bash
# Example gateway output
2025-01-02T10:30:15.123456> getting pkt from radio(466 bytes to read)
2025-01-02T10:30:15.124567> 
--> LoRa RX info: SNR=8 RSSI=-45
--> src=14 seq=42 len=89 SNR=8 RSSI=-45
--> \!TC/23.45/LPG/250.1/CH4/180.3/CO/45.2/H2/120.5/ALC/85.7/ADC/345
--> post-processing received data
```

### **Gateway Features:**
- ✅ **Automatic parsing** of sensor data format
- ✅ **Cloud integration** (ThingSpeak, TTN, custom)
- ✅ **Web interface** for monitoring
- ✅ **Data logging** and visualization
- ✅ **Alert systems** via email/SMS

## Troubleshooting

### Common Issues

#### 1. **Compilation Errors**
**Problem:** `fatal error: lmic.h: No such file or directory`  
**Solution:** Install MCCI LoRaWAN LMIC library via Library Manager

**Problem:** `SX127XLT.h: No such file or directory`  
**Solution:** Install SX12XX-LoRa library by Stuart Robinson

#### 2. **Memory Issues on Arduino Uno**
**Problem:** Sketch too large for memory  
**Solution:** 
- Use raw sensor examples (MCCI_ttn_Enhanced_Raw_*)
- Choose SX12XX library over enhanced MCCI examples
- Remove unused libraries

#### 3. **No Temperature Reading**
**Problem:** DS18B20 returns -127°C or 85°C  
**Solution:** 
- Check 4.7kΩ pullup resistor between DATA and VCC
- Verify wiring (VCC→5V, GND→GND, DATA→D4)
- Test with OneWire scanner example
- Ensure sensor is genuine DS18B20

#### 4. **No Gas Detection**
**Problem:** MQ2 always shows same values  
**Solution:**
- Ensure 20-second warmup minimum
- Check 5V power supply (gas sensors require 5V)
- Verify analog pin connections (A0 for MQ2)
- Allow 24-48 hour preheating for accurate readings
- Test in different gas environments

#### 5. **LoRa Transmission Failed**
**Problem:** No packets received at gateway/TTN  
**Solution:**
- **MCCI LMIC:** Verify TTN credentials (DevAddr, NwkSKey, AppSKey)
- **SX12XX:** Check gateway address and frequency settings
- Verify antenna connection and placement
- Ensure correct regional frequency (EU868/US915)
- Check pin mapping matches your hardware
- Test with ping-pong example first

#### 6. **TTN Decoder Errors**
**Problem:** Payload decode fails in TTN console  
**Solution:**
- Copy decoder function exactly from payload.txt
- Verify payload length matches expected bytes
- Check endianness (MSB vs LSB) in decoder
- Test decoder with sample hex values

#### 7. **Power Consumption Too High**
**Problem:** Battery drains quickly  
**Solution:**
- Enable low power modes (`#define LOW_POWER`)
- Increase transmission intervals
- Remove gas sensors for battery operation
- Use external power for continuous monitoring

#### 8. **Intermittent Sensor Readings**
**Problem:** Sensors work sometimes, not others  
**Solution:**
- Check loose connections
- Verify power supply stability (especially 5V for gas sensors)
- Add delay between sensor readings
- Implement error handling and retry logic

### Debug Tips
1. **Enable Serial Monitor** at correct baud rate (115200 for most examples)
2. **Check sensor readings** individually before LoRa transmission
3. **Verify payload format** in hex output matches expected structure
4. **Monitor TTN console** or gateway logs for received packets
5. **Use ping-pong examples** for basic communication testing
6. **Test sensors separately** before combining
7. **Verify library versions** match requirements

### Getting Help
- **Arduino Forum:** https://forum.arduino.cc/
- **TTN Community:** https://www.thethingsnetwork.org/community/
- **MCCI LMIC Issues:** https://github.com/mcci-catena/arduino-lmic/issues
- **SX12XX Library:** https://github.com/StuartsProjects/SX12XX-LoRa

## License and Credits

This collection builds upon excellent open-source work:

### **Libraries Used:**
- **MCCI LoRaWAN LMIC library** - MCCI Corporation
- **SX12XX-LoRa library** - Stuart Robinson  
- **Arduino Core libraries** - Arduino LLC
- **OneWire & DallasTemperature** - Jim Studt, Miles Burton
- **DHT sensor library** - Adafruit Industries

### **Gateway Compatibility:**
- **Low-Cost LoRa Gateway** - Congduc Pham (University of Pau, France)

### **Gas Sensor Formulas:**
- Based on manufacturer datasheets and community research
- Empirical formulas derived from sensor characterization

## Important Safety Notice

⚠️ **SAFETY DISCLAIMER:** These examples are for **educational and development purposes**. While they provide useful gas detection capabilities, they should **NOT** be used as the sole safety system in critical applications.

### **For Life-Safety Applications:**
- ✅ Use **certified commercial gas detectors**
- ✅ Install **multiple redundant sensors**
- ✅ Include **professional monitoring services**
- ✅ Follow **local safety codes and regulations**
- ✅ Regular **professional calibration and maintenance**

### **Recommended Use Cases:**
- 🧪 **Educational projects** and learning
- 📊 **Environmental monitoring** and data collection
- 🏠 **Home automation** and convenience
- 🔬 **Research and development**
- 📈 **Trend analysis** and early warning systems

**Always test thoroughly and understand the limitations of DIY sensors!**