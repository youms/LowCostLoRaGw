# Arduino LoRa Gas & Temperature Sensor Examples
==================================================

This folder contains Arduino sketches for LoRa-based gas detection and temperature monitoring using MCCI LMIC library. The examples are specifically designed for **Arduino Uno**, **Arduino Mega**, and **Arduino Micro** with **Dragino LoRa Shield** and demonstrate how to build low-cost IoT sensor nodes for environmental monitoring.

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

#### MQ9 Gas Sensor  
```
MQ9 Pin         Arduino Pin      Function
  VCC    ----------- 5V          (Power 5V)
  GND    ----------- GND         (Ground)
  AO     ----------- A1          (Analog Output)
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

## Library Dependencies

### Required Libraries (Install via Arduino Library Manager)
1. **MCCI LoRaWAN LMIC library** - Main LoRaWAN communication library
2. **OneWire** - For DS18B20 communication protocol
3. **DallasTemperature** - High-level DS18B20 temperature sensor library

### Installation Steps
1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search and install:
   - "MCCI LoRaWAN LMIC library" by MCCI Corporation
   - "OneWire" by Jim Studt
   - "DallasTemperature" by Miles Burton

## Configuration Setup

### Regional Frequency Configuration

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

**Note:** Option 1 (direct config.h edit) is simpler and overrides any project config settings.

### TTN (The Things Network) Configuration
All examples use **ABP (Activation By Personalization)**. You need to:

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

## Example Sketches

### 1. **`mq2_raw_sensor.ino`** - MQ2 Gas Sensor (Memory Optimized)
**Features:**
- ✅ Raw ADC readings for TTN-side gas calculations
- ✅ Memory optimized for Arduino Uno (~30% usage)
- ✅ 6-byte compact payload
- ✅ Basic gas detection alerts
- ✅ Sensor diagnostics

**Gas Detection Capability:**
- LPG (Liquefied Petroleum Gas)
- Methane (Natural Gas)
- Carbon Monoxide (CO)
- Hydrogen (H₂)
- Alcohol/Ethanol
- Propane
- Smoke particles

**Payload Structure (6 bytes):**
```
Byte 0-1: Raw ADC value (0-1023)
Byte 2-3: Sensor resistance Rs÷10 (ohms)
Byte 4-5: Rs/R0 ratio × 1000
```

### 2. **`mq2_ds18b20_combined.ino`** - Combined Gas & Temperature
**Features:**
- ✅ MQ2 gas sensor + DS18B20 temperature sensor
- ✅ Memory optimized for Arduino Uno (~35% usage)
- ✅ 8-byte payload (6 for MQ2 + 2 for temperature)
- ✅ Combined safety alerts
- ✅ Environmental monitoring

**Payload Structure (8 bytes):**
```
Byte 0-1: MQ2 ADC value
Byte 2-3: MQ2 resistance Rs÷10
Byte 4-5: Rs/R0 ratio × 1000  
Byte 6-7: Temperature × 100 (supports negative temps)
```

### 3. **`mq9_raw_sensor.ino`** - MQ9 Gas Sensor
**Features:**
- ✅ MQ9 specific gas detection (CO, CH4, LPG)
- ✅ Optimized for MQ9 characteristics
- ✅ Similar payload structure to MQ2
- ✅ CO-specific safety thresholds

**MQ9 Gas Detection:**
- Carbon Monoxide (primary)
- Methane
- LPG
- Flammable gases

### 4. **`multi_gas_temperature.ino`** - MQ2 + MQ9 + DS18B20
**Features:**
- ✅ Dual gas sensors + temperature
- ✅ Comprehensive environmental monitoring
- ✅ 12-byte payload for complete data
- ✅ Cross-sensor validation
- ✅ Advanced safety algorithms

**Payload Structure (12 bytes):**
```
Byte 0-1: MQ2 ADC value
Byte 2-3: MQ2 Rs/R0 ratio × 1000
Byte 4-5: MQ9 ADC value  
Byte 6-7: MQ9 Rs/R0 ratio × 1000
Byte 8-9: Temperature × 100
Byte 10-11: Reserved/Status flags
```

### 5. **`interactive_sender.ino`** - Manual Message Sender
**Features:**
- ✅ Send custom messages via Serial Monitor
- ✅ Interactive testing and debugging
- ✅ Variable payload length (1-64 bytes)
- ✅ Real-time transmission feedback

### 6. **`Lora_Enhanced_MQ2_DS18B20_SX12XXX.ino`** - Professional Environmental Station
**Features:**
- ✅ **Modular SX12XX library** support (SX126X, SX127X, SX128X)
- ✅ **Combined sensors** (DS18B20 + MQ2) with error handling
- ✅ **Advanced power management** with hibernation modes
- ✅ **Professional gateway compatibility** (Congduc Pham format)
- ✅ **EEPROM persistence** for packet sequence numbers
- ✅ **ACK support** with SNR reporting
- ✅ **Environmental alerts** with safety thresholds
- ✅ **Carrier sense** before transmission (Listen-Before-Talk)
- ✅ **CRC verification** and comprehensive error handling
- ✅ **Multi-platform support** (AVR, SAMD, Teensy, ESP8266/32)

**Enhanced Payload Format:**
```
\!TC/23.45/LPG/250.1/CH4/180.3/CO/45.2/H2/120.5/ALC/85.7/ADC/345
```

**Advanced Configuration Options:**
```cpp
#define WITH_EEPROM              // Persistent packet counters
#define LOW_POWER                // Sleep modes for battery operation  
#define LOW_POWER_HIBERNATE      // Deepest sleep (platform-specific)
#define WITH_ACK                 // Request acknowledgments from gateway
#define ENVIRONMENTAL_ALERTS     // Enhanced safety monitoring
#define PUBLIC_SYNCWORD         // LoRaWAN gateway compatibility
#define MY_FREQUENCY 868100000  // Custom frequency override
```

**Platform-Specific Features:**
- **Arduino AVR** (Uno/Nano/Pro Mini): LowPower library with 8-second cycles
- **Teensy** (LC/31/32/35/36): Snooze library with precise timing
- **SAMD21** (Zero/M0): Built-in RTC for accurate wake-up
- **ESP8266/ESP32**: Deep sleep with microsecond precision
- **Arduino Mega**: Extended memory support for complex projects

## SX12XX Library (Stuart Robinson)

The **professional environmental station** example uses the **SX12XX library** for advanced LoRa communication, offering significant advantages over MCCI LMIC for direct sensor applications.

### **SX12XX Advantages over MCCI LMIC:**
- ✅ **Multi-chip support** (SX126X, SX127X, SX128X) 
- ✅ **Direct LoRa control** without LoRaWAN protocol overhead
- ✅ **Better power management** with precise sleep/wake control
- ✅ **Lower memory usage** - perfect for Arduino Uno
- ✅ **Professional radio management** with carrier sense
- ✅ **Better downlink handling** with precise timing control
- ✅ **Modular design** - easy to add/remove features
- ✅ **Gateway compatibility** with packet formatting
- ✅ **Hardware abstraction** for multiple platforms

### **Supported LoRa Chips:**
| Chip Family | Models | Frequency | Key Features |
|-------------|--------|-----------|--------------|
| **SX127X** | SX1272, SX1276, SX1277, SX1278, SX1279 | 433/868/915 MHz | Most common, Dragino compatible |
| **SX126X** | SX1261, SX1262, SX1268 | 433/868/915 MHz | Latest generation, better efficiency |
| **SX128X** | SX1280, SX1281 | 2.4 GHz | High-speed, ranging capability |

### **Hardware Configuration Selection:**
```cpp
// Uncomment only ONE chip family
//#define SX126X          // For newer modules (SX1262, etc.)
#define SX127X           // For Dragino Shield (SX1276/RFM95W) 
//#define SX128X          // For 2.4GHz modules

// Pin definitions adjust automatically based on selection
#ifdef SX127X
#define NSS 10           // SPI Chip Select
#define NRESET 4         // Reset pin  
#define DIO0 -1          // Not used in polling mode
#define LORA_DEVICE DEVICE_SX1276
#endif
```

### **Professional Features:**

#### **1. Carrier Sense (Listen-Before-Talk):**
```cpp
// Check channel before transmission
LT.CarrierSense();
if (LT.transmitAddressed(message, size, type, dest, source, timeout, power, WAIT_TX)) {
  // Transmission successful
}
```

#### **2. Advanced Power Management:**
```cpp
// Put radio to sleep between transmissions  
LT.setSleep(CONFIGURATION_RETENTION);

// Platform-specific deep sleep
#ifdef __SAMD21G18A__
  rtc.standbyMode();              // M0/Zero RTC sleep
#elif defined ESP8266
  ESP.deepSleep(period*1000000);  // ESP8266 deep sleep
#else
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // AVR sleep
#endif
```

#### **3. Packet Acknowledgment System:**
```cpp
#ifdef WITH_ACK
  uint8_t p_type = PKT_TYPE_DATA | PKT_FLAG_ACK_REQ;
  
  if (LT.transmitAddressed(message, size, p_type, dest, source, timeout, power, WAIT_TX)) {
    if (LT.readAckStatus()) {
      Serial.print("ACK received! SNR: ");
      Serial.println(LT.readPacketSNRinACK());
    }
  }
#endif
```

#### **4. Professional Error Handling:**
```cpp
if (!LT.transmit(data, size, timeout, power, WAIT_TX)) {
  uint16_t IRQStatus = LT.readIrqStatus();
  Serial.print("Transmission failed - IRQ: 0x");
  Serial.println(IRQStatus, HEX);
  LT.printIrqStatus();  // Detailed error analysis
}
```

### **When to Use SX12XX vs MCCI LMIC:**

#### **Use SX12XX for:**
- 🔬 **Professional sensor networks** with custom protocols
- 🏠 **Point-to-point** communication systems
- 🔋 **Battery-powered** remote monitoring
- 🎯 **Custom gateway** integration (Congduc Pham style)
- ⚡ **Memory-constrained** Arduino Uno projects
- 🔄 **Bidirectional** communication needs
- 🧪 **Research and development** projects

#### **Use MCCI LMIC for:**
- 🌐 **Standard LoRaWAN** networks (TTN, ChirpStack)
- 🏢 **Commercial IoT** deployments  
- 🔐 **Secure communications** with encryption
- 📡 **Multi-operator** compatibility
- 🌍 **Global deployment** with regional compliance

## LoRa Mode

The examples use **LoRa Mode 1** by default, which provides the best range and sensitivity for sensor applications.

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

### **Mode Selection Guidelines:**

#### **For Gas/Temperature Sensors (Recommended: Mode 1)**
- 🏠 **Indoor monitoring:** Mode 1 (SF12BW125) - Maximum sensitivity
- 🏭 **Industrial areas:** Mode 1 (SF12BW125) - Penetrates obstacles  
- 🌾 **Outdoor long-range:** Mode 1 (SF12BW125) - Up to 15km rural
- 🏘️ **Urban environments:** Mode 3 (SF10BW125) - Good range, faster

#### **For Interactive/Testing Applications:**
- 🧪 **Development/testing:** Mode 6 (SF11BW500) - Balance of range/speed
- 💬 **Interactive sender:** Mode 8 (SF9BW500) - Fast response
- 📊 **High-frequency data:** Mode 10 (SF7BW500) - Maximum speed

### **Changing LoRa Mode in Code:**
```cpp
// In your Arduino sketch, modify this line:
#define LORAMODE 1  // Change to desired mode (1-10)

// Or for SX12XX library:
LT.setupLoRa(868100000, 0, LORA_SF10, LORA_BW_125, LORA_CR_4_5); // Mode 3
```

### **Range vs Speed Trade-off:**
- **Higher SF (12)** = Longer range, slower speed, more power efficient
- **Lower SF (7)** = Shorter range, faster speed, less power efficient  
- **Wider BW (500)** = Faster speed, shorter range
- **Narrower BW (125)** = Longer range, slower speed

## AES Encryption

For secure sensor data transmission, AES encryption can be enabled to protect against eavesdropping and tampering.

### **Enabling AES in MCCI LMIC:**
```cpp
// Add to your sketch before setup()
#define WITH_AES 1

// The LMIC library handles LoRaWAN encryption automatically
// using your NwkSKey and AppSKey
```

### **AES Features:**
- ✅ **AES-128 encryption** for payload data
- ✅ **LoRaWAN standard** encryption (automatic)
- ✅ **Message integrity** with MIC (Message Integrity Code)
- ✅ **Replay protection** with frame counters
- ✅ **Network-level security** with NwkSKey
- ✅ **Application-level security** with AppSKey

### **Security Levels:**

#### **LoRaWAN (Recommended):**
- 🔐 **Network encryption** (NwkSKey) - Protects routing info
- 🔐 **Application encryption** (AppSKey) - Protects sensor data
- 🔐 **Message authentication** (MIC) - Prevents tampering
- 🔐 **Frame counter** - Prevents replay attacks

#### **Custom AES (Advanced):**
```cpp
// For SX12XX library with custom encryption
#include "AES.h"

void encryptSensorData(uint8_t* data, uint8_t len) {
  uint8_t key[16] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
                     0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10};
  
  AES aes;
  aes.do_aes_encrypt(data, len, key, sizeof(key));
}
```

### **Memory Impact:**
- **MCCI LMIC:** ~2KB additional for LoRaWAN crypto
- **Custom AES:** ~1KB for AES library
- **Arduino Uno:** May need optimization for complex projects

### **When to Use Encryption:**
- 🏢 **Commercial deployments** - Always recommended
- 🏭 **Industrial monitoring** - Critical for security
- 🏠 **Home automation** - Protects privacy
- 🧪 **Development/testing** - Optional, can disable for debugging

## Receive Window (Downlink)

Downlink communication allows the gateway or server to send commands back to sensor nodes for configuration, control, or acknowledgment.

### **LoRaWAN Downlink (MCCI LMIC):**

#### **Enabling Downlink:**
```cpp
// Add to your sketch
#define WITH_RCVW 1  // Enable receive window

void onEvent(ev_t ev) {
  switch(ev) {
    case EV_TXCOMPLETE:
      if (LMIC.dataLen) {
        Serial.print("Downlink received: ");
        for (int i = 0; i < LMIC.dataLen; i++) {
          Serial.print((char)LMIC.frame[LMIC.dataBeg + i]);
        }
        Serial.println();
        processDownlinkCommand();
      }
      break;
  }
}
```

#### **LoRaWAN RX Windows:**
- **RX1 Window:** Opens 1 second after transmission (same frequency/SF)
- **RX2 Window:** Opens 2 seconds after transmission (869.525MHz, SF9 for EU)
- **Automatic timing** handled by LMIC library

### **Simple LoRa Downlink (SX12XX):**
```cpp
void sendSensorDataWithDownlink() {
  // Send uplink data
  uint8_t uplink[] = "Gas:250ppm Temp:23.5C";
  LT.transmit(uplink, sizeof(uplink), 10000, 14, WAIT_TX);
  
  // Open receive window immediately
  uint8_t downlink[64];
  uint8_t rxLen = LT.receive(downlink, sizeof(downlink), 5000, WAIT_RX);
  
  if (rxLen > 0) {
    Serial.println("Command received!");
    processCommand(downlink, rxLen);
  }
}

void processCommand(uint8_t* cmd, uint8_t len) {
  String command = String((char*)cmd);
  
  if (command.startsWith("LED")) {
    digitalWrite(LED_PIN, command.endsWith("ON") ? HIGH : LOW);
  } else if (command.startsWith("INTERVAL")) {
    // Change transmission interval
    updateInterval(command.substring(8).toInt());
  } else if (command.startsWith("CALIBRATE")) {
    // Recalibrate gas sensors
    calibrateSensors();
  }
}
```

### **Downlink Command Examples:**

#### **Device Control:**
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

### **Downlink Timing Strategies:**

#### **Immediate Response:**
```cpp
// Send data, listen immediately
transmitData();
if (receiveDownlink(2000)) {  // 2 second timeout
  processCommand();
}
```

#### **LoRaWAN-style Delays:**
```cpp
// Send data
transmitData();

// RX1 window (1s delay)
delay(1000);
if (receiveDownlink(1000)) {
  processCommand();
  return;
}

// RX2 window (additional 1s delay)  
delay(1000);
if (receiveDownlink(1000)) {
  processCommand();
}
```

#### **Power-Optimized:**
```cpp
// Short receive window to save battery
transmitData();
if (receiveDownlink(500)) {  // Very short timeout
  processCommand();
} else {
  enterSleepMode();  // No command, go to sleep
}
```

### **Downlink Best Practices:**
- ⏰ **Keep RX windows short** to save power
- 🔄 **Implement command acknowledgment** for reliability
- 🛡️ **Validate commands** before execution
- 📝 **Log downlink activity** for debugging
- 🔋 **Balance responsiveness vs battery life**

## Advanced Features

### Gateway-Compatible Data Format
The professional environmental station uses **Congduc Pham's gateway format** for maximum compatibility:

```cpp
// Standard format: \!SENSOR/value/SENSOR/value...
sprintf(message, "\\!TC/%s/LPG/%s/CH4/%s/CO/%s/H2/%s/ALC/%s/ADC/%d", 
        temp_str, lpg_str, ch4_str, co_str, h2_str, alcohol_str, sensorValue);

// Example output: \!TC/23.45/LPG/250.1/CH4/180.3/CO/45.2/H2/120.5/ALC/85.7/ADC/345
```

**Nomenclature Reference:**
- **TC**: Temperature Celsius  
- **LPG**: Liquefied Petroleum Gas (ppm)
- **CH4**: Methane (ppm)
- **CO**: Carbon Monoxide (ppm)  
- **H2**: Hydrogen (ppm)
- **ALC**: Alcohol (ppm)
- **ADC**: Raw ADC value for diagnostics

### EEPROM Persistence
Maintains packet sequence numbers across power cycles and reboots:

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
```

**Benefits:**
- ✅ **Continuous numbering** after device reset
- ✅ **Packet loss detection** at gateway
- ✅ **Network debugging** assistance
- ✅ **Data integrity** verification

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

**Alert Levels:**
| Parameter | Normal | High | Danger | Action |
|-----------|--------|------|--------|--------|
| **Temperature** | 0-35°C | 35-50°C | >50°C or <0°C | Monitor/Investigate |
| **CO** | <50 ppm | 50-200 ppm | >200 ppm | Ventilate/Evacuate |
| **LPG/Methane** | <1000 ppm | 1000-5000 ppm | >5000 ppm | Check sources/Ventilate |
| **Alcohol** | <1000 ppm | >1000 ppm | N/A | Monitor activity |

### Multi-Platform Low Power Support
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

### Comprehensive Error Handling
Professional-grade error detection and recovery:

```cpp
// Sensor error detection
bool tempError = (tempC == DEVICE_DISCONNECTED_C);
bool gasError = (sensorValue == 0 || Rs == 0);
bool sensorError = tempError && gasError;  // Both failed

// Graceful degradation - continue with working sensors
if (!sensorError) {
  // Build message with available data
  if (tempError) strcpy(temp_str, "ERR");
  if (gasError) strcpy(lpg_str, "ERR");
  
  // Transmit partial data rather than complete failure
  transmitData();
}
```

### CRC and Packet Verification
Built-in data integrity checking:

```cpp
if (LT.transmitAddressed(message, r_size, p_type, dest, source, timeout, power, WAIT_TX)) {
  uint16_t localCRC = LT.CRCCCITT(message, r_size, 0xFFFF);
  Serial.print("CRC: 0x");
  Serial.println(localCRC, HEX);
  
  // Gateway can verify packet integrity
}
```

### Board Auto-Detection
Automatic platform identification for optimal configuration:

```cpp
#ifdef ARDUINO_AVR_PRO
  Serial.println("Arduino Pro Mini detected");  
#endif
#ifdef __MK66FX1M0__
  Serial.println("Teensy36 detected");
#endif
#ifdef ESP32 
  Serial.println("ESP32 detected");
#endif
// ... supports 15+ different platforms
```

## Gas Concentration Formulas

All gas concentrations are calculated on TTN using these formulas:

| Gas | Formula | Detection Range |
|-----|---------|-----------------|
| **LPG** | `987.26 × (Rs/R0)^(-2.162)` ppm | 200-10,000 ppm |
| **Methane** | `2217.8 × (Rs/R0)^(-2.827)` ppm | 300-10,000 ppm |
| **CO** | `605.18 × (Rs/R0)^(-3.937)` ppm | 300-10,000 ppm |
| **Hydrogen** | `988.05 × (Rs/R0)^(-1.767)` ppm | 300-5,000 ppm |
| **Alcohol** | `75.906 × (Rs/R0)^(-1.691)` ppm | 100-2,000 ppm |
| **Propane** | `614.56 × (Rs/R0)^(-2.564)` ppm | 200-5,000 ppm |
| **Smoke** | `143.01 × (Rs/R0)^(-2.186)` ppm | Variable |

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

## TTN Payload Decoders

Each example includes corresponding TTN payload decoder functions that provide:

- ✅ **Gas concentration calculations** for all detectable gases
- ✅ **Safety level assessment** (Normal/Caution/Warning/Danger)
- ✅ **Environmental type detection** (Kitchen/Garage/Industrial)
- ✅ **Air quality indexing** (1-4 scale)
- ✅ **Action recommendations** (Monitor/Ventilate/Evacuate)
- ✅ **Sensor diagnostics** and calibration status
- ✅ **Historical data compatibility** for trend analysis

## Memory Usage Optimization

### Arduino Uno Memory Comparison
| Approach | Memory Used | Available Space |
|----------|------------|-----------------|
| **Raw sensors (recommended)** | ~30-35% | 20-23KB free |
| **Calculated approach** | ~70% | 9KB free |

### Optimization Benefits
- ✅ **More memory** for additional sensors
- ✅ **Room for LCD displays** or SD card logging  
- ✅ **Stable operation** with memory headroom
- ✅ **Future expansion** capability
- ✅ **Flexible gas formulas** - update on TTN without reprogramming

## Sensor Calibration

### MQ2/MQ9 Gas Sensors
1. **Preheat Period:** 24-48 hours in clean air for stable readings
2. **Warmup:** 20-second warmup in each power cycle
3. **Baseline (R0):** Resistance measured in clean air
4. **Regular Recalibration:** Monthly in known clean environment
5. **Temperature Compensation:** Consider ambient temperature effects

### DS18B20 Temperature Sensor
- ✅ **Factory calibrated** - no user calibration needed
- ✅ **±0.5°C accuracy** from -10°C to +85°C
- ✅ **Automatic error detection** for disconnected sensors

## Power Consumption

### Typical Current Draw
| Component | Active | Sleep | Notes |
|-----------|--------|-------|-------|
| **Arduino Uno** | ~20mA | ~0.03mA | With low-power library |
| **Dragino Shield** | ~120mA (TX) | ~1.5μA | During transmission |
| **MQ2/MQ9 Sensor** | ~150mA | N/A | Continuous heating |
| **DS18B20** | ~1.5mA | ~1μA | During conversion |

### Battery Life Estimation
- **With gas sensors:** 1-3 days (due to continuous heating)
- **Temperature only:** 6-12 months (with proper low-power management)
- **Recommendation:** Use external power for gas sensor applications

## Troubleshooting

### Common Issues

#### 1. Compilation Errors
**Problem:** `fatal error: lmic.h: No such file or directory`
**Solution:** Install MCCI LoRaWAN LMIC library via Library Manager

#### 2. Memory Issues on Arduino Uno
**Problem:** Sketch too large for memory
**Solution:** Use raw sensor examples instead of calculated versions

#### 3. No Temperature Reading
**Problem:** DS18B20 returns -127°C or 85°C
**Solution:** 
- Check 4.7kΩ pullup resistor
- Verify wiring (VCC, GND, DATA to pin 4)
- Test with OneWire scanner example

#### 4. No Gas Detection
**Problem:** MQ2/MQ9 always shows same values
**Solution:**
- Ensure 20-second warmup minimum
- Check 5V power supply (gas sensors need 5V)
- Verify analog pin connections (A0 for MQ2, A1 for MQ9)
- Allow 24-48 hour preheating for accurate readings

#### 5. LoRa Transmission Failed
**Problem:** No packets received at gateway
**Solution:**
- Verify TTN credentials (DevAddr, NwkSKey, AppSKey)
- Check antenna connection
- Ensure correct regional frequency (EU868/US915)
- Verify pin mapping matches your hardware

#### 6. High Memory Usage Warning
**Problem:** "Low memory available, stability problems may occur"
**Solution:** 
- Use raw sensor examples
- Remove unnecessary libraries
- Reduce string constants
- Consider Arduino Mega for complex projects

### Debug Tips
1. **Enable Serial Monitor** at 115200 baud for detailed logging
2. **Check sensor readings** before transmission
3. **Verify payload format** in hex output
4. **Monitor TTN console** for received packets
5. **Use interactive sender** for communication testing

## Environmental Applications

### Indoor Air Quality Monitoring
- **Kitchen safety:** LPG leak detection
- **Garage monitoring:** CO from vehicles
- **Industrial spaces:** Multiple gas monitoring
- **Home automation:** Temperature-based control

### Safety Systems
- **Gas leak alarms:** Immediate alerts for dangerous levels
- **HVAC integration:** Automatic ventilation control
- **Emergency response:** Remote monitoring capabilities
- **Data logging:** Historical trend analysis

### Agricultural Applications
- **Greenhouse monitoring:** Temperature and CO₂ levels
- **Livestock areas:** Methane monitoring
- **Storage facilities:** Gas buildup prevention
- **Cold storage:** Temperature monitoring

## License and Credits

This collection is based on:
- **MCCI LoRaWAN LMIC library** - MCCI Corporation
- **Arduino Core libraries** - Arduino LLC
- **OneWire & DallasTemperature** - Open source community
- **Gas sensor formulas** - Based on manufacturer datasheets and community research

## Support and Community

For questions and support:
- **Arduino Forum:** https://forum.arduino.cc/
- **TTN Community:** https://www.thethingsnetwork.org/community/
- **MCCI LMIC Issues:** https://github.com/mcci-catena/arduino-lmic/issues

**Remember:** Always test your gas detection systems thoroughly and never rely solely on DIY sensors for life-safety applications. Commercial certified equipment should be used for critical safety systems.
