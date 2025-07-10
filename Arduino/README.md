# Arduino LoRa Sketches

This repository contains a collection of Arduino sketches for LoRa communication using SX127X, SX126X, and SX128X radio modules. These sketches are designed to work with the Low Cost LoRa Gateway framework and provide various sensor examples and communication patterns.

## Repository Structure

```
Arduino/
├── libraries/
│   ├── SX1272/                  # SX1272 library (Congduc Pham modified)
│   └── SX12XX/                  # SX12XX library (Stuart Robinson modified)
├── Arduino_LoRa_SX12XX_DS18B20/ # DS18B20 temperature sensor with SX12XX
├── Enhanced_Lora_DS18B20_Downlink_Controlled/ # Downlink controlled DS18B20
├── Lora_Ping_Pong_SX12XXX/      # Ping-pong using SX12XX library
├── Lora_Enhanced_MQ2_DS18B20_SX12XXX/ # Combined MQ2 gas and DS18B20 temp sensor
├── Lora_Custom_Payload_SX12XXX/ # Custom payload examples
├── Lora_DS18B20_SX12XXX/        # Basic DS18B20 temperature sensor
├── MCCI_ttn_DS18B20_MQ2/        # TTN integration with DS18B20 and MQ2
└── MCCI_ttn_Enhanced_Raw_MQ2_DS18B20/ # Enhanced TTN with raw sensor data
```

## Libraries

### SX1272 Library
- **Path**: `libraries/SX1272/`
- **Description**: Modified SX1272 library by Congduc Pham
- **Original**: Based on Libelium's SX1272 library
- **Features**: 
  - Support for SX1272/SX1276 modules
  - Low power operations
  - Carrier sense capability
  - EEPROM configuration storage

### SX12XX Library
- **Path**: `libraries/SX12XX/`
- **Description**: Modified SX12XX library from Stuart Robinson
- **Original**: https://github.com/StuartsProjects/SX12XX-LoRa
- **Supported Chips**: SX1261, SX1262, SX1268, SX1272, SX1276, SX1277, SX1278, SX1279, SX1280, SX1281
- **Warning**: This version has been modified for the LowCostLoRaGw framework and may not be compatible with the original library

## Sketches

Based on the actual sketches available in the project knowledge:

### Temperature Sensors

#### Arduino_LoRa_SX12XX_DS18B20
- **Description**: DS18B20 digital temperature sensor using SX12XX library
- **Author**: Congduc Pham, University of Pau, France
- **Features**:
  - Digital temperature sensor (DS18B20)
  - OneWire communication
  - Precise temperature readings
  - SX12XX library compatibility
  - Extended version with AES and custom Carrier Sense features

#### Lora_DS18B20_SX12XXX
- **Description**: Basic DS18B20 temperature sensor with LoRa transmission
- **Features**:
  - DS18B20 temperature sensor support
  - Enhanced version with low power, ACK, EEPROM features
  - Compatible with Low-Cost LoRa Gateway
  - Hardware connection: DS18B20 data pin -> Arduino Pin 3

#### Enhanced_Lora_DS18B20_Downlink_Controlled
- **Description**: DS18B20 sensor with downlink control capability
- **Features**:
  - Supports 16 network parameter configurations
  - Downlink command control: /@C<index># where index is 0-15
  - Dynamic configuration changes via radio commands
  - Network characterization capabilities

### Gas and Environmental Sensors

#### Lora_Enhanced_MQ2_DS18B20_SX12XXX
- **Description**: Combined DS18B20 temperature and MQ2 multi-gas sensor
- **Features**:
  - Dual sensor monitoring (temperature + gas)
  - MQ2 gas sensor for CO, LPG, methane, alcohol detection
  - Environmental safety thresholds and alerts
  - Enhanced monitoring station capabilities
  - Hardware: DS18B20 on Pin 3, MQ2 on Pin A0

### Communication Tests

#### Lora_Ping_Pong_SX12XXX
- **Description**: Ping-pong implementation using SX12XX library
- **Author**: Congduc Pham, University of Pau, France (based on SX12XX examples)
- **Features**:
  - Simple ping-pong test with gateway
  - SX126X/SX127X/SX128X support
  - Addressed packet transmission
  - Real-time packet statistics
  - 4-byte header communication

### Custom Communication

#### Lora_Custom_Payload_SX12XXX
- **Description**: Custom payload format examples with interactive terminal
- **Features**:
  - Interactive serial input for custom messages
  - Real-time message composition and transmission
  - Professional communication features
  - ACK support for reliable messaging
  - Carrier sense before transmission
  - Point-to-point and broadcast messaging

### TTN Integration

#### MCCI_ttn_DS18B20_MQ2
- **Description**: TTN (The Things Network) integration with dual sensors
- **Features**: LoRaWAN protocol support with DS18B20 and MQ2 sensors

#### MCCI_ttn_Enhanced_Raw_MQ2_DS18B20
- **Description**: Enhanced TTN integration with raw sensor data
- **Features**: Raw ADC readings transmission with optimized payload structure

## Supported Hardware

### Arduino Boards
- Arduino Pro Mini
- Arduino Nano
- Arduino Uno
- Arduino Mega2560
- Arduino Due
- Arduino Zero/M0

### ESP Boards
- ESP8266 (ESP01, NodeMCU)
- ESP32

### Teensy Boards
- Teensy LC (MKL26Z64)
- Teensy 3.1/3.2 (MK20DX256)
- Teensy 3.5 (MK64FX512)
- Teensy 3.6 (MK66FX1M0)

### Adafruit Boards
- Feather 32U4
- Feather M0

### LoRa Modules
- SX1272/SX1276 (RFM95W, RFM96W, RFM98W)
- SX1261/SX1262/SX1268
- SX1280/SX1281

## Pin Configurations

### Default SX127X Pin Mapping
```cpp
#define NSS 10      // SPI Chip Select
#define NRESET 6    // Reset pin
#define DIO0 2      // DIO0 pin
#define DIO1 3      // DIO1 pin (optional)
#define DIO2 4      // DIO2 pin (optional)
```

### Default SX126X Pin Mapping
```cpp
#define NSS 10      // SPI Chip Select
#define NRESET 4    // Reset pin
#define RFBUSY 5    // Busy pin
#define DIO1 3      // DIO1 pin
```

## Configuration Features

### Power Management
- Low power hibernation modes
- Configurable sleep periods
- Power-efficient sensor reading
- Battery operation support

### Communication Settings
- Multiple frequency bands (433MHz, 868MHz, 915MHz)
- Configurable spreading factors (SF7-SF12)
- Variable bandwidth settings
- Adaptive data rates

### Security Features
- AES encryption support
- LoRaWAN security
- Application key management
- Device address configuration

## Getting Started

1. **Install Libraries**: Copy the `libraries/` folder contents to your Arduino libraries directory
2. **Select Sketch**: Choose the appropriate sketch for your application
3. **Configure Hardware**: Update pin definitions for your specific hardware setup
4. **Set Parameters**: Modify node address, frequency, and other parameters as needed
5. **Upload**: Compile and upload to your Arduino board

## Regional Regulations

The sketches support multiple regional frequency regulations:
- **ETSI Europe**: 868MHz band, 14dBm max power
- **FCC US**: 915MHz band, 14dBm max power
- **Senegal**: 868MHz band, 10dBm max power

## Examples Usage

### Simple Temperature Sensor
```cpp
// Basic temperature reading
double temp = sensor_getValue();
sprintf(message, "\\!TC/%.2f", temp);
```

### Gas Sensor Reading
```cpp
// MQ2 gas sensor
int gasLevel = analogRead(MQ2_PIN);
float gasRatio = calculateGasRatio(gasLevel);
```

### Ping-Pong Communication
```cpp
// Send ping and wait for pong
int result = sx1272.sendPacketTimeoutACK(DEFAULT_DEST_ADDR, message, size);
if (result == 0) {
    Serial.println("Pong received!");
}
```

## Troubleshooting

### Common Issues
1. **Module Not Detected**: Check SPI connections and power supply
2. **No Communication**: Verify frequency and LoRa parameters match gateway
3. **Poor Range**: Check antenna connection and transmission power settings
4. **High Power Consumption**: Enable low power modes and optimize sleep periods

### Debug Features
- Serial output for packet transmission status
- RSSI and SNR reporting
- Register dump capabilities
- Carrier sense monitoring

## Contributing

When contributing to this repository:
1. Follow the existing code structure
2. Use the defined hardware abstraction functions
3. Maintain compatibility with the LowCostLoRaGw framework
4. Test on multiple hardware platforms
5. Document any new features or changes

## License

This project is distributed under the GNU General Public License v3.0. See individual files for specific copyright information.

## Acknowledgments

- **Congduc Pham** (University of Pau, France) - LowCostLoRaGw framework and library modifications
- **Stuart Robinson** - Original SX12XX library
- **Libelium** - Original SX1272 library
- **MCCI Corporation** - LoRaWAN library support

## Support

For support and questions:
- Check the documentation in individual sketch folders
- Review the LowCostLoRaGw framework documentation
- Consult the original library repositories for hardware-specific issues