# Enhanced DS18B20 Downlink Controlled

## Overview

The Enhanced DS18B20 Downlink Controlled Arduino sketch provides a template for implementing remote LoRa commands for end devices. This modular implementation supports 16 predefined network parameter configurations that can be triggered remotely via downlink commands, making it ideal for network characterization and remote device management.

## Key Features

- **Remote Configuration**: Control device settings via LoRa downlink commands
- **16 Network Configurations**: Pre-defined parameter sets for comprehensive network testing
- **EEPROM Persistence**: Configuration settings survive device reboot
- **Modular Design**: Clean separation of concerns with dedicated header files
- **Custom Receive Window Timing**: Optimized for different spreading factors and bandwidths
- **DS18B20 Temperature Sensing**: Accurate temperature measurements with configurable payload sizes

## Architecture

### Core Files

```
Enhanced_Lora_DS18B20_Downlink_Controlled.ino  // Main sketch
NetworkParams.h                                 // Configuration parameters
DownlinkParser.h                               // Command parsing logic  
my_temp_sensor_code.h                          // DS18B20 sensor interface
```

### Modular Components

1. **Network Parameters Module** (`NetworkParams.h`)
   - Defines 16 test configurations with different SF/BW/CR/Payload combinations
   - Provides `updateLoRaParams()` function for dynamic parameter changes
   - Supports SX126X, SX127X, and SX128X chip families

2. **Downlink Parser Module** (`DownlinkParser.h`)
   - Handles receive window processing with adaptive timing
   - Parses downlink commands in format `/@C<index>#`
   - Manages EEPROM storage for persistent configuration

3. **Temperature Sensor Module** (`my_temp_sensor_code.h`)
   - DS18B20 sensor initialization and reading functions
   - Pin configuration and voltage scaling for different Arduino variants

## Configuration Parameters

The sketch supports 16 predefined network configurations organized in three categories:

### MIN Configurations (SF7 - Fast, Short Range)
- `MIN-SF7-BW125-T20`: SF7, BW125kHz, 20-byte payload
- `MIN-SF7-BW500-T20`: SF7, BW500kHz, 20-byte payload  
- `MIN-SF7-BW125-T50`: SF7, BW125kHz, 50-byte payload
- `MIN-SF7-BW125-T80`: SF7, BW125kHz, 80-byte payload

### MEAN Configurations (SF9 - Balanced)
- `MEAN-SF9-BW125-T20`: SF9, BW125kHz, 20-byte payload
- `MEAN-SF9-BW500-T20`: SF9, BW500kHz, 20-byte payload
- `MEAN-SF9-BW125-T50`: SF9, BW125kHz, 50-byte payload
- `MEAN-SF9-BW125-T80`: SF9, BW125kHz, 80-byte payload

### MAX Configurations (SF12 - Long Range, Slow)
- `MAX-SF12-BW125-T20`: SF12, BW125kHz, 20-byte payload
- `MAX-SF12-BW500-T20`: SF12, BW500kHz, 20-byte payload
- `MAX-SF12-BW125-T50`: SF12, BW125kHz, 50-byte payload
- `MAX-SF12-BW125-T80`: SF12, BW125kHz, 80-byte payload

### EXTRA Configurations (Additional Testing)
- `EXTRA-SF8-BW125-T30`: SF8, BW125kHz, 30-byte payload
- `EXTRA-SF10-BW125-T40`: SF10, BW125kHz, 40-byte payload  
- `EXTRA-SF11-BW125-T60`: SF11, BW125kHz, 60-byte payload
- `EXTRA-SF7-BW500-T100`: SF7, BW500kHz, 100-byte payload

## Hardware Setup

### Required Components
- Arduino (Uno, Nano, Pro Mini, etc.)
- SX126X, SX127X, or SX128X LoRa module
- DS18B20 temperature sensor
- 4.7kΩ pull-up resistor for DS18B20

### Pin Connections

#### LoRa Module (SX127X example)
```
Arduino Pin -> LoRa Module
10          -> NSS (Chip Select)
4           -> NRESET (Reset)
MOSI (11)   -> MOSI
MISO (12)   -> MISO  
SCK (13)    -> SCK
```

#### DS18B20 Sensor
```
Arduino Pin 3 -> DS18B20 Data Pin
5V/3.3V      -> DS18B20 VCC
GND          -> DS18B20 GND
4.7kΩ resistor between VCC and Data Pin
```

## Remote Commands

### Configuration Command
Change network parameters remotely:

**Format**: `/@C<index>#`

**Examples**:
- `/@C0#` - Switch to configuration 0 (MIN-SF7-BW125-T20)
- `/@C4#` - Switch to configuration 4 (MEAN-SF9-BW125-T20)  
- `/@C8#` - Switch to configuration 8 (MAX-SF12-BW125-T20)
- `/@C15#` - Switch to configuration 15 (EXTRA-SF7-BW500-T100)

**Valid Range**: 0-12 (configurations 13-15 are reserved)

### Node Address Command (Non-LoRaWAN mode)
Change device address:

**Format**: `/@A<address>#`

**Examples**:
- `/@A10#` - Set node address to 10
- `/@A25#` - Set node address to 25

**Valid Range**: 2-255 (0=broadcast, 1=gateway reserved)

## EEPROM Configuration

The sketch automatically saves the following to EEPROM when changed via downlink:

```cpp
struct sx1272config {
  uint8_t flag1;                    // Validation flag (0x12)
  uint8_t flag2;                    // Validation flag (0x35)  
  uint8_t seq;                      // Packet sequence number
  uint8_t addr;                     // Node address
  unsigned int idle_period;         // Transmission interval
  uint8_t overwrite;               // Configuration change flag
  uint8_t current_config_index;    // Active configuration index
};
```

### Configuration Reset

To reset to default configuration:

1. Flash board with `FORCE_DEFAULT_VALUE` uncommented
2. Wait 10 seconds for boot
3. Flash again with `FORCE_DEFAULT_VALUE` commented out

## Custom Receive Window Timing

The sketch implements adaptive receive window delays based on spreading factor:

```cpp
uint16_t getAdaptiveDelay(uint8_t currentParamIndex) {
  uint8_t currentSF = testParams[currentParamIndex].sf;
  
  switch(currentSF) {
    case LORA_SF7:  return 500;   // Fast response
    case LORA_SF8:  return 600;
    case LORA_SF9:  return 700;
    case LORA_SF10: return 800;
    case LORA_SF11: return 900;
    case LORA_SF12: return 1000;  // Slower response
  }
}
```

## Payload Format

Temperature data is transmitted in the format:
```
\!TC/<temperature>
```

The payload is automatically padded to match the target size of the current configuration using null bytes (0x00).

## Compilation Options

### Chip Selection
Choose your LoRa chip by uncommenting one option:
```cpp
//#define SX126X
#define SX127X    // <- Uncomment for SX127X
//#define SX128X
```

### Feature Flags
```cpp
#define WITH_EEPROM    // Enable EEPROM persistence
#define WITH_ACK       // Request acknowledgments
#define WITH_RCVW      // Enable receive windows
#define WITH_APPKEY    // Enable 4-byte app key filtering
```

## Usage Example

1. **Initial Setup**: Flash the sketch to your Arduino with default configuration
2. **Remote Configuration**: Send `/@C5#` to switch to MEAN-SF9-BW500-T20 configuration
3. **Verify Change**: Device will update LoRa parameters and save to EEPROM
4. **Temperature Monitoring**: Device continues sending temperature data with new parameters
5. **Persistence**: Configuration survives reboot and continues with saved settings

## Extending the Template

To adapt this template for other sensors:

1. **Modify Sensor Interface**: Update `my_temp_sensor_code.h` for your sensor
2. **Adjust Payload Format**: Change the message format in main loop
3. **Add Custom Commands**: Extend `DownlinkParser.h` with new command handlers
4. **Configure Parameters**: Modify `NetworkParams.h` for your use case

## Integration with LoRa Gateway

This sketch is compatible with Congduc Pham's Low-Cost LoRa Gateway. The gateway can:

- Receive temperature data in the standard format
- Send downlink commands during receive windows
- Log configuration changes and device responses
- Forward data to cloud platforms



## Troubleshooting

### Common Issues

**No Downlink Reception**:
- Check receive window timing
- Verify gateway transmission timing
- Ensure matching LoRa parameters between device and gateway

**EEPROM Not Saving**:
- Verify `WITH_EEPROM` is defined
- Check for successful command parsing
- Ensure sufficient delay after EEPROM writes

**Sensor Reading Errors**:
- Verify DS18B20 connections and pull-up resistor
- Check power supply stability
- Confirm pin definitions match hardware setup

**Configuration Not Changing**:
- Verify command format: `/@C<index>#`
- Check index is within valid range (0-12)
- Ensure receive window is properly timed

### Debug Output

The sketch provides detailed serial output for debugging:
- LoRa parameter settings
- Received downlink commands
- Configuration changes
- Temperature readings
- EEPROM operations

## License

This code is based on the SX12XX library examples and Congduc Pham's Low-Cost LoRa Gateway project. Please refer to the original project licenses for usage terms.
