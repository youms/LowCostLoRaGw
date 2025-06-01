////////////////////////////////////////////////////////////////
// Combined DS18B20 Temperature + MQ2 Multi-Gas Sensor - Header File
// Enhanced environmental monitoring station with SX12XXX library support
// Compatible with Congduc Pham's Low-Cost LoRa Gateway
////////////////////////////////////////////////////////////////

#ifdef SX126X
//*******  Setup hardware pin definitions here ! ***************
#define NSS 10                                  //select pin on LoRa device
#define NRESET 4                                //reset pin on LoRa device
#define RFBUSY 5                                //busy pin on LoRa device 
#define DIO1 -1                                 //DIO1 pin on LoRa device, used for RX and TX done
#define DIO2 -1                                 //DIO2 pin on LoRa device, normally not used so set to -1 
#define DIO3 -1                                 //DIO3 pin on LoRa device, normally not used so set to -1
#define RX_EN -1                                //pin for RX enable, used on some SX126X devices, set to -1 if not used
#define TX_EN -1                                //pin for TX enable, used on some SX126X devices, set to -1 if not used 
#define SW -1                                   //SW pin on some Dorji LoRa devices, used to power antenna switch, set to -1 if not used

#define LORA_DEVICE DEVICE_SX1262               //we need to define the device we are using

//*******  Setup LoRa Parameters Here ! ***************
//LoRa Modem Parameters
const uint32_t DEFAULT_CHANNEL = 865200000;     //frequency of transmissions in hertz
const uint32_t Offset = 0;                      //offset frequency for calibration purposes
uint8_t SpreadingFactor = LORA_SF12;            //LoRa spreading factor
uint8_t Bandwidth = LORA_BW_125;                //LoRa bandwidth
uint8_t CodeRate = LORA_CR_4_5;                 //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting, normally set to auto                               
#endif

////////////////////////////////////////////////////////////////
#ifdef SX127X
//*******  Setup hardware pin definitions here ! ***************

#define NSS 10                                  //select pin on LoRa device
#define NRESET 4                                //reset pin on LoRa device
#define DIO0 -1                                 //DIO0 pin on LoRa device, used for RX and TX done (polling mode)
#define DIO1 -1                                 //DIO1 pin on LoRa device, normally not used so set to -1 
#define DIO2 -1                                 //DIO2 pin on LoRa device, normally not used so set to -1

#define LORA_DEVICE DEVICE_SX1276               //we need to define the device we are using

//*******  Setup LoRa Parameters Here ! ***************
//LoRa Modem Parameters - optimized for sensor data transmission
const uint32_t DEFAULT_CHANNEL = 865200000;     //frequency of transmissions in hertz
const uint32_t Offset = 0;                      //offset frequency for calibration purposes
uint8_t SpreadingFactor = LORA_SF12;            //LoRa spreading factor (long range)
uint8_t Bandwidth = LORA_BW_125;                //LoRa bandwidth
uint8_t CodeRate = LORA_CR_4_5;                 //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting, normally set to auto

#define PA_BOOST true                           //enable PA_BOOST for higher power output
#endif

////////////////////////////////////////////////////////////////
#ifdef SX128X
//*******  Setup hardware pin definitions here ! ***************
#define NSS 10
#define NRESET 4
#define RFBUSY 3                                //busy pin on LoRa device 
#define DIO1 -1                                 //DIO1 pin on LoRa device, used for RX and TX done
#define DIO2 -1                 				//not used 
#define DIO3 -1                 				//not used                      
#define RX_EN -1                				//pin for RX enable, used on some SX1280 devices, set to -1 if not used
#define TX_EN -1                				//pin for TX enable, used on some SX1280 devices, set to -1 if not used      

#define LORA_DEVICE DEVICE_SX1280               //we need to define the device we are using 

//*******  Setup LoRa Parameters Here ! ***************
//LoRa Modem Parameters
const uint32_t DEFAULT_CHANNEL = 2403000000;    //frequency of transmissions in hertz
const uint32_t Offset = 0;                      //offset frequency for calibration purposes  
uint8_t SpreadingFactor = LORA_SF12;            //LoRa spreading factor
uint8_t Bandwidth = LORA_BW_0200;               //LoRa bandwidth
uint8_t CodeRate = LORA_CR_4_5;                 //LoRa coding rate                         

#define MAX_DBM                  10                 
#endif

#ifndef MAX_DBM
#define MAX_DBM                 14
#endif

#define DEFAULT_DEST_ADDR       1

// DS18B20 Temperature Sensor pin definitions
#define ONE_WIRE_BUS 3                          // DS18B20 data pin (avoiding conflicts with LoRa pins)

// MQ2 Gas Sensor pin definitions and parameters
#define MQ2_PIN A0                              // MQ2 sensor AO pin
#define WARMUP_DELAY 20000                      // 20 seconds for MQ2 sensor to warm up
#define RL_VALUE 10000                          // Load resistance in ohms (10kΩ)
#define VCC 5.0                                 // Circuit voltage

// Calibration values (should be measured in clean air after 24-48h preheating)
extern float R0_LPG;                           // Baseline resistance for LPG in clean air
extern float R0_METHANE;                       // Baseline resistance for Methane in clean air
extern float R0_CO;                            // Baseline resistance for CO in clean air

/*
 * Enhanced Combined Environmental LoRa Sensor Features:
 * 
 * DUAL SENSOR MONITORING:
 * - DS18B20: Precision digital temperature sensing (-55°C to +125°C)
 * - MQ2: Multi-gas detection (LPG, Methane, CO, H2, Alcohol, Propane, Smoke)
 * - Comprehensive environmental monitoring
 * - Cross-sensor correlation and alerts
 * - Graceful degradation (continues if one sensor fails)
 * 
 * ENVIRONMENTAL DETECTION CAPABILITIES:
 * - Temperature monitoring with freeze/heat warnings
 * - Fire/combustible gas detection
 * - Air quality assessment
 * - Carbon monoxide safety monitoring
 * - Alcohol vapor detection
 * - Comprehensive safety alerting system
 * 
 * LOW POWER FEATURES:
 * - Supports multiple sleep modes (AVR, Teensy, ESP8266, SAMD21)
 * - Optimized for environmental monitoring intervals
 * - LoRa radio sleep mode between transmissions
 * - Sensor power management and coordination
 * - Extended battery life for remote deployment
 * 
 * RELIABILITY FEATURES:
 * - EEPROM storage for packet sequence numbers (survives reboot)
 * - Optional ACK requests from gateway
 * - Multiple sensor readings averaged for accuracy
 * - Individual sensor error handling and recovery
 * - CRC calculation and verification
 * - Redundant sensor validation
 * - Environmental correlation analysis
 * 
 * PROFESSIONAL FEATURES:
 * - Gateway-compatible data format (\!TC/temp/LPG/value/CH4/value...)
 * - Carrier sense before transmission
 * - Comprehensive board detection
 * - Memory-efficient float to string conversion
 * - Configurable transmission intervals
 * - Multi-parameter environmental packet
 * - Safety threshold monitoring and alerting
 * - Raw ADC value reporting for diagnostics
 * 
 * Pin usage summary for this project:
 * 
 * DS18B20 Temperature Sensor:
 * - Data pin: Arduino Pin 3 (ONE_WIRE_BUS)
 * - VCC: 3.3V or 5V
 * - GND: Ground
 * 
 * MQ2 Gas Sensor:
 * - Analog pin: A0 (MQ2_PIN)
 * - VCC: 5V (sensor requires 5V operation)
 * - GND: Ground
 * - AO: Analog output to A0
 * - DO: Digital output (not used)
 * 
 * LoRa Module (SX127X example):
 * - NSS: Pin 10 (SPI Chip Select)
 * - NRESET: Pin 4 (Reset)
 * - MOSI: Pin 11 (SPI)
 * - MISO: Pin 12 (SPI)
 * - SCK: Pin 13 (SPI)
 * - VCC: 3.3V
 * - GND: Ground
 * 
 * CALIBRATION PROCEDURE:
 * 1. DS18B20: Usually factory calibrated, no user calibration needed
 * 2. MQ2: Power sensor for 24-48 hours in clean air for proper aging
 * 3. Record ADC values in clean air for R0 calibration
 * 4. Update R0 values in code for accurate gas concentration calculations
 * 5. Test with known concentrations for verification
 * 
 * ENVIRONMENTAL THRESHOLDS:
 * Temperature:
 * - <0°C: FREEZE warning
 * - >35°C: HIGH warning  
 * - >50°C: HOT warning
 * 
 * Gas Safety (PPM):
 * - CO: >50 HIGH, >200 DANGER
 * - LPG: >1000 HIGH, >5000 DANGER
 * - Methane: >1000 HIGH, >5000 DANGER
 * - Alcohol: >1000 HIGH
 * 
 * CONFIGURATION OPTIONS:
 * - node_addr: Change for each sensor node (default: 14)
 * - idlePeriodInMin: Transmission interval in minutes (default: 8 for environmental monitoring)
 * - Enable/disable features with #define statements:
 *   * WITH_EEPROM: Persistent packet counters
 *   * LOW_POWER: Sleep modes for battery operation
 *   * WITH_ACK: Request acknowledgments from gateway
 *   * MY_FREQUENCY: Custom frequency override
 *   * PUBLIC_SYNCWORD: For LoRaWAN gateway compatibility
 *   * ENVIRONMENTAL_ALERTS: Enhanced safety monitoring
 */