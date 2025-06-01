////////////////////////////////////////////////////////////////
// MQ2 Multi-Gas Sensor with LoRa - Header File
// Enhanced version with SX12XXX library support
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

// MQ2 Gas Sensor pin definitions and parameters
#define MQ2_PIN A0                              // Arduino's pin connected to AO pin of the MQ2 sensor
#define WARMUP_DELAY 20000                      // 20 seconds for MQ2 sensor to warm up
#define RL_VALUE 10000                          // Load resistance in ohms (10kΩ)
#define VCC 5.0                                 // Circuit voltage

// Calibration values (should be measured in clean air after 24-48h preheating)
extern float R0_LPG;                           // Baseline resistance for LPG in clean air
extern float R0_METHANE;                       // Baseline resistance for Methane in clean air
extern float R0_CO;                            // Baseline resistance for CO in clean air

/*
 * Enhanced MQ2 Multi-Gas LoRa Sensor Features:
 * 
 * GAS DETECTION CAPABILITIES:
 * - LPG (Liquefied Petroleum Gas)
 * - Methane (CH4)
 * - Carbon Monoxide (CO)
 * - Hydrogen (H2)
 * - Alcohol
 * - Propane
 * - Smoke detection
 * - Real-time safety alerts
 * 
 * LOW POWER FEATURES:
 * - Supports multiple sleep modes (AVR, Teensy, ESP8266, SAMD21)
 * - Configurable sleep periods (shorter for safety monitoring)
 * - LoRa radio sleep mode between transmissions
 * - Sensor averaging for power efficiency
 * 
 * RELIABILITY FEATURES:
 * - EEPROM storage for packet sequence numbers (survives reboot)
 * - Optional ACK requests from gateway
 * - Multiple sensor readings averaged for accuracy
 * - Error handling and validation
 * - CRC calculation and verification
 * - Gas concentration calculations in PPM
 * - Safety threshold monitoring
 * 
 * PROFESSIONAL FEATURES:
 * - Gateway-compatible data format (\!LPG/value/CH4/value/CO/value...)
 * - Carrier sense before transmission
 * - Comprehensive board detection
 * - Memory-efficient float to string conversion
 * - Configurable transmission intervals
 * - Multi-gas simultaneous monitoring
 * - Safety alert generation
 * - Raw ADC value reporting for diagnostics
 * 
 * Pin usage summary for this project:
 * 
 * MQ2 Gas Sensor:
 * - Analog pin: A0 (MQ2_PIN)
 * - VCC: 5V (sensor requires 5V operation)
 * - GND: Ground
 * - AO: Analog output to A0
 * - DO: Digital output (not used in this implementation)
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
 * 1. Power the sensor for 24-48 hours in clean air for proper aging
 * 2. Record ADC values in clean air for R0 calibration
 * 3. Update R0 values in the code for accurate gas concentration calculations
 * 4. Test with known gas concentrations for verification
 * 
 * SAFETY THRESHOLDS (PPM):
 * - CO: >50 HIGH, >200 DANGER
 * - LPG: >1000 HIGH, >5000 DANGER  
 * - Methane: >1000 HIGH, >5000 DANGER
 * - Alcohol: >1000 HIGH
 * 
 * CONFIGURATION OPTIONS:
 * - node_addr: Change for each sensor node (default: 13)
 * - idlePeriodInMin: Transmission interval in minutes (default: 5 for gas monitoring)
 * - Enable/disable features with #define statements:
 *   * WITH_EEPROM: Persistent packet counters
 *   * LOW_POWER: Sleep modes for battery operation
 *   * WITH_ACK: Request acknowledgments from gateway
 *   * MY_FREQUENCY: Custom frequency override
 *   * PUBLIC_SYNCWORD: For LoRaWAN gateway compatibility
 *   * MQ2_ADVANCED_CALIBRATION: Individual gas calibration
 */