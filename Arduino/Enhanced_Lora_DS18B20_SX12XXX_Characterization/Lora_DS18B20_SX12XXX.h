////////////////////////////////////////////////////////////////
// DS18B20 Temperature Sensor with LoRa - Header File
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

//NOTE: DS18B20 is connected to pin 3, so pin 4 is free for LoRa NRESET
#define NSS 10                                  //select pin on LoRa device
#define NRESET 4                                //reset pin on LoRa device (DS18B20 moved to pin 3)
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

// DS18B20 Temperature Sensor pin definition
#define ONE_WIRE_BUS 3                          // DS18B20 data pin (avoiding conflicts with LoRa pins)

/*
 * Enhanced DS18B20 LoRa Sensor Features:
 * 
 * LOW POWER FEATURES:
 * - Supports multiple sleep modes (AVR, Teensy, ESP8266, SAMD21)
 * - Configurable sleep periods (8s intervals for AVR, 60s for others)
 * - LoRa radio sleep mode between transmissions
 * - Power management for sensor readings
 * 
 * RELIABILITY FEATURES:
 * - EEPROM storage for packet sequence numbers (survives reboot)
 * - Optional ACK requests from gateway
 * - Multiple temperature readings averaged for accuracy
 * - Error handling for sensor disconnection
 * - CRC calculation and verification
 * 
 * PROFESSIONAL FEATURES:
 * - Gateway-compatible data format (\!TC/temp)
 * - Carrier sense before transmission
 * - Comprehensive board detection
 * - Memory-efficient float to string conversion
 * - Configurable transmission intervals
 * 
 * Pin usage summary for this project:
 * 
 * DS18B20 Temperature Sensor:
 * - Data pin: Arduino Pin 3 (ONE_WIRE_BUS)
 * - VCC: 3.3V or 5V (can be powered from digital pin for low power)
 * - GND: Ground
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
 * CONFIGURATION OPTIONS:
 * - node_addr: Change for each sensor node (default: 8)
 * - idlePeriodInMin: Transmission interval in minutes (default: 10)
 * - Enable/disable features with #define statements:
 *   * WITH_EEPROM: Persistent packet counters
 *   * LOW_POWER: Sleep modes for battery operation
 *   * WITH_ACK: Request acknowledgments from gateway
 *   * MY_FREQUENCY: Custom frequency override
 *   * PUBLIC_SYNCWORD: For LoRaWAN gateway compatibility
 */