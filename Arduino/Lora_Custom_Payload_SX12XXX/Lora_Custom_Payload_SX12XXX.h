////////////////////////////////////////////////////////////////
// LoRa Interactive Message Sender - Header File
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
//LoRa Modem Parameters - optimized for interactive messaging
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

// Interactive messaging configuration
#define MAX_MSG_LEN 64                          // Maximum message length
#define SERIAL_BAUD 115200                      // Serial monitor baud rate
#define ECHO_CHARACTERS true                    // Echo typed characters back
#define SHOW_DETAILED_INFO true                 // Show transmission details

/*
 * Enhanced LoRa Interactive Messaging Features:
 * 
 * INTERACTIVE COMMUNICATION:
 * - Real-time message input via Serial Monitor
 * - Character-by-character echo and backspace support
 * - Message length validation and feedback
 * - Immediate transmission upon Enter key
 * - User-friendly prompts and status messages
 * 
 * MODULAR CHIP SUPPORT:
 * - SX126X: Next-generation LoRa chips (SX1261, SX1262, SX1268)
 * - SX127X: Popular LoRa chips (SX1272, SX1276, SX1277, SX1278, SX1279)
 * - SX128X: 2.4GHz LoRa chips (SX1280, SX1281)
 * - Easy switching between chip families
 * 
 * PROFESSIONAL FEATURES:
 * - Gateway-compatible data format
 * - Carrier sense before transmission
 * - Comprehensive board detection
 * - CRC calculation and verification
 * - ACK support for reliable messaging
 * - Transmission status monitoring
 * 
 * COMMUNICATION FEATURES:
 * - Point-to-point LoRa messaging
 * - Broadcast messaging capability
 * - Packet sequence numbering
 * - Signal quality reporting (SNR, RSSI)
 * - Transmission timing information
 * 
 * Pin usage summary for this project:
 * 
 * LoRa Module (SX127X example):
 * - NSS: Pin 10 (SPI Chip Select)
 * - NRESET: Pin 4 (Reset)
 * - DIO0: Used in polling mode (-1)
 * - MOSI: Pin 11 (SPI)
 * - MISO: Pin 12 (SPI)
 * - SCK: Pin 13 (SPI)
 * - VCC: 3.3V
 * - GND: Ground
 * 
 * Serial Connection:
 * - Open Serial Monitor at 115200 baud
 * - Type messages and press Enter to send
 * - Use backspace to edit messages
 * - Maximum 63 characters per message
 * 
 * CONFIGURATION OPTIONS:
 * - node_addr: Source address for messages (default: 100)
 * - dest_addr: Destination address (default: 1 for gateway)
 * - Enable/disable features with #define statements:
 *   * WITH_ACK: Request acknowledgments for reliable delivery
 *   * MY_FREQUENCY: Custom frequency override
 *   * PUBLIC_SYNCWORD: For LoRaWAN gateway compatibility
 *   * ECHO_CHARACTERS: Echo typed characters
 *   * SHOW_DETAILED_INFO: Verbose transmission details
 * 
 * USAGE SCENARIOS:
 * - Remote terminal/command interface
 * - Emergency messaging system
 * - Sensor data collection with manual override
 * - Educational LoRa communication demonstrations
 * - Point-to-point chat applications
 * - Gateway testing and debugging
 * 
 * MESSAGE FORMAT:
 * - Plain text messages up to 63 characters
 * - Automatic packet header with source/destination
 * - CRC protection for data integrity
 * - Optional acknowledgment mechanism
 * - Real-time transmission status feedback
 */