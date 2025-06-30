/*
 *  Enhanced DS18B20 Temperature Sensor with LoRa transmission
 *  Downlink-Controlled Network Characterization Version
 *  Using SX12XX library for LoRa communication
 *  
 *  Supports 16 network parameter configurations triggered by downlink commands:
 *  Command format: /@C<index># where index is 0-15
 *  
 *  Hardware connections:
 *  - DS18B20 data pin -> Arduino Pin 3
 *  - LoRa module as defined in Lora_DS18B20_SX12XXX.h
 */

#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>


#define USE_SPI_TRANSACTION          //this is the standard behaviour of library, use SPI Transaction switching

// please uncomment only 1 choice 
//#define SX126X
#define SX127X
//#define SX128X

#ifdef SX126X
#include <SX126XLT.h>
#include <SX126X_RadioSettings.h>
SX126XLT LT;                                          
#endif

#ifdef SX127X
#include <SX127XLT.h>
#include <SX127X_RadioSettings.h>
//#include "Lora_DS18B20_SX12XXX_Enhanced.h" 
SX127XLT LT;                                          
#endif

#ifdef SX128X
#include <SX128XLT.h>
#include <SX128X_RadioSettings.h>
SX128XLT LT;                                         
#endif

// Print macros for Arduino compatibility
#define PRINTLN                   Serial.println("")
#define PRINT_CSTSTR(param)       Serial.print(F(param))
#define PRINTLN_CSTSTR(param)     Serial.println(F(param))
#define PRINT_STR(fmt,param)      Serial.print(param)
#define PRINTLN_STR(fmt,param)    Serial.println(param)
#define PRINT_VALUE(fmt,param)    Serial.print(param)
#define PRINTLN_VALUE(fmt,param)  Serial.println(param)
#define PRINT_HEX(fmt,param)      Serial.print(param,HEX)
#define PRINTLN_HEX(fmt,param)    Serial.println(param,HEX)
#define FLUSHOUTPUT               Serial.flush()



// Configuration from Arduino_LoRa_SX12XX_DS18B20 sketch
//#define WITH_EEPROM
#define WITH_APPKEY
#define WITH_ACK
#define WITH_RCVW
#define INVERTIQ_ON_RX

#define ONE_WIRE_BUS 3                          // DS18B20 data pin (avoiding conflicts with LoRa pins)
#define MY_FREQUENCY 868000000



// DS18B20 Temperature Sensor setup
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Global variables
uint8_t currentParamIndex = 0;                       // Current parameter set index (0-15)
uint8_t node_addr = 8;                              // Node address
const uint8_t idlePeriodInMin = 1;                 // Transmission interval
unsigned long nextTransmissionTime = 0;             // Next transmission time

// Message buffer
uint8_t message[100];

// Packet counters
uint32_t TXPacketCount = 0;

#ifdef WITH_EEPROM
#include <EEPROM.h>

struct sx1272config {
  uint8_t flag1;
  uint8_t flag2;
  uint8_t seq;
  uint8_t addr;
  unsigned int idle_period;  
  uint8_t overwrite;
  uint8_t current_config_index;  // Store current configuration index
};

sx1272config my_sx1272config;
#endif

//#include "Lora_DS18B20_SX12XXX_Enhanced.h"  // Include header with pin definitions
#include "NetworkParams.h"          // Include network parameter definitions
#include "DownlinkParser.h"         // Include downlink command parsing

// Simple float to string conversion to save memory
char *ftoa(char *a, double f, int precision)
{
 long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
 
 char *ret = a;
 long heiltal = (long)f;
 itoa(heiltal, a, 10);
 while (*a != '\0') a++;
 *a++ = '.';
 long desimal = abs((long)((f - heiltal) * p[precision]));
 if (desimal < p[precision-1]) {
  *a++ = '0';
 } 
 itoa(desimal, a, 10);
 return ret;
}

// Function to create padded payload to reach target size
void createPaddedPayload(char* dest, float temperature, uint8_t targetSize) {
  char float_str[10];
  ftoa(float_str, temperature, 2);
  
  // Start with basic temperature format: \!TC/temp
  int baseSize = sprintf(dest, "\\!TC/%s", float_str);
  
  // If we need more bytes, pad with spaces
  if (targetSize > baseSize) {
    for (int i = baseSize; i < targetSize; i++) {
      // dest[i] = ' ';  // Fill with spaces
      dest[i] = 0x00;  // Fill with null bytes
    }
    // dest[targetSize] = '\0';  // Null terminate
  }
}

void setup()
{
  delay(1000);
  
  Serial.begin(38400);
  while (!Serial);
  
  PRINTLN_CSTSTR("Enhanced DS18B20 LoRa Downlink-Controlled Network Characterization");
  PRINTLN_CSTSTR("Supports 16 configurations via downlink commands /@C<index>#");
  
  SPI.begin();

  // Initialize DS18B20 sensor
  sensors.begin();
  
  PRINT_CSTSTR("Found ");
  PRINT_VALUE("%d", sensors.getDeviceCount());
  PRINTLN_CSTSTR(" temperature sensor(s)");

  // Initialize LoRa device
#ifdef SX126X
  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, SW, LORA_DEVICE))
#endif
#ifdef SX127X
  if (LT.begin(NSS, NRESET, DIO0, DIO1, DIO2, LORA_DEVICE))
#endif
#ifdef SX128X
  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
#endif
  {
    PRINTLN_CSTSTR("LoRa Device found");
  }
  else
  {
    PRINTLN_CSTSTR("No LoRa device responding");
    while (1) { }
  }

  // Configure LoRa parameters
  loraConfig();

  // Print modem settings for debugging
  PRINTLN;
  LT.printModemSettings();                                     
  PRINTLN;
  LT.printOperatingSettings();                                 
  PRINTLN;

#ifdef WITH_EEPROM
  EEPROM.get(0, my_sx1272config);
  
  if (my_sx1272config.flag1==0x12 && my_sx1272config.flag2==0x34) {
    PRINT_CSTSTR("Get back previous sx1272 config\n");
    LT.setTXSeqNo(my_sx1272config.seq);
    node_addr = my_sx1272config.addr;
    currentParamIndex = my_sx1272config.current_config_index;
    
    PRINT_CSTSTR("Using packet sequence number of ");
    PRINT_VALUE("%d", LT.readTXSeqNo());
    PRINTLN;
    PRINT_CSTSTR("Restored node address: ");
    PRINT_VALUE("%d", node_addr);
    PRINTLN;
    PRINT_CSTSTR("Restored configuration index: ");
    PRINT_VALUE("%d", currentParamIndex);
    PRINTLN;
  }
  else {
    my_sx1272config.flag1=0x12;
    my_sx1272config.flag2=0x34;
    my_sx1272config.seq=LT.readTXSeqNo();
    my_sx1272config.addr=node_addr;
    my_sx1272config.current_config_index=currentParamIndex;
  }
#endif

  PRINT_CSTSTR("Setting Power: ");
  PRINTLN_VALUE("%d", MAX_DBM); 

  LT.setDevAddr(node_addr);
  PRINT_CSTSTR("Node address: ");
  PRINT_VALUE("%d", node_addr);
  PRINTLN;  
  
#ifdef SX126X
  PRINT_CSTSTR("SX126X - ");
#endif
#ifdef SX127X
  PRINT_CSTSTR("SX127X - ");
#endif
#ifdef SX128X
  PRINT_CSTSTR("SX128X - ");
#endif

  PRINTLN_CSTSTR("Downlink-Controlled Network Characterization Ready");
  
  // Initialize with current parameter set
  updateLoRaParams(testParams[currentParamIndex]);
  nextTransmissionTime = millis() + 5000; // First transmission in 5 seconds
  
  delay(500);
}

void loop()
{
  long startSend;
  long endSend;
  float tempC;
  bool sensorError = false;

  // Check if it's time for next transmission
  if (millis() >= nextTransmissionTime) {
    
    // Read temperature from DS18B20 sensor
    PRINTLN_CSTSTR("Reading temperature...");
/* 
    // Take multiple readings for accuracy
    tempC = 0.0;
    for (int i=0; i<3; i++) {
      sensors.requestTemperatures(); 
      float reading = sensors.getTempCByIndex(0);
      if (reading == DEVICE_DISCONNECTED_C) {
        sensorError = true;
        break;
      }
      tempC += reading;
      delay(100);
    }
      
    if (!sensorError) {
      tempC = tempC/3;
      PRINT_CSTSTR("Temperature: ");
      PRINT_VALUE("%.2f", tempC);
      PRINTLN_CSTSTR("°C");
    } else {
      PRINTLN_CSTSTR("Sensor error - using test value");
      tempC = 24.21;  // Test value when sensor disconnected
    }
*/ 
    tempC = 24.21;  // Test value when sensor disconnected

    // Create payload with target size from current configuration
    char payloadStr[100];
    createPaddedPayload(payloadStr, tempC, testParams[currentParamIndex].payloadSize);
    
    uint8_t len = strlen(payloadStr);
    
    PRINT_CSTSTR("Sending: ");
    PRINT_STR("%s", payloadStr);
    PRINT_CSTSTR(" (");
    PRINT_VALUE("%d", len);
    PRINTLN_CSTSTR(" bytes)");
    PRINTLN;
    
    startSend = millis();
    
    // Send the packet, return packet length sent if OK, otherwise 0 if transmit error
#ifdef WITH_ACK
    if (LT.transmitAddressed((uint8_t*)payloadStr, len, PKT_TYPE_DATA | PKT_FLAG_ACK_REQ, DEFAULT_DEST_ADDR, node_addr, 10000, MAX_DBM, WAIT_TX))
#else
    if (LT.transmitAddressed((uint8_t*)payloadStr, len, PKT_TYPE_DATA, DEFAULT_DEST_ADDR, node_addr, 10000, MAX_DBM, WAIT_TX))
#endif
    {
      endSend = millis();
      TXPacketCount++;
      
      PRINTLN;
      PRINT_CSTSTR("Sent OK - ");
      PRINT_VALUE("%d", len);
      PRINT_CSTSTR(" bytes in ");
      PRINT_VALUE("%ld", endSend - startSend);
      PRINTLN_CSTSTR("ms");

#ifdef WITH_ACK
      if (LT.readAckStatus()) {
        PRINT_CSTSTR("Received ACK from gateway ");
        PRINT_VALUE("%d", LT.readRXSource());
        PRINTLN;
        PRINT_CSTSTR("SNR of transmitted pkt is ");
        PRINT_VALUE("%d", LT.readPacketSNRinACK());
        PRINTLN; PRINTLN;          
      }
      else {
        PRINTLN;
        PRINTLN_CSTSTR("No ACK received");
      }
#endif

      ///////////////////////////////////////////////////////////////////
      // DOWNLINK RECEPTION BLOCK
      ///////////////////////////////////////////////////////////////////
#ifdef WITH_RCVW
      uint8_t RXPacketL = processDownlinkWindow(endSend);
      
      if (RXPacketL) {
        // Process the received downlink command
        bool configChanged = parseDownlinkCommand(message, RXPacketL, currentParamIndex, node_addr);
        
        if (configChanged) {
          // Update LoRa parameters if configuration was changed
          updateLoRaParams(testParams[currentParamIndex]);
          delay(30000);
          
#ifdef WITH_EEPROM
          // Save new configuration index to EEPROM
          my_sx1272config.current_config_index = currentParamIndex;
          my_sx1272config.addr = node_addr;
          EEPROM.put(0, my_sx1272config);
#endif
        }
      }
#endif

#ifdef WITH_EEPROM
      // Save packet number for next packet in case of reboot     
      my_sx1272config.seq = LT.readTXSeqNo();
      EEPROM.put(0, my_sx1272config);
#endif
    }
    else
    {
      endSend = millis();
      //if here there was an error transmitting packet
      uint16_t IRQStatus;
      IRQStatus = LT.readIrqStatus();
      PRINTLN;
      PRINT_CSTSTR("SendError,");
      PRINT_CSTSTR(",IRQreg,");
      PRINT_HEX("%d", IRQStatus);
      LT.printIrqStatus(); 
    }
    
    // Set next transmission time
    nextTransmissionTime = millis() + (unsigned long)idlePeriodInMin * 20 * 1000;
    
    PRINT_CSTSTR("Next transmission in ");
    PRINT_VALUE("%d", idlePeriodInMin * 20);
    PRINTLN_CSTSTR(" s");
    PRINTLN;
  }
  
  delay(100);
}