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

//#include <OneWire.h>
//#include <DallasTemperature.h>
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

#include "my_temp_sensor_code.h"
#include "my_gas_sensor_code.h"

///////////////////////////////////////////////////////////////////
// COMMENT THIS LINE IF YOU WANT TO DYNAMICALLY SET THE NODE'S ADDR 
// OR SOME OTHER PARAMETERS BY REMOTE RADIO COMMANDS (WITH_RCVW)
// LEAVE THIS LINE UNCOMMENTED IF YOU WANT TO USE THE DEFAULT VALUE
// AND CONFIGURE YOUR DEVICE BY CHANGING MANUALLY THESE VALUES IN 
// THE SKETCH.
//
// ONCE YOU HAVE FLASHED A BOARD WITHOUT FORCE_DEFAULT_VALUE, YOU 
// WILL BE ABLE TO DYNAMICALLY CONFIGURE IT AND SAVE THIS CONFIGU-
// RATION INTO EEPROM. ON RESET, THE BOARD WILL USE THE SAVED CON-
// FIGURATION.

// IF YOU WANT TO REINITIALIZE A BOARD, YOU HAVE TO FIRST FLASH IT 
// WITH FORCE_DEFAULT_VALUE, WAIT FOR ABOUT 10s SO THAT IT CAN BOOT
// AND FLASH IT AGAIN WITHOUT FORCE_DEFAULT_VALUE. THE BOARD WILL 
// THEN USE THE DEFAULT CONFIGURATION UNTIL NEXT CONFIGURATION.

//#define FORCE_DEFAULT_VALUE
///////////////////////////////////////////////////////////////////

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
#define WITH_EEPROM
//#define WITH_APPKEY
#define WITH_ACK
#define WITH_RCVW
#define INVERTIQ_ON_RX

//#define ONE_WIRE_BUS 3                          // DS18B20 data pin (avoiding conflicts with LoRa pins)
#define MY_FREQUENCY 868000000



// DS18B20 Temperature Sensor setup
// OneWire oneWire(ONE_WIRE_BUS);
// DallasTemperature sensors(&oneWire);

// Global variables
uint8_t currentParamIndex = 2;                      // Current parameter set index (0-15)
uint8_t node_addr = 30;                              // Node address
unsigned int idlePeriodInMin = 0;                   // Transmission interval
unsigned int idlePeriodInSec = 8;                  // Needed to obtain 15s sending difference, downlink wait times cause some delays
unsigned long nextTransmissionTime = 0;             // Next transmission time

// Scenario 2: Round Robin State Management
typedef enum {
  STATE_STANDBY,      // SF12, 865.2MHz, carrier sense ON
  STATE_MONITORING,   // SF7, 868.0MHz, carrier sense OFF
  STATE_POST_MONITOR_SLEEP // SF12, 865.2MHz, sleep for 10 minutes
} NodeState;

NodeState currentState = STATE_STANDBY;
uint32_t stateChangeTime = 0;
// uint32_t monitoringDuration = 5 * 60 * 1000;  // 5 minutes
// uint32_t postMonitorSleepDuration = 10 * 60 * 1000;  // 10 minutes sleep
bool carrierSenseEnabled = true;

// Scenario 2: Configurable mode parameters
uint8_t monitoringConfigIndex = 2;  // Default: SF7-BW125-T50
uint8_t standbyConfigIndex = 8;     // Default: SF12-BW125-T20 (can be changed)

// Scenario 2: Dynamic sleep calculation
uint8_t totalNodes = 3;                                    // Total nodes in round-robin
uint32_t monitoringPeriod = 5;           // 5 minutes monitoring
uint32_t calculateSleepDuration() {
  return (totalNodes - 1) * (monitoringPeriod + 3) * 60 * 1000; // add 3 minutes to match gateway
}

#ifdef WITH_APPKEY
///////////////////////////////////////////////////////////////////
// CHANGE HERE THE APPKEY, BUT IF GW CHECKS FOR APPKEY, MUST BE
// IN THE APPKEY LIST MAINTAINED BY GW.
uint8_t my_appKey[4]={5, 6, 7, 8};
///////////////////////////////////////////////////////////////////
#endif

///////////////////////////////////////////////////////////////////
// DO NOT CHANGE HERE
//unsigned char DevAddr[4] = { 0x00, 0x00, 0x00, node_addr };
///////////////////////////////////////////////////////////////////

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

// New overloaded function for combined sensor data
void createPaddedPayload(char* dest, float temperature, float gasCO, int gasLPG, 
                        int gasMethane, int gasPropane, int gasHydrogen, int gasSmoke, 
                        uint8_t targetSize) {
  char float_str[10];
  ftoa(float_str, temperature, 2);
  
  int baseSize;
  
  if (targetSize >= 50) {
    // T50 & T80: Full gas suite (fits comfortably in 50+ bytes)
    baseSize = sprintf(dest, "\\!TC/%s/CO/%d/LPG/%d/CH4/%d/C3H8/%d/H2/%d/SMK/%d", 
                      float_str, (int)gasCO, gasLPG, gasMethane, gasPropane, gasHydrogen, gasSmoke);
  } else if (targetSize >= 20) {
    // T20: Critical gases only (CO, LPG, CH4 - most important for safety)
    baseSize = sprintf(dest, "\\!TC/%s/CO/%d/LPG/%d/CH4/%d", 
                      float_str, (int)gasCO, gasLPG, gasMethane);
  } else {
    // Fallback: Temperature only (shouldn't happen with your 20/50/80 sizes)
    baseSize = sprintf(dest, "\\!TC/%s", float_str);
  }
  
  // Pad with null bytes to reach target size
  if (targetSize > baseSize) {
    for (int i = baseSize; i < targetSize; i++) {
      dest[i] = 0x00;
    }
  }
}

void setMonitoringMode() {
  PRINTLN_CSTSTR("Switching to MONITORING mode");
  currentParamIndex = monitoringConfigIndex;
  LT.setRfFrequency(868000000, Offset);  // Always 868.0 MHz
  updateLoRaParams(testParams[currentParamIndex]);
  carrierSenseEnabled = false;
  currentState = STATE_MONITORING;
  stateChangeTime = millis();
  
  PRINT_CSTSTR("MONITORING: ");
  Serial.print(testParams[currentParamIndex].name);
  PRINTLN_CSTSTR(" at 868.0MHz");
}

void setStandbyMode() {
  PRINTLN_CSTSTR("Switching to STANDBY mode");
  currentParamIndex = standbyConfigIndex;
  LT.setRfFrequency(865200000, Offset);  // Always 865.2 MHz
  updateLoRaParams(testParams[currentParamIndex]);
  carrierSenseEnabled = true;
  currentState = STATE_STANDBY;
  stateChangeTime = millis();
  
  PRINT_CSTSTR("STANDBY: ");
  Serial.print(testParams[currentParamIndex].name);
  PRINTLN_CSTSTR(" at 865.2MHz");
}

void setPostMonitorSleep() {
  PRINTLN_CSTSTR("Entering POST-MONITOR sleep");
  currentParamIndex = 8;  
  LT.setRfFrequency(865200000, Offset);
  updateLoRaParams(testParams[currentParamIndex]);
  carrierSenseEnabled = false;
  currentState = STATE_POST_MONITOR_SLEEP;
  
  uint32_t sleepDuration = calculateSleepDuration();
  PRINT_CSTSTR("Sleeping for ");
  PRINT_VALUE("%lu", sleepDuration / 60000);
  PRINTLN_CSTSTR(" minutes (waiting for other nodes)");
  
  delay(sleepDuration);  // Dynamic sleep based on network size
  
  PRINTLN_CSTSTR("Sleep complete, ready for next monitoring cycle");
  setStandbyMode();
}

void setup()
{
  digitalWrite(PIN_POWER,HIGH);

  delay(500);
  
  Serial.begin(38400);
  // while (!Serial);
  
  // PRINTLN_CSTSTR("Enhanced DS18B20 LoRa Downlink-Controlled Network Characterization");
  PRINTLN_CSTSTR("SCENARIO 2: Round Robin Frequency/SF Switching Ready");
  PRINTLN_CSTSTR("Supports 16 configurations via downlink commands /@C<index>#");
  
  SPI.begin();

  // Initialize DS18B20 sensor
  // sensors.begin();
  
  // PRINT_CSTSTR("Found ");
  // PRINT_VALUE("%d", ds18b20.getDeviceCount());
  // PRINTLN_CSTSTR(" temperature sensor(s)");

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
    delay(1000);
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

  /*
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
#endif */

#ifdef WITH_EEPROM

  // get config from EEPROM
  EEPROM.get(0, my_sx1272config);

  // found a valid config?
  if (my_sx1272config.flag1==0x12 && my_sx1272config.flag2==0x35) {
    PRINT_CSTSTR("Get back previous sx1272 config\n");
    // set sequence number for SX1272 library

#ifdef FORCE_DEFAULT_VALUE
    LT.setTXSeqNo(TXPacketCount);
#else
    LT.setTXSeqNo(my_sx1272config.seq);
#endif
    PRINT_CSTSTR("Using packet sequence number of ");
    PRINT_VALUE("%d", LT.readTXSeqNo());
    PRINTLN;    

#ifdef FORCE_DEFAULT_VALUE
    PRINT_CSTSTR("Forced to use default parameters\n");
    my_sx1272config.flag1=0x12;
    my_sx1272config.flag2=0x35;   
    my_sx1272config.seq=LT.readTXSeqNo(); 
    my_sx1272config.addr=node_addr;
    my_sx1272config.idle_period=idlePeriodInSec;
    my_sx1272config.current_config_index=currentParamIndex;
    // reset the overwrite flag    
    my_sx1272config.overwrite=0;
    EEPROM.put(0, my_sx1272config);
#else
    // get back the node_addr
    if (my_sx1272config.addr!=0 && my_sx1272config.overwrite==1) {
      
        PRINT_CSTSTR("Used stored address\n");
        node_addr=my_sx1272config.addr;        
    }
    else
        PRINT_CSTSTR("Stored node addr is null\n"); 

    // get back the idle period
    if (my_sx1272config.idle_period!=0 && my_sx1272config.overwrite==1) {
      
        PRINT_CSTSTR("Used stored idle period\n");
        idlePeriodInSec=my_sx1272config.idle_period;        
    }
    else
        PRINT_CSTSTR("Stored idle period is null\n");

    // get back the currentParamIndex
    if (my_sx1272config.current_config_index!=0 && my_sx1272config.overwrite==1) {
        PRINT_CSTSTR("Used stored configuration index\n");
        currentParamIndex=my_sx1272config.current_config_index;        
    }
    else
        PRINT_CSTSTR("Stored configuration index is null\n");
                 
#endif  
          
    PRINT_CSTSTR("Using node addr of ");
    PRINT_VALUE("%d", node_addr);
    PRINTLN;   

    PRINT_CSTSTR("Using idle period of ");
    PRINT_VALUE("%d", idlePeriodInSec);
    PRINTLN;     

    PRINT_CSTSTR("Using configuration index of ");
    PRINT_VALUE("%d", currentParamIndex);
    PRINTLN;
    
  }
  else {
    // otherwise, write config and start over
    my_sx1272config.flag1=0x12;
    my_sx1272config.flag2=0x35;  
    my_sx1272config.seq=LT.readTXSeqNo(); 
    my_sx1272config.addr=node_addr;
    my_sx1272config.idle_period=idlePeriodInSec;
    my_sx1272config.overwrite=0;
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

//  PRINTLN_CSTSTR("Downlink-Controlled Network Characterization Ready");
//  PRINTLN_CSTSTR("SCENARIO 1: Carrier Sense + Randomization Mode Ready");
  PRINTLN_CSTSTR("SCENARIO 2: Round Robin Frequency/SF Switching Ready");
  PRINT_CSTSTR("Base interval: ");
  PRINT_VALUE("%d", idlePeriodInSec);
  PRINTLN_CSTSTR(" seconds + 1-3s random");

  sensor_Init();
  gas_sensor_Init();
  
  // Initialize with current parameter set
  // currentParamIndex = 9;
  // updateLoRaParams(testParams[currentParamIndex]);
  // nextTransmissionTime = millis() + random(5000, 15000); // First transmission in 5 seconds
  setStandbyMode();  // Start in standby mode
  
  delay(500);
}

void loop()
{
  long startSend;
  long endSend;
  uint8_t app_key_offset=0;
  float tempC;
  bool sensorError = false;

  // Check if it's time for next transmission
  if (millis() >= nextTransmissionTime) {
    
    // Read temperature from DS18B20 sensor
    PRINTLN_CSTSTR("Reading temperature...");
/*
    // Take multiple readings for accuracy
    tempC = 0.0;
    for (int i=0; i<1; i++) {
      float reading = sensor_getValue();
      if (reading == DEVICE_DISCONNECTED_C) {
        sensorError = true;
        break;
      }
      tempC += reading;
      delay(100);
    }
      
    if (!sensorError) {
      tempC = tempC / 3;
      PRINT_CSTSTR("Temperature: ");
      PRINT_VALUE("%.2f", tempC);
      PRINTLN_CSTSTR("°C");
    } else {
      PRINTLN_CSTSTR("Sensor error - using test value");
      tempC = 24.21;  // Test value when sensor disconnected
    }



//    for (int i=0; i<5; i++) {
//        tempC += sensor_getValue();  
//        delay(100);
//    }

     // tempC = sensor_getValue();
    // tempC = 24.21;  // Test value when sensor disconnected
*/

#if defined WITH_APPKEY && not defined LORAWAN
      app_key_offset = sizeof(my_appKey);
      // set the app key in the payload
      memcpy(message,my_appKey,app_key_offset);
#endif

    tempC = sensor_getValue();
      if (tempC == -999.0) {
        PRINT_CSTSTR("ERROR - Sending custom value\n");
        tempC = random_value();
      }

    // Read all gas sensor data
    float gasCO = gas_sensor_getValue();
    int gasLPG = gas_sensor_getLPG();            
    int gasMethane = gas_sensor_getMethane();    
    int gasPropane = gas_sensor_getPropane();    
    int gasHydrogen = gas_sensor_getHydrogen();  
    int gasSmoke = gas_sensor_getSmoke(); 

    // Create payload with target size from current configuration
    uint8_t r_size;
    char payloadStr[100];
    // createPaddedPayload(payloadStr, tempC, testParams[currentParamIndex].payloadSize);
    createPaddedPayload(payloadStr, tempC, gasCO, gasLPG, gasMethane, gasPropane, gasHydrogen, gasSmoke, testParams[currentParamIndex].payloadSize);

    r_size = testParams[currentParamIndex].payloadSize;  // Use the target size directly
    
    // Copy padded message to transmission buffer
    memcpy(message, payloadStr, r_size);
    
 //   while (!configChanged){
//      PRINT_CSTSTR("DISCOVERY PHASE: SF12, BW125, CR5\n");
 //   }    
    PRINT_CSTSTR("Config: ");
    Serial.println(testParams[currentParamIndex].name);
    PRINTLN;
    
    PRINT_CSTSTR("Sending: ");
    PRINT_STR("%s", (char*)message);
    PRINTLN;
    
    PRINT_CSTSTR("Payload size is ");
    PRINT_VALUE("%d", r_size);
    PRINTLN;

    LT.printASCIIPacket(message, r_size);
    PRINTLN;
    
    // Check channel before transmission
    // LT.CarrierSense();

    // Check channel before transmission - state-dependent
    if (carrierSenseEnabled) {
      LT.CarrierSense();
      PRINTLN_CSTSTR("Carrier sense performed");
    } else {
      PRINTLN_CSTSTR("Carrier sense disabled (monitoring mode)");
    }


/*    
    uint8_t len = strlen(payloadStr);
    
    PRINT_CSTSTR("Sending: ");
    PRINT_STR("%s", payloadStr);
    PRINT_CSTSTR(" (");
    PRINT_VALUE("%d", len);
    PRINTLN_CSTSTR(" bytes)");
    PRINTLN;
*/
    uint8_t p_type=PKT_TYPE_DATA;

#ifdef WITH_APPKEY
      // indicate that we have an appkey
      p_type = p_type | PKT_FLAG_DATA_WAPPKEY;
#endif 
    
    startSend = millis();

#ifdef WITH_ACK
      p_type=PKT_TYPE_DATA | PKT_FLAG_ACK_REQ;
      // PRINTLN_CSTSTR("Will request an ACK");         
#endif

    // Send the packet, return packet length sent if OK, otherwise 0 if transmit error
#ifdef LORAWAN
      //will return packet length sent if OK, otherwise 0 if transmit error
      //we use raw format for LoRaWAN
      if (LT.transmit(message, r_size, 10000, MAX_DBM, WAIT_TX)) 
#else
      //will return packet length sent if OK, otherwise 0 if transmit error
      if (LT.transmitAddressed(message, r_size, p_type, DEFAULT_DEST_ADDR, node_addr, 10000, MAX_DBM, WAIT_TX))  
#endif
    {
      endSend = millis();
      TXPacketCount++;
      
      PRINTLN;
      PRINT_CSTSTR("Sent OK - ");
      PRINT_VALUE("%d", r_size);
      PRINT_CSTSTR(" bytes in ");
      PRINT_VALUE("%ld", endSend - startSend);
      PRINTLN_CSTSTR("ms");

// #ifdef WITH_ACK
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
//#endif

#ifdef WITH_EEPROM
      // save packet number for next packet in case of reboot     
      my_sx1272config.seq=LT.readTXSeqNo();
      EEPROM.put(0, my_sx1272config);
#endif
      PRINTLN;
      PRINT_CSTSTR("LoRa pkt size ");
      PRINT_VALUE("%d", r_size);
      PRINTLN;
      
      PRINT_CSTSTR("LoRa pkt seq ");   
      PRINT_VALUE("%d", LT.readTXSeqNo()-1);
      PRINTLN;
    
      PRINT_CSTSTR("LoRa Sent in ");
      PRINT_VALUE("%ld", endSend-startSend);
      PRINTLN;

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
          PRINT_CSTSTR("Transmission Settings Changed, Wait 30s");
          PRINTLN;
          delay(30000);
/*          
#ifdef WITH_EEPROM
          // Save new configuration index to EEPROM
          my_sx1272config.current_config_index = currentParamIndex;
          my_sx1272config.addr = node_addr;

          // Save packet number for next packet in case of reboot     
          my_sx1272config.seq = LT.readTXSeqNo();
          EEPROM.put(0, my_sx1272config);
          EEPROM.put(0, my_sx1272config);
#endif */
        }
      }
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
/*    
    // Set next transmission time
    nextTransmissionTime = millis() + (unsigned long)idlePeriodInMin * 10 * 1000;
    
    PRINT_CSTSTR("Next transmission in ");
    PRINT_VALUE("%d", idlePeriodInMin * 20);
    PRINTLN_CSTSTR(" s");
    PRINTLN;

    PRINTLN;
    PRINT_CSTSTR("Will send next value at\n");
    // can use a random part also to avoid collision
    nextTransmissionTime=millis()+((idlePeriodInSec==0)?(unsigned long)idlePeriodInMin*60*1000:(unsigned long)idlePeriodInSec*1000);
    //+(unsigned long)random(15,60)*1000;
    PRINT_VALUE("%ld", nextTransmissionTime);
    PRINTLN;    

    PRINTLN;
    PRINT_CSTSTR("Will send next value at\n");
    // SCENARIO 1: Add 1-3 second randomization to avoid collision
    uint32_t baseInterval = ((idlePeriodInSec==0)?(unsigned long)idlePeriodInMin*60*1000:(unsigned long)idlePeriodInSec*1000);
    // Generate random delay between 1000-3000 milliseconds (1.000-3.000 seconds)
    uint32_t randomDelay = 1000 + random(0, 2001);  // 1000 + [0-2000] = 1000-3000ms

    nextTransmissionTime = millis() + baseInterval + randomDelay;

    PRINT_CSTSTR("Base interval: ");
    PRINT_VALUE("%d", idlePeriodInSec);
    PRINT_CSTSTR("s + Random: ");
    PRINT_VALUE("%.3f", randomDelay/1000.0);
    PRINTLN_CSTSTR("s");
    PRINT_VALUE("%ld", nextTransmissionTime);
    PRINTLN;
*/

    // SCENARIO 2: State machine for round robin
    uint32_t currentTime = millis();
      
    // Set next transmission time based on state
    uint32_t baseInterval = ((idlePeriodInSec==0)?(unsigned long)idlePeriodInMin*60*1000:(unsigned long)idlePeriodInSec*1000);
    
    if (currentState == STATE_POST_MONITOR_SLEEP) {
      // Skip transmission during sleep
      nextTransmissionTime = currentTime + baseInterval;
      PRINTLN_CSTSTR("Unexpected: in sleep state");
    } else if (currentState == STATE_STANDBY) {
      // Add randomization for standby mode collision avoidance
      uint32_t randomDelay = 1000 + random(0, 2001);
      nextTransmissionTime = currentTime + baseInterval + randomDelay;
      PRINT_CSTSTR("Standby mode: ");
      PRINT_VALUE("%d", idlePeriodInSec);
      PRINT_CSTSTR("s + ");
      PRINT_VALUE("%.3f", randomDelay/1000.0);
      PRINTLN_CSTSTR("s random");
    } else {
      // Monitoring mode: fixed interval, no randomization
      nextTransmissionTime = currentTime + baseInterval;
      PRINT_CSTSTR("Monitoring mode: ");
      PRINT_VALUE("%d", idlePeriodInSec);
      PRINTLN_CSTSTR("s fixed interval");
    }
    
    PRINT_CSTSTR("Next transmission: ");
    PRINT_VALUE("%ld", nextTransmissionTime);
    PRINTLN;


  }

    // State status reporting every minute
    static uint32_t lastStatusTime = 0;
    if (millis() - lastStatusTime >= 60000) {
      PRINT_CSTSTR("Status - State: ");
      switch(currentState) {
        case STATE_MONITORING: PRINT_CSTSTR("MONITORING (868MHz)"); break;
        case STATE_STANDBY: PRINT_CSTSTR("STANDBY (865.2MHz)"); break;
        case STATE_POST_MONITOR_SLEEP: PRINT_CSTSTR("SLEEPING (865.2MHz)"); break;
      }
      PRINT_CSTSTR(", Config: ");
      Serial.println(testParams[currentParamIndex].name);
      lastStatusTime = millis();
    }

  delay(100);
}