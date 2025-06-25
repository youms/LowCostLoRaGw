/*
 *  Enhanced DS18B20 Temperature Sensor with LoRa transmission
 *  Network Characterization Version with Autonomous Transmit Parameter Changes
 *  Using SX12XX library for LoRa communication
 *  
 *  Changes transmit parameters autonomously every 20 minutes:
 *  - MIN: SF7, BW125, CR4/5 with T=20,50,80
 *  - MEAN: SF9, BW125, CR4/5 with T=20,50,80  
 *  - MAX: SF12, BW125, CR4/5 with T=20,50,80
 *  
 *  Hardware connections:
 *  - DS18B20 data pin -> Arduino Pin 3
 *  - LoRa module as defined in Lora_DS18B20_SX12XXX.h
 */

#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>


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

#define USE_SPI_TRANSACTION          //this is the standard behaviour of library, use SPI Transaction switching

// please uncomment only 1 choice 
//#define SX126X
#define SX127X
//#define SX128X

//#define WITH_ACK

#ifdef SX126X
#include <SX126XLT.h>
#include <SX126X_RadioSettings.h>
SX126XLT LT;                                          
#endif

#ifdef SX127X
#include <SX127XLT.h>
#include "Lora_DS18B20_SX12XXX.h"  // Include header with pin definitions
//#include <SX127X_RadioSettings.h>
SX127XLT LT;                                          
#endif

#ifdef SX128X
#include <SX128XLT.h>
#include <SX128X_RadioSettings.h>
SX128XLT LT;                                         
#endif

// Configuration
const uint8_t node_addr = 8;                        // Sensor node address
const uint8_t idlePeriodInMin = 20;                  // 20 minutes between parameter changes

// Network characterization parameters structure
struct NetworkTestParams {
  uint8_t sf;           // Spreading Factor  
  uint8_t bw;           // Bandwidth
  uint8_t cr;           // Coding Rate
  uint8_t payloadSize;  // Target payload size
  const char* name;     // Description
};

// Constants are different for different SX chips
#ifdef SX126X
  #define BW_125_VAL  LORA_BW_125    // 6 for SX126X
  #define CR_4_5_VAL  LORA_CR_4_5    // 0x01 for SX126X
#endif
#ifdef SX127X
  #define BW_125_VAL  LORA_BW_125    // 144 for SX127X 
  #define CR_4_5_VAL  LORA_CR_4_5    // 0x02 for SX127X
#endif
#ifdef SX128X
  #define BW_125_VAL  LORA_BW_0200   // 0x18 closest to 500kHz for SX128X (812.5kHz)
  #define CR_4_5_VAL  LORA_CR_4_5    // 0x01 for SX128X
#endif

// Test parameter sets (9 configurations total)
const NetworkTestParams testParams[] = {
  // MIN configurations
  {LORA_SF7, BW_125_VAL, CR_4_5_VAL, 20, "MIN-SF7-BW125-T20"},
  {LORA_SF7, BW_125_VAL, CR_4_5_VAL, 50, "MIN-SF7-BW125-T50"},
  {LORA_SF7, BW_125_VAL, CR_4_5_VAL, 80, "MIN-SF7-BW125-T80"},
  // MEAN configurations  
  {LORA_SF9, BW_125_VAL, CR_4_5_VAL, 20, "MEAN-SF9-BW125-T20"},
  {LORA_SF9, BW_125_VAL, CR_4_5_VAL, 50, "MEAN-SF9-BW125-T50"},
  {LORA_SF9, BW_125_VAL, CR_4_5_VAL, 80, "MEAN-SF9-BW125-T80"},
  // MAX configurations
  {LORA_SF12, BW_125_VAL, CR_4_5_VAL, 20, "MAX-SF12-BW125-T20"},
  {LORA_SF12, BW_125_VAL, CR_4_5_VAL, 50, "MAX-SF12-BW125-T50"},
  {LORA_SF12, BW_125_VAL, CR_4_5_VAL, 80, "MAX-SF12-BW125-T80"}
};

const uint8_t NUM_TEST_PARAMS = sizeof(testParams) / sizeof(testParams[0]);

// Global variables
uint8_t currentParamIndex = 0;                       // Current parameter set index
unsigned long lastParamChangeTime = 0;              // Last time parameters were changed
unsigned long nextTransmissionTime = 0;             // Next transmission time

// DS18B20 Temperature Sensor setup
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Message buffer
uint8_t message[100];

// Packet counters
uint32_t TXPacketCount = 0;

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

#ifdef WITH_EEPROM
#include <EEPROM.h>

struct SX1272Config {
  uint8_t flag1;
  uint8_t flag2; 
  uint8_t seq;
};

SX1272Config my_sx1272config;
#endif

// Function to pad temperature string to reach target payload size
void createPaddedPayload(char* dest, float temperature, uint8_t targetSize) {
  char float_str[10];
  ftoa(float_str, temperature, 2);
  
  // Start with basic temperature format: \!TC/temp
  int baseSize = sprintf(dest, "\\!TC/%s", float_str);
  
  // If we need more bytes, pad with null bytes (0x00) after the temperature string
  // The gateway reads the exact payload length, so null padding won't affect parsing
  if (targetSize > baseSize) {
    // Fill remaining bytes with null bytes - safe for gateway decoding
    for (int i = baseSize; i < targetSize; i++) {
      dest[i] = 0x00;  // Fill with null bytes
    }
  }
}

// Function to update LoRa parameters
void updateLoRaParams(const NetworkTestParams& params) {
  PRINT_CSTSTR("Updating LoRa parameters to: ");
  Serial.println(params.name);
  
  // Set new modulation parameters
#if defined SX126X || defined SX127X
  LT.setModulationParams(params.sf, params.bw, params.cr, LDRO_AUTO);
#endif
#ifdef SX128X
  LT.setModulationParams(params.sf, params.bw, params.cr);
#endif

  PRINT_CSTSTR("SF: ");
  switch(params.sf) {
    case LORA_SF7:  PRINTLN_CSTSTR("7"); break;
    case LORA_SF9:  PRINTLN_CSTSTR("9"); break;
    case LORA_SF12: PRINTLN_CSTSTR("12"); break;
  }
  
  PRINT_CSTSTR("BW: 500kHz, CR: 4/5, Target payload: ");
  PRINT_VALUE("%d", params.payloadSize);
  PRINTLN_CSTSTR(" bytes");
}

// Function to cycle to next parameter set
void cycleToNextParams() {
  currentParamIndex = (currentParamIndex + 1) % NUM_TEST_PARAMS;
  updateLoRaParams(testParams[currentParamIndex]);
  lastParamChangeTime = millis();
  
  PRINT_CSTSTR("Parameter set ");
  PRINT_VALUE("%d", currentParamIndex + 1);
  PRINT_CSTSTR(" of ");
  PRINT_VALUE("%d", NUM_TEST_PARAMS);
  PRINTLN;
}

// LoRa configuration function - CRITICAL ADDITION FROM WORKING CODE
void loraConfig() {
  //***************************************************************************************************
  //Setup Lora device
  //***************************************************************************************************
  
  LT.setMode(MODE_STDBY_RC);
#ifdef SX126X
  LT.setRegulatorMode(USE_DCDC);
  LT.setPaConfig(0x04, PAAUTO, LORA_DEVICE);
  LT.setDIO3AsTCXOCtrl(TCXO_CTRL_3_3V);
  LT.calibrateDevice(ALLDevices);                //is required after setting TCXO
  LT.calibrateImage(DEFAULT_CHANNEL);
  LT.setDIO2AsRfSwitchCtrl();
#endif
#ifdef SX128X
  LT.setRegulatorMode(USE_LDO);
#endif
  //set for LoRa transmissions                              
  LT.setPacketType(PACKET_TYPE_LORA);
  //set the operating frequency                 
  LT.setRfFrequency(DEFAULT_CHANNEL, Offset);                   
//run calibration after setting frequency
#ifdef SX127X
  LT.calibrateImage(0);
#endif
  //set LoRa modem parameters - Use initial configuration from testParams[0]
#if defined SX126X || defined SX127X
  LT.setModulationParams(testParams[0].sf, testParams[0].bw, testParams[0].cr, LDRO_AUTO);
#endif
#ifdef SX128X
  LT.setModulationParams(testParams[0].sf, testParams[0].bw, testParams[0].cr);
#endif                                     
  //where in the SX buffer packets start, TX and RX
  LT.setBufferBaseAddress(0x00, 0x00);
  //set packet parameters
#if defined SX126X || defined SX127X                     
  LT.setPacketParams(8, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);
#endif
#ifdef SX128X
  LT.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
#endif
  //syncword, LORA_MAC_PRIVATE_SYNCWORD = 0x12, or LORA_MAC_PUBLIC_SYNCWORD = 0x34
#if defined SX126X || defined SX127X
  LT.setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);
  //set for highest sensitivity at expense of slightly higher LNA current
  LT.setHighSensitivity();  //set for maximum gain
#endif
#ifdef SX126X
  //set for IRQ on TX done and timeout on DIO1
  LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);
#endif
#ifdef SX127X
  //set for IRQ on TX done
  LT.setDioIrqParams(IRQ_RADIO_ALL, IRQ_TX_DONE, 0, 0);
  LT.setPA_BOOST(PA_BOOST);
#endif
#ifdef SX128X
  LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);
#endif    
}

/*****************************
 _____      _               
/  ___|    | |              
\ `--.  ___| |_ _   _ _ __   
 `--. \/ _ \ __| | | | '_ \  
/\__/ /  __/ |_| |_| | |_) | 
\____/ \___|\__|\__,_| .__/  
                     | |     
                     |_|     
*****************************/

void setup()
{
  delay(1000);
  
  Serial.begin(38400);
  while (!Serial);
  
  PRINTLN_CSTSTR("Enhanced DS18B20 LoRa Network Characterization");
  PRINTLN_CSTSTR("Autonomous parameter changes every 20 minutes");
  
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

  // Configure LoRa parameters - CRITICAL ADDITION
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
    PRINT_CSTSTR("Using packet sequence number of ");
    PRINT_VALUE("%d", LT.readTXSeqNo());
    PRINTLN;  
  }
  else {
    my_sx1272config.flag1=0x12;
    my_sx1272config.flag2=0x34;
    my_sx1272config.seq=LT.readTXSeqNo();  
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

  PRINTLN_CSTSTR("Network Characterization Mode Ready");
  
  // Initialize with first parameter set
  updateLoRaParams(testParams[currentParamIndex]);
  lastParamChangeTime = millis();
  nextTransmissionTime = millis() + 10000; // First transmission in 10 seconds
  
  delay(500);
}

/*****************************
 _                       
| |                      
| |     ___   ___  _ __  
| |    / _ \ / _ \| '_ \ 
| |___| (_) | (_) | |_) |
\_____/\___/ \___/| .__/ 
                  | |    
                  |_|    
*****************************/

void loop()
{
  long startSend;
  long endSend;
  float tempC;
  bool sensorError = false;

  // Check if it's time to change parameters (every 20 minutes)
  if (millis() - lastParamChangeTime >= (unsigned long)idlePeriodInMin * 60 * 1000) {
    cycleToNextParams();
  }

  // Check if it's time for next transmission (every 15 seconds for testing)
  if (millis() >= nextTransmissionTime) {
    
    // Read temperature from DS18B20 sensor
    PRINTLN_CSTSTR("Reading temperature...");
    
    // Take multiple readings for accuracy
    tempC = 0.0;
    for (int i=0; i<5; i++) {
      sensors.requestTemperatures(); 
      tempC += sensors.getTempCByIndex(0);
      delay(100);
    }
    tempC = tempC/5;
    
    // Check if reading was successful
    if (tempC == DEVICE_DISCONNECTED_C) {
      PRINTLN_CSTSTR("Error: Could not read temperature data");
      sensorError = true;
      nextTransmissionTime = millis() + 10000; // Try again in 30 seconds
      return;
    }
    
    // Only transmit if we have valid sensor data
    if (!sensorError) {
      // Display temperature reading
      PRINT_CSTSTR("Mean temp is ");
      PRINT_VALUE("%.2f", tempC);
      PRINTLN_CSTSTR("°C");
      
      // Create padded payload based on current test parameters
      uint8_t r_size;
      char paddedMessage[100];  // Buffer for padded message
      
      // Create padded payload to target size
      createPaddedPayload(paddedMessage, tempC, testParams[currentParamIndex].payloadSize);
      r_size = testParams[currentParamIndex].payloadSize;  // Use the target size directly
      
      // Copy padded message to transmission buffer
      memcpy(message, paddedMessage, r_size);
      
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
      LT.CarrierSense();
      
      uint8_t p_type = PKT_TYPE_DATA;
      
#ifdef WITH_ACK
      p_type = PKT_TYPE_DATA | PKT_FLAG_ACK_REQ;
      PRINTLN_CSTSTR("Will request an ACK");         
#endif
      
      startSend = millis();
      
      // Transmit via LoRa
      if (LT.transmitAddressed(message, r_size, p_type, DEFAULT_DEST_ADDR, LT.readDevAddr(), 10000, MAX_DBM, WAIT_TX))
      {
        endSend = millis();                                          
        TXPacketCount++;
        uint16_t localCRC = LT.CRCCCITT(message, r_size, 0xFFFF);
        
        PRINT_CSTSTR("TXPacketCount ");
        PRINT_VALUE("%d", TXPacketCount);
        PRINT_CSTSTR(" LocalCRC 0x");
        PRINT_HEX("0x%04X", localCRC);
        PRINT_CSTSTR(" TransmitTime ");
        PRINT_VALUE("%d", endSend - startSend);
        PRINTLN_CSTSTR("mS");

#ifdef WITH_ACK
        PRINTLN_CSTSTR("Waiting for ACK");
        
        if (LT.readAckStatus())
        {
          endSend = millis();
          PRINT_CSTSTR("ACK received in ");
          PRINT_VALUE("%d", endSend - startSend);
          PRINTLN_CSTSTR("mS");
        }
        else
        {
          PRINTLN_CSTSTR("No ACK received");
        }
#endif

#ifdef WITH_EEPROM
        // Save packet sequence number to EEPROM
        my_sx1272config.seq = LT.readTXSeqNo();
        EEPROM.put(0, my_sx1272config);
#endif
      }
      else
      {
        endSend = millis();
        //if here there was an error transmitting packet
        PRINTLN_CSTSTR("Transmission failed");
        uint16_t IRQStatus = LT.readIrqStatus();
        PRINT_CSTSTR("IRQreg ");
        PRINT_HEX("0x%04X", IRQStatus);
        PRINTLN;
      }
      
      // Schedule next transmission (every 15 seconds for characterization)
      nextTransmissionTime = millis() + 5000;
    }
  }
  
  delay(100); // Small delay to prevent busy waiting
}