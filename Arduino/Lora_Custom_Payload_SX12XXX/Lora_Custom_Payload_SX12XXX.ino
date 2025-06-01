/*
 *  LoRa Interactive Message Sender with SX12XX Library
 *  Enhanced modular version for real-time messaging
 *  
 *  Sends user-typed messages via LoRa in real-time
 *  Compatible with Congduc Pham's Low-Cost LoRa Gateway
 *
 *  Hardware connections:
 *  - LoRa module as defined in LoRa_Interactive_SX12XXX.h
 *  - Serial Monitor at 115200 baud for message input
 */

#include <SPI.h>

#define USE_SPI_TRANSACTION          //this is the standard behaviour of library, use SPI Transaction switching
#include "LoRa_Custom_Payload_SX12XXX.h"  // Include header with pin definitions

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
SX127XLT LT;                                          
#endif

#ifdef SX128X
#include <SX128XLT.h>
#include <SX128X_RadioSettings.h>
SX128XLT LT;                                         
#endif

/********************************************************************
 _____              __ _                       _   _             
/  __ \            / _(_)                     | | (_)            
| /  \/ ___  _ __ | |_ _  __ _ _   _ _ __ __ _| |_ _  ___  _ __  
| |    / _ \| '_ \|  _| |/ _` | | | | '__/ _` | __| |/ _ \| '_ \ 
| \__/\ (_) | | | | | | | (_| | |_| | | | (_| | |_| | (_) | | | |
 \____/\___/|_| |_|_| |_|\__, |\__,_|_|  \__,_|\__|_|\___/|_| |_|
                          __/ |                                  
                         |___/                                   
********************************************************************/

///////////////////////////////////////////////////////////////////
// COMMENT OR UNCOMMENT TO CHANGE FEATURES. 
// ONLY IF YOU KNOW WHAT YOU ARE DOING!!! OTHERWISE LEAVE AS IT IS
#define WITH_ACK                                // Request ACK from receiver
//#define MY_FREQUENCY 868100000                // Custom frequency (optional)
//#define PUBLIC_SYNCWORD                       // For LoRaWAN gateway compatibility
///////////////////////////////////////////////////////////////////

// Interactive messaging state variables
char message[MAX_MSG_LEN] = "";
int msgIndex = 0;
bool newMessage = false;
bool transmissionComplete = true;
uint32_t TXPacketCount = 0;

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE NODE ADDRESSES
uint8_t node_addr = 100;        // This node's address (source)
uint8_t dest_addr = DEFAULT_DEST_ADDR;  // Destination address (1 = gateway)
//////////////////////////////////////////////////////////////////

/*****************************
 _____           _      
/  __ \         | |     
| /  \/ ___   __| | ___ 
| |    / _ \ / _` |/ _ \
| \__/\ (_) | (_| |  __/
 \____/\___/ \__,_|\___|
*****************************/ 

// we wrapped Serial.println to support the Arduino Zero or M0
#if defined __SAMD21G18A__ && not defined ARDUINO_SAMD_FEATHER_M0
#define PRINTLN                   SerialUSB.println("")              
#define PRINT_CSTSTR(param)       SerialUSB.print(F(param))
#define PRINTLN_CSTSTR(param)     SerialUSB.println(F(param))
#define PRINT_STR(fmt,param)      SerialUSB.print(param)
#define PRINTLN_STR(fmt,param)    SerialUSB.println(param)
#define PRINT_VALUE(fmt,param)    SerialUSB.print(param)
#define PRINTLN_VALUE(fmt,param)  SerialUSB.println(param)
#define PRINT_HEX(fmt,param)      SerialUSB.print(param,HEX)
#define PRINTLN_HEX(fmt,param)    SerialUSB.println(param,HEX)
#define FLUSHOUTPUT               SerialUSB.flush()
#else
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
#endif

#ifdef WITH_ACK
#define NB_RETRIES 2
#endif

/*****************************
 _   _ _   _ _ _ _   _          
| | | | |_(_) (_) |_(_) ___  ___
| | | | __| | | | __| |/ _ \/ __|
| |_| | |_| | | | |_| |  __/\__ \
 \___/ \__|_|_|_|\__|_|\___||___/
*****************************/ 

void printDeviceInfo() {
    PRINTLN_CSTSTR("=== Device Configuration ===");
    
    // Board detection
#ifdef ARDUINO_AVR_PRO
    PRINTLN_CSTSTR("Board: Arduino Pro Mini");
#endif
#ifdef ARDUINO_AVR_NANO
    PRINTLN_CSTSTR("Board: Arduino Nano");
#endif
#ifdef ARDUINO_AVR_UNO
    PRINTLN_CSTSTR("Board: Arduino Uno");
#endif
#ifdef ARDUINO_AVR_MEGA2560
    PRINTLN_CSTSTR("Board: Arduino Mega2560");
#endif
#ifdef ESP32 
    PRINTLN_CSTSTR("Board: ESP32");
#endif
#if defined ARDUINO_ESP8266_ESP01 || defined ARDUINO_ESP8266_NODEMCU || defined ESP8266
    PRINTLN_CSTSTR("Board: ESP8266");
#endif
    
    // LoRa chip info
#ifdef SX126X
    PRINTLN_CSTSTR("LoRa Chip: SX126X family");
#endif
#ifdef SX127X
    PRINTLN_CSTSTR("LoRa Chip: SX127X family");
#endif
#ifdef SX128X
    PRINTLN_CSTSTR("LoRa Chip: SX128X family");
#endif

    PRINT_CSTSTR("Source Address: ");
    PRINTLN_VALUE("%d", node_addr);
    PRINT_CSTSTR("Destination Address: ");
    PRINTLN_VALUE("%d", dest_addr);
    PRINT_CSTSTR("Frequency: ");
    PRINTLN_VALUE("%lu", DEFAULT_CHANNEL);
    PRINT_CSTSTR(" Hz");
    PRINT_CSTSTR("Max Message Length: ");
    PRINT_VALUE("%d", MAX_MSG_LEN - 1);
    PRINTLN_CSTSTR(" characters");
    
#ifdef WITH_ACK
    PRINTLN_CSTSTR("Acknowledgments: Enabled");
#else
    PRINTLN_CSTSTR("Acknowledgments: Disabled");
#endif

    PRINTLN_CSTSTR("============================");
}

void printTransmissionDetails(uint8_t* payload, int len) {
    PRINTLN_CSTSTR("=== Transmission Details ===");
    PRINT_CSTSTR("Message: \"");
    PRINT_STR("%s", message);
    PRINTLN_CSTSTR("\"");
    PRINT_CSTSTR("Length: ");
    PRINT_VALUE("%d", len);
    PRINTLN_CSTSTR(" bytes");
    PRINT_CSTSTR("From: ");
    PRINT_VALUE("%d", node_addr);
    PRINT_CSTSTR(" To: ");
    PRINTLN_VALUE("%d", dest_addr);
    
#if SHOW_DETAILED_INFO
    PRINT_CSTSTR("Payload (hex): ");
    for (int i = 0; i < len; i++) {
        if (payload[i] < 0x10) PRINT_CSTSTR("0");
        PRINT_HEX("%02X", payload[i]);
        PRINT_CSTSTR(" ");
    }
    PRINTLN;
    
    PRINT_CSTSTR("Payload (ASCII): ");
    for (int i = 0; i < len; i++) {
        if (payload[i] >= 32 && payload[i] <= 126) {
            PRINT_STR("%c", (char)payload[i]);
        } else {
            PRINT_CSTSTR(".");
        }
    }
    PRINTLN;
#endif
    PRINTLN_CSTSTR("============================");
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
******************************/

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
#ifdef MY_FREQUENCY
  LT.setRfFrequency(MY_FREQUENCY, Offset);
#else  
  LT.setRfFrequency(DEFAULT_CHANNEL, Offset);                   
#endif
//run calibration after setting frequency
#ifdef SX127X
  LT.calibrateImage(0);
#endif
  //set LoRa modem parameters
#if defined SX126X || defined SX127X
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate, Optimisation);
#endif
#ifdef SX128X
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
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
#ifdef PUBLIC_SYNCWORD
  LT.setSyncWord(LORA_MAC_PUBLIC_SYNCWORD);              
#else
  LT.setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);
#endif
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

void setup()
{
  delay(3000);
  
  // Start serial communication
#if defined __SAMD21G18A__ && not defined ARDUINO_SAMD_FEATHER_M0 
  SerialUSB.begin(SERIAL_BAUD);
#else
  Serial.begin(SERIAL_BAUD);  
#endif 

  // Print welcome message
  PRINTLN_CSTSTR("🚀 LoRa Interactive Message Sender (SX12XX Modular)");
  PRINTLN_CSTSTR("===================================================");
  printDeviceInfo();
  PRINTLN_CSTSTR("Type your message and press Enter to send via LoRa");
  PRINTLN_CSTSTR("Use backspace to edit, empty line to skip");
  PRINTLN_CSTSTR("===================================================");

  // Initialize SPI
  SPI.begin();

  //setup hardware pins used by device, then check if device is found
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
    PRINTLN_CSTSTR("✅ LoRa Device found");
    delay(1000);
  }
  else
  {
    PRINTLN_CSTSTR("❌ No LoRa device responding");
    while (1)
    {
    }
  }

  // Configure LoRa parameters
  loraConfig();

  PRINTLN;
  LT.printModemSettings();                                     
  PRINTLN;
  LT.printOperatingSettings();                                 
  PRINTLN;

  PRINT_CSTSTR("Setting Power: ");
  PRINTLN_VALUE("%d", MAX_DBM); 

  LT.setDevAddr(node_addr);
  PRINT_CSTSTR("Node address: ");
  PRINT_VALUE("%d", node_addr);
  PRINTLN;  
  
  PRINTLN_CSTSTR("✅ LoRa Interactive Sender ready");
  PRINTLN_CSTSTR("===================================");
  
  transmissionComplete = true;
  PRINTLN_CSTSTR("💬 Ready for input. Type your message and press Enter.");
  PRINT_CSTSTR(">> ");
  
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
    // Process serial input
    while (Serial.available() > 0 && !newMessage) {
        char inChar = (char)Serial.read();
        
        // Echo character if enabled and printable
        #if ECHO_CHARACTERS
        if (inChar >= 32 && inChar <= 126) {
            Serial.print(inChar);
        }
        #endif
        
        // Process the incoming character
        if (inChar == '\n' || inChar == '\r') {
            Serial.println(); // Clean line ending
            if (msgIndex > 0) {
                message[msgIndex] = '\0';
                newMessage = true;
                PRINT_CSTSTR("📝 Message ready: \"");
                PRINT_STR("%s", message);
                PRINTLN_CSTSTR("\"");
            } else {
                PRINTLN_CSTSTR("📝 Empty message - skipped");
                PRINT_CSTSTR(">> ");
            }
        } 
        else if (inChar == 8 || inChar == 127) { // Backspace/DEL
            if (msgIndex > 0) {
                msgIndex--;
                message[msgIndex] = '\0';
                #if ECHO_CHARACTERS
                PRINT_CSTSTR("\b \b"); // Erase from terminal
                #endif
            }
        } 
        else if (msgIndex < MAX_MSG_LEN - 1 && inChar >= 32 && inChar <= 126) {
            // Add printable character if space available
            message[msgIndex++] = inChar;
        } 
        else if (msgIndex >= MAX_MSG_LEN - 1) {
            Serial.println();
            PRINT_CSTSTR("⚠️  Message too long! Maximum ");
            PRINT_VALUE("%d", MAX_MSG_LEN - 1);
            PRINTLN_CSTSTR(" characters.");
            PRINT_CSTSTR(">> ");
            // Reset buffer
            memset(message, 0, MAX_MSG_LEN);
            msgIndex = 0;
        }
    }
    
    // Send message if ready and transmission complete
    if (newMessage && transmissionComplete) {
        PRINTLN_CSTSTR("📡 Preparing LoRa transmission...");
        
        // Convert message to byte array
        uint8_t payload[MAX_MSG_LEN];
        int len = strlen(message);
        
        for (int i = 0; i < len; i++) {
            payload[i] = (uint8_t)message[i];
        }
        
        // Display transmission details
        printTransmissionDetails(payload, len);
        
        // Check channel before transmission
        LT.CarrierSense();
        
        uint8_t p_type = PKT_TYPE_DATA;
        
#ifdef WITH_ACK
        p_type = PKT_TYPE_DATA | PKT_FLAG_ACK_REQ;
        PRINTLN_CSTSTR("📋 Will request an ACK");         
#endif
        
        long startSend = millis();
        
        // Transmit via LoRa
        if (LT.transmitAddressed(payload, len, p_type, dest_addr, node_addr, 10000, MAX_DBM, WAIT_TX))
        {
            long endSend = millis();                                          
            TXPacketCount++;
            uint16_t localCRC = LT.CRCCCITT(payload, len, 0xFFFF);
            
            PRINT_CSTSTR("✅ Transmission successful! CRC: ");
            PRINT_HEX("0x%04X", localCRC);
            PRINTLN;

#ifdef WITH_ACK
            if (LT.readAckStatus()) {
                PRINT_CSTSTR("📨 Received ACK from node ");
                PRINT_VALUE("%d", LT.readRXSource());
                PRINTLN;
                PRINT_CSTSTR("📶 SNR of transmitted packet: ");
                PRINT_VALUE("%d", LT.readPacketSNRinACK());          
                PRINTLN;
            }
            else {
                PRINTLN_CSTSTR("📪 No ACK received");
            }
#endif

            PRINT_CSTSTR("📦 Packet size: ");
            PRINT_VALUE("%d", len);
            PRINTLN_CSTSTR(" bytes");
            
            PRINT_CSTSTR("🔢 Packet sequence: ");   
            PRINT_VALUE("%d", LT.readTXSeqNo()-1);    
            PRINTLN;

            PRINT_CSTSTR("⏱️  Transmission time: ");
            PRINT_VALUE("%ld", endSend-startSend);
            PRINTLN_CSTSTR("ms");
            
            PRINTLN_CSTSTR("📡 Message sent successfully!");
        }
        else
        {
            //if here there was an error transmitting packet
            uint16_t IRQStatus;
            IRQStatus = LT.readIrqStatus();                      
            PRINT_CSTSTR("❌ Transmission Error - IRQ: ");
            PRINT_HEX("0x%04X", IRQStatus);
            PRINTLN;
            LT.printIrqStatus(); 
        }
        
        PRINTLN_CSTSTR("===============================");
        
        // Clear the message buffer
        memset(message, 0, MAX_MSG_LEN);
        msgIndex = 0;
        newMessage = false;
        transmissionComplete = true;
        
        // Ready for next message
        PRINTLN_CSTSTR("✅ Ready for next message. Type and press Enter.");
        PRINT_CSTSTR(">> ");
    }
}