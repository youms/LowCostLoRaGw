/*
 *  DHT22 Temperature & Humidity Sensor with LoRa transmission
 *  Enhanced version with low power, ACK, EEPROM and advanced features
 *  Using SX12XX library for LoRa communication
 *  
 *  Reads temperature and humidity from DHT22 sensor and transmits via LoRa
 *  Compatible with Congduc Pham's Low-Cost LoRa Gateway
 *
 *  Hardware connections:
 *  - DHT22 data pin -> Arduino Pin 3 (changed from 4 to avoid conflict)
 *  - LoRa module as defined in settings file
 */

#include <DHT.h>
#include <SPI.h>
#include "Lora_DH22_SX12XXX.h"

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
#define WITH_EEPROM                             // Save packet sequence numbers
#define LOW_POWER                               // Enable low power mode
#define LOW_POWER_HIBERNATE                     // Use deepest sleep mode
#define WITH_ACK                                // Request ACK from gateway
//#define LOW_POWER_TEST                        // Test mode with shorter sleep
//#define MY_FREQUENCY 868100000                // Custom frequency (optional)
//#define PUBLIC_SYNCWORD                       // For LoRaWAN gateways
//#define DHT_POWER_PIN 8                       // External power control pin for DHT22
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// ADD HERE OTHER PLATFORMS THAT DO NOT SUPPORT EEPROM
#if defined ARDUINO_SAM_DUE || defined __SAMD21G18A__
#undef WITH_EEPROM
#endif

///////////////////////////////////////////////////////////////////
// ADD HERE OTHER PLATFORMS THAT DO NOT SUPPORT LOW POWER LIB
#if defined ARDUINO_SAM_DUE || defined _VARIANT_ARDUINO_DUE_X_
#undef LOW_POWER
#endif
///////////////////////////////////////////////////////////////////

// DHT22 Sensor Setup
#define DHT_TYPE DHT22
DHT dht(DHT22_PIN, DHT_TYPE);

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE NODE ADDRESS 
uint8_t node_addr = 9;
//////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// CHANGE HERE THE TIME IN MINUTES BETWEEN 2 READING & TRANSMISSION
unsigned int idlePeriodInMin = 10;
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// IF YOU SEND A LONG STRING, INCREASE THE SIZE OF MESSAGE
uint8_t message[100];
///////////////////////////////////////////////////////////////////

//keep track of the number of successful transmissions
uint32_t TXPacketCount = 0;

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

#ifdef WITH_EEPROM
#include <EEPROM.h>
#endif

#ifdef WITH_ACK
#define NB_RETRIES 2
#endif

#ifdef LOW_POWER
// this is for the Teensy36, Teensy35, Teensy31/32 & TeensyLC
// need v6 of Snooze library
#if defined __MK20DX256__ || defined __MKL26Z64__ || defined __MK64FX512__ || defined __MK66FX1M0__
#define LOW_POWER_PERIOD 60
#include <Snooze.h>
SnoozeTimer timer;
SnoozeBlock sleep_config(timer);
#elif defined ARDUINO_ESP8266_ESP01 || defined ARDUINO_ESP8266_NODEMCU || defined ESP8266
#define LOW_POWER_PERIOD 60
//we will use the deepSleep feature, so no additional library
#else // for all other boards based on ATMega168, ATMega328P, ATMega32U4, ATMega2560, ATMega256RFR2, ATSAMD21G18A
#define LOW_POWER_PERIOD 8
// you need the LowPower library from RocketScream
// https://github.com/rocketscream/Low-Power
#include "LowPower.h"

#ifdef __SAMD21G18A__
// use the RTC library
#include "RTCZero.h"
/* Create an rtc object */
RTCZero rtc;
#endif
#endif
unsigned int nCycle = idlePeriodInMin*60/LOW_POWER_PERIOD;
#endif

unsigned long nextTransmissionTime = 0L;

#ifdef WITH_EEPROM
struct sx1272config {
  uint8_t flag1;
  uint8_t flag2;
  uint8_t seq;
  // can add other fields such as LoRa mode,...
};

sx1272config my_sx1272config;
#endif

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

// Calculate heat index (feels like temperature)
float calculateHeatIndex(float tempC, float humidity) {
  if (tempC < 26.7) return tempC; // Below 80°F, heat index not applicable
  
  float tempF = tempC * 9.0/5.0 + 32.0; // Convert to Fahrenheit for calculation
  
  float hi = 0.5 * (tempF + 61.0 + ((tempF - 68.0) * 1.2) + (humidity * 0.094));
  
  if (hi < 80.0) {
    return (hi - 32.0) * 5.0/9.0; // Convert back to Celsius
  }
  
  // Full heat index calculation
  hi = -42.379 + 2.04901523 * tempF + 10.14333127 * humidity
       - 0.22475541 * tempF * humidity - 0.00683783 * tempF * tempF
       - 0.05481717 * humidity * humidity + 0.00122874 * tempF * tempF * humidity
       + 0.00085282 * tempF * humidity * humidity - 0.00000199 * tempF * tempF * humidity * humidity;
       
  return (hi - 32.0) * 5.0/9.0; // Convert back to Celsius
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
  
  // Start serial communication for debugging purposes
#if defined __SAMD21G18A__ && not defined ARDUINO_SAMD_FEATHER_M0 
  SerialUSB.begin(38400);
#else
  Serial.begin(38400);  
#endif 

  randomSeed(analogRead(0)+analogRead(0)+analogRead(0));

  // Print a start message
  PRINT_CSTSTR("DHT22 Temperature & Humidity Sensor with LoRa SX12XX (Enhanced)\n"); 

  // Start up the DHT22 sensor
#ifdef DHT_POWER_PIN
  pinMode(DHT_POWER_PIN, OUTPUT);
  digitalWrite(DHT_POWER_PIN, HIGH);
  delay(2000); // Give DHT22 time to stabilize
#endif

  dht.begin();
  PRINTLN_CSTSTR("DHT22 temperature & humidity sensor initialized");

#ifdef LOW_POWER
#ifdef __SAMD21G18A__
  rtc.begin();
#endif  
#endif

#ifdef ARDUINO_AVR_PRO
  PRINT_CSTSTR("Arduino Pro Mini detected\n");  
#endif
#ifdef ARDUINO_AVR_NANO
  PRINT_CSTSTR("Arduino Nano detected\n");   
#endif
#ifdef ARDUINO_AVR_UNO
  PRINT_CSTSTR("Arduino Uno detected\n");   
#endif
#ifdef ARDUINO_AVR_MEGA2560
  PRINT_CSTSTR("Arduino Mega2560 detected\n");  
#endif
#ifdef ARDUINO_SAM_DUE
  PRINT_CSTSTR("Arduino Due detected\n");  
#endif
#ifdef __MK66FX1M0__
  PRINT_CSTSTR("Teensy36 MK66FX1M0 detected\n");
#endif
#ifdef __MK64FX512__
  PRINT_CSTSTR("Teensy35 MK64FX512 detected\n");
#endif
#ifdef __MK20DX256__
  PRINT_CSTSTR("Teensy31/32 MK20DX256 detected\n");
#endif
#ifdef __MKL26Z64__
  PRINT_CSTSTR("TeensyLC MKL26Z64 detected\n");
#endif
#if defined ARDUINO_SAMD_ZERO && not defined ARDUINO_SAMD_FEATHER_M0
  PRINT_CSTSTR("Arduino M0/Zero detected\n");
#endif
#ifdef ARDUINO_AVR_FEATHER32U4 
  PRINT_CSTSTR("Adafruit Feather32U4 detected\n"); 
#endif
#ifdef ARDUINO_SAMD_FEATHER_M0
  PRINT_CSTSTR("Adafruit FeatherM0 detected\n");
#endif
#if defined ARDUINO_ESP8266_ESP01 || defined ARDUINO_ESP8266_NODEMCU || defined ESP8266
  PRINT_CSTSTR("Expressif ESP8266 detected\n");
#endif
#if defined ARDUINO_Heltec_WIFI_LoRa_32 || defined ARDUINO_WIFI_LoRa_32  || defined HELTEC_LORA
  PRINT_CSTSTR("Heltec WiFi LoRa 32 detected\n");
#endif
#ifdef ESP32 
  PRINT_CSTSTR("ESP32 detected\n");
#endif

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
    PRINTLN_CSTSTR("LoRa Device found");
    delay(1000);
  }
  else
  {
    PRINTLN_CSTSTR("No LoRa device responding");
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

#ifdef WITH_EEPROM
#if defined ARDUINO_ESP8266_ESP01 || defined ARDUINO_ESP8266_NODEMCU
  EEPROM.begin(512);
#endif
  // get config from EEPROM
  EEPROM.get(0, my_sx1272config);

  // found a valid config?
  if (my_sx1272config.flag1==0x12 && my_sx1272config.flag2==0x34) {
    PRINT_CSTSTR("Get back previous sx1272 config\n");
    // set sequence number for SX1272 library
    LT.setTXSeqNo(my_sx1272config.seq);
    PRINT_CSTSTR("Using packet sequence number of ");
    PRINT_VALUE("%d", LT.readTXSeqNo());
    PRINTLN;  
  }
  else {
    // otherwise, write config and start over
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

  PRINTLN_CSTSTR("DHT22 sensor ready");
  
#ifdef LOW_POWER
  PRINTLN_CSTSTR("Low power mode enabled");
#endif
#ifdef WITH_ACK
  PRINTLN_CSTSTR("ACK mode enabled");
#endif
#ifdef WITH_EEPROM
  PRINTLN_CSTSTR("EEPROM persistence enabled");
#endif
  
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
  float tempC, humidity;
  bool sensorError = false;

#ifndef LOW_POWER
  // Check if it's time for next transmission
  if (millis() > nextTransmissionTime) {
#endif

    // Power up DHT22 if using external power control
#ifdef DHT_POWER_PIN
    digitalWrite(DHT_POWER_PIN, HIGH);
    delay(2000); // DHT22 needs time to stabilize
#endif

    // Read temperature and humidity from DHT22 sensor
    PRINTLN_CSTSTR("Reading DHT22 sensor...");
    
    // Take multiple readings for accuracy
    float tempSum = 0.0;
    float humSum = 0.0;
    int validReadings = 0;
    
    for (int i = 0; i < 5; i++) {
      float t = dht.readTemperature();
      float h = dht.readHumidity();
      
      // Check if readings are valid
      if (!isnan(t) && !isnan(h)) {
        tempSum += t;
        humSum += h;
        validReadings++;
      }
      delay(200); // DHT22 needs time between readings
    }
    
#ifdef DHT_POWER_PIN
    digitalWrite(DHT_POWER_PIN, LOW); // Power down sensor
#endif

    // Check if we got valid readings
    if (validReadings == 0) {
      PRINTLN_CSTSTR("Error: Could not read DHT22 data");
      sensorError = true;
#ifndef LOW_POWER
      delay(idlePeriodInMin * 60 * 1000);
      return;
#endif
    }
    
    // Only transmit if we have valid sensor data
    if (!sensorError) {
      // Calculate average values
      tempC = tempSum / validReadings;
      humidity = humSum / validReadings;
      
      // Calculate heat index
      float heatIndex = calculateHeatIndex(tempC, humidity);
      
      // Display sensor readings
      PRINT_CSTSTR("Temperature: ");
      PRINT_VALUE("%.2f", tempC);
      PRINTLN_CSTSTR("°C");
      
      PRINT_CSTSTR("Humidity: ");
      PRINT_VALUE("%.2f", humidity);
      PRINTLN_CSTSTR("%");
      
      PRINT_CSTSTR("Heat Index: ");
      PRINT_VALUE("%.2f", heatIndex);
      PRINTLN_CSTSTR("°C");
      
      // Prepare LoRa message in gateway-compatible format
      uint8_t r_size;
      char temp_str[10], hum_str[10], hi_str[10];
      
      // Use memory-efficient float to string conversion
      ftoa(temp_str, tempC, 2);
      ftoa(hum_str, humidity, 1);
      ftoa(hi_str, heatIndex, 2);
      
      // Format: \!TC/temp/HU/humidity/HI/heat_index
      r_size = sprintf((char*)message, "\\!TC/%s/HU/%s/HI/%s", temp_str, hum_str, hi_str);
      
      PRINT_CSTSTR("Sending: ");
      PRINT_STR("%s", (char*)message);
      PRINTLN;
      
      PRINT_CSTSTR("Real payload size is ");
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
        
        PRINT_CSTSTR("CRC: ");
        PRINT_HEX("0x%04X", localCRC);
        PRINTLN;

#ifdef WITH_ACK
        if (LT.readAckStatus()) {
          PRINT_CSTSTR("Received ACK from gateway ");
          PRINT_VALUE("%d", LT.readRXSource());
          PRINTLN;
          PRINT_CSTSTR("SNR of transmitted pkt is ");
          PRINT_VALUE("%d", LT.readPacketSNRinACK());          
          PRINTLN;
        }
        else {
          PRINTLN_CSTSTR("No ACK received");
        }
#endif
      }
      else
      {
        endSend = millis();
        //if here there was an error transmitting packet
        uint16_t IRQStatus;
        IRQStatus = LT.readIrqStatus();                      
        PRINT_CSTSTR("Send Error - IRQ: ");
        PRINT_HEX("0x%04X", IRQStatus);
        PRINTLN;
        LT.printIrqStatus(); 
      }

#ifdef WITH_EEPROM
      // save packet number for next packet in case of reboot    
      my_sx1272config.seq = LT.readTXSeqNo();    
      EEPROM.put(0, my_sx1272config);
#if defined ARDUINO_ESP8266_ESP01 || defined ARDUINO_ESP8266_NODEMCU || defined ESP8266
      EEPROM.commit();
#endif
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
      PRINTLN_CSTSTR("ms");
    }

#ifdef LOW_POWER
    // Enter sleep mode (whether we transmitted or had sensor error)
    PRINT_CSTSTR("Switch to power saving mode\n");

    //CONFIGURATION_RETENTION=RETAIN_DATA_RAM on SX128X
    //parameter is ignored on SX127X
    LT.setSleep(CONFIGURATION_RETENTION);
      
    FLUSHOUTPUT;    
#ifdef LOW_POWER_TEST
    delay(10000);
#else            
    delay(10);
#endif

#ifdef __SAMD21G18A__
    // For Arduino M0 or Zero we use the built-in RTC
    rtc.setTime(17, 0, 0);
    rtc.setDate(1, 1, 2000);
    rtc.setAlarmTime(17, idlePeriodInMin, 0);
    rtc.enableAlarm(rtc.MATCH_HHMMSS);
    rtc.standbyMode();
    
    LowPower.standby();
    
    PRINT_CSTSTR("SAMD21G18A wakes up from standby\n");      
    FLUSHOUTPUT;
#else

#if defined __MK20DX256__ || defined __MKL26Z64__ || defined __MK64FX512__ || defined __MK66FX1M0__
    // warning, setTimer accepts value from 1ms to 65535ms max
    // by default, LOW_POWER_PERIOD is 60s for those microcontrollers      
    timer.setTimer(LOW_POWER_PERIOD*1000);
#endif

    nCycle = idlePeriodInMin*60/LOW_POWER_PERIOD;
              
    for (uint8_t i=0; i<nCycle; i++) {  

#if defined ARDUINO_AVR_MEGA2560 || defined ARDUINO_AVR_PRO || defined ARDUINO_AVR_NANO || defined ARDUINO_AVR_UNO || defined ARDUINO_AVR_MINI || defined __AVR_ATmega32U4__ 
        // ATmega2560, ATmega328P, ATmega168, ATmega32U4
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
        
#elif defined __MK20DX256__ || defined __MKL26Z64__ || defined __MK64FX512__ || defined __MK66FX1M0__
        // Teensy31/32 & TeensyLC
#ifdef LOW_POWER_HIBERNATE
        Snooze.hibernate(sleep_config);
#else            
        Snooze.deepSleep(sleep_config);
#endif
#elif defined ARDUINO_ESP8266_ESP01 || defined ARDUINO_ESP8266_NODEMCU || defined ESP8266
        //in microseconds
        ESP.deepSleep(LOW_POWER_PERIOD*1000*1000);
#else
        // use the delay function
        delay(LOW_POWER_PERIOD*1000);
#endif                        
        PRINT_CSTSTR(".");
        FLUSHOUTPUT;
        delay(1);                        
    }
#endif      
    
#else
    PRINT_VALUE("%ld", nextTransmissionTime);
    PRINTLN;
    PRINT_CSTSTR("Will send next value at ");
    nextTransmissionTime = millis() + (unsigned long)idlePeriodInMin*60*1000;
    PRINT_VALUE("%ld", nextTransmissionTime);
    PRINTLN;
  }
#endif

  LT.wake();
}