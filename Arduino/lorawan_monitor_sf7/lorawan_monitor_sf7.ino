/*******************************************************************************
 * LoRaWAN Packet Monitor for Single-Channel Gateway
 * 
 * Purpose: Monitor LoRaWAN traffic between your end device and gateway
 * Frequency: 868.1 MHz
 * Data Rate: SF7BW125
 * 
 * This sketch captures RAW LoRaWAN packets (encrypted) and displays them
 * with RSSI, SNR, and timing information to verify gateway downlinks.
 * 
 * CONFIGURATION:
 * Set MONITOR_DOWNLINKS to choose what to monitor:
 *   false = Monitor UPLINKS (normal IQ - packets from node to gateway)
 *   true  = Monitor DOWNLINKS (inverted IQ - packets from gateway to node)
 * 
 * Based on MCCI LMIC raw example
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// ============================================================================
// CONFIGURATION: Choose what to monitor
// ============================================================================
#define MONITOR_DOWNLINKS true  // false = uplinks, true = downlinks
// ============================================================================

// Pin mapping - ADJUST TO YOUR HARDWARE
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},  // DIO0, DIO1, DIO2
};

// Callbacks required by LMIC but not used in raw mode
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
void onEvent (ev_t ev) { }

// Global variables for packet statistics
uint32_t packetCount = 0;
uint32_t lastPacketTime = 0;

// Enable RX mode with callback
void rx(osjobcb_t func) {
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime(); // RX _now_
  // Enable continuous RX - will trigger callback when packet received
  os_radio(RADIO_RXON);
}

// Callback executed when a packet is received
static void rx_func (osjob_t* job) {
  uint32_t currentTime = millis();
  packetCount++;
  
  Serial.println();
  Serial.println(F("========================================"));
  Serial.print(F("PACKET #"));
  Serial.print(packetCount);
  Serial.print(F(" @ "));
  Serial.print(currentTime);
  Serial.println(F(" ms"));
  
  // Calculate time since last packet
  if (lastPacketTime > 0) {
    Serial.print(F("Time since last: "));
    Serial.print(currentTime - lastPacketTime);
    Serial.println(F(" ms"));
  }
  lastPacketTime = currentTime;
  
  // Display packet length
  Serial.print(F("Length: "));
  Serial.print(LMIC.dataLen);
  Serial.println(F(" bytes"));
  
  // Display RSSI (subtract RSSI_OFF which is 64)
  Serial.print(F("RSSI: "));
  Serial.print(LMIC.rssi - 64);
  Serial.println(F(" dBm"));
  
  // Display SNR (SNR is stored as SNR*4)
  Serial.print(F("SNR: "));
  Serial.print((int8_t)(LMIC.snr / 4));
  Serial.println(F(" dB"));
  
  // Display raw packet in HEX format
  Serial.println(F("Raw Packet (HEX):"));
  for(uint8_t i = 0; i < LMIC.dataLen; i++) {
    if (LMIC.frame[i] < 0x10) Serial.print('0');
    Serial.print(LMIC.frame[i], HEX);
    Serial.print(' ');
    if ((i + 1) % 16 == 0) Serial.println();  // New line every 16 bytes
  }
  Serial.println();
  
  // Decode LoRaWAN header (first few bytes are unencrypted)
  if (LMIC.dataLen >= 12) {
    Serial.println(F("LoRaWAN Header Info:"));
    
    // MHDR - MAC Header (1 byte)
    uint8_t mhdr = LMIC.frame[0];
    uint8_t mtype = (mhdr >> 5) & 0x07;
    Serial.print(F("  MHDR: 0x"));
    if (mhdr < 0x10) Serial.print('0');
    Serial.print(mhdr, HEX);
    Serial.print(F(" - Type: "));
    
    switch(mtype) {
      case 0: Serial.println(F("Join Request")); break;
      case 1: Serial.println(F("Join Accept")); break;
      case 2: Serial.println(F("Unconfirmed Data Up")); break;
      case 3: Serial.println(F("Unconfirmed Data Down")); break;
      case 4: Serial.println(F("Confirmed Data Up")); break;
      case 5: Serial.println(F("Confirmed Data Down")); break;
      default: Serial.println(F("Unknown")); break;
    }
    
    // DevAddr (4 bytes, LSB first) - only for Data messages
    if (mtype >= 2 && mtype <= 5) {
      Serial.print(F("  DevAddr: "));
      for(int8_t i = 4; i >= 1; i--) {
        if (LMIC.frame[i] < 0x10) Serial.print('0');
        Serial.print(LMIC.frame[i], HEX);
      }
      Serial.println();
      
      // FCtrl (1 byte)
      uint8_t fctrl = LMIC.frame[5];
      Serial.print(F("  FCtrl: 0x"));
      if (fctrl < 0x10) Serial.print('0');
      Serial.print(fctrl, HEX);
      
      if (mtype <= 3) {  // Uplink
        Serial.print(F(" - ADR:"));
        Serial.print((fctrl & 0x80) ? 1 : 0);
        Serial.print(F(" ADRACKReq:"));
        Serial.print((fctrl & 0x40) ? 1 : 0);
        Serial.print(F(" ACK:"));
        Serial.print((fctrl & 0x20) ? 1 : 0);
      } else {  // Downlink
        Serial.print(F(" - ADR:"));
        Serial.print((fctrl & 0x80) ? 1 : 0);
        Serial.print(F(" ACK:"));
        Serial.print((fctrl & 0x20) ? 1 : 0);
        Serial.print(F(" FPending:"));
        Serial.print((fctrl & 0x10) ? 1 : 0);
      }
      Serial.print(F(" FOptsLen:"));
      Serial.println(fctrl & 0x0F);
      
      // FCnt (2 bytes, LSB first)
      uint16_t fcnt = LMIC.frame[6] | (LMIC.frame[7] << 8);
      Serial.print(F("  FCnt: "));
      Serial.println(fcnt);
      
      // Note: FPort and payload are encrypted
      Serial.println(F("  [FPort and payload encrypted]"));
    }
  }
  
  Serial.println(F("========================================"));
  Serial.println();
  
  // CRITICAL: Restart RX to continue monitoring
  rx(rx_func);
}

void setup() {
  // Wait for serial port
  Serial.begin(38400);
  while (!Serial) delay(10);
  
  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("LoRaWAN Packet Monitor"));
  Serial.println(F("========================================"));
  Serial.println(F("Frequency: 868.1 MHz"));
  Serial.println(F("SF: 7, BW: 125 kHz, CR: 4/5"));
  Serial.println();
  
#if MONITOR_DOWNLINKS
  Serial.println(F("Mode: DOWNLINK monitoring (Inverted IQ)"));
  Serial.println(F("Will capture packets FROM gateway TO node"));
#else
  Serial.println(F("Mode: UPLINK monitoring (Normal IQ)"));
  Serial.println(F("Will capture packets FROM node TO gateway"));
#endif
  
  Serial.println(F("========================================"));
  Serial.println();

  // Initialize LMIC
  os_init();
  
  // Configure for EU868 SF7 on 868.1 MHz
  LMIC.freq = 869524992;  // 868.1 MHz - your single channel
  LMIC.datarate = DR_SF12;  // SF7 - matching your node settings
  LMIC.txpow = 14;         // Not used but required
  
  // Set IQ inversion based on what we want to monitor
  // false (normal IQ) = monitor uplinks
  // true (inverted IQ) = monitor downlinks
  LMIC.noRXIQinversion = !MONITOR_DOWNLINKS;
  
  // Set radio parameters: SF7, BW125, CR4/5
  LMIC.rps = updr2rps(LMIC.datarate);
  
  Serial.println(F("Radio configured and listening..."));
  Serial.println();
  
  // Start continuous RX
  rx(rx_func);
}

void loop() {
  // Execute LMIC scheduled jobs and events
  os_runloop_once();
}
