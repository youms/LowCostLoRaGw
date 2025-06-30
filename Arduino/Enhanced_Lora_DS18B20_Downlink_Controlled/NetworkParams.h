////////////////////////////////////////////////////////////////
// Network Parameters Module - FIXED VERSION
// Defines the 16 network characterization configurations
// Based on Enhanced_Lora_DS18B20_SX12XXX_Characterization.ino
////////////////////////////////////////////////////////////////

#ifndef NETWORK_PARAMS_H
#define NETWORK_PARAMS_H

uint8_t currentSF;
uint8_t currentBW;
uint8_t currentCR;

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
  #define BW_500_VAL  LORA_BW_500    // 9 for SX126X 
  #define CR_4_5_VAL  LORA_CR_4_5    // 0x01 for SX126X
#endif
#ifdef SX127X
  #define BW_125_VAL  LORA_BW_125    // 144 for SX127X 
  #define BW_500_VAL  LORA_BW_500    // 176 for SX127X
  #define CR_4_5_VAL  LORA_CR_4_5    // 0x02 for SX127X
#endif
#ifdef SX128X
  #define BW_125_VAL  LORA_BW_0200   // 0x18 closest to 125kHz for SX128X
  #define BW_500_VAL  LORA_BW_0800   // 0x1A closest to 500kHz for SX128X
  #define CR_4_5_VAL  LORA_CR_4_5    // 0x01 for SX128X
#endif

// Test parameter sets (16 configurations total)
const NetworkTestParams testParams[] = {
  // MIN configurations (SF7)
  {LORA_SF7, BW_125_VAL, CR_4_5_VAL, 20, "MIN-SF7-BW125-T20"},
  {LORA_SF7, BW_500_VAL, CR_4_5_VAL, 20, "MIN-SF7-BW500-T20"},
  {LORA_SF7, BW_125_VAL, CR_4_5_VAL, 50, "MIN-SF7-BW125-T50"},
  {LORA_SF7, BW_125_VAL, CR_4_5_VAL, 80, "MIN-SF7-BW125-T80"},
  
  // MEAN configurations (SF9)
  {LORA_SF9, BW_125_VAL, CR_4_5_VAL, 20, "MEAN-SF9-BW125-T20"},
  {LORA_SF9, BW_500_VAL, CR_4_5_VAL, 20, "MEAN-SF9-BW500-T20"},
  {LORA_SF9, BW_125_VAL, CR_4_5_VAL, 50, "MEAN-SF9-BW125-T50"},
  {LORA_SF9, BW_125_VAL, CR_4_5_VAL, 80, "MEAN-SF9-BW125-T80"},
  
  // MAX configurations (SF12)
  {LORA_SF12, BW_125_VAL, CR_4_5_VAL, 20, "MAX-SF12-BW125-T20"},
  {LORA_SF12, BW_500_VAL, CR_4_5_VAL, 20, "MAX-SF12-BW500-T20"},
  {LORA_SF12, BW_125_VAL, CR_4_5_VAL, 50, "MAX-SF12-BW125-T50"},
  {LORA_SF12, BW_125_VAL, CR_4_5_VAL, 80, "MAX-SF12-BW125-T80"},
  
  // EXTRA configurations for comprehensive testing
  {LORA_SF8, BW_125_VAL, CR_4_5_VAL, 30, "EXTRA-SF8-BW125-T30"},
  {LORA_SF10, BW_125_VAL, CR_4_5_VAL, 40, "EXTRA-SF10-BW125-T40"},
  {LORA_SF11, BW_125_VAL, CR_4_5_VAL, 60, "EXTRA-SF11-BW125-T60"},
  {LORA_SF7, BW_500_VAL, CR_4_5_VAL, 100, "EXTRA-SF7-BW500-T100"}
};

const uint8_t NUM_TEST_PARAMS = sizeof(testParams) / sizeof(testParams[0]);

// Function to update LoRa parameters
void updateLoRaParams(const NetworkTestParams& params) {
  PRINT_CSTSTR("Updating LoRa parameters to: ");
  Serial.println(params.name);
  
  // CRITICAL FIX: Update global variables first
  currentSF = params.sf;
  currentBW = params.bw;
  currentCR = params.cr;
  
  // Set new modulation parameters
#if defined SX126X || defined SX127X
  LT.setModulationParams(currentSF, Bandwidth, CodeRate, Optimisation);
#endif
#ifdef SX128X
  LT.setModulationParams(currentSF, Bandwidth, CodeRate);
#endif

  PRINT_CSTSTR("SF: ");
  switch(params.sf) {
    case LORA_SF7:  PRINTLN_CSTSTR("7"); break;
    case LORA_SF8:  PRINTLN_CSTSTR("8"); break;
    case LORA_SF9:  PRINTLN_CSTSTR("9"); break;
    case LORA_SF10: PRINTLN_CSTSTR("10"); break;
    case LORA_SF11: PRINTLN_CSTSTR("11"); break;
    case LORA_SF12: PRINTLN_CSTSTR("12"); break;
  }
  
  PRINT_CSTSTR("BW: ");
  switch(params.bw) {
#ifdef SX127X
    case LORA_BW_125:  PRINTLN_CSTSTR("125KHz"); break;
    case LORA_BW_500:  PRINTLN_CSTSTR("500KHz"); break;
#endif
#ifdef SX126X
    case LORA_BW_125:  PRINTLN_CSTSTR("125KHz"); break;
    case LORA_BW_500:  PRINTLN_CSTSTR("500KHz"); break;
#endif
#ifdef SX128X
    case LORA_BW_0200: PRINTLN_CSTSTR("203KHz"); break;
    case LORA_BW_0800: PRINTLN_CSTSTR("812KHz"); break;
#endif
  }

  
  PRINT_CSTSTR("CR: 4/5, Target payload: ");
  PRINT_VALUE("%d", params.payloadSize);
  PRINTLN_CSTSTR(" bytes");
}

// LoRa configuration function - from working code
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

  // CRITICAL FIX: Initialize global variables first
  currentSF = testParams[0].sf;
  currentBW = testParams[0].bw;
  currentCR = testParams[0].cr;

  //set LoRa modem parameters - Use initial configuration from testParams[0]
#if defined SX126X || defined SX127X
  LT.setModulationParams(currentSF, Bandwidth, CodeRate, Optimisation);
#endif
#ifdef SX128X
  LT.setModulationParams(currentSF, Bandwidth, CodeRate);
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

#endif // NETWORK_PARAMS_H