////////////////////////////////////////////////////////////////
// Downlink Parser Module - FIXED VERSION
// Handles downlink command parsing and receive window processing
// Based on Arduino_LoRa_SX12XX_DS18B20.ino with fixes for reception issues
////////////////////////////////////////////////////////////////

#ifndef DOWNLINK_PARSER_H
#define DOWNLINK_PARSER_H

#ifdef WITH_RCVW
// CRITICAL FIX: Increased delay before opening receive window
// Original was 1000ms, but network delays might require more time
// #define DELAY_BEFORE_RCVW 1000

// Forward declarations for global variables and functions from main sketch
extern uint8_t currentParamIndex;
extern const NetworkTestParams testParams[];
extern const uint8_t NUM_TEST_PARAMS;

// Forward declarations of LoRa configuration constants from NetworkParams.h
extern const uint32_t Offset;

// CRITICAL FIX: Forward declarations for global LoRa parameters (like working code)
extern uint8_t currentSF;
extern uint8_t currentBW; 
extern uint8_t currentCR;


// Function to parse command value from downlink string
long getCmdValue(int &i, char* cmdstr, char* strBuff=NULL) {
  char seqStr[10]="******";
  
  int j=0;
  // character '#' will indicate end of cmd value
  while ((char)cmdstr[i]!='#' && ( i < strlen(cmdstr) && j<strlen(seqStr))) {
    seqStr[j]=(char)cmdstr[i];
    i++;
    j++;
  }
  
  // put the null character at the end
  seqStr[j]='\0';
  
  if (strBuff) {
    strcpy(strBuff, seqStr);        
  }
  else
    return (atol(seqStr));
}


// Function based on LoRaWAN specification + empirical gateway behavior
uint16_t getAdaptiveDelay(uint8_t currentParamIndex) {
  uint8_t currentSF = testParams[currentParamIndex].sf;
  
  // Base LoRaWAN RX1 timing (1000ms) minus empirical gateway processing overhead
  switch(currentSF) {
    case LORA_SF7:  return 500;   // 1000ms - 500ms processing overhead
    case LORA_SF8:  return 600;   // 1000ms - 400ms processing overhead  
    case LORA_SF9:  return 700;   // 1000ms - 300ms processing overhead
    case LORA_SF10: return 800;   // 1000ms - 200ms processing overhead
    case LORA_SF11: return 900;   // 1000ms - 100ms processing overhead
    case LORA_SF12: return 1000;  // 1000ms - 0ms (gateway ready exactly at spec)
    default:        return 1000;  
  }
}



/*
// Function to calculate optimal receive timeout based on current configuration
uint16_t calculateOptimalTimeout(uint8_t rxWindow, uint8_t currentParamIndex) {
  uint16_t timeout;
  
  // Get current spreading factor from configuration
  uint8_t currentSF = testParams[currentParamIndex].sf;
  
  // Base timeout calculation based on spreading factor
  // These values are empirically determined for reliable reception
  switch(currentSF) {
    case LORA_SF7:  timeout = 300; break;  // Fast, short packets
    case LORA_SF8:  timeout = 350; break;
    case LORA_SF9:  timeout = 400; break;
    case LORA_SF10: timeout = 450; break;
    case LORA_SF11: timeout = 550; break;
    case LORA_SF12: timeout = 600; break;  // Slow, long packets
    default:        timeout = 500; break;  // Safe default
  }
  
  // RX2 window always uses SF12, so adjust timeout accordingly
  if (rxWindow == 2) {
    timeout = 650;  // SF12 needs more time + some margin
  }
  
  // Add small margin for timing variations
  timeout += 50;
  
  PRINT_CSTSTR("Calculated timeout for RX");
  PRINT_VALUE("%d", rxWindow);
  PRINT_CSTSTR(" with SF");
  switch(currentSF) {
    case LORA_SF7: PRINT_CSTSTR("7"); break;
    case LORA_SF8: PRINT_CSTSTR("8"); break;
    case LORA_SF9: PRINT_CSTSTR("9"); break;
    case LORA_SF10: PRINT_CSTSTR("10"); break;
    case LORA_SF11: PRINT_CSTSTR("11"); break;
    case LORA_SF12: PRINT_CSTSTR("12"); break;
  }
  PRINT_CSTSTR(": ");
  PRINT_VALUE("%d", timeout);
  PRINTLN_CSTSTR("ms");
  
  return timeout;
}
*/


// ENHANCED Function to process downlink receive window with debugging
uint8_t processDownlinkWindow(long endSend) {
#ifdef LORAWAN 
  uint8_t rxw_max=2;
#else
  uint8_t rxw_max=1;
#endif

  PRINT_CSTSTR("Starting downlink receive window process\n");
  PRINT_CSTSTR("End send time: ");
  PRINT_VALUE("%ld", endSend);
  PRINTLN;

#ifdef INVERTIQ_ON_RX
  // CRITICAL FIX: Ensure I/Q inversion is properly set for RX
  PRINTLN_CSTSTR("Inverting I/Q for RX");
  LT.invertIQ(true);
#endif

  uint8_t rxw=1;
  uint8_t RXPacketL = 0;
  uint16_t DELAY_BEFORE_RCVW = getAdaptiveDelay(currentParamIndex);
  
  do {
    PRINT_CSTSTR("Wait for ");
    PRINT_VALUE("%d", (endSend + rxw * DELAY_BEFORE_RCVW) - millis());
    PRINT_CSTSTR("ms");

    
    //target 1s which is RX1 for LoRaWAN in most regions
    //then target 1s more which is RX2 for LoRaWAN in most regions
    while (millis()-endSend < rxw*DELAY_BEFORE_RCVW) 
      ;

    /*
    // With this more precise timing:
    unsigned long targetTime = endSend + (unsigned long)rxw * DELAY_BEFORE_RCVW;
    while (millis() < targetTime)
      ;
    */
    PRINTLN;
    PRINT_CSTSTR("Opening receive window RX");
    PRINT_VALUE("%d", rxw);
    PRINTLN;
    
    // CRITICAL FIX: Longer timeout and better error handling
    uint16_t rxTimeout = 850; // Use same timeout as working code
    
    PRINT_CSTSTR("Waiting for packet with timeout: ");
    PRINT_VALUE("%d", rxTimeout);
    PRINTLN_CSTSTR("ms");
    PRINTLN;

    // wait for incoming packets
    RXPacketL = LT.receiveAddressed(message, sizeof(message), rxTimeout, WAIT_RX);
    
    PRINT_CSTSTR("Received packet length: ");
    PRINT_VALUE("%d", RXPacketL);
    PRINTLN;
    
    //we received something in RX1
    if (RXPacketL && rxw==1) {
      PRINTLN_CSTSTR("Packet received in RX1 - skipping RX2");
      rxw=rxw_max+1;
    }
    else {
      // try RX2 only if we are in LoRaWAN mode and nothing has been received in RX1
      if (++rxw<=rxw_max) {
        PRINTLN;
        PRINT_CSTSTR("Switching to RX");
        PRINT_VALUE("%d", rxw);
        PRINTLN_CSTSTR(" window configuration");
        
#ifdef BAND868
        //change freq to 869.525 as we are targeting RX2 window
        PRINT_CSTSTR("Set downlink frequency to 869.525MHz\n");
        LT.setRfFrequency(869525000, Offset);
#elif defined BAND900
        //TODO for 900MHz band
        PRINT_CSTSTR("900MHz RX2 not implemented\n");
#elif defined BAND433
        //change freq to 434.665 as we are targeting RX2 window
        PRINT_CSTSTR("Set downlink frequency to 434.665MHz\n");
        LT.setRfFrequency(434665000, Offset);
#elif defined BAND2400
        //no changes in 2400 band              
        PRINT_CSTSTR("No frequency change for 2400MHz band\n");
#endif
        //change to SF12 as we are targeting RX2 window
        //valid for EU868 and EU433 band        
        PRINT_CSTSTR("Set to SF12\n");
#if defined SX126X || defined SX127X
        LT.setModulationParams(LORA_SF12, Bandwidth, CodeRate, Optimisation);
#endif
#ifdef SX128X
        LT.setModulationParams(LORA_SF12, Bandwidth, CodeRate);
#endif            
      }
      else {
#ifdef LORAWAN              
        //set back to the reception frequency
        PRINT_CSTSTR("Set back frequency\n");
#ifdef MY_FREQUENCY
        LT.setRfFrequency(MY_FREQUENCY, Offset);
#else  
        LT.setRfFrequency(DEFAULT_CHANNEL, Offset);                   
#endif              
        //set back the SF - CRITICAL FIX: Use global variables like the working code
        PRINT_CSTSTR("Set back SF\n");
        
#if defined SX126X || defined SX127X
        LT.setModulationParams(currentSF, currentBW, currentCR, Optimisation);
#endif
#ifdef SX128X
        LT.setModulationParams(currentSF, currentBW, currentCR);
#endif
#endif              
      }
    }
  } while (rxw<=rxw_max);

#ifdef INVERTIQ_ON_RX
  // Restore I/Q to normal
  PRINTLN_CSTSTR("I/Q back to normal");
  LT.invertIQ(false);
#endif   

  if (!RXPacketL) {
    PRINTLN_CSTSTR("No downlink packet received in any window");
    PRINTLN;
  } else {
    PRINT_CSTSTR("Successfully received downlink packet: ");
    PRINT_VALUE("%d", RXPacketL);
    PRINTLN_CSTSTR(" bytes");
  }
  
  return RXPacketL;
}

// Function to parse downlink commands - ENHANCED with better error handling
bool parseDownlinkCommand(uint8_t* message, uint8_t RXPacketL, uint8_t& currentParamIndex, uint8_t& node_addr) {
  bool configChanged = false;
  int i = 0;
  long cmdValue;

#ifndef LORAWAN
  char print_buff[50];

  sprintf((char*)print_buff, "^p%d,%d,%d,%d,%d,%d,%d\n",        
             LT.readRXDestination(),
             LT.readRXPacketType(),                   
             LT.readRXSource(),
             LT.readRXSeqNo(),                   
             RXPacketL,
             LT.readPacketSNR(),
             LT.readPacketRSSI());                                   
  PRINT_STR("%s",(char*)print_buff);         

  PRINT_CSTSTR("frame hex: "); 
  
  for (i=0; i<RXPacketL; i++) {               
    if (message[i]<16)
      PRINT_CSTSTR("0");
    PRINT_HEX("%X", message[i]);
    PRINT_CSTSTR(" ");       
  }
  PRINTLN;

  message[RXPacketL]=(char)'\0'; // Ensure null termination
  // in non-LoRaWAN, we try to print the characters
  PRINT_CSTSTR("Received message: ");
  PRINT_STR("%s",(char*)message);
  PRINTLN;
  i=0;            
#else
  // For LoRaWAN mode, decode the packet first
  extern int local_lorawan_decode_pkt(uint8_t* message, uint8_t len);
  i=local_lorawan_decode_pkt(message, RXPacketL);
                    
  //set the null character at the end of the payload in case it is a string
  if (RXPacketL > 4) {
    message[RXPacketL-4]=(char)'\0';
  } else {
    message[RXPacketL]=(char)'\0';
  }       
#endif

  PRINTLN;
  FLUSHOUTPUT;
 
  // commands have following format /@C5# for configuration 5
  if (i>=0 && i<RXPacketL && message[i]=='/' && i+1<RXPacketL && message[i+1]=='@') {

    char cmdstr[20]; // Increased buffer size for safety
    // copy the downlink payload, up to sizeof(cmdstr)-1 to ensure null termination
    int copyLen = min((int)sizeof(cmdstr)-1, RXPacketL-i);
    strncpy(cmdstr,(char*)(message+i), copyLen); 
    cmdstr[copyLen] = '\0'; // Ensure null termination
        
    PRINT_CSTSTR("Parsing downlink command: ");
    PRINT_STR("%s", cmdstr);
    PRINTLN;      
    i=2;   

    if (i < strlen(cmdstr)) {
      switch ((char)cmdstr[i]) {

#ifndef LORAWAN
        // set the node's address, /@A10# to set the address to 10 for instance
        case 'A': 
          i++;
          cmdValue=getCmdValue(i, cmdstr);
          
          // cannot set addr greater than 255
          if (cmdValue > 255)
            cmdValue = 255;
          // cannot set addr lower than 2 since 0 is broadcast and 1 is for gateway
          if (cmdValue < 2)
            cmdValue = node_addr;
          // set node addr        
          node_addr=cmdValue; 
          
          LT.setDevAddr(node_addr);
          PRINT_CSTSTR("Set LoRa node addr to ");
          PRINT_VALUE("%d", node_addr);  
          PRINTLN;     

          configChanged = true;
          break;        
#endif

        // Configuration command /@C<index># to set configuration 0-15
        case 'C':
          i++;
          cmdValue=getCmdValue(i, cmdstr);
          
          // Validate configuration index (0-15)
          extern const uint8_t NUM_TEST_PARAMS;
          if (cmdValue >= 0 && cmdValue < NUM_TEST_PARAMS) {
            currentParamIndex = cmdValue;
            
            PRINT_CSTSTR("Set configuration to index ");
            PRINT_VALUE("%d", currentParamIndex);
            PRINT_CSTSTR(" (");
            extern const NetworkTestParams testParams[];
            PRINT_STR("%s", testParams[currentParamIndex].name);
            PRINTLN_CSTSTR(")");
            
            configChanged = true;
          } else {
            PRINT_CSTSTR("Invalid configuration index: ");
            PRINT_VALUE("%ld", cmdValue);
            PRINT_CSTSTR(" (valid range: 0-");
            PRINT_VALUE("%d", NUM_TEST_PARAMS-1);
            PRINTLN_CSTSTR(")");
          }
          break;

        // Set transmission interval /@I10# to set to 10 minutes for instance
        case 'I': 
          i++;
          cmdValue=getCmdValue(i, cmdstr);

          // This is just for display - idlePeriodInMin is const in main sketch
          PRINT_CSTSTR("Transmission interval command received: ");
          PRINT_VALUE("%ld", cmdValue);  
          PRINTLN_CSTSTR(" minutes (Note: interval is fixed in this version)");         
          break;  

        // Toggle LED example /@L1# 
        case 'L': 
          i++;
          cmdValue=getCmdValue(i, cmdstr);
          
          PRINT_CSTSTR("LED toggle command: ");
          PRINT_VALUE("%ld", cmdValue);
          PRINTLN;
          
          // Add your LED control code here if needed
          break;

        default:
          PRINT_CSTSTR("Unrecognized command: ");
          if (i < strlen(cmdstr)) {
            PRINT_STR("%c", cmdstr[i]);
          }
          PRINTLN;
          break;
      }
    } else {
      PRINT_CSTSTR("Invalid command format - index out of bounds\n");
    }
  } else {
    PRINT_CSTSTR("Invalid command format (expected /@X#)\n");
    PRINT_CSTSTR("Received bytes: ");
    for (int j=0; j<RXPacketL && j<10; j++) {
      PRINT_HEX("%02X", message[j]);
      PRINT_CSTSTR(" ");
    }
    PRINTLN;
  }

  return configChanged;
}

#endif // WITH_RCVW

#endif // DOWNLINK_PARSER_H