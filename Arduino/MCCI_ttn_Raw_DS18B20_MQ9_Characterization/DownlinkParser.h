////////////////////////////////////////////////////////////////
// Downlink Parser - LoRaWAN / MCCI LMIC version
//
// Supported commands (ASCII, sent from TTN application):
//   /@C<index>#  — change characterization config (0 to NUM_TEST_PARAMS-1)
//   /@I<secs>#   — change transmission interval (min 5 s)
////////////////////////////////////////////////////////////////

#ifndef DOWNLINK_PARSER_H
#define DOWNLINK_PARSER_H

extern uint8_t currentParamIndex;
extern unsigned int idlePeriodInSec;
extern const NetworkTestParams testParams[];
extern const uint8_t NUM_TEST_PARAMS;

long getCmdValue(int &i, char* cmdstr, char* strBuff = NULL) {
    char seqStr[10] = "******";
    int j = 0;
    while ((char)cmdstr[i] != '#' && (i < (int)strlen(cmdstr)) && j < (int)(sizeof(seqStr) - 1)) {
        seqStr[j++] = (char)cmdstr[i++];
    }
    seqStr[j] = '\0';
    if (strBuff) { strcpy(strBuff, seqStr); return 0; }
    return atol(seqStr);
}

bool parseDownlinkCommand(uint8_t* message, uint8_t RXPacketL, uint8_t& currentParamIndex) {
    bool configChanged = false;

    Serial.print(F("Downlink hex: "));
    for (int j = 0; j < RXPacketL; j++) {
        if (message[j] < 0x10) Serial.print('0');
        Serial.print(message[j], HEX);
        Serial.print(' ');
    }
    Serial.println();

    Serial.print(F("Downlink ASCII: "));
    for (int j = 0; j < RXPacketL; j++) {
        Serial.print((message[j] >= 32 && message[j] <= 126) ? (char)message[j] : '.');
    }
    Serial.println();

    if (RXPacketL < 4 || message[0] != '/' || message[1] != '@') {
        Serial.println(F("Downlink: not a command (expected /@X<val>#)"));
        return false;
    }

    char cmdstr[20];
    int copyLen = min((int)sizeof(cmdstr) - 1, (int)RXPacketL);
    strncpy(cmdstr, (char*)message, copyLen);
    cmdstr[copyLen] = '\0';

    int i = 2;
    long cmdValue;

    Serial.print(F("Parsing command: "));
    Serial.println(cmdstr);

    switch (cmdstr[i]) {

        case 'C':
            i++;
            cmdValue = getCmdValue(i, cmdstr);
            if (cmdValue >= 0 && cmdValue < NUM_TEST_PARAMS) {
                currentParamIndex = (uint8_t)cmdValue;
                Serial.print(F("Config -> index "));
                Serial.print(currentParamIndex);
                Serial.print(F(" ("));
                Serial.print(testParams[currentParamIndex].name);
                Serial.println(')');
                configChanged = true;
            } else {
                Serial.print(F("Invalid config index: "));
                Serial.print(cmdValue);
                Serial.print(F(" (valid: 0-"));
                Serial.print(NUM_TEST_PARAMS - 1);
                Serial.println(')');
            }
            break;

        case 'I':
            i++;
            cmdValue = getCmdValue(i, cmdstr);
            if (cmdValue >= 5) {
                idlePeriodInSec = (unsigned int)cmdValue;
                Serial.print(F("Interval -> "));
                Serial.print(idlePeriodInSec);
                Serial.println(F(" s"));
                configChanged = true;
            } else {
                Serial.println(F("Interval too short (min 5 s)"));
            }
            break;

        default:
            Serial.print(F("Unknown command: "));
            Serial.println(cmdstr[i]);
            break;
    }

    return configChanged;
}

#endif // DOWNLINK_PARSER_H
