////////////////////////////////////////////////////////////////
// Downlink Parser for Fake DHT22+MQ9+Dust
//
// Supported commands (ASCII, sent from TTN console):
//   /@I<secs>#     — change TX interval (min 30 s)
//   /@S<minutes>#  — sleep (go silent) for N minutes (1–720)
//   /@R#           — software restart (watchdog reset)
//   /@Z#           — factory reset (clear EEPROM, reset counters, restart)
////////////////////////////////////////////////////////////////

#ifndef DOWNLINK_PARSER_H
#define DOWNLINK_PARSER_H

enum DownlinkAction : uint8_t {
    DL_NONE = 0,
    DL_INTERVAL_CHANGED,
    DL_SLEEP,
    DL_RESTART,
    DL_FACTORY_RESET
};

struct DownlinkResult {
    DownlinkAction action;
    unsigned long  value;   // new interval (s) or sleep duration (min)
};

// Parse a decimal integer from cmdstr starting at position i, up to '#'
static long getCmdValue(int &i, char* cmdstr) {
    char seqStr[10] = "******";
    int j = 0;
    while ((char)cmdstr[i] != '#' && (i < (int)strlen(cmdstr)) && j < (int)(sizeof(seqStr) - 1)) {
        seqStr[j++] = (char)cmdstr[i++];
    }
    seqStr[j] = '\0';
    return atol(seqStr);
}

// Parse a downlink command. message[] must be null-terminated.
DownlinkResult parseDownlinkCommand(uint8_t* message, uint8_t len) {
    DownlinkResult result = { DL_NONE, 0 };

    // Debug hex dump
    Serial.print(F("Downlink hex: "));
    for (int j = 0; j < len; j++) {
        if (message[j] < 0x10) Serial.print('0');
        Serial.print(message[j], HEX);
        Serial.print(' ');
    }
    Serial.println();

    // Debug ASCII
    Serial.print(F("Downlink ASCII: "));
    for (int j = 0; j < len; j++) {
        Serial.print((message[j] >= 32 && message[j] <= 126) ? (char)message[j] : '.');
    }
    Serial.println();

    // Commands: /@X<value>#  (min 3 chars for no-value cmds like /@R#)
    if (len < 3 || message[0] != '/' || message[1] != '@') {
        Serial.println(F("Not a command (expected /@X...)"));
        return result;
    }

    char cmdstr[20];
    int copyLen = min((int)sizeof(cmdstr) - 1, (int)len);
    strncpy(cmdstr, (char*)message, copyLen);
    cmdstr[copyLen] = '\0';

    int i = 2;
    long cmdValue;

    Serial.print(F("Command: ")); Serial.println(cmdstr);

    switch (cmdstr[i]) {

        case 'I':   // /@I<secs># — set TX interval (min 30 s)
            i++;
            cmdValue = getCmdValue(i, cmdstr);
            if (cmdValue >= 30) {
                result.action = DL_INTERVAL_CHANGED;
                result.value = (unsigned long)cmdValue;
                Serial.print(F("Interval -> ")); Serial.print(result.value); Serial.println(F(" s"));
            } else {
                Serial.println(F("Interval too short (min 30 s)"));
            }
            break;

        case 'S':   // /@S<minutes># — sleep for N minutes (1–720)
            i++;
            cmdValue = getCmdValue(i, cmdstr);
            if (cmdValue >= 1 && cmdValue <= 720) {
                result.action = DL_SLEEP;
                result.value = (unsigned long)cmdValue;
                Serial.print(F("Sleep -> ")); Serial.print(result.value); Serial.println(F(" min"));
            } else {
                Serial.println(F("Sleep range: 1-720 min"));
            }
            break;

        case 'R':   // /@R# — software restart
            result.action = DL_RESTART;
            Serial.println(F("Restart requested"));
            break;

        case 'Z':   // /@Z# — factory reset
            result.action = DL_FACTORY_RESET;
            Serial.println(F("Factory reset requested"));
            break;

        default:
            Serial.print(F("Unknown command: "));
            Serial.println(cmdstr[i]);
            break;
    }

    return result;
}

#endif // DOWNLINK_PARSER_H
