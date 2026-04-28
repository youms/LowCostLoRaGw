/*******************************************************************************
 * Modified LoRaWAN code for interactive serial input with MCCI LMIC
 * This code sends data to TTN only when text is entered in the Serial Monitor
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define ARDUINO_LMIC_PROJECT_CONFIG_H_SUPPRESS_WARNING

// ── Single-channel configuration ─────────────────────────────────────────────
// ABP note: only SCG_init() and SCG_enforce() are called in this sketch.
// SCG_on_joined() and SCG_on_join_txcomplete() are OTAA-only and not needed
// for ABP since there is no join procedure.
//
// Override defaults here before the include if needed, e.g.:
//   #define SCG_DATARATE  DR_SF9
//   #define SCG_RX2_FREQ  434665000   // for EU433
#define SCG_RX2_DR  DR_SF9            // original ABP sketch used SF9 for RX2
#include <single_channel_lmic.h>

// ── ABP session credentials ───────────────────────────────────────────────────
// LoRaWAN Network Session Key (MSB format)
static const PROGMEM u1_t NWKSKEY[16] = {
    // 0x55, 0xF7, 0x14, 0xA6, 0x58, 0x11, 0xDE, 0xAF, 0xF4, 0xAC, 0x6B, 0x65, 0x3F, 0xA5, 0xAB, 0xAD
    0xDB, 0xAC, 0xE3, 0x0A, 0xBC, 0x8D, 0x0A, 0xEC, 0x70, 0xE2, 0xE5, 0xB8, 0x0E, 0xB4, 0x1D, 0x81
};

// LoRaWAN Application Session Key (MSB format)
static const u1_t PROGMEM APPSKEY[16] = {
    // 0xBC, 0xA5, 0x0B, 0x27, 0x85, 0x31, 0xF0, 0x28, 0xBE, 0x75, 0x9A, 0x66, 0x84, 0x15, 0x76, 0xED
    0xA9, 0xCB, 0xDB, 0x31, 0x32, 0x40, 0x2D, 0x17, 0xEE, 0xEE, 0xAC, 0x12, 0x73, 0x39, 0x24, 0x0A
};

// LoRaWAN end-device address (DevAddr) - Change this for every node!
static const u4_t DEVADDR = // 0x260BE0A2;
                             0x260B9642;

// These callbacks are only used in over-the-air activation (OTAA).
// For ABP, we leave them empty but they must be present.
void os_getArtEui(u1_t* buf) { }
void os_getDevEui(u1_t* buf) { }
void os_getDevKey(u1_t* buf) { }

// ── Serial input state ────────────────────────────────────────────────────────
#define MAX_MSG_LEN 64
char message[MAX_MSG_LEN] = "";
int  msgIndex = 0;
bool newMessage = false;
bool transmissionComplete = true;  // flag to track if previous transmission is complete

static osjob_t sendjob;

// ── Pin mapping — Dragino LoRa Shield v1.4 on Arduino Uno/Mega ───────────────
// Adjust these pins if your hardware is different!
const lmic_pinmap lmic_pins = {
    .nss  = 10,               // NSS (CS) pin
    .rxtx = LMIC_UNUSED_PIN,  // Not used on this shield
    .rst  = 9,                // Reset pin
    .dio  = {2, 6, 7},        // DIO0, DIO1, DIO2
};

// ─────────────────────────────────────────────────────────────────────────────

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(F(": "));
    switch (ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;

        case EV_JOINED:
            // For ABP this may fire when LMIC_setSession() is called internally.
            // We call SCG_on_joined() defensively to re-enforce single-channel
            // state in case the session setup altered channelMap or datarate.
            Serial.println(F("EV_JOINED"));
            {
                u4_t netid = 0;
                devaddr_t devaddr = 0;
                u1_t nwkKey[16];
                u1_t artKey[16];
                LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
                Serial.print(F("netid: "));
                Serial.println(netid, DEC);
                Serial.print(F("devaddr: "));
                Serial.println(devaddr, HEX);
                Serial.print(F("AppSKey: "));
                for (size_t i = 0; i < sizeof(artKey); ++i) {
                    if (i != 0) Serial.print(F("-"));
                    printHex2(artKey[i]);
                }
                Serial.println();
                Serial.print(F("NwkSKey: "));
                for (size_t i = 0; i < sizeof(nwkKey); ++i) {
                    if (i != 0) Serial.print(F("-"));
                    printHex2(nwkKey[i]);
                }
                Serial.println();
            }
            // Re-enforce single-channel state after session setup.
            // Handles: CFList channel injection, DR reset, link check mode.
            // See single_channel_lmic.h — SCG_on_joined() for details.
            SCG_on_joined();
            break;

        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;

        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

            // Print detailed txrxFlags for debugging
            Serial.print(F("txrxFlags: 0x"));
            Serial.println(LMIC.txrxFlags, HEX);

            // Decode which window received data
            if (LMIC.txrxFlags & TXRX_DNW1)
                Serial.println(F("Received in RX1 window"));
            else if (LMIC.txrxFlags & TXRX_DNW2)
                Serial.println(F("Received in RX2 window"));
            else if (LMIC.dataLen == 0)
                Serial.println(F("No downlink received in any window"));

            // Check for ACK
            if (LMIC.txrxFlags & TXRX_ACK)
                Serial.println(F("Received ack"));

            // Check for NACK
            if (LMIC.txrxFlags & TXRX_NACK)
                Serial.println(F("NACK - no acknowledgment"));

            // Check for port
            if (LMIC.txrxFlags & TXRX_PORT) {
                Serial.print(F("Downlink port: "));
                Serial.println(LMIC.frame[LMIC.dataBeg - 1]);
            }

            // Check for downlink data
            if (LMIC.dataLen) {
                Serial.print(F("Received "));
                Serial.print(LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
                Serial.print(F("Downlink data: "));
                for (int i = 0; i < LMIC.dataLen; i++) {
                    if (LMIC.frame[LMIC.dataBeg + i] < 0x10) Serial.print(F("0"));
                    Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
                    Serial.print(F(" "));
                }
                Serial.println();

                // Print as ASCII if printable
                Serial.print(F("Downlink data (ASCII): "));
                for (int i = 0; i < LMIC.dataLen; i++) {
                    if (LMIC.frame[LMIC.dataBeg + i] >= 32 && LMIC.frame[LMIC.dataBeg + i] <= 126) {
                        Serial.print((char)LMIC.frame[LMIC.dataBeg + i]);
                    } else {
                        Serial.print(F("."));
                    }
                }
                Serial.println();
            }

            // Set flag indicating transmission is complete
            transmissionComplete = true;
            Serial.println(F("✓ Ready for next message. Type and press Enter."));
            break;

        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            // Not expected for ABP but handled defensively.
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            SCG_on_join_txcomplete();
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned)ev);
            break;
    }
}

void do_send(osjob_t* j) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        transmissionComplete = false;
    } else {
        // Convert char array to byte array for sending
        uint8_t payload[MAX_MSG_LEN];
        int len = strlen(message);

        // Copy message to payload buffer
        for (int i = 0; i < len; i++) {
            payload[i] = (uint8_t)message[i];
        }

        // Display message details
        Serial.println(F("=== Sending Message ==="));
        Serial.print(F("Text: \""));
        Serial.print(message);
        Serial.println(F("\""));
        Serial.print(F("Length: "));
        Serial.print(len);
        Serial.println(F(" bytes"));
        Serial.print(F("Payload (hex): "));
        for (int i = 0; i < len; i++) {
            if (payload[i] < 0x10) Serial.print(F("0"));
            Serial.print(payload[i], HEX);
            Serial.print(F(" "));
        }
        Serial.println();
        Serial.print(F("Payload (ASCII): "));
        for (int i = 0; i < len; i++) {
            if (payload[i] >= 32 && payload[i] <= 126) {
                Serial.print((char)payload[i]);
            } else {
                Serial.print(F("."));
            }
        }
        Serial.println();
        Serial.println(F("======================"));

        // Prepare upstream data transmission at the next possible time
        LMIC_setTxData2(1, payload, len, 0);  // port 1, variable length, no ack
        Serial.println(F("📡 Packet queued for transmission..."));

        // Clear the message buffer
        memset(message, 0, MAX_MSG_LEN);
        msgIndex = 0;
        newMessage = false;
        transmissionComplete = false;
    }
    // Next TX is NOT scheduled automatically - will only happen when new data is entered
}

void setup() {
    delay(3000);  // Give time to open serial monitor
    Serial.begin(38400);
    Serial.println(F("🚀 LoRaWAN Interactive Message Sender (MCCI LMIC)"));
    Serial.println(F("================================================"));
    Serial.println(F("Type your message and press Enter to send it to TTN"));
    Serial.println(F("Maximum message length: 64 characters"));
    Serial.println(F("================================================"));

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Let LMIC compensate for +/- 20% clock error to better handle
    // downlink messages on imprecise oscillators (e.g. Arduino ceramic resonator)
    LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);

    // Set static session parameters for ABP.
    // Instead of dynamically establishing a session by joining the network,
    // precomputed session parameters are provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here; LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession(0x13, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession(0x13, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    // ── Band-specific channel setup ───────────────────────────────────────────
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set. SCG_init() MUST be called after
    // this block.
    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // The LMIC doesn't let you change the three basic settings, but we
    // show them here.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);  // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);  // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);  // g2-band
    // TTN defines an additional channel at 869.525 MHz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    #elif defined(CFG_us915) || defined(CFG_au915)
    // NA-US and AU channels 0-71 are configured automatically
    // but only one group of 8 (a subband) should be active.
    // TTN recommends the second sub band, 1 in a zero-based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);

    #elif defined(CFG_as923)
    // Set up the channels used in your country. Only two are defined by default,
    // and they cannot be changed. Use BAND_CENTI to indicate 1% duty cycle.
    // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    // ... extra definitions for channels 2..n here

    #elif defined(CFG_kr920)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // ... extra definitions for channels 3..n here.

    #elif defined(CFG_in866)
    // Set up the channels used in your country. Three are defined by default,
    // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
    // BAND_MILLI.
    // LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
    // ... extra definitions for channels 3..n here.

    #else
    # error Region not supported
    #endif

    // ── Single-channel enforcement ────────────────────────────────────────────
    // Disables channels 1-8, locks channelShuffleMap, disables ADR and link
    // check, sets DR/power/RX2. Must run after LMIC_setupChannel() calls above.
    // For ABP, this replaces the separate LMIC_setAdrMode(), LMIC_setDrTxpow(),
    // LMIC_setLinkCheckMode() calls. See single_channel_lmic.h for full details.
    SCG_init();

    // We don't send anything yet — waiting for user input
    transmissionComplete = true;
    Serial.println(F("✅ Ready for input. Type your message and press Enter."));
    Serial.print(F(">> "));
}

void loop() {
    // Check if there's data available in the Serial buffer
    while (Serial.available() > 0 && newMessage == false) {
        char inChar = (char)Serial.read();

        // Echo the character back (for better user experience)
        if (inChar >= 32 && inChar <= 126) {  // Printable characters
            Serial.print(inChar);
        }

        // Process the incoming character
        if (inChar == '\n' || inChar == '\r') {
            Serial.println();  // New line for clean output
            // End of message
            if (msgIndex > 0) {  // Only if we have actual content
                message[msgIndex] = '\0';  // Null-terminate the string
                newMessage = true;
                Serial.print(F("📝 Message received: \""));
                Serial.print(message);
                Serial.println(F("\""));
            } else {
                Serial.print(F(">> "));  // Prompt for new input if empty line
            }
        } else if (inChar == 8 || inChar == 127) {  // Backspace or DEL
            if (msgIndex > 0) {
                msgIndex--;
                message[msgIndex] = '\0';
                Serial.print(F("\b \b"));  // Erase character from terminal
            }
        } else if (msgIndex < MAX_MSG_LEN - 1 && inChar >= 32 && inChar <= 126) {
            // Add character to buffer if there's space and it's printable
            message[msgIndex++] = inChar;
        } else if (msgIndex >= MAX_MSG_LEN - 1) {
            Serial.println();
            Serial.println(F("⚠️  Message too long! Maximum 63 characters."));
            Serial.print(F(">> "));
            // Reset buffer
            memset(message, 0, MAX_MSG_LEN);
            msgIndex = 0;
        }
    }

    // If we have a new message and previous transmission is complete, send it
    if (newMessage && transmissionComplete) {
        Serial.println(F("📡 Preparing transmission..."));
        do_send(&sendjob);
    }

    // Heartbeat LED — blinks at ~1 Hz using millis() bit 9 (toggles every 512 ms)
    unsigned long now = millis();
    if ((now & 512) != 0) {
        digitalWrite(13, HIGH);
    } else {
        digitalWrite(13, LOW);
    }

    // Continuously enforce single-channel compliance before each LMIC tick.
    // Catches runtime drift from LinkAdrReq MAC commands, DR changes, or any
    // other internal library state change affecting txChnl or datarate.
    // See single_channel_lmic.h — SCG_enforce() for full details.
    SCG_enforce();

    // Run LMIC loop
    os_runloop_once();

    // Show prompt when ready for new input
    static bool promptShown = false;
    if (transmissionComplete && !newMessage && !promptShown) {
        Serial.print(F(">> "));
        promptShown = true;
    }
    if (!transmissionComplete || newMessage) {
        promptShown = false;
    }
}
