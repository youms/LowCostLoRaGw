#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// MQ2 Gas Sensor Setup
#define MQ2_PIN A0       // Arduino's pin connected to AO pin of the MQ2 sensor
#define WARMUP_DELAY 20000 // 20 seconds for MQ2 sensor to warm up

// LoRaWAN Network Session Key (NWKSKEY) - PROGMEM to save RAM
static const PROGMEM u1_t NWKSKEY[16] = { 0x55, 0xF7, 0x14, 0xA6, 0x58, 0x11, 0xDE, 0xAF, 0xF4, 0xAC, 0x6B, 0x65, 0x3F, 0xA5, 0xAB, 0xAD };

// LoRaWAN Application Session Key (APPSKEY) - PROGMEM to save RAM
static const u1_t PROGMEM APPSKEY[16] = { 0xBC, 0xA5, 0x0B, 0x27, 0x85, 0x31, 0xF0, 0x28, 0xBE, 0x75, 0x9A, 0x66, 0x84, 0x15, 0x76, 0xED };

// LoRaWAN End-Device Address (DevAddr)
// IMPORTANT: This should be unique for each device on the network in a real deployment
static const u4_t DEVADDR = 0x260BE0A2; 

// These callbacks are only used in over-the-air activation, so they are left empty here for ABP.
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// Payload buffer (2 bytes for MQ2 sensor data)
static uint8_t mydata[2];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 30;

// Pin mapping for LoRa module (confirm this matches your hardware setup)
// This is a common mapping for Heltec LoRa 32 or similar boards.
// Adjust if your board or shield has a different pinout.
const lmic_pinmap lmic_pins = {
    .nss = 10,            // Chip select for LoRa module
    .rxtx = LMIC_UNUSED_PIN, // For LoRa modules with single RX/TX pin, otherwise LMIC_UNUSED_PIN
    .rst = 9,             // Reset pin for LoRa module
    .dio = {2, 6, 7},     // DIO pins (DIO0, DIO1, DIO2) - check your board's schematic!
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE: // Transmission complete, including waiting for RX windows
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK) {
              Serial.println(F("Received ack"));
            }
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
         default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Read MQ2 sensor value
        int sensorValue = analogRead(MQ2_PIN);

        Serial.print(F("MQ2 Sensor Value: "));
        Serial.println(sensorValue);

        // Prepare upstream data transmission
        // sensorValue is an int (0-1023), fits in 2 bytes
        mydata[0] = highByte(sensorValue);
        mydata[1] = lowByte(sensorValue);
        
        //LMIC_setTxData2(port, data, data_len, confirmed_uplink_true_or_false)
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0); // port 1, 2 bytes, no confirmed uplink
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(38400); // Matched to ttn_temp_sensor.ino
    while (!Serial); // Wait for Serial to be ready (especially for boards like Leonardo/Micro)
    Serial.println(F("Starting TTN MQ2 Gas Sensor Sketch"));

    Serial.println(F("Warming up MQ2 sensor..."));
    pinMode(MQ2_PIN, INPUT);
    delay(WARMUP_DELAY); 
    Serial.println(F("MQ2 sensor warmup complete."));

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data are preserved.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, preconfigure the session parameters here.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into its own RAM-based struct.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running on AVR, these values are already in RAM.
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868) // EU868 region
    // Set up the channels used by TTN. Adopted from the official TTN examples.
    // TTN uses SF9 for its RX2 window.
    //LMIC_selectSubBand(1); // For EU868
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK), BAND_MILLI);       // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B devices.
    //LMIC_setupChannel(9, 869525000, DR_RANGE_MAP(DR_SF9, DR_SF9), BAND_CENTI);
    #elif defined(CFG_us915) // US915 region
    // NA-US channels 0-71 are configured automatically
    // Channels 0-7 + 64 for upstream, 8-15 + 65 for downstream
    //LMIC_selectSubBand(1); // Sub-band 2 for US915
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    //LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    // DR_SF12 for EU868 is a good default.
    LMIC_setDrTxpow(DR_SF12,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
